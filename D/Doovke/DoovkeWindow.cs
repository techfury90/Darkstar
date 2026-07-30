using System;
using System.Runtime.InteropServices;
using System.Threading;
using System.Windows.Forms;
using SDL2;

namespace D.Doovke
{
    /// <summary>
    /// Interactive front end for the Dove/Daybreak machine.
    ///
    /// Forked from Darkstar's DWindow: a WinForms host with a Panel whose handle SDL renders
    /// into (SDL_CreateWindowFrom), an ARGB8888 streaming texture, and an SDL message-loop
    /// thread.  What differs is the source of pixels (DoveDisplayController's 1-byte-per-pixel
    /// mono frame instead of the DLion display) and the keyboard encoding.
    /// </summary>
    public sealed class DoovkeWindow : Form
    {
        private DoovkeMachine _machine;
        private readonly object _machineLock = new object();

        // ---- display ----
        private readonly Panel _displayBox;
        private IntPtr _sdlWindow = IntPtr.Zero;
        private IntPtr _sdlRenderer = IntPtr.Zero;
        private IntPtr _displayTexture = IntPtr.Zero;
        private int[] _32bppDisplayBuffer;
        private int _displayWidth = 1152;
        private int _displayHeight = 861;
        private readonly ReaderWriterLockSlim _textureLock = new ReaderWriterLockSlim();
        private Thread _sdlThread;
        private volatile bool _sdlRunning;

        /// <summary>
        /// Display refresh interval.  The Dove display is INTERLACED, like the DLion's: the
        /// ~26.3 ms / ~38 Hz figure that falls out of DoveIOPIO's retrace period (210400 clocks
        /// at 8 MHz) is the FRAME rate, i.e. both fields.  A field therefore arrives every
        /// ~13.2 ms (~76 Hz), and that is the cadence the screen should be redrawn at.
        /// </summary>
        private const int RefreshIntervalMs = 13;

        // ---- machine ----
        private Thread _machineThread;
        private volatile bool _machineRunning;
        private volatile bool _paused;
        private const int BatchSteps = 50000;   // steps per lock acquisition

        private readonly System.Windows.Forms.Timer _refreshTimer;
        private readonly StatusStrip _statusStrip;
        private readonly ToolStripStatusLabel _statusLabel;
        private string _floppyPath;

        public DoovkeWindow(DoovkeMachine machine, string initialFloppy)
        {
            _machine = machine;
            _floppyPath = initialFloppy;

            Text = "Doovke - Xerox 6085 (Dove/Daybreak)";
            StartPosition = FormStartPosition.CenterScreen;
            KeyPreview = true;

            _displayBox = new Panel
            {
                Dock = DockStyle.Fill,
                BackColor = System.Drawing.Color.Black,
                TabStop = false,
            };
            Controls.Add(_displayBox);

            _statusLabel = new ToolStripStatusLabel("starting...");
            _statusStrip = new StatusStrip();
            _statusStrip.Items.Add(_statusLabel);
            Controls.Add(_statusStrip);

            Controls.Add(BuildMenu());

            _32bppDisplayBuffer = new int[_displayWidth * _displayHeight];

            _refreshTimer = new System.Windows.Forms.Timer { Interval = RefreshIntervalMs };
            _refreshTimer.Tick += (s, e) => { PollMouse(); UpdateAndRender(); AutoSaveRigidDisk(); };

            Load += OnWindowLoad;
            if (!string.IsNullOrEmpty(Environment.GetEnvironmentVariable("DOVE_CURSOR_DUMP")))
                _machine.Display.BorderLog = new System.Collections.Generic.List<string>();

            if (!string.IsNullOrEmpty(Environment.GetEnvironmentVariable("DOVE_CP_STATE")))
            {
                _machine.Io.GateLog = new System.Collections.Generic.List<string>();
                _machine.Io.ConfigEeprom.ReadLog = new System.Collections.Generic.List<int>();
                _machine.Io.ConfigEeprom.WriteLog = new System.Collections.Generic.List<int>();
                // These two exist in I8272 for exactly this shape of stall -- CommandCount climbing
                // into the thousands while ReadCount stays pinned -- but were never allocated, so
                // the ring stayed empty.  ReadTrace also records the missing-track case
                // (ST0=0x40, ST1=0x01), which is what a geometry gap in the IMD conversion looks
                // like from the controller's side.
                if (_machine.Io.Fdc != null)
                {
                    _machine.Io.Fdc.RecentLog = new string[256];
                    _machine.Io.Fdc.ReadTrace = new System.Collections.Generic.List<string>();
                    _machine.Io.Fdc.SectorHeads = new System.Collections.Generic.List<string>();
                }
                // BOOTSTRAPIOR (0x03E00) write history -- the floppy boot handshake lives here.
                _machine.Memory.BootIorSeq =
                    new System.Collections.Generic.Dictionary<int, System.Collections.Generic.List<byte>>();
                _machine.Memory.BootIorLog = new System.Collections.Generic.List<string>();
                _machine.Memory.IocbWatch = new System.Collections.Generic.Dictionary<int, long[]>();
                // CPU faults, and who polls the umbilical i8255.
                _machine.Iop.FaultLog = new System.Collections.Generic.List<string>();
                _machine.Io.PollSites = new System.Collections.Generic.Dictionary<int, long[]>();
                _machine.Io.DmaLog = new System.Collections.Generic.List<string>();
                _machine.Io.FdcDestLog = new System.Collections.Generic.List<string>();
                _machine.Memory.BufReadHist = new System.Collections.Generic.Dictionary<int, long[]>();
            }

            if (!string.IsNullOrEmpty(Environment.GetEnvironmentVariable("DOVE_MP_LOG")))
                _machine.Cp.WrmpLog = new System.Collections.Generic.List<string>();

            // DOVE_MP_TRACE=<path>: the panel as DISPLAYED, sampled from the cursor sprite buffer.
            //
            // This exists because the @WRMP opcode hook (DOVE_MP_LOG) is not trustworthy: it tests
            // _ib[_ibPtr & 1] for the alpha byte, which picked up 259 false posts from a stale IB
            // byte and MISSED the one code that was actually on screen.  The sprite is the panel --
            // the ROM draws the digits into ED00-ED1F -- so sampling it needs no opcode decoding and
            // no assumption about which microcode path posted.  Raw bytes are logged and decoded
            // offline; rendering glyphs to digits in here would just be a second thing to get wrong.
            if (!string.IsNullOrEmpty(Environment.GetEnvironmentVariable("DOVE_MP_TRACE")))
            {
                _mpTrace = new System.Collections.Generic.List<string>();
                _mpTracePath = Environment.GetEnvironmentVariable("DOVE_MP_TRACE");
                _machine.Display.OnCursorWrite += (port, value) =>
                {
                    if (_mpTrace.Count >= 20000) return;
                    byte[] b = _machine.Display.GetCursorBuffer();
                    string hex = BitConverter.ToString(b);
                    if (hex == _lastSprite) return;          // only log real changes
                    _lastSprite = hex;
                    string line = "CPi=" + _machine.Cp.InstructionCount + " " + hex;
                    _mpTrace.Add(line);
                    // Append as we go, not only on close.  Waiting for shutdown meant the only way
                    // to learn whether the machine was alive was to kill it -- and a 7x00 phase can
                    // run 642 MILLION instructions with no disk I/O and a frozen cursor, which is
                    // indistinguishable from a wedge from outside.  Several healthy runs were closed
                    // as "hangs" on exactly that guess.  Flushing live means the panel can be read
                    // from the file while the machine keeps running.
                    try { System.IO.File.AppendAllText(_mpTracePath, line + System.Environment.NewLine); }
                    catch { }
                };
            }

            // DOVE_DRAM_DUMP=<path> dumps shared DRAM on the way out, so a state reached
            // interactively (a wedge, an MP code) can be analysed without having to hit the menu
            // before closing -- closing is the thing you were going to do anyway.
            FormClosing += (s, e) =>
            {
                // Never let the diagnostics block the shutdown that saves the pack.
                try { DumpSharedDramTo(Environment.GetEnvironmentVariable("DOVE_DRAM_DUMP")); }
                catch { }
                try { FlushMapWatch(Environment.GetEnvironmentVariable("DOVE_MAP_WATCH_LOG")); }
                catch { }
                try { FlushMpLog(Environment.GetEnvironmentVariable("DOVE_MP_LOG")); }
                catch { }
                try { DumpCursorSprite(Environment.GetEnvironmentVariable("DOVE_CURSOR_DUMP")); }
                catch { }
                try
                {
                    string cs = Environment.GetEnvironmentVariable("DOVE_CURSOR_DUMP");
                    if (!string.IsNullOrEmpty(cs))
                        System.IO.File.AppendAllText(cs,
                            System.Environment.NewLine + _machine.Display.DescribeCursorState()
                            + System.Environment.NewLine + _machine.Display.DescribeVerticalCs()
                            + System.Environment.NewLine + "TopBorderLines=" + _machine.Display.TopBorderLines
                            + System.Environment.NewLine
                            + (_machine.Display.BorderLog == null ? "" :
                               System.Environment.NewLine + "border/control write history:"
                               + System.Environment.NewLine + "  "
                               + string.Join(System.Environment.NewLine + "  ",
                                             _machine.Display.BorderLog.ToArray()))
                            + System.Environment.NewLine);
                }
                catch { }
                try
                {
                    string cp = Environment.GetEnvironmentVariable("DOVE_CP_STATE");
                    if (!string.IsNullOrEmpty(cp))
                    {
                        string cpState;
                        lock (_machineLock) cpState = _machine.Cp.DescribeState();
                        // Which port is actually being hammered?  A spin shows up here as one port
                        // with a count orders of magnitude above the rest.  Reading a single
                        // suspected port (as the input-port gate log does) cannot distinguish "this
                        // is the spin" from "this is incidental polling while the spin is elsewhere".
                        var pc = _machine.Io.PortCounts;
                        if (pc != null && pc.Count > 0)
                        {
                            var items = new System.Collections.Generic.List<string>();
                            var keys = new System.Collections.Generic.List<ushort>(pc.Keys);
                            keys.Sort((x, y) => pc[y].CompareTo(pc[x]));
                            for (int i = 0; i < keys.Count && i < 14; i++)
                                items.Add(keys[i].ToString("X4") + "=" + pc[keys[i]]);
                            cpState += System.Environment.NewLine
                                + "hottest I/O ports (count desc):" + System.Environment.NewLine
                                + "  " + string.Join("  ", items.ToArray())
                                + System.Environment.NewLine;
                        }

                        // What did the 93C46 actually return, and did bit 11 ever rise?
                        //
                        // The firmware's %ReadEEProm sets carry on failure, and InitCPSpecific then
                        // defaults csBankConfiguration to fourKEEPromFormat -- ONE bank -- after
                        // which CheckBlock discards every CP block destined for a bank it believes
                        // absent.  So a failed EEPROM READ is enough to leave the WCS empty, which is
                        // why overriding the EEPROM's own bank nibble changed nothing.
                        //
                        // Bit 11 carries read DATA during a read and READY/BUSY otherwise (same 0x0800
                        // mask), so a histogram of it across ALL reads is the thing to look at.  The
                        // earlier gate log printed only the first and last eight of 300 samples, which
                        // could not show whether it varied in between.
                        cpState += System.Environment.NewLine
                            + "WCS writes: byteOUTs=" + _machine.Io.WcsByteWrites
                            + " wordOUTs=" + _machine.Io.WcsWordWrites
                            + "  ports " + (_machine.Io.WcsFirstPort < 0 ? "none"
                                : _machine.Io.WcsFirstPort.ToString("X4") + ".." + _machine.Io.WcsLastPort.ToString("X4"))
                            + "   (a full CP load = 23,574 byte OUTs)"
                            + System.Environment.NewLine
                            + "  bank register 0xE000 OUTs=" + _machine.Io.BankRegWrites
                            + " lastValue=" + (_machine.Io.BankRegLastValue < 0 ? "(never)"
                                : "0x" + _machine.Io.BankRegLastValue.ToString("X2"))
                            + "   (WriteCntlStore writes this ONCE, AL=0, before its byte OUTs)"
                            + System.Environment.NewLine;

                        // ---- CPU faults ----
                        // A flood of reads on port 0x74 is the IOP sitting in the Burdock/Bindweed
                        // remote-debugger interface, polling the umbilical i8255 for a host debugger
                        // that will never attach -- i.e. it took an unhandled x86 exception.  Vector 6
                        // is the prime suspect because Dispatch raises it for UNIMPLEMENTED opcodes as
                        // well as invalid ones, so a gap in this core presents to the guest as a crash.
                        var flog = _machine.Iop.FaultLog;
                        if (flog != null)
                        {
                            cpState += System.Environment.NewLine + "CPU faults (" + flog.Count + "):"
                                     + System.Environment.NewLine;
                            if (flog.Count == 0)
                                cpState += "  (none -- no divide error, BOUND or invalid opcode was taken)"
                                         + System.Environment.NewLine;
                            foreach (string fl in flog) cpState += "  " + fl + System.Environment.NewLine;
                        }
                        // ---- does the .db walk survive the CP Start block? ----
                        var brh = _machine.Memory.BufReadHist;
                        if (brh != null)
                        {
                            cpState += System.Environment.NewLine
                                + "BOOT-FILE CURSOR: buffer reads=" + _machine.Memory.BufReadTotal
                                + "  highest file offset read=" + _machine.Memory.MaxFileOffsetRead
                                + System.Environment.NewLine
                                + "  file map: 0xB800 CP Start at 5458, 0xD000 CP WriteData at 6040."
                                + "  A 2-byte over-consume in CallDumpCSAddrBlock (CX=5, the Daisy"
                                + " two-address skip, vs 3 for Daybreak's one) puts the next type read at 6042."
                                + System.Environment.NewLine;
                            var bk = new System.Collections.Generic.List<int>(brh.Keys);
                            bk.Sort();
                            cpState += "  reads by file offset (offset=value xN):" + System.Environment.NewLine + "   ";
                            int col = 0;
                            foreach (int fo in bk)
                            {
                                long[] r = brh[fo];
                                string tag = fo == 8600 ? "<TYPE-hi>" : fo == 8601 ? "<TYPE-lo>"
                                           : fo == 8602 ? "<BODY>" : "";
                                cpState += " " + fo + "=" + r[1].ToString("X2") + "x" + r[0] + tag;
                                if (++col % 8 == 0) cpState += System.Environment.NewLine + "   ";
                            }
                            if (bk.Count == 0) cpState += " (no reads in 5400..6119)";
                            cpState += System.Environment.NewLine;
                        }
                        var fdl = _machine.Io.FdcDestLog;
                        if (fdl != null)
                        {
                            cpState += "floppy DMA dests / file offsets (" + fdl.Count + "):"
                                     + System.Environment.NewLine;
                            foreach (string s2 in fdl) cpState += "  " + s2 + System.Environment.NewLine;
                        }

                        var dmal = _machine.Io.DmaLog;
                        if (dmal != null)
                        {
                            cpState += "floppy DMA transfers (" + dmal.Count + "):" + System.Environment.NewLine;
                            foreach (string dl in dmal) cpState += "  " + dl + System.Environment.NewLine;
                            if (dmal.Count == 0) cpState += "  (none)" + System.Environment.NewLine;
                        }
                        // Interrupt vector table (linear 0x000-0x1FF = vectors 00-7F).  Opie installs its
                        // service calls here; an entry left at 0000:xxxx sends the INT that uses it into
                        // low memory, where execution walks the table itself until it meets a 0xCC byte
                        // (the low half of an offset like 0x00CC) and takes an INT 3.
                        byte[] ivt = _machine.Memory.SramRaw;
                        if (ivt != null && ivt.Length >= 0x200)
                        {
                            cpState += "IVT vectors 00-7F (seg:off, '-' = 0000:0000):" + System.Environment.NewLine;
                            for (int v = 0; v < 0x80; v += 8)
                            {
                                string ln = "  " + v.ToString("X2") + ":";
                                for (int i = 0; i < 8; i++)
                                {
                                    int a = (v + i) * 4;
                                    int off = ivt[a] | (ivt[a + 1] << 8);
                                    int seg = ivt[a + 2] | (ivt[a + 3] << 8);
                                    ln += (off == 0 && seg == 0) ? "  ----:----"
                                        : "  " + seg.ToString("X4") + ":" + off.ToString("X4");
                                }
                                cpState += ln + System.Environment.NewLine;
                            }
                        }
                        long[] vc = _machine.Iop.VectorCounts;
                        if (vc != null)
                        {
                            cpState += "vectors taken (nonzero):";
                            for (int v = 0; v < vc.Length; v++)
                                if (vc[v] != 0) cpState += " " + v.ToString("X2") + "=" + vc[v];
                            cpState += System.Environment.NewLine;
                        }
                        var psites = _machine.Io.PollSites;
                        if (psites != null && psites.Count > 0)
                        {
                            cpState += "umbilical i8255 poll sites (port/PC -> count, first..last IOP clock):"
                                     + System.Environment.NewLine;
                            var pk = new System.Collections.Generic.List<int>(psites.Keys);
                            pk.Sort(delegate (int a, int b) { return psites[b][0].CompareTo(psites[a][0]); });
                            for (int i = 0; i < pk.Count && i < 12; i++)
                            {
                                long[] r = psites[pk[i]];
                                cpState += "  port 0x" + ((pk[i] >> 20) & 0xFFF).ToString("X2")
                                         + " PC=" + (pk[i] & 0xFFFFF).ToString("X5")
                                         + " -> " + r[0] + "  (" + r[1] + ".." + r[2] + ")"
                                         + System.Environment.NewLine;
                            }
                            cpState += "  distinct sites=" + psites.Count + System.Environment.NewLine;
                        }
                        // Whatever the ROM's error reporter / debugger managed to say on the umbilical
                        // serial console.  This is the payload of the whole 0x74 investigation.
                        string uartTx = _machine.Io.DiagUartTx.ToString();
                        cpState += "umbilical console TX (" + _machine.Io.DiagUartTxRaw.Count + " bytes):"
                                 + System.Environment.NewLine;
                        cpState += uartTx.Length == 0 ? "  (nothing transmitted)" + System.Environment.NewLine
                                                     : uartTx + System.Environment.NewLine;
                        // Raw hex as well: this channel feeds the external MP-code/status readout box,
                        // so the payload is a status protocol rather than ASCII and the printable
                        // rendering above loses it.
                        if (_machine.Io.DiagUartTxRaw.Count > 0)
                        {
                            cpState += "  raw:";
                            var rawTx = _machine.Io.DiagUartTxRaw;
                            for (int i = 0; i < rawTx.Count && i < 64; i++)
                                cpState += " " + rawTx[i].ToString("X2");
                            cpState += System.Environment.NewLine;
                        }

                        var fdc = _machine.Io.Fdc;
                        if (fdc != null)
                        {
                            cpState += System.Environment.NewLine
                                + "FDC: commands=" + fdc.CommandCount
                                + " reads=" + fdc.ReadCount
                                + " intRaises=" + fdc.IntRaises
                                + System.Environment.NewLine
                                + "  gathered=" + fdc.TotalGathered + "B  dmaSent=" + fdc.TotalDmaSent + "B"
                                + "  overGathered=" + (fdc.TotalGathered - fdc.TotalDmaSent) + "B"
                                // gathered > sent is EXPECTED and not loss: we pull every sector from
                                // R to EOT while the guest's DMA takes only what it programmed,
                                // terminated by TC.  What matters is whether dmaSent equals what the
                                // guest asked for -- check it against sectors x 512, not against
                                // gathered.  (I first labelled this difference "lost", which is wrong.)
                                + "  dmaSent/512=" + (fdc.TotalDmaSent / 512.0).ToString("F1") + " sectors";
                            if (fdc.CommandCount > 0)
                                cpState += " (" + (fdc.IntRaises / (double)fdc.CommandCount).ToString("F2")
                                         + " per command)";
                            cpState += System.Environment.NewLine + "  by site:";
                            for (int i = 0; i < fdc.IntRaisesBySite.Length; i++)
                                if (fdc.IntRaisesBySite[i] != 0)
                                    cpState += " s" + i + "=" + fdc.IntRaisesBySite[i];
                            cpState += System.Environment.NewLine;
                        }

                        if (fdc != null && fdc.RecentLog != null)
                        {
                            cpState += System.Environment.NewLine + "FDC last commands (ring):"
                                     + System.Environment.NewLine;
                            int n = fdc.RecentLog.Length;
                            long from = fdc.RecentPos > 24 ? fdc.RecentPos - 24 : 0;
                            for (long i = from; i < fdc.RecentPos; i++)
                            {
                                string entry = fdc.RecentLog[(int)(i % n)];
                                if (!string.IsNullOrEmpty(entry)) cpState += "  " + entry + System.Environment.NewLine;
                            }
                        }
                        if (fdc != null && fdc.SectorHeads != null)
                        {
                            cpState += System.Environment.NewLine + "delivered payload heads ("
                                     + fdc.SectorHeads.Count + "):" + System.Environment.NewLine;
                            foreach (string h in fdc.SectorHeads)
                                cpState += "  " + h + System.Environment.NewLine;
                        }
                        if (fdc != null && fdc.ReadTrace != null)
                        {
                            cpState += System.Environment.NewLine + "FDC read trace ("
                                     + fdc.ReadTrace.Count + " reads), all:"
                                     + System.Environment.NewLine;
                            int f = 0;   // ALL reads: the last 12 hid where the sequence STARTED
                            for (int i = f; i < fdc.ReadTrace.Count; i++)
                                cpState += "  " + fdc.ReadTrace[i] + System.Environment.NewLine;
                        }

                        // ---- BOOTSTRAPIOR (0x03E00): the floppy boot handshake ----
                        // RamBoot's bootTask emits MP 0199 two instructions after it releases the
                        // producer, then parks at %WaitForCondition(bootBufferFull, noTimeout).  So 0199
                        // is a normal progress marker, not a stuck code, and the task IS running.  The
                        // one value that decides whether it ever moves on is
                        // flppyIOCB.OperationState (flppyIOCB+1), which FloppyRead compares against
                        // OperationCompleted = 6.  Every requested byte can be delivered -- and we have
                        // proven they are, header and all -- without that byte ever being stored.
                        // ---- 80186 integrated interrupt controller ----
                        // Vector 0x60 fired 83,631,176 times in one 130-second run, straight into
                        // IVT[0x60] = FC00:035D, which the IVT dump shows is the ROM's DEFAULT handler
                        // (the same entry installed for 0x24-0x29, 0x2F, 0x36-0x39, 0x3B, 0x3C, 0x3E,
                        // 0x3F).  Nothing in DoveIOPIO raises master IR0, so 0x60 is not an external
                        // 8259 line at all: master IR6 cascades the 80186's OWN controller, and
                        // AcknowledgeInternal returns (IntVector & 0xF8) | requestBit -- so this is
                        // internal request bit 0 with the vector base at 0x60.  InternalAckByBit says
                        // which source, and it was already being counted and never printed.
                        if (_machine.Io.Pcb != null)
                        {
                            var pcb = _machine.Io.Pcb;
                            cpState += System.Environment.NewLine
                                + "80186 internal PIC: acks=" + pcb.AckCount + " eois=" + pcb.EoiCount
                                + " autoEOI=" + pcb.AutoEoi
                                + "  vecBase=0x" + (pcb.GetRegisterWord(0x20) & 0xF8).ToString("X2")
                                + "  request=0x" + pcb.InternalRequest.ToString("X4")
                                + " mask=0x" + pcb.GetRegisterWord(0x28).ToString("X4")
                                + " inService=0x" + pcb.InternalInService.ToString("X4")
                                + System.Environment.NewLine + "  acks by request bit:";
                            for (int i = 0; i < pcb.InternalAckByBit.Length; i++)
                                if (pcb.InternalAckByBit[i] != 0)
                                    cpState += " b" + i + "=" + pcb.InternalAckByBit[i];
                            cpState += System.Environment.NewLine + "  timer maxcount hits by request bit:";
                            for (int i = 0; i < pcb.TimerFireByReq.Length; i++)
                                if (pcb.TimerFireByReq[i] != 0)
                                    cpState += " b" + i + "=" + pcb.TimerFireByReq[i];
                            cpState += System.Environment.NewLine
                                + "  T0 cnt=" + pcb.GetRegisterWord(0x50).ToString("X4")
                                + " maxA=" + pcb.GetRegisterWord(0x52).ToString("X4")
                                + " maxB=" + pcb.GetRegisterWord(0x54).ToString("X4")
                                + " ctl=" + pcb.GetRegisterWord(0x56).ToString("X4")
                                + " | T1 cnt=" + pcb.GetRegisterWord(0x58).ToString("X4")
                                + " maxA=" + pcb.GetRegisterWord(0x5A).ToString("X4")
                                + " ctl=" + pcb.GetRegisterWord(0x5E).ToString("X4")
                                + " | T2 cnt=" + pcb.GetRegisterWord(0x60).ToString("X4")
                                + " maxA=" + pcb.GetRegisterWord(0x62).ToString("X4")
                                + " ctl=" + pcb.GetRegisterWord(0x66).ToString("X4")
                                + System.Environment.NewLine;
                        }

                        // ---- floppy IOCB: which OperationState byte, and does it ever reach 6? ----
                        var iw = _machine.Memory.IocbWatch;
                        if (iw != null)
                        {
                            byte[] sr = _machine.Memory.SramRaw;
                            cpState += System.Environment.NewLine
                                + "FLOPPY IOCB WATCH  (downloaded BOOTSTRAPIOR 0x00580-0x006AD, ROM's 0x03E00;"
                                + " OperationState = floppyIOCB+1, EVEN base so ODD address)"
                                + System.Environment.NewLine;

                            // Candidates by the static image's own signature: even address holding
                            // MesaTRUE then OperationWaiting(3), per RAMFlpBt's initialRec.
                            if (sr != null)
                            {
                                cpState += "  even-aligned (MesaTRUE,03) signature scan:";
                                int hits = 0;
                                for (int a = 0x0500; a < 0x3E80; a += 2)
                                {
                                    if (!(a < 0x0900 || a >= 0x3D00)) continue;
                                    if (a + 1 >= sr.Length) break;
                                    if ((sr[a] == 0x01 || sr[a] == 0xFF) && sr[a + 1] == 0x03)
                                    { cpState += " 0x" + a.ToString("X4"); hits++; }
                                }
                                cpState += hits == 0 ? " (none -- no IOCB left in Waiting)" : "";
                                cpState += System.Environment.NewLine;
                            }

                            // Busiest addresses in the windows.  A retry loop's CMP dominates reads.
                            var ik = new System.Collections.Generic.List<int>(iw.Keys);
                            ik.Sort(delegate (int a, int b)
                            { return (iw[b][0] + iw[b][1]).CompareTo(iw[a][0] + iw[a][1]); });
                            cpState += "  busiest bytes (reads/writes, w3=stores of Waiting, w6=stores of Completed):"
                                     + System.Environment.NewLine;
                            for (int i = 0; i < ik.Count && i < 14; i++)
                            {
                                long[] r = iw[ik[i]];
                                cpState += "    0x" + ik[i].ToString("X4")
                                    + (((ik[i]) & 1) != 0 ? " [odd]" : "      ")
                                    + " reads=" + r[0] + " writes=" + r[1]
                                    + " w3=" + r[2] + " w6=" + r[3]
                                    + " last=0x" + r[4].ToString("X2")
                                    + " lastPC=" + r[5].ToString("X5");
                                if (r[2] > 0) cpState += " pcW3=" + r[8].ToString("X5");
                                if (r[3] > 0) cpState += " pcW6=" + r[9].ToString("X5");
                                cpState += "  IOP " + r[6] + ".." + r[7] + System.Environment.NewLine;
                            }
                            // The headline: anything re-armed to Waiting but never Completed.
                            cpState += "  re-armed to Waiting(3) but NEVER Completed(6):";
                            int stuck = 0;
                            foreach (System.Collections.Generic.KeyValuePair<int, long[]> kv in iw)
                                if (kv.Value[2] > 0 && kv.Value[3] == 0)
                                { cpState += " 0x" + kv.Key.ToString("X4") + "(x" + kv.Value[2] + ")"; stuck++; }
                            cpState += stuck == 0 ? " (none)" : "";
                            cpState += System.Environment.NewLine;
                        }

                        var bseq = _machine.Memory.BootIorSeq;
                        if (bseq != null)
                        {
                            byte[] sramNow = _machine.Memory.SramRaw;
                            cpState += System.Environment.NewLine
                                + "BOOTSTRAPIOR window 0x3C00-0x3FFF: " + bseq.Count + " addresses written"
                                + System.Environment.NewLine;

                            // The headline question, answered by one scan.
                            var completedAt = new System.Collections.Generic.List<int>();
                            foreach (System.Collections.Generic.KeyValuePair<int,
                                     System.Collections.Generic.List<byte>> kv in bseq)
                                if (kv.Value.Contains((byte)6)) completedAt.Add(kv.Key);
                            completedAt.Sort();
                            cpState += "  OperationCompleted(6) ever stored: "
                                + (completedAt.Count == 0 ? "NO -- at no address in the window"
                                   : "yes, at " + completedAt.Count + " address(es)");
                            for (int i = 0; i < completedAt.Count && i < 12; i++)
                                cpState += " 0x" + completedAt[i].ToString("X4");
                            cpState += System.Environment.NewLine;

                            // Per-address value sequences, but only where a state enum or a condition
                            // encoding appears -- printing all 1024 addresses would bury the answer.
                            var keys = new System.Collections.Generic.List<int>(bseq.Keys);
                            keys.Sort();
                            cpState += "  value sequences (addresses carrying 3..7, or a 0x80/0x01 condition byte):"
                                + System.Environment.NewLine;
                            int shown = 0;
                            foreach (int a in keys)
                            {
                                System.Collections.Generic.List<byte> sq = bseq[a];
                                bool interesting = false;
                                foreach (byte v in sq)
                                    if ((v >= 3 && v <= 7) || v == 0x80 || v == 0x01) { interesting = true; break; }
                                if (!interesting || shown >= 48) continue;
                                shown++;
                                string line = "    0x" + a.ToString("X4") + " (IOR"
                                    + (a >= 0x3E00 ? "+0x" + (a - 0x3E00).ToString("X3")
                                                   : "-0x" + (0x3E00 - a).ToString("X3")) + ") :";
                                foreach (byte v in sq) line += " " + v.ToString("X2");
                                if (sq.Count >= 24) line += " ...";
                                int idx = a - 0; // SramBase is 0
                                if (sramNow != null && idx >= 0 && idx < sramNow.Length)
                                    line += "   [now " + sramNow[idx].ToString("X2") + "]";
                                cpState += line + System.Environment.NewLine;
                            }
                            if (shown == 0) cpState += "    (none)" + System.Environment.NewLine;

                            var blog = _machine.Memory.BootIorLog;
                            if (blog != null)
                            {
                                cpState += "  ordered state-valued writes (" + blog.Count + "):"
                                    + System.Environment.NewLine;
                                int from = blog.Count > 40 ? blog.Count - 40 : 0;
                                for (int i = from; i < blog.Count; i++)
                                    cpState += "    " + blog[i] + System.Environment.NewLine;
                                if (blog.Count == 0) cpState += "    (none)" + System.Environment.NewLine;
                            }

                            // Final image of the two structures named in the source: the conditions at
                            // the base and FloppyBootArea at +0x30 (floppyIOCBDone is its offset 0).
                            if (sramNow != null && sramNow.Length >= 0x3E80)
                            {
                                cpState += "  final bytes 0x3E00-0x3E7F:" + System.Environment.NewLine;
                                for (int row = 0x3E00; row < 0x3E80; row += 16)
                                {
                                    string h = "    " + row.ToString("X4") + " (+0x"
                                             + (row - 0x3E00).ToString("X2") + "): ";
                                    for (int i = 0; i < 16; i++) h += sramNow[row + i].ToString("X2") + " ";
                                    cpState += h + System.Environment.NewLine;
                                }
                            }
                        }

                        var rl = _machine.Io.ConfigEeprom.ReadLog;
                        cpState += System.Environment.NewLine + "93C46 completed READs: "
                                 + (rl == null ? "(not logging)" : rl.Count.ToString())
                                 + System.Environment.NewLine;
                        if (rl != null)
                            for (int i = 0; i < rl.Count && i < 16; i++)
                                cpState += "  addr " + ((rl[i] >> 16) & 0x3F).ToString("X2")
                                         + " -> " + (rl[i] & 0xFFFF).ToString("X4")
                                         + System.Environment.NewLine;

                        var g = _machine.Io.GateLog;
                        if (g != null && g.Count > 0)
                        {
                            cpState += System.Environment.NewLine
                                + "input-port (0x80) reads -- the microcode-load gates, first 8 and last 8:"
                                + System.Environment.NewLine + "  ";
                            var pick = new System.Collections.Generic.List<string>();
                            for (int i = 0; i < g.Count && i < 8; i++) pick.Add(g[i]);
                            for (int i = System.Math.Max(8, g.Count - 8); i < g.Count; i++) pick.Add(g[i]);
                            cpState += string.Join(System.Environment.NewLine + "  ", pick.ToArray())
                                + System.Environment.NewLine
                                + "  total input-port reads = " + _machine.Io.InputPortReads
                                + ", of which bit 11 high = " + _machine.Io.InputPortB11High
                                + System.Environment.NewLine;
                        }
                        System.IO.File.WriteAllText(cp, cpState);
                    }
                }
                catch { }
                try
                {
                    // already appended live; nothing to do on close.
                }
                catch { }
                SaveRigidDisk();
                Shutdown();
            };
        }

        /// <summary>
        /// Size the window so the display panel is exactly the framebuffer's size, i.e. 1:1 with
        /// no scaling.  SDL stretches the texture to the panel, so the panel has to be the exact
        /// pixel size or the image is resampled.  The menu and status bars sit outside it.
        /// </summary>
        private void SizeToDisplay()
        {
            int chrome = 0;
            if (MainMenuStrip != null) chrome += MainMenuStrip.Height;
            if (_statusStrip != null) chrome += _statusStrip.Height;

            ClientSize = new System.Drawing.Size(_displayWidth, _displayHeight + chrome);

            // If that would not fit on the screen, fall back to the largest size that does --
            // still 1:1 for as much of the display as is visible, and the window is resizable.
            var wa = Screen.FromControl(this).WorkingArea;
            if (Width > wa.Width || Height > wa.Height)
            {
                Width = Math.Min(Width, wa.Width);
                Height = Math.Min(Height, wa.Height);
            }
        }

        /// <summary>
        /// Flush the pack if it has been written since the last save and enough time has
        /// passed.  A format or an install represents a great deal of wall-clock time and
        /// lives only in memory until written out; losing it to a crash or a mis-click is
        /// far more expensive than a periodic 60 MB write.
        /// </summary>
        private void AutoSaveRigidDisk()
        {
            if (string.IsNullOrEmpty(_machine.RigidDiskPath)) return;
            long ops = _machine.Io.Rdc.DobOps;
            if (ops == _lastSavedRdcOps) return;                       // nothing written since
            if (_sinceSave.ElapsedMilliseconds < AutoSaveIntervalMs) return;
            _lastSavedRdcOps = ops;
            _sinceSave.Reset(); _sinceSave.Start();
            SaveRigidDisk();
        }

        private System.Collections.Generic.List<string> _mpTrace;
        private string _lastSprite;
        private string _mpTracePath;

        private const int AutoSaveIntervalMs = 120000;   // 2 minutes
        private long _lastSavedRdcOps = -1;
        private readonly System.Diagnostics.Stopwatch _sinceSave = System.Diagnostics.Stopwatch.StartNew();

        private void SaveRigidDisk()
        {
            if (string.IsNullOrEmpty(_machine.RigidDiskPath)) return;
            try { lock (_machineLock) _machine.SaveRigidDisk(); }
            catch (Exception ex)
            {
                MessageBox.Show(this, "Could not save the rigid disk: " + ex.Message,
                                "Rigid disk", MessageBoxButtons.OK, MessageBoxIcon.Warning);
            }
        }

        /// <summary>
        /// Write the whole 4 MB shared DRAM array to a file, for offline analysis.
        ///
        /// This exists because the in-emulator scanners that preceded it were not trustworthy and
        /// were believed anyway.  A "locate the CP map by its vacant stamps" probe found zero
        /// stamps, reported base 0xFFFFFFFF, and then happily printed map entries read from that
        /// base -- all zeros, which read as "the page map does not resolve".  A companion scan
        /// concluded the boot file's last pages were "absent from the whole array" while a
        /// write-point trace proved them present, correct, and readback-verified.  Both were
        /// searching from a bad base; neither said so.
        ///
        /// A dump has no such failure mode: the bytes are the bytes, the analysis happens outside,
        /// and a wrong hypothesis about where a structure lives cannot disguise itself as evidence
        /// about what the structure contains.  Prefer this over adding another in-process scanner.
        ///
        /// The CP and IOP share this one array (there is no bridge).  CP word W is byte 2W, big
        /// endian; the VM map lives at CP word 0x40000 = byte 0x80000, entry for virtual page V at
        /// byte 0x80000 + 2V, decoding as realPage = ((w &amp; 0x1F) &lt;&lt; 8) | (w &gt;&gt; 8) with flags
        /// referenced 0x0080, dirty 0x0040, writeProtect 0x0020 and a vacant stamp of 0x0060.
        /// The 16 KB IOP-local SRAM at 0x00000-0x03FFF is a SEPARATE array and is NOT in this dump.
        /// </summary>
        private void DumpSharedDram()
        {
            using (var dlg = new SaveFileDialog())
            {
                dlg.Title = "Dump shared DRAM";
                dlg.Filter = "Raw memory image (*.bin)|*.bin|All files (*.*)|*.*";
                dlg.FileName = "dove_dram.bin";
                if (dlg.ShowDialog(this) != DialogResult.OK) return;
                try
                {
                    long n = DumpSharedDramTo(dlg.FileName);
                    MessageBox.Show(this, "Wrote " + n.ToString("N0") + " bytes to " + dlg.FileName,
                                    "Dump shared DRAM", MessageBoxButtons.OK, MessageBoxIcon.Information);
                }
                catch (Exception ex)
                {
                    MessageBox.Show(this, "Could not write the dump: " + ex.Message,
                                    "Dump shared DRAM", MessageBoxButtons.OK, MessageBoxIcon.Warning);
                }
            }
        }

        /// <summary>
        /// Write the CP's map-write watch log (DOVE_MAP_WATCH) out on the way down.  Pairs with the
        /// DRAM dump: the dump gives the map's final state, this gives every change to the watched
        /// entries along the way, which is what a post-mortem snapshot cannot show once demand
        /// paging is remapping pages underneath the guest.
        /// </summary>
        private void FlushMapWatch(string path)
        {
            if (string.IsNullOrEmpty(path)) return;
            System.Collections.Generic.List<string> log;
            lock (_machineLock) log = _machine.Cp.MapWatchLog;
            if (log == null) return;
            System.IO.File.WriteAllLines(path, log.ToArray());
        }

        /// <summary>
        /// Write out every maintenance-panel post (DOVE_MP_LOG).  See DoveCentralProcessor.WrmpLog:
        /// the MP code is a cursor sprite on this machine, so this is the only way to read it as a
        /// number, in order, rather than decoding a bitmap by eye.
        /// </summary>
        private void FlushMpLog(string path)
        {
            if (string.IsNullOrEmpty(path)) return;
            System.Collections.Generic.List<string> log;
            lock (_machineLock) log = _machine.Cp.WrmpLog;
            if (log == null) return;
            System.IO.File.WriteAllLines(path, log.ToArray());
        }

        /// <summary>
        /// DOVE_CURSOR_DUMP=&lt;path&gt; -- write the 16x16 hardware cursor sprite as ASCII art.
        ///
        /// On Daybreak the MP code IS the cursor sprite: the ROM draws the digits into the 32-byte
        /// sprite buffer at ED00-ED1F.  So this is the ONLY ground truth for "what does the panel
        /// say" -- the @WRMP hook captures what the CP posts through SpecialSetMP, which is a
        /// different thing and can disagree (it missed the code being displayed, and it also picks
        /// up false positives from a stale IB byte).  Decoding four 4x5 digits by eye from a zoomed
        /// screenshot is exactly the step worth removing from the loop.
        /// </summary>
        private void DumpCursorSprite(string path)
        {
            if (string.IsNullOrEmpty(path)) return;
            byte[] buf;
            lock (_machineLock) buf = _machine.Display.GetCursorBuffer();
            var sb = new System.Text.StringBuilder();
            sb.AppendLine("16x16 hardware cursor sprite (ED00-ED1F), MSB first:");
            for (int row = 0; row < 16; row++)
            {
                // Intel byte order in the buffer: 2 bytes per 16-px row, ED00 = even byte.
                int hi = buf[row * 2], lo = buf[row * 2 + 1];
                int bits = (hi << 8) | lo;
                var line = new System.Text.StringBuilder();
                for (int b = 15; b >= 0; b--) line.Append(((bits >> b) & 1) != 0 ? '#' : '.');
                sb.AppendLine(string.Format("{0,2}: {1}  {2:X2} {3:X2}", row, line, hi, lo));
            }
            sb.AppendLine();
            sb.AppendLine("raw: " + BitConverter.ToString(buf));
            System.IO.File.WriteAllText(path, sb.ToString());
        }

        /// <summary>Returns bytes written; a null or empty path is a no-op returning 0.</summary>
        private long DumpSharedDramTo(string path)
        {
            if (string.IsNullOrEmpty(path)) return 0;
            byte[] ram, sram;
            // Hold the machine lock for the copy only -- writing 4 MB to disk under it would
            // stall the emulation thread for the duration of the I/O.
            lock (_machineLock)
            {
                ram = (byte[])_machine.Memory.SystemRaw.Clone();
                sram = (byte[])_machine.Memory.SramRaw.Clone();
            }
            System.IO.File.WriteAllBytes(path, ram);
            // The IOP-local SRAM is a SEPARATE array and was silently missing from this dump.
            // Anything the IOP keeps in its own memory -- the boot buffer, its DOB, its stacks --
            // lives there, so searching only the DRAM for it comes back empty and looks like proof.
            System.IO.File.WriteAllBytes(path + ".sram", sram);
            return ram.Length + sram.Length;
        }

        private MenuStrip BuildMenu()
        {
            var menu = new MenuStrip();

            var file = new ToolStripMenuItem("&File");
            file.DropDownItems.Add("&Load Floppy...", null, (s, e) => OnLoadFloppy());
            file.DropDownItems.Add("&Change Floppy...", null, (s, e) => OnChangeFloppy());
            file.DropDownItems.Add("&Eject Floppy", null, (s, e) => { lock (_machineLock) _machine.EjectFloppy(0); });
            file.DropDownItems.Add(new ToolStripSeparator());
            file.DropDownItems.Add("&Save Rigid Disk Now", null, (s, e) => SaveRigidDisk());
            file.DropDownItems.Add("&Dump Shared DRAM...", null, (s, e) => DumpSharedDram());
            file.DropDownItems.Add(new ToolStripSeparator());
            file.DropDownItems.Add("E&xit", null, (s, e) => Close());
            menu.Items.Add(file);

            var mach = new ToolStripMenuItem("&Machine");
            var pause = new ToolStripMenuItem("&Pause") { CheckOnClick = true };
            pause.CheckedChanged += (s, e) => _paused = pause.Checked;
            mach.DropDownItems.Add(pause);
            mach.DropDownItems.Add(new ToolStripSeparator());

            // Boot-device selection takes a raw byte: 0x63 = F1 .. 0x6C = F10.
            var boot = new ToolStripMenuItem("Boot &Device Key");
            for (int i = 0; i < 10; i++)
            {
                byte code = (byte)(0x63 + i);
                boot.DropDownItems.Add("F" + (i + 1) + "  (station " + code + ")", null,
                                       (s, e) => { SendKey(code, true); SendKey(code, false); });
            }
            mach.DropDownItems.Add(boot);
            mach.DropDownItems.Add("Send Scan &Code...", null, (s, e) => OnSendScanCode());
            mach.DropDownItems.Add(new ToolStripSeparator());
            mach.DropDownItems.Add("&Reset (Power Cycle)", null, (s, e) => PowerCycle(null));
            // Boot-device keys are only accepted during the selection screen early in the boot,
            // so choosing a device after the fact means power-cycling first.  These do both.
            var bootFrom = new ToolStripMenuItem("Power Cycle && &Boot From");
            for (int i = 0; i < 10; i++)
            {
                byte code = (byte)(0x63 + i);
                bootFrom.DropDownItems.Add("F" + (i + 1) + "  (0x" + code.ToString("X2") + ")", null,
                                           (s, e) => PowerCycle(code));
            }
            mach.DropDownItems.Add(bootFrom);
            menu.Items.Add(mach);

            MainMenuStrip = menu;
            return menu;
        }

        // ---- SDL setup (fork of DWindow-IO) ---------------------------------------------

        private void OnWindowLoad(object sender, EventArgs e)
        {
            if (SDL.SDL_Init(SDL.SDL_INIT_VIDEO) < 0)
                throw new InvalidOperationException("SDL_Init failed.");

            _sdlWindow = SDL.SDL_CreateWindowFrom(_displayBox.Handle);
            if (_sdlWindow == IntPtr.Zero)
                throw new InvalidOperationException("SDL_CreateWindowFrom failed.");

            _sdlRenderer = SDL.SDL_CreateRenderer(_sdlWindow, -1, SDL.SDL_RendererFlags.SDL_RENDERER_ACCELERATED);
            if (_sdlRenderer == IntPtr.Zero)
                _sdlRenderer = SDL.SDL_CreateRenderer(_sdlWindow, -1, SDL.SDL_RendererFlags.SDL_RENDERER_SOFTWARE);
            if (_sdlRenderer == IntPtr.Zero)
                throw new InvalidOperationException("SDL_CreateRenderer failed.");

            CreateDisplayTexture();

            _sdlRunning = true;
            _sdlThread = new Thread(SDLMessageLoopThread) { IsBackground = true, Name = "Doovke SDL" };
            _sdlThread.Start();

            SizeToDisplay();
            StartMachine();
            _refreshTimer.Start();
        }

        private void CreateDisplayTexture()
        {
            _textureLock.EnterWriteLock();
            try
            {
                SDL.SDL_SetHint(SDL.SDL_HINT_RENDER_SCALE_QUALITY, "nearest");
                if (_displayTexture != IntPtr.Zero) SDL.SDL_DestroyTexture(_displayTexture);
                _displayTexture = SDL.SDL_CreateTexture(
                    _sdlRenderer,
                    SDL.SDL_PIXELFORMAT_ARGB8888,
                    (int)SDL.SDL_TextureAccess.SDL_TEXTUREACCESS_STREAMING,
                    _displayWidth,
                    _displayHeight);
                if (_displayTexture == IntPtr.Zero)
                    throw new InvalidOperationException("SDL_CreateTexture failed.");
            }
            finally { _textureLock.ExitWriteLock(); }
        }

        private void SDLMessageLoopThread()
        {
            while (_sdlRunning)
            {
                SDL.SDL_Event e;
                if (SDL.SDL_WaitEventTimeout(out e, 100) != 0)
                {
                    if (e.type == SDL.SDL_EventType.SDL_QUIT) break;
                }
            }
        }

        /// <summary>Snapshot the machine's framebuffer, expand mono to ARGB, and present.</summary>
        private void UpdateAndRender()
        {
            byte[] frame; int w, h; long instr; bool changing; bool haveDisk;
            lock (_machineLock)
            {
                frame = _machine.RenderFrame(out w, out h);
                instr = _machine.IopInstructions;
                changing = _machine.DiskChangeInProgress;
                haveDisk = _machine.Io.Fdc.Drives[0] != null;
            }

            if (frame != null && w > 0 && h > 0)
            {
                if (w != _displayWidth || h != _displayHeight)
                {
                    _displayWidth = w; _displayHeight = h;
                    _32bppDisplayBuffer = new int[w * h];
                    CreateDisplayTexture();
                    SizeToDisplay();
                }

                // Mono (1 byte per pixel, non-zero = lit) -> opaque black/white ARGB.
                unchecked
                {
                    int white = (int)0xFFFFFFFF;
                    int black = (int)0xFF000000;
                    for (int i = 0; i < _32bppDisplayBuffer.Length; i++)
                        _32bppDisplayBuffer[i] = frame[i] != 0 ? white : black;
                }
                RenderDisplay();
            }

            if (_machineFault != null)
            {
                _statusLabel.Text = "MACHINE STOPPED -- " + _machineFault;
                return;
            }
            // 82586: Channel Attention rings from the driver, and commands completed by the
            // controller.  Attn climbing while cmds stays at 0 means the chip is not
            // answering; both climbing means the driver's requests are being completed.
            var enet = _machine.Io.Ethernet;
            var rdc = _machine.Io.Rdc;
            var pack = _machine.Io.Disk;

            // Rigid disk: operations, where the head is, and how much of the pack has been
            // formatted.  A real format makes several passes over the whole surface, so the
            // percentage is of DISTINCT sectors touched -- it goes to 100% once, not 25 times.
            string rdcText = rdc.DobOps == 0
                ? "RDC idle"
                : string.Format("RDC {0:N0} ops  C{1}/H{2}/S{3}  fmt {4:N0}/{5:N0} ({6:F1}%)",
                    rdc.DobOps, rdc.LastCyl, rdc.LastHead, rdc.LastSector,
                    pack.DistinctSectorsFormatted, D.IO.Micropolis1325.TotalSectors,
                    100.0 * pack.DistinctSectorsFormatted / D.IO.Micropolis1325.TotalSectors);

            var fdc = _machine.Io.Fdc;
            string fdcText = string.Format("FDC {0:N0}c/{1}s/{2}r",
                                           fdc.CommandCount, fdc.NoMediaStalls, fdc.ResetCount);

            _statusLabel.Text = string.Format(
                "IOP {0:N0}   CP {1:N0}   ENet {2:N0}/{3:N0}   {4}   " + fdcText + "   floppy: {5}{6}{7}",
                instr, _machine.Cp.InstructionCount,
                _machine.Io.EnetAttnWrites, enet.CommandsExecuted, rdcText,
                haveDisk ? System.IO.Path.GetFileName(_floppyPath ?? "(image)") : "(empty)",
                changing ? "   [changing disk]" : string.Empty,
                _paused ? "   [paused]" : string.Empty);
        }

        private void RenderDisplay()
        {
            if (_displayTexture == IntPtr.Zero) return;
            _textureLock.EnterReadLock();
            try
            {
                IntPtr bits; int pitch;
                if (SDL.SDL_LockTexture(_displayTexture, IntPtr.Zero, out bits, out pitch) != 0) return;
                Marshal.Copy(_32bppDisplayBuffer, 0, bits, _32bppDisplayBuffer.Length);
                SDL.SDL_UnlockTexture(_displayTexture);

                SDL.SDL_RenderCopy(_sdlRenderer, _displayTexture, IntPtr.Zero, IntPtr.Zero);
                SDL.SDL_RenderPresent(_sdlRenderer);
            }
            finally { _textureLock.ExitReadLock(); }
        }

        // ---- machine thread -------------------------------------------------------------

        private void StartMachine()
        {
            _machineRunning = true;
            _machineThread = new Thread(MachineLoop) { IsBackground = true, Name = "Doovke machine" };
            _machineThread.Start();
        }

        private void MachineLoop()
        {
            while (_machineRunning)
            {
                if (_paused) { Thread.Sleep(20); continue; }
                try
                {
                    lock (_machineLock)
                    {
                        for (int i = 0; i < BatchSteps && _machineRunning && !_paused; i++) _machine.Step();
                    }
                }
                catch (Exception e)
                {
                    // This thread is a background thread: an unhandled exception here would
                    // terminate the process with no explanation.  Stop and report instead.
                    _machineFault = e.GetType().Name + ": " + e.Message;
                    _machineRunning = false;
                }
            }
        }

        /// <summary>
        /// Power-cycle: tear the machine down and build a fresh one from the same ROM/EEPROM,
        /// remounting whatever image is in the drive.  Rebuilding rather than poking a reset line
        /// is what makes it a true cold start -- memory, devices and the control store all clear.
        /// If a boot-device key is given, it is queued for the selection screen (twice, as the
        /// firmware requires).
        /// </summary>
        private void PowerCycle(byte? bootKey)
        {
            _machineRunning = false;
            var t = _machineThread;
            if (t != null) t.Join(2000);

            lock (_machineLock)
            {
                // Flush the pack before tearing the machine down: a power cycle throws away
                // all of memory, and the rigid disk lives there until it is written out.
                string rigidPath = _machine.RigidDiskPath;
                if (!string.IsNullOrEmpty(rigidPath)) _machine.SaveRigidDisk();

                var fresh = new DoovkeMachine(_machine.BootRomPath, _machine.ConfigEepromPath);
                if (!string.IsNullOrEmpty(_floppyPath)) fresh.LoadFloppy(0, _floppyPath);
                // Re-attach the pack, or the new machine comes up blank AND with no path --
                // which would also make every later Save silently do nothing.
                if (!string.IsNullOrEmpty(rigidPath)) fresh.LoadRigidDisk(rigidPath);
                if (bootKey.HasValue) fresh.ScheduleBootDeviceKey(bootKey.Value);
                _machine = fresh;
            }

            StartMachine();
        }

        private void Shutdown()
        {
            _refreshTimer.Stop();
            _machineRunning = false;
            var mt = _machineThread; if (mt != null) mt.Join(2000);

            _sdlRunning = false;
            var ev = new SDL.SDL_Event { type = SDL.SDL_EventType.SDL_QUIT };
            SDL.SDL_PushEvent(ref ev);
            var st = _sdlThread; if (st != null) st.Join(1000);

            if (_displayTexture != IntPtr.Zero) SDL.SDL_DestroyTexture(_displayTexture);
            if (_sdlRenderer != IntPtr.Zero) SDL.SDL_DestroyRenderer(_sdlRenderer);
            if (_sdlWindow != IntPtr.Zero) SDL.SDL_DestroyWindow(_sdlWindow);
        }

        // ---- keyboard --------------------------------------------------------------------

        protected override void OnKeyDown(KeyEventArgs e)
        {
            // Alt releases a captured mouse, the same gesture Darkstar uses.
            if (e.Alt) { ReleaseMouse(); e.Handled = true; e.SuppressKeyPress = true; return; }

            byte station;
            if (DoovkeKeyboard.TryStation(e.KeyCode, out station))
            {
                // Auto-repeat resends KeyDown without an intervening KeyUp; the guest holds a
                // down/up bitmap, so repeats are redundant and only cost UART bandwidth.
                if (_keysDown.Add(e.KeyCode)) SendKey(station, true);
                e.Handled = true;
                e.SuppressKeyPress = true;
            }
            base.OnKeyDown(e);
        }

        protected override void OnKeyUp(KeyEventArgs e)
        {
            byte station;
            if (DoovkeKeyboard.TryStation(e.KeyCode, out station))
            {
                _keysDown.Remove(e.KeyCode);
                SendKey(station, false);
                e.Handled = true;
                e.SuppressKeyPress = true;
            }
            base.OnKeyUp(e);
        }

        /// <summary>Release every held key -- on focus loss, so nothing sticks down.</summary>
        private void ReleaseAllKeys()
        {
            foreach (var k in new System.Collections.Generic.List<Keys>(_keysDown))
            {
                byte station;
                if (DoovkeKeyboard.TryStation(k, out station)) SendKey(station, false);
            }
            _keysDown.Clear();
        }

        protected override void OnDeactivate(EventArgs e)
        {
            ReleaseAllKeys();
            ReleaseMouse();
            base.OnDeactivate(e);
        }

        private void SendKey(byte station, bool down)
        {
            lock (_machineLock) _machine.QueueKey(station, down);
        }

        // ---- mouse -----------------------------------------------------------------------
        //
        // Driven by polling rather than by mouse events: SDL owns the display panel's window
        // handle, so whether a given WM_MOUSEMOVE surfaces as a WinForms event or is consumed
        // by SDL is not something worth depending on.  Cursor.Position and Control.MouseButtons
        // read the real state regardless of who receives the messages.
        //
        // Motion is relative: while captured, the host pointer is warped back to the centre of
        // the display each poll and the distance it moved becomes the delta.

        private void PollMouse()
        {
            // Track the host buttons every poll so capture can trigger on the EDGE of a
            // press.  Level-triggering re-grabbed the pointer on any poll where a button
            // happened to be down -- which is every poll while a menu item is being clicked
            // or a dialog dragged, making the whole UI unusable.
            bool anyDown = Control.MouseButtons != MouseButtons.None;
            bool pressEdge = anyDown && !_hostButtonsWereDown;
            _hostButtonsWereDown = anyDown;

            if (!_mouseCaptured)
            {
                // Press inside the display captures, as in Darkstar -- but only when this
                // window is genuinely the one being used.  A modal dialog makes ActiveForm
                // the dialog, and an open menu drops down OVER the display area, so without
                // these the pointer gets grabbed the moment a menu is clicked.
                if (pressEdge
                    && Form.ActiveForm == this && ContainsFocus && !AnyMenuOpen()
                    && _displayBox.ClientRectangle.Contains(_displayBox.PointToClient(Cursor.Position)))
                {
                    CaptureMouse();
                }
                return;
            }

            var centre = _displayBox.PointToScreen(new System.Drawing.Point(
                _displayBox.ClientSize.Width / 2, _displayBox.ClientSize.Height / 2));
            var now = Cursor.Position;
            int dx = now.X - centre.X, dy = now.Y - centre.Y;
            if (dx != 0 || dy != 0)
            {
                // Display pixels are host pixels: the panel is sized 1:1 to the framebuffer.
                lock (_machineLock) _machine.QueueMouseMotion(dx, dy);
                Cursor.Position = centre;
            }

            var b = Control.MouseButtons;
            bool left = (b & MouseButtons.Left) != 0;
            bool right = (b & MouseButtons.Right) != 0;
            bool middle = (b & MouseButtons.Middle) != 0;

            // Straight through: the guest systems implement Point+Adjust chording for Menu
            // themselves, so synthesising it here would double up.
            SetButton(DoovkeKeyboard.ScanPoint, left, ref _pointDown);
            SetButton(DoovkeKeyboard.ScanAdjust, right, ref _adjustDown);
            SetButton(DoovkeKeyboard.ScanMenu, middle, ref _menuDown);
        }

        /// <summary>True while any menu drop-down is showing (it overlays the display area).</summary>
        private bool AnyMenuOpen()
        {
            if (MainMenuStrip == null) return false;
            foreach (ToolStripItem item in MainMenuStrip.Items)
            {
                ToolStripMenuItem mi = item as ToolStripMenuItem;
                if (mi != null && mi.DropDown != null && mi.DropDown.Visible) return true;
            }
            return false;
        }

        private void SetButton(byte scan, bool want, ref bool have)
        {
            if (want == have) return;
            have = want;
            lock (_machineLock) _machine.QueueMouseButton(scan, want);
        }

        private void CaptureMouse()
        {
            if (_mouseCaptured) return;
            _mouseCaptured = true;
            Cursor.Position = _displayBox.PointToScreen(new System.Drawing.Point(
                _displayBox.ClientSize.Width / 2, _displayBox.ClientSize.Height / 2));
            Cursor.Hide();
        }

        private void ReleaseMouse()
        {
            if (!_mouseCaptured) return;
            _mouseCaptured = false;
            // Let go of any button the guest still thinks is down.
            SetButton(DoovkeKeyboard.ScanPoint, false, ref _pointDown);
            SetButton(DoovkeKeyboard.ScanAdjust, false, ref _adjustDown);
            SetButton(DoovkeKeyboard.ScanMenu, false, ref _menuDown);
            Cursor.Show();
        }

        private readonly System.Collections.Generic.HashSet<Keys> _keysDown =
            new System.Collections.Generic.HashSet<Keys>();
        private bool _mouseCaptured;
        private bool _hostButtonsWereDown;
        private volatile string _machineFault;
        private bool _pointDown, _adjustDown, _menuDown;

        // ---- menu handlers ---------------------------------------------------------------

        private string PickImage()
        {
            using (var dlg = new OpenFileDialog())
            {
                dlg.Filter = "Floppy images (*.imd;*.img;*.dsk)|*.imd;*.img;*.dsk|All files (*.*)|*.*";
                return dlg.ShowDialog(this) == DialogResult.OK ? dlg.FileName : null;
            }
        }

        private void OnLoadFloppy()
        {
            string path = PickImage();
            if (path == null || !ImageIsReadable(path)) return;
            lock (_machineLock) _machine.LoadFloppy(0, path);
            _floppyPath = path;
        }

        private void OnChangeFloppy()
        {
            string path = PickImage();
            if (path == null || !ImageIsReadable(path)) return;
            // Goes through a real no-media gap so the guest latches the change.
            lock (_machineLock) _machine.ChangeFloppy(0, path);
            _floppyPath = path;
        }

        /// <summary>
        /// Report an unreadable image up front rather than letting it fail later on the
        /// machine thread, where the drive would just silently stay empty.
        /// </summary>
        private bool ImageIsReadable(string path)
        {
            string err = DoovkeMachine.ValidateImage(path);
            if (err == null) return true;
            string nl = Environment.NewLine;
            MessageBox.Show(this, "Cannot read this disk image:" + nl + nl + err + nl + nl
                            + "The drive has been left as it was.",
                            "Bad disk image", MessageBoxButtons.OK, MessageBoxIcon.Warning);
            return false;
        }

        private void OnSendScanCode()
        {
            using (var dlg = new Form())
            {
                dlg.Text = "Send Scan Code";
                dlg.FormBorderStyle = FormBorderStyle.FixedDialog;
                dlg.ClientSize = new System.Drawing.Size(260, 90);
                dlg.StartPosition = FormStartPosition.CenterParent;
                var lbl = new Label { Text = "Scan code (hex):", Left = 10, Top = 14, Width = 100 };
                var box = new TextBox { Left = 115, Top = 11, Width = 120, Text = "63" };
                var ok = new Button { Text = "Send", Left = 100, Top = 45, DialogResult = DialogResult.OK };
                dlg.Controls.Add(lbl); dlg.Controls.Add(box); dlg.Controls.Add(ok);
                dlg.AcceptButton = ok;
                if (dlg.ShowDialog(this) != DialogResult.OK) return;
                byte code;
                try { code = Convert.ToByte(box.Text.Trim(), 16); }
                catch { MessageBox.Show(this, "Not a hex byte: " + box.Text); return; }
                lock (_machineLock) _machine.QueueRaw(code);
            }
        }
    }
}
