using System;
using System.IO;
using D.CP;
using D.IOP;

namespace D.Doovke
{
    /// <summary>
    /// The Dove/Daybreak (Xerox 6085) machine.
    ///
    /// Darkstar is the DLion (8010) target; Doovke is Dove/Daybreak.  The two share the
    /// low-level parts (AM2901, Microinstruction, FloppyDisk) but diverge completely at the
    /// CP, the IOP and the display, which is why they are separate build targets rather than
    /// a runtime switch.
    ///
    /// The assembly below is exactly the configuration that boots the Offline Diagnostics
    /// system to its floppy prompt:  the IOP (80186) runs the real Opie boot firmware out of
    /// the boot ROM; the CP (AM2901 microengine) runs the Mesa microcode the IOP streams into
    /// the writable control store; and both share ONE physical DRAM array (DoveIOPMemory's
    /// SystemRaw) with no bridge -- the twisted bus is compensated for in-guest.
    /// </summary>
    public sealed class DoovkeMachine
    {
        private readonly DoveIOPMemory _mem;
        private readonly DoveIOPIO _io;
        private readonly DoveCentralProcessor _cp;
        private readonly I80186Pcb _pcb;
        private readonly i80186 _iop;
        private readonly byte[] _sysRam;
        private readonly int _ramMask;

        /// <summary>CP microinstructions executed per IOP instruction.</summary>
        private const int CpStepsPerIopInstruction = 4;

        public DoveIOPMemory Memory { get { return _mem; } }
        public DoveIOPIO Io { get { return _io; } }
        public DoveCentralProcessor Cp { get { return _cp; } }
        public i80186 Iop { get { return _iop; } }
        public DoveDisplayController Display { get { return _io.Display; } }

        public long IopInstructions { get; private set; }

        /// <summary>80186 clocks elapsed, so media timing can be expressed in real seconds.</summary>
        public long ElapsedClocks { get; private set; }
        public const long IopClockHz = 8000000;

        /// <summary>
        /// How long the drive must stay empty across a disk change.  Calibrated from the real
        /// machine (door open + reinsert + spin-up).  The guest only latches "media changed" if
        /// its poll/retry loop actually observes the not-ready window, so swapping the bytes
        /// underneath a running command -- or using too short a gap -- reproduces the classic
        /// "Gotek swapped too fast" bug: nothing goes not-ready, the latch never sets, and the
        /// guest keeps operating on the previous disk's identity.
        /// </summary>
        public const double DiskChangeGapSeconds = 3.0;

        // Keystrokes queued for a future instruction count.  The boot-device selection needs two
        // presses a couple of million instructions apart, which is awkward to do by hand right
        // after a power cycle, so callers can just queue them.
        private readonly System.Collections.Generic.List<KeyPress> _scheduledKeys =
            new System.Collections.Generic.List<KeyPress>();
        private struct KeyPress { public long At; public byte Code; public bool Down; }

        // Keyboard and mouse share one 8251, which holds exactly one unread byte, so all
        // input funnels through this FIFO and is handed over only when the Rx is free.
        // Order matters: a mouse report is the three bytes FF,dX,dY and nothing may be
        // interleaved into it or KEYMO would read a key event as a delta.
        private readonly System.Collections.Generic.Queue<byte> _input =
            new System.Collections.Generic.Queue<byte>();

        // The guest's mouse position is absolute, but the IOP only ever *accumulates* the
        // deltas we send into it, so we send relative motion and let the guest own the
        // position.  (Steering an absolute shadow toward the host pointer would desync the
        // moment the guest clamped at a screen edge: our shadow would keep travelling while
        // the guest sat still, and the pointer would then ignore that much motion coming
        // back.)  Motion accumulates here and is drained in >=1-pixel reports.
        private int _mousePendingX, _mousePendingY;

        private long _insertAtClock = -1;
        private string _pendingImage;
        private int _pendingDrive;

        /// <summary>True while a disk change is in progress (drive deliberately empty).</summary>
        public bool DiskChangeInProgress { get { return _insertAtClock >= 0; } }
        public bool Halted { get { return _iop.Halted; } }

        /// <summary>Paths this machine was built from, so a power cycle can rebuild it.</summary>
        public string BootRomPath { get; private set; }
        public string ConfigEepromPath { get; private set; }

        public DoovkeMachine(string bootRomPath, string configEepromPath)
        {
            if (string.IsNullOrEmpty(bootRomPath)) throw new ArgumentNullException("bootRomPath");
            BootRomPath = bootRomPath;
            ConfigEepromPath = configEepromPath;

            _mem = new DoveIOPMemory(bootRomPath);
            _io = new DoveIOPIO(_mem);

            if (!string.IsNullOrEmpty(configEepromPath))
            {
                _io.LoadConfigEeprom(File.ReadAllBytes(configEepromPath));
            }

            // Vertical-retrace period in IOP clocks.  The boot polls the keyboard off retrace,
            // so this paces the boot-device selection; 20000 is the value the proven boot uses
            // (the 210400 default is the true ~26.3 ms field rate and is far slower).
            // ★MEASURED 2026-07-30: this 10.5x-fast value is a bring-up hack with real consequences.
            // The true field is 210,400 cycles (~26.3 ms at 8 MHz); 20,000 makes retrace fire 10.5x too
            // often, and the firmware COUNTS retrace fields to time the 35 s boot-device timeout, so
            // that becomes ~3.3 s.  Against the true anchor the CP runs 1.13 M instr/s and the IOP
            // 0.32 M instr/s -- a 3.54:1 ratio, which is right (an 8 MHz 80186 does ~1-2 MIPS against a
            // Daybreak CP at ~7.3 M microinstructions/s) -- but the 8254, hard-coded at 12.8 CP
            // instructions per count, then ticks at ~88 kHz where the guest programs 3125 for 50 ms and
            // therefore expects 62.5 kHz.  So Pilot gets two timebases that disagree with each other and
            // with real time, by DIFFERENT factors.  That is the family the germ's MP 0935 came from.
            //
            // DOVE_RETRACE=<cycles> overrides it, so the true rate can be tested without disturbing the
            // proven ViewPoint boot that was tuned around 20,000.
            {
                string rp = Environment.GetEnvironmentVariable("DOVE_RETRACE");
                int rpv;
                _io.RetracePeriod = (!string.IsNullOrEmpty(rp) && int.TryParse(rp, out rpv) && rpv > 0)
                    ? rpv : 20000;
            }

            // The CP takes its microcode from the writable control store the IOP loads.
            _cp = new DoveCentralProcessor(_io.ControlStore);

            // CP <-> DRAM.  The CP is big-endian (Mesa) and addresses memory by 16-bit word.
            _sysRam = _mem.SystemRaw;
            _ramMask = _sysRam.Length - 1;
            _cp.ReadWord = ReadCpWord;
            _cp.WriteWord = WriteCpWord;

            // ---- CP <-> IOP control links ------------------------------------------------
            // The IOP owns the CP's run/reset lines and the two doorbell latches; the CP is
            // otherwise inert.  Without these the microcode loads but the CP never starts.
            //   OnWriteCSReg     : CSReg bit 9 = RUN (bit 8 is the IOP->CP doorbell)
            //   OnMesaReset      : the MesaReset line, asserted low
            //   OnReadMesaIntLatch: IN 0xB0 clears the CP->IOP doorbell latch
            //   OnMesaInterrupt  : CP rings the IOP (SetMPIntIOP)
            _io.OnWriteCSReg = v => _cp.WriteCSReg(v);
            _io.OnMesaReset = release => _cp.SetReset(!release);
            _io.OnReadMesaIntLatch = () => { _cp.MesaInterruptRequest = false; };
            _cp.OnMesaInterrupt = () => _io.RaiseMesaInterrupt();

            _pcb = new I80186Pcb();
            _iop = new i80186(_mem, _io, _pcb);
            // Let the RDC's FCB dump record IOP register state: the disk handler's failure is a
            // register-corruption signature (direct stores land, register-indirect ones do not),
            // so BX/DI/ES at the stall is the decisive reading.
            if (_io != null && _io.Rdc != null)
                _io.Rdc.CpuState = () => string.Format(
                    "CS:IP={0:X4}:{1:X4} AX={2:X4} BX={3:X4} CX={4:X4} DX={5:X4} SI={6:X4} DI={7:X4} BP={8:X4} SP={9:X4} DS={10:X4} ES={11:X4} SS={12:X4} F={13:X4}{14}",
                    _iop.GetCS, _iop.IP, _iop.GetAX, _iop.GetBX, _iop.GetCX, _iop.GetDX,
                    _iop.GetSI, _iop.GetDI, _iop.GetBP, _iop.GetSP, _iop.GetDS, _iop.GetES,
                    _iop.GetSS, _iop.Flags, _iop.Halted ? " HALTED" : "")
                    + "  VECTORS(last32 vec@CS:IP):" + _iop.RecentVectors();

            // Watch the ROM's client-condition word (BOOTSTRAPIOR + 0x30) and diskConditionWork.
            // A completed operation should notify the ROM exactly ONCE.  A second notify latches
            // preNotifyFlag, so the ROM's NEXT %WaitForCondition returns immediately without
            // blocking -- it then tests diskError (still 0 from the restore), falls through
            // JNC GoToRAM, and jumps into a boot buffer nothing ever filled.  That is consistent
            // with everything observed: two op-0 restores, no op 6 ever issued, and the IOP
            // wandering 12KB of zeros from 00A0:0000 to the undefined opcode at 00A0:30AE.
            if (_mem != null && _io != null && _io.Rdc != null)
            {
                int prevCond = -1;
                _mem.OnSramWrite = (addr, val) =>
                {
                    if (addr != 0x03E30 && addr != 0x03E31 && addr != 0x03A98 && addr != 0x03A99) return;
                    var w = _io.Rdc.LogWriter;
                    if (w == null) return;
                    int cur = _mem.ReadByte(0x03E30) | (_mem.ReadByte(0x03E31) << 8);
                    if (cur == prevCond && addr >= 0x03A98) return;
                    prevCond = cur;
                    try
                    {
                        w.WriteLine(string.Format(
                            "  COND W [{0:X5}]<-{1:X2}  ROMcond={2:X4} diskCondWork={3:X4}  from CS:IP={4:X4}:{5:X4}",
                            addr, val, cur,
                            _mem.ReadByte(0x03A98) | (_mem.ReadByte(0x03A99) << 8),
                            _iop.GetCS, _iop.IP));
                    }
                    catch { }
                };
            }

            // The 80186's integrated interrupt controller drives master-8259 IR6.  Without this
            // SyncInternalIrq() bails on a null _pcb and IR6 is never raised -- POST polls the
            // master IRR at port 0x00 for exactly 0x40 (IR6) and hangs when it never appears.
            _io.SetPcb(_pcb);

            // Straps read by the firmware, both already correct by default but called out
            // because the boot depends on them:
            //   _io.InputPortWord  bit 6 = 1  -> machine ID says Daybreak.  If this reads 0
            //                                   the .db loader skips every CP microcode block.
            //   _io.Display.TypeSize bit 0 = 0 -> 19" CRT (1152x861).  bit 0 = 1 is the 15"
            //                                   monitor (832x633); the firmware reads 0xECCC
            //                                   to size the bitmap.
            // Expose them here so the UI can offer a 15"/19" choice later.

            // Hardware-interrupt delivery.  IIOBus186 carries port I/O only, so the CPU takes
            // maskable interrupts ONLY through this callback -- leave it null and every POST
            // step that waits on an IRQ spins forever (the first is the keyboard-UART loopback
            // at FC00:38BF, which OUTs to port 0x30 and waits for the 8251's interrupt).
            _iop.InterruptAcknowledge = () => _io.AcknowledgeInterrupt();

            // The 80186 must be reset to land on the reset vector at 0xFFFF0; without this it
            // runs from an uninitialised state and never reaches POST, the floppy or the CP.
            _iop.Reset();
        }

        private ushort ReadCpWord(int wordAddress)
        {
            int b = (wordAddress << 1) & _ramMask;
            return (ushort)((_sysRam[b] << 8) | _sysRam[(b + 1) & _ramMask]);
        }

        private void WriteCpWord(int wordAddress, ushort value)
        {
            int b = (wordAddress << 1) & _ramMask;
            _sysRam[b] = (byte)(value >> 8);
            _sysRam[(b + 1) & _ramMask] = (byte)value;
        }

        /// <summary>Run one IOP instruction and the CP steps that accompany it.</summary>
        public void Step()
        {
            // Time base for the subsystems that need to know how far the machine has run
            // (memory timing, the RDC's host clock).  Leaving these at zero stalls POST.
            _mem.HostClock = IopInstructions;
            _mem.CurrentPC = _iop.InstructionAddress;
            _io.CurrentPC = _iop.InstructionAddress;
            _io.RdcHostClock = IopInstructions;

            // Arm the forward trace the instant the .db walk reads its last byte (the low half of
            // the CP WriteData BlockType word at boot-file offset 8601).
            if (_mem.CursorStopHit && !_iop.ForwardTraceArmed && _iop.ForwardTrace != null)
            {
                _iop.ForwardTracePrelude = _iop.RecentPcs(40);
                _iop.ForwardTraceArmed = true;
            }

            int clocks = _iop.Execute();
            _io.Tick(clocks);
            _cp.AdvancePitClocks(clocks);   // 8254 shares the display's 8 MHz timebase
            ElapsedClocks += clocks;

            // Fire any keystrokes that have come due.
            for (int i = _scheduledKeys.Count - 1; i >= 0; i--)
            {
                if (IopInstructions >= _scheduledKeys[i].At)
                {
                    QueueKey(_scheduledKeys[i].Code, _scheduledKeys[i].Down);
                    _scheduledKeys.RemoveAt(i);
                }
            }

            PumpInput();

            // Complete a pending disk change once the drive has been empty long enough.
            if (_insertAtClock >= 0 && ElapsedClocks >= _insertAtClock)
            {
                _insertAtClock = -1;
                if (!string.IsNullOrEmpty(_pendingImage)) LoadFloppy(_pendingDrive, _pendingImage);
                _pendingImage = null;
            }
            // Execute() no-ops while the CP is halted, so this is safe before the IOP starts it.
            _cp.Execute(CpStepsPerIopInstruction);
            IopInstructions++;
        }

        public void Run(long iopInstructions)
        {
            for (long i = 0; i < iopInstructions; i++) Step();
        }

        /// <summary>
        /// Deliver a key to the guest.  The boot-device selection at the "Boot device?" stage
        /// and every later prompt (including the Offline Diagnostics floppy prompt) arrive this
        /// way; without one the boot parks waiting for a boot device.
        /// </summary>
        public void InjectKey(byte scanCode)
        {
            _io.InjectKeyboard(scanCode);
        }

        /// <summary>
        /// Queue a key press or release.  The wire byte is the KeyStation number, with bit 7
        /// set for a release; the IOP's KEYMO handler maintains Pilot's down/up bitmap from
        /// that (down = 0, up = 1, so a press clears the bit).
        /// </summary>
        public void QueueKey(byte station, bool down)
        {
            byte b = down ? station : (byte)(station | DoovkeKeyboard.ReleaseFlag);
            lock (_input) _input.Enqueue(b);
        }

        /// <summary>Queue a raw wire byte, exactly as the keyboard would have sent it.</summary>
        public void QueueRaw(byte b)
        {
            lock (_input) _input.Enqueue(b);
        }

        /// <summary>
        /// Accumulate host mouse motion, in display pixels.  Motion is coalesced and split
        /// into reports as the UART drains, so callers can feed this as fast as they like.
        /// </summary>
        public void QueueMouseMotion(int dx, int dy)
        {
            lock (_input) { _mousePendingX += dx; _mousePendingY += dy; }
        }

        /// <summary>Press or release a mouse button (stations 13/14/15).</summary>
        public void QueueMouseButton(byte station, bool down)
        {
            QueueKey(station, down);
        }

        /// <summary>
        /// Drain one byte of input if the UART can take it.  Mouse motion is only turned into
        /// a report when the queue is otherwise empty, which coalesces a burst of host motion
        /// into a single delta instead of flooding the 8251.
        /// </summary>
        private void PumpInput()
        {
            if (_io.KeyboardRxReady) return;

            lock (_input)
            {
                if (_input.Count == 0 && (_mousePendingX != 0 || _mousePendingY != 0))
                {
                    int dx = Clamp127(_mousePendingX);
                    int dy = Clamp127(_mousePendingY);
                    _mousePendingX -= dx; _mousePendingY -= dy;
                    _input.Enqueue(0xFF);            // mouse-report prefix
                    _input.Enqueue(SignMagnitude(dx));
                    _input.Enqueue(SignMagnitude(dy));
                }
                if (_input.Count > 0) _io.InjectKeyboard(_input.Dequeue());
            }
        }

        private static int Clamp127(int d) { return d > 127 ? 127 : (d < -127 ? -127 : d); }

        /// <summary>Deltas are sign-magnitude: bit 7 is the sign, bits 0-6 the magnitude.</summary>
        private static byte SignMagnitude(int d)
        {
            return d < 0 ? (byte)(0x80 | (-d)) : (byte)d;
        }

        /// <summary>Deliver a key once the machine reaches the given instruction count.</summary>
        /// <summary>Schedule a press (down = true) or release (down = false).</summary>
        public void ScheduleKey(byte scanCode, long atInstruction, bool down)
        {
            _scheduledKeys.Add(new KeyPress { At = atInstruction, Code = scanCode, Down = down });
        }

        /// <summary>
        /// Schedule a full keystroke, held down for holdInstructions.  A hold is REQUIRED: the
        /// guest polls the down/up bitmap periodically, so a press and release delivered
        /// back-to-back are both consumed between two polls and the key is never seen at all
        /// (measured -- station 27 typed this way echoed nothing, while the same station held
        /// by a human types fine).
        /// </summary>
        public void ScheduleKeystroke(byte scanCode, long atInstruction, long holdInstructions)
        {
            ScheduleKey(scanCode, atInstruction, true);
            ScheduleKey(scanCode, atInstruction + holdInstructions, false);
        }

        /// <summary>
        /// Queue the boot-device selection for a machine that has just been powered on.
        /// The firmware needs the key TWICE: the SelectionLoop consumes the first press to
        /// highlight the icon and the second to boot.  Defaults match the proven boot.
        /// </summary>
        public void ScheduleBootDeviceKey(byte scanCode)
        {
            // Press only, NO release: measured 2026-07-26, sending station|0x80 after the
            // press stops the machine booting (FDC 0 / CP 0) where press-only boots cleanly.
            // The boot ROM's SelectionLoop runs before Pilot and stashes the raw byte at
            // HexValue, so it does not use the down/up bitmap convention KEYMO keeps later.
            ScheduleKey(scanCode, 12000000, true);
            ScheduleKey(scanCode, 14000000, true);
        }

        /// <summary>
        /// Attach the rigid-disk pack to a file.  WITHOUT this the pack exists only in memory
        /// and everything written to it -- including a multi-pass format that takes an hour --
        /// is discarded when the process exits.  Missing file = a blank (unformatted) pack,
        /// which is the correct starting state for Offline Diagnostics' Format.
        /// </summary>
        public void LoadRigidDisk(string path)
        {
            RigidDiskPath = path;
            _io.Disk.Load(path);
        }

        /// <summary>Flush the pack to its file; no-op if none was attached.</summary>
        public void SaveRigidDisk()
        {
            if (!string.IsNullOrEmpty(RigidDiskPath)) _io.Disk.Save();
        }

        public string RigidDiskPath { get; private set; }

        /// <summary>Mount an image immediately (use at power-on; for a swap use ChangeFloppy).</summary>
        public void LoadFloppy(int drive, string path)
        {
            // A malformed image throws out of the IMD parser.  On the deferred-insert path
            // that happens inside Step(), on the machine thread, where an unhandled exception
            // takes the whole process down -- so record it and leave the drive empty instead.
            try
            {
                var disk = new D.IO.FloppyDisk(path);
                string wrongMachine = DescribeWrongMachineMedia(disk);
                if (wrongMachine != null)
                {
                    _io.Fdc.Drives[drive] = null;
                    LastFloppyError = System.IO.Path.GetFileName(path) + ": " + wrongMachine;
                    return;
                }
                _io.Fdc.Drives[drive] = disk;
                LastFloppyError = null;
            }
            catch (Exception e)
            {
                _io.Fdc.Drives[drive] = null;
                LastFloppyError = path + ": " + e.Message;
            }
        }

        /// <summary>
        /// Reject 8-inch DLion (Dandelion, 1108/8010) media, which is NOT readable by a 6085.
        ///
        /// A Daybreak has a 5.25-inch drive: 40 cylinders, 2 heads, 9 x 512-byte MFM sectors on
        /// tracks 1-39 and a 16-sector FM/MFM IPL track 0.  Dandelion 8-inch media is 77 cylinders,
        /// 15 x 512 with 26 x 128 / 26 x 256 tracks and 500 kbps FM -- ~10,624 bytes per track.
        ///
        /// Without this guard the mismatch is silent and expensive.  Mounting the 1108 Medley set
        /// (dir "1108_medley_1.1_imd", header "IMD DMK Tracks:77x10624 DSMD") produces a boot that
        /// looks like an emulator fault at every level: the ROM reads C=5 H=0 S=1 for the loader it
        /// jumps into, gets Dandelion Mesa data instead of 8086 code (F1 00 6D 03 ... where a 6085
        /// disk holds B8 00 00 8E C0 ... = MOV AX,0 / MOV ES,AX), executes it, branches out of the
        /// 512-byte buffer, runs 5,862 instructions through zeroed memory, wraps 0x0FFFF -> 0x00000,
        /// walks the interrupt vector table and takes an INT 3 off the first 0xCC byte it meets.  The
        /// ROM's error reporter then spins ~500,000,000 times on the umbilical UART's TxRDY bit, and
        /// the control store is never written.  Every one of those symptoms is downstream of the
        /// wrong file being mounted, and each was chased as a bug in its own right.
        /// </summary>
        private static string DescribeWrongMachineMedia(D.IO.FloppyDisk disk)
        {
            int maxCyl = -1, sectors = 0, size = 0;
            for (int c = 0; c < 77; c++)
            {
                for (int h = 0; h < 2; h++)
                {
                    D.IO.Track t = null;
                    try { t = disk.GetTrack(c, h); } catch { }
                    if (t == null) continue;
                    if (c > maxCyl) { maxCyl = c; }
                    if (c >= 1 && sectors == 0) { sectors = t.SectorCount; size = t.SectorSize; }
                }
            }
            // 40-cylinder media is a Daybreak disk; a few images run a track or two long.  Anything
            // reaching cylinder 44 is 8-inch geometry, and the 1108 set reaches 76.
            if (maxCyl >= 44)
                return "8-inch DLion (Dandelion 1108/8010) media -- " + (maxCyl + 1) + " cylinders"
                     + (sectors > 0 ? ", " + sectors + "x" + size : "")
                     + ".  The 6085 has a 5.25-inch drive (40 cylinders, 9x512); this disk cannot be"
                     + " read by a Daybreak.  Use the 1186 floppy set, or run it under Darkstar (DLion).";
            return null;
        }

        /// <summary>Why the last image failed to mount, or null if it mounted.</summary>
        public string LastFloppyError { get; private set; }

        /// <summary>
        /// Try to parse an image without mounting it, so a caller can report a bad one before
        /// disturbing the running machine.  Returns null if it is readable.
        /// </summary>
        public static string ValidateImage(string path)
        {
            try
            {
                var disk = new D.IO.FloppyDisk(path);
                return DescribeWrongMachineMedia(disk);   // null when it is usable Daybreak media
            }
            catch (Exception e) { return e.Message; }
        }

        /// <summary>
        /// Remove the diskette.  From here every media-requiring FDC command stalls, so the
        /// IOP's software timeout fires and the guest sees notReady.
        /// </summary>
        public void EjectFloppy(int drive)
        {
            _io.Fdc.Drives[drive] = null;
        }

        /// <summary>
        /// Swap diskettes the way a human does: eject, leave the drive genuinely empty for
        /// DiskChangeGapSeconds, then insert.  Routing every change through a real no-media gap
        /// is what makes the guest notice -- the diagnostics' own retry loop times out while the
        /// drive is empty, latches the change, and then reads the new disk cleanly.
        /// The insertion happens from Step(), so callers never block.
        /// </summary>
        public void ChangeFloppy(int drive, string path)
        {
            ChangeFloppy(drive, path, DiskChangeGapSeconds);
        }

        public void ChangeFloppy(int drive, string path, double gapSeconds)
        {
            EjectFloppy(drive);
            _pendingDrive = drive;
            _pendingImage = path;
            _insertAtClock = ElapsedClocks + (long)(gapSeconds * IopClockHz);
        }

        /// <summary>
        /// Render the current display into a byte-per-pixel mono frame.
        /// Returns null if the display has not been programmed yet.
        /// </summary>
        public byte[] RenderFrame(out int width, out int height)
        {
            var display = _io.Display;
            display.RenderMono();
            width = display.Width;
            height = display.Height;
            return display.Frame;
        }
    }
}
