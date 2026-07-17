using System;
using System.Collections.Generic;
using System.Linq;
using D.IOP;

namespace DoveTrace
{
    class Program
    {
        static DoveIOPMemory _mem;
        static DoveIOPIO _io;
        static I80186Pcb _pcb;
        static i80186 _cpu;
        static long _intCount = 0;
        static int _lastVector = -1;
        static bool _forceRetrace = System.Environment.GetEnvironmentVariable("DOVE_FORCE_RETRACE") == "1";
        static long _videoAt = -1;
        static bool _hw30 = false;
        static int _hw30trace = 0;
        static long _v30 = 0;
        static long _fecaa = 0;
        static int _mpWrites = 0;
        static bool _hw23 = false;
        static bool _hw35 = false;
        static int _wTs = -1, _wLogs = 0;   // mesaProcessorTask taskState@0x7C62 watch
        static long _v23post = 0;
        static long _dbgInstr = 0;
        static int _rd0FF = 0, _wr0FF = 0, _linkVecW = 0;
        static int _map0FFw = -1, _map0FFlogs = 0;
        static int _mpEs = -1, _mpSi = -1, _mpTcb = -1;
        static System.Collections.Generic.HashSet<int> _flowSeen = new System.Collections.Generic.HashSet<int>();
        static long _holeReads = 0, _holeWrites = 0;
        static System.Collections.Generic.HashSet<int> _holeReadPages = new System.Collections.Generic.HashSet<int>();
        static System.Collections.Generic.HashSet<int> _holeWritePages = new System.Collections.Generic.HashSet<int>();
        static string _lastSpriteSig = "";
        static System.Collections.Generic.List<System.Collections.Generic.KeyValuePair<long, byte[]>> _spriteSnaps = new System.Collections.Generic.List<System.Collections.Generic.KeyValuePair<long, byte[]>>();
        static string RenderSprite(byte[] b, bool swap)
        {
            var sb = new System.Text.StringBuilder();
            for (int r = 0; r < 16; r++)
            {
                int w = swap ? (b[2 * r] | (b[2 * r + 1] << 8)) : ((b[2 * r] << 8) | b[2 * r + 1]);
                for (int col = 15; col >= 0; col--) sb.Append(((w >> col) & 1) != 0 ? '#' : '.');
                sb.Append('\n');
            }
            return sb.ToString();
        }
        static long _lastFdcInstr = 0;
        static long _lastFdcCmdCount = 0;
        static D.CP.DoveCentralProcessor _cp;
        static long _cpStartedAt = -1;
        static int _cpDoorbells = 0, _cpAcks = 0, _cpCsLogs = 0, _clrLog = 0;
        static int _v35count = 0, _v35postGMT = 0, _fcbCmdWrites = 0; static long _v35lastInstr = 0;
        static int _copyCount = 0, _copyMin = 0x7FFFFFFF, _copyMax = -1; static System.Collections.Generic.List<string> _copySample = new System.Collections.Generic.List<string>();
        class Mpc { public ushort first, last; public int count; public long firstInstr, lastInstr; }
        static System.Collections.Generic.Dictionary<int, Mpc> _mpCand = new System.Collections.Generic.Dictionary<int, Mpc>();
        static ushort _cpCsPrev = 0;
        static long _cpInitTraps = 0;
        static Dictionary<int, long> _hwVec = new Dictionary<int, long>();
        static Dictionary<int, long> _ipSeg = new Dictionary<int, long>();
        static Dictionary<int, long> _svc = new Dictionary<int, long>();
        static Dictionary<string, HashSet<int>> _svcSites = new Dictionary<string, HashSet<int>>();

        static void Main(string[] args)
        {
            string romPath = args.Length > 0 ? args[0]
                : "dove_build_kit/firmware/boot_rom/dove_iop_V2_K_merged_load_at_FC000.bin";

            _mem = new DoveIOPMemory(romPath);
            _mem.CmdByteLog = new System.Collections.Generic.List<string>();
            _io = new DoveIOPIO(_mem);
            _io.Display.TraceRegisters = true;
            _io.RetracePeriod = int.Parse(System.Environment.GetEnvironmentVariable("DOVE_RETRACE") ?? "20000");
            string eepromPath = System.Environment.GetEnvironmentVariable("DOVE_EEPROM")
                ?? "C:/Users/techf/Desktop/Darkstar/.claude/worktrees/laughing-gould-251705/dove_build_kit/firmware/config/U128_IOP_sn_071888_08CA_8kCS_3.7mb.bin";
            try { _io.LoadConfigEeprom(System.IO.File.ReadAllBytes(eepromPath)); Console.WriteLine("Config EEPROM loaded: " + eepromPath); }
            catch (Exception e) { Console.WriteLine("EEPROM load FAILED: " + e.Message); }
            _io.ConfigEeprom.ReadLog = new List<int>();
            _io.Fdc.Log = new List<string>();
            _io.CpLoadLog = new List<string>();

            // ---- Dove Central Processor: executes the microcode the IOP loads ----
            _cp = new D.CP.DoveCentralProcessor(_io.ControlStore);
            _cp.Trace = new List<string>();
            _cp.RhLog = new List<string>();
            _cp.FyLog = new List<string>();
            _cp.ShiftLog = new List<string>();
            _cp.XferLog = new List<string>();
            _cp.QLog = new List<string>();
            _cp.R5Log = new List<string>();
            _cp.WriteLog = new List<string>();
            _cp.ReadLog = new List<string>();
            _cp.OpLog = new List<string>();
            _cp.StackLog = new List<string>();
            _cp.R0Log = new List<string>();
            _cp.TrapLog = new List<string>();
            _cp.SemaLog = new List<string>();
            _cp.LoopTrace = new List<string>();
            _cp.LoopTraceFrom = long.Parse(Environment.GetEnvironmentVariable("DOVE_LOOPTRACE_FROM") ?? "0");   // XFER-entry trace window (CPi)
            _cp.MapReadLog = new List<string>();
            _cp.EscLog = new List<string>();
            _cp.WrmpLog = new List<string>();      // every @WRMP (zESC alpha 0x77) = THE MP-post chokepoint
            _cp.LoopLog = new List<string>();
            { var _la = Environment.GetEnvironmentVariable("DOVE_LOOP_ADDR"); _cp.LoopAddr = string.IsNullOrEmpty(_la) ? -1 : Convert.ToInt32(_la, 16); }
            _cp.LoopFrom = int.Parse(Environment.GetEnvironmentVariable("DOVE_LOOP_FROM") ?? "2147483647");
            _cp.RingFrom = int.Parse(Environment.GetEnvironmentVariable("DOVE_RING_FROM") ?? "2147483647");
            _cp.StkTrapLog = new List<string>();   // TechRef Table 2.11 stack over/underflow detector
            _cp.OpLog = new List<string>();
            _cp.LinkLog = new List<string>();
            _cp.IbLog = new List<string>();
            _cp.LgcLog = new List<string>();
            _cp.LinkVecWriteLog = new List<string>();
            _cp.CaptureLog = new List<string>();
            _cp.XferChainLog = new List<string>();
            _cp.XferTraceFrom = long.Parse(Environment.GetEnvironmentVariable("DOVE_XFER_FROM") ?? "10000000");
            _cp.MapArrRead = new List<string>();
            _cp.SdReadLog = new List<string>();
            _cp.IbLogFrom = int.Parse(Environment.GetEnvironmentVariable("DOVE_IBLOG_FROM") ?? "2226");
            _cp.IbLogTo   = int.Parse(Environment.GetEnvironmentVariable("DOVE_IBLOG_TO")   ?? "2360");
            _cp.OpLogFrom = int.Parse(Environment.GetEnvironmentVariable("DOVE_OPLOG_FROM") ?? "1820");
            _cp.OpLogTo   = int.Parse(Environment.GetEnvironmentVariable("DOVE_OPLOG_TO")   ?? "2600");
            _cp.IORgnWriteLog = new System.Collections.Generic.List<string>();
            byte[] sysRam = _mem.SystemRaw;
            int ramMask = sysRam.Length - 1;
            // CP is big-endian (Mesa); MAR is a 16-bit-word physical address.
            _cp.ReadWord = a => { int b = (a << 1) & ramMask; ushort v = (ushort)((sysRam[b] << 8) | sysRam[(b + 1) & ramMask]);
                // Hole hypothesis: CP words 0x10000-0x3FFFF = phys 0x20000-0x80000 = the 128k-VRAM..512k-mainmem hole.
                if (a >= 0x10000 && a < 0x40000) { _holeReads++; if (_holeReadPages.Add(a >> 8) && _holeReadPages.Count <= 40) Console.WriteLine("*** germ READS HOLE CP word " + a.ToString("X5") + " (phys 0x" + (a<<1).ToString("X5") + " = " + ((a<<1)>>10) + "k) = " + v.ToString("X4") + " @CPi " + _cp.InstructionCount); }
                // TEMP: trace the germ's reads during the early FindStartOfIORegion + GetHandlerIORegionPtr compute
                // (before it first touches vp 0x100 @CPi 1829): map array 0x40000-0x40200 + segment table 0x52000-0x52040.
                if (_cp.InstructionCount < 95 && _rd0FF < 60)
                { int rp = ((v&0x1F)<<8)|(v>>8); int bs = ((v&0xFF)<<8)|(v>>8);
                  Console.WriteLine("*** germ reads " + a.ToString("X5") + " = " + v.ToString("X4") + " (real0x"+rp.ToString("X3")+" bswap0x"+bs.ToString("X4")+" mod256=0x"+((rp)&0xFF).ToString("X2")+")  @CPi " + _cp.InstructionCount + " CPaddr " + _cp.CurrentAddress.ToString("X3")); _rd0FF++; }
                return v; };
            _cp.WriteWord = (a, v) => { int b = (a << 1) & ramMask; sysRam[b] = (byte)(v >> 8); sysRam[(b + 1) & ramMask] = (byte)v;
                if (a >= 0x48204 && a <= 0x48220 && _linkVecW < 60) { Console.WriteLine("*** [CP] LINKVEC WRITE CP word 0x" + a.ToString("X5") + " = " + v.ToString("X4") + " @CPi " + _cp.InstructionCount + " CPaddr " + _cp.CurrentAddress.ToString("X3")); _linkVecW++; }
                if (a >= 0x10000 && a < 0x40000) { _holeWrites++; if (_holeWritePages.Add(a >> 8) && _holeWritePages.Count <= 40) Console.WriteLine("*** germ WRITES HOLE CP word " + a.ToString("X5") + " (phys 0x" + (a<<1).ToString("X5") + " = " + ((a<<1)>>10) + "k) = " + v.ToString("X4") + " @CPi " + _cp.InstructionCount + " CPaddr " + _cp.CurrentAddress.ToString("X3")); }
                if (a == 0x400FF && _wr0FF < 30) { Console.WriteLine("*** WRITE map[vp 0x0FF] (CP word 0x400FF) = " + v.ToString("X4") + " -> real 0x" + ((((v&0x1F)<<8)|(v>>8))).ToString("X3") + "  @CPi " + _cp.InstructionCount + " CPaddr " + _cp.CurrentAddress.ToString("X3")); _wr0FF++; }
                // MP-code hunt: track locations written with small MP-code-like values (hex 0x0100-0x0FFF
                // covers both hex 05xx/09xx and decimal 500-999) so the germ's maintenance-panel word surfaces.
                if (v >= 0x0100 && v <= 0x0FFF) {
                    Mpc e; if (!_mpCand.TryGetValue(a, out e)) { e = new Mpc { first = v, last = v, count = 1, firstInstr = _cp.InstructionCount, lastInstr = _cp.InstructionCount }; _mpCand[a] = e; }
                    else { e.last = v; e.count++; e.lastInstr = _cp.InstructionCount; }
                }
                if (a == 0x58008 && _fcbCmdWrites++ < 20) Console.WriteLine("*** CP posts FCB word6 (0x58008 = timeOfDayIsValid|command) = " + v.ToString("X4") + " @IOPinstr " + _dbgInstr + " CPinstr " + _cp.InstructionCount + " CPaddr " + _cp.CurrentAddress.ToString("X3"));
                // Capture the destination range of one boot-wait-loop iteration's block copy (CPinstr window
                // around the first word6 post) to see if it's a small FCB access or a large blit-sized block.
                if (_cp.InstructionCount >= 6457640 && _cp.InstructionCount <= 6458140) {
                    _copyCount++; if (a < _copyMin) _copyMin = a; if (a > _copyMax) _copyMax = a;
                    if (_copySample.Count < 24) _copySample.Add("CPword " + a.ToString("X5") + " = " + v.ToString("X4") + " @" + _cp.CurrentAddress.ToString("X3"));
                } };
            _io.OnWriteCSReg = v => {
                bool wasRun = (_cpCsPrev & 0x0200) != 0, isRun = (v & 0x0200) != 0;
                if (isRun != wasRun && _cpCsLogs++ < 40) Console.WriteLine("*** CSReg=" + v.ToString("X4") + " (" + (isRun ? "RUN" : "HALT") + ") @IOPinstr " + _dbgInstr);
                _cpCsPrev = v; _cp.WriteCSReg(v);
            };
            _io.OnMesaReset = release => { _cp.SetReset(!release); Console.WriteLine("*** MesaReset " + (release ? "RELEASE" : "assert") + " @instr " + _dbgInstr); };
            _io.OnReadMesaIntLatch = () => {
                if (_cpAcks == 30) {
                    byte[] r = _mem.SystemRaw; int M = r.Length - 1;
                    Console.WriteLine("=== IORegion dump @IN0xB0#30 instr " + _dbgInstr + " ===");
                    Console.Write("  phys A4000 locks+segtab: ");
                    for (int i = 0; i < 0x50; i++) Console.Write(r[(0xA4000 + i) & M].ToString("X2") + (((i & 1) == 1) ? " " : ""));
                    Console.WriteLine();
                    int seg44 = r[0xA4044 & M] | (r[0xA4045 & M] << 8);
                    int B = (0xA0000 + 16 * seg44) & M;
                    Console.WriteLine("  segEntry[A4044] LE=" + seg44.ToString("X4") + " (CP BE=" + ((r[0xA4044 & M] << 8) | r[0xA4045 & M]).ToString("X4") + ") -> FCB base B=" + B.ToString("X5"));
                    Console.Write("  FCB B+0..0x20: ");
                    for (int i = 0; i < 0x20; i++) Console.Write(r[(B + i) & M].ToString("X2") + (((i & 1) == 1) ? " " : ""));
                    Console.WriteLine();
                    Console.WriteLine("  cmd byte B+0xD=" + r[(B + 0xD) & M].ToString("X2") + " CP-word B+0xC(BE)=" + ((r[(B + 0xC) & M] << 8) | r[(B + 0xD) & M]).ToString("X4")
                        + " | mesaHasLock A4000=" + ((r[0xA4001 & M] << 8) | r[0xA4000 & M]).ToString("X4") + " iopReqLock A4002=" + ((r[0xA4003 & M] << 8) | r[0xA4002 & M]).ToString("X4"));
                }
                if (_cpAcks++ < 20) {
                    int cur = _mem.ReadByte(0x4354) | (_mem.ReadByte(0x4355) << 8);
                    int tic = _mem.ReadByte(0x7C5A) | (_mem.ReadByte(0x7C5B) << 8);
                    int ts = _mem.ReadByte(0x7C62);
                    Console.WriteLine("*** IOP IN 0xB0 @IOPinstr " + _dbgInstr + "  curTCB[4354]=" + cur.ToString("X4") + " taskICPtr[7C5A]=" + tic.ToString("X4") + " taskState[7C62]=" + ts.ToString("X2"));
                }
                _cp.ClearMesaInterrupt();
            };
            _cp.OnMesaInterrupt = () => {
                byte sIrrBefore = _io.PicSlave.Request;
                _io.RaiseMesaInterrupt();
                if (_cpDoorbells++ < 12) {
                    var s = _io.PicSlave; var m = _io.PicMaster;
                    Console.WriteLine("*** CP SetMPIntIOP (doorbell, IR5) @IOPinstr " + _dbgInstr + " CPinstr " + _cp.InstructionCount + " CPaddr " + _cp.CurrentAddress.ToString("X3")
                        + "\n        CPU IF=" + (_cpu.FlagIF?1:0) + " Halted=" + (_cpu.Halted?1:0)
                        + " | SLAVE base=" + s.VectorBase.ToString("X2") + " IRR=" + s.Request.ToString("X2") + "(was " + sIrrBefore.ToString("X2") + ") IMR=" + s.InterruptMask.ToString("X2") + " ISR=" + s.InService.ToString("X2") + " (IR5 masked=" + ((s.InterruptMask&0x20)!=0?1:0) + ")"
                        + " | MASTER IRR=" + m.Request.ToString("X2") + " IMR=" + m.InterruptMask.ToString("X2") + " ISR=" + m.InService.ToString("X2"));
                }
            };
            _cp.OnClearMesaInterrupt = () => {
                bool wasPending = (_io.PicSlave.Request & 0x20) != 0;
                _io.ClearMesaLatch();
                if (_cp.InstructionCount > 6451900 && _clrLog++ < 20)
                    Console.WriteLine("*** CP ClrMPIntIOP (clears IR5 latch, wasPending=" + (wasPending?1:0) + ") @IOPinstr " + _dbgInstr + " CPinstr " + _cp.InstructionCount + " CPaddr " + _cp.CurrentAddress.ToString("X3"));
            };
            string floppyPath = System.Environment.GetEnvironmentVariable("DOVE_FLOPPY");
            if (!string.IsNullOrEmpty(floppyPath))
            {
                try { _io.Fdc.Drives[0] = new D.IO.FloppyDisk(floppyPath); Console.WriteLine("Floppy drive 0: " + floppyPath); }
                catch (Exception e) { Console.WriteLine("Floppy load FAILED: " + e.Message); }
            }
            _pcb = new I80186Pcb();
            _cpu = new i80186(_mem, _io, _pcb);
            _io.SetPcb(_pcb);
            _cpu.InterruptAcknowledge = () => { int v = _io.AcknowledgeInterrupt(); if (v >= 0) { _intCount++; _lastVector = v; if (v == 0x30) _v30++; long hc; _hwVec.TryGetValue(v, out hc); _hwVec[v] = hc + 1; if (v == 0x23 && _dbgInstr > 11_999_999) { _v23post++; if (!_hw23) { _hw23 = true; _hw30trace = 40; Console.WriteLine("=== vector 0x23 (keyboard IR3) handler POST-INJECT @instr " + _dbgInstr + " ==="); } }
                if (v == 0x35) { _v35count++; _v35lastInstr = _dbgInstr; if (_dbgInstr > 16400000 && _v35postGMT++ < 8) Console.WriteLine("=== vector 0x35 (mesa IR5) POST-readGMT @IOPinstr " + _dbgInstr + " (cmd byte @B0010=" + _mem.SystemRaw[0xB0010].ToString("X2") + " @B0011=" + _mem.SystemRaw[0xB0011].ToString("X2") + ") ==="); if (!_hw35) { _hw35 = true; int ip = _mem.ReadByte(0xD4) | (_mem.ReadByte(0xD5) << 8); int cs = _mem.ReadByte(0xD6) | (_mem.ReadByte(0xD7) << 8); Console.WriteLine("=== vector 0x35 (mesa IR5) FIRST @IOPinstr " + _dbgInstr + "  IVT[0x35]->" + cs.ToString("X4") + ":" + ip.ToString("X4") + " phys " + (((cs << 4) + ip) & 0xFFFFF).ToString("X5") + " ==="); _hw30trace = 160; } } } return v; };
            _cpu.OnSoftwareInterrupt = (v, site) =>
            {
                long c; _svc.TryGetValue(v, out c); _svc[v] = c + 1;
                if (v == 0x6A || v == 0x6B || v == 0x74 || v == 0x78 || v == 0x64)
                {
                    string key = v.ToString("X2");
                    if (!_svcSites.ContainsKey(key)) _svcSites[key] = new HashSet<int>();
                    _svcSites[key].Add(site);
                }
            };
            // Watch NON-ZERO writes to the DDC cursor-pattern port (ED00-1F) -- the MP
            // handler (DayBreakMP) rasterizes the boot code straight to this port.
            _io.Display.OnCursorWrite = (port, v) =>
            {
                // The retrace task's copy loop lives at FD619-FD62B; any OTHER writer
                // of the cursor buffer is DayBreakMP (the MP-code rasterizer).
                int pc = _cpu.InstructionAddress;
                bool retraceCopy = pc >= 0xFD619 && pc <= 0xFD62B;
                if (!retraceCopy && _mpWrites < 60)
                {
                    Console.WriteLine("  NON-RETRACE cursor write [" + port.ToString("X4") + "]=" + v.ToString("X2") + " @PC " + Hex5(pc));
                    _mpWrites++;
                }
            };
            _cpu.Reset();

            Console.WriteLine("== Dove boot ROM trace ==");
            Console.WriteLine("ROM: " + romPath);
            Console.WriteLine(String.Format("Reset -> {0} : {1}", Hex5(_cpu.InstructionAddress), Bytes(_cpu.InstructionAddress, 8)));
            Console.WriteLine();

            long BUDGET = 50_000_000;
            { var _b = Environment.GetEnvironmentVariable("DOVE_BUDGET"); if (!string.IsNullOrEmpty(_b)) BUDGET = long.Parse(_b); }
            const long EPOCH = 1_000_000;
            const int VERBOSE = 2;

            var epochSet = new HashSet<int>();
            long instr = 0;
            int lastAddr = -1, sameAddrRun = 0;
            int prevDistinct = -1, stableEpochs = 0;
            string stopReason = "budget exhausted";

            // Ring buffer of recent distinct-address instructions (skips long delay loops).
            var ring = new LinkedList<string>();
            int ringLastAddr = -1;

            var watch = new HashSet<int>(new[] {
                0xFE4FF,   // floppy driver init
                0xFE387,   // sense loop returns CF=1 (ST0=0x80)
                0xFE384,   // sense loop returns CF=0
                0xFE493,   // -> Specify path
                0xFE584,   // Specify command issue
                0xFE426,   // Recalibrate command issue
                0xFE4AD }); // Recalibrate loop
            var watchHit = new HashSet<int>();

            while (instr < BUDGET)
            {
                int addr = _cpu.InstructionAddress;
                _dbgInstr = instr;
                _mem.HostClock = instr;
                // TEMP: watch the vp 0x0FF IORegion map entry (CP word 0x400FF) — does it start 0x3D05 (real 0x53D)
                // and only later become 0x5F05 (real 0x55F)?  That's the FindStartOfIORegion ordering race.
                { ushort mv = _cp.ReadWord(0x400FF); if (mv != _map0FFw && _map0FFlogs < 30) { _map0FFw = mv;
                    Console.WriteLine("*** map[vp 0x0FF] (CP word 0x400FF) = " + mv.ToString("X4") + " -> real 0x" + ((((mv&0x1F)<<8)|(mv>>8))).ToString("X3") + "  @IOPinstr " + instr + " CPi " + _cp.InstructionCount); _map0FFlogs++; } }
                { int seg = (addr >> 16) & 0xF; long sc; _ipSeg.TryGetValue(seg, out sc); _ipSeg[seg] = sc + 1; }

                // Watch the mesaProcessorTask TCB lifecycle (operator answer 7).  Healthy
                // taskState@0x7C62: 00->04(InitializeTask)->08(SystemLoop dispatch)->87(first
                // wait).  Low-nibble 2 == jammed (suspect #1: taskICPtr Null).
                if (instr > 12_500_000 && _wLogs < 300)
                {
                    int ts = _mem.ReadByte(0x7C62);
                    if (ts != _wTs)
                    {
                        int tic = _mem.ReadByte(0x7C5A) | (_mem.ReadByte(0x7C5B) << 8);
                        int cur = _mem.ReadByte(0x4354) | (_mem.ReadByte(0x4355) << 8);
                        int icb = _mem.ReadByte(0x430E) | (_mem.ReadByte(0x430F) << 8);
                        Console.WriteLine("*** mesaTCB taskState[7C62] " + (_wTs < 0 ? "??" : _wTs.ToString("X2")) + "->" + ts.ToString("X2")
                            + " @instr " + instr + " PC " + Hex5(addr) + " taskICPtr[7C5A]=" + tic.ToString("X4")
                            + " curTCB[4354]=" + cur.ToString("X4") + " ICB[430E]=" + icb.ToString("X4"));
                        _wTs = ts; _wLogs++;
                    }
                }

                if (watch.Contains(addr) && watchHit.Add(addr))
                    Console.WriteLine(String.Format("  WATCH {0} @instr {1}  AX={2:X4} [0x12 via DS]", Hex5(addr), instr, _cpu.GetAX));

                // Bindweed umbilical debugger-detect flow: FFCE2=detect, FFD1C=NoDebugger(AX=FF), FFD20=Debugger(AX=00),
                // FFC98=post-detect CMP, FFCA3=->StartOPIE, FFCA1=debugger-slave spin, FFF84/FFFC4=send-to-debugger.
                if ((addr == 0xFFCEE || addr == 0xFFD1C || addr == 0xFFD20 || addr == 0xFFC95 || addr == 0xFFC98
                        || addr == 0xFFCA3 || addr == 0xFFCA1 || addr == 0xFFF84 || addr == 0xFFFC4) && _flowSeen.Add(addr))
                    Console.WriteLine("### FLOW @" + Hex5(addr) + " instr " + instr + "  " + _cpu);

                if (_hw30trace > 0)
                {
                    Console.WriteLine(String.Format("  30> {0}  {1,-20}  {2}", Hex5(addr), Bytes(addr, 5), _cpu));
                    _hw30trace--;
                }

                // TEMP: trace the mesaProcessor task's execution right after the germ's command doorbell
                // (ring @IOP 14392517) to see whether it reads fcb.command (0xB0010) and dispatches, or
                // acks IR5 and returns to wait without servicing.
                if (instr >= 14392517 && instr < 14392760)
                    Console.WriteLine(String.Format("  MT> {0}  {1,-18}  {2}", Hex5(addr), Bytes(addr, 5), _cpu));

                // Simulate a boot-device selection: poke the selection byte at phys
                // 0x3DB2 (0x3DA:0x12) with a device code (0x63 + index).  The
                // SelectionLoop @FEC41 consumes it, highlights the icon, and boots.
                long pokeAt = long.Parse(System.Environment.GetEnvironmentVariable("DOVE_POKE_AT") ?? "0");
                byte pokeCode = (byte)Convert.ToInt32(System.Environment.GetEnvironmentVariable("DOVE_POKE_CODE") ?? "63", 16);
                if (pokeAt > 0 && instr == pokeAt)
                {
                    _io.InjectKeyboard(pokeCode);
                    Console.WriteLine("*** injected keyboard scancode 0x" + pokeCode.ToString("X2") + " (IR3) @instr " + instr + "  [press 1]");
                }
                // Second press ~250ms later (the diagnostics disk needs the floppy function key pressed twice).
                // DOVE_POKE_DELAY = IOP-instruction gap for the second press (default ~2M ≈ 250ms).
                long pokeDelay = long.Parse(System.Environment.GetEnvironmentVariable("DOVE_POKE_DELAY") ?? "2000000");
                if (pokeAt > 0 && pokeDelay > 0 && instr == pokeAt + pokeDelay)
                {
                    _io.InjectKeyboard(pokeCode);
                    Console.WriteLine("*** injected keyboard scancode 0x" + pokeCode.ToString("X2") + " (IR3) @instr " + instr + "  [press 2]");
                }
                if (addr == 0xFECAA) _fecaa++;
                if (_io.Fdc.Log != null && _io.Fdc.Log.Count == 19 && !_hw30) { _hw30 = true; Console.WriteLine("=== after first Read (log=19), Timer1 mode FF5E=" + _pcb.GetRegisterWord(0x5E).ToString("X4") + " cnt=" + _pcb.GetRegisterWord(0x58).ToString("X4") + " maxA=" + _pcb.GetRegisterWord(0x5A).ToString("X4") + " ==="); }
                if (_hw30 && addr == 0xFE164 && _mpEs < 20) { Console.WriteLine("  FE160 IN FFC8 (FinalDMACount) = 0x" + _cpu.GetAX.ToString("X4")); if (_mpEs == 0) _hw30trace = 260; _mpEs++; }
                if (addr == 0xFC527 && _cpu.GetAX == 7 && instr > 12_000_000 && false) { }
                if (_hw30 && addr == 0xFC55C && _mpSi < 0) { _mpSi = _cpu.GetSI; _mpTcb = _cpu.GetDS << 4; _mpEs = _cpu.GetES << 4;
                    Console.Write("  BEFORE: MP TCB " + _cpu.GetDS.ToString("X4") + ":" + _mpSi.ToString("X4") + " = ");
                    for (int i = 0; i < 16; i++) Console.Write(_mem.ReadByte(_mpTcb + _mpSi + i).ToString("X2"));
                    Console.WriteLine(" (tag=" + _mem.ReadByte(_mpTcb+_mpSi).ToString("X2") + " state=" + _mem.ReadByte(_mpTcb+_mpSi+0xe).ToString("X2") + ")"); }
                if (_hw30 && addr == 0xFC562 && _mpTcb >= 0) {
                    Console.Write("  AFTER:  MP TCB = ");
                    for (int i = 0; i < 16; i++) Console.Write(_mem.ReadByte(_mpTcb + _mpSi + i).ToString("X2"));
                    Console.WriteLine(" (tag=" + _mem.ReadByte(_mpTcb+_mpSi).ToString("X2") + " state=" + _mem.ReadByte(_mpTcb+_mpSi+0xe).ToString("X2") + ")");
                    Console.Write("  sysQ[146] head/tail: "); for (int i=0;i<6;i++) Console.Write(_mem.ReadByte(_mpEs+0x146+i).ToString("X2")+" ");
                    Console.Write(" | timerQ[16e]: "); for (int i=0;i<6;i++) Console.Write(_mem.ReadByte(_mpEs+0x16e+i).ToString("X2")+" ");
                    Console.WriteLine(); _mpTcb = -9; _hw30trace = 900; }

                if (instr < VERBOSE)
                {
                    Console.WriteLine(String.Format("{0,6}  {1}  {2,-22}  {3}",
                        instr, Hex5(addr), Bytes(addr, 6), _cpu));
                }

                // Record into the ring only when the address changes, so tight
                // delay/poll loops don't flush the useful history.
                if (addr != ringLastAddr)
                {
                    ring.AddLast(String.Format("{0,9}  {1}  {2,-22}  {3}", instr, Hex5(addr), Bytes(addr, 6), _cpu));
                    if (ring.Count > 80) ring.RemoveFirst();
                    ringLastAddr = addr;
                }

                if (addr == lastAddr) { sameAddrRun++; } else { sameAddrRun = 0; lastAddr = addr; }
                if (sameAddrRun > 3_000_000)
                {
                    stopReason = "STUCK: same address (self-loop / JMP $) at " + Hex5(addr);
                    break;
                }

                epochSet.Add(addr);

                try
                {
                    int cyc = _cpu.Execute();
                    _io.Tick(cyc);
                    if (_forceRetrace && instr > 4_000_000) _io.PicSlave.ForceUnmask(0);   // EXPERIMENT: unmask slave IR0 only after POST/init (in the idle loop)
                }
                catch (Exception e)
                {
                    stopReason = "EXCEPTION: " + e.Message;
                    break;
                }

                if (_cpu.Halted)
                {
                    stopReason = "HLT at " + Hex5(_cpu.InstructionAddress);
                    break;
                }

                instr++;

                // Always step the CP (Execute no-ops while halted, and tracks the
                // halt->run edge so InitTrap re-enters at bank 0 / loc 0).
                if (_cp.Running && _cpStartedAt < 0) { _cpStartedAt = instr; Console.WriteLine("*** CP first RUN @instr " + instr + " (bank " + _cp.Bank + ", entry " + _cp.CurrentAddress.ToString("X3") + ") word0=" + _io.ControlStore.GetWord(0,0).ToString("X12") + " word1=" + _io.ControlStore.GetWord(0,1).ToString("X12") + " word1F=" + _io.ControlStore.GetWord(0,0x1F).ToString("X12")); }
                if (_cp.Running && (instr % 2000000 == 0)) Console.WriteLine("   [CP-run check @instr " + instr + "] word0=" + _io.ControlStore.GetWord(0,0).ToString("X12") + " running=" + _cp.Running + " CPinstrs=" + _cp.InstructionCount);
                _cp.Execute(4);
                if (_cp.InitTrapCount != _cpInitTraps) { _cpInitTraps = _cp.InitTrapCount; if (_cpInitTraps <= 12) Console.WriteLine("*** CP InitTrap #" + _cpInitTraps + " (unhalt->bank0/loc0) @IOPinstr " + instr); }

                if (_io.Fdc.CommandCount != _lastFdcCmdCount) { _lastFdcCmdCount = _io.Fdc.CommandCount; _lastFdcInstr = instr; }

                // Snapshot the 16x16 cursor sprite (where DayBreakMP rasterizes the MP code) whenever it changes.
                if (instr % 50000 == 0)
                {
                    byte[] spr = _io.Display.GetCursorBuffer();
                    string sig = BitConverter.ToString(spr);
                    if (sig != _lastSpriteSig) { _lastSpriteSig = sig; if (_spriteSnaps.Count < 160) _spriteSnaps.Add(new KeyValuePair<long, byte[]>(instr, spr)); }
                }

                if (instr % EPOCH == 0)
                {
                    int distinct = epochSet.Count;
                    Console.WriteLine(String.Format(
                        "  [epoch {0,3}M] {1} hwInt={3} lastV={4:X2} | mISR={15:X2} mIRR={16:X2} mIMR={11:X2} | sISR={17:X2} sIRR={18:X2} sIMR={12:X2} | vertret raised={13} D0={14}",
                        instr / 1_000_000, Hex5(_cpu.InstructionAddress),
                        _io.Display.RegisterLog.Count, _cpu.HardwareInterruptCount, _cpu.LastHardwareVector,
                        _pcb.Timer2Mode, _pcb.Timer2Count, _pcb.Timer2MaxA,
                        _pcb.InternalRequest, _pcb.InternalInService, _pcb.InternalMask,
                        _io.PicMaster.InterruptMask, _io.PicSlave.InterruptMask,
                        _io.RetraceCount, _io.RetraceClearReads,
                        _io.PicMaster.InService, _io.PicMaster.Request,
                        _io.PicSlave.InService, _io.PicSlave.Request));
                    Console.WriteLine("            fdc: cmds=" + _io.Fdc.CommandCount + " reads=" + _io.Fdc.ReadCount +
                                      " lastC/H/R=" + _io.Fdc.LastReadC + "/" + _io.Fdc.LastReadH + "/" + _io.Fdc.LastReadR +
                                      " lastFdc@" + _lastFdcInstr + " wcs=" + (_io.HighPortWrites.Count > 0 && System.Linq.Enumerable.Any(_io.HighPortWrites, kv => kv.Key < 0xE000)));

                    // (Diagnostic note only: report when the CP control-store load begins.)
                    long wcsWords = 0; _io.HighPortWrites.TryGetValue(0x8000, out wcsWords);

                    // Once VIDEO is enabled, run until the floppy load quiesces (no FDC
                    // command for 8M instructions) -- that is the natural end of the boot
                    // load (or a stall), which is where the CP hand-off should occur.
                    if (_io.Display.VideoEnabled)
                    {
                        if (_videoAt < 0) _videoAt = instr;
                        else if (_lastFdcInstr > 0 && instr - _lastFdcInstr > long.Parse(Environment.GetEnvironmentVariable("DOVE_QUIESCE_GAP") ?? "8000000"))
                        {
                            stopReason = "FLOPPY LOAD QUIESCED (last FDC cmd @instr " + _lastFdcInstr + ", read " + _io.Fdc.ReadCount + " sectors)";
                            break;
                        }
                    }
                    if (distinct < 400 && distinct == prevDistinct) stableEpochs++;
                    else stableEpochs = 0;

                    prevDistinct = distinct;
                    epochSet.Clear();
                }
            }

            Console.WriteLine();
            Console.WriteLine("== recent distinct-address history (last " + ring.Count + ") ==");
            foreach (var line in ring) Console.WriteLine(line);
            Console.WriteLine();
            Console.WriteLine("== STOP: " + stopReason + " ==");
            Console.WriteLine("Instructions executed: " + instr);
            Console.WriteLine("Hardware interrupts serviced: " + _intCount + " (last vector 0x" + _lastVector.ToString("X2") + ")");
            Console.WriteLine("Final: " + _cpu);
            Console.WriteLine("Bytes @ final IP: " + Bytes(_cpu.InstructionAddress, 8));
            Console.WriteLine();

            Console.WriteLine("PCB reloc = " + _pcb.RelocationRegister.ToString("X4") +
                              (_pcb.IsMemoryMapped ? " (mem)" : " (I/O @ " + _pcb.WindowBase.ToString("X4") + ")") +
                              (_pcb.IsIRmxMode ? " iRMX" : ""));
            Console.WriteLine("ControlReg=" + _io.ControlReg.ToString("X4") + "  ResetReg=" + _io.ResetReg.ToString("X4"));

            Console.WriteLine();
            Console.WriteLine("LED / POST progress (" + _io.LedHistory.Count + " distinct):");
            Console.WriteLine("  " + string.Join(" -> ", _io.LedHistory.Select(v => v.ToString("X4"))));

            Console.WriteLine();
            // --- 93C46 EEPROM: what the firmware read vs the loaded image ---
            var ee = _io.ConfigEeprom;
            Console.WriteLine("93C46 reads by firmware (" + (ee.ReadLog == null ? 0 : ee.ReadLog.Count) + "):");
            if (ee.ReadLog != null)
            {
                int shown = 0; bool mismatch = false;
                foreach (int r in ee.ReadLog)
                {
                    int addr = (r >> 16) & 0x3F, word = r & 0xFFFF;
                    int img = ee.Words[addr];
                    bool ok = word == img;
                    if (!ok) mismatch = true;
                    if (shown++ < 20) Console.WriteLine(String.Format("  addr {0,2}: read {1:X4}  image {2:X4}  {3}", addr, word, img, ok ? "" : "<-- MISMATCH"));
                }
                Console.WriteLine("  reads match image: " + (!mismatch));
                // Distinct words read + how many times (identifies config fields the loader consults).
                var cnt = new SortedDictionary<int, int>();
                foreach (int r in ee.ReadLog) { int a = (r >> 16) & 0x3F; int c; cnt.TryGetValue(a, out c); cnt[a] = c + 1; }
                Console.Write("  distinct words read (addr x count): ");
                foreach (var kv in cnt) Console.Write("w" + kv.Key + "=" + ee.Words[kv.Key].ToString("X4") + "(" + kv.Value + ")  ");
                Console.WriteLine();
            }
            Console.WriteLine("Image words: [0..7]=" + string.Join(" ", System.Linq.Enumerable.Range(0, 8).Select(i => ee.Words[i].ToString("X4"))) +
                              "  [62]=" + ee.Words[62].ToString("X4") + " [63]=" + ee.Words[63].ToString("X4"));
            Console.WriteLine();
            Console.WriteLine("VERTRET: fields raised=" + _io.RetraceCount + "  vector-0x30 delivered=" + _v30 + "  0xD0 clear-reads=" + _io.RetraceClearReads);
            Console.WriteLine("Device-selected branch (FECAA) hits=" + _fecaa + "  selection byte 0x3DB2 now=0x" + _mem.ReadByte(0x3DB2).ToString("X2"));
            Console.WriteLine("vector 0x23 (keyboard IR3) delivered AFTER injection: " + _v23post);
            Console.WriteLine("8272 FDC resets=" + _io.Fdc.ResetCount + "  commands(" + _io.Fdc.Log.Count + "): " + string.Join("  ", _io.Fdc.Log));
            Console.Write("IP execution by 64KB segment: ");
            foreach (var kv in _ipSeg.OrderBy(k => k.Key)) Console.Write("seg" + kv.Key.ToString("X") + "=" + kv.Value + "  ");
            Console.WriteLine();
            Console.WriteLine("Floppy DMA span (IOP linear): [" + (_io.MinDmaDest < 0 ? "none" : "0x" + _io.MinDmaDest.ToString("X5")) +
                              " .. 0x" + _io.MaxDmaDest.ToString("X5") + "]  totalBytes=" + _io.DmaByteTotal);
            Console.WriteLine("8272 totals: commands=" + _io.Fdc.CommandCount + "  reads=" + _io.Fdc.ReadCount +
                              "  lastRead C/H/R=" + _io.Fdc.LastReadC + "/" + _io.Fdc.LastReadH + "/" + _io.Fdc.LastReadR +
                              "  lastFDCcmd@instr=" + _lastFdcInstr);
            Console.Write("HIGH-PORT (>=0x8000) writes by bucket: ");
            foreach (var kv in _io.HighPortWrites) Console.Write("0x" + kv.Key.ToString("X4") + "=" + kv.Value + "  ");
            Console.WriteLine(_io.HighPortWrites.Count == 0 ? "(none)" : "");
            Console.WriteLine("CP-load control sequence (" + (_io.CpLoadLog == null ? 0 : _io.CpLoadLog.Count) + " events):");
            if (_io.CpLoadLog != null) Console.WriteLine("  " + string.Join("  ", _io.CpLoadLog));
            Console.WriteLine();
            Console.WriteLine("=== Dove CP ===");
            Console.WriteLine("CP running=" + _cp.Running + " startedAt=" + _cpStartedAt + " CPinstrs=" + _cp.InstructionCount +
                              " @addr=" + _cp.CurrentAddress.ToString("X3") + " bank=" + _cp.Bank + " mesaIntReq=" + _cp.MesaInterruptRequest);
            if (_cp.USnapshot != null) {
                Console.WriteLine("=== U-register DIFF (first @666 snapshot -> end): candidates for uPTC/uWP/uWDC ===");
                var u = _cp.URegs; var s = _cp.USnapshot;
                for (int i=0;i<256;i++) if (u[i]!=s[i]) Console.WriteLine("   U["+i.ToString("X2")+"]: "+s[i].ToString("X4")+" -> "+u[i].ToString("X4")+"  (delta "+((ushort)(u[i]-s[i]))+")");
            } else Console.WriteLine("=== U snapshot NOT captured (never reached @666 after 6.5M) ===");
            Console.WriteLine("CP timer: fires=" + _cp.TimerFireCount + " intStatReads=" + _cp.IntStatReads + " mesaIntBrFires=" + _cp.MesaIntBrFires + " (TimerPeriod=" + _cp.TimerPeriod + ")  IE=" + _cp.IE + " SetIeCount=" + _cp.SetIeCount);
            Console.Write("CP function hits: ");
            foreach (var kv in _cp.FuncHits) Console.Write(kv.Key + "=" + kv.Value + "  ");
            Console.WriteLine();
            Console.WriteLine("CP RH loads (" + (_cp.RhLog == null ? 0 : _cp.RhLog.Count) + "): " + (_cp.RhLog == null ? "" : string.Join("  ", _cp.RhLog)));
            Console.WriteLine("=== WCS scan: SetMPIntIOP(fSfY1 fY1)/ClrMPIntIOP(fY0)/IOXIn-<-ib(ESC alpha read) sites ===");
            {
                var cs = _io.ControlStore;
                int nSet=0, nClr=0;
                for (int bnk=0; bnk<4; bnk++) for (int a=0; a<4096; a++) {
                    ulong w = cs.GetWord(bnk, a); if (w==0) continue;
                    var mi = new D.CP.Microinstruction(w);
                    if (mi.fSfY == D.CP.FunctionSelectFY.fyNorm && (int)mi.fY==1) { if(nSet<40) Console.WriteLine("   SetMPIntIOP @ bank"+bnk+":"+a.ToString("X3")+"  "+mi.Disassemble(-1)); nSet++; }
                    if (mi.fSfY == D.CP.FunctionSelectFY.fyNorm && (int)mi.fY==0) { nClr++; }
                }
                Console.WriteLine("   TOTAL SetMPIntIOP="+nSet+"  ClrMPIntIOP="+nClr);
                for (int bnk=0; bnk<4; bnk++) {
                    int cnt=0, lo=99999, hi=-1; var gaps=new List<string>(); int runStart=-1;
                    for (int a=0; a<4096; a++){ bool nz = cs.GetWord(bnk,a)!=0; if(nz){cnt++; if(a<lo)lo=a; if(a>hi)hi=a; if(runStart>=0 && a-runStart>8){gaps.Add(runStart.ToString("X3")+"-"+a.ToString("X3"));} runStart=a;} }
                    if(cnt>0) Console.WriteLine("   bank"+bnk+": "+cnt+" non-zero words, addr range "+lo.ToString("X3")+".."+hi.ToString("X3")+"  gaps(>8): "+string.Join(",", gaps.Take(12)));
                }
            }
            Console.WriteLine("=== GERM MESA OPCODE STREAM (IBDisp dispatches) ===");
            Console.WriteLine("=== GERM MESA OPCODE STREAM (IBDisp dispatches) ===");
            if (_cp.OpLog != null) foreach (var l in _cp.OpLog) Console.WriteLine("   " + l);
            {
                byte[] mp = _mem.SystemRaw; int MP2 = mp.Length - 1;
                int e3 = (mp[(0x40003<<1)&MP2]<<8)|mp[((0x40003<<1)+1)&MP2];
                int rp3 = ((e3 & 0x1F)<<8) | (e3>>8);
                Console.WriteLine("=== map entry vp3 @word 0x40003 = 0x" + e3.ToString("X4") + " -> real page 0x" + rp3.ToString("X3") + " => pRequest word 0x" + ((rp3<<8)|0xA0).ToString("X5") + " ===");
                int pr=(rp3<<8)|0xA0; Console.Write("   pRequest bytes: "); for(int i=0;i<8;i++){int b=((pr+i)<<1)&MP2; Console.Write(((mp[b]<<8)|mp[b+1]).ToString("X4")+" ");} Console.WriteLine();
            }
            Console.WriteLine("=== GERM IORegion READS (<-MD of word>=0x50000 before @666) ===");
            if (_cp.ReadLog != null) foreach (var l in _cp.ReadLog) Console.WriteLine("   " + l);
            Console.WriteLine("=== GERM BOOT REQUEST (last MDR<- stores before @666 wait) ===");
            if (_cp.WriteLog != null) foreach (var l in _cp.WriteLog) Console.WriteLine("   " + l);
            {
                byte[] rq = _mem.SystemRaw; int MM = rq.Length - 1;
                Action<string,int,int> dumpw = (label, w0, n) => {
                    Console.Write("   " + label + " (word 0x" + w0.ToString("X5") + " phys 0x" + (w0<<1).ToString("X6") + "): ");
                    for (int i=0;i<n;i++){ int b=((w0+i)<<1)&MM; Console.Write(((rq[b]<<8)|rq[b+1]).ToString("X4")+" "); }
                    Console.WriteLine();
                };
                Console.WriteLine("=== boot-request candidate regions (CP big-endian words) ===");
                dumpw("IORegion   ", 0x52000, 16);
                dumpw("IORegion+10", 0x52010, 16);
                dumpw("pReq vp3?  ", 0x483A0, 8);
                dumpw("low 0xCE   ", 0x000CE, 16);
                dumpw("word 58000 ", 0x58000, 16);
            }
            Console.WriteLine("=== MAP vacant-STAMP writes: count=" + _mem.MapStampCount + " Seg2(>=0x90000)=" + _mem.MapStampSeg2
                + " sysRange=[" + (_mem.MapStampMinSys<0?0:_mem.MapStampMinSys).ToString("X5") + "," + _mem.MapStampMaxSys.ToString("X5") + "]"
                + "  (if Seg2=0 and max<0x90000 → fill stopped at 64KB boundary = confirmed crossover bug)");
            Console.WriteLine("=== vector 0x35 (mesa IR5) total fires=" + _v35count + " lastFire@IOPinstr=" + _v35lastInstr + " ===");
            Console.WriteLine("=== IB _ib[0]/refill trace (find the stale _ib[0]=0x37 even-lane read; DLion traps/refills on Empty) ===");
            foreach (var l in _cp.IbLog) Console.WriteLine("   " + l);
            Console.WriteLine("=== XFER-ENTRY L2/link diagnostic: does L2's low bit track the entry byte-parity? (xcO=IBPtr<-1 vs xcE=Cin<-pc16) ===");
            foreach (var l in _cp.LinkLog) Console.WriteLine("   " + l);
            Console.WriteLine("=== MONITOR SIZE: 0xECCC strap reads + programmed quadwords/line (17=19\"/861 lines, 13=15\"/633) ===");
            Console.WriteLine("   0xECCC (_typeSize) = 0x" + _io.Display.TypeSize.ToString("X2") + " (bit0=0 => 19\")  reads-by-firmware=" + _io.Display.TypeSizeReads);
            Console.WriteLine("   RegQuadwords(0xEC88) programmed = " + _io.Display.QuadwordsPerLine + " qw/line  => " + (_io.Display.QuadwordsPerLine >= 16 ? "19\" (861 lines)" : _io.Display.QuadwordsPerLine == 0 ? "(not programmed yet)" : "15\" (633 lines)"));
            Console.WriteLine("=== HOLE hypothesis: phys 0x20000-0x80000 (128k VRAM..512k mainmem gap) = CP words 0x10000-0x40000 ===");
            Console.WriteLine("   germ CP reads=" + _holeReads + " (distinct 512B pages " + _holeReadPages.Count + "), writes=" + _holeWrites + " (distinct pages " + _holeWritePages.Count + ")  [hole exonerated: 0/0]");
            Console.WriteLine("=== IOP hex-LED / MP-code history (POST + boot progress), latest=" + _io.Led.ToString("X4") + " ===");
            Console.WriteLine("   " + string.Join(" ", _io.LedHistory.Select(x => x.ToString("X4"))));
            Console.WriteLine("=== Diagnostic RS232 UART TX (serial console, " + _io.DiagUartTxRaw.Count + " bytes) ===");
            Console.WriteLine("   text: " + _io.DiagUartTx.ToString());
            Console.WriteLine("   hex : " + string.Join(" ", _io.DiagUartTxRaw.Select(b => b.ToString("X2"))));
            Console.WriteLine("=== CURSOR-SPRITE MP-code snapshots (" + _spriteSnaps.Count + " distinct 16x16 frames; DayBreakMP draws the code here) ===");
            foreach (var snap in _spriteSnaps)
            {
                Console.WriteLine("--- @instr " + snap.Key + "  bytes=" + string.Join("", snap.Value.Select(x => x.ToString("X2"))) + " ---");
                Console.Write(RenderSprite(snap.Value, false));
            }
            Console.WriteLine("=== map-fault gate X-bus values in the loop (XRefBr referenced / XwdDisp dirty) ===");
            foreach (var l in _cp.TrapLog) Console.WriteLine("   " + l);
            Console.WriteLine("=== ONE 0900-LOOP ITERATION at microinstruction level (from CPi " + _cp.LoopTraceFrom + ") ===");
            foreach (var l in _cp.LoopTrace) Console.WriteLine("   " + l);
            Console.WriteLine("=== aLOCKMEM xchg trace: FloppyQueueSemaphore word 0x58000 (first 70 accesses) — xchg must return OLD (0) on the first acquire ===");
            foreach (var l in _cp.SemaLog) Console.WriteLine("   " + l);
            Console.WriteLine("=== IORegion base-pointer STORE watch (FindStartOfIORegion:111 write; 0xC000 right / 0xE200 wrong; empty = never ran) ===");
            foreach (var l in _cp.IORgnWriteLog) Console.WriteLine("  " + l);
            Console.WriteLine("  (total IORegion-candidate stores: " + _cp.IORgnWriteLog.Count + ")");
            Console.WriteLine("=== OPCODE dispatch sequence CPi 1700-1830 (find the vp-push opcode before F8 09) ===");
            foreach (var l in _cp.OpLog) Console.WriteLine("  " + l);
            Console.WriteLine("=== CODE BYTES around the F8 09 (aGMF) — germ code, real page 0x4B1, the vp-push opcodes before F8 09 ===");
            {
                for (int w = 0x4B1E4; w <= 0x4B1F4; w++)
                {
                    ushort v = _cp.ReadWord(w);
                    Console.WriteLine("  word " + w.ToString("X5") + " = " + v.ToString("X4") + "  bytes: " + (v >> 8).ToString("X2") + " " + (v & 0xff).ToString("X2"));
                }
            }
            Console.WriteLine("=== @ESC (F8) alpha-dispatch trace: F8 09 (aGMF, ESC0n[9]) vs F8 89 (aNOTIFYIOP, ESC8n[9]) — where does high-nibble-0 land? ===");
            foreach (var l in _cp.EscLog) Console.WriteLine(l);
            // ProcessorFace.SpecialSetMP is MACHINE CODE [zESC, aWRMP] and posts straight to hardware
            // WITHOUT touching the ProcessorFace.mp global ("Does not set ProcessorFace.mp" --
            // ProcessorFace.mesa:27-34), so @WRMP (MiscDaybreak.mc:113, at[7,10,ESC7n] -> alpha 0x77) is the
            // ONLY place every MP post is visible, in order.  Probing the mp global is structurally blind.
            Console.WriteLine("=== @WRMP (zESC alpha 0x77) -- EVERY maintenance-panel post, ordered (" + _cp.WrmpLog.Count + ") ===");
            foreach (var l in _cp.WrmpLog) Console.WriteLine("   " + l);
            // TechRef Table 2.11 detector.  NOT vectored: microstore 0 is BootTrap (InitDaybreak.mc:19/41)
            // = the boot-button/INIT vector = a full machine re-init.  The FIRST underflow is the wound;
            // everything after it is silent-wrap noise (once one underflow wraps, every sp reading is fiction).
            Console.WriteLine("=== TABLE 2.11 STACK TRAPS (first " + _cp.StkTrapLog.Count + " shown) ===");
            foreach (var l in _cp.StkTrapLog) Console.WriteLine(l);
            Console.WriteLine("=== MICROWORD PATH / LOOP STATE (" + _cp.LoopLog.Count + " lines) ===");
            foreach (var l in _cp.LoopLog) Console.WriteLine(l);
            Console.WriteLine("=== aGMF / FindStartOfIORegion Map<- references near IORegion end (target: reads vp 0x0FF; MAPA=4 -> MAR 0x400FF) ===");
            foreach (var l in _cp.MapReadLog) Console.WriteLine("   " + l);
            Console.WriteLine("=== GetHandlerIORegionPtr math (fcb = base + 8*(ByteSwap[segments[16]] - 0x400); correct = vp 0x0DE) ===");
            {
                int seg16 = _cp.ReadWord(0x52022);                       // IORTable word 2+2*16, CP word 0x52022 = phys 0xA4044
                int seg16bs = ((seg16 & 0xFF) << 8) | ((seg16 >> 8) & 0xFF);
                int vp0FF = _cp.ReadWord(0x400FF);                       // map word for vp 0x0FF (ioRegionAfterEnd-1)
                int realEnd = ((vp0FF & 0x1F) << 8) | (vp0FF >> 8);
                int count = (realEnd % 256) + 1 - 0x20;                  // ioRegionPageOffset=0x20
                int baseVp = 0x100 - count;
                int fcbWord = (baseVp << 8) + 8 * (seg16bs - 0x400);
                Console.WriteLine("  segments[16] (CP word 0x52022) raw=" + seg16.ToString("X4") + "  ByteSwap=" + seg16bs.ToString("X4") + "  (target 0x07C0)");
                Console.WriteLine("  FindStartOfIORegion: map[vp 0x0FF]=" + vp0FF.ToString("X4") + " -> GetState.real=0x" + realEnd.ToString("X3") + " (target 0x55F); count=0x" + count.ToString("X") + " base=vp 0x" + baseVp.ToString("X3") + " (target 0x0C0)");
                Console.WriteLine("  => fcb = word 0x" + fcbWord.ToString("X4") + " = vp 0x" + (fcbWord >> 8).ToString("X3") + " (target 0x0DE / real 0x53E; germ actually hit vp 0x100)");
                // BASE-IS-WRONG cross-check: if FindStartOfIORegion is SKIPPED, germ keeps default base vp 0xE2.
                // With base 0xE2, IORegion[16].ioRegionSegment is read from vp 0xE2's location (real 0x542 = CP 0x54222).
                {
                    int segE2 = _cp.ReadWord(0x54222);
                    int segE2bs = ((segE2 & 0xFF) << 8) | ((segE2 >> 8) & 0xFF);
                    int fcbE2corr = (0xE2 << 8) + 8 * (seg16bs - 0x400);      // base 0xE2 + CORRECT S (0x7C3)
                    int fcbE2own  = (0xE2 << 8) + 8 * (segE2bs - 0x400);      // base 0xE2 + its own read at 0x542
                    Console.WriteLine("  [default base vp 0xE2] word@0x22 read-there (CP 0x54222)=" + segE2.ToString("X4") + " bs=" + segE2bs.ToString("X4"));
                    Console.WriteLine("     base 0xE2 + correct S(0x7C3) => word 0x" + (fcbE2corr & 0xFFFF).ToString("X4") + " = vp 0x" + ((fcbE2corr >> 8) & 0xFF).ToString("X3"));
                    Console.WriteLine("     base 0xE2 + own S(there)    => word 0x" + (fcbE2own & 0xFFFF).ToString("X4") + " = vp 0x" + ((fcbE2own >> 8) & 0xFF).ToString("X3") + "   (germ actually hit vp 0x100 -> whichever matches identifies the read path)");
                }
                Console.Write("  segment-table scan for 0x07C0 / 0x0C00 (slot=word, handlerID=(word-2)/2): ");
                for (int w = 2; w <= 0x30; w += 2)
                {
                    int v = _cp.ReadWord(0x52000 + w); int vbs = ((v & 0xFF) << 8) | ((v >> 8) & 0xFF);
                    if (vbs == 0x07C0 || vbs == 0x0C00 || vbs == 0x07C3)
                        Console.Write("slot" + w.ToString("X") + "(hid" + ((w - 2) / 2) + ")=" + vbs.ToString("X4") + " ");
                }
                Console.WriteLine();
            }
            Console.WriteLine("=== SCAN loaded code memory for ProcessorFace.Start entry = CD FF F8 09 (zLIB 0xFF push-255 + zESC aGMF GetState) ===");
            {
                byte[] R = _mem.SystemRaw;
                int hits = 0;
                for (int b = 0; b < R.Length - 3; b++)
                {
                    if (R[b]==0xCD && R[b+1]==0xFF && R[b+2]==0xF8 && R[b+3]==0x09)
                    {
                        int cpword = b >> 1; int realpage = cpword >> 8; int par = b & 1;
                        Console.WriteLine("  HIT phys byte 0x" + b.ToString("X6") + " = CP word 0x" + cpword.ToString("X5") + " (real pg 0x" + realpage.ToString("X3") + ")  CD byte-parity " + (par==0?"EVEN(hi/pc16=0)":"ODD(lo/pc16=1)"));
                        if (++hits >= 12) { Console.WriteLine("  (>=12 hits, stopping)"); break; }
                    }
                }
                if (hits == 0) Console.WriteLine("  (no CD FF F8 09 in physical DRAM -- check byte order / not-yet-loaded)");
                Console.WriteLine("  [germ XFER landed R5=0x99B6, phys word 0x499B6 = phys byte 0x9336C; loaded word there = 0x05F9, no CD]");
                // Broader: locate every GetState (F8 09 = zESC aGMF) and every DSHIFT (F8 17), print preceding 4 bytes.
                Console.WriteLine("  -- all GetState (F8 09) sites with preceding 4 bytes (FindStartOfIORegion's should be preceded by push-255) --");
                int g = 0;
                for (int b = 4; b < R.Length - 1 && g < 30; b++)
                    if (R[b]==0xF8 && R[b+1]==0x09)
                    { Console.WriteLine("     F8 09 @phys 0x" + b.ToString("X6") + " (CPword 0x"+(b>>1).ToString("X5")+" rpg 0x"+((b>>1)>>8).ToString("X3")+") prec: " + R[b-4].ToString("X2")+" "+R[b-3].ToString("X2")+" "+R[b-2].ToString("X2")+" "+R[b-1].ToString("X2")); g++; }
                if (g==0) Console.WriteLine("     (no F8 09 anywhere -> code byte order is NOT big-endian-contiguous, or germ code absent)");
                // Sanity: is ANY germ code present? scan for the known dispatched run C0 3B 12 CC (from prior traces).
                int s = 0; for (int b=0;b<R.Length-3;b++) if(R[b]==0xC0&&R[b+1]==0x3B&&R[b+2]==0x12&&R[b+3]==0xCC){Console.WriteLine("  sanity: C0 3B 12 CC @phys 0x"+b.ToString("X6")); if(++s>=3)break;}
                if (s==0) Console.WriteLine("  sanity: C0 3B 12 CC NOT found either -> byte order differs; will try swapped");
                // BYTE DUMP for control-flow decode: Start region (page 0x499, germ enters @0x99B6) + FindStart inline (page 0x4AE).
                // Each CP word -> 2 opcode bytes (hi=byte-PC even, lo=byte-PC odd). Opcode stream reads hi,lo,hi,lo...
                Console.WriteLine("  == page 0x499 (Start body / DoCommand; germ's XFER enters at R5=0x99B6) ==");
                for (int w = 0x49998; w <= 0x499BE; w++)
                    Console.WriteLine("    R5=0x" + (w & 0xFFFF).ToString("X4") + ": " + R[w<<1].ToString("X2") + " " + R[(w<<1)+1].ToString("X2") + (w==0x499B6?"   <== germ lands here (dispatches lo=0xF9)":""));
                Console.WriteLine("  == page 0x4AE (FindStartOfIORegion inline; CD FF C0 F8 09 = push LONG 255 + GetState @ R5=0xAE05) ==");
                for (int w = 0x4ADFC; w <= 0x4AE12; w++)
                    Console.WriteLine("    R5=0x" + (w & 0xFFFF).ToString("X4") + ": " + R[w<<1].ToString("X2") + " " + R[(w<<1)+1].ToString("X2") + (w==0x4AE05?"   <== CD FF (push 255 lo word) starts here":""));
                // Entry-vector region: germ read initialpc=0x06F4 from real word 0x4801A during the XFER to Start.
                Console.WriteLine("  == codebase/GFT table (real words 0x48016-0x48020; germ read 0x4801A=0x163C as codebase) ==");
                for (int w = 0x48016; w <= 0x48020; w++)
                    Console.WriteLine("    word 0x" + w.ToString("X5") + " = 0x" + (((R[w<<1]<<8)|R[(w<<1)+1]) & 0xFFFF).ToString("X4") + (w==0x4801A?"   <== codebase 0x163C":""));
                Console.WriteLine("  == entry-vector region (real words 0x48204-0x48218; germ read 0x4820C/D as initialpc=0x06F4) ==");
                for (int w = 0x48204; w <= 0x48218; w++)
                    Console.WriteLine("    word 0x" + w.ToString("X5") + " = 0x" + (((R[w<<1]<<8)|R[(w<<1)+1]) & 0xFFFF).ToString("X4") + (w==0x4820C?"   <== germ read here (0x06F4)":"") + (w==0x4820D?"   <== or here":""));
                Console.WriteLine("  == DRAM scan for true ep-0 initialpc 0x2F92 (target) ==");
                { int hh=0; for (int b=0; b<R.Length-1 && hh<12; b+=2){ if(((R[b]<<8)|R[b+1])==0x2F92){ int cw=b>>1; Console.WriteLine("    0x2F92 @ CPword 0x"+cw.ToString("X5")+" (rpg 0x"+(cw>>8).ToString("X3")+")"); hh++; } } if(hh==0) Console.WriteLine("    (0x2F92 not present as a big-endian word -> entry vector encodes initialpc differently, or Start's true initialpc != 0x2F92)"); }
                Console.WriteLine("  == scan: 0x2F92 byteswapped (0x922F) OR word-PC (0x17C9) anywhere in DRAM ==");
                { int hh=0; for (int b=0; b<R.Length-1 && hh<12; b+=2){ int w=(R[b]<<8)|R[b+1]; if(w==0x922F||w==0x17C9){ int cw=b>>1; Console.WriteLine("    0x"+w.ToString("X4")+" @ CPword 0x"+cw.ToString("X5")+" (rpg 0x"+(cw>>8).ToString("X3")+")"); hh++; } } if(hh==0) Console.WriteLine("    (neither 0x922F nor 0x17C9 present)"); }
                Console.WriteLine("  == scan: ALL [gf=0x0B5D, pc] ControlLink pairs in DRAM (codebase 0x163C: word = 0x163C + pc>>1; Start needs word 0x2E05) ==");
                { int hh=0; for (int b=0; b<R.Length-3 && hh<24; b+=2){ int w0=(R[b]<<8)|R[b+1]; if(w0==0x0B5D){ int w1=(R[b+2]<<8)|R[b+3]; int cw=b>>1; int ew=(0x163C+(w1>>1))&0xFFFF; Console.WriteLine("    [gf=0B5D, pc=0x"+w1.ToString("X4")+"] @ CPword 0x"+cw.ToString("X5")+"  -> entry word 0x"+ew.ToString("X4")+(ew==0x2E05?"  <== START!":"")); hh++; } } if(hh==0) Console.WriteLine("    (no 0x0B5D links found)"); }
            }
            Console.WriteLine("=== GERM VM MAP: which virtual pages map into/near the IORegion [0x520,0x560)?  target: the FCB vp must decode to real 0x53E, entry 0x3EC5 (not 0x80C5=0x580) ===");
            {
                for (int vp = 0x00; vp <= 0x180; vp++)
                {
                    int w = _cp.ReadWord(0x40000 + vp);
                    int rp = ((w & 0x1F) << 8) | (w >> 8);
                    if (rp >= 0x510 && rp < 0x5A0)
                        Console.WriteLine("  vp 0x" + vp.ToString("X3") + " -> real 0x" + rp.ToString("X3") + "  (map word " + w.ToString("X4") + ", off-from-0x520=" + (rp - 0x520) + ")");
                }
                int wf = _cp.ReadWord(0x40100);
                Console.WriteLine("  [FCB access vp 0x100] map word 0x40100 = " + wf.ToString("X4") + " -> real 0x" + (((wf & 0x1F) << 8) | (wf >> 8)).ToString("X3"));
            }
            Console.WriteLine("=== SD TRAP TABLE (real 0x48200+, virtual 0x0200+; germ read SD[0x20E]=null FIRST then escalated to SD[0x20C]=[0B5D,06F4]=GermWorldError) ===");
            {
                byte[] sd = _mem.SystemRaw;
                for (int w = 0x48200; w <= 0x48230; w += 2)
                {
                    int gf = (sd[w*2]<<8)|sd[w*2+1];
                    int pc = (sd[(w+1)*2]<<8)|sd[(w+1)*2+1];
                    int virt = 0x200 + (w - 0x48200);
                    int slot = (w - 0x48200) / 2;
                    Console.WriteLine("  SD[" + slot.ToString("X2") + "] real 0x" + w.ToString("X5") + " (virt 0x" + virt.ToString("X3") + ") = [gf=" + gf.ToString("X4") + ", pc=" + pc.ToString("X4") + "]" + (gf==0&&pc==0?"  <== NULL":(gf==0x0B5D?"  (GermOps handler)":"")) + (w==0x4820E?"  <### intended trap slot (NULL)":"") + (w==0x4820C?"  <### escalation slot (ControlTrap->GermWorldError)":""));
                }
            }
            Console.WriteLine("=== R5-ADVANCE ops: is delta == pc16-in (correct PC+PC16) or always 1 (PC16-as-constant)? ===");
            foreach (var l in _cp.CaptureLog) Console.WriteLine("  " + l);
            Console.WriteLine("=== (old CAPTURES header) aD=2/3: compare RefillE @400 (real MAR<-, should NOT capture) vs legit Map<- captures ===");
            foreach (var l in _cp.CaptureLog) Console.WriteLine("  " + l);
            Console.WriteLine("=== OQ54 MAP-ARRAY READS (GetState[0xFF]/FindStartOfIORegion inputs): what real page does vp 0xFF decode to at the germ's read time? If vp0xFF->0x55F region => input OK (fault is 13-bit round-trip/downstream); if ->0x580 region => upstream map wrong ===");
            foreach (var l in _cp.MapArrRead) Console.WriteLine("  " + l);
            Console.WriteLine("=== OQ59 SD TRAP-TABLE READS (the trap DISPATCH): watch SD[7]/sCodeTrap read first (null?) then escalate to SD[6]/sControlTrap=[0B5D,06F4]=ControlTrap@real0x499B6. R5 before = the faulting pc. ===");
            foreach (var l in _cp.SdReadLog) Console.WriteLine("  " + l);
            Console.WriteLine("=== XFER-CHAIN (OQ52): register-writing MAR<- microwords near the wedge. Columns: cap(splice)=current model, plainF=DLion model, split(hi|Flo)=split-2901. Watch @AAE/@33F (XFER PC-set) and @400 (RefillE). Key Q: is F==Y (splice==plainF) or F!=Y? ===");
            foreach (var l in _cp.XferChainLog) Console.WriteLine("  " + l);
            Console.WriteLine("=== SD-INSTALL WRITES: every store to real 0x48200-0x48230 (SD table). Watch SD[7]@0x4820E (sCodeTrap) vs siblings SD[6]@0x4820C/SD[3]@0x48206 ===");
            foreach (var l in _cp.LinkVecWriteLog) Console.WriteLine("  " + l);
            Console.WriteLine("=== LGC global-frame resolution trace (UvG virtual addr the germ translates for its GF; should be vp 0x0B=gframe 0x0B5C, NOT vp 0x09) ===");
            foreach (var l in _cp.LgcLog) Console.WriteLine("  " + l);
            Console.WriteLine("=== GLOBAL-FRAME MAP (vp 0x00-0x14): GermOpsImpl gframe 0x0B5C -> vp 0x0B; germ reads its GF at real 0x489 (unwritten). map-decode vs GFT-entry? ===");
            {
                byte[] RR = _mem.SystemRaw; int M = RR.Length - 1;
                for (int vp = 0x00; vp <= 0x14; vp++)
                {
                    int w = _cp.ReadWord(0x40000 + vp);
                    int rp = ((w & 0x1F) << 8) | (w >> 8);
                    int rpAlt = ((w & 0x0F) << 8) | (w >> 8); // DLion 12-bit decode, for contrast
                    string tag = (vp == 0x0B) ? "  <== GF vp (gframe 0x0B5C); germ frame reads land at real page 0x489" : "";
                    Console.WriteLine("  vp 0x" + vp.ToString("X2") + " -> real 0x" + rp.ToString("X3") + " (map word " + w.ToString("X4") + "; 12-bit-decode=0x" + rpAlt.ToString("X3") + ")" + tag);
                }
                // Dump the actual frame bytes at real page 0x489 (gframe 0x0B5C offset 0x5C) and where a NON-null frame might be.
                Console.Write("  frame bytes @ real 0x489 offset 0x5C (CP word 0x4895C): ");
                for (int i=0;i<12;i++){ int ba=((0x4895C)*2 + i)&M; Console.Write(RR[ba].ToString("X2")+" "); } Console.WriteLine();
                // scan real pages 0x480-0x4C0 for the first non-zero-ish global frame (a link cluster / gframe self-ptr)
                Console.WriteLine("  -- real pages 0x480-0x494 first-16-bytes (find where germ's global frame data actually IS) --");
                for (int rp=0x480; rp<=0x494; rp++){ int nz=0; for(int i=0;i<512;i++){ if(RR[((rp<<8)*2+i)&M]!=0) nz++; } Console.WriteLine("    real 0x"+rp.ToString("X3")+": "+nz+"/512 nonzero bytes"); }
            }
            Console.WriteLine("=== CODE-PAGE MAP: vp 0x14..0x30 (germ code); is it contiguous, and does vp 0x2E -> real 0x4AE (Start's codebase word 0x2E05 -> real 0x4AE05 = CD FF C0 F8 09)? ===");
            {
                for (int vp = 0x14; vp <= 0x30; vp++)
                {
                    int w = _cp.ReadWord(0x40000 + vp);
                    int rp = ((w & 0x1F) << 8) | (w >> 8);
                    string tag = "";
                    if (vp == 0x19) tag = "  <== germ's XFER target (word 0x19B6 -> real " + ((rp<<8)|0xB6).ToString("X5") + " = 05 F9?)";
                    if (vp == 0x2E) tag = "  <== Start's vp (word 0x2E05 -> real " + ((rp<<8)|0x05).ToString("X5") + " = CD FF C0 F8 09?)";
                    Console.WriteLine("  vp 0x" + vp.ToString("X2") + " -> real 0x" + rp.ToString("X3") + " (map word " + w.ToString("X4") + ")" + tag);
                }
                // Dump the actual bytes at the two candidate real words.
                byte[] RR = _mem.SystemRaw; int M = RR.Length - 1;
                int r19 = (((_cp.ReadWord(0x40019) & 0x1F) << 8) | (_cp.ReadWord(0x40019) >> 8));
                int r2e = (((_cp.ReadWord(0x4002E) & 0x1F) << 8) | (_cp.ReadWord(0x4002E) >> 8));
                Console.Write("  bytes @ vp0x19 word 0x19B6 (real 0x" + ((r19<<8)|0xB6).ToString("X5") + "): ");
                for (int i=0;i<8;i++){ int ba=(((r19<<8)|0xB6)*2 + i)&M; Console.Write(RR[ba].ToString("X2")+" "); } Console.WriteLine();
                Console.Write("  bytes @ vp0x2E word 0x2E04 (real 0x" + ((r2e<<8)|0x04).ToString("X5") + "): ");
                for (int i=0;i<8;i++){ int ba=(((r2e<<8)|0x04)*2 + i)&M; Console.Write(RR[ba].ToString("X2")+" "); } Console.WriteLine();
            }
            Console.WriteLine("=== DUAL-WINDOW: germ fcb.command (phys 0xB0010/11) vs IOP mesaProcessorCommand (handler reads DS:000D, DS=07C3 -> linear 0x7C3D -> IOP map) ===");
            {
                byte[] R = _mem.SystemRaw;
                Console.WriteLine("  germ writes phys 0xB0010=" + R[0xB0010].ToString("X2") + " 0xB0011=" + R[0xB0011].ToString("X2") + "  (CP word 58008)");
                Console.WriteLine("  IOP handler reads linear 0x7C3D -> mapped phys, value = " + _mem.ReadByte(0x7C3D).ToString("X2") + "  (this is what MOV BL,[000D] sees)");
                Console.Write("  IOP FCB view: linear 0x7C30..0x7C40 (mapped): ");
                for (int i = 0x7C30; i <= 0x7C40; i++) Console.Write(_mem.ReadByte(i).ToString("X2") + " ");
                Console.WriteLine();
                Console.Write("  germ FCB storage: phys 0xB0004..0xB0018 (raw): ");
                for (int i = 0xB0004; i <= 0xB0018; i++) Console.Write(R[i].ToString("X2") + " ");
                Console.WriteLine();
                // Where does the germ's fcb.command (phys 0xB0010) sit in IOP-linear terms, and vice versa?
                Console.WriteLine("  IOP map regs 8/9 = " + _mem.GetMapRegister(8).ToString("X2") + "/" + _mem.GetMapRegister(9).ToString("X2")
                    + "  (mapreg8=5 sends IOP linear page0 -> phys 0xA0000+, so linear 0x7C3D -> phys 0x" + (((_mem.GetMapRegister(8)&0x7F)<<17)|0x7C3D).ToString("X5") + ")");
            }
            Console.WriteLine("=== IOP-side FCB command-word access (phys 0xB0010 even=CP-hi/IOP-lo, 0xB0011 odd=CP-lo/IOP-hi); germ set 58008=0301 => 0xB0010=03,0xB0011=01, polls for noCommand ===");
            Console.WriteLine("   total IOP cmd-byte accesses logged (ring, last 120): " + _mem.CmdByteLog.Count);
            foreach (var l in _mem.CmdByteLog) Console.WriteLine("   " + l);
            Console.Write("=== CODE WINDOW real-words 0x499A8..0x499D0 (big-endian words, CP view): ");
            for (int w = 0x499A8; w <= 0x499D0; w++) Console.Write(_cp.ReadWord(w).ToString("X4") + " ");
            Console.WriteLine();
            { System.Text.StringBuilder bs = new System.Text.StringBuilder("=== CODE BYTE STREAM real-bytes from word 0x499A8: ");
              for (int w = 0x499A8; w <= 0x499D0; w++) { ushort ww = _cp.ReadWord(w); bs.Append((ww>>8).ToString("X2")).Append(' ').Append((ww&0xff).ToString("X2")).Append(' '); }
              Console.WriteLine(bs.ToString()); }
            Console.WriteLine("=== GERM MP-code hunt (CP writes value 0x0100-0x0FFF) -- top 20 by write-count ===");
            Console.WriteLine("   (final CPinstr = " + _cp.InstructionCount + ")");
            foreach (var kv in _mpCand.OrderByDescending(k => k.Value.count).Take(8))
                Console.WriteLine("   CPword " + kv.Key.ToString("X5") + " : " + kv.Value.first.ToString("X4") + "->" + kv.Value.last.ToString("X4") + " (" + kv.Value.count + "x, CPi " + kv.Value.firstInstr + ".." + kv.Value.lastInstr + ")");
            Console.WriteLine("   -- ProcessorHeadDove.mp candidates: any value in HEX 0x0900-0x0999 OR DECIMAL 0x0384-0x03E7 (MP 900s):");
            foreach (var kv in _mpCand.Where(k => (k.Value.last >= 0x0900 && k.Value.last <= 0x0999) || (k.Value.last >= 0x0384 && k.Value.last <= 0x03E7)).OrderByDescending(k => k.Value.count).Take(30))
                Console.WriteLine("   CPword " + kv.Key.ToString("X5") + " : " + kv.Value.first.ToString("X4") + "->" + kv.Value.last.ToString("X4") + "(dec " + (int)kv.Value.last + ") (" + kv.Value.count + "x, CPi " + kv.Value.firstInstr + ".." + kv.Value.lastInstr + ")");
            Console.WriteLine("=== block-copy dest range (one boot-wait iteration, CPinstr 7088-7590): count=" + _copyCount
                + " CPword [" + (_copyMin==0x7FFFFFFF?0:_copyMin).ToString("X5") + ".." + (_copyMax<0?0:_copyMax).ToString("X5") + "] span=" + ((_copyMax<0?0:_copyMax)-(_copyMin==0x7FFFFFFF?0:_copyMin)) + " words ===");
            foreach (var l in _copySample) Console.WriteLine("   " + l);
            {
                Console.WriteLine("=== TEST 1: is the frame VIRTUAL (vpage 0x489 -> mapped real page P) or is rhL:L a raw real page? ===");
                int mv = _cp.ReadWord(0x40489);                       // map entry for vpage 0x489
                int rp = ((mv & 0x1F) << 8) | (mv >> 8);              // decode -> real page P
                Console.WriteLine("   map[vpage 0x489] @ real word 0x40489 = " + mv.ToString("X4") + " -> real page P=" + rp.ToString("X3")
                    + (mv==0x0060?"  (VACANT -> frame not backed -> allocation/populate bug)":(mv==0?"  (0000 unwritten)":"  (PRESENT -> frame is virtual->P; direct LLa read of raw 0x489 is the bug)")));
                Console.WriteLine("   if virtual: frame word @ real (P<<8)|0xA1 = " + ((rp<<8)|0xA1).ToString("X5") + " = " + _cp.ReadWord((rp<<8)|0xA1).ToString("X4")
                    + "  (vs direct raw read 0x489A1 = " + _cp.ReadWord(0x489A1).ToString("X4") + ")");
                Console.WriteLine("   nearby map: vp488=" + _cp.ReadWord(0x40488).ToString("X4") + " vp489=" + mv.ToString("X4") + " vp48A=" + _cp.ReadWord(0x4048A).ToString("X4"));
                Console.WriteLine("   === DRAM size + P-decode calibration ===");
                Console.WriteLine("   SystemRaw bytes=0x" + _mem.SystemRaw.Length.ToString("X") + " (=" + (_mem.SystemRaw.Length>>1) + " words, max real page 0x" + (_mem.SystemRaw.Length>>9).ToString("X") + ")");
                // candidate real pages for the frame + their word@0xA1; calibrate byteswap on vp488 (0x908 vs 0x809)
                int[] cand = {0x908, 0x909, 0x90A, 0x809, 0x808, 0x89};
                foreach (int p in cand) Console.WriteLine("   real pg " + p.ToString("X3") + " word@0xA1 (word " + ((p<<8)|0xA1).ToString("X5") + ") = " + _cp.ReadWord((p<<8)|0xA1).ToString("X4") + " ; word@0x9C = " + _cp.ReadWord((p<<8)|0x9C).ToString("X4"));
                // where are the block-copy args actually? scan real pages 0x100..max for the exact tuple 06CE,0B5D,0384,07CE,0B5D within a page
                Console.Write("   arg-tuple scan (06CE 0B5D 0384 07CE): pages holding 0x06CE then 0x0384 nearby: ");
                for (int p=0x100; p < (_mem.SystemRaw.Length>>9); p++){ for(int o=0;o<0x100;o++){ if(_cp.ReadWord((p<<8)|o)==0x06CE){ bool near=false; for(int k=-4;k<=8;k++){ int oo=o+k; if(oo>=0&&oo<0x100&&_cp.ReadWord((p<<8)|oo)==0x0384) near=true;} if(near){ Console.Write(p.ToString("X3")+":"+o.ToString("X2")+" "); } } } }
                Console.WriteLine();
            }
            Console.WriteLine("=== MAP REGION probe: Seg1 [0x400,0x480) vacant=0x0060; Seg2 [0x480,0x500) should ALSO be stamped ===");
            int[] _mapProbes = {0x40010, 0x44000, 0x47000, 0x47F00, 0x48000, 0x48100, 0x48900, 0x489A1, 0x4C000, 0x4FF00};
            foreach (int _w in _mapProbes) Console.WriteLine("   CP word " + _w.ToString("X5") + " (real pg " + (_w>>8).ToString("X3") + ") = " + _cp.ReadWord(_w).ToString("X4"));
            Console.WriteLine("CP R5Log (R5/RH5 landing a code address = XCode-tail signature):");
            if (_cp.R5Log != null) foreach (var l in _cp.R5Log) Console.WriteLine("   " + l);
            Console.WriteLine("CP QLog (Q-register changes; looking for ->B1C6):");
            if (_cp.QLog != null) foreach (var l in _cp.QLog) if (l.Contains("31C6")||l.Contains("B1C6")||l.Contains("163C")||l.Contains("1B8A")||l.Contains("963C")) Console.WriteLine("   " + l);
            Console.WriteLine("CP XferLog @3C0-3D0 (XCode / codebase-add path):");
            if (_cp.XferLog != null) foreach (var l in _cp.XferLog) Console.WriteLine("   " + l);
            Console.WriteLine("CP StackLog (germ builds @BLTL args + @BLTL consumes them):");
            if (_cp.StackLog != null) foreach (var l in _cp.StackLog) Console.WriteLine("   " + l);
            Console.WriteLine("CP R0Log (every TOS(R0) change in the arg-push slice):");
            if (_cp.R0Log != null) foreach (var l in _cp.R0Log) Console.WriteLine("   " + l);
            Console.WriteLine("CP shift ops (RShift1 on 0x3715 is the XFER PC shift):");
            if (_cp.ShiftLog != null) foreach (var l in _cp.ShiftLog) Console.WriteLine("   " + l);
            Console.WriteLine("CP fY candidates (MAPA<- has xBus=4):");
            if (_cp.FyLog != null) foreach (var l in _cp.FyLog) Console.WriteLine("   " + l);
            Console.WriteLine("CP MAR<- top word addresses (phys byte = word<<1):");
            foreach (var kv in _cp.MarAccess.OrderByDescending(k => k.Value).Take(16))
                Console.WriteLine("   word " + kv.Key.ToString("X5") + " (phys " + (kv.Key << 1).ToString("X5") + ") x" + kv.Value);
            Console.WriteLine("CP unimplemented (" + _cp.Unimplemented.Count + " distinct-ish, first 25):");
            for (int i = 0; i < _cp.Unimplemented.Count && i < 25; i++) Console.WriteLine("  " + _cp.Unimplemented[i]);
            Console.WriteLine("CP first executed microinstructions:");
            for (int i = 0; i < _cp.Trace.Count && i < 30; i++) Console.WriteLine("  " + _cp.Trace[i]);
            Console.WriteLine();

            // ---- DECISIVE: map boundary + XFER walk (0x001B -> gfi6 -> GFT[6]) ----
            {
                byte[] r3 = _mem.SystemRaw;
                Func<int,int> mapEnt = vp => { int b = 0x80000 + 2*vp; return (r3[b]<<8)|r3[b+1]; };       // CP big-endian
                Func<int,int> realPg = e => ((e & 0x1F) << 8) | (e >> 8);
                Console.WriteLine("=== XFER WALK / MAP BOUNDARY ===");
                // 128/256 boundary: is vpage 0x8000 (first entry of upper half) a written map entry?
                int e7FFF = mapEnt(0x7FFF), e8000 = mapEnt(0x8000), e8001 = mapEnt(0x8001);
                Console.WriteLine("  boundary: vp7FFF=" + e7FFF.ToString("X4") + " vp8000=" + e8000.ToString("X4") + " vp8001=" + e8001.ToString("X4")
                    + "  => vp8000=0000(unwritten) means map is 128 pages");
                // eePromMemSize: VMMSizeInPages = 32 << (memSize & 3).  Try to show it from the EEPROM image.
                Console.WriteLine("  (VMMSizeInPages: 128=>23-bit=>vp8000 unmapped; 256=>24-bit=>vp8000 mapped)");
                // XFER walk: SD[sBoot] at MDS0 word 0x202 = vpage 2, offset 2.
                int rpVp2 = realPg(mapEnt(2));
                int sdLinkAddr = rpVp2 * 256 + 0x2;                 // word address within real page
                int linkT = (r3[2*sdLinkAddr]<<8)|r3[2*sdLinkAddr+1];
                int linkTT = (r3[2*(sdLinkAddr+1)]<<8)|r3[2*(sdLinkAddr+1)+1];
                Console.WriteLine("  vp2 -> rp " + rpVp2.ToString("X") + " ; SD[sBoot] link T=" + linkT.ToString("X4") + " TT=" + linkTT.ToString("X4") + " (tag " + (linkT & 3) + ")");
                // GFT is at vpage 0x200 (GFTHi=2).  gfi = (T & ~3)/4.  GFT[gfi] = 4 words.
                int gfi = (linkT & ~3) / 4;
                int rpGft = realPg(mapEnt(0x200));
                int gftWordAddr = rpGft * 256 + gfi * 4;
                Console.Write("  GFT@vp200 -> rp " + rpGft.ToString("X") + " ; gfi=" + gfi + " ; GFT[" + gfi + "] 4 words: ");
                for (int i = 0; i < 4; i++) Console.Write(((r3[2*(gftWordAddr+i)]<<8)|r3[2*(gftWordAddr+i)+1]).ToString("X4") + " ");
                Console.WriteLine();
                Console.WriteLine("  (valid germ GFTItem: globalHi & codeHi near 0; large => wrong page / GFT-swap not done)");
                Console.WriteLine("  vp0 -> rp " + realPg(mapEnt(0)).ToString("X") + " ; vp200 -> rp " + rpGft.ToString("X") + " (GFT-swap: vp200 should hold the GFT real page)");
            }

            // ---- Map extent decider: split-half histogram + germ locator ----
            {
                byte[] r2 = _mem.SystemRaw;
                Func<int,int,(int,int,int)> tally = (lo,hi) => {
                    int vac=0,zero=0,other=0;
                    for (int w=lo; w<hi; w++){ int b=0x80000+2*w; int e=(r2[b]<<8)|r2[b+1];
                        if(e==0x0060)vac++; else if(e==0x0000)zero++; else other++; }
                    return (vac,zero,other);
                };
                var a=tally(0,32768); var bH=tally(32768,65536);
                Console.WriteLine("=== MAP EXTENT DECIDER ===");
                Console.WriteLine("  lower half [0x80000-0x8FFFF] vpage 0..7FFF : vacant0060="+a.Item1+" zero0000="+a.Item2+" other(present)="+a.Item3);
                Console.WriteLine("  upper half [0x90000-0x9FFFF] vpage 8000..FFFF: vacant0060="+bH.Item1+" zero0000="+bH.Item2+" other(present)="+bH.Item3);
                Console.WriteLine("  => upper-half vacant>0 means map is 256 pages (0x482 inside it = COLLISION); ~0 means 128 pages (0x482 free)");
                // Locate the germ GFT signature 09 64 09 F8 anywhere in DRAM.
                Console.Write("  GFT sig (09 64 09 F8) at phys: ");
                for (int b=0; b<r2.Length-4; b+=2) if(r2[b]==0x09&&r2[b+1]==0x64&&r2[b+2]==0x09&&r2[b+3]==0xF8) Console.Write(b.ToString("X6")+" ");
                Console.WriteLine();
                // Germ SD link 00 1B 37 15 anywhere.
                Console.Write("  germ SD-link (00 1B 37 15) at phys: ");
                for (int b=0; b<r2.Length-4; b+=2) if(r2[b]==0x00&&r2[b+1]==0x1B&&r2[b+2]==0x37&&r2[b+3]==0x15) Console.Write(b.ToString("X6")+" ");
                Console.WriteLine();
                // Low real pages [256..640) = phys 0x20000..0x50000: non-zero density.
                Console.Write("  low real pages nz density (32KB blocks 0x20000..0x50000): ");
                for (int b0=0x20000; b0<0x50000; b0+=0x8000){int nz=0;for(int i=0;i<0x8000;i+=2)if(r2[b0+i]!=0||r2[b0+i+1]!=0)nz++;Console.Write(b0.ToString("X5")+"="+nz+" ");}
                Console.WriteLine();
            }

            // ---- Map registers + germ locator (pure observation) ----
            {
                byte[] r1 = _mem.SystemRaw;
                Console.Write("=== IOP map registers 8-15 (E018-E01F): ");
                for (int i = 8; i < 16; i++) Console.Write("reg" + i + "=" + _mem.GetMapRegister(i).ToString("X2") + " ");
                Console.WriteLine();
                Console.Write("  phys 90200 (real page 481, germ vp1): ");
                for (int i = 0; i < 16; i++) Console.Write(r1[0x90200 + i].ToString("X2") + " ");
                Console.WriteLine();
                Console.Write("  phys 90400 (real page 482, germ vp2 / SD): ");
                for (int i = 0; i < 16; i++) Console.Write(r1[0x90400 + i].ToString("X2") + " ");
                Console.WriteLine();
                Console.Write("  phys 90404 (SD[sBoot] @ real word 48202) = ");
                Console.WriteLine(((r1[0x90404] << 8) | r1[0x90405]).ToString("X4") + " (CP big-endian)");
                Console.WriteLine("  non-zero byte density per 64KB of the 4MB DRAM:");
                for (int blk = 0; blk < 64; blk++)
                {
                    int nz = 0, b0 = blk << 16;
                    for (int i = 0; i < 0x10000; i += 4) if (r1[b0 + i] != 0) nz++;
                    if (nz > 0) Console.WriteLine("    phys " + b0.ToString("X6") + "-" + (b0 + 0xFFFF).ToString("X6") + "  nz(sampled/16384)=" + nz);
                }
            }

            // ---- VM map array audit (operator answer 11 contract steps 1-5) ----
            // Map array = 65536 entries at real word 0x40000 => phys bytes 0x80000-0x9FFFF.
            // CP reads entries big-endian.  Vacant == 0x0060.  IORegion entry == 0x20C5.
            {
                byte[] r0 = _mem.SystemRaw;
                var hist = new SortedDictionary<int, int>();
                for (int w = 0; w < 65536; w++)
                {
                    int b = 0x80000 + 2 * w;
                    int e = (r0[b] << 8) | r0[b + 1];
                    int c; hist.TryGetValue(e, out c); hist[e] = c + 1;
                }
                Console.WriteLine("=== VM map array audit (phys 0x80000..0x9FFFF, 65536 entries, CP big-endian) ===");
                Console.Write("  value histogram (top 6): ");
                foreach (var kv in hist.OrderByDescending(k => k.Value).Take(6)) Console.Write(kv.Key.ToString("X4") + " x" + kv.Value + "   ");
                Console.WriteLine();
                Func<int, string> ent = vp => { int b = 0x80000 + 2 * vp; int e = (r0[b] << 8) | r0[b + 1]; int rp = ((e & 0x1F) << 8) | (e >> 8); return "vp" + vp.ToString("X") + "=" + e.ToString("X4") + "(rp " + rp.ToString("X") + (((e & 0xE0) == 0x60) ? " VACANT" : "") + ")"; };
                Console.WriteLine("  " + ent(0) + "  " + ent(1) + "  " + ent(2) + "  " + ent(0xC0) + "  " + ent(0xC1) + "  " + ent(0xB1C8));
                Console.WriteLine("  [expect] ResetMap => all 0060 ; MapIORegion vp C0 => 20C5 (rp 520=1312)");
            }

            var cstore = _io.ControlStore;
            Console.WriteLine("Control store: laneWrites=" + cstore.LaneWrites + " (=" + (cstore.LaneWrites / 6) + " words) bank=" + cstore.Bank +
                              " maxAddr[0..3]=" + cstore.MaxAddr[0] + "/" + cstore.MaxAddr[1] + "/" + cstore.MaxAddr[2] + "/" + cstore.MaxAddr[3]);
            Console.WriteLine("  CS[0] word0=" + cstore.GetWord(0, 0).ToString("X12") + " word1=" + cstore.GetWord(0, 1).ToString("X12") +
                              " word2=" + cstore.GetWord(0, 2).ToString("X12") + " word[max]=" + cstore.GetWord(0, cstore.MaxAddr[0]).ToString("X12"));
            Console.WriteLine("  WCS 16-bit word-OUTs (OUT DX,AX into 0x8000-0xDFFF): " + _io.WcsWordWrites + "  (if large, MoonRise loads via WORD writes -> my WriteWord splits them to consecutive csAddrs = scrambled)");
            {   // Search RAM for the Sunlight .db first-record (DW 0000,01ED,87B2,A01F -> LE bytes 00 00 ED 01 B2 87 1F A0). Is the 1F byte intact in RAM?
                var raw = _mem.SystemRaw; int hits = 0;
                for (int i = 0; i < raw.Length - 10 && hits < 6; i++)
                    if (raw[i] == 0xED && raw[i+1] == 0x01 && raw[i+2] == 0xB2 && raw[i+3] == 0x87)
                    { Console.WriteLine("  .db(LE) @0x" + i.ToString("X5") + ": " + string.Join(" ", System.Linq.Enumerable.Range(-2,10).Select(k => raw[i+k].ToString("X2"))) + "  (lane5 byte should be 1F)"); hits++; }
                for (int i = 0; i < raw.Length - 10 && hits < 12; i++)
                    if (raw[i] == 0x01 && raw[i+1] == 0xED && raw[i+2] == 0x87 && raw[i+3] == 0xB2)
                    { Console.WriteLine("  .db(BE) @0x" + i.ToString("X5") + ": " + string.Join(" ", System.Linq.Enumerable.Range(0,8).Select(k => raw[i+k].ToString("X2"))) + "  (lane5 byte should be 1F)"); hits++; }
                if (hits == 0) Console.WriteLine("  .db first-record pattern (01ED/87B2) not found in RAM (already overwritten)");
            }
            Console.WriteLine("  first WCS lane writes (port/lane/cs/val -- shows MoonRise's actual byte stream):");
            foreach (var l in cstore.LaneLog) Console.WriteLine("    " + l);
            // Sanity: decode a few loaded microwords with the real DLion disassembler.
            // A gross byte-lane packing error would throw or produce nonsense.
            for (int a = 1; a <= 8; a++)
            {
                ulong w = cstore.GetWord(0, a);
                if (w == 0) continue;
                try { Console.WriteLine("  disasm CS[0][" + a + "] " + w.ToString("X12") + " : " + new D.CP.Microinstruction(w).Disassemble(-1)); }
                catch (Exception e) { Console.WriteLine("  disasm CS[0][" + a + "] " + w.ToString("X12") + " FAILED: " + e.Message); }
            }
            Console.WriteLine("Last FDC DMA: dest=0x" + _io._lastDmaDest.ToString("X5") + " len=" + _io._lastDmaLen
                + "  DMA0 regs FFC4=" + _pcb.GetRegisterWord(0xC4).ToString("X4") + " FFC6=" + _pcb.GetRegisterWord(0xC6).ToString("X4")
                + " FFC8(count)=" + _pcb.GetRegisterWord(0xC8).ToString("X4") + " FFCA(ctrl)=" + _pcb.GetRegisterWord(0xCA).ToString("X4"));
            if (_io._lastDmaLen > 0) { Console.Write("DMA dest data (first 32B @0x" + _io._lastDmaDest.ToString("X5") + "): ");
                for (int i = 0; i < 32; i++) Console.Write(_mem.ReadByte(_io._lastDmaDest + i).ToString("X2")); Console.WriteLine(); }
            { Console.Write("EEPROM device-type word in SRAM: ");
              for (int a = 0; a < 0x4000; a += 2) { int wv = _mem.ReadByte(a) | (_mem.ReadByte(a+1)<<8);
                  if (wv == 0x4004 || wv == 0x4000) Console.Write("[" + a.ToString("X4") + "]=" + wv.ToString("X4") + " "); }
              Console.WriteLine();
              // Dump the EEPROM cache region around 0x3A88 (word 21 = base+42, base=0x3A5E)
              Console.Write("EEPROM cache @3A5E words0-31: ");
              for (int i = 0; i < 32; i++) { int a = 0x3A5E + 2*i; Console.Write((_mem.ReadByte(a)|(_mem.ReadByte(a+1)<<8)).ToString("X4") + (i==21?"* ":" ")); }
              Console.WriteLine(); }
            Console.Write("HW interrupt vectors delivered: ");
            foreach (var kv in _hwVec.OrderBy(k => k.Key)) Console.Write("0x" + kv.Key.ToString("X2") + "=" + kv.Value + " ");
            Console.WriteLine();
            {
                var cb = _io.Display.GetCursorBuffer();
                Console.Write("Cursor sprite ED00-1F: ");
                foreach (var b in cb) Console.Write(b.ToString("X2"));
                Console.WriteLine();
                Console.WriteLine("Cursor sprite 16x16:");
                for (int r = 0; r < 16; r++)
                {
                    int patt = (cb[r * 2] << 8) | cb[r * 2 + 1];
                    string line = "";
                    for (int c = 0; c < 16; c++) line += ((patt >> (15 - c)) & 1) != 0 ? "#" : ".";
                    Console.WriteLine("  " + line);
                }
            }
            Console.WriteLine("PIC log after first VERTRET (" + _io.PicLog.Count + " events): " + string.Join(" ", _io.PicLog.Take(80)));
            Console.WriteLine();
            Console.WriteLine("Opie SVC (int nn) call counts:");
            foreach (var kv in _svc.OrderBy(k => k.Key))
                Console.WriteLine(String.Format("  int 0x{0:X2} : {1}", kv.Key, kv.Value));
            Console.Write("ISR@03C54 (Timer2/GenericInterruptProcessing) bytes: ");
            for (int i = 0; i < 160; i++) Console.Write(_mem.ReadByte(0x3C54 + i).ToString("X2"));
            Console.WriteLine();
            Console.WriteLine("Op-dispatch table @0x200 (lcall es:[0x200+op*4]):");
            for (int op = 0; op < 8; op++)
            {
                int p = 0x200 + op * 4;
                int ip = _mem.ReadWord(p), cs = _mem.ReadWord(p + 2);
                Console.WriteLine(String.Format("  op {0} -> {1:X4}:{2:X4} ({3:X5})", op, cs, ip, ((cs << 4) + ip) & 0xFFFFF));
            }
            Console.WriteLine("Boot-select wait flag @0x3DB2 = 0x" + _mem.ReadByte(0x3DB2).ToString("X2") + " (waits for 0x6B)");
            Console.WriteLine("es:[0x549]=" + _mem.ReadByte(0x549).ToString("X2") + " es:[0x53c]=" + _mem.ReadWord(0x53C).ToString("X4"));
            Console.WriteLine("IVT handler addresses:");
            foreach (int v in new[] { 0x20, 0x25, 0x26, 0x30, 0x38, 0x3D, 0x64, 0x68, 0x6A, 0x6B, 0x74, 0x77 })
            {
                int ip = _mem.ReadWord(v * 4);
                int cs = _mem.ReadWord(v * 4 + 2);
                Console.WriteLine(String.Format("  int 0x{0:X2} -> {1:X4}:{2:X4} ({3:X5})", v, cs, ip, ((cs << 4) + ip) & 0xFFFFF));
            }
            Console.WriteLine("SVC calling sites (create/boot/wait):");
            foreach (var kv in _svcSites.OrderBy(k => k.Key))
                Console.WriteLine("  int 0x" + kv.Key + " from: " + string.Join(" ", kv.Value.OrderBy(a => a).Select(a => a.ToString("X5"))));

            // --- Display controller state + framebuffer dump ---
            var disp = _io.Display;
            Console.WriteLine();
            Console.WriteLine("Display: VIDEO=" + disp.VideoEnabled + " nonInterlace=" + disp.NonInterlace +
                              " quadwords/line=" + disp.QuadwordsPerLine + " mixFn=" + disp.MixFunction +
                              " bitmapStartWord=0x" + disp.BitmapStartWord.ToString("X") +
                              " cursor@(" + (disp.CursorWord * 16 + disp.CursorBitOffset) + "," + disp.CursorLine + ") disabled=" + disp.CursorDisabled);
            Console.WriteLine("Display register writes (" + disp.RegisterLog.Count + "):");
            Console.WriteLine("  " + string.Join("  ", disp.RegisterLog));

            disp.RenderMono();
            if (disp.Frame != null && disp.Width > 0)
            {
                string fbPath = System.Environment.GetEnvironmentVariable("DOVE_FB") ?? "dove_fb.bin";
                using (var fs = new System.IO.FileStream(fbPath, System.IO.FileMode.Create))
                using (var bw = new System.IO.BinaryWriter(fs))
                {
                    bw.Write(disp.Width);
                    bw.Write(disp.Height);
                    bw.Write(disp.Frame);
                }
                int set = 0; foreach (var b in disp.Frame) if (b != 0) set++;
                Console.WriteLine(String.Format("Framebuffer {0}x{1} dumped to {2}  ({3} of {4} pixels set)",
                    disp.Width, disp.Height, fbPath, set, disp.Frame.Length));
            }

            using (var fs = new System.IO.FileStream(
                System.Environment.GetEnvironmentVariable("DOVE_VRAM") ?? "dove_vram.bin",
                System.IO.FileMode.Create))
            {
                fs.Write(_mem.SystemRaw, 0, 0x40000);   // 256 KB covers the 1152x861 bitmap
            }
            Console.WriteLine("Raw display DRAM (256KB) -> dove_vram.bin");

            Console.Write("Map regs E010-E01F: ");
            for (int i = 0; i < 16; i++) Console.Write(_mem.GetMapRegister(i).ToString("X2") + " ");
            Console.WriteLine();
            {
                int bestStart = -1, bestLen = 0, curStart = -1, curLen = 0;
                for (int a = 0; a < 0x100000; a++)
                {
                    if (_mem.ReadByte(a) == 0xBB) { if (curStart < 0) { curStart = a; curLen = 0; } curLen++; if (curLen > bestLen) { bestLen = curLen; bestStart = curStart; } }
                    else curStart = -1;
                }
                Console.WriteLine("Largest 0xBB (desktopGray) run: start=0x" + bestStart.ToString("X5") + " len=" + bestLen + " (" + (bestLen / 1024) + "KB)");
            }
            Console.WriteLine();
            Console.WriteLine("Top I/O ports by access count:");
            foreach (var kv in _io.PortCounts.OrderByDescending(k => k.Value).Take(25))
            {
                Console.WriteLine(String.Format("  port {0:X4} : {1}", kv.Key, kv.Value));
            }
        }

        static string Hex5(int a) { return a.ToString("X5"); }

        static string Bytes(int phys, int n)
        {
            var sb = new System.Text.StringBuilder();
            for (int i = 0; i < n; i++) sb.Append(_mem.ReadByte(phys + i).ToString("X2")).Append(' ');
            return sb.ToString().Trim();
        }
    }
}
