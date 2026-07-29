using System;
using System.Collections.Generic;
using D.IOP;

namespace D.CP
{
    /// <summary>
    /// The Dove/6085 (Daybreak) Central Processor microengine.
    ///
    /// The Daybreak CP is a descendant of the Dandelion (DLion) CP: same AM2901 ALU,
    /// same 48-bit microword format.  This engine therefore reuses the shared
    /// <see cref="AM2901"/> and <see cref="Microinstruction"/> classes verbatim and
    /// executes microcode out of the <see cref="DoveControlStore"/> the IOP loads.
    ///
    /// It differs from the DLion CP in three ways the roadmap calls out:
    ///  * 8K dual-bank control store (LoadBank, which the DLion leaves unimplemented);
    ///  * memory is the shared IOP DRAM (word access via the ReadWord/WriteWord hooks),
    ///    not a DLion MemoryController;
    ///  * the "peripheral bus" is the IOP: MesaIntRq raises mesaProcessorInterrupt to
    ///    the 80186 (the "CP init microcode finished" signal), and there is no direct
    ///    Display/Disk/Ethernet controller coupling.
    ///
    /// This is a v1 core: it implements the sequencer (X/Y bus, ALU, U/RH/link/stack
    /// registers, NIA + link, the C1..C3 cycle model), physical MAR&lt;-/MDR&lt;-/&lt;-MD,
    /// ExitKern and MesaIntRq.  Special functions that the boot microcode does not need
    /// yet are counted and logged (see <see cref="Unimplemented"/>) rather than guessed,
    /// so the engine can be grown against the real firmware exactly as the IOP was.
    /// </summary>
    public sealed class DoveCentralProcessor
    {
        private readonly DoveControlStore _cs;
        private readonly AM2901 _alu = new AM2901();

        // Register files (sizes mirror the DLion CP).
        private readonly ushort[] _u = new ushort[256];
        private readonly byte[] _rh = new byte[16];

        // Last 256 microstore addresses, always recorded.  A wedge with no I/O is a spin, and the
        // shape of the spin is the whole diagnosis: a handful of distinct addresses repeating is a
        // tight poll (waiting on a device or a cell that never changes), while a long non-repeating
        // trace is real work that merely produced no disk traffic.  Cheap enough to leave on.
        private readonly int[] _uRing = new int[256];
        private long _uRingN;   // long: an int wrapped negative after 41e9 instructions and broke the dump

        /// <summary>
        /// A human-readable snapshot of CP state, for DOVE_CP_STATE.  Includes the Mesa-level
        /// pointers, because naming the spinning module is what makes a wedge actionable when the
        /// code is ViewPoint and we have no source for it: GFI at least says WHICH module, and if
        /// it turns out to be Pilot or Mesa we do have the source.
        ///
        /// L = local frame (RH3:R3), GF = global frame (RH2:R2); the global link at L-2 holds
        /// GFI &lt;&lt; 2, which is the module identity.  All of these are long pointers whose high part
        /// is FIVE bits -- see the MAR splice; a 4-bit mask silently aliases any frame above real
        /// page 4096, which is most of them once demand paging is running.
        /// </summary>
        public string DescribeState()
        {
            var sb = new System.Text.StringBuilder();
            int L = ((_rh[3] & 0x1F) << 16) | _alu.R[3];
            int GF = ((_rh[2] & 0x1F) << 16) | _alu.R[2];
            int gfi = -1, gl = -1;
            if (ReadWord != null && L > 0x100) { gl = ReadWord(L - 2); gfi = gl >> 2; }
            sb.AppendLine("CP state at shutdown");
            sb.AppendLine("  CPi          = " + InstructionCount);
            sb.AppendLine("  microstore   = 0x" + _tpc[_task].ToString("X3") + "   cycle=" + (_cycle & 3) + "  stackP=" + _stackP);
            sb.AppendLine("  macro PC     = RH5:R5 = " + _rh[5].ToString("X2") + ":" + _alu.R[5].ToString("X4")
                          + "  pc16=" + (_pc16 ? 1 : 0)
                          + "  -> word 0x" + ((((_rh[5] & 0x1F) << 16) | _alu.R[5])).ToString("X5"));
            sb.AppendLine("  L (frame)    = 0x" + L.ToString("X5") + "   GF = 0x" + GF.ToString("X5"));
            sb.AppendLine("  globalLink   = " + (gl < 0 ? "??" : "0x" + gl.ToString("X4")) + "   GFI = " + gfi);
            sb.AppendLine("  MAR=0x" + _mar.ToString("X5") + "  MAPA=" + _mapA.ToString("X"));
            sb.AppendLine("  trapCode=" + _trapCode + "  ibPtr=" + (int)_ibPtr + " (" + _ibPtr + ")"
                          + "  ibEmptyPending=" + _ibEmptyPending
                          + "  IbEmptyTraps=" + IbEmptyTraps
                          + "  InitTraps=" + InitTrapCount + "  stackTraps=" + _stkTraps);
            sb.Append("  R  =");
            for (int i = 0; i < 16; i++) sb.Append(" " + _alu.R[i].ToString("X4"));
            sb.AppendLine();
            sb.Append("  RH =");
            for (int i = 0; i < 16; i++) sb.Append(" " + _rh[i].ToString("X2"));
            sb.AppendLine();

            // The spin itself.
            var seen = new System.Collections.Generic.List<int>();
            var order = new System.Collections.Generic.List<int>();
            int n = _uRingN < 256 ? (int)_uRingN : 256;
            for (int i = 0; i < n; i++)
            {
                int a = _uRing[(int)((_uRingN - n + i) & 0xFF)];
                order.Add(a);
                if (!seen.Contains(a)) seen.Add(a);
            }
            // Disassemble the microwords on the path into address 0.  The predecessor is the
            // whole question when trapCode is 0: nothing raised an error, so something
            // BRANCHED to ErrTrap, and this names the instruction that did it.
            if (NiaWatchLog != null && NiaWatchLog.Count > 0)
            {
                sb.AppendLine("  NIA watch (" + NiaWatchLog.Count + " hits, last 14):");
                int f = NiaWatchLog.Count > 14 ? NiaWatchLog.Count - 14 : 0;
                for (int i = f; i < NiaWatchLog.Count; i++) sb.AppendLine("    " + NiaWatchLog[i]);
            }
            for (int bk = 0; bk < 2; bk++)
            {
                sb.Append("  bank" + bk + " words 000-00F:");
                for (int w = 0; w < 16; w++)
                {
                    ulong wd = _cs.GetWord(bk, w);
                    sb.Append(wd == 0 ? " ----" : " " + (wd & 0xFFFF).ToString("X4"));
                }
                sb.AppendLine();
            }
            sb.Append("  control-store occupancy:");
            for (int bk = 0; bk < DoveControlStore.NumBanks; bk++)
            {
                int nz = 0;
                for (int w = 0; w < DoveControlStore.WordsPerBank; w++)
                    if (_cs.GetWord(bk, w) != 0) nz++;
                sb.Append("  bank" + bk + "=" + nz);
            }
            sb.AppendLine();
            sb.AppendLine("  path microwords:");
            foreach (int ua in new int[] { 0x18F, 0xC04, 0xBDA, 0x5F8, 0x268, 0xD7C, 0x500, 0x003, 0x000 })
            {
                string d;
                try { d = "word=" + _cs.GetWord(_execBank, ua).ToString("X12") + "  " + Fetch(_execBank, ua).Disassemble(-1); }
                catch (Exception e) { d = "<" + e.GetType().Name + ">"; }
                sb.AppendLine("    " + ua.ToString("X3") + ": " + d);
            }
            if (ErrTrapLog != null && ErrTrapLog.Count > 0)
            {
                sb.AppendLine("  ErrTrap entries (" + ErrTrapLog.Count + "):");
                int from = ErrTrapLog.Count > 12 ? ErrTrapLog.Count - 12 : 0;
                for (int i = from; i < ErrTrapLog.Count; i++) sb.AppendLine("    " + ErrTrapLog[i]);
            }
            else sb.AppendLine("  ErrTrap entries: NONE (never entered microstore 0)");
            sb.AppendLine("  last " + n + " microstore addresses: " + seen.Count + " distinct");
            sb.Append("    distinct:");
            foreach (int a in seen) sb.Append(" " + a.ToString("X3"));
            sb.AppendLine();
            sb.Append("    sequence:");
            foreach (int a in order) sb.Append(" " + (a >> 12) + ":" + (a & 0xFFF).ToString("X3"));
            sb.AppendLine();
            return sb.ToString();
        }
        private readonly int[] _link = new int[8];
        private readonly int[] _tpc = new int[8];
        private int _stackP;

        // Sequencer state.
        private int _task;              // Daybreak has one instruction stream (index 0)
        private int _cycle = 1;         // micro-cycle C1..C3
        private int _execBank;          // control-store bank the CP fetches from
        private int _bankTarget;        // pending Bank<- target (one-hot-decoded index)
        private int _bankChangePending; // delay-slot counter for Bank<-
        private ushort _xBus, _yBus, _lastYBus;
        private int _niaModifier;
        private int _xDispNow;          // XDisp bits for the CURRENT microword (see the XDisp case)
        private int _xDispSame = -1;
        private bool XDispSameWord
        {
            get
            {
                if (_xDispSame < 0)
                    // DEFAULT OFF.  Making XDisp same-word broke the boot at MP 0200 with heavy
                    // display corruption -- XDisp is used throughout (stack-depth dispatches in
                    // BBInit/TextBlt/RESSupport, the ib paths), so changing when its bits land
                    // moves every one of those targets.  The 18F measurement stands (mod=000
                    // with xBus=2004, nia=000 into ErrTrap) but the remedy does not: opt in with
                    // DOVE_XDISP_SAME to reproduce the experiment.
                    _xDispSame = (Environment.GetEnvironmentVariable("DOVE_XDISP_SAME") != null) ? 1 : 0;
                return _xDispSame == 1;
            }
        }
        private bool _altUAddr;
        private bool _pc16;
        private bool _lastRefWasMap;   // OQ52: was the last c1 memory ref a Map<- (translate)?  <-MD then returns the
                                       // DECODED REAL PAGE (map's translation output), not a memory word (TechRef §2.5.3.2).
        private bool _mInt;             // IOP->CP doorbell (rInt bit 14).  A MesaIntBr source.
        private bool _timerInt;         // CP 8254 counter0 output (rInt bit 15 = the ~50ms Pilot
                                        // scheduler heartbeat).  Also a MesaIntBr source.
        // MesaIntBr (InterruptsDaybreak.mc:68) fires on ANY of rInt bit13 MesaInt / bit14 IOP /
        // bit15 timer -- not just the IOP doorbell.  counter0 is a mode-2 rate generator; model it
        // as a periodic edge every _timerPeriod CP instructions.
        // ---- 8254 PIT on the Mesa bus (what aRRIT reads) ----------------------------------
        // Channels 1+2 form ONE 32-bit down-counter: timer = (T2<<16)|T1.  Ports are the FULL
        // 8-bit RH[rB] on an fYNorm 0xC "IO<-" reference (TechRef fY table "IO-", drives the MDC
        // IORef line) -- IO refs bypass the MAR splice entirely, so the port never nibble-collapses.
        //   0x41 = T1Count (low 16)   0x42 = T2Count (high 16)   0x43 = T012Control
        // Writing 0xDC (T12Latch) to 0x43 snapshots BOTH halves simultaneously; reads are
        // LSB-then-MSB per channel.  Values are returned RAW (still counting down) -- the germ does
        // its own down-count->elapsed conversion (MiscDaybreak.mc:156,162).
        // Rate: counter-0 reload 0x0C35 (3125) = 50 ms => 62.5 kHz => 16 us/count; against the
        // existing TimerPeriod (40000 CP instr ~ 50 ms) that is ~12.8 CP instructions per count.
        // ON by default: without the readable 8254 count-down aRRIT reads zeros, the germ's
        // zJNZB timing gate fires on a phantom, and the boot dies in the zESC a00 guard (MP 0935).
        public bool Io8254Enabled = true;
        private uint _pit32 = 0xFFFFFFFF;
        private int _pitAccum;
        private ushort _pitLatch1, _pitLatch2;
        private bool _pitLatched, _pitMsb1, _pitMsb2;
        private bool _ioRefPending; private int _ioPort;
        public long IoRefCount, PitReadCount;
        private long _timerCounter;
        private ushort Pit8254Read()
        {
            PitReadCount++;
            switch (_ioPort)
            {
                case 0x41:
                {
                    ushort v = _pitLatched ? _pitLatch1 : (ushort)(_pit32 & 0xFFFF);
                    ushort b = _pitMsb1 ? (ushort)((v >> 8) & 0xFF) : (ushort)(v & 0xFF);
                    if (_pitMsb1) _pitLatched = false;   // T1 MSB is the last byte of the protocol
                    _pitMsb1 = !_pitMsb1;
                    return b;
                }
                case 0x42:
                {
                    ushort v = _pitLatched ? _pitLatch2 : (ushort)((_pit32 >> 16) & 0xFFFF);
                    ushort b = _pitMsb2 ? (ushort)((v >> 8) & 0xFF) : (ushort)(v & 0xFF);
                    _pitMsb2 = !_pitMsb2;
                    return b;
                }
                default: return 0;
            }
        }
        private void Pit8254Write(ushort v)
        {
            if (_ioPort == 0x43 && (v & 0xFF) == 0xDC)
            {
                _pitLatch1 = (ushort)(_pit32 & 0xFFFF);
                _pitLatch2 = (ushort)((_pit32 >> 16) & 0xFFFF);
                _pitLatched = true; _pitMsb1 = false; _pitMsb2 = false;
            }
        }
        public int TimerPeriod = 40000;
        public long TimerFireCount;     // diagnostics: timer edges raised
        public long IntStatReads;       // diagnostics: <-IntStat reads (timer acks)
        public long MesaIntBrFires;     // diagnostics: MesaIntBr taken (interrupt seen)
        private bool _ie;               // interrupt enable (SetIE fY0xF / ClrIE fY0xE); germ enables via @EI
        public long SetIeCount;
        public bool IE { get { return _ie; } }
        public ushort[] URegs { get { return _u; } }
        public ushort[] USnapshot;      // U registers captured for the harness U-register diff
        private int _trapCode;          // <-IntStat X[8-9]: 1=InitTrap, 2=stack, 3=IB-empty
        // Raised by an ib read on an empty buffer, vectored at the end of the click (see the
        // IB-Empty trap block in the NIA computation).  Env-gated so the old behaviour --
        // detect and log, never trap -- is one variable away if this regresses something.
        private bool _ibEmptyPending;
        public long IbEmptyTraps;
        public System.Collections.Generic.List<string> NiaWatchLog =
            new System.Collections.Generic.List<string>();
        private int _niaWatch = -2;
        public System.Collections.Generic.List<string> ErrTrapLog =
            new System.Collections.Generic.List<string>();
        private int _ibEmptyEnabled = -1;
        private bool IbEmptyTrapEnabled
        {
            get
            {
                if (_ibEmptyEnabled < 0)
                    _ibEmptyEnabled = (Environment.GetEnvironmentVariable("DOVE_NO_IBEMPTY_TRAP") == null) ? 1 : 0;
                return _ibEmptyEnabled == 1;
            }
        }
        private void NoteIbEmptyRead()
        {
            if (_ibPtr == IBState.Empty && IbEmptyTrapEnabled) _ibEmptyPending = true;
        }
        // Map-array base: MAPA<- sets it (from the X bus).  MAPA<-4 => real word 0x40000
        // (CPKernel.mapOffset) is the only value the boot path uses.
        private int _mapA = 4;
        private byte _ibFront;          // instruction-buffer front byte (empty at boot)

        // Mesa instruction buffer (IB): the CP fetches 16-bit code words into _ib and
        // dispatches on the front opcode byte.  Ported from the DLion CentralProcessor;
        // the mesa-core fY/fZ codes (fY3=IBDisp, fY6=LoadIB, fZ1=IBPtr<-1, fZ2=IBPtr<-0)
        // are UNCHANGED on Daybreak (only the IOP/housekeeping fY codes were reassigned).
        // IB read-pointer state (values match the DLion IBState so the transition table
        // and <-ErrnIBnStkp encoding carry over verbatim).  Private/nested to avoid
        // colliding with D.CP.IBState in the full-app build.
        private enum IBState { Empty = 0, Byte = 1, Full = 2, Word = 3 }
        private readonly byte[] _ib = new byte[2];
        // MEASUREMENT (operator step 2): track the CP-word address each IB byte came from, so we can
        // compare R5 (the fetch pointer) against the IB-FRONT word at each dispatch.  Uniform offset =>
        // R5 is consistently one word ahead => word[R5] everywhere is correct; non-uniform => R5's
        // advance (pc16/dispatch-exit) is the real bug and no fetch patch holds.  Diagnostic-only.
        private int _ibFrontWord;   // word offset (R5-format, = mar & 0xFFFF) the front byte was loaded from
        private int _ibWord;        // word offset the _ib[] pair was loaded from
        private IBState _ibPtr = IBState.Empty;
        private int _niaModType;        // 0=Normal (OR), 1=IBDispatch (replace [4-7], OR [8-11]), 2=IBRefillTrap (replace [0-3])
        private static readonly IBState[] _nextIBPtr = { IBState.Empty, IBState.Empty, IBState.Word, IBState.Byte };

        // Memory latches.
        private int _mar;
        private ushort _marLowSplice;   // the spliced MAR low-16 (for address-capture register writes)
        private bool _refillEPending;   // set by RefillE (@400); consumed by the immediately-following RefillNE (@500)
                                        // so the two-word empty-buffer refill fetches CONSECUTIVE words (see @500 below).

        // Control signals driven by the IOP.
        private bool _reset = true;     // powers up held in reset (resetMesaProcessor)
        private bool _run;              // WriteCSReg run bit

        // Decode cache, one Microinstruction[] per control-store bank.
        private readonly Microinstruction[][] _cache;

        public DoveCentralProcessor(DoveControlStore controlStore)
        {
            _cs = controlStore;
            _cache = new Microinstruction[DoveControlStore.NumBanks][];
            for (int i = 0; i < DoveControlStore.NumBanks; i++)
                _cache[i] = new Microinstruction[DoveControlStore.WordsPerBank];
        }

        // ---- Memory interface (shared IOP DRAM) ----
        /// <summary>Read a 16-bit word at a CP physical word address (CP big-endian view).</summary>
        public Func<int, ushort> ReadWord;
        /// <summary>Write a 16-bit word at a CP physical word address.</summary>
        public Action<int, ushort> WriteWord;

        // ---- CP -> IOP interrupt (mesaProcessorInterrupt) ----
        /// <summary>Set by SetMPIntIOP (fY=1); the IOP acks it via IN 0xB0.</summary>
        public bool MesaInterruptRequest;
        public Action OnMesaInterrupt;       // SetMPIntIOP (fY=1): assert the latch (edge)
        public Action OnClearMesaInterrupt;  // ClrMPIntIOP (fY=0): drop the latch

        /// <summary>IOP acknowledges the CP-&gt;IOP interrupt (IN 0xB0 = Clear Mesa Int Latch).</summary>
        public void ClearMesaInterrupt() { MesaInterruptRequest = false; }

        // ---- Diagnostics ----
        public long InstructionCount;
        public readonly Dictionary<int, long> MarAccess = new Dictionary<int, long>();  // physical word MAR<- addrs
        public List<string> RhLog;   // if set, logs the first LoadRH operations
        public List<string> FyLog;   // if set, logs unidentified fYNorm codes (to pin MAPA<-)
        public List<string> ShiftLog; // if set, logs shift ops (to pin the RShift1 MSB source)
        public List<string> XferLog;  // if set, logs instrs in [0x3C0,0x3D0] (the XCode codebase-add)
        public List<string> QLog;     // if set, logs every Q-register change
        public List<string> R5Log;    // if set, logs R5(PC)/RH5(rhPC) landing a code-address value
        public List<string> WriteLog; // if set, rolling log of MDR<- stores (frozen at the @666 wait)
        public List<string> IORgnWriteLog; // TEMP: watch stores of the IORegion base pointer (0xC000 right / 0xE200 wrong)
        public List<string> ReadLog;  // if set, rolling log of <-MD reads of the IORegion (word >= 0x50000)
        public List<string> OpLog;    // if set, logs each dispatched mesa opcode (IBDisp ibFront)
        public int OpLogFrom = 1700, OpLogTo = 1830;  // CPi window for OpLog
        public List<string> LinkLog;  // TEMP: L2/link-register dump at XFER-entry LoadIBs (xcE/xcO parity diag)
        public List<string> IbLog; public int IbLogFrom = 0x7FFFFFFF, IbLogTo = 0;  // TEMP: IB _ib[0]/refill trace
        public List<string> LgcLog;  // TEMP: LGC global-frame resolution (UvG virtual addr -> real page); frame lands vp0x09/real0x489 wrong (should vp0x0B/real0x48B)
        public List<string> LinkVecWriteLog;  // TEMP: writes to link-vector candidate pages real 0x489/0x48A/0x48B + BLTL dest 0x580 (find where the germ builds its EFC link vector)
        public List<string> CaptureLog;        // TEMP: every aD=2/3 address-capture (find the field that separates RefillE @400 real MAR<- from legit Map<- captures)
        public List<string> XferChainLog;      // OQ52: register-writing MAR<- microwords near the wedge (Rold/Y/F/splice + candidate write-back models)
        public long XferTraceFrom = 0;         // start capturing XferChainLog after this CPi
        public List<string> MapArrRead;        // OQ54: every CP-side <-MD of the map array 0x400C0-0x40100 (vp 0xC0..0x100 + vp 0xFF) = GetState[0xFF]/FindStartOfIORegion inputs
        public List<string> SdReadLog;         // OQ59: every <-MD of the SD trap table (real 0x48200-0x48220) = trap dispatch reading SD[n] handler [gf,pc]; catch the CodeTrap(SD7)->ControlTrap(SD6) escalation
        // MP-940 stall: which real address the dominant busy-spin (Mesa PC 0x898F/0x99D7) READS,
        // and the last value seen -- pins the cell Pilot's Store is polling forever.
        public System.Collections.Generic.Dictionary<int, long> PollAddrHist;
        public System.Collections.Generic.Dictionary<int, int> PollValByAddr;  // last _xBus VALUE read at each poll addr (the actual data, not a SystemRaw mis-index)
        public int PollLastVal, PollLastAddr;
        public List<string> StackLog; // if set, logs stack push/pop with stackP + value (to trace @BLTL arg build)
        public List<string> R0Log;    // if set, logs every R0(TOS) change (to trace TOS<-value / TOS<-STK writes)
        public List<string> TrapLog;   // TEMP: map-fault gate (XRefBr/XwdDisp) X-bus values in the loop
        public List<string> SemaLog;   // TEMP: every access to the FloppyQueueSemaphore word (aLOCKMEM xchg trace)
        public int SemaWord = 0x58000; // word to trace for SemaLog
        public List<string> LoopTrace; // TEMP: one full 0900-loop iteration at microinstruction level
        public long LoopTraceFrom = 10430575;
        public List<string> XWtLog;    // TEMP: microword-field decode + XWtOKDisp(fY=0xB) branch at the net-0 zSGB faulting store (CPi 1300-1400)
        public List<string> MapReadLog; // TEMP: Map<- references near the IORegion end (aGMF / FindStartOfIORegion)
        public List<string> EscLog;    // TEMP: @ESC (F8) alpha-dispatch trace (aGMF F8 09 vs aNOTIFYIOP F8 89)
        /// <summary>
        /// Every @WRMP (zESC alpha 0x77) -- THE maintenance-panel post chokepoint.  Enabled by
        /// DOVE_MP_LOG=&lt;path&gt;, written out on close.
        ///
        /// This is the authoritative answer to "which MP code are we at".  On Daybreak the code is
        /// rendered as the mouse cursor sprite, so the only other way to read it is to look at the
        /// screen and decode a bitmap by eye -- fine for 0900 vs 0935, but the difference between
        /// 0934 and 0935 is one glyph, and an entire line of enquiry can hang off which one it is.
        /// The log also gives the ORDER and the count, which a display cannot: it shows whether a
        /// code was the first failure or overwrote an earlier one, and whether it was posted once
        /// or is being re-posted in a loop.
        /// </summary>
        public List<string> WrmpLog;
        public List<string> KfcbLog;   // error-raise capture: every zKFCB (0xF0) in [KfcbLogFrom,KfcbLogTo] -- logs
        public long KfcbLogFrom;       // the signal descriptor + arg on the stack so the INNER (root) raise is readable
        public long KfcbLogTo = long.MaxValue;
        // Dispatch ring: raw (op + pending IB pair) at EVERY dispatch, last 64 kept (auto-overwrite).
        // Dumped at each KFCB so the raise-site byte stream `9a XX fd 00 YY f0 23` is reconstructable
        // regardless of the operand straddling the IB word boundary (int arrays = no per-dispatch alloc).
        public int[] DispRingCPi, DispRingOp, DispRingIb, DispRingPtr, DispRingR5, DispRingRH5;
        public int DispRingPos;
        public int RetLogFrom = 15300, RetLogTo = 15340;   // TEMP: zRET (0xEF) trace window
        // Live <-MD read trace across the SignalHandler walk: the ONLY trustworthy A/B decider.
        // Logs every memory read (mar>=0x40000) in [WalkRdFrom,WalkRdTo] so we see the real frame-chain
        // addresses SignalHandler actually reads (the static handle->real follow is stale/incomplete).
        public List<string> WalkRdLog; public long WalkRdFrom = long.MaxValue, WalkRdTo = 0; public int WalkRdGfi = 109;
        // StashPC watch: capture every WriteWord that stores the buggy saved-PC 0x5CA3 or targets
        // GFI 99's frame [F-1]=0x788EB.  With R5/pc16/Q at the write we see whether R5 already lacked
        // bit 7 (upstream page-cross corruption) or the stash itself dropped it -- the operator's #3.
        public List<string> StashWatchLog; public int StashWatchAddr = -1, StashWatchVal = -1;
        public long StashWatchFrom = 0, StashWatchTo = long.MaxValue;
        // Process save/restore stkptr round-trip watch: <-ErrnIBnStkp (SAVE side, returns ~stackP) and
        // every stackP<- (RESTORE side).  Answers: does resume-stkptr == save-stkptr, or save+1?
        public List<string> ProcSwLog; public long ProcSwFrom = long.MaxValue, ProcSwTo = 0;
        // Per-MICROWORD sp trace: every microword that moves _stackP, with the mechanism and the
        // decoded Daybreak stack fields -- to find the exact microword where the eval stack gains a slot.
        public List<string> SpTraceLog; public long SpTraceFrom = long.MaxValue, SpTraceTo = 0;
        // IE provenance: every ClrIE/SetIE with its microword + macro PC, to see what last set IE
        // before the aRRIT-polled loop (and whether a ClrIE the germ issued was honoured).
        public List<string> IeLog; public long IeFrom = long.MaxValue, IeTo = 0;
        // Raw-microword dump for named addresses: prints the 48-bit word + every decoded field, so the
        // U-address formation can be diffed against the TechRef encoding (uStkDepth must resolve to 0x33).
        public List<string> UwDumpLog; public int UwDumpA = -1, UwDumpB = -1, UwDumpC = -1;
        // U-address census: every Uaddr-mode U access, keyed by the address my decoder forms.
        // Validates rA/fZ bit positions + nibble order against the source RegDef table.
        public long[] UAddrHist; public long UAddrFrom = long.MaxValue, UAddrTo = 0;
        // Module code-region census: for every code read, attribute it to the CURRENT frame's GFI
        // ([L-2]>>2) and record min/max code address + samples.  This bounds each module's REAL code
        // extent from live execution -- the only trustworthy bound (GFT "next codebase" is not: GFI 99's
        // verified live code lies past the next codebase, so entries are stale or layout is non-contiguous).
        public long[] ModCensCount; public int[] ModCensMin, ModCensMax; public string[] ModCensSample;
        public long ModCensFrom = long.MaxValue, ModCensTo = 0;
        // Opcode-length census: OpLenHist[op, n] = times opcode `op` consumed n IB bytes (n=0 means the next
        // dispatch was non-sequential, i.e. this op branched/transferred).
        public long[,] OpLenHist; public long OpLenFrom = long.MaxValue, OpLenTo = 0;
        private int _opLenPrevOp = -1, _opLenPrevPos = 0, _opLenPrevRH = -1;
        // Exact dispatch trace: raw (op, R5, RH5, pc16) per dispatch.  Offline the opcode BYTE POSITION is
        // recovered by matching the op value against the real code bytes, giving EXACT instruction lengths
        // (the R5-derived census is off: zESC is provably 2 bytes yet measures 3B most of the time).
        public int[] DispTrOp, DispTrR5, DispTrRH5, DispTrPc16, DispTrTOS, DispTrSp, DispTrU1, DispTrU2, DispTrInt, DispTrMA, DispTrCPi; public int DispTrN;
        public long DispTrFrom = long.MaxValue, DispTrTo = 0;
        public int[] AddrHist;          // TEMP: microword-address histogram -- names microcode spins
        public List<string> LoopLog;    // TEMP: full-state dump across microcode-spin iterations
        public int LoopAddr = -1, LoopFrom = int.MaxValue;
        public int RingFrom = int.MaxValue;   // TEMP: arm the entry-path ring buffer
        public List<string> StkTrapLog;       // TechRef Table 2.11 stack over/underflow detector
        // THE MAP-SPIN PROBE.  The live wound is a 13-microword map set-ref fix-up that never
        // converges (@49C Q+1 / @062 Map<- / @140 byte(80) or R6), 247,891 iterations = 99.9% of
        // the run.  Inside it the map word reads back 0x0000 every pass.  0x0000 is NOT "vacant"
        // (vacant == 0x60 in the Daybreak map format) -- all-zeros means NEVER WRITTEN.  So either
        // Q+1 has walked past the end of the initialized map array, or we are addressing the wrong
        // array.  The map address is MAPA-relative, not a constant: mar = (_mapA<<16) | vpage,
        // and boot does MAPA<-4 => base 0x40000 (InitDaybreak.mc:154); the DLion reference
        // hardcodes 0x10000.  Dump the computed address, MAPA, and Q's trajectory to tell
        // "walked past the end" (base sane, vpage climbing) from "wrong base" (MAPA stale/unset).
        public List<string> SpinMapLog;
        public long SpinMapFrom = long.MaxValue;
        // Count microwords carrying LoadMap=1 by the cycle they actually execute in.
        // Map<- is c1-ONLY in the microcode: 19 of 19 `Map <- ...` in dove_build_kit/daybreak_ucode/uc
        // are annotated ,c1; ZERO at ,c2 or ,c3.  So the `case 1:` gate below is right, and any
        // LoadMap microword reaching c2/c3 is silently dropped -- its map reference never happens.
        // If that count is large, the germ is executing Map<- at the wrong click phase (or running
        // code it should never have reached), and no map fix-up downstream of it can ever converge.
        public readonly long[] LoadMapByCycle = new long[4];

        /// <summary>
        /// DOVE_MAP_WATCH=&lt;vpage&gt;[,&lt;vpage&gt;...] -- log every write to those virtual pages' map
        /// entries, as it happens, with the CP instruction count.
        ///
        /// WHY: a post-mortem DRAM dump only shows the map as it stood when the machine stopped.
        /// That is sound while nothing remaps -- and it was, for the germ and for a fully
        /// boot-loaded image like the Utility Pilot (lastVMPage = lastBootLoadedPage, no faults).
        /// It stops being sound the moment demand paging runs: "virtual V resolves correctly now"
        /// no longer establishes "virtual V resolved correctly when the guest read it".  BWSDove
        /// boot-loads 426 pages of a 2803-page file, so the remaining 2377 arrive through the fault
        /// path, and this is the first image on this emulator that exercises it.
        ///
        /// The map array is real words [mapBase, mapBase+0x10000) where mapBase = (MAPA &amp; 0xF) &lt;&lt; 16;
        /// the entry for virtual page V is at mapBase+V, decoding as
        /// realPage = ((w &amp; 0x1F) &lt;&lt; 8) | (w &gt;&gt; 8) with flags in w &amp; 0xE0.
        /// </summary>
        private HashSet<int> _mapWatch;
        private int _mapWatchLeft = -1;
        public List<string> MapWatchLog;

        /// <summary>
        /// The companion to MapWatch: every Map&lt;- REFERENCE to a watched virtual page, logged with
        /// the entry the microcode is about to read back at c3.  Writes alone cannot answer the
        /// question that matters -- "what did the guest see when it dereferenced this page?" --
        /// because between two writes the entry is read an unbounded number of times, and on
        /// Daybreak the translation is done by the microcode itself (MAR&lt;- never translates), so
        /// the value returned here IS the address the guest ends up reading.
        /// </summary>
        /// <summary>
        /// After a Map&lt;- of a watched virtual page, log the next few &lt;-MD reads with their address
        /// and VALUE.  This is the last unmeasured link in a translated access: the map entry tells
        /// you which real page the microcode will splice, and the DRAM dump tells you what that page
        /// held when the machine stopped, but neither tells you the word the guest actually got.
        ///
        /// Reads come in a fixed order: the Map&lt;- reference itself reads the map ENTRY at c3, then
        /// the microcode splices the real page and issues a MAR&lt;- whose c3 reads the DATA.  So the
        /// entry read and the data read both appear here; identify the data read by its address
        /// (realPage &lt;&lt; 8 | offset) rather than by position, since the microcode may interleave.
        /// </summary>
        private int _mapReadArm;
        private void MapWatchFollowRead()
        {
            if (_mapReadArm <= 0 || MapWatchLog == null || MapWatchLog.Count >= 12000) return;
            _mapReadArm--;
            MapWatchLog.Add("  RD  addr=" + _mar.ToString("X5")
                + " (realPage 0x" + (_mar >> 8).ToString("X4") + " word " + (_mar & 0xFF) + ")"
                + "  value=" + _xBus.ToString("X4")
                + (_xBus == 6303 ? "  == 6303 StartList.VersionID" : "")
                + "  CPi=" + InstructionCount);
        }

        private void MapWatchRead(int vpage, int addr)
        {
            if (_mapWatchLeft <= 0 || _mapWatch == null || !_mapWatch.Contains(vpage)) return;
            if (MapWatchLog.Count >= 12000) return;
            _mapReadArm = 6;          // entry read + the spliced data read(s) that follow
            int w = ReadWord != null ? ReadWord(_mar) : -1;
            MapWatchLog.Add("MAPR vp=" + vpage + " (0x" + vpage.ToString("X4") + ")"
                + "  entry=" + (w < 0 ? "????" : w.ToString("X4"))
                + "  real 0x" + (w < 0 ? "???" : (((w & 0x1F) << 8) | (w >> 8)).ToString("X4"))
                + "  flags " + (w < 0 ? "??" : (w & 0xE0).ToString("X2"))
                + (w >= 0 && (w & 0xE0) == 0x60 ? " VACANT" : "")
                + "  @" + addr.ToString("X3") + " CPi=" + InstructionCount);
        }
        private void MapWatch(int addr, ushort value)
        {
            if (_mapWatchLeft < 0)
            {
                _mapWatch = new HashSet<int>();
                _mapWatchLeft = 0;
                string s = Environment.GetEnvironmentVariable("DOVE_MAP_WATCH");
                if (!string.IsNullOrEmpty(s))
                {
                    foreach (string t in s.Split(','))
                    {
                        int v;
                        string u = t.Trim();
                        if (u.StartsWith("0x") || u.StartsWith("0X"))
                        {
                            if (int.TryParse(u.Substring(2), System.Globalization.NumberStyles.HexNumber,
                                              null, out v)) _mapWatch.Add(v);
                        }
                        else if (int.TryParse(u, out v)) _mapWatch.Add(v);
                    }
                    if (_mapWatch.Count > 0)
                    {
                        _mapWatchLeft = 4000;
                        if (MapWatchLog == null) MapWatchLog = new List<string>();
                    }
                }
            }
            if (_mapWatchLeft == 0) return;

            int mapBase = (_mapA & 0xF) << 16;
            int vpage = addr - mapBase;
            if (vpage < 0 || vpage > 0xFFFF || !_mapWatch.Contains(vpage)) return;

            _mapWatchLeft--;
            int oldw = ReadWord != null ? ReadWord(addr) : -1;
            MapWatchLog.Add("MAPW vp=" + vpage + " (0x" + vpage.ToString("X4") + ")"
                + "  " + (oldw < 0 ? "????" : oldw.ToString("X4"))
                + " -> " + value.ToString("X4")
                + "   real 0x" + (oldw < 0 ? "???" : (((oldw & 0x1F) << 8) | (oldw >> 8)).ToString("X4"))
                + " -> 0x" + ((((value & 0x1F) << 8) | (value >> 8))).ToString("X4")
                + "   flags " + (oldw < 0 ? "??" : (oldw & 0xE0).ToString("X2"))
                + " -> " + (value & 0xE0).ToString("X2")
                + "   @" + _mar.ToString("X5") + " CPi=" + InstructionCount);
        }
        public List<string> MapPhaseLog;   // the FIRST Map<- executed outside c1 (the invariant DLion asserts)
        // OVERALL microword distribution across c1/c2/c3.  This picks the lane:
        //   ~1/3 each  => the rotation is SOUND and we have a pure PHASE OFFSET (hunt the one event).
        //   skewed     => the ROTATION itself is broken (the click model needs real work).
        // Clicks rotate uniformly, so an even split is the null hypothesis.
        public readonly long[] CycleHist = new long[4];
        // Per-macro cycle pinning is a CONSTRAINT, not a convention (TmMacroTablesDaybreak `cy:`):
        //   c1 = issue address : MAR<- (real), Map<- (map array), IO<-, Refresh
        //   c2 = write data    : MDR<-, IBDisp
        //   c3 = read data     : <-MD   (gated mem && _cycle==3)
        // The click IS the memory cycle.  A Map<- at c2 is not "the wrong slot" -- it is issuing an
        // address during the data phase.  That is why the DLion reference throws on it.
        public readonly long[] MdrByCycle = new long[4];    // MDR<- (mem, c2) -- c1/c3 => BUG
        public readonly long[] IbDispByCycle = new long[4]; // IBDisp (c2)     -- outside c1/c2 => BUG
        // ENTERING-OPCODE PROBE (see the IBDisp site): the last mesa opcode dispatched, with its inputs.
        // When the run ends frozen in the @AB0 spin, this IS the opcode that entered the stuck primitive.
        public int _lastDispOp = -1, _lastDispR5, _lastDispRH5, _lastDispR2, _lastDispRH2;
        public int _lastDispR3, _lastDispRH3, _lastDispTOS, _lastDispSp, _lastDispAddr;
        // MP-940 alpha capture: for a zESC (0xF8) dispatch, the ESC alpha = the byte after it in the IB,
        // extracted the SAME way the CP executes it (_ib[ibPtr&1]) to avoid re-deriving the byte-PC lane.
        public int _lastDispAlpha = -1, _lastDispIb0, _lastDispIb1, _lastDispIbPtr;
        public long _lastDispCPi;
        // INVOKING-OPCODE latch: the operator's reframe -- @AB0 is a bulk microcode SCAN primitive
        // (carries MesaIntBr, 49K CPi long => it is the FOREGROUND being interrupted), and zLL6/zJZB are
        // its interval-timer interrupt SERVICE, not its caller.  So the steady-state "last dispatched
        // opcode" is a red herring.  The invoking opcode is the LAST dispatch before @AB0's FIRST long
        // burst -- a normal Read primitive scans a small structure and returns within a few iterations;
        // the runaway scans thousands before the timer interrupts it.  Latch when @AB0 has run > threshold
        // iterations since the last IBDisp, capturing the opcode that invoked it.  ONE-SHOT.
        public int _ab0Run;                 // @AB0 executions since the last IBDisp (reset on dispatch)
        public bool _ab0InvokeLatched;
        public string _ab0Invoke;           // the captured invoking-opcode snapshot
        public int Ab0RunThreshold = 3000;  // > any legit bulk-read scan, < the runaway's ~4000/burst
        // CARRY PROBE (operator candidate B): does @AB0's scan pointer ever carry R5 into RH5?
        // The scan walks R5 by 8; if it wraps 0xFFFF->0 without advancing RH5, it cycles vpages 0x100-0x1FF
        // forever instead of covering the [0x100, numberVirtualPages) interval.  Track the distinct RH5
        // values and the R5 span seen at @AB0, after the invocation latch fires.
        public int _ab0Rh5First = -1, _ab0Rh5Distinct, _ab0R5Min = 0x10000, _ab0R5Max = -1;
        public long _ab0Wraps;   // count of R5 0xFF..->00.. wraps observed
        private int _ab0R5Prev = -1;
        // IOP->CP doorbell (wakeup) watch: did the IOP ring the CP after the transfer, and was IE off?
        public long _mIntAsserts;
        // MP-940: split the IOP->CP doorbell (up-notify wake) by germ vs Pilot.  The germ POLLED, so
        // this path may never have run; Pilot WAITS on conditions.  If zero asserts after the germ
        // finishes (CPi>44.7M), the IOP->CP notify chain is the gate -- full stop.
        public long MIntAssertsPilot, LastMIntAssertCP;
        public List<string> MIntLog;
        public List<string> FrameChainLog;   // Mesa frame/return-link chain at the @AB0 invocation
        public List<string> IntStatSpinLog;   // <-IntStat reads in the spin -- does the germ beat at the timer period?
        private long _lastIntStatCPi;
        // pageCross-cancel DETECTOR (not yet wired to actually cancel -- measure first).
        // DLion latches _marPageCrossBr on a page-crossing MAR<- (CentralProcessor.cs:650) and uses it on the
        // NEXT instruction to cancel a pending IBDisp (:759) and a pending MDR<- (:664).  Dove has the branch
        // but neither cancel, so a cancel that should fire silently doesn't -- and the path length changes.
        private bool _marPageCrossBr;            // set by a page-crossing MAR<-, consumed next instruction
        private bool _pageCrossCancelPending;    // = _marPageCrossBr latched at the top of THIS instruction
        // MP-935 FIX: mirror DLion CentralProcessor.cs:664 -- a page-crossing MAR<- cancels the FOLLOWING MDR<-
        // store (the microcode's pageCross branch re-issues it with the carried address).  Dove had the branch and
        // the IBDisp cancel (:759) but NOT this MDR<- cancel, so the un-carried store (real page, offset wrapped to
        // 0) executed and clobbered whatever sat at page-offset 0 -- e.g. GFI 74's EFC4 control link (MP-935 root).
        private readonly bool _pageCrossMdrCancel = Environment.GetEnvironmentVariable("DOVE_NO_PAGECROSS_MDR_CANCEL") != "1";
        public long _pageCrossCount;             // how many MAR<- actually crossed a page
        public long _ibDispCancels;        // IBDisps cancelled by a preceding pageCross (DLion CentralProcessor.cs:759)
        public List<string> CancelLog;           // the first 40, with call-site state
        // A/B switch for the <-ErrnIBnStkp StkP field.  DOVE_STK_DEPTH=1 -> report true depth
        // (_stackP+1, the c2c301c model); default -> report ~_stackP, which is what
        // ProcListXferDaybreak DSKf assumes: `TT <- ~ErrnIBnStkp; TT <- TT and 0F` recovers the RAW
        // stackP by complementing this field, so the field must BE ~stackP.
        public static readonly bool _stkFieldDepth =
            System.Environment.GetEnvironmentVariable("DOVE_STK_DEPTH") == "1";
        private int _stkTraps;
        private readonly string[] _ring = new string[64];
        private int _ringPos; private bool _ringDumped;
        private int _loopCd, _loopIters;
        public int HistFrom = int.MaxValue;
        private int _escCd;            // countdown of instructions to log after an F8 dispatch
        private bool _writeFrozen;
        public readonly Dictionary<string, long> FuncHits = new Dictionary<string, long>();
        public readonly List<string> Unimplemented = new List<string>();
        public bool HaltOnUnimplemented;
        public List<string> Trace;                 // if non-null, per-instruction disassembly

        public bool Running { get { return _run && !_reset; } }
        public int CurrentAddress { get { return _tpc[_task]; } }
        public int Bank { get { return _execBank; } }
        public AM2901 ALU { get { return _alu; } }
        public ushort[] U { get { return _u; } }
        public byte[] RH { get { return _rh; } }

        // ---- Control-port surface (driven by DoveIOPIO) ----

        /// <summary>WriteResetReg bit 6 (resetMesaProcessor).  True = hold CP in reset.</summary>
        public void SetReset(bool asserted)
        {
            if (asserted && !_reset) Reset();
            _reset = asserted;
        }

        /// <summary>
        /// WriteCSReg (port 0xB0), IOP TechRef Table 2.4.  ★ bit9 (0x0200 Halt-Mesa-Proc',
        /// active-low) is THE run bit; bit14 (0x4000) is only the CS-interface mode (0 =
        /// IOP-writable WCS, 1 = normal fetch) and does NOT gate the run.  So both the
        /// loader's 0x0200 (b14 low, WCS still writable) and the kernel's 0x4200 count as
        /// "run"; halt is 0x0000.  Bit8 (0x0100) is the IOP->CP interrupt doorbell.
        /// </summary>
        public void WriteCSReg(ushort value)
        {
            _csReg = value;
            _run = (value & 0x0200) != 0;              // b9 Halt' = run bit
            if ((value & 0x0100) != 0)
            {
                // b8 IOP->CP doorbell = the IOP trying to WAKE the CP (e.g. an I/O completion).
                // Log these: if the IOP rings after the boot transfer but IE=false drops it, that is the
                // missed wakeup.  If the IOP NEVER rings after the transfer, the notify is upstream (IOP side).
                _mIntAsserts++;
                LastMIntAssertCP = InstructionCount;
                if (InstructionCount > 44700000) MIntAssertsPilot++;
                if (MIntLog != null && MIntLog.Count < 60)
                    MIntLog.Add("IOP->CP doorbell (CSReg b8) asserted @CPi " + InstructionCount
                        + "  IE=" + (_ie ? 1 : 0) + " (taken? needs IE)  csReg=0x" + value.ToString("X4"));
                _mInt = true;   // b8 IOP->CP doorbell sets the int reg
            }
        }
        private ushort _csReg;
        public ushort CSReg { get { return _csReg; } }

        public void Reset()
        {
            _cycle = 1;
            _task = 0;
            _execBank = 0;
            _bankTarget = 0;
            _bankChangePending = 0;
            _stackP = 0;
            _niaModifier = 0;
            _niaModType = 0;
            _altUAddr = false;
            _pc16 = false;
            _refillEPending = false;
            _mInt = false;
            _timerInt = false;
            _timerCounter = 0;
            _ibFront = 0;
            _ibFrontWord = 0;
            _ibWord = 0;
            _ibPtr = IBState.Empty;
            Array.Clear(_ib, 0, _ib.Length);
            _xBus = _yBus = _lastYBus = 0;
            _mar = 0;
            Array.Clear(_tpc, 0, _tpc.Length);
            Array.Clear(_link, 0, _link.Length);
            // Register files survive nothing meaningful across a reset; clear for determinism.
            Array.Clear(_u, 0, _u.Length);
            Array.Clear(_rh, 0, _rh.Length);
            MesaInterruptRequest = false;
        }

        private Microinstruction Fetch(int bank, int addr)
        {
            Microinstruction mi = _cache[bank][addr];
            ulong word = _cs.GetWord(bank, addr);
            // Rebuild the cache entry if unset or stale (the loader may rewrite words).
            if (mi == null || mi.Word != word)
            {
                mi = new Microinstruction(word);
                _cache[bank][addr] = mi;
            }
            return mi;
        }

        /// <summary>Execute up to <paramref name="cycles"/> micro-cycles (no-op while halted).</summary>
        public void Execute(int cycles)
        {
            for (int c = 0; c < cycles; c++)
            {
                bool running = Running;
                if (running && !_prevRunning)
                {
                    // Unhalt / reset-release generates an InitTrap: the CP re-enters at
                    // bank 0, location 0 regardless of the last bank-register write, and
                    // the interrupt-status trap code is 1 (InitTrap) -- the emulator image's
                    // ErrTrap@0 reads it via <-IntStat to dispatch into its boot path.
                    _tpc[_task] = 0;
                    _execBank = 0;
                    _bankChangePending = 0;
                    _cycle = 1;
                    _trapCode = 1;
                    InitTrapCount++;
                }
                _prevRunning = running;
                if (!running) return;
                Step();
            }
        }
        private bool _prevRunning;
        private readonly HashSet<int> _xkeySeen = new HashSet<int>();
        public long InitTrapCount;

        private void Note(string fn)
        {
            long n; FuncHits.TryGetValue(fn, out n); FuncHits[fn] = n + 1;
        }

        private void Unimpl(string fn, Microinstruction mi)
        {
            Note(fn);
            if (Unimplemented.Count < 200)
                Unimplemented.Add("@" + _tpc[_task].ToString("X3") + " " + fn + " : " + mi.Disassemble(-1));
            if (HaltOnUnimplemented) _run = false;
        }

        private void Step()
        {
            if (_niaWatch == -2)
            {
                string w = Environment.GetEnvironmentVariable("DOVE_NIA_WATCH");
                int wv;
                _niaWatch = (!string.IsNullOrEmpty(w) &&
                             int.TryParse(w, System.Globalization.NumberStyles.HexNumber, null, out wv)) ? wv : -1;
            }
            int addr = _tpc[_task];
            _uRing[(int)(_uRingN++ & 0xFF)] = (_execBank << 12) | addr;   // bank-tagged: an untagged ring cannot attribute a spin   // last 256 microstore addresses -- characterises a spin
            if (AddrHist != null && InstructionCount >= HistFrom) AddrHist[addr & 0xFFF]++;
            Microinstruction mi = Fetch(_execBank, addr);

            // TEMP: entry-path ring -- records the last 64 microwords and dumps them at the FIRST
            // hit of LoopAddr, so we can see how control ARRIVED (CALL vs fell-in-from-a-dispatch).
            if (LoopLog != null && !_ringDumped && InstructionCount >= RingFrom)
            {
                _ring[_ringPos & 63] = "   @" + addr.ToString("X3") + " c" + _cycle + " X=" + _xBus.ToString("X4")
                    + " mar=" + _mar.ToString("X5")
                    + " Regb/R11=" + _alu.R[11].ToString("X4") + " bW/R2=" + _alu.R[2].ToString("X4")
                    + " VD/R1=" + _alu.R[1].ToString("X4")
                    + " L1=" + _link[1].ToString("X") + " L3=" + _link[3].ToString("X")
                    + " Lw=" + mi.LinkAddress + " niaMod=" + _niaModifier.ToString("X3") + "  " + mi.Disassemble(-1);
                _ringPos++;
                if (addr == LoopAddr)
                {
                    _ringDumped = true;
                    LoopLog.Add("=== PATH INTO @" + addr.ToString("X3") + " -- FIRST hit, CPi=" + InstructionCount
                        + " (last 64 microwords; look for CALL[ComMap] vs a dispatch falling in) ===");
                    for (int k = 0; k < 64; k++) { string e = _ring[(_ringPos + k) & 63]; if (e != null) LoopLog.Add(e); }
                    LoopLog.Add("=== (end path) ===");
                }
            }

            // TEMP: microcode-spin iteration dump -- full register/link state at the loop head, then
            // every microword of the iteration.  Names the counter and the exit-branch outcome.
            if (LoopLog != null && addr == LoopAddr && InstructionCount >= LoopFrom && _loopIters < 4)
            {
                _loopIters++;
                LoopLog.Add("=== ITER " + _loopIters + " @" + addr.ToString("X3") + " CPi=" + InstructionCount
                    + "  R0-15=" + string.Join(",", System.Linq.Enumerable.Select(System.Linq.Enumerable.Range(0, 16), i => _alu.R[i].ToString("X4")))
                    + "  RH0-15=" + string.Join(",", System.Linq.Enumerable.Select(System.Linq.Enumerable.Range(0, 16), i => _rh[i].ToString("X2")))
                    + "  Q=" + _alu.Q.ToString("X4")
                    + "  L0-7=" + string.Join(",", System.Linq.Enumerable.Select(System.Linq.Enumerable.Range(0, 8), i => _link[i].ToString("X")))
                    + "  sp=" + _stackP + " ===");
                _loopCd = 30;
            }
            if (_loopCd > 0 && LoopLog != null)
            {
                LoopLog.Add("   @" + addr.ToString("X3") + " c" + _cycle + " X=" + _xBus.ToString("X4") + " mar=" + _mar.ToString("X5")
                    + " R4=" + _alu.R[4].ToString("X4") + " RH4=" + _rh[4].ToString("X2")
                    + " R6=" + _alu.R[6].ToString("X4") + " RH6=" + _rh[6].ToString("X2")
                    + " Q=" + _alu.Q.ToString("X4")
                    + " L1=" + _link[1].ToString("X") + " L3=" + _link[3].ToString("X")
                    + " niaMod=" + _niaModifier.ToString("X3") + "  " + mi.Disassemble(-1));
                _loopCd--;
            }

            // TEMP: @ESC alpha-dispatch trace (log the microaddr path after an F8 dispatch).
            if (_escCd > 0 && EscLog != null)
            {
                EscLog.Add("  @" + addr.ToString("X3") + " c" + _cycle + " ib=" + _ibFront.ToString("X2")
                    + " mar=" + _mar.ToString("X5") + " X=" + _xBus.ToString("X4")
                    + " L=[" + _rh[3].ToString("X2") + ":" + _alu.R[3].ToString("X4") + "]"
                    + " Q=" + _alu.Q.ToString("X4") + " aD=" + mi.aD + " rB=" + mi.rB
                    + " sp=" + _stackP + " TOS=" + _alu.R[0].ToString("X4")
                    + " push=" + (mi.Push ? 1 : 0) + " pop=" + (mi.Pop ? 1 : 0) + " dpop=" + (mi.DoublePop ? 1 : 0)
                    + " stkOp=" + (mi.StackOperation ? 1 : 0) + " stkTest=" + mi.StackTest + " ldSP=" + (mi.LoadStackP ? 1 : 0)
                    + " niaMod=" + _niaModifier.ToString("X3")
                    + "  " + mi.Disassemble(-1));
                _escCd--;
            }

            // TEMP: microword-field decode at execution time for the net-0 zSGB faulting deref path.
            if (XWtLog != null && InstructionCount >= 1250 && InstructionCount <= 1400)
                XWtLog.Add("MW @" + addr.ToString("X3") + " c" + _cycle + " CPi=" + InstructionCount
                    + " mem=" + (mi.mem ? 1 : 0) + " LoadMap=" + (mi.LoadMap ? 1 : 0) + " MarMapMDR=" + (mi.MarMapMDR ? 1 : 0)
                    + " fSfY=" + (int)mi.fSfY + " fY=" + mi.fY.ToString("X") + " rB=" + mi.rB
                    + " INIA=" + mi.INIA.ToString("X3") + " X=" + _xBus.ToString("X4") + " Y=" + _yBus.ToString("X4") + " mar=" + _mar.ToString("X5")
                    + "  " + mi.Disassemble(-1));

            // TEMP: capture one full 0900-loop iteration at the microinstruction level.
            if (LoopTrace != null && InstructionCount >= LoopTraceFrom && InstructionCount < LoopTraceFrom + 520)
                LoopTrace.Add("@" + addr.ToString("X3") + " c" + _cycle + " CPi=" + InstructionCount
                    + " R5=" + _alu.R[5].ToString("X4") + " RH5=" + _rh[5].ToString("X") + " pc16=" + (_pc16?1:0) + " ibPtr=" + _ibPtr + " ibF=" + _ibFront.ToString("X2")
                    + " fw=" + _ibFrontWord.ToString("X4") + " off=" + ((_alu.R[5] - _ibFrontWord) & 0xFFFF).ToString("X4")   // MEASUREMENT: R5 vs IB-front word
                    + " Q=" + _alu.Q.ToString("X4") + " X=" + _xBus.ToString("X4") + " Y=" + _yBus.ToString("X4") + " mar=" + _mar.ToString("X5")
                    + " R[0-7]=" + _alu.R[0].ToString("X4") + "," + _alu.R[1].ToString("X4") + "," + _alu.R[2].ToString("X4") + "," + _alu.R[3].ToString("X4") + "," + _alu.R[4].ToString("X4") + "," + _alu.R[6].ToString("X4") + "," + _alu.R[7].ToString("X4")
                    + "  " + mi.Disassemble(-1));

            if (Trace != null && Trace.Count < 4000)
                Trace.Add("[" + _execBank + ":" + addr.ToString("X3") + "] c" + _cycle
                    + " fSfY=" + (int)mi.fSfY + " fY=" + mi.fY.ToString("X") + " fSfZ=" + (int)mi.fSfZ + " fZ=" + mi.fZ.ToString("X")
                    + "  " + mi.Disassemble(-1));

            bool cIn = mi.Cin;
            ushort _r5old = _alu.R[5]; byte _rh5old = _rh[5];
            int niaModifier = _niaModifier;
            _niaModifier = 0;
            _xDispNow = 0;
            int niaModType = _niaModType;   // latched with niaModifier (applied one instruction late)
            _niaModType = 0;
            bool altUAddr = _altUAddr;
            _altUAddr = false;
            _lastYBus = _yBus;

            // ---- X bus source ----
            _xBus = mi.Byte;                       // constant (0 if none)

            // IOXIn (fSfZ=3): read an on-chip status source onto the X bus (operator
            // answer 9; NO IOP data port -- CP<->IOP is shared memory + the two latches).
            if (mi.fSfZ == FunctionSelectFZ.IOXIn)
            {
                switch (mi.fZ)
                {
                    // NB: X bits are numbered MSB-first, so X[8-15] is the LOW byte:
                    // X[8-9]=bits 7:6, X[10-11]=bits 5:4, X[12-15]=bits 3:0,
                    // X13=bit 2, X14=bit 1, X15=bit 0.
                    case 0x9:   // <-IntStat: X-bus is MSB-first (bit0=0x8000 .. bit15=0x0001).  The
                                // reasons sit at X[13]=MesaIntRq=0x0004, X[14]=IOP=0x0002,
                                // X[15]=timer=0x0001.  (The 8254 status ".2" is NOT the X-bus value.)
                                // X[8-9]=trap code.  Reading acks the timer edge (read-to-clear).
                        // fZ 9 = ErrnIBInt.  Proven from Daybreak.dfn:488 -- RegDef[ULsave, U, 3A]
                        // encodes "rA = L = 3, fZ = ErrnIBnStkp = A" in the U-register address (low
                        // nibble = fZ), and ErrnIBInt is the one-off assembler built-in for the fZ 9
                        // slot (it appears exactly once in the whole microcode, at Refill.mc:196).
                        //
                        // fZ 9 and fZ A share u45's FIRST enable half, so X[8-9] (trap, true) and
                        // X[10-11] (~ibPtr) are IDENTICAL between them; they diverge only below, in
                        // ~stackP (A) vs IntStat (9).  We were omitting ~ibPtr here, which is what
                        // made ErrTrap's dispatch nibble -- {Trap0,Trap1,~ibPtr0,~ibPtr1} after the
                        // LRot12 -- read as pure zero rather than (EKErr << 2) | ~ibPtr.
                        _xBus = (ushort)(((_trapCode & 3) << 6)
                                       | (((~(int)_ibPtr) & 0x3) << 4)
                                       | (_mInt ? 0x2 : 0) | (_timerInt ? 0x1 : 0));
                        IntStatReads++;
                        // TIMER-DRIVEN-SPIN PROBE: log <-IntStat reads in the spin window with the CPi delta
                        // since the last read, and whether the timer bit was set.  If the germ reads IntStat
                        // (un-IE-gated, so the timer bit is visible with IE off) at ~TimerPeriod (40000 CPi)
                        // cadence, the free-running 8254 is driving the @AB0 Scan spin despite IE=off.
                        if (IntStatSpinLog != null && InstructionCount >= 8000000 && IntStatSpinLog.Count < 40)
                            IntStatSpinLog.Add("<-IntStat @CPi " + InstructionCount + "  dCPi=" + (InstructionCount - _lastIntStatCPi)
                                + "  timerBit=" + (_timerInt ? 1 : 0) + " mIntBit=" + (_mInt ? 1 : 0) + " IE=" + (_ie ? 1 : 0));
                        _lastIntStatCPi = InstructionCount;
                        _timerInt = false;
                        // MP-940 FIX (CONFIRMED + VERIFIED): the IOP->CP doorbell (_mInt) is read-to-clear on
                        // <-IntStat, SYMMETRIC with the timer edge above.  The Daybreak interrupt microcode reads
                        // IntStat and ORs it into rInt (5 sites), then clears rInt at IntReturns -- there is NO
                        // port-write clearing an interrupt latch, so read-to-clear across ALL THREE bits
                        // (13=MesaInt / 14=IOP / 15=timer) is the only model the microcode is consistent with.
                        // Without this, the doorbell latch (set by IOP WriteCSReg b8) is NEVER cleared except by
                        // Reset -> under Pilot (IE=1) MesaIntBr re-fires every idle pass -> IdleLoop diverts to
                        // IdleInt -> resumes the waiting PSB instead of rescheduling -> aMW (Monitor.Wait) never
                        // blocks -> MP-940 stall.  Verified: with this, aMW blocks, the reschedule switches
                        // processes, and the boot advances past 940.  (env DOVE_ACK_MINT=0 restores the bug for A/B.)
                        if (Environment.GetEnvironmentVariable("DOVE_ACK_MINT") != "0") _mInt = false;
                        _trapCode = 0;   // read-to-clear: the InitTrap code is acked by the @0 read
                        break;
                    case 0xA:   // <-ErrnIBnStkp: X[8-9]=trap, X[10-11]=~ibPtr, X[12-15]=~stackP.
                                // The StkP field is ~stackP, and _stackP counts UP from 0 == the true depth
                                // (TechRef Table 2.11: push -> +1, trap at 15).  MEASURED CORRECT -- do not
                                // "fix" this field again:
                                //   At the germ's aBITBLT (zESC alpha 0x2B) _stackP=2 (a 2-word BBptr), so this
                                //   returns ~2 = 0x0D; BBInit.mc:28/29 (Xbus <- ErrnIBnStkp, XDisp;
                                //   DISP4[HowBigStack,08]) then lands on microstore 0x03D, and 0x03D IS
                                //   bbNormEntry -- the trace executes BBInit.mc:35/37/38 (VS <- UBitBltArg;
                                //   VS <- VS and ~0F; rhRet <- argMap, CALL[SrcMapSpec]) and goes on to run
                                //   bbGetArg (BBInit.mc:44-60), reading the bbTable at offsets 8,A,3,7,9 in
                                //   source order with UWidth = 0x0010 = the 16x1 probe blt.  It does NOT reach
                                //   BandBLT's interrupt entry.
                                // The trap for the unwary: `hbs.N` names the stackP VALUE N, not a microstore
                                // offset.  The assembler places hbs.N at base|(~N & 7) -- so ~2=0x0D reaching
                                // 0x038|5 = 0x03D is exactly how you arrive at hbs.2.  Reading "hbs.2" as a
                                // dispatch value of 2 is what produced the (false) story that this field was
                                // broken.  cf. TextBlt.mc:186 `TTgetsSTKRet[0B] {1011 is one's complement of
                                // stackP = 4}` -- the same complement convention, spelled out in the source.
                                // Other consumers agree the field is ~stackP: ProcListXferDaybreak.mc:372 DSKf
                                // (`TT <- ~ErrnIBnStkp` to recover the raw pointer), Floyd.mc:37, Misc.mc:369.
                                // c2c301c reported (_stackP+1) here and was reverted by bc9d83e; DOVE_STK_DEPTH=1
                                // re-creates it (DSKf then clamps to 0E at every trap entry -- ProcListXfer:381).
                        _xBus = (ushort)(((_trapCode & 3) << 6) | (((~(int)_ibPtr) & 0x3) << 4)
                            | ((_stkFieldDepth ? (_stackP + 1) : ~_stackP) & 0xf));
                        if (ProcSwLog != null && InstructionCount >= ProcSwFrom && InstructionCount < ProcSwTo
                            && ProcSwLog.Count < 400)
                            ProcSwLog.Add("SAVE <-ErrnIBnStkp CPi=" + InstructionCount + " @" + addr.ToString("X3")
                                + "  stackP=" + _stackP + "  field(~sp)=0x" + (_xBus & 0xf).ToString("X")
                                + "  ibPtr=" + (int)_ibPtr + " trap=" + _trapCode);
                        _trapCode = 0;   // read-to-clear
                        break;
                    case 0xB:   // <-RH
                        _xBus = _rh[mi.rB];
                        break;
                    case 0xC:   // <-ibNA (front, no advance)
                        NoteIbEmptyRead();
                        _xBus = _ibFront;
                        break;
                    case 0xD:   // <-ib (front, then advance: refill ibFront from IB and decrement ibPtr)
                        if (IbLog != null && InstructionCount >= IbLogFrom && InstructionCount < IbLogTo)
                            IbLog.Add("@" + addr.ToString("X3") + " CPi=" + InstructionCount + " <-ib reads front=" + _ibFront.ToString("X2") + " ptr=" + _ibPtr + " (next front<-_ib[" + (((int)_ibPtr) & 0x1) + "]=" + _ib[((int)_ibPtr) & 0x1].ToString("X2") + ") ib=[" + _ib[0].ToString("X2") + "," + _ib[1].ToString("X2") + "]" + (_ibPtr == IBState.Empty ? "  <<< READ FROM EMPTY (DLion would trap/refill)" : ""));
                        NoteIbEmptyRead();
                        _xBus = _ibFront;
                        _ibFront = _ib[((int)_ibPtr) & 0x1];
                        _ibPtr = _nextIBPtr[(int)_ibPtr];
                        _ibFrontWord = _ibWord;   // MEASUREMENT: front now comes from the _ib pair's word
                        break;
                    case 0xE:   // <-ibLow
                        NoteIbEmptyRead();
                        _xBus = (ushort)(_ibFront & 0xf);
                        break;
                    case 0xF:   // <-ibHigh
                        NoteIbEmptyRead();
                        _xBus = (ushort)((_ibFront >> 4) & 0xf);
                        break;
                    default:    // 6/8 = ExtStat/DebB (Burdock debug byte pipe): 0
                        _xBus = 0;
                        Note("IOXIn:" + mi.fZ.ToString("X"));
                        break;
                }

                // ErrTrap forensics.  Refill.mc:196 is  "rInt <- ErrnIBInt, ClrIntErr, ... c1, at[0]"
                // and c2 dispatches DISP4 on rInt LRot12.  Slot 3 = UnexpectedErr = a terminal
                // GOTO-self, and slot 3 is what you get when the dispatch nibble reads 0 -- i.e.
                // when NO EKErr is set.  So the one number that matters is the value this read
                // actually latches into rInt.  Record it, with the fZ code that produced it:
                // ErrnIBInt is a THIRD mnemonic (only Refill.mc:196 uses it; everything else reads
                // ErrnIBnStkp), and only the ErrnIBnStkp encoding -- trap at bits 6-7, ~ibPtr at
                // 4-5 -- yields the documented slots 7/0B/0F for EKErr 1/2/3.  If ErrTrap assembles
                // to a different fZ we serve as something else (or as the default 0), every trap
                // funnels to slot 3 regardless of the code, which is exactly the observed wedge.
                if (addr == 0 && ErrTrapLog != null && ErrTrapLog.Count < 200)
                {
                    // The 24 microwords that led here.  trapCode=0 with no trap counted means
                    // we did not TAKE a trap -- something branched to address 0 -- so the
                    // predecessor is the whole question.  The ring excludes this instruction.
                    var pre = new System.Text.StringBuilder("    came from:");
                    for (int k = 24; k >= 1; k--)
                    {
                        int pv = _uRing[(int)((_uRingN - 1 - k) & 0xFF)];
                        pre.Append(" " + (pv >> 12) + ":" + (pv & 0xFFF).ToString("X3"));
                    }
                    ErrTrapLog.Add(pre.ToString());
                    ErrTrapLog.Add("addr0 bank=" + _execBank + " c" + _cycle + " fZ=" + mi.fZ.ToString("X")
                        + " -> xBus=" + _xBus.ToString("X4")
                        + "  nibble(bits4-7)=" + ((_xBus >> 4) & 0xF).ToString("X")
                        + "  trapCode=" + _trapCode + " ibPtr=" + (int)_ibPtr
                        + " stackP=" + _stackP + " CPi=" + InstructionCount);
                }
            }

            // Cin<-pc16 : load the ALU carry-in from the byte-PC bit (pc16) and schedule pc16 to
            // INVERT at end of cycle.  This is the byte-granular half of the 17-bit macro-PC
            // {R5, pc16}: `SE<-pc16, R5<-R5+1` advances the byte pointer by one byte -- when pc16=1
            // (low byte) R5 carries to the next word and pc16->0; when pc16=0 (high byte) R5 stays
            // and pc16->1.  Two encodings (HW ref 2.3.7 / microcode ref pg 21):
            //   fX form (XFunction.LoadCinFrompc16):     Cin <- pc16 unconditionally.
            //   fZ form (ZNormFunction.LoadCinFrompc16): Cin <- pc16 only if the microword Cin bit is
            //           already 1; if Cin=0 the fZ form yields Cin=0 (NOT pc16) -- a hardware quirk.
            // BOTH forms invert pc16 at end of cycle.  Missing this toggle froze pc16 at 0, so R5
            // never carried across a word boundary and fell one word behind the IB front word,
            // corrupting @LIW's word-crossing immediate fetch (the phantom-@BLTL saga).
            bool invertPc16 = false;
            if (mi.fX == XFunction.LoadCinFrompc16)
            {
                cIn = _pc16;
                invertPc16 = true;
            }
            else if (mi.fSfZ == FunctionSelectFZ.fzNorm && mi.fZ == (int)ZNormFunction.LoadCinFrompc16)
            {
                if (cIn) cIn = _pc16;
                invertPc16 = true;
            }

            if (UwDumpLog != null && UwDumpLog.Count < 40
                && (addr == UwDumpA || addr == UwDumpB || addr == UwDumpC))
            {
                UwDumpLog.Add("uw @" + addr.ToString("X3") + " raw=0x" + mi.Word.ToString("X12")
                    + "  rA=" + mi.rA.ToString("X") + " rB=" + mi.rB.ToString("X")
                    + " fSfZ=" + (int)mi.fSfZ + " fSfY=" + (int)mi.fSfY
                    + " fX=" + ((int)mi.fX).ToString("X") + " fY=" + mi.fY.ToString("X")
                    + " fZ=" + mi.fZ.ToString("X")
                    + " | UAddress=(rA<<4)|fZ=0x" + mi.UAddress.ToString("X2")
                    + "  alt=(fZ<<4)|rA=0x" + (((mi.fZ << 4) | mi.rA)).ToString("X2")
                    + "  SURead=" + (mi.SURead ? 1 : 0) + " SUWrite=" + (mi.SUWrite ? 1 : 0)
                    + " stackP=" + _stackP + " CPi=" + InstructionCount
                    + " | " + mi.Disassemble(-1));
            }
            if (UAddrHist != null && (mi.SURead || mi.SUWrite) && (int)mi.fSfZ >= 2
                && InstructionCount >= UAddrFrom && InstructionCount < UAddrTo)
                UAddrHist[mi.UAddress & 0xFF]++;
            if (mi.SURead)
            {
                switch ((int)mi.fSfZ)
                {
                    case 0:
                    case 1:
                        _xBus = _u[_stackP];
                        break;
                    default: // 2,3
                        _xBus = altUAddr
                            ? _u[(mi.rA << 4) | (_lastYBus & 0xf)]
                            : _u[mi.UAddress];
                        break;
                }
            }

            // <-MD : memory read result appears on the X bus in C3.
            if (mi.mem && _cycle == 3 && ReadWord != null)
            {
                // Two-source <-MD contract (analyst source dig 2026-07-15, GetMapFlags/SMFa Misc.mc +
                // XCode/XMapG/@FF/@WRMDS): the two "sources" are the two c1 ADDRESS computations, not two
                // read forms.  Map<- addresses the map array (MAPA_base + vpage); MAR<- addresses real
                // memory.  In BOTH cases <-MD returns the RAW word at _mar -- there is NO decode.  The germ
                // ucode consumes the raw map entry two ways: translation loads it straight into rhRx/Rx and
                // the real page falls out of the register nibble/byte routing (map fmt |rp[5-12]|r|d|w|rp[0-4]|),
                // GetMapFlags LRot12's out the flag bits.  A pre-decoded <-MD re-mangles it -> R5=EEEE / MP-0200.
                if (_ioRefPending) { _xBus = Pit8254Read(); _ioRefPending = false; }
                else _xBus = ReadWord(_mar);
                MapWatchFollowRead();
                if (WalkRdLog != null && InstructionCount >= WalkRdFrom && InstructionCount < WalkRdTo
                    && _mar >= 0x40000 && WalkRdLog.Count < 4000)
                {
                    // Spectator gate: capture ONLY while the germ's current frame is GFI 109 (Signals =
                    // SignalHandler/CheckCatch).  current GFI = [L-2]>>2 (the local frame's global link).
                    // Tag reads by target: low-MDS 0x781xx = AllocationVector (IGNORE - the allocator
                    // masquerade filter); 0x78xxx-0x79xxx = frame words ([F-1]=ReadPC,[F-3]=walk step);
                    // else = codebase (zCATCH scan / enable intervals).
                    int Lr = ((_rh[3] & 0x1F) << 16) | _alu.R[3];
                    int curGFI = (Lr > 0x100) ? (ReadWord(Lr - 2) >> 2) : -1;
                    if (WalkRdGfi < 0 || curGFI == WalkRdGfi)   // WalkRdGfi<0 = log every frame (no GFI gate)
                    {
                        string tg = (_mar >= 0x78100 && _mar <= 0x781FF) ? "AV" :
                                    ((_mar >= 0x78000 && _mar <= 0x7A000) ? "FRM" : "COD");
                        WalkRdLog.Add("CPi=" + InstructionCount + " @" + addr.ToString("X3") + " " + tg
                            + " mar=0x" + _mar.ToString("X5") + " =0x" + _xBus.ToString("X4")
                            + " L=0x" + Lr.ToString("X5") + " R5=" + _alu.R[5].ToString("X4") + " RH5=" + _rh[5].ToString("X"));
                    }
                }
                // Module code-region census (env DOVE_MODCENS_FROM/TO).  Excludes the VM-map array
                // (0x40000-0x41FFF) and the frame heap (0x78000-0x7A000) so only real code counts.
                if (ModCensCount != null && InstructionCount >= ModCensFrom && InstructionCount < ModCensTo
                    && _mar >= 0x42000 && !(_mar >= 0x78000 && _mar <= 0x7A000))
                {
                    int Lc = ((_rh[3] & 0x1F) << 16) | _alu.R[3];
                    if (Lc > 0x1000)
                    {
                        int gw = ReadWord(Lc - 2);
                        if ((gw & 3) == 0)
                        {
                            int gi = gw >> 2;
                            if (gi > 0 && gi < 256)
                            {
                                ModCensCount[gi]++;
                                if (_mar < ModCensMin[gi]) ModCensMin[gi] = _mar;
                                if (_mar > ModCensMax[gi]) ModCensMax[gi] = _mar;
                                if (ModCensSample[gi] == null) ModCensSample[gi] = "";
                                if (ModCensSample[gi].Length < 70)
                                    ModCensSample[gi] += " 0x" + _mar.ToString("X5") + "=" + _xBus.ToString("X4");
                            }
                        }
                    }
                }
                // MP-940 stall: histogram the addresses the dominant busy-spin reads (Mesa PC
                // 0x898F/0x99D7).  The hottest = the cell Pilot's Store polls forever -> names the gate.
                if (PollAddrHist != null && InstructionCount > 50000000 && (_lastDispR5 == 0x898F || _lastDispR5 == 0x99D7))
                {
                    long pc; PollAddrHist.TryGetValue(_mar, out pc); PollAddrHist[_mar] = pc + 1;
                    if (PollValByAddr != null) PollValByAddr[_mar] = _xBus;
                    PollLastVal = _xBus; PollLastAddr = _mar;
                }
                // TEMP: GetHandlerIORegionPtr:126 reads IORegion.segments[16] at IORegion+0x22 words.
                // Real segment table is at CP word 0x52000 (real 0x520 = IORegion vp 0xC0); segments[16] at 0x52022.
                // If this fires, GetHandlerIORegionPtr ran on the CORRECT IORegion. If it never fires, IORegion is the default.
                if (IORgnWriteLog != null && IORgnWriteLog.Count < 70 && _mar >= 0x52000 && _mar <= 0x52040)
                    IORgnWriteLog.Add("SEGTBL read word " + _mar.ToString("X5") + " = " + _xBus.ToString("X4") + " @" + addr.ToString("X3") + " CPi=" + InstructionCount);
                if (ReadLog != null && !_writeFrozen && _mar >= 0x50000)   // reads of the IORegion/high mailbox
                {
                    ReadLog.Add("@" + addr.ToString("X3") + " CPi=" + InstructionCount + " <-MD word " + _mar.ToString("X5") + " = " + _xBus.ToString("X4"));
                    if (ReadLog.Count > 40) ReadLog.RemoveAt(0);
                }
                // OQ54: map-array reads = GetState inputs.  vp = _mar - 0x40000; decode realpage = ((w&0x1F)<<8)|(w>>8).
                if (MapArrRead != null && _mar >= 0x400C0 && _mar <= 0x40101 && MapArrRead.Count < 300)
                {
                    int rp = ((_xBus & 0x1F) << 8) | (_xBus >> 8);
                    MapArrRead.Add("<-MD map[vp 0x" + (_mar - 0x40000).ToString("X3") + "] = " + _xBus.ToString("X4") + " -> real 0x" + rp.ToString("X3") + " @" + addr.ToString("X3") + " CPi=" + InstructionCount);
                }
                // OQ59: SD trap-table reads (real 0x48200+, SD[n].gf@+2n, SD[n].pc@+2n+1). Watch the trap dispatch
                // read SD[7](sCodeTrap 0x4820E/F) then escalate to SD[6](sControlTrap 0x4820C/D)=[0B5D,06F4]=GermWorldError.
                if (SdReadLog != null && _mar >= 0x48200 && _mar <= 0x48220 && SdReadLog.Count < 6000)
                {
                    int slot = (_mar - 0x48200) / 2; string field = ((_mar & 1) == 0) ? "gf" : "pc";
                    // The trap microroutine reads SD[n] with no IBDisp in between, so _lastDispOp is the
                    // FAULTING opcode (the mesa bytecode whose real-microcode IBDisp entry routed to the trap
                    // routine that is now reading this SD link).  Emit it + its PC so the fault names itself.
                    SdReadLog.Add("<-MD SD[" + slot.ToString("X2") + "]." + field + " (real " + _mar.ToString("X5") + ") = " + _xBus.ToString("X4") + " @" + addr.ToString("X3") + " R5=" + _alu.R[5].ToString("X4") + " RH5=" + _rh[5].ToString("X") + " CPi=" + InstructionCount
                        + "  [faultOp=0x" + _lastDispOp.ToString("X2") + " @disp" + _lastDispAddr.ToString("X3") + " dispPC={" + _lastDispRH5.ToString("X2") + ":" + _lastDispR5.ToString("X4") + "} dispCPi=" + _lastDispCPi + " gap=" + (InstructionCount - _lastDispCPi) + "]");
                }
                if (SemaLog != null && _mar == SemaWord && SemaLog.Count < 70)
                    SemaLog.Add("@" + addr.ToString("X3") + " <-MD word " + _mar.ToString("X5") + " -> Xbus " + _xBus.ToString("X4")
                        + " (R0/TOS=" + _alu.R[0].ToString("X4") + " T=" + _alu.R[mi.rA].ToString("X4") + ") CPi=" + InstructionCount + "  " + mi.Disassemble(-1));
            }

            // ---- EARLY L-rotate: the shifter feeds the 2901's D input ----
            // The shifter's input is the Y bus and its output drives the X bus, which is
            // wired to D (TmMacroTablesDaybreak: LRot8 => xOut:t, yIn:t).  With aD=aPass
            // (ABypass) the Y bus is A = R[rA] *before* the ALU, so the whole chain is
            // combinational within one cycle:
            //     A -> Y -> shifter -> X -> D -> ALU(pass) -> F -> RAM[rB]
            // This is the idiom used by `rIORgn <- rIORgn LRot8` (CS048) and by the
            // mailbox-pointer assembly (rMailBox <- rMailBox LRot8/LRot4).  The DLion's
            // LateLRotN deliberately excludes ABypass because that case rotates early.
            if (mi.ABypass && mi.fSfZ == FunctionSelectFZ.fzNorm)
            {
                ushort a = _alu.R[mi.rA];
                switch ((ZNormFunction)mi.fZ)
                {
                    case ZNormFunction.LRot0: _xBus = a; break;
                    case ZNormFunction.LRot12: _xBus = (ushort)((a << 12) | (a >> 4)); break;
                    case ZNormFunction.LRot8: _xBus = (ushort)((a << 8) | (a >> 8)); break;
                    case ZNormFunction.LRot4: _xBus = (ushort)((a << 4) | (a >> 12)); break;
                }
            }

            // ---- ALU ----
            ushort _shiftIn = _alu.R[mi.rA];
            ushort _bOld = _alu.R[mi.rB];
            ushort _r0Old = _alu.R[0];   // TOS(R0) BEFORE this instr's write -- for the mesa push spill
            ushort _qBefore = _alu.Q;
            // NOTE: the DLion MAR<- upper-byte splice (loadMAR arg) is a DLion page-relative
            // addressing quirk that overwrites F[8:15] with R[rB][8:15].  Daybreak forms MAR
            // separately (RH[rB] high + Ybus low) and this splice only corrupts register/Q
            // write-backs (e.g. a plain "Q<- Q" at @AAE turned 0x31C6 into 0xB1C6).  Keep it off.
            _yBus = _alu.Execute(mi, _xBus, cIn, false);   // splice OFF
            if (QLog != null && _alu.Q != _qBefore && QLog.Count < 200)
                QLog.Add("[" + _execBank + ":" + addr.ToString("X3") + "] c" + _cycle
                    + " aS=" + mi.aS + " aF=" + mi.aF + " aD=" + mi.aD + " AluDst=" + mi.AluDestination + " fX=" + mi.fX.ToString("X")
                    + " rA=" + mi.rA + "(" + _shiftIn.ToString("X4") + ") rB=" + mi.rB + "(" + _bOld.ToString("X4") + ") Cin=" + (cIn ? 1 : 0)
                    + " X=" + _xBus.ToString("X4") + " Q:" + _qBefore.ToString("X4") + "->" + _alu.Q.ToString("X4")
                    + " Y=" + _yBus.ToString("X4") + "  " + mi.Disassemble(-1));
            if (ShiftLog != null && mi.Shift && ShiftLog.Count < 30)
                ShiftLog.Add("@" + addr.ToString("X3") + " shift fX=" + mi.fX.ToString("X") + " R[" + mi.rA + "]=" + _shiftIn.ToString("X4")
                    + " Cin=" + (mi.Cin ? 1 : 0) + " cIn=" + (cIn ? 1 : 0) + " aD=" + mi.aD + " AluDst=" + mi.AluDestination + " -> Y=" + _yBus.ToString("X4") + " R[" + mi.rB + "]=" + _alu.R[mi.rB].ToString("X4"));

            // Latch the previous instruction's pageCross for THIS instruction's cancel test, then clear it
            // (DLion CentralProcessor.cs:279-280 does exactly this at the top of each instruction).
            _pageCrossCancelPending = _marPageCrossBr;
            _marPageCrossBr = false;

            // INVOKING-OPCODE latch: count @AB0 executions since the last dispatch; when the first burst
            // runs long enough to be the runaway (not a legit small scan), capture the invoking opcode.
            if (addr == 0xAB0)
            {
                _ab0Run++;
                // CARRY PROBE: once the runaway is latched, watch RH5 and R5.
                if (_ab0InvokeLatched)
                {
                    int rh5 = _rh[5], r5 = _alu.R[5];
                    if (_ab0Rh5First < 0) { _ab0Rh5First = rh5; _ab0Rh5Distinct = 1; }
                    else if (rh5 != _ab0Rh5First && _ab0Rh5Distinct < 2) _ab0Rh5Distinct = 2;  // saw a 2nd RH5
                    if (r5 < _ab0R5Min) _ab0R5Min = r5;
                    if (r5 > _ab0R5Max) _ab0R5Max = r5;
                    if (_ab0R5Prev >= 0 && _ab0R5Prev > 0xF000 && r5 < 0x1000) _ab0Wraps++;   // 0xFF..->00.. wrap
                    _ab0R5Prev = r5;
                }
                if (_ab0Run == Ab0RunThreshold && !_ab0InvokeLatched)
                {
                    _ab0InvokeLatched = true;
                    int va = ((_lastDispRH5 & 0x1F) << 16) | _lastDispR5;
                    _ab0Invoke = "INVOKING OP=0x" + _lastDispOp.ToString("X2") + " @" + _lastDispAddr.ToString("X3")
                        + " dispatchedCPi=" + _lastDispCPi + "  ab0FirstBurstCPi=" + InstructionCount
                        + "   inputs: R5=" + _lastDispR5.ToString("X4") + " RH5=" + _lastDispRH5.ToString("X2")
                        + " (PC vaddr=" + va.ToString("X5") + ", vpage=0x" + (va >> 8).ToString("X3") + ")"
                        + "  R2=" + _lastDispR2.ToString("X4") + " RH2=" + _lastDispRH2.ToString("X2")
                        + "  R3=" + _lastDispR3.ToString("X4") + " RH3=" + _lastDispRH3.ToString("X2")
                        + "  TOS=" + _lastDispTOS.ToString("X4") + " sp=" + _lastDispSp
                        + "  |  @AB0 scan-state now: R5=" + _alu.R[5].ToString("X4") + " RH5=" + _rh[5].ToString("X2")
                        + " R2=" + _alu.R[2].ToString("X4") + " RH2=" + _rh[2].ToString("X2") + " mar=" + _mar.ToString("X5");
                    // FRAME CHAIN at the invocation: L = (RH3<<16)|R3.  Mesa frame header (PrincOps): L-4=fsi/
                    // LocalWord, L-3=returnlink (caller's L), L-2=globallink (module global frame GF), L-1=pc.
                    // Follow returnlink to build the call stack -- names WHICH germ routine entered the scheduler
                    // (the poll-based floppy transfer-wait canNOT put the germ in @AB0, so this is a DIFFERENT
                    // block: a Monitor.Wait / Process op / fault).  GF (globallink) + codebase name the module.
                    if (ReadWord != null && FrameChainLog != null)
                    {
                        int L = ((_lastDispRH3 & 0x1F) << 16) | _lastDispR3;
                        int mds = L & 0xFF0000;   // MDS bank; frame/GF links are MDS-relative 16-bit words
                        FrameChainLog.Add("=== FRAME CHAIN at @AB0 invocation (L=0x" + L.ToString("X5")
                            + ", PC=0x" + va.ToString("X5") + ", MDS base=0x" + mds.ToString("X5") + ") ===");
                        for (int depth = 0; depth < 10 && (L & 0xFFFF) >= 4; depth++)
                        {
                            int fsi = ReadWord(L - 4), ret = ReadWord(L - 3), gl = ReadWord(L - 2), pc = ReadWord(L - 1);
                            // Global frame (module) from the globallink; its codebase pointer names the module.
                            int gf = mds | (gl & 0xFFFF);
                            int cbLo = ReadWord(gf - 2), cbHi = ReadWord(gf - 1);   // GF-relative codebase (GFT-style [gf,cb])
                            FrameChainLog.Add("  [" + depth + "] L=0x" + L.ToString("X5")
                                + "  fsi/LW=0x" + fsi.ToString("X4") + "  returnlink=0x" + ret.ToString("X4")
                                + "  GF=0x" + gf.ToString("X5") + "(gl=0x" + gl.ToString("X4") + ")"
                                + "  codebase~[0x" + cbHi.ToString("X4") + ":" + cbLo.ToString("X4") + "]  pc=0x" + pc.ToString("X4"));
                            if (ret == 0 || (ret & 1) != 0) break;                 // NIL / non-frame (proc-desc) link
                            int next = mds | (ret & 0xFFFF);
                            if (next == L) break;                                  // self-loop
                            L = next;                                              // caller frame (MDS-relative link)
                        }
                    }
                }
            }

            // Overall rotation census -- the lane-picker (see CycleHist).  Counted for EVERY microword.
            CycleHist[_cycle & 3]++;
            // The other two pinned macros, as free invariants: MDR<- is cy:c2, IBDisp is cy:c2.
            if (mi.mem && !mi.LoadMap) MdrByCycle[_cycle & 3]++;

            // Tally LoadMap microwords by the cycle they land in (see LoadMapByCycle), and log the
            // FIRST offences.  The DLion reference asserts this is impossible -- CentralProcessor.cs
            // case 2/case 3 both `throw new InvalidOperationException("Map<- in c2"/"c3")`.  The Dove
            // port dropped that assertion, so the condition now happens ~992,007 times per boot and is
            // silently turned into an unconditional MDR<- wild store to a stale MAR (MarMapMDR =
            // mem||LoadMap lets a mem=0 Map<- word into the block, and case 2 writes without checking
            // mi.mem).  Log, do NOT throw: the first offence is what we want, not a crash at offence
            // one million.  (Same reasoning as the Table 2.11 detector above.)
            if (mi.LoadMap)
            {
                LoadMapByCycle[_cycle & 3]++;
                if (_cycle != 1 && MapPhaseLog != null && MapPhaseLog.Count < 40)
                    MapPhaseLog.Add("*** Map<- IN c" + _cycle + " (DLion throws here) @" + addr.ToString("X3")
                        + " CPi=" + InstructionCount + "  mem=" + (mi.mem ? 1 : 0)
                        + " rB=" + mi.rB + " RH" + mi.rB + "=" + _rh[mi.rB].ToString("X2")
                        + " Y=" + _yBus.ToString("X4")
                        + "  -> would MDR<- stale MAR " + _mar.ToString("X5") + " = " + _yBus.ToString("X4")
                        + "   " + mi.Disassemble(-1));
            }

            // ---- MAR<- / Map<- / MDR<- (Y bus + YH=_rh[rB]) ----
            if (mi.MarMapMDR)
            {
                switch (_cycle)
                {
                    case 1:
                        if (mi.LoadMap)
                        {
                            // Map<- is its own c1 reference, directed at the MAP ARRAY rather
                            // than main memory: VA = (RH[rB]<<16)|Ybus (24-bit), vpage = VA>>8,
                            // and the cycle addresses real word (MAPA<<16) + vpage.  The c2
                            // MDR<- then WRITES that entry and c3 <-MD READS it; the microcode
                            // itself decodes the split entry and issues a real MAR<-.
                            // (MAR<- never translates -- the boot path relies on that.)
                            int va = (_rh[mi.rB] << 16) | _yBus;
                            _mar = ((_mapA & 0xF) << 16) | ((va >> 8) & 0xFFFF);
                            _lastRefWasMap = true;   // OQ52: this c1 ref is a Map<-(translate) -> <-MD returns the decoded real page
                            int _vpg = (va >> 8) & 0xFFFF;
                            if (LgcLog != null && _vpg <= 0x1FF && LgcLog.Count < 400 && ReadWord != null)
                            {
                                int mw = ReadWord(0x40000 + _vpg); int rp = ((mw & 0x1F) << 8) | (mw >> 8);
                                LgcLog.Add("Map<- va=" + va.ToString("X6") + " (RH" + mi.rB + "=" + _rh[mi.rB].ToString("X2") + " Y=" + _yBus.ToString("X4") + ") vp=0x" + _vpg.ToString("X3") + " -> mapword " + mw.ToString("X4") + " -> real 0x" + rp.ToString("X3") + " @" + addr.ToString("X3") + " CPi=" + InstructionCount);
                            }
                            // OQ56: catch EVERY aGMF (GetState) @8E5 for all vpages, tagged with call-site R5/RH5:
                            // distinguishes FindStart's GetState[0xFF] (baked arg 0xFF -> vp 0xFF) from
                            // GetRealPage[PageFromLongPointer] (computed arg -> vp 0x100). Which sites & which pages?
                            if (MapReadLog != null && (addr == 0x8E5 || (_vpg >= 0x0C0 && _vpg <= 0x110)) && MapReadLog.Count < 250)
                                MapReadLog.Add("Map<- @" + addr.ToString("X3") + (addr == 0x8E5 ? "(aGMF)" : "") + " callsite R5=" + _alu.R[5].ToString("X4") + " RH5=" + _rh[5].ToString("X") + " vpage=0x" + _vpg.ToString("X3") + " -> MAR=" + _mar.ToString("X5")
                                    + " | va=" + va.ToString("X6") + " rh[" + mi.rB + "]=" + _rh[mi.rB].ToString("X2") + " Ybus=" + _yBus.ToString("X4") + " Rold=" + _bOld.ToString("X4") + " CPi=" + InstructionCount);
                            // THE MAP-SPIN PROBE: every Map<- once armed, with the base and the
                            // resolved entry.  Answers the operator's question directly --
                            // is _mar inside the initialized map array, and is _mapA still 4?
                            if (SpinMapLog != null && InstructionCount >= SpinMapFrom && SpinMapLog.Count < 60)
                            {
                                int mw = ReadWord != null ? ReadWord(_mar) : -1;
                                SpinMapLog.Add("Map<- @" + addr.ToString("X3") + " CPi=" + InstructionCount
                                    + "  MAPA=" + _mapA.ToString("X") + " -> base 0x" + ((_mapA & 0xF) << 16).ToString("X5")
                                    + "  va=" + va.ToString("X6") + " (RH" + mi.rB + "=" + _rh[mi.rB].ToString("X2")
                                    + " Y=" + _yBus.ToString("X4") + ")  vp=0x" + _vpg.ToString("X4")
                                    + "  MAR=" + _mar.ToString("X5")
                                    + "  mapword=" + (mw < 0 ? "??" : mw.ToString("X4"))
                                    + "  Q=" + _alu.Q.ToString("X4") + " R6=" + _alu.R[6].ToString("X4")
                                    + " RH2=" + _rh[2].ToString("X2"));
                            }
                            MapWatchRead(_vpg, addr);
                            Note("Map<-");
                        }
                        else
                        {
                            // Daybreak MAR<- (real ref): the address is the map-decode SPLICE (baseline) --
                            //   bits 16-19 <- RH[rB] (rpHigh);  bits 8-15 <- R[rB]_old high byte (rpLow);
                            //   bits  0- 7 <- Ybus low byte (offset), no carry across byte 8.
                            // aF|3 => RorS (aF 0-3 => R[rB]) or notRxorS (aF 4-7 => ~R[rB]).
                            ushort hi = (((int)mi.aF | 0x3) == (int)AluFunction.RorS)
                                ? (ushort)(_bOld & 0xff00)
                                : (ushort)((~_bOld) & 0xff00);
                            _marLowSplice = (ushort)((_yBus & 0x00ff) | hi);
                            // rpHigh is FIVE bits, not four.  The map entry is |rp[5-12]|r|d|w|rp[0-4]|,
                            // decoding as realPage = ((w & 0x1F) << 8) | (w >> 8) -- 13 bits, which is
                            // exactly the 8192 pages of a 4 MB machine.  Masking RH to 0xF here built a
                            // 12-bit page number, so every real page >= 0x1000 aliased 4096 pages down:
                            // 0x11EE was read as 0x01EE.  MAR is a 21-bit WORD address (2M words = 4 MB),
                            // so bits 16-20 are all available and nothing else needs widening.
                            //
                            // Nothing hit it until now because every earlier boot lived in low memory.
                            // The germ, and a fully boot-loaded image like the Utility Pilot, sit well
                            // under real page 0x1000; only once full Pilot came up with demand paging did
                            // allocation push pages above 4096.  BWSDove's StartList landed at real
                            // 0x11EE holding 6303, the map entry correctly said 0x11EE, and the splice
                            // fetched 0x01EE -- zeros -- so PilotControl:192 compared 0 against
                            // StartList.VersionID and died with MP 0934 (cBadBootFile).
                            _mar = ((_rh[mi.rB] & 0x1f) << 16) | _marLowSplice;
                            // ---- RefillE->RefillNE two-word gap fix ----
                            // The empty-buffer refill fetches TWO words: RefillE (@400) fetches word[PC], then falls
                            // through to RefillNE (@500) for word[PC+1].  RefillE's aD=2 register-capture is the
                            // prefetch convention -- it leaves R5 one word ahead (R5 = PC+1) rather than PC-1 -- and
                            // is load-bearing (removing it collapses the germ at the very first refill).  But that +1
                            // is ALSO applied by RefillNE's own `MAR<-[rhPC, PC+1]` (Ybus = R5+1), so the second fetch
                            // lands at PC+2 and SKIPS the middle code word.  Usually harmless, but when that middle
                            // word is real 0x4B1D7 = C1 F8 73 (WriteWDC + the sCodeTrap install) the germ mis-decodes,
                            // SD[7] stays null, and it wedges in the CodeTrap->ControlTrap loop at MP 0900.  Fix: in the
                            // RefillE->RefillNE path ONLY, fetch word[R5] (the +1 is already baked into R5), not
                            // word[R5+1].  Standalone RefillNE (no preceding RefillE) is untouched -- it keeps Ybus.
                            // word[R5] on ALL RefillNE (@500), not RefillE-only: under the one-word-ahead R5
                            // (the @400 prefetch +1), word[R5] == the hardware's word[PC+1] = the next word after
                            // the dispatch, so every RefillNE tops the IB with the CONSECUTIVE word.  The former
                            // fetch of word[R5+1] (=_yBus) lands one word too far and skips (D7, then D8 after a
                            // word-consuming 2-byte ESC).  off=0xFFFF was a measurement artifact (R5 committed at
                            // L487 before the IBDisp); the real defect is this two-ahead RefillNE fetch.
                            _refillEPending = (addr == 0x400);   // (retained; no longer gates the fetch)
                            _lastRefWasMap = false;   // this c1 ref is a MAR<-(real) -> <-MD returns the raw memory word
                            if (MarAccess != null && MarAccess.Count < 4000)
                            { long mc; MarAccess.TryGetValue(_mar, out mc); MarAccess[_mar] = mc + 1; }
                            if (mi.mem && (_alu.PgCarry ^ (((int)mi.aF & 0x1) == 1)))
                            {
                                _niaModifier |= 0x2;                     // pageCross branch
                                // DETECTOR ONLY (does not yet cancel).  The DLion reference ALSO latches
                                // `_marPageCrossBr = true` here (CentralProcessor.cs:650) and uses it on the
                                // NEXT instruction as `pageCrossCancel` to CANCEL a pending IBDisp
                                // (":759  // This is canceled if the last memory operation resulted in a page
                                // cross.  if (!pageCrossCancel)") and a pending MDR<- (":664").  The Dove port
                                // has the pageCross BRANCH but neither the latch nor either cancel.
                                // TechRef: pageCross (MAR<-'s 2901) or IB-Empty cancels a pending
                                // IBDisp/IB-Refill -- ibPtr unaffected, INIA unmodified => control lands on
                                // dispatch-table entry 0.  A cancel that should fire and doesn't changes the
                                // path length => a permanent phase shift.  Measure first, wire it after.
                                _marPageCrossBr = true;
                                _pageCrossCount++;
                            }
                            // Address-capture (aD=0/3 ONLY): the register/Q receives the map-translated real
                            // address (the splice) -- the germ's translation-extraction write.  aD=2 (RAMA, the
                            // RefillE @400 microword) is NOT captured: the register keeps the ALU F (=PC-1), which
                            // the following @208/@500 restores to PC (net-0, the ISA-correct macro-PC).  Capturing
                            // the splice for aD=2 was the "crutch" that over-advanced R5 by one word and corrupted
                            // the entire mesa decode (RefillNE then skipped operand bytes from dispatch #1);
                            // offC!=0 was its signature.  Deleting it restores offC=0 across every dispatch.
                            int fAluTrue = (mi.aD == 0) ? _alu.Q : _alu.R[mi.rB];   // (diagnostic) the ALU output, before the capture
                            if (mi.aD == 0) _alu.Q = _marLowSplice;
                            else if (mi.aD == 3) _alu.R[mi.rB] = _marLowSplice;
                            // XFER-CHAIN TRACE (OQ52): dump the last N register-writing MAR<- microwords with every
                            // input the reconciliation needs -- old-rB, Ybus, ALU F, the map-decoded real page for
                            // this rh, the splice, and what each candidate model would write to the dest register.
                            // Bounded window (XferTraceFrom..+300) so a low XferTraceFrom captures the RefillE region
                            // (CPi ~1458) without the 10.4M wedge loop rolling it out of the buffer.
                            if (XferChainLog != null && InstructionCount > XferTraceFrom && InstructionCount < XferTraceFrom + 700)
                            {
                                if (XferChainLog.Count > 4000) XferChainLog.RemoveRange(0, 300);   // rolling, amortized O(1)
                                int fAlu = fAluTrue;
                                XferChainLog.Add(
                                    "@" + addr.ToString("X3") + " CPi=" + InstructionCount +
                                    " aD=" + mi.aD + " aF=" + ((int)mi.aF).ToString("X") + " rB=" + mi.rB +
                                    " rh[rB]=" + _rh[mi.rB].ToString("X2") + " Rold=" + _bOld.ToString("X4") +
                                    " Y=" + _yBus.ToString("X4") + " F=" + fAlu.ToString("X4") +
                                    " splice=" + _marLowSplice.ToString("X4") + " MAR=" + _mar.ToString("X5") +
                                    " | cap(splice)=" + _marLowSplice.ToString("X4") +
                                    " plainF=" + fAlu.ToString("X4") +
                                    " split(hi|Flo)=" + (hi | (fAlu & 0xff)).ToString("X4"));
                            }
                        }
                        break;
                    case 2:
                        // MDR<- -- DLion CentralProcessor.cs:664: cancel the store if the PRECEDING MAR<- crossed a
                        // page (the un-carried splice address is wrong; the pageCross branch re-issues it correctly).
                        if (_ioRefPending) { Pit8254Write(_yBus); _ioRefPending = false; }
                        else if (WriteWord != null && !(_pageCrossCancelPending && _pageCrossMdrCancel))
                        {
                            MapWatch(_mar, _yBus);
                            WriteWord(_mar, _yBus);
                        }
                        // StashPC watch (env DOVE_STASHWATCH): catch the write that saves GFI 99's PC.
                        if (StashWatchLog != null && StashWatchLog.Count < 2000
                            && InstructionCount >= StashWatchFrom && InstructionCount < StashWatchTo
                            && (_mar == StashWatchAddr || _yBus == StashWatchVal))
                        {
                            int Lr = ((_rh[3] & 0x1F) << 16) | _alu.R[3];
                            StashWatchLog.Add("CPi=" + InstructionCount + " @" + addr.ToString("X3")
                                + (_mar == StashWatchAddr ? " [ADDR]" : "") + (_yBus == StashWatchVal ? " [VAL]" : "")
                                + " mar=0x" + _mar.ToString("X5") + " <-0x" + _yBus.ToString("X4")
                                + " R5=" + _alu.R[5].ToString("X4") + " RH5=" + _rh[5].ToString("X")
                                + " pc16=" + (_pc16 ? 1 : 0)
                                + " R2=" + _alu.R[2].ToString("X4") + " R6=" + _alu.R[6].ToString("X4")
                                + " Q=" + _alu.Q.ToString("X4") + " L=0x" + Lr.ToString("X5"));
                        }
                        if (LinkVecWriteLog != null && LinkVecWriteLog.Count < 200)
                        {
                            // DECISIVE (OQ48): trace the SD-install burst (GermOpsImpl:1099-1102).  SD[sCodeTrap]=SD[7]@real0x4820E
                            // is NULL while siblings SD[6]@0x4820C / SD[3]@0x48206 are bound.  Does 0x4820E ever get a store,
                            // and with what value?  Capture real 0x48200-0x48230 (SD table + margin to catch mis-addressed stores).
                            if (_mar >= 0x48200 && _mar <= 0x48230)
                                LinkVecWriteLog.Add("SD-W real " + _mar.ToString("X5") + " = " + _yBus.ToString("X4") + " @" + addr.ToString("X3")
                                    + " R0=" + _alu.R[0].ToString("X4") + " R1=" + _alu.R[1].ToString("X4") + " R2=" + _alu.R[2].ToString("X4") + " R3=" + _alu.R[3].ToString("X4") + " Q=" + _alu.Q.ToString("X4") + " CPi=" + InstructionCount);
                        }
                        // TEMP: watch the FindStartOfIORegion write of the IORegion LONG POINTER (LongPointerFromPage[base]).
                        // base 0xC0 -> 0xC000 (correct), base 0xE2 -> 0xE200 (wrong). Also catch page forms 0x00C0/0x00E2.
                        // Watch every page-pointer store 0x??00 with high byte 0x80..0xFF (catches IORegion 0x9F00/0xC000/0xE200) — any value, any address, both banks.
                        if (IORgnWriteLog != null && IORgnWriteLog.Count < 70
                            && (_yBus & 0x00FF) == 0 && (_yBus >> 8) >= 0x80 && (_yBus >> 8) <= 0xFF)
                            IORgnWriteLog.Add("PGPTR [bank" + _execBank + "] store word " + _mar.ToString("X5") + " = " + _yBus.ToString("X4") + " @" + addr.ToString("X3") + " CPi=" + InstructionCount);
                        if (WriteLog != null && !_writeFrozen)
                        {
                            WriteLog.Add("@" + addr.ToString("X3") + " CPi=" + InstructionCount + " MDR<- word " + _mar.ToString("X5") + " = " + _yBus.ToString("X4"));
                            if (WriteLog.Count > 48) WriteLog.RemoveAt(0);
                        }
                        if (SemaLog != null && _mar == SemaWord && SemaLog.Count < 70)
                            SemaLog.Add("@" + addr.ToString("X3") + " MDR<- word " + _mar.ToString("X5") + " <- Ybus " + _yBus.ToString("X4")
                                + " (R0/TOS=" + _alu.R[0].ToString("X4") + ") CPi=" + InstructionCount + "  " + mi.Disassemble(-1));
                        break;
                }
            }

            // ---- Late L-rotate (X bus <- rotated Y bus) ----
            if (mi.LateLRotN)
            {
                switch ((ZNormFunction)mi.fZ)
                {
                    case ZNormFunction.LRot0: _xBus = _yBus; break;
                    case ZNormFunction.LRot12: _xBus = (ushort)((_yBus << 12) | (_yBus >> 4)); break;
                    case ZNormFunction.LRot8: _xBus = (ushort)((_yBus << 8) | (_yBus >> 8)); break;
                    case ZNormFunction.LRot4: _xBus = (ushort)((_yBus << 4) | (_yBus >> 12)); break;
                }
            }

            // ---- fX: LoadRH ----
            if (mi.fX == XFunction.LoadRH)
            {
                _rh[mi.rB] = (byte)_xBus;
                if (RhLog != null && RhLog.Count < 60)
                    RhLog.Add("@" + addr.ToString("X3") + " RH" + mi.rB + "<-" + ((byte)_xBus).ToString("X2") + " (xBus=" + _xBus.ToString("X4") + " mem=" + (mi.mem ? 1 : 0) + ")");
            }

            // ---- fY = DispBr: conditional branches + dispatches modify the NEXT NIA ----
            if (mi.fSfY == FunctionSelectFY.DispBr)
            {
                switch ((YDispBrFunction)mi.fY)
                {
                    case YDispBrFunction.NegBr: if (_alu.Neg) _niaModifier |= 1; break;
                    case YDispBrFunction.ZeroBr: if (_alu.Zero) _niaModifier |= 1; break;
                    case YDispBrFunction.NZeroBr: if (!_alu.Zero) _niaModifier |= 1; break;
                    case YDispBrFunction.MesaIntBr:
                        // An interrupt is TAKEN only when enabled (IE).  With IE off the reason
                        // stays latched (_mInt/_timerInt) but the branch is not taken -- the germ
                        // runs its interrupt-masked setup (incl. @BLTL) to completion, reaches @EI
                        // (uWDC 1->0 -> SetIE), enables IE, and only then idles/schedules.  Without
                        // this gate we steal the timer mid-setup and dump the CP into a premature idle.
                        if ((_mInt || _timerInt) && _ie) { _niaModifier |= 1; MesaIntBrFires++; }
                        break;
                    case YDispBrFunction.PgCarryBr: if (_alu.PgCarry) _niaModifier |= 1; break;
                    case YDispBrFunction.CarryBr: if (_alu.CarryOut) _niaModifier |= 1; break;
                    case YDispBrFunction.XRefBr:
                        // Map-fault gate: dispatch on the map entry's REFERENCED bit.  On Daybreak the
                        // hardware map word carries flags at bits 5-7 (writeProtect 0x20, dirty 0x40,
                        // referenced 0x80) -- NOT the DLion's X[11]/0x10.  The MapFix ORs referenced in
                        // (`MDR <- Rx or map.referenced`) then retries + re-checks here; testing 0x10
                        // (mis-ported from the DLion) never cleared for an entry with referenced already
                        // set (e.g. 0x80C5), so the germ's boot-file inload page-walk MapFix looped
                        // forever on the same page.  Test 0x0080 so the OK/no-fault branch is taken when
                        // referenced is present.  (XwdDisp already tests dirty 0x40/writeProtect 0x20.)
                        _niaModifier |= (_xBus & 0x80) >> 7;
                        break;
                    case YDispBrFunction.NibCarryBr: if (_alu.NibCarry) _niaModifier |= 1; break;
                    case YDispBrFunction.XDisp:
                        // XDisp contributes to THIS microword's next address, not the one after.
                        //
                        // _niaModifier is latched at the top of the instruction and applied a
                        // microword late, which is right for the condition branches above (the
                        // ALU result they test is not settled until late in the click).  It is
                        // wrong for XDisp: the X bus is already valid when the dispatch is
                        // decoded, and the microcode places the targets relative to the SAME
                        // word's INIA.  Measured at 18F -- 'XDisp LRot12 Xbus<- R2', rawINIA 00F
                        // so trueINIA 000 -- with xBus=2004 the nibble is 4 and the healthy boot
                        // path goes to 004, but the deferred modifier left mod=000 and nia=000,
                        // which is ErrTrap.  Nothing had trapped; a dispatch simply evaluated to
                        // zero, and address 0 happens to be the trap vector -- indistinguishable
                        // from a real trap except by click phase (a trap can only arrive in c1;
                        // this arrived in c2).
                        //
                        // DOVE_XDISP_LATE restores the old deferred behaviour.
                        if (XDispSameWord) _xDispNow |= (_xBus & 0xf);
                        else _niaModifier |= (_xBus & 0xf);
                        break;
                    case YDispBrFunction.YDisp: _niaModifier |= (_yBus & 0xf); break;
                    case YDispBrFunction.XC2npcDisp:
                        _niaModifier |= (_xBus & 0xc) | (_cycle == 2 ? 0x2 : 0x0) | (_pc16 ? 0x0 : 0x1);
                        break;
                    case YDispBrFunction.YIODisp:
                        // fY 0xB: on the DAYBREAK this slot is XWtOKDisp -- the map WRITE-permission
                        // gate (TmMacroTablesDaybreak: XWtOKDisp=11) -- NOT the DLion's YIODisp, which
                        // reused the same fY code (reused fY slots collide across machines; the port
                        // kept the DLion meaning).  It feeds BRANCH[upDMap, DMapOK, 0D]: the write
                        // proceeds (DMapOK) iff the map entry is dirty (0x40) AND not writeProtect
                        // (0x20); referenced was already guaranteed by XRefBr running first.  Every
                        // other state -- dirty clear (needs set-dirty) or writeProtect set (protection)
                        // -- faults to upDMap.  Running this as YIODisp computed a garbage niaModifier
                        // off the Y bus, so a write to an already-dirty writable page (e.g. 0x80C5) took
                        // the fault branch, fell into the MapFix set-dirty retry loop, and the germ's
                        // boot-file inload page-walk never advanced past cGerm 0900.
                        // The OK condition drives NIA BIT 1, not bit 0.  TechRef 2.3.3.1: "A particular
                        // condition bit is ignored when its corresponding position in INIA equals 1" -- so
                        // BRANCH[false, true, mask]'s third operand IS the INIA low nibble, and the bits it
                        // SETS are the cancelled ones.  Both XWtOKDisp sites use mask 0D = 1101, leaving only
                        // bit 1 (0x2) free:
                        //   ProcListXferDaybreak XFStartRead: BRANCH[XFMUD, XFMOK, 0D]  -- XFMUD @0x?AD, XFMOK @0x?AF
                        //   LoadStore.mc:389    @SGB c3:      BRANCH[SGa,   SGb,   0D]
                        // With `|= 1` the OK bit landed on a position INIA already had set (0xD = 1101), so it
                        // was cancelled by definition and BOTH arms resolved to the same address -- the branch
                        // could never select.  XFER therefore always took XFMUD -> WMapFix, skipping
                        // `MAR <- L <- [rhL, Q-LF.pc] {fix L}`, so L exited XFER still holding the RAW MAP WORD
                        // from `L <- MD, rhL <- MD` (0x89C4: high byte 0x89 = real page, low byte 0xC4 = the map
                        // flag byte) instead of the link's offset (0x89D0).  The germ then ran ~12,000 microinstrs
                        // on a pseudo-frame at 0x489C4 that AllocSub never allocated -- reads/writes silently
                        // "worked" because the address lands inside frame page 0x489 -- until Start's zRET read
                        // that non-frame's header (fsi/returnlink at L-4/L-3 = virgin zeros), XFERed through a
                        // null link, and hit `[] <- Q, ZeroBr, BRANCH[$, ControlTrap]` -> GermWorldError[902].
                        // TechRef Table 2.8: XWtOKDisp = 1,1,(X.08 ^ X.09 ^ X.10'),0 -> INIA[8-11].
                        //   X.08 = ref (0x80), X.09 = dirty (0x40), X.10' = NOT writeProtect (0x20).
                        // The constant 0xC is cancelled at every mask-0D site (INIA low nibble 0xD has bits
                        // 0x4/0x8 set), which is exactly why all 31 XWtOKDisp use sites show only bit 0x2 live.
                        // It is therefore inert wherever the microcode actually uses this function; included
                        // for fidelity to the table.  (The `ref` term was missing: it agreed on map word
                        // 0x89C4 only because ref happened to be set, and would diverge on a dirty-but-
                        // unreferenced page.  If this ever regresses, trust the measured 31/31 over the table.)
                        _niaModifier |= 0xC | (((_xBus & 0x80) != 0 && (_xBus & 0x40) != 0 && (_xBus & 0x20) == 0) ? 0x2 : 0);
                        if (XWtLog != null && InstructionCount >= 1250 && InstructionCount <= 1400)
                            XWtLog.Add("  ** XWtOKDisp(fY=0xB) @" + addr.ToString("X3") + " CPi=" + InstructionCount
                                + " X=" + _xBus.ToString("X4") + " dirty(0x40)=" + ((_xBus & 0x40) != 0 ? 1 : 0) + " wp(0x20)=" + ((_xBus & 0x20) != 0 ? 1 : 0)
                                + " -> OK/DMapOK=" + (((_xBus & 0x40) != 0 && (_xBus & 0x20) == 0) ? 1 : 0) + " niaMod=" + _niaModifier.ToString("X3")
                                + " INIA=" + mi.INIA.ToString("X3") + " trueNIA=" + (mi.INIA ^ 0xFFF).ToString("X3"));
                        if (TrapLog != null && InstructionCount > 10400000 && TrapLog.Count < 24)
                            TrapLog.Add("XWtOKDisp @" + addr.ToString("X3") + " X=" + _xBus.ToString("X4") +
                                        " INIA=" + mi.INIA.ToString("X3") + " trueINIA=" + (mi.INIA ^ 0x00F).ToString("X3") +
                                        " (base low nib=" + ((mi.INIA ^ 0x00F) & 0xF).ToString("X") + ", mask0D free bits=" +
                                        (0x0D & ~((mi.INIA ^ 0x00F) & 0xF)).ToString("X") + ") OK=" + ((_xBus&0x40)!=0 && (_xBus&0x20)==0));
                        break;
                    case YDispBrFunction.XwdDisp:
                        _niaModifier |= (_xBus & 0x60) >> 5;
                        if (TrapLog != null && InstructionCount > 10400000 && TrapLog.Count < 30)
                            TrapLog.Add("XwdDisp @" + addr.ToString("X3") + " X=" + _xBus.ToString("X4") + " &0x60=" + (_xBus&0x60).ToString("X2") + " CPi=" + InstructionCount);
                        break;
                    case YDispBrFunction.XHDisp: _niaModifier |= ((_xBus & 0x8000) >> 15) | ((_xBus & 0x0800) >> 10); break;
                    case YDispBrFunction.XLDisp: _niaModifier |= (_xBus & 0x1) | ((_xBus & 0x80) >> 6); break;
                    case YDispBrFunction.PgCrOvDisp:
                        _niaModifier |= (_alu.PgCarry ^ (((int)mi.aF & 0x1) == 1) ? 0x2 : 0x0) | (_alu.Overflow ? 0x1 : 0x0);
                        break;
                }
            }

            // ---- fY functions (Daybreak REASSIGNED the DLion fY codes; do NOT use the
            //      DLion YNormFunction names.  See dove-cp-microcode-decode.) ----
            if (mi.fSfY == FunctionSelectFY.fyNorm)
            {
                switch (mi.fY)
                {
                    case 0x0:   // ClrMPIntIOP: lower ONLY the CP-side doorbell line (interface spec §3/§8).
                                // @NOTIFYIOP is a Set->Clr pulse; the Clr just returns the line low so the
                                // NEXT SetMPIntIOP re-edges.  It must NOT clear the IOP-side IR5 latch --
                                // that latch is cleared solely by the IOP's IN 0xB0.  (Clearing IR5 here
                                // kills the germ's own doorbell before the IOP ISR reads it = §8 pitfall #1.)
                        MesaInterruptRequest = false;
                        Note("ClrMPIntIOP");
                        break;
                    case 0xC:   // IO<- (Daybreak fYNorm 0xC).  TechRef fY table renders it "IO-"; the
                                // MDC decodes it into IORef, routing this reference to Mesa-bus I/O
                                // (the 8254 among them) instead of DRAM.  My decoder previously had
                                // 0xC = ClrDPRq (a DLion name) and swallowed the reference silently,
                                // so aRRIT's reads fell through to DRAM and returned zeros.
                        if (Io8254Enabled) { _ioRefPending = true; _ioPort = _rh[mi.rB] & 0xFF; IoRefCount++; }
                        Note("IO<-");
                        break;
                    case 0x2:   // ClrIntErr: clear the error/trap latch (the trap code the germ
                                // reads via <-IntStat X[8-9] / <-ErrnIBnStkP).  Was a no-op, which
                                // left the InitTrap code (1) stuck set forever -> phantom trap.
                        _trapCode = 0;
                        Note("ClrIntErr");
                        break;
                    case 0xE:   // ClrIE: disable interrupts (Daybreak fYNorm 0xE, TmMacroTables:274).
                        if (IeLog != null && InstructionCount >= IeFrom && InstructionCount < IeTo && IeLog.Count < 600)
                            IeLog.Add("ClrIE  CPi=" + InstructionCount + " @" + addr.ToString("X3")
                                + "  IE " + (_ie ? 1 : 0) + " -> 0"
                                + "  R5=" + _alu.R[5].ToString("X4") + " RH5=" + _rh[5].ToString("X2"));
                        _ie = false;
                        Note("ClrIE");
                        break;
                    case 0xF:   // SetIE: enable interrupts (Daybreak fYNorm 0xF, TmMacroTables:276).
                        if (IeLog != null && InstructionCount >= IeFrom && InstructionCount < IeTo && IeLog.Count < 600)
                            IeLog.Add("SetIE  CPi=" + InstructionCount + " @" + addr.ToString("X3")
                                + "  IE " + (_ie ? 1 : 0) + " -> 1"
                                + "  R5=" + _alu.R[5].ToString("X4") + " RH5=" + _rh[5].ToString("X2"));
                        _ie = true; SetIeCount++;
                        Note("SetIE");
                        break;
                    case 0x1:   // SetMPIntIOP: raise the CP->IOP interrupt (init-done doorbell)
                        if (!MesaInterruptRequest && OnMesaInterrupt != null) OnMesaInterrupt();
                        MesaInterruptRequest = true;
                        Note("SetMPIntIOP");
                        break;
                    case 0xA:   // MAPA<- : set the map-array base to real word (n << 16).
                                // Takes its value from the X BUS, not the Y bus
                                // (TmMacroTablesDaybreak:395, "6-Sep-84 JoM" bug fix).
                                // Pinned empirically: @050 fY=A xBus=0004 == MAPA<-4.
                        _mapA = _xBus & 0xF;
                        Note("MAPA<-");
                        break;
                    case 0xD:   // Bank<- : load the fetch-bank latch from Y[12-15], 1 delay slot
                        {
                            int code = (_yBus >> 12) & 0xf;   // one-hot chip-select nibble
                            _bankTarget = (code & 8) == 0 ? 0 : ((code & 4) != 0 ? 1 : ((code & 2) != 0 ? 2 : 3));
                            _bankChangePending = 2;           // N writes; N+1 old bank; N+2 new
                            Note("Bank<-");
                        }
                        break;
                    case 0x3:   // IBDisp: dispatch on the mesa opcode in ibFront (mesa-core, DLion-identical).
                        {
                            // ---- pageCross CANCEL (DLion CentralProcessor.cs:759) ----
                            // "This is canceled if the last memory operation resulted in a page cross."
                            // TechRef: pageCross (MAR<-'s 2901) or IB-Empty cancels a pending IBDisp/IB-Refill;
                            // ibPtr is unaffected and INIA is unmodified, so control lands on dispatch-table
                            // entry 0 instead of on the opcode's entry.  Dove had the pageCross BRANCH
                            // (_niaModifier |= 0x2) but not this cancel, so a dispatch the hardware suppresses
                            // was taken anyway -- once per boot, at CPi 15337, on the AlwaysIBDisp right after
                            // `MAR<- R5+1` with R5=0xB1FF (the last word of page 0xB1).  From that instant the
                            // germ ran a path the microcode never intended: Map<- (pinned cy:c1 by the
                            // assembler) started landing in c2, where the map reference is silently dropped
                            // -- 992,007 of them, c3=0, i.e. one clean divergence rather than drift -- and each
                            // became a wild store to a stale MAR.  Downstream: ControlTrap -> GermWorldError ->
                            // the 247,891-iteration map set-ref spin.
                            if (_pageCrossCancelPending)
                            {
                                _ibDispCancels++;
                                if (CancelLog != null && CancelLog.Count < 40)
                                    CancelLog.Add("*** IBDisp CANCELLED by pageCross @" + addr.ToString("X3")
                                        + " c" + _cycle + " CPi=" + InstructionCount
                                        + "  ibPtr=" + _ibPtr + " ibFront=" + _ibFront.ToString("X2")
                                        + " R5=" + _alu.R[5].ToString("X4") + " RH5=" + _rh[5].ToString("X")
                                        + "  (INIA unmodified, ibPtr unaffected -> dispatch-table entry 0)");
                                Note("IBDisp-cancelled");
                                break;
                            }
                            // AlwaysIBDisp = IBDisp + IBPtr<-1 (fZ=1): a non-trapping dispatch.
                            bool alwaysIBDisp = (mi.fSfZ == FunctionSelectFZ.fzNorm && mi.fZ == 0x1);
                            // IE-GATE THE INTERRUPT DIVERT (Daybreak spec wlrqnvtff, invariants T-8/T-16/T-17,
                            // TechRef tr2:1534-1544,4329-4331): IE is ONE gate with TWO consumers -- MesaIntBr
                            // (line ~951) AND this IBDisp IB-Refill interrupt trap.  Both must divert on the SAME
                            // condition: a pending interrupt (IOP `_mInt`=IntStat.1 OR timer `_timerInt`=IntStat.2)
                            // AND `_ie`.  The bug was this trap firing on raw `_mInt` (ungated) while `_mInt` is set
                            // unconditionally (line ~279) and the 8254 free-runs from BootTrap step 3 (before ClrIE
                            // at step 5): an interrupts-off germ (imports no Process/Monitor) with a pending int was
                            // trapped to the interrupt vector on every dispatch -> dragged into the scheduler `Scan`
                            // forever.  IE off (ClrIE) => NO interrupt-triggered divert.  The IB-refill term
                            // (`_ibPtr != Full`) is NOT an interrupt and stays ungated (IntStat read is also ungated;
                            // only the DIVERT is IE-gated).
                            bool mIntDivert = (_mInt || _timerInt) && _ie;
                            if ((_ibPtr != IBState.Full || mIntDivert) && !alwaysIBDisp)
                            {
                                // IB not full (or Mesa int pending): trap to the refill/interrupt
                                // handler instead of dispatching.  INIA[0-3] (=nia[11-8]) replaced.
                                if (mIntDivert)
                                    _niaModifier |= (_ibPtr == IBState.Empty || _ibPtr == IBState.Full) ? 0x600 : 0x700;
                                else
                                    _niaModifier |= (_ibPtr == IBState.Empty) ? 0x400 : 0x500;
                                _niaModType = 2;   // IBRefillTrap
                                if (IbLog != null && InstructionCount >= IbLogFrom && InstructionCount < IbLogTo)
                                    IbLog.Add("@" + addr.ToString("X3") + " CPi=" + InstructionCount + " IBDisp TRAP ptr=" + _ibPtr + " -> refill vector 0x" + (_niaModifier & 0x700).ToString("X3") + (_ibPtr == IBState.Empty ? " (RefillE)" : " (RefillNE)"));
                            }
                            else
                            {
                                // Normal dispatch: ibFront replaces INIA[4-7] and ORs INIA[8-11].
                                // Log opcodes in the CPi window OR anywhere in Start's code page (RH5=4, R5 0xAE00-0xAEFF = real page 0x4AE)
                                bool inStartPage = (_rh[5] == 4 && _alu.R[5] >= 0xAE00 && _alu.R[5] < 0xAF00);
                                // NB: the old `|| inStartPage` OR-gate filled the 400-entry cap at CPi ~2900 and made
                                // every later window silently empty.  Window only; inStartPage is just a label now.
                                if (OpLog != null && OpLog.Count < 400 && InstructionCount >= OpLogFrom && InstructionCount < OpLogTo)
                                    OpLog.Add((inStartPage ? "[START] " : "") + "CPi=" + InstructionCount + " @" + addr.ToString("X3") + " OP=0x" + _ibFront.ToString("X2") + " R5=" + _alu.R[5].ToString("X4") + " RH5=" + _rh[5].ToString("X") + " pc16=" + (_pc16?1:0) + " ibPtr=" + _ibPtr + " fw=" + _ibFrontWord.ToString("X4") + " offC=" + ((_alu.R[5] - _ibFrontWord) & 0xFFFF).ToString("X4") + " ib=[" + _ib[0].ToString("X2") + "," + _ib[1].ToString("X2") + "] TOS=" + _alu.R[0].ToString("X4") + " sp=" + _stackP + " L=[" + _rh[3].ToString("X2") + ":" + _alu.R[3].ToString("X4") + "]->" + ((((_rh[3] & 0x1F) << 16) | _alu.R[3])).ToString("X5"));
                                    // offC = committed-R5 offset (sampled post-L487 ALU commit, unlike the top-of-Step off at L307
                                    // which reads R5 PRE-commit and manufactures the 0xFFFF word-crossing artifact -- see audit wf_2e3020ed).
                                // ENTERING-OPCODE PROBE: record every dispatched mesa opcode with the
                                // register inputs it was handed.  @AB0's IBDisp is flat (it's a microcode
                                // primitive that loops), so the LAST opcode dispatched before the run freezes
                                // in the @AB0 spin IS the opcode that entered the stuck primitive -- it names
                                // the primitive via the dispatch table, no symbol listing needed.  Capturing
                                // R5/RH5/R2/RH2/TOS at dispatch shows the INPUT the primitive walked from.
                                _lastDispOp = _ibFront;
                                _lastDispCPi = InstructionCount;
                                _lastDispAddr = addr;
                                _lastDispR5 = _alu.R[5]; _lastDispRH5 = _rh[5];
                                _lastDispR2 = _alu.R[2]; _lastDispRH2 = _rh[2];
                                _lastDispR3 = _alu.R[3]; _lastDispRH3 = _rh[3];
                                _lastDispTOS = _alu.R[0]; _lastDispSp = _stackP;
                                // MP-940: capture the ESC alpha for a zESC dispatch (byte after 0xF8 = _ib[ibPtr&1]).
                                _lastDispAlpha = (_ibFront == 0xF8) ? _ib[((int)_ibPtr) & 0x1] : -1;
                                _lastDispIb0 = _ib[0]; _lastDispIb1 = _ib[1]; _lastDispIbPtr = (int)_ibPtr;
                                // Dispatch ring: cheap int-array capture of (op + pending pair) at THIS dispatch,
                                // so a KFCB dump can replay the raise-site byte stream incl. straddled operands.
                                if (DispRingCPi != null && InstructionCount >= KfcbLogFrom && InstructionCount < KfcbLogTo)
                                {
                                    int di = DispRingPos & 63;
                                    DispRingCPi[di] = (int)InstructionCount;
                                    DispRingOp[di] = _ibFront;
                                    DispRingIb[di] = (_ib[0] << 8) | _ib[1];
                                    DispRingPtr[di] = (int)_ibPtr;
                                    DispRingR5[di] = _alu.R[5];
                                    DispRingRH5[di] = _rh[5];
                                    DispRingPos++;
                                }
                                if (DispTrOp != null && InstructionCount >= DispTrFrom && InstructionCount < DispTrTo
                                    && DispTrN < DispTrOp.Length)
                                {
                                    DispTrOp[DispTrN] = _ibFront; DispTrR5[DispTrN] = _alu.R[5];
                                    DispTrRH5[DispTrN] = _rh[5]; DispTrPc16[DispTrN] = _pc16 ? 1 : 0;
                                    // stack state feeding this dispatch -- for a conditional branch this IS its input
                                    DispTrTOS[DispTrN] = _alu.R[0]; DispTrSp[DispTrN] = _stackP;
                                    int sp1 = _stackP - 1, sp2 = _stackP - 2;
                                    DispTrU1[DispTrN] = (sp1 >= 0 && sp1 < _u.Length) ? _u[sp1] : -1;
                                    DispTrU2[DispTrN] = (sp2 >= 0 && sp2 < _u.Length) ? _u[sp2] : -1;
                                    // interrupt state feeding IBDisp: MesaIntBr = (_mInt||_timerInt) && _ie
                                    DispTrInt[DispTrN] = (_mInt ? 1 : 0) | (_timerInt ? 2 : 0) | (_ie ? 4 : 0)
                                                       | (((_mInt || _timerInt) && _ie) ? 8 : 0);
                                    DispTrMA[DispTrN] = (int)_mIntAsserts;
                                    DispTrCPi[DispTrN] = (int)InstructionCount;
                                    DispTrN++;
                                }
                                // Opcode-LENGTH census (env DOVE_OPLEN_FROM/TO): byte position of this dispatch is
                                // {R5,pc16} => 2*R5 + pc16.  The delta to the NEXT dispatch = how many IB bytes this
                                // opcode consumed.  Gives an empirical "opcode -> length" table to diff against the
                                // ISA: any opcode whose length is wrong (or bimodal) is a front-end decode bug.
                                if (OpLenHist != null && InstructionCount >= OpLenFrom && InstructionCount < OpLenTo)
                                {
                                    int pos = (_alu.R[5] << 1) | (_pc16 ? 1 : 0);
                                    if (_opLenPrevOp >= 0 && _rh[5] == _opLenPrevRH)
                                    {
                                        int d = pos - _opLenPrevPos;
                                        if (d >= 1 && d <= 4) OpLenHist[_opLenPrevOp, d]++;
                                        else if (d != 0) OpLenHist[_opLenPrevOp, 0]++;   // non-sequential (branch/xfer)
                                    }
                                    _opLenPrevOp = _ibFront; _opLenPrevPos = pos; _opLenPrevRH = _rh[5];
                                }
                                _ab0Run = 0;   // a dispatch happened -> reset the @AB0-burst counter
                                _niaModifier |= _ibFront;
                                _niaModType = 1;   // IBDispatch
                                // TEMP: BitBlt probe -- catch aBITBLT, the blt at the end of ProcessorHeadDove.Start.
                                // aBITBLT is reachable ONLY as zESC alpha 0x2B: BBInit.mc:23 places @BITBLT at
                                // at[0B,10,ESC2n], and Misc.mc:63/64 dispatch ESCHi on alpha's high nibble then
                                // ESC2n on its low nibble.  There is NO opcode[] for any BLT.
                                // The old `_ibFront == 0x76` arm was WRONG: 0x76 == 166'b == @SGDB
                                // (LoadStore.mc:404 `opcode[166'b]`), a Store-Global -- not "PILOTBITBLT".  It fired
                                // on every @SGDB dispatch and printed a "BITBLT ENTRY" banner with whatever stack
                                // state @SGDB happened to have (6 of 7 banners in a 17M run were phantoms).  Those
                                // phantom numbers are the likely source of the "TOS=0x0A06, sp=1 at the BitBlt
                                // probe" claim, which does not reproduce.  Match alpha only.
                                // Trace the setup so the bbTable geometry the microcode reads is visible:
                                // the following microwords' mar = [rhSrcA, SrcA+offset] read addr, X = the value.
                                // zRET (0xEF, opcode 357'b, ProcListXferDaybreak.mc:182 @RET: MAR <- [rhL, L-LF.word]).
                                // Start's zRET @CPi 15325 XFERs through a NULL link read from real 0x489C0/1 -> sControlTrap.
                                // Trigger only in that window (zRET is far too common otherwise).
                                if (EscLog != null && EscLog.Count < 400 && _ibFront == 0xEF
                                    && InstructionCount > RetLogFrom && InstructionCount < RetLogTo)
                                {
                                    EscLog.Add("=== zRET (0xEF) @" + addr.ToString("X3") + " CPi=" + InstructionCount
                                        + "  R0..R7=" + string.Join(",", System.Linq.Enumerable.Select(System.Linq.Enumerable.Range(0,8), i => _alu.R[i].ToString("X4")))
                                        + "  RH0..RH7=" + string.Join(",", System.Linq.Enumerable.Select(System.Linq.Enumerable.Range(0,8), i => _rh[i].ToString("X2")))
                                        + " sp=" + _stackP + "  (next: @RET MAR<-[rhL, L-LF.word]) ===");
                                    _escCd = 40;
                                }
                                if (EscLog != null && EscLog.Count < 400 &&
                                    _ibFront == 0xF8 && _ib[((int)_ibPtr) & 0x1] == 0x2B)
                                {
                                    byte alpha = _ib[((int)_ibPtr) & 0x1];
                                    // THE DUMP: the eval-stack state at the aBITBLT dispatch.  Under the
                                    // CURRENT model (mesaPush==false) push is pointer-only, so _u[] is written
                                    // ONLY by SUWrite microwords and R0 is NOT a TOS cache.  Print the SU array
                                    // and the pointer together so "how many words did the germ actually push,
                                    // and where did they land" is answered by data rather than by arithmetic.
                                    EscLog.Add("=== BITBLT ENTRY OP=0x" + _ibFront.ToString("X2")
                                        + (_ibFront == 0xF8 ? " alpha=0x" + alpha.ToString("X2") : "")
                                        + " @" + addr.ToString("X3") + " ib=[" + _ib[0].ToString("X2") + "," + _ib[1].ToString("X2")
                                        + "] CPi=" + InstructionCount
                                        + "  sp=" + _stackP + " TOS(R0)=" + _alu.R[0].ToString("X4")
                                        + "  U[0-7]=" + string.Join(",", System.Linq.Enumerable.Select(System.Linq.Enumerable.Range(0, 8), i => _u[i].ToString("X4")))
                                        + "  RH0=" + _rh[0].ToString("X2")
                                        + "  (next mar/X = bbTable field reads) ===");
                                    _escCd = 60;
                                }
                                // @WRMP (MiscDaybreak.mc:113, at[7,10,ESC7n]) = zESC alpha 0x77 -- THE maintenance-panel
                                // chokepoint.  ProcessorFace.SpecialSetMP is MACHINE CODE [zESC, aWRMP] and posts
                                // STRAIGHT to hardware without touching the ProcessorFace.mp global ("Does not set
                                // ProcessorFace.mp" -- ProcessorFace.mesa:27-34), so this dispatch is the only place
                                // every MP post is visible, in order.  TOS (R0) = the mesa arg = the MP code.
                                if (WrmpLog != null && WrmpLog.Count < 20000 && _ibFront == 0xF8 && _ib[((int)_ibPtr) & 0x1] == 0x77)
                                    WrmpLog.Add("MP <- " + _alu.R[0].ToString("X4") + " (dec " + _alu.R[0]
                                        + ")  @CPi " + InstructionCount + " @" + addr.ToString("X3")
                                        + " R5=" + _alu.R[5].ToString("X4") + " RH5=" + _rh[5].ToString("X") + " sp=" + _stackP);
                                // ERROR-RAISE capture: zKFCB (0xF0) with an error alpha = SD[sError/sErrorList/
                                // sReturnError/sReturnErrorList].  At IBDisp the args are on the stack BEFORE the
                                // raise runs: TOS(R0) + _u[sp-1..] hold the DESC'd signal descriptor {taggedGFI,index}
                                // and the ErrorType ordinal.  The FIRST raise in the storage window is the INNER root
                                // (inline-constant ordinal), the re-raise wrapper (GFI 120) fires after it.
                                // The KFCB alpha straddles the IB word boundary, so we can't filter by it here;
                                // log EVERY zKFCB (0xF0) in the [From,To] window and identify error raises by the
                                // stack (a signal descriptor {taggedGFI 0x01xx, index} + the ErrorType arg).
                                if (KfcbLog != null && _ibFront == 0xF0 && InstructionCount >= KfcbLogFrom
                                    && InstructionCount < KfcbLogTo && KfcbLog.Count < 500)
                                {
                                    string stk = "";
                                    for (int k = 1; k <= 6; k++)
                                        stk += (_stackP - k >= 0 ? _u[_stackP - k].ToString("X4") : "----") + ",";
                                    KfcbLog.Add("KFCB CPi=" + InstructionCount + " R5=" + _alu.R[5].ToString("X4")
                                        + " RH5=" + _rh[5].ToString("X") + " sp=" + _stackP
                                        + " TOS=" + _alu.R[0].ToString("X4") + " stk-1..-6=" + stk
                                        + " ib=[" + _ib[0].ToString("X2") + "," + _ib[1].ToString("X2")
                                        + "] ptr=" + _ibPtr + " fw=" + _ibFrontWord.ToString("X4")
                                        // signal-walk anchor: L (=d.raiser = current local frame) + pc16 + all links
                                        + "  L=" + (((_rh[3] & 0x1F) << 16) | _alu.R[3]).ToString("X5")
                                        + " rhL:L=" + _rh[3].ToString("X2") + ":" + _alu.R[3].ToString("X4")
                                        + " GF=" + (((_rh[2] & 0x1F) << 16) | _alu.R[2]).ToString("X5")
                                        + " pc16=" + (_pc16 ? 1 : 0)
                                        + " links=" + string.Join(",", System.Linq.Enumerable.Select(System.Linq.Enumerable.Range(0, 8), i => _link[i].ToString("X"))));
                                    // Read the LIVE frame words at L (=d.raiser) via the VM map -- the STOP memdump's
                                    // frame heap is reused (all-zero), so the return-link/PC must be captured now.
                                    // ReadWord(w) reads phys byte w<<1; VMmap word for vp is at phys 0x80000 -> word
                                    // 0x40000+vp; frame word = rp*256 + (vw&0xFF), rp=((mw&0x1F)<<8)|(mw>>8).
                                    if (ReadWord != null)
                                    {
                                        // Full SignalHandler+CheckCatch emulation with the CERTIFIED GetFrame primitive
                                        // (FastXfer.mc:345-350): GetFrame(link) = SINGLE MDS translation, real =
                                        // map(0x30000 + (link&~3)); tag=link&3 (frame0/proc1/indirect2/rep3).  Frame
                                        // header: [F-1]=PC, [F-2]=globalLink(=GFI<<2), [F-3]=returnLink.  CheckCatch:
                                        // codebase=GFT[GFI]; catchCode=cb[3]; count=cb[cc/2]; EnableTable=cb+cc/2+1+count;
                                        // items stride-3 {start,length,idx}; catch iff (PC-1) in [start,start+length).
                                        System.Func<int, int> mapReal = vw => {
                                            int vp = (vw >> 8) & 0x1FFF; int mw = ReadWord(0x40000 + vp);
                                            int rp = ((mw & 0x1F) << 8) | (mw >> 8); return rp * 256 + (vw & 0xFF);
                                        };
                                        System.Func<int, int> vrd = vw => ReadWord(mapReal(vw));   // read a virtual word
                                        // CheckCatch Part 1: CodeBytes[cb][bytePC] -- inline zCATCH scan.  Big-endian:
                                        // even byte = hi (w>>8), odd = lo (w&0xFF).  cb is a word base, bpc a byte offset.
                                        System.Func<int, int, int> cbyte = (cbase, bpc) => {
                                            int w = vrd(cbase + ((bpc & 0x1FFFF) >> 1));
                                            return ((bpc & 1) != 0) ? (w & 0xFF) : ((w >> 8) & 0xFF);
                                        };
                                        int raiserReal = ((_rh[3] & 0x1F) << 16) | _alu.R[3];
                                        int link = ReadWord(raiserReal - 3);   // ReadReturnLink[raiser] -> start at caller
                                        string walk = "";
                                        for (int depth = 0; depth < 14; depth++)
                                        {
                                            int tag = link & 3;
                                            if (tag == 2) {
                                                // indirect ShortControlLink -> points to a 2-word LONG ControlLink; the
                                                // microcode keeps Q=link (tag intact) and reads with Q+-1 arithmetic, NOT masked.
                                                // Read Q-1/Q/Q+1 to pin the offset, then re-dispatch on the ControlLink's w0 tag.
                                                int qm1 = vrd(0x30000 + link - 1), q0 = vrd(0x30000 + link), qp1 = vrd(0x30000 + link + 1);
                                                walk += "\n      [d" + depth + "] link=" + link.ToString("X4") + " INDIRECT MDS[Q-1,Q,Q+1]="
                                                      + qm1.ToString("X4") + "," + q0.ToString("X4") + "," + qp1.ToString("X4");
                                                // re-dispatch: pick the word that looks like a valid frame link (tag-0, nonzero handle)
                                                int nx = (qp1 != 0 && (qp1 & 3) == 0) ? qp1 : ((q0 != 0 && (q0 & 3) == 0) ? q0 : qm1);
                                                link = nx; continue;
                                            }
                                            if (tag != 0) { walk += "\n      [d" + depth + "] link=" + link.ToString("X4") + " tag=" + tag + " (non-frame STOP)"; break; }
                                            if ((link & 0xFFFC) == 0) { walk += "\n      [d" + depth + "] NULL link -> top of stack, no catch"; break; }
                                            int fReal = mapReal(0x30000 + (link & 0xFFFC));
                                            int pc = ReadWord(fReal - 1), gl = ReadWord(fReal - 2), rl = ReadWord(fReal - 3);
                                            int gfi = gl >> 2, gvw = 0x20000 + 4 * gfi;
                                            int cb = ((vrd(gvw + 3) << 16) | vrd(gvw + 2)) & 0x1FFFFF;
                                            int cc = vrd(cb + 3), pcm1 = (pc - 1) & 0xFFFF;
                                            // Part 1 data: the byte AT the return PC (candidate zCATCH) + its operand (catchIndex).
                                            int bPC = cbyte(cb, pc), bPC1 = cbyte(cb, pc + 1), bPCm1 = cbyte(cb, pcm1);
                                            string res;
                                            if (cc == 0) res = "cc=0 no-catch-table";
                                            else {
                                                int evoff = cc >> 1, count = vrd(cb + evoff), et = cb + evoff + 1 + count, n = vrd(et);
                                                res = "cc=" + cc.ToString("X") + " n=" + n.ToString("X");
                                                bool got = false;
                                                for (int i = 0; i < n && i < 40; i++)
                                                {
                                                    int st = vrd(et + 1 + i * 3), ln = vrd(et + 2 + i * 3);
                                                    if (st <= pcm1 && pcm1 < st + ln) { res += " *** CATCH @[" + st.ToString("X") + "," + (st + ln).ToString("X") + ") idx=" + vrd(et + 3 + i * 3).ToString("X"); got = true; break; }
                                                }
                                                if (!got) { res += " NO-MATCH intervals="; for (int i = 0; i < n && i < 8; i++) res += "[" + vrd(et + 1 + i * 3).ToString("X") + "," + (vrd(et + 1 + i * 3) + vrd(et + 2 + i * 3)).ToString("X") + ")"; }
                                            }
                                            walk += "\n      [d" + depth + "] link=" + link.ToString("X4") + " F=0x" + fReal.ToString("X5")
                                                  + " GFI=" + gfi + " PC=" + pc.ToString("X4") + " PC-1=" + pcm1.ToString("X4") + " cb=0x" + cb.ToString("X")
                                                  + " code[PC-1,PC,PC+1]=" + bPCm1.ToString("X2") + "," + bPC.ToString("X2") + "," + bPC1.ToString("X2") + " " + res;
                                            if (res.Contains("CATCH")) break;
                                            link = rl;
                                        }
                                        KfcbLog.Add("    SIGWALK+CHECKCATCH (GetFrame=single MDS xlate):" + walk);
                                    }
                                    // Replay the dispatch ring (byte stream leading INTO this f0) so the inline
                                    // 9a-XX ordinal at the raise site is directly readable, no stack-slot guessing.
                                    if (DispRingCPi != null)
                                    {
                                        int start = DispRingPos - 40; if (start < 0) start = 0;
                                        for (int r = start; r < DispRingPos; r++)
                                        {
                                            int di = r & 63;
                                            KfcbLog.Add("    disp CPi=" + DispRingCPi[di]
                                                + " op=" + DispRingOp[di].ToString("X2")
                                                + " ib=[" + ((DispRingIb[di] >> 8) & 0xFF).ToString("X2") + ","
                                                + (DispRingIb[di] & 0xFF).ToString("X2") + "]"
                                                + " ptr=" + DispRingPtr[di]
                                                + " R5=" + DispRingR5[di].ToString("X4")
                                                + " RH5=" + DispRingRH5[di].ToString("X2"));
                                        }
                                    }
                                }
                                if (IbLog != null && InstructionCount >= IbLogFrom && InstructionCount < IbLogTo)
                                    IbLog.Add("@" + addr.ToString("X3") + " CPi=" + InstructionCount + " IBDisp dispatched " + _ibFront.ToString("X2") + " ptr=" + _ibPtr + " -> advance front<-_ib[" + (((int)_ibPtr) & 0x1) + "]=" + _ib[((int)_ibPtr) & 0x1].ToString("X2") + " ib=[" + _ib[0].ToString("X2") + "," + _ib[1].ToString("X2") + "]");
                                _ibFront = _ib[((int)_ibPtr) & 0x1];
                                _ibPtr = _nextIBPtr[(int)_ibPtr];   // DecrementIBPtr
                                _ibFrontWord = _ibWord;             // MEASUREMENT: front now comes from the _ib pair's word
                            }
                            IbDispByCycle[_cycle & 3]++;   // IBDisp is pinned cy:c2 -- outside c1/c2 => BUG
                            Note("IBDisp");
                        }
                        break;
                    case 0x6:   // LoadIB: fill the instruction buffer from the X bus (2 opcode bytes).
                        {
                            bool loadIBPtr1 = (mi.fSfZ == FunctionSelectFZ.fzNorm && mi.fZ == 0x1);
                            bool wasEmptyLoad = (_ibPtr == IBState.Empty);   // MEASUREMENT: empty-path load sets the front byte from this word
                            // DIAG: at XFER-entry LoadIBs (xcE has Cin<-pc16 / xcO has IBPtr<-1), dump L2 (=_link[2])
                            // and all links + pc16 so we can see whether the entry parity comes from pc16 or L2's low bit.
                            if (false && LinkLog != null && LinkLog.Count < 60 && (invertPc16 || loadIBPtr1))
                            {
                                string lk = ""; for (int i = 0; i < 8; i++) lk += _link[i].ToString("X");
                                LinkLog.Add("@" + addr.ToString("X3") + " CPi=" + InstructionCount + " IB<- fZ=" + mi.fZ.ToString("X")
                                    + (loadIBPtr1 ? " [IBPtr<-1/xcO]" : "") + (invertPc16 ? " [Cin<-pc16/xcE]" : "")
                                    + " pc16=" + (_pc16 ? 1 : 0) + " L2=" + _link[2].ToString("X") + "(lo=" + (_link[2] & 1) + ")"
                                    + " links[0-7]=" + lk + " R5=" + _alu.R[5].ToString("X4") + " RH5=" + _rh[5].ToString("X") + " X=" + _xBus.ToString("X4"));
                            }
                            if (loadIBPtr1)
                            {
                                if (_ibPtr != IBState.Empty)
                                {
                                    _ib[0] = (byte)(_xBus >> 8);
                                    _ib[1] = (byte)_xBus;
                                    _ibPtr = IBState.Full;
                                }
                                else
                                {
                                    _ibPtr = IBState.Byte;
                                    _ibFront = (byte)_xBus;
                                }
                            }
                            else
                            {
                                _ib[1] = (byte)_xBus;
                                if (_ibPtr != IBState.Empty)
                                {
                                    _ib[0] = (byte)(_xBus >> 8);
                                    _ibPtr = IBState.Full;
                                }
                                else
                                {
                                    _ibFront = (byte)(_xBus >> 8);
                                    _ibPtr = IBState.Word;
                                }
                            }
                            // MEASUREMENT: the _ib pair (and, on the empty path, the front byte) now come from _mar's word.
                            _ibWord = _mar & 0xFFFF;
                            if (wasEmptyLoad) _ibFrontWord = _ibWord;
                            if (IbLog != null && InstructionCount >= IbLogFrom && InstructionCount < IbLogTo)
                                IbLog.Add("@" + addr.ToString("X3") + " CPi=" + InstructionCount + " LoadIB " + (loadIBPtr1 ? "[,,1/odd]" : "[even]") + " X=" + _xBus.ToString("X4") + " -> ib=[" + _ib[0].ToString("X2") + "," + _ib[1].ToString("X2") + "] front=" + _ibFront.ToString("X2") + " ptr=" + _ibPtr + (_ib[0] == 0x37 ? "  <<< _ib[0] STALE 0x37 (empty-path left it)" : ""));
                            Note("LoadIB");
                        }
                        break;
                    default:
                        // MAPA<- / IO<- / ClrIE / SetIE / ClrIntErr / ... : side effects that
                        // don't affect control flow -- stateful no-ops for now.  MAPA<- takes
                        // its value from the X bus (TmMacroTablesDaybreak:395, a 1984 bug fix);
                        // log the candidates so the right fY code can be pinned (xBus==4).
                        Note("fY:" + mi.fY.ToString("X"));
                        if (FyLog != null && FyLog.Count < 40 && mi.fY != 0x8)
                            FyLog.Add("@" + addr.ToString("X3") + " fY=" + mi.fY.ToString("X") + " xBus=" + _xBus.ToString("X4") + " yBus=" + _yBus.ToString("X4"));
                        break;
                }
            }

            // ---- fZ functions (Daybreak: fZ A/B = ClrLOCK/SetLOCK; Bank<- is fY=D, NOT
            //      fZ.  AltUaddr (7) and the LateLRot codes (C-F, handled above) carry
            //      over from the DLion layout.) ----
            if (mi.fSfZ == FunctionSelectFZ.fzNorm)
            {
                switch (mi.fZ)
                {
                    case 0x1:   // LoadIBPtr<-1 : advance the IB read pointer to the byte position.
                                // Skip when this fZ is a modifier for IBDisp/LoadIB (fY 3/6),
                                // where the fY case already consumes it (AlwaysIBDisp / IB<- ,,1).
                        if (!(mi.fSfY == FunctionSelectFY.fyNorm && (mi.fY == 0x3 || mi.fY == 0x6)))
                        {
                            if (_ibPtr != IBState.Byte) { _ibPtr = IBState.Byte; _ibFront = _ib[1]; }
                        }
                        break;
                    case 0x2:   // LoadIBPtr<-0 : reset the IB read pointer to the word position.
                        if (_ibPtr != IBState.Word) { _ibPtr = IBState.Word; _ibFront = _ib[0]; }
                        break;
                    case 0x7:   // AltUaddr
                        _altUAddr = true;
                        break;
                    // Refresh / ClrLOCK/SetLOCK / LRot* / Noop*: no control-flow state modelled here yet.
                    // NB: fZ 0x3 (Cin<-pc16) is handled up-front (before the ALU) so its carry-in
                    // reaches Execute; only its end-of-cycle pc16 toggle happens below.
                }
            }

            // pc16 inverts at the end of the cycle whenever this microword loaded Cin from pc16
            // (either encoding).  (HW ref, section 2.3.7 -- matches DLion CentralProcessor.)
            if (invertPc16) _pc16 = !_pc16;

            // ---- SU register write (U<-) ----  MUST precede the stack pointer update: the DLion
            //      CentralProcessor writes _u[_stackP] at the CURRENT pointer, THEN moves it at the
            //      end of the microinstruction ("Stack modifications occur at the end", HWref).
            //      Doing push/pop first (as before) landed every push-with-write one slot high and
            //      wrecked the mesa eval stack (e.g. @BLTL args at the wrong depth).
            // NOTE: TOS==R0 spill-and-cache on push (Daybreak @LI0/LIn/PushT) is the intended model,
            // but a blanket/`SUWrite`-gated implementation regressed the germ (bare/subroutine pushes
            // must stay pointer-only, and the arg pushes don't carry SUWrite) -- pending the exact
            // mesa-push encoding from the operator.  Reverted to pointer-only push for now.
            const bool mesaPush = false;

            if (mi.SUWrite && !mesaPush)
            {
                switch ((int)mi.fSfZ)
                {
                    case 0:
                    case 1:
                        _u[_stackP] = _yBus;
                        break;
                    default: // 2,3
                        if (altUAddr) _u[(mi.rA << 4) | (_lastYBus & 0xf)] = _yBus;
                        else _u[mi.UAddress] = _yBus;
                        break;
                }
            }

            // ---- Stack pointer update (end of microinstruction) ----
            // stackP<- (fYnorm slot 5, TmMacroTablesDaybreak [["stackP<-",5],[fS01:norm,fY:a1,yIn:t,yl:t]])
            // takes the Y bus and BYPASSES Table 2.11 entirely -- the table governs only pop/push, so this
            // can install any value, legal or not.  GermOpsImpl:848-849 CodeTrap wraps the module-start call
            // in a save/restore pair: `d.state <- STATE` ("Must be first"; ControlTrap's comment on the
            // identical construct says reading STATE "resets stack pointer") ... Call[MainBody[d.gfi]] ...
            // `STATE <- d.state` ("Must be last"), where state is a PrincOps.StateVector carrying
            // stk[0..14) AND stkptr -- "holds args of called procedure".  If STATE<- doesn't restore stkptr,
            // the germ resumes on a pointer nobody set.  Log every non-pop/push write to _stackP.
            int _spBefore = _stackP;
            if (mi.LoadStackP)
            {
                if (StkTrapLog != null && StkTrapLog.Count < 40)
                    StkTrapLog.Add("--- stackP<- @" + addr.ToString("X3") + " CPi=" + InstructionCount
                        + "   sp " + _stackP + " -> " + (_yBus & 0xf) + "   (Ybus=" + _yBus.ToString("X4") + ")   "
                        + mi.Disassemble(-1));
                if (ProcSwLog != null && InstructionCount >= ProcSwFrom && InstructionCount < ProcSwTo
                    && ProcSwLog.Count < 400)
                    ProcSwLog.Add("RESTORE stackP<-  CPi=" + InstructionCount + " @" + addr.ToString("X3")
                        + "  sp " + _stackP + " -> " + (_yBus & 0xf) + "   (Ybus=0x" + _yBus.ToString("X4") + ")");
                _stackP = (_yBus & 0xf);
            }

            // ---- TechRef Table 2.11: stack pointer overflow/underflow DETECTOR ----
            //   functions            stackP   trap is      if stackP is
            //   pop                   -1      underflow      0
            //   push                  +1      overflow      15
            //   fXpop, push            0      underflow      0
            //   push, fZpop            0      overflow      15
            //   fXpop, fZpop          -1      underflow      0 or 1
            //   fXpop, fZpop, push     0      underflow      0 or 1
            // "If a pop or push is executed with the values of the stackPointer given in Table 2.11
            //  then a trap to location 0 in c1 occurs.  However, stackP is still modified."
            // The MOTION above is already correct for all six rows (StackTest gates the three
            // combined rows to delta 0); what was missing is the trap.  Rows 3 and 4 have the SAME
            // delta but OPPOSITE conditions, so FxPop/FzPop must be tested separately -- Pop
            // (= fxPop||fzPop) cannot distinguish them.
            // Tested on the PRE-update _stackP, which is what the table's column means.
            // *** NOT VECTORED ON PURPOSE ***: microstore 0 is BootTrap (InitDaybreak.mc:19
            // StartAddress[BootTrap], :41 "From trap branch in Refill.mc") = the boot-button/INIT
            // vector -- ClrIntErr, ClrLOCK, ClrMPIntIOP, G<-0, reprogram the 8254s = a FULL MACHINE
            // RE-INIT.  Nothing "relies on" this trap; it is the hardware's detector for broken
            // microcode.  Vectoring it before the first underflow is fixed would faithfully reboot
            // the machine at the first offence.  Detector first, vector later as the regression test.
            if (mi.StackOperation)
            {
                bool fx = mi.FxPop, fz = mi.FzPop, pu = mi.Push;
                bool trap; string row;
                if (fx && fz && pu) { row = "fXpop,fZpop,push (d=0,  underflow@0|1)"; trap = (_stackP <= 1); }
                else if (fx && fz)  { row = "fXpop,fZpop      (d=-1, underflow@0|1)"; trap = (_stackP <= 1); }
                else if (pu && fz)  { row = "push,fZpop       (d=0,  overflow@15)";   trap = (_stackP == 15); }
                else if (pu && fx)  { row = "fXpop,push       (d=0,  underflow@0)";   trap = (_stackP == 0); }
                else if (pu)        { row = "push             (d=+1, overflow@15)";   trap = (_stackP == 15); }
                else                { row = "pop              (d=-1, underflow@0)";   trap = (_stackP == 0); }
                if (trap)
                {
                    _trapCode = 2;   // Table 2.10 priority 2; surfaces on X[8-9] via <-ErrnIBnStkp
                    _stkTraps++;
                    if (StkTrapLog != null && StkTrapLog.Count < 40)
                        StkTrapLog.Add("*** STACK TRAP #" + _stkTraps + " @" + addr.ToString("X3")
                            + " CPi=" + InstructionCount + "  sp(before)=" + _stackP
                            + "  " + row + "   " + mi.Disassemble(-1));
                }
            }

            // Only StackTest==None actually moves the 16-deep pointer.  Push+pop combos are
            // NON-modifying over/underflow TESTS (HWref p.33) -- applying Push/Pop raw corrupted depth.
            if (mi.StackOperation && mi.StackTest == StackTestType.None)
            {
                if (mi.Push)
                {
                    if (mesaPush)
                    {
                        _u[_stackP] = _r0Old;     // spill the outgoing TOS(R0) into the SU array
                        _alu.R[0] = _yBus;        // cache the pushed value into TOS(R0)
                    }
                    _stackP = (_stackP + 1) & 0xf;
                }
                else if (mi.DoublePop) _stackP = (_stackP - 1) & 0xf;
                else if (mi.Pop) _stackP = (_stackP - 1) & 0xf;
            }

            // ALSO log StackOperation microwords that moved NOTHING because StackTest != None
            // (my decoder treats those combined rows as non-modifying over/underflow TESTS).  If
            // Daybreak expects motion on any of them, the missing delta is invisible to a
            // changed-sp-only trace -- this is where a hidden +/-1 would hide.
            if (SpTraceLog != null && _stackP == _spBefore && mi.StackOperation
                && mi.StackTest != StackTestType.None
                && InstructionCount >= SpTraceFrom && InstructionCount < SpTraceTo
                && SpTraceLog.Count < 3000)
            {
                SpTraceLog.Add("NOMOVE @" + addr.ToString("X3") + " CPi=" + InstructionCount
                    + "  sp " + _stackP + " (unchanged)  test=" + mi.StackTest
                    + " push=" + (mi.Push ? 1 : 0) + " pop=" + (mi.Pop ? 1 : 0)
                    + " fxPop=" + (mi.FxPop ? 1 : 0) + " fzPop=" + (mi.FzPop ? 1 : 0)
                    + " dblPop=" + (mi.DoublePop ? 1 : 0)
                    + " | " + mi.Disassemble(-1));
            }
            if (SpTraceLog != null && _stackP != _spBefore
                && InstructionCount >= SpTraceFrom && InstructionCount < SpTraceTo
                && SpTraceLog.Count < 3000)
            {
                string how = mi.LoadStackP ? "stackP<-" :
                             (mi.Push ? "PUSH" : (mi.DoublePop ? "DOUBLEPOP" : (mi.Pop ? "POP" : "?")));
                SpTraceLog.Add("uw @" + addr.ToString("X3") + " CPi=" + InstructionCount
                    + " R5=" + _alu.R[5].ToString("X4") + " RH5=" + _rh[5].ToString("X2")
                    + " op=" + _ibFront.ToString("X2")
                    + "  sp " + _spBefore + " -> " + _stackP + "  [" + how + "]"
                    + " stkOp=" + (mi.StackOperation ? 1 : 0) + " test=" + mi.StackTest
                    + " push=" + (mi.Push ? 1 : 0) + " pop=" + (mi.Pop ? 1 : 0)
                    + " fxPop=" + (mi.FxPop ? 1 : 0) + " fzPop=" + (mi.FzPop ? 1 : 0)
                    + " dblPop=" + (mi.DoublePop ? 1 : 0) + " suW=" + (mi.SUWrite ? 1 : 0)
                    + " | " + mi.Disassemble(-1));
            }

            // ---- Next instruction address ----
            // The WCS stores INIA's low nibble COMPLEMENTED (TechRef Fig 2.6); the true
            // successor is (rawINIA XOR 0x00F).  How the modifier merges depends on the
            // dispatch type (mesa IB dispatch replaces bit-fields rather than OR-ing).
            niaModifier |= _xDispNow;      // same-word XDisp bits (opt-in, see XDispSameWord)
            // ---- CONFIRMED CORRECT: the ^ 0x00F complement ----
            // The TechRef control-store bit map names the field "pINIA.00-07" then
            // "pINIA.08'-11'" -- the PRIMES mark the low four bits as stored complemented,
            // which is exactly this XOR.  Independently confirmed by execution: C04 has raw
            // INIA 180 and demonstrably branches to 18F (0x180 ^ 0x00F = 0x18F).
            //
            // Cancellation still works as 2.3.3.1 describes (a condition bit is ignored
            // where INIA has a 1) -- it applies to the DE-complemented value.  An earlier
            // note here called this XOR the suspect behind MP 7700; that was WRONG.
            //
            // 18F is SMF (Set Map Flags), Misc.mc:273 -- "Xbus <- TT LRot12, XDisp" in c1,
            // whose c2 successor carries DISP4[SMFb,1].  Its eight targets are 01/03/05/07/
            // 09/0B/0D/0F: the ",1" forces INIA's low bit, so slot 0 is unreachable BY
            // CONSTRUCTION -- same trick as DISP4[ErrTrap,3], whose only live slots are
            // 3/7/B/F.  So no dispatch table can collide with the trap vectors, and our
            // nia=000 means the base we used is not the one the microcode intends.
            // Also still wrong per Table 2.8, independent of this: dispatch widths differ (Br=1
            // bit into [11]; XwdDisp/XHDisp/XLDisp/PgCrOvDisp=2 into [10-11]; XDisp/YDisp/
            // XC2npcDisp/XWtOKDisp/LnDisp=4 into [8-11]; IBDisp=8 into [4-11]) and we OR 4 bits
            // for everything; IBDisp SETS NIA[4-7] rather than ORing; and Link is a THIRD
            // contributor to the same OR.
            int trueINIA = mi.INIA ^ 0x00F;
            int nia;
            switch (niaModType)
            {
                case 1:  // IBDispatch: ibFront hi nibble replaces INIA[4-7], lo nibble ORs INIA[8-11]
                    nia = (trueINIA & 0xf0f) | niaModifier;
                    break;
                case 2:  // IBRefillTrap: trap vector replaces INIA[0-3] (=nia[11-8])
                    nia = (trueINIA & 0x0ff) | niaModifier;
                    break;
                default: // Normal: OR the branch/dispatch bits in
                    nia = trueINIA | niaModifier;
                    break;
            }
            // DOVE_NIA_WATCH=<hex addr>: why did this microword compute the next address it did?
            // 18F is the ErrTrap dispatch -- 'XDisp LRot12 Xbus<- R2', raw INIA 00F, so
            // trueINIA = 00F ^ 00F = 000 and nia is PURELY the dispatch nibble.  Healthy boot
            // resolves it to 4; a nibble of 0 lands on microstore 0 and is indistinguishable
            // from a trap.  Log the inputs so the zero can be attributed rather than guessed.
            if (_niaWatch >= 0 && addr == _niaWatch && NiaWatchLog != null && NiaWatchLog.Count < 300)
                NiaWatchLog.Add("@" + addr.ToString("X3") + " c" + _cycle
                    + "  rawINIA=" + mi.INIA.ToString("X3") + " trueINIA=" + trueINIA.ToString("X3")
                    + "  mod=" + niaModifier.ToString("X3") + " modType=" + niaModType
                    + "  xBus=" + _xBus.ToString("X4") + " R2=" + _alu.R[2].ToString("X4")
                    + "  bank=" + _execBank + "->" + _bankTarget + " bankPend=" + _bankChangePending
                    + "  -> nia=" + nia.ToString("X3")
                    + (nia == 0 ? "   *** LANDS ON ErrTrap ***" : "")
                    + "  CPi=" + InstructionCount);
            _tpc[_task] = nia;

            // ---- IB-Empty trap (EKErr 3) ----
            // Refill.mc:  "Control comes to IBEmpty Trap if an <-ib, <-ibNA, <-ibLow, or <-ibHigh
            // were executed on an empty buffer (left empty by the NERefill code when the PC is on
            // the last word of a page) ... the trap always occurs in the click AFTER the one which
            // error'd."  So the flag is raised by the read and vectored here, at the end of the
            // click, sending the next click to microstore 0 (ErrTrap).
            //
            // ErrTrap dispatches on rInt LRot12, i.e. ErrnIBInt bits 4-7 = (EKErr << 2) | ~ibPtr:
            //   1 Boot -> slot 7 XferIndirect   2 Stack -> slot 0B StackErr   3 IBEmpty -> slot 0F
            // and slot 3 -- reached when NO code is set -- is UnexpectedErr, an unconditional
            // GOTO[UnexpectedErr] self-loop with T <- sHardwareError.  That is exactly where this
            // emulator wedged at MP 7700: it detected the empty-buffer read (there was even a log
            // line saying "DLion would trap/refill") but never set EKErr, so the microcode's own
            // ErrTrap dispatched the no-code case and span forever at microstore 0x003.
            //
            // This is the instruction-stream page-crossing mechanism -- IBEmpty restores PC/pc16/
            // TOS/stackP from SaveState and faults in the new PC page -- which is why full Pilot
            // with demand paging is the first thing to need it.
            if (_ibEmptyPending && _cycle == 3)      // end of the erroring click; trap on the next one
            {
                _ibEmptyPending = false;
                if (_trapCode == 0)         // "smaller values of EKErr have priority over the larger"
                {
                    _trapCode = 3;
                    _tpc[_task] = 0;
                    _cycle = 0;             // ++ below makes the trap click start at c1
                    IbEmptyTraps++;
                }
            }

            if (mi.LinkAddress != -1)
            {
                // Link write (capture, NIA[7]=0): store the RESOLVED NIA[8-11] = latched-DispBr only.
                // Reverted the coincident `| _niaModifier` term: it leaked THIS cycle's XWtOKDisp OK bit into
                // the L2<-L2.SG return link at @SGB c2 (LoadStore.mc:388), so an already-dirty-page zSGB store
                // (OK=1) had its return link corrupted and mis-routed to the wild Map<- deref (real 0x5C=0xBBBB)
                // while not-yet-dirty stores (OK=0) worked.  The reference line captures only `nia`.
                if ((nia & 0x10) == 0) _link[mi.LinkAddress] = nia & 0xf;   // link write (capture resolved NIA[8-11])
                else _niaModifier |= _link[mi.LinkAddress];                 // link read (LnDisp) — latched (current-cycle broke germ boot)
                if (LinkLog != null && mi.LinkAddress == 2 && LinkLog.Count < 80)
                    LinkLog.Add("L2op @" + addr.ToString("X3") + " CPi=" + InstructionCount + " " + ((nia & 0x10) == 0 ? "WRITE" : "READ ")
                        + " nia=" + nia.ToString("X3") + " trueINIA=" + trueINIA.ToString("X3") + " latchedMod=" + niaModifier.ToString("X3")
                        + " curMod=" + _niaModifier.ToString("X3") + " L2now=" + _link[2].ToString("X") + " pc16=" + (_pc16 ? 1 : 0)
                        + " ~pc16=" + (_pc16 ? 0 : 1) + " fY=" + mi.fY.ToString("X") + " fSfY=" + (int)mi.fSfY);
            }

            // Watch: R5(PC) / RH5(rhPC) ever landing a code-address-like value (the XCode-tail signature).
            if (R5Log != null && R5Log.Count < 60)
            {
                if (_rh[5] != _rh5old && _rh[5] != 0)
                    R5Log.Add("@" + addr.ToString("X3") + " RH5 " + _rh5old.ToString("X") + "->" + _rh[5].ToString("X") + " (R5=" + _alu.R[5].ToString("X4") + " X=" + _xBus.ToString("X4") + " CPi=" + InstructionCount + ")  " + mi.Disassemble(-1));
                if (_alu.R[5] != _r5old && _alu.R[5] > 0x0100)
                    R5Log.Add("@" + addr.ToString("X3") + " R5 " + _r5old.ToString("X4") + "->" + _alu.R[5].ToString("X4") + " (RH5=" + _rh[5].ToString("X") + " Y=" + _yBus.ToString("X4") + " CPi=" + InstructionCount + ")  " + mi.Disassemble(-1));
            }
            if (IbLog != null && InstructionCount >= IbLogFrom && InstructionCount < IbLogTo && _alu.R[5] != _r5old)
            {
                IbLog.Add("   R5-WRITE @" + addr.ToString("X3") + " CPi=" + InstructionCount + " R5 " + _r5old.ToString("X4") + "->" + _alu.R[5].ToString("X4") + "  " + mi.Disassemble(-1));
            }

            InstructionCount++;

            // CP 8254 counter0 (mode-2 rate generator) heartbeat: present a timer-interrupt edge
            // (rInt bit 15) every TimerPeriod instructions.  This is the Pilot scheduler tick the
            // germ's waitForInterrupt idles on; without it MesaIntBr never fires and @666 spins.
            if (++_timerCounter >= TimerPeriod) { _timerCounter = 0; _timerInt = true; TimerFireCount++; }
            // 8254 channels 1+2: ~12.8 CP instructions per count (62.5 kHz), counting DOWN.
            if (Io8254Enabled) { _pitAccum += 10; if (_pitAccum >= 128) { _pitAccum -= 128; _pit32--; } }

            // Bank<- takes effect one instruction late: the write at N leaves N+1 still
            // fetching the old bank, N+2 the new one.
            if (_bankChangePending > 0 && --_bankChangePending == 0)
                _execBank = _bankTarget;

            _cycle++;
            if (_cycle > 3) _cycle = 1;   // no tasks on Daybreak; c1/c2/c3 is memory timing
        }
    }
}
