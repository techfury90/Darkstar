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
        private long _timerCounter;
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
        public List<string> WrmpLog;   // TEMP: every @WRMP (zESC alpha 0x77) = THE maintenance-panel post chokepoint
        public int RetLogFrom = 15300, RetLogTo = 15340;   // TEMP: zRET (0xEF) trace window
        public int[] AddrHist;          // TEMP: microword-address histogram -- names microcode spins
        public List<string> LoopLog;    // TEMP: full-state dump across microcode-spin iterations
        public int LoopAddr = -1, LoopFrom = int.MaxValue;
        public int RingFrom = int.MaxValue;   // TEMP: arm the entry-path ring buffer
        public List<string> StkTrapLog;       // TechRef Table 2.11 stack over/underflow detector
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
            if ((value & 0x0100) != 0) _mInt = true;   // b8 IOP->CP doorbell sets the int reg
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
            int addr = _tpc[_task];
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
                        _xBus = (ushort)(((_trapCode & 3) << 6) | (_mInt ? 0x2 : 0) | (_timerInt ? 0x1 : 0));
                        IntStatReads++;
                        _timerInt = false;
                        _trapCode = 0;   // read-to-clear: the InitTrap code is acked by the @0 read
                        break;
                    case 0xA:   // <-ErrnIBnStkp: X[8-9]=trap, X[10-11]=~ibPtr, X[12-15]=~stackP.
                                // The StkP field reports ~stackP_hw, and the hardware's stackP counts DOWN from
                                // full, so ~stackP_hw == the TRUE stack DEPTH.  Our _stackP counts the SU-array
                                // spills and does NOT include the TOS (cached in R0), so depth = _stackP + 1.
                                // Reporting (~_stackP) sent BitBlt's HowBigStack dispatch (BBInit.mc:28/29,
                                // DISP4[HowBigStack]) to hbs.E (BandBLT's interrupt-resume entry, BandBLT.mc:58)
                                // instead of hbs.2 (bbNormEntry) -- so bbGetArg/MDtoRbb0 never read the bbTable
                                // and the germ's 16x1 probe blt ran BandBLT's unbounded band walk (MP stuck 0900).
                                // Depth checks out across the table: 1-word Arg0 -> hbs.1 (BandBLT fresh),
                                // 2-word BBptr -> hbs.2 (BitBlt fresh), 12 -> hbs.C (interrupt resume).
                        _xBus = (ushort)(((_trapCode & 3) << 6) | (((~(int)_ibPtr) & 0x3) << 4)
                            | ((_stkFieldDepth ? (_stackP + 1) : ~_stackP) & 0xf));
                        _trapCode = 0;   // read-to-clear
                        break;
                    case 0xB:   // <-RH
                        _xBus = _rh[mi.rB];
                        break;
                    case 0xC:   // <-ibNA (front, no advance)
                        _xBus = _ibFront;
                        break;
                    case 0xD:   // <-ib (front, then advance: refill ibFront from IB and decrement ibPtr)
                        if (IbLog != null && InstructionCount >= IbLogFrom && InstructionCount < IbLogTo)
                            IbLog.Add("@" + addr.ToString("X3") + " CPi=" + InstructionCount + " <-ib reads front=" + _ibFront.ToString("X2") + " ptr=" + _ibPtr + " (next front<-_ib[" + (((int)_ibPtr) & 0x1) + "]=" + _ib[((int)_ibPtr) & 0x1].ToString("X2") + ") ib=[" + _ib[0].ToString("X2") + "," + _ib[1].ToString("X2") + "]" + (_ibPtr == IBState.Empty ? "  <<< READ FROM EMPTY (DLion would trap/refill)" : ""));
                        _xBus = _ibFront;
                        _ibFront = _ib[((int)_ibPtr) & 0x1];
                        _ibPtr = _nextIBPtr[(int)_ibPtr];
                        _ibFrontWord = _ibWord;   // MEASUREMENT: front now comes from the _ib pair's word
                        break;
                    case 0xE:   // <-ibLow
                        _xBus = (ushort)(_ibFront & 0xf);
                        break;
                    case 0xF:   // <-ibHigh
                        _xBus = (ushort)((_ibFront >> 4) & 0xf);
                        break;
                    default:    // 6/8 = ExtStat/DebB (Burdock debug byte pipe): 0
                        _xBus = 0;
                        Note("IOXIn:" + mi.fZ.ToString("X"));
                        break;
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
                _xBus = ReadWord(_mar);
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
                if (SdReadLog != null && _mar >= 0x48200 && _mar <= 0x48220 && SdReadLog.Count < 200)
                {
                    int slot = (_mar - 0x48200) / 2; string field = ((_mar & 1) == 0) ? "gf" : "pc";
                    SdReadLog.Add("<-MD SD[" + slot.ToString("X2") + "]." + field + " (real " + _mar.ToString("X5") + ") = " + _xBus.ToString("X4") + " @" + addr.ToString("X3") + " R5=" + _alu.R[5].ToString("X4") + " RH5=" + _rh[5].ToString("X") + " CPi=" + InstructionCount);
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
                            _mar = ((_rh[mi.rB] & 0xf) << 16) | _marLowSplice;
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
                        if (WriteWord != null) WriteWord(_mar, _yBus);   // MDR<-
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
                    case YDispBrFunction.XDisp: _niaModifier |= (_xBus & 0xf); break;
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
                    case 0x2:   // ClrIntErr: clear the error/trap latch (the trap code the germ
                                // reads via <-IntStat X[8-9] / <-ErrnIBnStkP).  Was a no-op, which
                                // left the InitTrap code (1) stuck set forever -> phantom trap.
                        _trapCode = 0;
                        Note("ClrIntErr");
                        break;
                    case 0xE:   // ClrIE: disable interrupts (Daybreak fYNorm 0xE, TmMacroTables:274).
                        _ie = false;
                        Note("ClrIE");
                        break;
                    case 0xF:   // SetIE: enable interrupts (Daybreak fYNorm 0xF, TmMacroTables:276).
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
                            // AlwaysIBDisp = IBDisp + IBPtr<-1 (fZ=1): a non-trapping dispatch.
                            bool alwaysIBDisp = (mi.fSfZ == FunctionSelectFZ.fzNorm && mi.fZ == 0x1);
                            if ((_ibPtr != IBState.Full || _mInt) && !alwaysIBDisp)
                            {
                                // IB not full (or Mesa int pending): trap to the refill/interrupt
                                // handler instead of dispatching.  INIA[0-3] (=nia[11-8]) replaced.
                                if (_mInt)
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
                                    OpLog.Add((inStartPage ? "[START] " : "") + "CPi=" + InstructionCount + " @" + addr.ToString("X3") + " OP=0x" + _ibFront.ToString("X2") + " R5=" + _alu.R[5].ToString("X4") + " RH5=" + _rh[5].ToString("X") + " pc16=" + (_pc16?1:0) + " ibPtr=" + _ibPtr + " fw=" + _ibFrontWord.ToString("X4") + " offC=" + ((_alu.R[5] - _ibFrontWord) & 0xFFFF).ToString("X4") + " ib=[" + _ib[0].ToString("X2") + "," + _ib[1].ToString("X2") + "] TOS=" + _alu.R[0].ToString("X4") + " sp=" + _stackP + " L=[" + _rh[3].ToString("X2") + ":" + _alu.R[3].ToString("X4") + "]->" + ((((_rh[3] & 0xF) << 16) | _alu.R[3])).ToString("X5"));
                                    // offC = committed-R5 offset (sampled post-L487 ALU commit, unlike the top-of-Step off at L307
                                    // which reads R5 PRE-commit and manufactures the 0xFFFF word-crossing artifact -- see audit wf_2e3020ed).
                                _niaModifier |= _ibFront;
                                _niaModType = 1;   // IBDispatch
                                // TEMP: BitBlt probe -- catch aBITBLT (F8 2B = ESC2n[0x0B]) or PILOTBITBLT (0x76),
                                // the BitBlt at the end of ProcessorHeadDove.Start.  Trace the setup so the bbTable
                                // geometry the microcode reads (Width @offset+8, Height @offset+0) is visible:
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
                                    (_ibFront == 0x76 || (_ibFront == 0xF8 && _ib[((int)_ibPtr) & 0x1] == 0x2B)))
                                {
                                    byte alpha = _ib[((int)_ibPtr) & 0x1];
                                    EscLog.Add("=== BITBLT ENTRY OP=0x" + _ibFront.ToString("X2")
                                        + (_ibFront == 0xF8 ? " alpha=0x" + alpha.ToString("X2") : "")
                                        + " @" + addr.ToString("X3") + " ib=[" + _ib[0].ToString("X2") + "," + _ib[1].ToString("X2")
                                        + "] CPi=" + InstructionCount + "  (next mar/X = bbTable field reads) ===");
                                    _escCd = 60;
                                }
                                // @WRMP (MiscDaybreak.mc:113, at[7,10,ESC7n]) = zESC alpha 0x77 -- THE maintenance-panel
                                // chokepoint.  ProcessorFace.SpecialSetMP is MACHINE CODE [zESC, aWRMP] and posts
                                // STRAIGHT to hardware without touching the ProcessorFace.mp global ("Does not set
                                // ProcessorFace.mp" -- ProcessorFace.mesa:27-34), so this dispatch is the only place
                                // every MP post is visible, in order.  TOS (R0) = the mesa arg = the MP code.
                                if (WrmpLog != null && _ibFront == 0xF8 && _ib[((int)_ibPtr) & 0x1] == 0x77)
                                    WrmpLog.Add("MP <- " + _alu.R[0].ToString("X4") + " (dec " + _alu.R[0]
                                        + ")  @CPi " + InstructionCount + " @" + addr.ToString("X3")
                                        + " R5=" + _alu.R[5].ToString("X4") + " RH5=" + _rh[5].ToString("X") + " sp=" + _stackP);
                                if (IbLog != null && InstructionCount >= IbLogFrom && InstructionCount < IbLogTo)
                                    IbLog.Add("@" + addr.ToString("X3") + " CPi=" + InstructionCount + " IBDisp dispatched " + _ibFront.ToString("X2") + " ptr=" + _ibPtr + " -> advance front<-_ib[" + (((int)_ibPtr) & 0x1) + "]=" + _ib[((int)_ibPtr) & 0x1].ToString("X2") + " ib=[" + _ib[0].ToString("X2") + "," + _ib[1].ToString("X2") + "]");
                                _ibFront = _ib[((int)_ibPtr) & 0x1];
                                _ibPtr = _nextIBPtr[(int)_ibPtr];   // DecrementIBPtr
                                _ibFrontWord = _ibWord;             // MEASUREMENT: front now comes from the _ib pair's word
                            }
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
            if (mi.LoadStackP)
            {
                if (StkTrapLog != null && StkTrapLog.Count < 40)
                    StkTrapLog.Add("--- stackP<- @" + addr.ToString("X3") + " CPi=" + InstructionCount
                        + "   sp " + _stackP + " -> " + (_yBus & 0xf) + "   (Ybus=" + _yBus.ToString("X4") + ")   "
                        + mi.Disassemble(-1));
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

            // ---- Next instruction address ----
            // The WCS stores INIA's low nibble COMPLEMENTED (TechRef Fig 2.6); the true
            // successor is (rawINIA XOR 0x00F).  How the modifier merges depends on the
            // dispatch type (mesa IB dispatch replaces bit-fields rather than OR-ing).
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
            _tpc[_task] = nia;

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

            // Bank<- takes effect one instruction late: the write at N leaves N+1 still
            // fetching the old bank, N+2 the new one.
            if (_bankChangePending > 0 && --_bankChangePending == 0)
                _execBank = _bankTarget;

            _cycle++;
            if (_cycle > 3) _cycle = 1;   // no tasks on Daybreak; c1/c2/c3 is memory timing
        }
    }
}
