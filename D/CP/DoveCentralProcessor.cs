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
        private IBState _ibPtr = IBState.Empty;
        private int _niaModType;        // 0=Normal (OR), 1=IBDispatch (replace [4-7], OR [8-11]), 2=IBRefillTrap (replace [0-3])
        private static readonly IBState[] _nextIBPtr = { IBState.Empty, IBState.Empty, IBState.Word, IBState.Byte };

        // Memory latches.
        private int _mar;
        private ushort _marLowSplice;   // the spliced MAR low-16 (for address-capture register writes)

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
        public List<string> ReadLog;  // if set, rolling log of <-MD reads of the IORegion (word >= 0x50000)
        public List<string> OpLog;    // if set, logs each dispatched mesa opcode (IBDisp ibFront)
        public List<string> StackLog; // if set, logs stack push/pop with stackP + value (to trace @BLTL arg build)
        public List<string> R0Log;    // if set, logs every R0(TOS) change (to trace TOS<-value / TOS<-STK writes)
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
            _mInt = false;
            _timerInt = false;
            _timerCounter = 0;
            _ibFront = 0;
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
            Microinstruction mi = Fetch(_execBank, addr);

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
                    case 0xA:   // <-ErrnIBnStkp: X[8-9]=trap, X[10-11]=~ibPtr, X[12-15]=~stackP
                        _xBus = (ushort)(((_trapCode & 3) << 6) | (((~(int)_ibPtr) & 0x3) << 4) | ((~_stackP) & 0xf));
                        _trapCode = 0;   // read-to-clear
                        break;
                    case 0xB:   // <-RH
                        _xBus = _rh[mi.rB];
                        break;
                    case 0xC:   // <-ibNA (front, no advance)
                        _xBus = _ibFront;
                        break;
                    case 0xD:   // <-ib (front, then advance: refill ibFront from IB and decrement ibPtr)
                        _xBus = _ibFront;
                        _ibFront = _ib[((int)_ibPtr) & 0x1];
                        _ibPtr = _nextIBPtr[(int)_ibPtr];
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
                _xBus = ReadWord(_mar);
                if (ReadLog != null && !_writeFrozen && _mar >= 0x50000)   // reads of the IORegion/high mailbox
                {
                    ReadLog.Add("@" + addr.ToString("X3") + " CPi=" + InstructionCount + " <-MD word " + _mar.ToString("X5") + " = " + _xBus.ToString("X4"));
                    if (ReadLog.Count > 40) ReadLog.RemoveAt(0);
                }
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
                            Note("Map<-");
                        }
                        else
                        {
                            // Daybreak MAR = the map-decode SPLICE (operator: XReadx, Xfer.mc:966-971 —
                            // the splice IS how a real address is formed from a split map entry, and it
                            // is universal, not a DLion-only quirk):
                            //   bits 16-19 <- RH[rB] (= rpHigh, the map word's low nibble)
                            //   bits  8-15 <- R[rB]_old upper byte (= rpLow, the map word's high byte)
                            //   bits  0- 7 <- Ybus low byte (the offset); no carry across byte 8.
                            // aF|3 => RorS (aF 0-3 => R[rB]) or notRxorS (aF 4-7 => ~R[rB]).
                            ushort hi = (((int)mi.aF | 0x3) == (int)AluFunction.RorS)
                                ? (ushort)(_bOld & 0xff00)
                                : (ushort)((~_bOld) & 0xff00);
                            _marLowSplice = (ushort)((_yBus & 0x00ff) | hi);
                            _mar = ((_rh[mi.rB] & 0xf) << 16) | _marLowSplice;
                            if (MarAccess != null && MarAccess.Count < 4000)
                            { long mc; MarAccess.TryGetValue(_mar, out mc); MarAccess[_mar] = mc + 1; }
                            if (mi.mem && (_alu.PgCarry ^ (((int)mi.aF & 0x1) == 1)))
                            {
                                _niaModifier |= 0x2;                     // pageCross branch
                            }
                            // Address-capture (operator: reg <- [rh,offset], e.g. MAR <- Q <- [rhPC,Q+0]):
                            // a c1 MAR<- that also writes a register captures the SPLICED real address
                            // (the MAR low-16), not the plain ALU result -- this is how PC ends up holding
                            // the real code pointer (@AAE Q<-0xB1C6 -> @33F R5<-Q).  aD=0 writes Q; aD 2/3
                            // write R[rB].  (A c3 <-MD register write instead loads MD, handled above.)
                            if (mi.aD == 0) _alu.Q = _marLowSplice;
                            else if (mi.aD == 2 || mi.aD == 3) _alu.R[mi.rB] = _marLowSplice;
                        }
                        break;
                    case 2:
                        if (WriteWord != null) WriteWord(_mar, _yBus);   // MDR<-
                        if (WriteLog != null && !_writeFrozen)
                        {
                            WriteLog.Add("@" + addr.ToString("X3") + " CPi=" + InstructionCount + " MDR<- word " + _mar.ToString("X5") + " = " + _yBus.ToString("X4"));
                            if (WriteLog.Count > 48) WriteLog.RemoveAt(0);
                        }
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
                    case YDispBrFunction.XRefBr: _niaModifier |= (_xBus & 0x10) >> 4; break;
                    case YDispBrFunction.NibCarryBr: if (_alu.NibCarry) _niaModifier |= 1; break;
                    case YDispBrFunction.XDisp: _niaModifier |= (_xBus & 0xf); break;
                    case YDispBrFunction.YDisp: _niaModifier |= (_yBus & 0xf); break;
                    case YDispBrFunction.XC2npcDisp:
                        _niaModifier |= (_xBus & 0xc) | (_cycle == 2 ? 0x2 : 0x0) | (_pc16 ? 0x0 : 0x1);
                        break;
                    case YDispBrFunction.YIODisp:
                        // Ethernet/IO dispatch; the IORegion side isn't modelled yet.
                        _niaModifier |= (_yBus & 0xc);
                        Note("YIODisp");
                        break;
                    case YDispBrFunction.XwdDisp: _niaModifier |= (_xBus & 0x60) >> 5; break;
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
                            }
                            else
                            {
                                // Normal dispatch: ibFront replaces INIA[4-7] and ORs INIA[8-11].
                                if (OpLog != null && OpLog.Count < 200)
                                    OpLog.Add("@" + addr.ToString("X3") + " CPi=" + InstructionCount + " OPCODE=0x" + _ibFront.ToString("X2") + " (PC R5=" + _alu.R[5].ToString("X4") + " ibPtr=" + _ibPtr + " ib=[" + _ib[0].ToString("X2") + "," + _ib[1].ToString("X2") + "])");
                                _niaModifier |= _ibFront;
                                _niaModType = 1;   // IBDispatch
                                _ibFront = _ib[((int)_ibPtr) & 0x1];
                                _ibPtr = _nextIBPtr[(int)_ibPtr];   // DecrementIBPtr
                            }
                            Note("IBDisp");
                        }
                        break;
                    case 0x6:   // LoadIB: fill the instruction buffer from the X bus (2 opcode bytes).
                        {
                            bool loadIBPtr1 = (mi.fSfZ == FunctionSelectFZ.fzNorm && mi.fZ == 0x1);
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
            if (mi.LoadStackP) _stackP = (_yBus & 0xf);

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
                if ((nia & 0x10) == 0) _link[mi.LinkAddress] = nia & 0xf;   // link write
                else _niaModifier |= _link[mi.LinkAddress];                 // link read
            }

            // Watch: R5(PC) / RH5(rhPC) ever landing a code-address-like value (the XCode-tail signature).
            if (R5Log != null && R5Log.Count < 60)
            {
                if (_rh[5] != _rh5old && _rh[5] != 0)
                    R5Log.Add("@" + addr.ToString("X3") + " RH5 " + _rh5old.ToString("X") + "->" + _rh[5].ToString("X") + " (R5=" + _alu.R[5].ToString("X4") + " X=" + _xBus.ToString("X4") + " CPi=" + InstructionCount + ")  " + mi.Disassemble(-1));
                if (_alu.R[5] != _r5old && _alu.R[5] > 0x0100)
                    R5Log.Add("@" + addr.ToString("X3") + " R5 " + _r5old.ToString("X4") + "->" + _alu.R[5].ToString("X4") + " (RH5=" + _rh[5].ToString("X") + " Y=" + _yBus.ToString("X4") + " CPi=" + InstructionCount + ")  " + mi.Disassemble(-1));
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
