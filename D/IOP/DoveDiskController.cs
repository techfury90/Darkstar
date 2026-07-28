using System;
using D.IO;

namespace D.IOP
{
    /// <summary>
    /// Behavioral model (Dove_RigidDisk_8X305_Reference.md §8.1 strategy A) of the Xerox 6085
    /// Rigid Disk Controller: the 8X305 command/status register pair at I/O 0x0214 plus the
    /// AM2942 DMA + FIFO at 0x0200-0x0216.  Emulates the REGISTER EFFECTS, not the 8X305 opcodes.
    ///
    /// Protocol (validated against a live IOP-firmware trace):
    ///   init 0x0200&lt;-6 (AM2942 mode 3); 0x0214&lt;-0 (go idle);
    ///   program AM2942 (0x0210 dir, 0x0208/0x020A addr, 0x020C count) + 0x0216 StartDMA;
    ///   0x0214&lt;-1 Fetch DOB   (status 0x41 done + RDiskCtlrIntr);
    ///   0x0214&lt;-2 Execute DOB (status 0x42 done / 0xC2 error + RDiskCtlrIntr);
    ///   0x0214&lt;-3 Store DOB   (status 0x43 done + RDiskCtlrIntr);
    ///   program AM2942 + StartDMA to move the updated DOB back to memory.
    ///
    /// The DOB is a 34-word block; multi-byte CHS/count/geometry fields are Mesa ByteSwapped.
    ///
    /// FIELD MAP (field-engineer manual, every entry cross-checked against a full install trace
    /// of 33,933 DOBs -- the values in brackets are what this machine actually sends):
    ///    0  CRC syndrome                                  [0000 always]
    ///    1  reserved, must be zero                        [0000 always]
    ///    2  number of sectors to transfer, NEGATIVE       [-1 -2 -3 -16 -19 -32 -128]
    ///    3  max sector number                             [16]
    ///    4  microcode revision level + heads              [rev 0, 8 heads]
    ///    5  cylinders                                     [960]
    ///    6  inv/non-inv flag + first sector number/track
    ///    7  reduce write current cylinder                 [FFFF = never]
    ///    8  precomp cylinder                              [32767 = never]
    ///    9  SECTOR LENGTH: -1 normal, -2 long block       [-1 in every DOB of the install]
    ///       (rest of the word is 0x00).  -2 is for DISK RECOVERY UTILITIES: the longer block
    ///       exposes the sector's CRC/ECC bytes so the utility can inspect and correct them.
    ///       It pairs with word 0 -- read the syndrome to find a bad sector, then re-read at
    ///       -2 to see the check bytes.  WE DO NOT DECODE THIS, and supporting it means
    ///       modelling ECC bytes we do not store, not just honouring the field.  Harmless
    ///       while it stays -1; a recovery utility would be served a plain 512 bytes with no
    ///       check bytes and told it succeeded.  We are at least self-consistent on the other
    ///       half: word 0 is passed through untouched, so the syndrome reads back as the 0 the
    ///       guest sent -- "no CRC error", which is true of a platter that cannot bit-rot.
    ///   10  header error type      11  label error type   12  data error type
    ///   13  (see below)            14  current cylinder   15  always 1  [0001 always]
    ///   16  header cylinder        17  header head/sector PACKED into one word
    ///   18  reserved 1  [0000]     19  reserved 2  [0000]
    ///   20  drive/controller status                       21  operation to be done
    ///   22  number of tracks to format, 2's complement    [-1 on all 7,680 op-1 formats]
    ///   23-32  the 10-word label image                    33  padding
    ///
    /// Word 22 is meaningful ONLY for op 1 and carries leftover buffer content otherwise --
    /// 0x8900 on every op 2/3, co-occurring exactly with word 33 = 0x4F58 across 19,270 DOBs.
    /// Read it anywhere but FormatTracks and you are reading somebody else's stale field.
    ///
    /// The manual's table shows a padding BYTE before the label image and another after it,
    /// which would put the label at byte 47 rather than word 23 (byte 46).  Ours is at word 23
    /// and the machine agrees: FPLow/FPHi decode to the correct page for all 122,880 pages,
    /// including across the 65,536 boundary, which a one-byte shift could not survive.  The
    /// leftover 0x4F in word 33 would also have to be part of bootCLHi, which is nonsense as a
    /// chain link and unremarkable as padding.  Treat the table as drawn loosely there.
    ///
    /// Word 13 is absent from the manual table as transcribed, which would put current cylinder
    /// at 13 and always-1 at 14.  The machine says otherwise and the code follows the machine:
    /// word 13 takes two values (one of them an error byte), word 14 takes 960 -- a cylinder --
    /// and word 15 is a constant 1.  So a fourth error word sits at 13; we call it LastError.
    ///
    /// Words 3/4/5 mean the GUEST tells us the geometry and we ignore it in favour of the
    /// compile-time Micropolis1325 constants.  They agree only because the drive is configured
    /// to match; comparing them would be a cheap assertion, and is the natural place to drive
    /// the other supported drive types from.
    /// StartDMA performs the actual byte movement between the physical DRAM (bypasses the IOP map)
    /// and either the DOB image (word-count &le; 34) or a 512-byte data page (word-count 256).
    /// The two RDC interrupts (RDiskCtlrIntr, RDiskDmaIntr') are surfaced to DoveIOPIO which edges
    /// them onto the slave 8259 (IR3 / IR2 -&gt; master IR5).  The CP-side Store wake is done by the
    /// IOP firmware ISR (it harvests the DOB and posts mesaClientCondition) -- NOT here.
    /// </summary>
    public sealed class DoveDiskController
    {
        private readonly DoveIOPMemory _mem;
        private readonly Micropolis1325 _disk;

        // AM2942 DMA state.  _dmaAddr is a PHYSICAL DRAM byte address (bit0 = 0, word aligned).
        private int _dmaAddr, _dmaCount;     // _dmaCount = raw value written to 0x020C
        private int _dmaDir;                 // 0x0210 bit0: 1 = mem->disk (read from memory), 0 = disk->mem

        // 8X305 command/status.
        private byte _status;                // read at 0x0214: e d ss xx rr
        private int _lastCc = 0;             // last command 00/01/02/03
        private bool _storePending;          // cc=3 issued, waiting for the FIFO->mem StartDMA

        // Interrupt lines to DoveIOPIO (edge-raised onto the slave 8259; ctlr = read-clear).
        public bool CtlrInt;                 // RDiskCtlrIntr  -> slave IR3
        public bool DmaInt;                  // RDiskDmaIntr'  -> slave IR2

        // The 34-word DOB image (host-order words, exactly as they sit in DRAM, big-endian).
        private readonly ushort[] _dob = new ushort[34];
        private readonly byte[] _dataBuf = new byte[512];   // staging for one data page

        // Multi-sector transfer, driven by diskMinusSectorCount.  One executeDOB covers N
        // sectors, but the data moves as N SEPARATE page-DMAs (DiskDove.asm's DiskDMADataXfer
        // loops DoDiskDMA, one 256-word page each, blocking on that page's interrupt).  So the
        // controller cannot do the whole transfer at command time: it consumes one sector per
        // page-DMA and only reports completion when the sector count reaches zero -- which is
        // what produces N DMA interrupts and exactly ONE controller interrupt.
        // Multi-sector streaming state.  DiskDove.asm:1101-1130 (DiskDMADataXfer) issues ONE
        // StartDMA per 512-byte page, blocks on %WaitForInterrupt() with no timeout, then
        // re-arms the AM2942 and goes round again until diskPageCount hits zero.  So the
        // controller transfers a single sector per StartDMA, must raise the DMA interrupt after
        // EVERY one, and raises the controller-completion interrupt only when the run ends.
        // The FIRMWARE advances the source pointer (gated by incrementDataPtr), so each
        // StartDMA already carries the right address -- the controller must not compute one.
        private bool _xferActive;
        private int _xferOp, _xferCyl, _xferHead, _xferSector, _xferRemaining;
        private bool _xferError;
        private int _xferLastCyl, _xferLastHead, _xferLastSector;   // ends ON the last sector
        private ushort _xferL5, _xferL6;                            // run's base label words
        private int _xferIndex;                                     // sector number within the run
        private int _dataDmaAddr; private bool _dataDmaArmed;   // read data-page DMA armed pre-Execute, emitted at cc=2
        private int _dobAddr = -1;   // physical address of the DOB (last <=34-word DMA target) -- IOCB is just below it
        // DIAGNOSTIC gate: DOVE_NO_DATA_DMA=1 suppresses the 512-byte data-page transfer (both directions)
        // to test whether the data DMA write into mds0 (phys 0x95800) is what corrupts the germ frame heap.
        private readonly bool _noDataDma =
            System.Environment.GetEnvironmentVariable("DOVE_NO_DATA_DMA") == "1";
        // Blank-page read model.  INVARIANT (operator, not to be questioned): the drive is MFM and
        // "unformatted" means NO sector headers exist, so EVERY read fails by construction -- there is
        // no low-level format.  DEFAULT = not-found.  A real machine boots straight past an all-failing
        // unformatted drive, so the germ MUST tolerate the not-found; the bug is that my emulator turns
        // that tolerable "no format" result into a fatal AsynchronousVMIOError.  (DOVE_DISK_LLFORMAT=1
        // is an experiment-only escape hatch and violates the invariant; do not use for boot.)
        private readonly bool _notFoundOnBlank =
            System.Environment.GetEnvironmentVariable("DOVE_DISK_LLFORMAT") != "1";

        public System.Collections.Generic.List<string> Log;   // optional diagnostic
        public long HostClock;

        /// <summary>Optional per-operation trace sink (see DOVE_RDC_LOG).</summary>
        public System.IO.TextWriter LogWriter;

        /// <summary>
        /// Clocks between a command being accepted and its interrupt being asserted.
        /// The controller must NOT complete inside the OUT that writes the command
        /// register: the driver's sequence is "issue command, then wait for the
        /// interrupt", so asserting it synchronously means the 80186 takes the interrupt
        /// at the next instruction boundary -- before the wait is entered -- and a wait
        /// that arms rather than checking a latch never sees it.  Reads as a missed
        /// interrupt from the driver's side.  (Exactly the fault found in the 82586.)
        /// Status is still updated immediately so the polling path -- which is what the
        /// formatter uses, and which works -- is unaffected.
        /// </summary>
        public int InterruptDelayClocks = 400;      // ~50us at 8 MHz

        private int _ctlrIntDelay = -1, _dmaIntDelay = -1;

        private void ScheduleCtlrInt() { _ctlrIntDelay = InterruptDelayClocks; }
        private void ScheduleDmaInt() { _dmaIntDelay = InterruptDelayClocks; }

        /// <summary>Advance the controller's own timing; called from the IOP tick.</summary>
        // One-shot "parked" sample.  The go-idle dump fires at OUT 0214h -- BEFORE the window
        // that fails -- and every register is correct there (BX=0050, BP=0002, ES:DI=03E3:0002 =
        // the IOCB).  To see what the handler did INSIDE the window we need a sample after the
        // machine goes quiet, which is the only moment that distinguishes an aborted core from
        // execution that continued with corrupted state.
        private long _idleClocks;
        private bool _parkedDumped;
        private int _parkedCount;

        public void Tick(int clocks)
        {
            if (!_parkedDumped && LogWriter != null && DobOps > 0)
            {
                _idleClocks += clocks;
                // Sample repeatedly: SystemIdle is a BUSY LOOP (STI / poke the arbiter /
                // JMP SystemLoop), so an Opie that is merely idle still moves.  A CS:IP that is
                // identical across samples means the IOP is genuinely stuck, not idling.
                if (_idleClocks > 40000000)
                {
                    _idleClocks = 0;
                    if (++_parkedCount >= 4) _parkedDumped = true;
                    DumpFcb("PARKED sample " + _parkedCount);
                }
            }
            if (_ctlrIntDelay >= 0)
            {
                _ctlrIntDelay -= clocks;
                if (_ctlrIntDelay <= 0) { _ctlrIntDelay = -1; CtlrInt = true; }
            }
            if (_dmaIntDelay >= 0)
            {
                _dmaIntDelay -= clocks;
                if (_dmaIntDelay <= 0)
                {
                    _dmaIntDelay = -1;
                    // If the previous DMA interrupt has not been acknowledged (read of
                    // 0x0210), the edge-triggered raise produces NO new edge and this
                    // completion is silently lost -- the DMA task then waits forever on its
                    // no-timeout WaitForInterrupt and the page loop stops after one page.
                    if (DmaInt) DmaIntLost++;
                    DmaInt = true;
                    DmaIntRaised++;
                }
            }
        }

        /// <summary>DMA-interrupt bookkeeping: raised, acknowledged, and lost to a missing edge.</summary>
        public long DmaIntRaised, DmaIntAcked, DmaIntLost;

        /// <summary>Operations executed and where the head last was, for the status display.</summary>
        public long DobOps;
        public int LastCyl, LastHead, LastSector;

        // Bisect gate: DOVE_RDC_STUB=1 replicates the pre-session permissive stub EXACTLY
        // (fake done+ctlrInt on cc, DmaInt-once on StartDMA, NO memory access, NO DOB round-trip).
        // Used to isolate whether a boot regression comes from construction/fields vs behavior.
        private readonly bool _stub =
            System.Environment.GetEnvironmentVariable("DOVE_RDC_STUB") == "1";

        public DoveDiskController(DoveIOPMemory mem, Micropolis1325 disk)
        {
            _mem = mem;
            _disk = disk;
        }

        // ---- physical DRAM access (AM2942 bypasses the 80186 map; index _system directly) ----
        private byte[] Sys { get { return _mem.SystemRaw; } }
        private int Mask { get { return DoveIOPMemory.SystemSize - 1; } }
        private ushort RdWord(int phys) { var s = Sys; return (ushort)((s[phys & Mask] << 8) | s[(phys + 1) & Mask]); }
        private void WrWord(int phys, ushort v) { var s = Sys; s[phys & Mask] = (byte)(v >> 8); s[(phys + 1) & Mask] = (byte)v; }
        private static ushort Bswap(ushort v) { return (ushort)((v << 8) | (v >> 8)); }

        // AM2942 word-count decode: value = two's-complement-8 of N, shifted left 1, on bits 8-1.
        private int WordCount()
        {
            int neg = (_dmaCount >> 1) & 0xFF;
            return neg == 0 ? 256 : (0x100 - neg);
        }

        // ---- port interface (called from DoveIOPIO) ----
        /// <summary>
        /// Port-level trace to the log FILE (the in-memory Log is capped at 800 and is a
        /// different facility).  Bounded, because a full install issues millions of port
        /// operations -- but a boot attempt issues few, so the cap is generous enough to cover
        /// the ROM's entire rigid-disk sequence from reset.  Set DOVE_RDC_PORTLOG to change it;
        /// 0 disables.  This exists because the DOB trace shows only EXECUTED DOBs, and the boot
        /// ROM stalls at MP 0149 after two restores without ever issuing a read -- whatever it
        /// checks is a register poll, invisible at the DOB level.
        /// </summary>
        private int _portLogged;
        private readonly int _portLogLimit =
            int.TryParse(System.Environment.GetEnvironmentVariable("DOVE_RDC_PORTLOG"), out var _pl) ? _pl : 4000;

        private void LogPort(string s)
        {
            _idleClocks = 0;
            if (LogWriter == null || _portLogged >= _portLogLimit) return;
            _portLogged++;
            try { LogWriter.WriteLine(s); } catch { LogWriter = null; }
        }

        /// <summary>
        /// Dump the disk FCB (segment DISKIOR, located at 0x03A70, 0xB8 bytes -- ROMSysB2.mp2).
        /// Called on the go-idle command, which is exactly where the boot ROM parks at MP 0149.
        ///
        /// NOTE the little-endian reader: RdWord is big-endian because the DOB sits in DRAM that
        /// way, but the FCB is an ordinary 80186 structure written by IOP code.
        ///
        /// The decisive field is diskCurrentClient (0x03AB0): 0 = Mesa branch, 2 = IOP branch.
        /// The ROM registers as the IOP client, so a 0 here means the handler took the Mesa path
        /// and notified diskMesaClientCondition -- which the ROM never initialised, so the notify
        /// is discarded at the nonNilPtr test and the ROM sleeps in its noTimeout wait forever.
        /// </summary>
        /// <summary>
        /// Supplied by the machine so the FCB dump can record IOP register state.  Real-mode
        /// 80186 has no memory faults, so a bad ES:DI reads garbage and runs on -- "stopped at
        /// the instruction" is impossible on hardware.  The write evidence points at register
        /// corruption instead: the DIRECT-addressed store landed, both REGISTER-INDIRECT stores
        /// did not.  Expected at the stall: BX = 0x0050 (OFFSET diskFCB.rd0).
        /// </summary>
        public Func<string> CpuState;

        private const int FcbBase = 0x03A70, FcbLen = 0xB8;
        private int _fcbDumps;

        /// Route through DoveIOPMemory, NOT the Sys/DRAM array: IOP 0x00000-0x03FFF is the 16 KB
        /// LOCAL SRAM, a different array entirely, and the FCB at 0x03A70 lives there.  Reading it
        /// out of Sys returns DRAM fill (0xEE/0xBB) and looks exactly like an uninitialised FCB.
        private ushort RdLE(int a) { return (ushort)(_mem.ReadByte(a) | (_mem.ReadByte(a + 1) << 8)); }
        private byte RdB(int a) { return _mem.ReadByte(a); }

        private void DumpFcb(string why)
        {
            if (LogWriter == null || (_fcbDumps >= 8 && !why.StartsWith("PARKED"))) return;
            if (why.StartsWith("PARKED") && _parkedCount > 1 && CpuState != null)
            {
                try { LogWriter.WriteLine("  " + why + "  CPU " + CpuState()); } catch { LogWriter = null; }
                return;
            }
            _fcbDumps++;
            try
            {
                var sb = new System.Text.StringBuilder();
                sb.Append("  FCB (").Append(why).Append(") @IOP").Append(HostClock).AppendLine();
                if (CpuState != null) sb.Append("    CPU ").Append(CpuState()).AppendLine();
                sb.Append("    diskCurrentClient=").Append(RdLE(0x03AB0).ToString("X4"))
                  .Append(RdLE(0x03AB0) == 2 ? " (IOP branch)" : RdLE(0x03AB0) == 0 ? " (MESA BRANCH <<<)" : " (?)")
                  .Append("  handlerState=").Append(RdLE(0x03AAE).ToString("X4"))
                  .Append("  startMesa=").Append(RdLE(0x03AAA).ToString("X4"))
                  .Append("  startIOP=").Append(RdLE(0x03AAC).ToString("X4")).AppendLine();
                sb.Append("    rd0.diskMesaNext=").Append(RdLE(0x03AC2).ToString("X4"))
                  .Append(RdLE(0x03AC2) != 0 ? "  <<< NON-ZERO (a)" : "  (0, ok)")
                  .Append("  IOPNextLow/High=").Append(RdLE(0x03ACC).ToString("X4")).Append('/')
                  .Append(RdLE(0x03ACE).ToString("X4")).AppendLine();
                sb.Append("    IOPcond.handlerID=").Append(RdB(0x03ADE).ToString("X2"))
                  .Append("  conditionPtr=").Append(RdLE(0x03AE0).ToString("X4"))
                  .Append((RdLE(0x03AE0) & 0x8000) == 0 ? "  <<< no nonNilPtr (d)" : "  (nonNilPtr set, ok)")
                  .Append("  clientMask=").Append(RdLE(0x03AE2).ToString("X4")).AppendLine();
                sb.Append("    cond work/DMADone/DMAWork=").Append(RdLE(0x03A98).ToString("X4")).Append('/')
                  .Append(RdLE(0x03A96).ToString("X4")).Append('/').Append(RdLE(0x03A94).ToString("X4"))
                  .Append("  shadow status/cmd=").Append(RdB(0x03AB6).ToString("X2")).Append('/')
                  .Append(RdB(0x03AB7).ToString("X2"))
                  .Append("  dmaStatus=").Append(RdLE(0x03ABA).ToString("X4")).AppendLine();
                sb.Append("    unexpected ctlrInt=").Append(RdLE(0x03ABC))
                  .Append("  unexpected dmaInt=").Append(RdLE(0x03ABE)).AppendLine();
                // BOOTSTRAPIOR = 0x03E00.  diskComplete is set TRUE immediately after the
                // notify, so FALSE here while the command visibly completed proves the handler
                // died between the two-way error test and NotifyClientCondition -- the window
                // where BP must still hold 0 or 2 for the indexed jumps.
                sb.Append("    BOOT: diskComplete=").Append(RdB(0x03E86).ToString("X2"))
                  .Append(RdB(0x03E86) == 0 ? "  <<< FALSE: notify never ran" : "  (TRUE, notify ran)")
                  .Append("  diskInProgress=").Append(RdB(0x03E87).ToString("X2"))
                  .Append("  diskError=").Append(RdB(0x03E82).ToString("X2")).AppendLine();
                sb.Append("    BOOT: ctlrErrType=").Append(RdB(0x03E84).ToString("X2"))
                  .Append("  dmaErrType=").Append(RdB(0x03E85).ToString("X2"))
                  .Append("  ROMcondition[0x03E30]=").Append(RdLE(0x03E30).ToString("X4"))
                  .Append((RdLE(0x03E30) & 0x8000) != 0 ? " (ROM PARKED, waiting)"
                          : RdLE(0x03E30) == 1 ? " (notified, nobody waiting)" : " (neither)")
                  .AppendLine();
                // The locator's 0x03A70 is image-relative; the runtime address differs (that
                // region reads as 0xBB fill).  Find the FCB empirically: the ROM writes
                // diskIOPNextHigh = (0x51<<8)|8 into it, so scan for that signature.
                if (_fcbDumps == 1)
                {
                    var hits = new System.Text.StringBuilder("    SCAN for 0x5108 (diskIOPNextHigh):");
                    int found = 0;
                    for (int a = 0; a < 0x40000 && found < 12; a += 2)
                    {
                        if (RdLE(a) == 0x5108) { hits.Append(" 0x").Append(a.ToString("X5")); found++; }
                    }
                    if (found == 0) hits.Append(" none");
                    // Also report where low memory is actually populated.
                    hits.AppendLine().Append("    non-0xBB spans in 0x00000-0x08000:");
                    int runStart = -1;
                    for (int a = 0; a <= 0x8000; a++)
                    {
                        bool live = a < 0x8000 && RdB(a) != 0xBB;
                        if (live && runStart < 0) runStart = a;
                        else if (!live && runStart >= 0)
                        {
                            if (a - runStart >= 16) hits.Append(" 0x").Append(runStart.ToString("X5"))
                                                        .Append("-0x").Append((a - 1).ToString("X5"));
                            runStart = -1;
                        }
                    }
                    LogWriter.WriteLine(hits.ToString());
                }
                // THE DI DIFFERENTIAL.  DiskReadLabelAndData does three stores, and only the
                // middle one goes through DI:
                //     MOV bootDeviceIORSpace...diskPageCount, 1            (direct)
                //     MOV [DI].diskOperation, ReadDiskLabelAndData         (via DI)  <-- 6
                //     MOV bootDeviceIORSpace...diskDataXferDirection, read (direct)
                // DI must survive %WaitForCondition on the bootstrap stack.  operation still 0
                // (restore) WHILE the direct-addressed direction reads "read" proves the direct
                // stores landed and the DI-relative one did not -- i.e. DI was not preserved
                // across the blocking wait.  That also explains the second "restore": it is not
                // a retry, it is the READ executing with a stale operation byte.
                sb.Append("    DI-DIFF: DOB.diskOperation[0x03EB8]=").Append(RdB(0x03EB8).ToString("X2"))
                  .Append(RdB(0x03EB8) == 6 ? " (6 = ReadDiskLabelAndData, ok)"
                          : RdB(0x03EB8) == 0 ? "  <<< 0 = still Recalibrate: DI store LOST" : " (?)")
                  .Append("  IOCB.dataXferDirection[0x03E7C]=").Append(RdB(0x03E7C).ToString("X2"))
                  .Append("  pageCount[0x03E7A]=").Append(RdLE(0x03E7A).ToString("X4")).AppendLine();
                // OPIE SCHEDULING.  %ContinueAtSystemLevel is a plain round-robin yield --
                // InsertInSystemQueue then dispatch -- so the disk handler should come back
                // within one trip through systemQueue.  Three readings split the failure:
                //   queue head 0xFF (empty)        -> InsertInSystemQueue never linked it
                //   head=2, task waitForSystemState -> enqueued, dispatcher never ran it
                //   task systemState, not running   -> dispatched, IRET went to a bad address
                //                                      (the [mapImage][ES][IP][CS][FLAGS] frame
                //                                       misaligned by a missing map-image push)
                sb.Append("    OPIE: diskTask.taskState[0x03A7E]=").Append(RdLE(0x03A7E).ToString("X4"))
                  .Append("  taskSP[0x03A78]=").Append(RdLE(0x03A78).ToString("X4"))
                  .Append("  taskCondition[0x03A74]=").Append(RdLE(0x03A74).ToString("X4"))
                  .Append("  taskQueue[0x03A70]=").Append(RdLE(0x03A70).ToString("X4")).Append('/')
                  .Append(RdLE(0x03A72).ToString("X4")).AppendLine();
                sb.Append("    OPIE: systemQueue.head[0x03B30]=").Append(RdLE(0x03B30).ToString("X4"))
                  .Append((RdB(0x03B30) == 0xFF || RdB(0x03B31) == 0xFF) ? "  <<< EMPTY (nilHandlerID)" : "")
                  .Append("  dmaTask.taskState[0x03A90]=").Append(RdLE(0x03A90).ToString("X4")).AppendLine();
                sb.Append("    raw:");
                for (int i = 0; i < FcbLen; i++)
                {
                    if ((i % 16) == 0) sb.AppendLine().Append("      +").Append(i.ToString("X2")).Append(' ');
                    sb.Append(RdB(FcbBase + i).ToString("X2")).Append(' ');
                }
                LogWriter.WriteLine(sb.ToString());
            }
            catch { LogWriter = null; }
        }

        public byte ReadReg(ushort port)
        {
            if (_stub)
            {
                byte sv;
                switch (port)
                {
                    case 0x0214: sv = _status; CtlrInt = false; break;   // read clears RDiskCtlrIntr
                    case 0x0210: sv = 0x00; break;                        // DMA status idle (does NOT clear DmaInt)
                    case 0x0204: sv = (byte)_dmaCount; break;
                    case 0x0206: sv = (byte)(_dmaAddr >> 1); break;
                    default:     sv = 0x00; break;
                }
                if (Log != null && Log.Count < 800)
                    Log.Add("R  0x" + port.ToString("X4") + " -> 0x" + sv.ToString("X2") + " @IOP" + HostClock);
                return sv;
            }
            byte v;
            switch (port)
            {
                case 0x0214: v = _status; CtlrInt = false; break;       // read-clears RDiskCtlrIntr
                case 0x0210: v = DmaStatus(); DmaInt = false; DmaIntAcked++; break;   // read-clears RDiskDmaIntr'
                case 0x0204: v = (byte)((-WordCount()) & 0xFF); break;  // 2's-comp count on bits 8-1... (poll rarely used)
                case 0x0206: v = (byte)(_dmaAddr >> 1); break;          // addr bits 8-1
                default:     v = 0x00; break;
            }
            if (Log != null && Log.Count < 800)
                Log.Add("R  0x" + port.ToString("X4") + " -> 0x" + v.ToString("X2") + " @IOP" + HostClock);
            LogPort("  PORT R 0x" + port.ToString("X4") + " -> 0x" + v.ToString("X2")
                    + (port == 0x0214 ? "  (status)" : port == 0x0210 ? "  (dma status)" : "")
                    + " @IOP" + HostClock);
            return v;
        }

        public void WriteReg(ushort port, ushort value)
        {
            if (Log != null && Log.Count < 800)
                Log.Add("W  0x" + port.ToString("X4") + " <- 0x" + value.ToString("X4") + " @IOP" + HostClock
                    + (port == 0x0214 ? "  (cmd cc=" + (value & 3) + ")" : ""));
            LogPort("  PORT W 0x" + port.ToString("X4") + " <- 0x" + value.ToString("X4")
                    + (port == 0x0214 ? "  (cmd cc=" + (value & 3) + ")" : "")
                    + " @IOP" + HostClock);
            if (port == 0x0214 && (value & 3) == 0) DumpFcb("go-idle");
            if (_stub)
            {
                switch (port)
                {
                    case 0x0214:
                        int scc = value & 0x3;
                        _status = (byte)scc;
                        if (scc != 0) { _status |= 0x40; ScheduleCtlrInt(); }   // fake done + RDiskCtlrIntr
                        break;
                    case 0x0208: _dmaAddr = (value & 0x7FFF) << 9; break;
                    case 0x020A: _dmaAddr = (_dmaAddr & ~0x1FE) | ((value & 0xFF) << 1); break;
                    case 0x020C: _dmaCount = value; break;
                    case 0x0210: _dmaDir = value & 1; break;
                    case 0x0216: ScheduleDmaInt(); break;                    // StartDMA -> DmaInt
                    default: break;
                }
                return;
            }
            switch (port)
            {
                case 0x0200:                                            // AM2942 control register
                    // %ResetDiskDMA writes mode 3 here right after pulsing the reset line.
                    _dmaWasReset = (value & 0xFF) == 0x06;
                    break;
                case 0x0214: Command((byte)value); break;
                // AM2942 address halves are order-independent: the firmware writes 0x020A BEFORE 0x0208,
                // so 0x0208 must set ONLY addr[23:9] and preserve addr[8:1] (the "auto-reinit" is the DMA
                // word-COUNT counters, not the address low bits).  Reconstructs the validated DOB @0x9547C.
                case 0x0208: _dmaAddr = (_dmaAddr & 0x1FE) | ((value & 0xFFFE) << 8); break;   // addr[23:9]
                case 0x020A: _dmaAddr = (_dmaAddr & ~0x1FE) | (value & 0x1FE); break;          // addr[8:1]
                case 0x020C: _dmaCount = value; break;                  // word count (2's-comp << 1)
                case 0x0210: _dmaDir = value & 1; break;                // direction bit
                case 0x0216: StartDma(); break;                         // StartDMA -> move the bytes now
                default: break;
            }
        }

        // AM2942 DMA status (0x0210): idle after each transfer.  We complete synchronously, so
        // RunSM=0 and EndOfXfer' asserts "done"; report a benign idle byte.
        private byte DmaStatus()
        {
            // Active-low ("Bar") signals: 0 = the good/quiescent condition.  0x00 therefore
            // reads as "no error, FIFO empty, transfer ended", which is what every one of the
            // firmware's tests needs to proceed (DoDiskCommand entry, CheckFIFOEmpty, and the
            // per-page check in DoDiskDMA all require their bits to be 0).
            //
            // The exception is the reset signature: TestForDiskDMAReset does
            // AND AL,0x33 / CMP AL,0x20 (diskDMAFIFOFullBar), so immediately after a
            // %ResetDiskDMA the status must read 0x20 or the firmware concludes the reset did
            // not take.  Only the recovery choreography looks at this, which is why a static
            // 0x00 has been survivable so far.
            return _dmaWasReset ? (byte)0x20 : (byte)0x00;
        }

        private bool _dmaWasReset;

        private void Command(byte cmd)
        {
            _lastCc = cmd & 0x3;
            // A new command supersedes any transfer still in flight.  Leaving one latched
            // would suppress every later completion interrupt -- which is exactly what turned
            // a stalled multi-sector write into an endless retry of a operation that
            // otherwise succeeded.
            if (_lastCc == 1 || _lastCc == 2) _xferActive = false;
            switch (_lastCc)
            {
                case 0:  // go to idle
                    _status = 0x00;
                    break;
                case 1:  // Fetch DOB into the controller (the DOB was DMA'd mem->FIFO by the prior StartDMA)
                    _status = 0x41;              // done + rr=01
                    ScheduleCtlrInt();
                    break;
                case 2:  // Execute the DOB operation
                    ExecuteDob();                // sets _status 0x42 / 0xC2, fills DOB error fields + _dataBuf
                    if (_dataDmaArmed)           // now _dataBuf is valid -> emit the read data page to mem
                    {
                        if (!_noDataDma)
                            for (int i = 0; i < 512; i++) Sys[(_dataDmaAddr + i) & Mask] = _dataBuf[i];
                        _dataDmaArmed = false;
                    }
                    // A streaming run is NOT complete yet -- FinishTransfer raises the single
                    // controller-completion interrupt once the last sector has gone.
                    if (!_xferActive) ScheduleCtlrInt();
                    break;
                case 3:  // Store DOB back (the FIFO->mem StartDMA follows and writes it out)
                    _status = 0x43;
                    _storePending = true;
                    ScheduleCtlrInt();
                    break;
            }
        }

        // StartDMA (0x0216): move the programmed word-count between physical DRAM and the FIFO,
        // discriminating DOB (count <= 34) from a 512-byte data page (count 256) by the count.
        private void StartDma()
        {
            _dmaWasReset = false;          // a transfer means we are out of the reset state
            int n = WordCount();
            if (LogWriter != null)
            {
                try { LogWriter.WriteLine("    DMA words=" + n + " dir=" + (_dmaDir == 1 ? "mem->FIFO" : "FIFO->mem")
                        + " addr=0x" + _dmaAddr.ToString("X5") + " (" + (n <= 34 ? "DOB" : "data page") + ")"); }
                catch { LogWriter = null; }
            }
            if (n <= 34)
            {
                // DOB transfer.
                _dobAddr = _dmaAddr;            // remember where the DOB lives (IOCB is just below it)
                if (_dmaDir == 1)               // mem->FIFO: ingest the DOB image
                {
                    for (int w = 0; w < 34; w++) _dob[w] = RdWord(_dmaAddr + w * 2);
                }
                else                            // FIFO->mem: write the updated DOB back
                {
                    for (int w = 0; w < 34; w++) WrWord(_dmaAddr + w * 2, _dob[w]);
                    _storePending = false;
                }
            }
            else
            {
                // Data page.  Once a multi-sector operation is running, each page-DMA moves
                // exactly one more sector and steps the address.
                // NOTE the fall-through to ScheduleDmaInt below: the firmware blocks on
                // %WaitForInterrupt() after every page, so returning early here would hang it
                // on page 2 of every run.  That is the defect that made the earlier attempt at
                // this look like "a succeeding read retried 194 times".
                if (_xferActive)
                {
                    TransferNextSector(_dmaAddr);
                }
                else if (_dmaDir == 1)          // mem->disk: capture data from memory NOW (write ops feed
                {
                    if (!_noDataDma) for (int i = 0; i < 512; i++) _dataBuf[i] = Sys[(_dmaAddr + i) & Mask];
                }
                else                            // disk->mem (read): the firmware arms this StartDMA BEFORE
                {                               // cc=2, so DON'T emit yet -- _dataBuf isn't filled until
                    _dataDmaArmed = true;       // Execute runs ReadSector.  Record the target; emit at cc=2.
                    _dataDmaAddr = _dmaAddr;
                }
            }
            ScheduleDmaInt();                   // RDiskDmaIntr' (AM2942 end-of-transfer)
        }

        // ---- DOB decode helpers ----
        // TWO byte-order conventions coexist in the DOB (DiskIOFaceDove.mesa):
        //  * Class A -- single-byte bit-fields: value in a HALF of the word, NO swap.  Mesa bit 0 = MSB,
        //    so bits 0..7 = HIGH byte (>>8), bits 8..15 = LOW byte (&0xFF).  (operation, sectorsPerTrack,
        //    headsPerCylinder, startingSectorOnTrack, writeEndCount, eccFlag, ErrorStatus.error -> >>8;
        //    currentVersion -> &0xFF.)  Verified vs memdump: op w21=0x0600->6, spt w3=0x1000->16, heads
        //    w4=0x0800->8.
        //  * Class B -- ByteSwappedWord: FULL 16-bit swap (negativeSectorCount, cylindersPerDrive,
        //    reducedWriteCylinder, preCompensationCylinder, currentCylinder, sectorValid,
        //    negativeFormatTrackCount).
        // The CHS header (ByteSwappedDiskAddress) is MIXED: cylinder(w16)=ByteSwappedWord;
        // sector(w17:0..7)=HIGH byte; head(w17:8..15)=LOW byte.  (cylinder-0 hides a wrong cyl decode at
        // the probe but corrupts every non-zero-cylinder seek -- hence the explicit split below.)
        private int Operation { get { return _dob[21] >> 8; } }

        /// <summary>
        /// diskMinusSectorCount -- DOB byte +4, i.e. WORD 2.  A two's-complement negative
        /// count of sectors, which the controller counts up to zero; it is the ONE transfer
        /// length, reused for every operation type (the type is diskOperation at byte 42 =
        /// word 21).  There is no separate format-track-count field.
        ///
        /// This was previously read from word 22, which is diskLabelError -- a status word.
        /// It read as 0 and decoded as "1", so single-sector work looked correct and every
        /// multi-sector transfer silently moved only its first sector.
        /// </summary>
        private int SectorCount()
        {
            int neg = Bswap(_dob[2]);
            if (neg == 0) return 1;
            int n = 0x10000 - neg;
            return (n >= 1 && n <= 256) ? n : 1;
        }
        /// <summary>diskStartSec -- DOB byte +12, the high byte of word 6.</summary>
        private int StartSec { get { return (_dob[6] >> 8) & 0xFF; } }
        private int HdrCyl { get { return Bswap(_dob[16]); } }
        private int HdrSector { get { return _dob[17] >> 8; } }
        private int HdrHead { get { return _dob[17] & 0xFF; } }
        private void SetErr(int word, int errByte) { _dob[word] = (ushort)((errByte & 0xFF) << 8); }   // ErrorType = hi byte
        private const int W_HeaderError = 10, W_LabelError = 11, W_DataError = 12, W_LastError = 13;
        private const int W_CurrentCyl = 14, W_DriveCtlrStatus = 20;

        // Error type bytes (§7.1).  The "not found" family is what an UNFORMATTED platter returns
        // (no address marks/headers to find) -- see DiskHeadLabeledDukeA:880-892.
        private const int ErrNone = 0x00, ErrLabelVerify = 0x23, ErrSectorNotFound = 0x81;

        /// <summary>
        /// The healthy half of DriveAndControllerStatus: dskNotWriteFault + dskNotStoredIndxMrk
        /// both TRUE.  Active-low, so these must be SET to mean "no write fault, index mark
        /// present".  dskNotTrack0 is ORed in separately by the caller (clear at cylinder 0).
        /// </summary>
        /// UNVERIFIED, so it is OFF by default: set DOVE_DRIVE_HEALTHY=1 to enable.
        ///
        /// The field ORDER is known (Disk.def, above) but the bit POSITIONS and which half of
        /// word 20 holds DiskDriveStatusRec are NOT -- 0x0A00 is inferred from an older comment
        /// in this file, not from the source.  Enabling it changed the boot ROM's behaviour not
        /// at all (2 restores then idle, identical), and if the drive status actually lives in
        /// the LOW byte then 0x0A00 is writing into the CONTROLLER-status half instead, which
        /// could break the install path that currently works.  Flat zero is what produced a
        /// complete 20-disk install, so that stays the default until the layout is confirmed.
        private static readonly ushort DriveHealthy =
            System.Environment.GetEnvironmentVariable("DOVE_DRIVE_HEALTHY") == "1" ? (ushort)0x0A00 : (ushort)0x0000;
        private const int ErrLabelAddrMark = 0x21, ErrDataAddrMark = 0x31;

        private void ExecuteDob()
        {
            // zero the four ErrorStatus fields at the start of every op (ZROERS)
            SetErr(W_HeaderError, ErrNone); SetErr(W_LabelError, ErrNone);
            SetErr(W_DataError, ErrNone);   SetErr(W_LastError, ErrNone);

            int cyl = HdrCyl, head = HdrHead, sector = HdrSector;
            int wantSectors = SectorCount();
            DobOps++; LastCyl = cyl; LastHead = head; LastSector = sector;
            int op = Operation;
            bool error = false;

            // Default completion drive state (overwrites the firmware's request placeholder, e.g. the
            // 0xCE58 notReady / 0xFFFF currentCylinder it pre-fills): heads at the addressed
            // cylinder, drive READY, no fault.
            //
            // DiskDriveStatusRec (Disk.def), Mesa MSB-first, in the HIGH byte of the word:
            //   0x8000 dskDriveNotReady    0x4000 dskSeekNotComplete   0x2000 dskUnu0
            //   0x1000 dskAddrMark         0x0800 dskNotStoredIndxMrk  0x0400 dskNotTrack0
            //   0x0200 dskNotWriteFault    0x0100 dskLock
            //
            // SIX OF EIGHT ARE ACTIVE-LOW, so flat zero is NOT the healthy state -- it asserts a
            // write fault and a missing index mark.  Pilot's NeedsRecalibrate returns TRUE on
            // writeFault, and the 8x305's OPEND branches on WriteFault being NONZERO for the OK
            // path (zero falls through to WRSTAT/WriteFaultError).  That is what made the boot
            // ROM restore the drive over and over instead of issuing its first read.
            _dob[W_CurrentCyl] = Bswap((ushort)cyl);
            _dob[W_DriveCtlrStatus] = (ushort)(DriveHealthy | (cyl != 0 ? 0x0400 : 0x0000));

            switch (op)
            {
                case 0:  // restore / recalibrate -- seek to track 0, no data
                    _dob[W_CurrentCyl] = Bswap(0x0000);     // currentCylinder = 0 (recalibrated)
                    // At track 0, so dskNotTrack0 stays clear -- but the active-low health bits
                    // must still be SET, or the restore reports a write fault and the boot ROM
                    // recalibrates forever instead of issuing its first read.
                    _dob[W_DriveCtlrStatus] = DriveHealthy;
                    break;

                // MULTI-SECTOR ops 2/3/6.  The firmware programs ONE page-sized StartDMA and
                // expects the controller to walk the buffer itself, a page per sector: measured,
                // successive multi-sector ops advance their buffer by exactly wantSectors*512.
                //
                // Transferring only the first sector and reporting SUCCESS is what emptied the
                // boot files.  op 3 arrives as 32-sector runs; we wrote 1 and dropped 31, so
                // file 0103 ended up with data at file pages 1, 33, 65, 97 ... -- one page per
                // run -- 91 of 2803 pages, 96.8% zeros, while every label landed correctly
                // (op 4 does loop).  Structurally perfect, functionally empty, and silent
                // because the guest was told the whole run succeeded and never retried.
                // The controller logged it 16,940 times as "NOTE op=N asked for M".
                // Ops 2/3/6 do sector 0 here, from the page StartDMA already moved, and then hand
                // the rest of the run to the streaming path: one sector per subsequent StartDMA,
                // at whatever address the firmware programs.  Do NOT compute addr + k*512 -- the
                // firmware owns the pointer and advances it itself (incrementDataPtr), and
                // delivering a whole run into a one-page buffer froze the machine twice.
                case 2:  // readData  -- verify label, DMA data->mem
                case 6:  // readLabelAndData -- copy label, DMA data->mem
                    error = ReadSector(cyl, head, sector, op == 2 /*verifyLabel*/);
                    if (!error && wantSectors > 1) StartStream(op, cyl, head, sector, wantSectors);
                    break;

                case 3:  // writeData -- tag "vvw": verify header, VERIFY label, write data
                    error = WriteSector(cyl, head, sector, false, true /*verifyLabel*/);
                    if (!error && wantSectors > 1) StartStream(op, cyl, head, sector, wantSectors);
                    break;

                case 4:  // writeLabelAndData -- store label + data, over a RUN of sectors
                    // diskMinusSectorCount is the RUN LENGTH, and it is independent of
                    // diskPageCount: one data page is supplied and replicated, while the
                    // labels are the structured payload and are stamped across the whole run
                    // with the file page number stepping per sector
                    // (FID, FT and the boot chain link are the same in every page of a run;
                    // FPLow/FPHi step per sector).  The older wording here cited
                    // CompatibilityDiskFace.mesa, which is Pilot 15 -- the release that
                    // deprecated labels.  We are Pilot 14; do not reason from that source.
                    // Writing only one sector here leaves the rest of the run holding their
                    // formatted labels, and Pilot's verify pass over the run then fails at
                    // the second sector and retries the write forever.
                    // The 8x305's WTLBDC loop runs diskMinusSectorCount sectors: NXTSC
                    // counts SECNTH/SECNTL up toward zero and sets DTARG purely from that,
                    // with no FIFO involvement -- so the run length really is the sector
                    // count.  The loop starts at the DOB header (VFYHD searches for that
                    // exact header image) and the FIRST sector takes the DOB label's filePage
                    // unchanged; NXTSC then steps sector and filePage together.
                    //
                    // The earlier ~64-writes-per-cylinder measurement that seemed to refute
                    // this was recovery traffic: the client re-issues the op after each
                    // read-back fails label-verify, which our own incomplete write caused.
                    error = WriteLabelRun(cyl, head, sector, wantSectors);
                    break;

                case 5:  // readLabel (skip data)
                    error = ReadLabelOnly(cyl, head, sector);
                    break;

                case 7:  // verifyData -- over the whole run, like writeLabelAndData
                    // The count applies here too: the client asks to verify N sectors and we
                    // were checking one and reporting one page of progress, so it walked the
                    // disk a sector at a time -- 128 operations where the firmware expects
                    // one.  No data moves for a verify, so the run needs no DMA coupling.
                    {
                        int vc = cyl, vh = head, vs = sector;
                        for (int k = 0; k < wantSectors && !error; k++)
                        {
                            if (vc >= Micropolis1325.Cylinders) break;
                            error = !_disk.IsFormatted(Micropolis1325.Page(vc, vh, vs));
                            if (++vs < Micropolis1325.SectorsPerTrack) continue;
                            vs = 0;
                            if (++vh < Micropolis1325.Heads) continue;
                            vh = 0; vc++;
                        }
                    }
                    if (error) SetErr(W_DataError, ErrSectorNotFound);
                    break;

                case 1:  // formatTracks
                    FormatTracks(cyl, head);
                    break;

                default:
                    // illegal / not-yet-modelled op -- report cleanly, no data
                    break;
            }

            // COMPLETION BOOKKEEPING.  The Pilot-14 authority is DiskHeadDove, and it uses TWO
            // DIFFERENT paths -- conflating them is what made this block contradict itself for
            // months:
            //
            //   SUCCESS -> progress comes from iocb.runLength.  The next request's start comes
            //              from dob.header put through IncrementClientHeader -- i.e. PILOT
            //              OWNS THE +1, which is why we end ON the last sector (below).
            //   ERROR   -> only here is progress the header delta:
            //                dobPagesCompleted = pageNumber(dob.header) - pageNumber(clientHeader)
            //                pagesCompleted    = read ? MIN[dob, dma] : dob
            //              with the dma half fed by the IOP DMA decrementing iocb.pageCount as
            //              it transfers (DiskDove.asm:1094-1104).  A non-goodCompletion status
            //              -> DiskStatusToDataStatus -> ioError -> Space.IOError[page] -> 0935.
            //
            // The earlier wording here cited "DataTransferImpl.Poll" and presented the ERROR
            // formula as the success criterion.  There is no DataTransferImpl.Poll in Pilot 14
            // -- that is a Pilot 15 layer name, and Pilot 15 is the release that deprecated
            // labels.  Taken literally it also could not have been true: it demands a header
            // delta > 0, yet single-sector ops (9,215 op-2s, 7,412 op-3s) return a delta of 0
            // by design and the install completed all 20 disks.
            //
            // The IOP passes the header through UNTOUCHED; it only decrements the page count.
            // The request CHS is still in (cyl,head,sector); label/currentCyl/status already
            // reflect completion.
            //
            // NOTE the error path is asymmetric with what we do: we skip the advance entirely
            // when error is set, so a run that fails partway reports a delta of 0 rather than
            // the sectors it did complete.  Correct for the 1-sector failures we actually see
            // (all 776 label rejections are single-sector), wrong in principle for a partial
            // multi-sector run -- Pilot would be told nothing happened.
            // Streaming runs set their own header in FinishTransfer, once the run really ends.
            if (!error && !_xferActive && (op == 2 || op == 3 || op == 4 || op == 5 || op == 6 || op == 7))
            {
                // The returned header ends ON the last sector processed, not one past it.
                // Measured three times over: with an advance of 0 the client's next request
                // arrived one page on, with 1 it arrived two on, with 128 it arrived 129 on.
                // client_next = our_returned + 1 consistently, so for consecutive operations
                // to abut exactly the header must end on the last sector stepped.
                //
                // Returning one past it made every operation jump a whole cylinder plus a
                // sector (+129), so the sector crept by one per op and the head only moved
                // when it wrapped, sixteen cylinders later.
                // The HEADER ends ON the last sector stepped; the returned LABEL filePage
                // (below) ends one past.  They are not the same rule, however much they look
                // like they should be -- controlled measurement, same build, one variable:
                //
                //   advance N-1  ->  client's next request lands +N   (correct, abutting)
                //   advance N    ->  client's next request lands +N+1 (marches a cylinder)
                //
                // Measured in both directions -- and now EXPLAINED rather than merely observed:
                // DiskHeadDove runs the returned header through IncrementClientHeader to get
                // the next start, so Pilot supplies the +1 itself.  Ending one past would apply
                // it twice.  This is not the controller disagreeing with the software; ends-on
                // is the contract.
                // Only the ops that actually walk their run may advance by it.  op 3 now does;
                // ops 2/6 deliberately still transfer one sector (see the case above), so they
                // must keep reporting one page of progress or the client's next request skips
                // the 127 pages we never gave it.
                int advance = ((op == 3 || op == 4 || op == 7) ? wantSectors : 1) - 1;
                int linear = (cyl * Micropolis1325.Heads + head) * Micropolis1325.SectorsPerTrack
                             + sector + advance;
                int ns = linear % Micropolis1325.SectorsPerTrack;
                int track = linear / Micropolis1325.SectorsPerTrack;
                int nh = track % Micropolis1325.Heads;
                int nc = track / Micropolis1325.Heads;
                _dob[16] = Bswap((ushort)nc);                 // cylinder  (ByteSwappedWord)
                _dob[17] = (ushort)((ns << 8) | nh);          // sector(hi) : head(lo)
                if (LogWriter != null && advance > 1)
                {
                    try { LogWriter.WriteLine("    HEADER ADVANCE +" + advance + " -> CHS=["
                            + nc + "," + nh + "," + ns + "]"); }
                    catch { LogWriter = null; }
                }
            }
            if (Log == null && LogWriter == null) { /* no logging */ }
            else if (Log == null)
            {
                // File-only path: keep the same content without growing an in-memory list.
                try { LogWriter.WriteLine("DOB op=" + op + " req CHS=[" + cyl + "," + head + "," + sector + "]"
                    + " err=" + error
                    + " HdrErr=" + (_dob[W_HeaderError] >> 8).ToString("X2")
                    + " LblErr=" + (_dob[W_LabelError] >> 8).ToString("X2")
                    + " DatErr=" + (_dob[W_DataError] >> 8).ToString("X2")
                    + " -> hdr w16=" + _dob[16].ToString("X4") + " w17=" + _dob[17].ToString("X4")
                    + " dmaInt r/a/lost=" + DmaIntRaised + "/" + DmaIntAcked + "/" + DmaIntLost
                    + " dmaCount=0x" + _dmaCount.ToString("X4") + " sectors=" + wantSectors
                    + " (w2=" + _dob[2].ToString("X4") + ")" + " @IOP" + HostClock);
                    // Full DOB + the IOCB words just below it.  The transfer's page count has
                    // to be in one of the words we do not decode, and the firmware tracks
                    // remaining pages in the IOCB (DiskDove.asm:1094-1104).
                    var sb2 = new System.Text.StringBuilder("    DOB:");
                    for (int k = 0; k < 34; k++) sb2.Append(' ').Append(k).Append('=').Append(_dob[k].ToString("X4"));
                    if (_dobAddr >= 0)
                    {
                        sb2.Append("   IOCB:");
                        for (int off = -0x14; off <= 0; off += 2)
                            sb2.Append(' ').Append(off).Append('=').Append(RdWord(_dobAddr + off).ToString("X4"));
                    }
                    LogWriter.WriteLine(sb2.ToString()); }
                catch { LogWriter = null; }   // never let logging take the machine down
            }
            if (Log != null)   // DOB completion lines are rare (one per op) -- bypass the 800 register-log cap
            {
                // Snapshot the IOCB words just below the DOB (firmware decrements iocb.pageCount there per
                // DiskDove.asm:1094-1104).  Read RAW phys words; report byte-swapped too.  DOB-8=iocb.pageCount.
                string iocb = "";
                if (_dobAddr >= 0)
                    for (int off = -0x14; off <= 0; off += 2)
                    {
                        int a = _dobAddr + off; ushort v = RdWord(a);
                        iocb += "[" + (off).ToString() + "]" + v.ToString("X4") + "(bs " + Bswap(v).ToString("X4") + ") ";
                    }
                Log.Add("DOB op=" + op + " req CHS=[" + cyl + "," + head + "," + sector + "]"
                    + " err=" + error + " -> hdr w16=" + _dob[16].ToString("X4") + " w17=" + _dob[17].ToString("X4")
                    + " HdrErr=" + (_dob[W_HeaderError] >> 8).ToString("X2") + " currentCyl=" + _dob[W_CurrentCyl].ToString("X4")
                    + " dmaCount=0x" + _dmaCount.ToString("X4") + " dobAddr=0x" + _dobAddr.ToString("X5") + " @IOP" + HostClock
                    + "  IOCB(off from DOB): " + iocb);
            }

            // MEASURED, and it refutes the assumed invariant: for an op whose
            // diskMinusSectorCount is -128, the IOP issues exactly ONE page-DMA, not 128.
            // So diskPageCount and diskMinusSectorCount are NOT simply negatives of each
            // other on this path, and a model that waits for one page per sector waits
            // forever.  Until the relationship is known, complete on the pages actually
            // supplied rather than inventing transfers.
            if (wantSectors > 1 && LogWriter != null)
            {
                try { LogWriter.WriteLine("    NOTE op=" + op + " asked for " + wantSectors
                        + " sectors; only the DMA'd page(s) were moved"); }
                catch { LogWriter = null; }
            }

            _status = (byte)(error ? 0xC2 : 0x42);          // e+d on error, else done
        }

        /// <summary>Step the running transfer address, carrying sector -> head -> cylinder.</summary>
        private void StepAddress()
        {
            if (++_xferSector < Micropolis1325.SectorsPerTrack) return;
            _xferSector = 0;
            if (++_xferHead < Micropolis1325.Heads) return;
            _xferHead = 0;
            _xferCyl++;
        }

        /// <summary>
        /// Move one more sector of a multi-sector operation, called from each page-DMA.
        /// </summary>
        /// <summary>
        /// Arm the streaming path after sector 0 of a multi-sector op has been done inline.
        /// Positions at the NEXT sector; each subsequent StartDMA moves one more.
        /// </summary>
        private void StartStream(int op, int cyl, int head, int sector, int sectors)
        {
            int lin = Micropolis1325.Page(cyl, head, sector) + 1;
            _xferSector = lin % Micropolis1325.SectorsPerTrack;
            int tr = lin / Micropolis1325.SectorsPerTrack;
            _xferHead = tr % Micropolis1325.Heads;
            _xferCyl = tr / Micropolis1325.Heads;
            _xferLastCyl = cyl; _xferLastHead = head; _xferLastSector = sector;
            _xferOp = op;
            _xferRemaining = sectors - 1;
            _xferError = false;
            _xferIndex = 0;
            _xferL5 = _dob[23 + 5]; _xferL6 = _dob[23 + 6];
            _xferActive = true;
        }

        private void TransferNextSector(int addr)
        {
            int c = _xferCyl, h = _xferHead, sct = _xferSector;
            bool err = false;

            // Each sector of the run carries its own file page, so the expected label has to
            // step with it -- verifying the whole run against its first label matches sector 0
            // and rejects sector 1.
            _xferIndex++;
            StepExpectedFilePage(_xferL5, _xferL6, _xferIndex);

            switch (_xferOp)
            {
                case 2:
                case 6:
                    err = ReadSector(c, h, sct, _xferOp == 2 /*verifyLabel*/);
                    if (!err && !_noDataDma)
                        for (int i = 0; i < 512; i++) Sys[(addr + i) & Mask] = _dataBuf[i];
                    break;
                case 3:
                case 4:
                    if (!_noDataDma)
                        for (int i = 0; i < 512; i++) _dataBuf[i] = Sys[(addr + i) & Mask];
                    // op 3 is vvw: the label verify applies to every sector of the run, not
                    // just the first.  The 4-argument overload skips it.
                    err = WriteSector(c, h, sct, _xferOp == 4 /*storeLabel*/, _xferOp == 3 /*verifyLabel*/);
                    break;
            }

            DobOps++; LastCyl = c; LastHead = h; LastSector = sct;
            _xferLastCyl = c; _xferLastHead = h; _xferLastSector = sct;
            StepAddress();

            if (err) { _xferError = true; FinishTransfer(); return; }
            if (--_xferRemaining <= 0) FinishTransfer();
        }

        /// <summary>
        /// End a multi-sector operation: leave the DOB header ON the last sector transferred --
        /// NOT one past it.  DiskHeadDove runs the returned header through IncrementClientHeader
        /// to derive the next start, so Pilot supplies the +1 itself and ending one past applies
        /// it twice (measured: every operation then marches a cylinder plus a sector).  Restore
        /// the run's base label, set the status, and raise the single completion interrupt.
        /// </summary>
        private void FinishTransfer()
        {
            _xferActive = false;
            // LEAVE the label stepped to the last sector -- do NOT restore the run's base.
            // The label follows the same contract as the CHS header: end on the last, and Pilot
            // supplies the +1.  Measured: a 128-sector run over disk 6584..6711 = filePage
            // 227..354; restoring the base made the client's next request expect 227+1=228 while
            // the disk at 6712 holds 355, and the mismatch never converged.  Returning 354 makes
            // it expect 355 and match.  WriteLabelRun already returns its end page this way.
            _dob[16] = Bswap((ushort)_xferLastCyl);
            _dob[17] = (ushort)((_xferLastSector << 8) | _xferLastHead);
            _status = (byte)(_xferError ? 0xC2 : 0x42);
            if (LogWriter != null)
            {
                try { LogWriter.WriteLine("    XFER DONE op=" + _xferOp + " err=" + _xferError
                        + " next CHS=[" + _xferCyl + "," + _xferHead + "," + _xferSector + "]"); }
                catch { LogWriter = null; }
            }
            ScheduleCtlrInt();
        }

        /// <summary>
        /// writeLabelAndData over a run of sectors: the single supplied data page is written
        /// to every sector, and each sector gets the DOB's label with filePage stepped by its
        /// position in the run.
        /// </summary>
        /// <summary>
        /// Point the DOB's expected label at file page base+k, for sector k of a run.
        ///
        /// The on-disk label's file page steps with the sector -- WriteLabelRun does exactly this
        /// when it WRITES a run -- so verifying a run against its first label would match sector 0
        /// and reject sector 1, aborting one page in.  Only FPLow/FPHi move; every other word,
        /// including the flag half of word 6, is the client's and is left alone.
        /// </summary>
        private void StepExpectedFilePage(ushort w5, ushort w6, int k)
        {
            if (k == 0) { _dob[23 + 5] = w5; _dob[23 + 6] = w6; return; }
            int fp = (Bswap(w5) | (((w6 >> 1) & 0x7F) << 16)) + k;
            _dob[23 + 5] = Bswap((ushort)(fp & 0xFFFF));
            _dob[23 + 6] = (ushort)((w6 & 0xFF01) | (((fp >> 16) & 0x7F) << 1));
        }

        private bool WriteLabelRun(int cyl, int head, int sector, int sectors)
        {
            int c0 = cyl, h0 = head, s0 = sector;
            var template = new ushort[Micropolis1325.LabelWords];
            for (int i = 0; i < template.Length; i++) template[i] = _dob[23 + i];
            // filePage spans label bytes 10-12, and word 5 and word 6 are byte-ordered
            // DIFFERENTLY -- this asymmetry is the trap I fell into twice:
            //   byte 10 = filePage bits 8-15   byte 11 = bits 0-7   -> word 5, byte-SWAPPED
            //   byte 12 = (filePageHi << 1) | flag                  -> word 6 LOW byte, RAW
            //   byte 13 = more flags                                -> word 6 HIGH byte
            // So word 6 must NOT be byte-swapped, filePageHi is only 7 bits, and byte 13 has
            // nothing to do with the page number -- without the mask it leaks into it.
            //
            // FIELD NAMES, from the field-engineer manual -- the label is
            //   0-4 FID0..FID4   5 FPLow   6 FPHi&Flags   7 FT   8 bootCLLo   9 bootCLHi
            // Word 6's high byte is FLAGS, not "pageZeroAttributes", and word 7 is FT (file
            // type), not "attributesInAllPages".  Both of those older names came from Pilot 15
            // source, which deprecated labels -- we run Pilot 14, where they are load-bearing.
            // Measured on a full pack and consistent with the manual: word 6's high byte takes
            // exactly two values in 122,880 pages (0x00 on all but two), which no attributes
            // byte would; word 7 takes 12 values dominated by 0x0600 free extent, which is a
            // type field.  Words 8-9 are the boot chain link, previously called dontCare.
            //
            // Invisible on everything seen so far because every filePage has been under
            // 65536, making filePageHi zero regardless of byte order or mask.  It bites in
            // the upper half of a 122,880-page volume, where a byte-swapped read turns
            // +65536 into +16M.
            int basePage = Bswap(template[5]) | (((template[6] >> 1) & 0x7F) << 16);
            // The flag half of word 6 is the CLIENT'S and is copied byte-for-byte (below).
            // Do not synthesize it: carrying a decoded value through a run put 0x0200 on
            // sectors that wanted 0x0000, and zeroing it everywhere stopped the install dead
            // at disk 9.  Measured on a full pack, nothing is mis-stamped -- zero pages carry
            // a flag at filePage != 0 -- so verbatim copy is correct and stays.

            for (int k = 0; k < sectors; k++)
            {
                if (cyl >= Micropolis1325.Cylinders) break;

                var label = (ushort[])template.Clone();
                // First sector of the run takes the DOB's filePage verbatim; each subsequent
                // one steps with the sector.  Seeding this one low is what left every page
                // number in the run off by one.
                int filePage = basePage + k;
                label[5] = Bswap((ushort)(filePage & 0xFFFF));
                // Copy word 6 and touch ONLY the file-page bits (byte 12 bits 1-7).  The flag
                // half -- byte 13 and byte 12's bit 0 -- is the client's and is preserved
                // byte-for-byte.  The controller does not synthesize, vary or zero flags.
                label[6] = (ushort)((template[6] & 0xFF01) | (((filePage >> 16) & 0x7F) << 1));

                _disk.WriteSector(Micropolis1325.Page(cyl, head, sector), _dataBuf, label);

                if (++sector < Micropolis1325.SectorsPerTrack) continue;
                sector = 0;
                if (++head < Micropolis1325.Heads) continue;
                head = 0;
                cyl++;
            }

            // Hand back the LAST page written, not one past it -- the same rule the header
            // follows, and for the same reason: the client adds one itself.
            //
            // Measured: with base + N, a rewrite of [0,0,1] came back with base 2 where the
            // guest wanted 1, and it looped writing volume structures.  With base + N - 1 the
            // rewrite lands on the page it was already on.  The source reads as though the
            // returned label seeds the next base verbatim; as with the header, the machine
            // says a page is consumed between here and the client.
            int endPage = basePage + sectors - 1;
            _dob[23 + 5] = Bswap((ushort)(endPage & 0xFFFF));
            _dob[23 + 6] = (ushort)((template[6] & 0xFF01) | (((endPage >> 16) & 0x7F) << 1));

            if (LogWriter != null)
            {
                // incrementDataPtr lives in the IOCB flag word 9 words below the DOB (bit 8,
                // Mesa MSB-0 = 0x0080).  When it is CLEAR the client is filling -- every page
                // DMA re-reads one source page, so replicating that page across the run is
                // right.  When it is SET each sector should get DISTINCT data, and replicating
                // silently writes the same page 128 times.  We do not model ECC, so wrong data
                // raises no error at all: the controller stays clean and the guest dies later.
                int iocbFlags = _dobAddr >= 0 ? RdWord(_dobAddr - 18) : 0;
                try { LogWriter.WriteLine("    RUN FLAGS iocb-9=" + iocbFlags.ToString("X4")
                        + " incrementDataPtr=" + (((iocbFlags & 0x0080) != 0) ? "SET(distinct)" : "clear(fill)")
                        + "  bswapped=" + Bswap((ushort)iocbFlags).ToString("X4")); }
                catch { LogWriter = null; }
                try { LogWriter.WriteLine("    LABEL RUN " + sectors + " sectors from CHS=["
                        + c0 + "," + h0 + "," + s0 + "] filePage from "
                        + basePage + ", fileID=" + Bswap(template[0])); }
                catch { LogWriter = null; }
            }
            return false;
        }

        private bool ReadSector(int cyl, int head, int sector, bool verifyLabel)
        {
            int page = Micropolis1325.Page(cyl, head, sector);
            if (!_disk.IsFormatted(page))
            {
                if (_notFoundOnBlank)
                {
                    // UNFORMATTED-platter model (env-gated): no address marks -> not-found family.
                    // This raises Space.IOError on the germ's PV read (AsynchronousVMIOError -> 0935).
                    SetErr(W_HeaderError, ErrSectorNotFound);
                    SetErr(W_LabelError, ErrLabelAddrMark);
                    SetErr(W_DataError, ErrDataAddrMark);
                    SetErr(W_LastError, ErrSectorNotFound);
                    for (int i = 0; i < 512; i++) _dataBuf[i] = 0xE5;
                    return true;                    // -> status 0xC2
                }
                // LOW-LEVEL-FORMATTED blank drive (default): sector headers exist, so the read SUCCEEDS
                // and returns ZERO data + ZERO label.  The completion is goodCompletion (with the header
                // advanced below); Pilot's PV seal/version check -- not the disk layer -- decides the
                // volume is uninitialized.  No Space.IOError.
                for (int i = 0; i < 512; i++) _dataBuf[i] = 0x00;
                if (!verifyLabel) for (int i = 0; i < 10; i++) _dob[23 + i] = 0;
                return false;                       // success
            }
            var data = _disk.ReadData(page);
            var label = _disk.ReadLabel(page);
            if (verifyLabel && label != null)
            {
                // compare the on-disk label (words 0-7 significant) against the DOB label (w23-30)
                // Word 6 (FPHi&Flags) IS significant here -- do not be tempted to mask it
                // to make mismatches go away.  Errors on this path are survivable in any case:
                // op 2 is vvr and RDSK transfers the data before it tests ERRTYP, so the client
                // reads the true label back and retries.  It is the WRITE side (vvw) that loses
                // pages.
                for (int i = 0; i < 8; i++)
                {
                    if (label[i] != _dob[23 + i])
                    {
                        // Record both sides: a verify failure is only meaningful next to the
                        // label the software expected and the one actually on the platter.
                        if (LogWriter != null)
                        {
                            var sb = new System.Text.StringBuilder();
                            sb.Append("  LABEL MISMATCH page=").Append(page)
                              .Append(" CHS=[").Append(cyl).Append(',').Append(head).Append(',').Append(sector)
                              .Append("] word").Append(i).Append("  disk=");
                            for (int k = 0; k < 10; k++) sb.Append(label[k].ToString("X4")).Append(' ');
                            sb.Append(" expected=");
                            for (int k = 0; k < 10; k++) sb.Append(_dob[23 + k].ToString("X4")).Append(' ');
                            try { LogWriter.WriteLine(sb.ToString()); } catch { LogWriter = null; }
                        }
                        SetErr(W_LabelError, ErrLabelVerify);
                        return true;
                    }
                }
            }
            else if (!verifyLabel)
            {
                for (int i = 0; i < 10; i++) _dob[23 + i] = label != null ? label[i] : (ushort)0;   // copy label into DOB
            }
            Array.Copy(data, _dataBuf, 512);
            return false;
        }

        private bool WriteSector(int cyl, int head, int sector, bool storeLabel)
        {
            return WriteSector(cyl, head, sector, storeLabel, false);
        }

        private bool WriteSector(int cyl, int head, int sector, bool storeLabel, bool verifyLabel)
        {
            if (verifyLabel)
            {
                // REJECTING HERE IS CORRECT AND LOAD-BEARING -- do not soften it.  op 3 is vvw:
                // WTDC verifies the label and jumps to ERENDD before WTDTA, so a mismatch does
                // not write the page (unlike the read side, where RDSK transfers first and only
                // then tests ERRTYP).  That rejection is how Pilot learns its guess was wrong.
                //
                // Word 6 bit 0x0200 is `temporary` (PilotDisk.Label: filePageHi bits 0-6,
                // pad1 7-13, temporary 14, pad2 15 -- Mesa bit order, so 0x0002 logical =
                // 0x0200 in the DOB).  DiskBackingStore.mesa:47 marks it "HINT: if wrong, a
                // retry will be performed": Pilot writes its best guess of temporary, and a
                // wrong guess is REJECTED BY DESIGN, then re-issued with the corrected value.
                //
                // Measured over a full 20-disk install: 776 rejections across 62 pages, every
                // one of them filePage 0, and ALL 62 converge to a successful write afterwards.
                // Zero pages were lost.  They were read as "dropped writes" for a long time --
                // they are the hint-miss protocol working.  Count converging retries before
                // calling a rejection a defect.  Pad audit over the same pack: zero violations
                // (bit0, 0x0100 and 0xFC00 all clear on all 122,880 pages), so the encoder puts
                // temporary exactly at 0x0200 and the corrected retry verifies.
                var onDisk = _disk.ReadLabel(Micropolis1325.Page(cyl, head, sector));
                if (onDisk != null)
                    for (int i = 0; i < 8; i++)
                        if (onDisk[i] != _dob[23 + i])
                        {
                            if (LogWriter != null)
                            {
                                var sb = new System.Text.StringBuilder();
                                sb.Append("  WRITE LABEL MISMATCH page=").Append(Micropolis1325.Page(cyl, head, sector))
                                  .Append(" CHS=[").Append(cyl).Append(',').Append(head).Append(',').Append(sector)
                                  .Append("] word").Append(i).Append("  disk=");
                                for (int k = 0; k < 10; k++) sb.Append(onDisk[k].ToString("X4")).Append(' ');
                                sb.Append(" expected=");
                                for (int k = 0; k < 10; k++) sb.Append(_dob[23 + k].ToString("X4")).Append(' ');
                                // Whole DOB as well: we only decode 12 of its 34 words, and the
                                // fields we ignore (0,1,3,4,5,7,8,9,15,18,19,33) include Sector
                                // Length -- normally -1, but -2 for the longer data blocks some
                                // diagnostics use, which we would silently serve as 512 bytes.
                                sb.Append(" DOB=");
                                for (int k = 0; k < _dob.Length; k++) sb.Append(_dob[k].ToString("X4")).Append(' ');
                                try { LogWriter.WriteLine(sb.ToString()); } catch { LogWriter = null; }
                            }
                            SetErr(W_LabelError, ErrLabelVerify);
                            return true;
                        }
            }
            int page = Micropolis1325.Page(cyl, head, sector);
            var label = new ushort[10];
            for (int i = 0; i < 10; i++) label[i] = _dob[23 + i];
            if (storeLabel) _disk.WriteSector(page, _dataBuf, label);
            else _disk.WriteData(page, _dataBuf);
            return false;
        }

        private bool ReadLabelOnly(int cyl, int head, int sector)
        {
            int page = Micropolis1325.Page(cyl, head, sector);
            if (!_disk.IsFormatted(page))
            {
                if (_notFoundOnBlank)               // unformatted -> label address mark not found
                {
                    SetErr(W_LabelError, ErrLabelAddrMark);
                    SetErr(W_LastError, ErrSectorNotFound);
                    return true;
                }
                for (int i = 0; i < 10; i++) _dob[23 + i] = 0;   // LL-formatted: zero label, success
                return false;
            }
            var label = _disk.ReadLabel(page);
            for (int i = 0; i < 10; i++) _dob[23 + i] = label != null ? label[i] : (ushort)0;
            return false;
        }

        private void FormatTracks(int cyl, int head)
        {
            // negativeFormatTrackCount (w22, byte-swapped 2's-complement) = -(tracks to format)
            int nfc = Bswap(_dob[22]);
            int tracks = nfc == 0 ? 1 : (0x10000 - nfc);
            for (int t = 0; t < tracks; t++)
            {
                int c = cyl + (head + t) / Micropolis1325.Heads;
                int h = (head + t) % Micropolis1325.Heads;
                if (c >= Micropolis1325.Cylinders) break;
                for (int s = 0; s < Micropolis1325.SectorsPerTrack; s++)
                    _disk.FormatSector(Micropolis1325.Page(c, h, s));
            }
            _dob[W_CurrentCyl] = Bswap((ushort)cyl);
            _dob[W_DriveCtlrStatus] = (ushort)(DriveHealthy | (cyl != 0 ? 0x0400 : 0x0000));
        }
    }
}
