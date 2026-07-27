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
        private bool _xferActive;
        private int _xferOp, _xferCyl, _xferHead, _xferSector, _xferRemaining;
        private bool _xferError;
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
        public void Tick(int clocks)
        {
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
            return v;
        }

        public void WriteReg(ushort port, ushort value)
        {
            if (Log != null && Log.Count < 800)
                Log.Add("W  0x" + port.ToString("X4") + " <- 0x" + value.ToString("X4") + " @IOP" + HostClock
                    + (port == 0x0214 ? "  (cmd cc=" + (value & 3) + ")" : ""));
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
                    ScheduleCtlrInt();
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
                if (_xferActive) { TransferNextSector(_dmaAddr); return; }

                if (_dmaDir == 1)               // mem->disk: capture data from memory NOW (write ops feed
                    { if (!_noDataDma) for (int i = 0; i < 512; i++) _dataBuf[i] = Sys[(_dmaAddr + i) & Mask]; }
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
            // 0xCE58 notReady / 0xFFFF currentCylinder it pre-fills): heads at the addressed cylinder,
            // drive READY, no fault.  DriveAndControllerStatus is MSB-first active-low --
            // notReady(0x8000)/notTrack0(0x0400) = FALSE (0) for the good state; notTrack0 set iff cyl!=0.
            _dob[W_CurrentCyl] = Bswap((ushort)cyl);
            _dob[W_DriveCtlrStatus] = (ushort)(cyl != 0 ? 0x0400 : 0x0000);

            switch (op)
            {
                case 0:  // restore / recalibrate -- seek to track 0, no data
                    _dob[W_CurrentCyl] = Bswap(0x0000);     // currentCylinder = 0 (recalibrated)
                    _dob[W_DriveCtlrStatus] = 0x0000;       // ready, at track 0
                    break;

                case 2:  // readData  -- verify label, DMA data->mem
                case 6:  // readLabelAndData -- copy label, DMA data->mem
                    error = ReadSector(cyl, head, sector, op == 2 /*verifyLabel*/);
                    break;

                case 3:  // writeData -- verify label, write data
                    error = WriteSector(cyl, head, sector, false);
                    break;

                case 4:  // writeLabelAndData -- store label + data, over a RUN of sectors
                    // diskMinusSectorCount is the RUN LENGTH, and it is independent of
                    // diskPageCount: one data page is supplied and replicated, while the
                    // labels are the structured payload and are stamped across the whole run
                    // with the file page number stepping per sector
                    // (CompatibilityDiskFace.mesa: fileID, attributesInAllPages and dontCare
                    // are the same in every page of a run; filePage is incremented in each
                    // successive page; pageZeroAttributes is written only for page zero).
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

                case 7:  // verifyData
                    error = !_disk.IsFormatted(Micropolis1325.Page(cyl, head, sector));
                    if (error) SetErr(W_DataError, ErrSectorNotFound);
                    break;

                case 1:  // formatTracks
                    FormatTracks(cyl, head);
                    break;

                default:
                    // illegal / not-yet-modelled op -- report cleanly, no data
                    break;
            }

            // COMPLETION BOOKKEEPING (root cause of cStorage/AsynchronousVMIOError->cGermAllocFault):
            // Pilot's DataTransferImpl.Poll judges op success by pagesCompleted, NOT by the error bytes:
            //   dobPagesCompleted = pageNumber(dob.header) - pageNumber(op.clientHeader)
            //   pagesCompleted    = read ? MIN[dob, dma] : dob   (must be > 0 or the op is "no progress")
            // A non-goodCompletion status -> DiskStatusToDataStatus -> ioError -> Space.IOError[page]
            // -> uncaught -> 0935.  So on a SUCCESSFUL sector transfer we must advance dob.header
            // (CHS words 16-17) to the sector AFTER the last one transferred, so pageNumber advances
            // by the N sectors done (N=1 per DOB in this model).  The request CHS is still in
            // (cyl,head,sector); the label/currentCyl/status fields already reflect completion.
            if (!error && (op == 2 || op == 3 || op == 4 || op == 5 || op == 6))
            {
                // Advance the returned header by the sectors ACTUALLY TRANSFERRED, not by one.
                // For a write, Pilot takes pagesCompleted straight from this header --
                // PageNumber(returned) - PageNumber(start) -- and then assigns
                // op.clientHeader <- the returned header, with no increment of its own.  So
                // reporting +1 after a 128-sector run claims one page of progress out of 128,
                // and the client re-issues against a header that never catches up.  That is
                // the writer-strides-2 / reader-strides-1 split.
                int advance = (op == 4) ? wantSectors : 1;
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
        private void TransferNextSector(int addr)
        {
            int c = _xferCyl, h = _xferHead, sct = _xferSector;
            bool err = false;

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
                    err = WriteSector(c, h, sct, _xferOp == 4 /*storeLabel*/);
                    break;
            }

            DobOps++; LastCyl = c; LastHead = h; LastSector = sct;
            StepAddress();

            if (err) { _xferError = true; FinishTransfer(); return; }
            if (--_xferRemaining <= 0) FinishTransfer();
        }

        /// <summary>
        /// End a multi-sector operation: leave the DOB header at the sector AFTER the last one
        /// transferred (Pilot derives pagesCompleted from that advance, not from the error
        /// bytes), set the status, and raise the single controller-completion interrupt.
        /// </summary>
        private void FinishTransfer()
        {
            _xferActive = false;
            _dob[16] = Bswap((ushort)_xferCyl);
            _dob[17] = (ushort)((_xferSector << 8) | _xferHead);
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
        private bool WriteLabelRun(int cyl, int head, int sector, int sectors)
        {
            int c0 = cyl, h0 = head, s0 = sector;
            var template = new ushort[Micropolis1325.LabelWords];
            for (int i = 0; i < template.Length; i++) template[i] = _dob[23 + i];
            int basePage = Bswap(template[5]) | (Bswap(template[6]) << 16);

            for (int k = 0; k < sectors; k++)
            {
                if (cyl >= Micropolis1325.Cylinders) break;

                var label = (ushort[])template.Clone();
                // First sector of the run takes the DOB's filePage verbatim; each subsequent
                // one steps with the sector.  Seeding this one low is what left every page
                // number in the run off by one.
                int filePage = basePage + k;
                label[5] = Bswap((ushort)(filePage & 0xFFFF));
                // Word 6 is filePageHi, and pageZeroAttributes shares it.  Zeroing it
                // wholesale threw away the top 16 bits of every page number above 65535 --
                // and the volume is 122,880 pages, so everything past halfway carried a
                // truncated page.  Carry the high word; page zero keeps the client's
                // attributes, which is the only page they belong to.
                label[6] = filePage == 0
                         ? template[6]
                         : Bswap((ushort)((filePage >> 16) & 0xFFFF));

                _disk.WriteSector(Micropolis1325.Page(cyl, head, sector), _dataBuf, label);

                if (++sector < Micropolis1325.SectorsPerTrack) continue;
                sector = 0;
                if (++head < Micropolis1325.Heads) continue;
                head = 0;
                cyl++;
            }

            if (LogWriter != null)
            {
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
                for (int i = 0; i < 8; i++)
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
            else if (!verifyLabel)
            {
                for (int i = 0; i < 10; i++) _dob[23 + i] = label != null ? label[i] : (ushort)0;   // copy label into DOB
            }
            Array.Copy(data, _dataBuf, 512);
            return false;
        }

        private bool WriteSector(int cyl, int head, int sector, bool storeLabel)
        {
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
            _dob[W_DriveCtlrStatus] = 0x0000;
        }
    }
}
