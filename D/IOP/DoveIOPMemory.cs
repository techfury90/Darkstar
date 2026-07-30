/*
    BSD 2-Clause License

    Copyright Vulcan Inc. 2017-2018 and Living Computer Museum + Labs 2018
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright notice, this
      list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer in the documentation
      and/or other materials provided with the distribution.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
    AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
    IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED.IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
    FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
    DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
    SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
    OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

using System;
using System.IO;

namespace D.IOP
{
    /// <summary>
    /// The Dove IOP's physical memory bus (IOP-TR §3.1).  Decodes:
    ///
    ///   0x00000 - 0x03FFF   Local SRAM  (16 KB, LMCS/LCS')  -- IVT, kernel data, ALL
    ///                       handler stacks; private, NOT aliased into DRAM.
    ///   0xFC000 - 0xFFFFF   Local EPROM (16 KB, UMCS/UCS')  -- boot ROM, reset 0xFFFF0
    ///   0x04000 - 0xFBFFF   main system DRAM, windowed through 8 map registers
    ///                       (E018-E01F, IOP's regs 8-15): each covers a 128 KB page,
    ///                       phys = (mapreg[N] << 17) | (IOPlinear & 0x1FFFF).
    ///
    /// With the boot ROM's OUT 0xE018,0x05 (MapReg8=5, never changed), IOP linear
    /// 0x4000..0x1FFFF = physical DRAM byte 0xA0000+L -- so the IORegion at IOP linear
    /// 0x4000 = phys 0xA4000 = the CP's real word 0x52000, the SAME bytes.  RAM-Opie code
    /// at IOP 0x10000+ = phys 0xB0000+.  Everything except the 16 KB SRAM and the EPROM
    /// is the one shared DRAM array; there is NO separate mid-SRAM window.
    /// </summary>
    public class DoveIOPMemory : IPhysicalMemory
    {
        public const int EpromBase = 0xFC000;
        public const int EpromSize = 0x4000;    // 16 KB
        public const int SramBase = 0x00000;
        public const int SramSize = 0x4000;     // 16 KB
        public const int SystemSize = 0x400000; // 4 MB system DRAM backing

        public DoveIOPMemory(string romPath)
        {
            _rom = new byte[EpromSize];
            LoadRom(romPath);

            _sram = new byte[SramSize];

            // Backing store for the system DRAM that the IOP map window relocates
            // into (and that the display DMA scans out of).  4 MB covers the real
            // 3.7 MB configuration plus map-frame headroom.
            _system = new byte[SystemSize];

            for (int i = 0; i < _mapRegisters.Length; i++) _mapRegisters[i] = NilMapData;
        }

        public byte ReadByte(int address)
        {
            address &= 0xFFFFF;

            if (address >= EpromBase)
            {
                return _rom[address - EpromBase];
            }
            if (address < SramBase + SramSize)
            {
                return _sram[address - SramBase];
            }

            int sys = TranslateMap(address);
            byte rv = sys < 0 ? (byte)0xFF : _system[sys];
            // TEMP: log the IOP's access to the whole ProcessorHead FCB header (phys 0xB0000..0xB001F).
            // Whether the mesa task ever READS fcb.command (0xB0010/11) and dispatches decides the wedge.
            if (CmdByteLog != null && sys >= 0xB0000 && sys < 0xB0020 && CmdByteLog.Count < 140)
                CmdByteLog.Add("R  phys=" + sys.ToString("X5") + " -> " + rv.ToString("X2") + " @IOP " + HostClock);
            // MP-940 hop-3: the WorkNotifier task reads workMaskCount (MOV DL,workMaskCount) and
            // walks workMaskConditionPtrs BEFORE the CMP DI,DX/JGE bail.  Log Opie-data READS in a
            // tight window just after doorbell #614 (IOP 25,621K) to expose both addresses.
            // MP-940: how far does the workNotifier scan actually WALK?  Watch reads of
            // workMaskCount (phys 0xA43B8) and the workMaskConditionPtrs table (0xA43BA..0xA43CF,
            // 8 occupied entries then zeros) during the post-doorbell runs.  The highest table
            // offset read = the observed scan extent -- no bit->index derivation needed.
            if (TblReadLog != null && TblReadLog.Count < 250 && HostClock > 25621100
                && sys >= 0xA43B8 && sys <= 0xA43D0)
                TblReadLog.Add("R phys 0x" + sys.ToString("X5") + " (tbl+0x" + (sys-0xA43BA).ToString("X2") + ") -> 0x" + rv.ToString("X2") + " @IOP" + HostClock);
            // MP-940 DISCRIMINATOR: the workNotifier task must READ workNotifierBits (MOV SI,ptr /
            // XCHG [SI],AX).  The down-notify ISR also reads it -- but there are NO doorbells after
            // #614, so ANY read of phys 0xA430E after that ISR completes can only be the workNotifier
            // task.  Zero such reads => the task never ran => break is upstream of WorkNtfr.
            if (WnbReadLog != null && WnbReadLog.Count < 200 && HostClock > 25621100 && (sys == 0xA430E || sys == 0xA430F))
                WnbReadLog.Add("R phys 0x" + sys.ToString("X5") + " -> 0x" + rv.ToString("X2") + " @IOP" + HostClock + " PC=" + CurrentPC.ToString("X5"));
            if (OpieReadLog != null && OpieReadLog.Count < 300
                && HostClock >= 25620950 && HostClock <= 25623000
                && sys >= 0xA4000 && sys < 0xA4800)
                OpieReadLog.Add("R phys 0x" + sys.ToString("X5") + " (lin 0x" + (sys-0xA0000).ToString("X4") + ") -> 0x" + rv.ToString("X2") + " @IOP" + HostClock);
            return rv;
        }

        public void WriteByte(int address, byte value)
        {
            address &= 0xFFFFF;

            if (address >= EpromBase)
            {
                // EPROM: read-only.
                return;
            }
            if (address < SramBase + SramSize)
            {
                if (OnSramWrite != null) OnSramWrite(address, value);
                // BOOTSTRAPIOR WATCH (0x03E00, IOP-local SRAM).  RamBoot's bootTask parks at
                // %WaitForCondition(bootBufferFull, noTimeout) and FloppyRead only proceeds when
                // flppyIOCB.OperationState (flppyIOCB+1) reads OperationCompleted = 6.  Every byte
                // that decides this lives in this 1KB window, so instead of guessing where flppyIOCB
                // sits, record the value SEQUENCE per address: the IOCB identifies itself by its
                // static image (OperationIsQueued=TRUE then OperationState=OperationWaiting=3) and
                // the question "does any byte here ever become 6?" is then answered by inspection.
                // Condition words read 0x0000 idle / 0x0001 stored-wakeup / 0x8000|ptr waiter.
                if (BootIorSeq != null && address >= 0x3C00 && address < 0x4000)
                {
                    System.Collections.Generic.List<byte> sq;
                    if (!BootIorSeq.TryGetValue(address, out sq))
                    { sq = new System.Collections.Generic.List<byte>(); BootIorSeq[address] = sq; }
                    if (sq.Count < 24) sq.Add(value);
                    if (BootIorLog != null && BootIorLog.Count < 600 && value >= 3 && value <= 7)
                        BootIorLog.Add("W 0x" + address.ToString("X4") + " (IOR+0x" + (address - 0x3E00).ToString("X3")
                            + ") <- " + value + StateName(value) + " PC=" + CurrentPC.ToString("X5") + " @IOP" + HostClock);
                }
                _sram[address - SramBase] = value;
                return;
            }

            int sys = TranslateMap(address);
            if (sys >= 0)
            {
                // MP-940 UP-NOTIFY WATCH: does the IOP ever write the mesaProcessor FCB's async
                // notify words -- word[0] notifiersLockMask @0xA7C30/31, word[1] upNotifyBits
                // @0xA7C32/33 -- or the Dekker pair mesaHasLock@0xA4000 / iopReqLock@0xA4002?
                // The CP busy-spins reading upNotifyBits==0x0000; if the IOP NEVER writes it, the
                // IOP->CP notify chain is the gate (germ polled, Pilot waits on conditions).
                if (NotifyWriteLog != null && NotifyWriteLog.Count < 300
                    && ((sys >= 0xA7C30 && sys <= 0xA7C33) || (sys >= 0xA4000 && sys <= 0xA4003)))
                {
                    string what = sys <= 0xA4003 ? (sys < 0xA4002 ? "mesaHasLock" : "iopReqLock")
                                                 : (sys < 0xA7C32 ? "notifiersLockMask" : "upNotifyBits");
                    NotifyWriteLog.Add("IOP W phys 0x" + sys.ToString("X5") + " (" + what + ") <- 0x" + value.ToString("X2") + " @IOP" + HostClock);
                }
                // MP-940: WHICH handler is the dispatched one, and where does it dead-end?  Watch IOP
                // writes to the floppy(0xA79B0)/disk(0xA45C0)/ethernet(0xA46F0) FCBs, but ONLY in the
                // Pilot stall window so germ-era traffic doesn't consume the cap.
                if (HostClock > 26000000
                    && ((sys >= 0xA79B0 && sys < 0xA7A30) || (sys >= 0xA45C0 && sys < 0xA4640) || (sys >= 0xA46F0 && sys < 0xA4770)))
                {
                    string h = sys >= 0xA79B0 ? "floppy" : sys >= 0xA46F0 ? "ethernet" : "disk";
                    int bas = sys >= 0xA79B0 ? 0xA79B0 : sys >= 0xA46F0 ? 0xA46F0 : 0xA45C0;
                    // LIFECYCLE (uncapped): when does the cycle start/stop, how many iterations?
                    if (h == "floppy") { FloppyFcbWrites++; if (FloppyFcbFirst == 0) FloppyFcbFirst = HostClock; FloppyFcbLast = HostClock;
                        // state histogram on the +0x0E state byte -- names the cycle
                        if (sys == 0xA79BE && FloppyStateHist != null) { long c; FloppyStateHist.TryGetValue(value, out c); FloppyStateHist[value] = c + 1; } }
                    else if (h == "disk") DiskFcbWrites++; else EtherFcbWrites++;
                    // sampled log so we see the whole span, not just the first 315 instructions
                    if (HandlerFcbLog != null && (FloppyFcbWrites % 20000 == 1 || HandlerFcbLog.Count < 12))
                        HandlerFcbLog.Add("IOP W " + h + "FCB+0x" + (sys - bas).ToString("X2") + " <- 0x" + value.ToString("X2") + " @IOP" + HostClock + " (#" + FloppyFcbWrites + ")");
                }
                // MP-940 hop-2.5: does the workNotifier TASK ever get SCHEDULED?  currentTaskTCBPtr
                // (lin 0x4354 = phys 0xA4354, confirmed: SystemIdle writes 0xFFFF there) records every
                // task dispatch.  Track distinct TCB pointers + first/last time; a task that is never
                // woken never appears.  If no NEW TCB shows up after doorbell #614 (IOP 25,621K), the
                // workNotifier task is never scheduled -> the break is upstream of WorkNtfr entirely.
                if (TcbTrack != null && sys == 0xA4355)
                {
                    int ptr = _system[0xA4354] | (value << 8);   // HIGH byte written last -> full word valid
                    long[] rec; if (!TcbTrack.TryGetValue(ptr, out rec)) { rec = new long[3]; rec[1] = HostClock; TcbTrack[ptr] = rec; }
                    rec[0]++; rec[2] = HostClock;
                }
                // MP-940 hop-3 / GetWorkMask (IOPKernl.asm:549-580): every handler registration
                // writes workMaskConditionPtrs[slot] (word) then bumps workMaskCount (byte) with
                // strictly-increasing small values, init-only.  Capture (a) per-address small-value
                // sequences to FIND workMaskCount by that signature, and (b) an ordered write log of
                // the Opie data region so each increment can be paired with its preceding word write
                // (which carries handlerID<<1 in its HIGH byte).
                if (WmcTrack != null && sys >= 0xA4000 && sys < 0xA8000 && value < 64)
                {
                    System.Collections.Generic.List<byte> lv;
                    if (!WmcTrack.TryGetValue(sys, out lv)) { lv = new System.Collections.Generic.List<byte>(); WmcTrack[sys] = lv; }
                    if (lv.Count < 32) lv.Add(value);
                }
                if (OpieInitLog != null && OpieInitLog.Count < 3000 && sys >= 0xA4300 && sys < 0xA4400)
                    OpieInitLog.Add(sys.ToString("X5") + " " + value.ToString("X2") + " " + HostClock);
                // MP-940 RMW DISCRIMINATOR: append WRITES of workNotifierBits to the same log as the
                // READS, so ordering is preserved.  XCHG [SI],AX is a read-modify-write => each read is
                // followed within a few instrs by a write (of 0).  A plain read (some other code) shows
                // reads with no paired write.  This decides "the task is looping and something re-arms
                // the bit" vs "the task is not the reader at all".
                if (WnbReadLog != null && WnbReadLog.Count < 200 && HostClock > 25621100
                    && (sys == 0xA430E || sys == 0xA430F))
                    WnbReadLog.Add("  W phys 0x" + sys.ToString("X5") + " <- 0x" + value.ToString("X2") + " @IOP" + HostClock + " PC=" + CurrentPC.ToString("X5"));
                // MP-940 hop-3: workNotifierBits lives in the DOWNLOADED RAM-Opie data (phys
                // 0xB0000+), not the 16KB SRAM.  Track address -> last 0x80/0x40/0x00 write after
                // IOP 25.5M; an address left LATCHED at 0x80 is workNotifierBits with the bits
                // never consumed (the WorkNtfr XCHG sits after the CMP DI,DX / JGE bail).
                if (WnbTrack != null && HostClock >= 25500000 && sys >= 0xA0000 && sys < 0xC0000
                    && (value == 0x80 || value == 0x40 || value == 0x00))
                {
                    int[] rec; if (!WnbTrack.TryGetValue(sys, out rec)) { rec = new int[3]; WnbTrack[sys] = rec; }
                    rec[0] = value; rec[1] = (int)(HostClock / 1000); rec[2]++;
                }
                // Diagnostic: track the vacant-stamp writes (high byte 0x60) into the map storage
                // [0x80000,0xA0000) -- to see whether the fill crosses the 64KB boundary at 0x90000.
                if (value == 0x60 && sys >= 0x80000 && sys < 0xA0000)
                {
                    MapStampCount++;
                    if (sys < MapStampMinSys) MapStampMinSys = sys;
                    if (sys > MapStampMaxSys) MapStampMaxSys = sys;
                    if (sys >= 0x90000) MapStampSeg2++;
                }
                // TRANSFER-COMPLETION WRITE WATCH: window by IOP time to the boot-transfer completion
                // (reads=115 at IOP ~16.2M) and log EVERY non-DMA-data write to the CP-visible DRAM, so we
                // catch whatever pollable status the IOP wrote at completion -- wherever it lives (config FCB
                // at 0xA7C3C, a separate FLOPPY FCB, or a BootChannel status).  If the only completion signal
                // is the doorbell (CSReg b8, rung at CPi 7347564) and NOTHING is written to DRAM here, an
                // interrupts-off germ can never observe completion.  Exclude the bulk DMA data dests
                // (phys 0xA0EF0 single-page buffer + the 0x131A00 high page) so status writes stand out.
                if (CmdByteLog != null && HostClock >= 15900000 && HostClock <= 16500000
                    && !(sys >= 0xA0EE0 && sys <= 0xA0F10) && !(sys >= 0x1319E0 && sys <= 0x131C10)
                    && CmdByteLog.Count < 200)
                    CmdByteLog.Add("W  phys=" + sys.ToString("X5") + " (CPword 0x" + (sys >> 1).ToString("X5") + ") <- " + value.ToString("X2") + " @IOP " + HostClock);
                // FLOPPY IOCB OperationState-write watch: catch every write of Completed(6)/Failed(7) into
                // the first64K where (addr-23) carries the operation signature (word1 function==0x0001 BE) ->
                // it's the OperationState byte (word 11 low = byte 23).  Snapshot the head's decision fields so
                // a Completed forward read can be DIFFED against the Failed C5/H0/R6 read.
                if (IocbLog != null && (value == 6 || value == 7) && sys >= 0x80000 && sys < 0xA0000 && IocbLog.Count < 400)
                {
                    int b = sys - 23;
                    if (b >= 0x80000 && _system[b + 2] == 0 && _system[b + 3] == 1)
                    {
                        byte[] r = _system;
                        int cyl = (r[b + 4] << 8) | r[b + 5];
                        int sec = (r[b + 6] << 8) | r[b + 7];
                        int curCmd = (r[b + 72] << 8) | r[b + 73];
                        int numCmd = (r[b + 70] << 8) | r[b + 71];
                        int totXfer = (r[b + 48] << 8) | r[b + 49];
                        int finalDma = (r[b + 66] << 8) | r[b + 67];
                        IocbLog.Add("OpState<-" + value + (value == 6 ? "(Completed)" : "(FAILED)")
                            + " C=" + cyl + " sec=" + sec + " curCmd=" + curCmd.ToString("X4") + " numCmd=" + numCmd.ToString("X4")
                            + " totXfer=" + totXfer + " finalDma=" + finalDma + " NRBRead=" + r[b + 143]
                            + " res=" + r[b + 144].ToString("X2") + r[b + 145].ToString("X2") + r[b + 146].ToString("X2") + r[b + 147].ToString("X2") + r[b + 148].ToString("X2") + r[b + 149].ToString("X2") + r[b + 150].ToString("X2")
                            + " iocb=" + b.ToString("X5") + " @IOP" + HostClock);
                    }
                }
                _system[sys] = value;
            }
        }

        /// <summary>TEMP: IOP-side reads/writes of the ProcessorHead FCB header (0xB0000..0xB001F).</summary>
        public System.Collections.Generic.List<string> CmdByteLog;
        /// <summary>TEMP: snapshot of the floppy IOCB head-decision fields at each OperationState write.</summary>
        public System.Collections.Generic.List<string> IocbLog;

        /// <summary>
        /// BOOTSTRAPIOR (0x03E00) write history, per address -> the sequence of values stored.
        /// NOTE the existing IocbLog above watches a DIFFERENT structure: it triggers on a write of
        /// 6 or 7 anywhere in DRAM [0x80000,0xA0000) and treats (addr-23) as the head, which is the
        /// rigid-disk IOCB layout.  The floppy boot IOCB puts OperationState at +1, in SRAM, and is
        /// not covered by it.
        /// </summary>
        public System.Collections.Generic.Dictionary<int, System.Collections.Generic.List<byte>> BootIorSeq;
        /// <summary>BOOTSTRAPIOR writes carrying an OperationState enum value (3..7), in order.</summary>
        public System.Collections.Generic.List<string> BootIorLog;

        private static string StateName(byte v)
        {
            switch (v)
            {
                case 3: return "(Waiting)";
                case 4: return "(InProgress)";
                case 5: return "(Aborted)";
                case 6: return "(COMPLETED)";
                case 7: return "(Failed)";
                default: return "";
            }
        }
        /// <summary>TEMP: current IOP instruction count, set by the harness for CmdByteLog timestamps.</summary>
        public long HostClock;
        /// <summary>Current IOP instruction address, set by the harness -- identifies WHO touches a watched cell.</summary>
        public int CurrentPC;

        /// <summary>MP-940: IOP writes to the mesaProcessor FCB notify words + the Dekker lock pair.</summary>
        public System.Collections.Generic.List<string> NotifyWriteLog;
        /// <summary>MP-940: IOP writes to the floppy/disk/ethernet handler FCBs during the Pilot stall.</summary>
        public System.Collections.Generic.List<string> HandlerFcbLog;
        /// <summary>MP-940 hop-3: DRAM addr -> {lastVal,lastTimeK,count} for 0x80/0x40/0x00 writes after IOP25.5M.</summary>
        public System.Collections.Generic.Dictionary<int,int[]> WnbTrack;
        /// <summary>MP-940 hop-3: Opie-data reads in the post-doorbell window (finds workMaskCount + workMaskConditionPtrs).</summary>
        public System.Collections.Generic.List<string> OpieReadLog;
        public System.Collections.Generic.Dictionary<int,System.Collections.Generic.List<byte>> WmcTrack;
        public System.Collections.Generic.List<string> OpieInitLog;
        /// <summary>MP-940: currentTaskTCBPtr value -> {count, firstIOP, lastIOP} = which tasks ever get scheduled.</summary>
        public System.Collections.Generic.Dictionary<int,long[]> TcbTrack;
        /// <summary>MP-940 discriminator: reads of workNotifierBits (phys 0xA430E).</summary>
        public System.Collections.Generic.List<string> WnbReadLog;
        /// <summary>MP-940: reads of workMaskCount + workMaskConditionPtrs = observed scan extent.</summary>
        public System.Collections.Generic.List<string> TblReadLog;
        /// <summary>MP-940 lifecycle: uncapped floppy/disk/ethernet FCB write counts + span + the +0x0E state histogram.</summary>
        public long FloppyFcbWrites, DiskFcbWrites, EtherFcbWrites, FloppyFcbFirst, FloppyFcbLast;
        public System.Collections.Generic.Dictionary<byte, long> FloppyStateHist;
        public long MapStampCount, MapStampSeg2;
        public int MapStampMinSys = int.MaxValue, MapStampMaxSys = -1;

        /// <summary>
        /// Relocate an 80186 address through the IOP window map registers (8-15).
        /// The 1 MB space is 8 pages of 128 KB; page p's register holds a 7-bit
        /// frame that replaces A17-A23, placing the page anywhere in system DRAM.
        /// Returns -1 for an unmapped (nil) page.
        /// </summary>
        private int TranslateMap(int address)
        {
            int page = (address >> 17) & 7;
            byte reg = _mapRegisters[8 + page];
            if (reg == NilMapData) return -1;
            return (((reg & 0x7F) << 17) | (address & 0x1FFFF)) & (SystemSize - 1);
        }

        /// <summary>
        /// Read a word directly from system/display DRAM by byte address, bypassing
        /// the IOP window map.  Used by the display DMA to scan the bitmap that the
        /// firmware wrote (via its map window) at bitMapOrg.
        /// </summary>
        public ushort ReadDisplayWord(int byteAddr)
        {
            byteAddr &= (SystemSize - 1);
            int hi = (byteAddr + 1) & (SystemSize - 1);
            return (ushort)(_system[byteAddr] | (_system[hi] << 8));
        }

        public ushort ReadWord(int address)
        {
            return (ushort)(ReadByte(address) | (ReadByte(address + 1) << 8));
        }

        public void WriteWord(int address, ushort value)
        {
            WriteByte(address, (byte)value);
            WriteByte(address + 1, (byte)(value >> 8));
        }

        /// <summary>Set one of the 16 map registers (E010-E01F); regs 8-15 are the IOP window.</summary>
        public void SetMapRegister(int index, byte value)
        {
            _mapRegisters[index & 0xF] = value;
        }

        public byte GetMapRegister(int index)
        {
            return _mapRegisters[index & 0xF];
        }

        /// <summary>Raw system/display DRAM backing (for diagnostics / VRAM dumps).</summary>
        public byte[] SystemRaw { get { return _system; } }

        /// <summary>
        /// The 16 KB IOP-LOCAL SRAM (0x00000-0x03FFF).  A separate array from SystemRaw, which is
        /// why a "memory dump" that only writes SystemRaw silently omits it -- the same trap that
        /// cost the MP 0149 hunt a day, and that just made a search for the CP microcode block come
        /// back empty when the boot buffer lives here.
        /// </summary>
        public byte[] SramRaw { get { return _sram; } }

        /// <summary>Diagnostic hook fired on every SRAM byte write (address, value).</summary>
        public System.Action<int, byte> OnSramWrite;

        private void LoadRom(string romPath)
        {
            using (FileStream fs = new FileStream(romPath, FileMode.Open, FileAccess.Read))
            {
                if (fs.Length != EpromSize)
                {
                    throw new InvalidOperationException(
                        String.Format("Dove boot ROM {0} has unexpected size 0x{1:X} (expected 0x{2:X})",
                            romPath, fs.Length, EpromSize));
                }
                fs.Read(_rom, 0, EpromSize);
            }
        }

        private const byte NilMapData = 0xFF;

        private readonly byte[] _rom;
        private readonly byte[] _sram;
        private readonly byte[] _system;
        private readonly byte[] _mapRegisters = new byte[16];
    }
}
