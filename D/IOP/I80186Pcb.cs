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

namespace D.IOP
{
    /// <summary>
    /// The 80186's integrated Peripheral Control Block (PCB): a 256-byte register
    /// file holding the chip-select unit, the two DMA channels, the three timers,
    /// the integrated interrupt controller, and the relocation register.
    ///
    /// The block is relocatable to any 256-byte boundary in either memory or I/O
    /// space via the relocation register at offset 0xFE.  After RESET the register
    /// is 0x20FFH, placing the PCB in I/O space at 0xFF00 (IOP-TR Table 2.2); the
    /// Dove firmware re-programs it to 0x60FFH to select iRMX interrupt mode (still
    /// I/O at 0xFF00).
    ///
    /// For the first bring-up milestone (run the boot ROM to the Opie idle loop)
    /// this models the chip-select / DMA / relocation registers as passive storage
    /// so POST writes succeed; the timers and interrupt controller are stubbed here
    /// and get real behaviour in a later pass (Dove IOP memory/IO environment).
    /// </summary>
    public class I80186Pcb
    {
        public I80186Pcb()
        {
            Reset();
        }

        public void Reset()
        {
            for (int i = 0; i < _regs.Length; i++)
            {
                _regs[i] = 0;
            }

            // Relocation register reset value: PCB in I/O space at 0xFF00.
            RelocationRegister = 0x20FF;

            // UMCS reset value (upper chip-select: 1 KB @ top, per Table 2.2).
            WriteRegisterWord(OffsetUMCS, 0xFFFB);
        }

        //
        // Relocation register (PCB offset 0xFE) and the window it selects.
        //

        public ushort RelocationRegister
        {
            get { return ReadRegisterWord(OffsetReloc); }
            set { WriteRegisterWord(OffsetReloc, value); }
        }

        /// <summary>
        /// True if the PCB currently responds in memory space; false if in I/O space.
        /// (Relocation register bit 12 = MEM/IO#.)
        /// </summary>
        public bool IsMemoryMapped
        {
            get { return (RelocationRegister & 0x1000) != 0; }
        }

        /// <summary>
        /// True if the integrated interrupt controller is in iRMX (slave) mode.
        /// (Relocation register bit 14.)
        /// </summary>
        public bool IsIRmxMode
        {
            get { return (RelocationRegister & 0x4000) != 0; }
        }

        /// <summary>
        /// Base address of the 256-byte PCB window in whichever space it occupies.
        /// (Relocation register bits 11..0 supply A19..A8.)
        /// </summary>
        public int WindowBase
        {
            get { return (RelocationRegister & 0x0FFF) << 8; }
        }

        /// <summary>
        /// If <paramref name="address"/> lies within the PCB window in the given space,
        /// returns true and sets <paramref name="offset"/> to the 0..255 register offset.
        /// </summary>
        public bool TryMapMemory(int address, out int offset)
        {
            return TryMap(IsMemoryMapped, address & 0xFFFFF, out offset);
        }

        public bool TryMapIO(int port, out int offset)
        {
            return TryMap(!IsMemoryMapped, port & 0xFFFF, out offset);
        }

        private bool TryMap(bool spaceMatches, int address, out int offset)
        {
            if (spaceMatches)
            {
                int b = WindowBase;
                if (address >= b && address < b + 0x100)
                {
                    offset = address - b;
                    return true;
                }
            }

            offset = 0;
            return false;
        }

        //
        // Register access (offset is 0..255 into the PCB).
        //

        public byte ReadByte(int offset)
        {
            return _regs[offset & 0xFF];
        }

        public void WriteByte(int offset, byte value)
        {
            // The integrated-PIC EOI register (0xFF22) is written as a byte by Opie's
            // ClearAndDismissInterrupt (OUT DX,AL) and as a word by POST -- both must
            // issue the end-of-interrupt.
            if ((offset & 0xFF) == OffsetEoi)
            {
                HandleEoi(value);
                return;
            }
            _regs[offset & 0xFF] = value;
        }

        public ushort ReadWord(int offset)
        {
            return ReadRegisterWord(offset);
        }

        public void WriteWord(int offset, ushort value)
        {
            if ((offset & 0xFF) == OffsetEoi)
            {
                HandleEoi(value);
                return;
            }
            WriteRegisterWord(offset, value);
        }

        // ---- Integrated interrupt controller (feeds the external master 8259 IR6) ----

        /// <summary>
        /// True when the integrated controller has an unmasked request that is not
        /// blocked by a higher-priority in-service interrupt.  Drives master IR6.
        /// </summary>
        public bool InternalInterruptPending
        {
            get
            {
                int ready = ReadRegisterWord(OffsetIntRequest) & ~ReadRegisterWord(OffsetIntMask);
                int isr = _autoEoi ? 0 : ReadRegisterWord(OffsetInService);
                for (int i = 0; i < 8; i++)
                {
                    int bit = 1 << i;
                    if ((isr & bit) != 0) return false;
                    if ((ready & bit) != 0) return true;
                }
                return false;
            }
        }

        /// <summary>
        /// Acknowledge (cascade INTA from the master 8259 on IR6): move the winning
        /// internal source to in-service and return its vector, or -1 if none.
        /// </summary>
        public int AcknowledgeInternal()
        {
            int ready = ReadRegisterWord(OffsetIntRequest) & ~ReadRegisterWord(OffsetIntMask);
            int isr = _autoEoi ? 0 : ReadRegisterWord(OffsetInService);
            for (int i = 0; i < 8; i++)
            {
                int bit = 1 << i;
                if ((isr & bit) != 0) break;
                if ((ready & bit) != 0)
                {
                    // POST reads the in-service register and expects the bit set, so we
                    // latch it in normal mode; auto-EOI mode (once Opie enables it) does
                    // not, so the ongoing timer heartbeat isn't blocked by a missing EOI.
                    if (!_autoEoi) WriteRegisterWord(OffsetInService, (ushort)(isr | bit));
                    WriteRegisterWord(OffsetIntRequest, (ushort)(ReadRegisterWord(OffsetIntRequest) & ~bit));
                    AckCount++; InternalAckByBit[i]++;
                    return (ReadRegisterWord(OffsetIntVector) & 0xF8) | i;
                }
            }
            return -1;
        }

        /// <summary>Auto-EOI mode for the integrated interrupt controller (Opie enables it after POST).</summary>
        public bool AutoEoi { get { return _autoEoi; } set { _autoEoi = value; } }
        private bool _autoEoi;

        // Diagnostics for the integrated interrupt controller + Timer 2.
        public readonly long[] InternalAckByBit = new long[8];   // WatchDog check: which internal-PIC sources actually get serviced (Timer2 = bit 5)
        public readonly long[] TimerFireByReq = new long[16];    // request-bit -> times the timer reached maxA and requested an int
        public ushort InternalRequest { get { return ReadRegisterWord(OffsetIntRequest); } }
        public ushort InternalInService { get { return ReadRegisterWord(OffsetInService); } }
        public ushort InternalMask { get { return ReadRegisterWord(OffsetIntMask); } }
        public ushort Timer2Mode { get { return ReadRegisterWord(OffsetT2Mode); } }
        public ushort Timer2Count { get { return ReadRegisterWord(OffsetT2Count); } }
        public ushort Timer2MaxA { get { return ReadRegisterWord(OffsetT2MaxA); } }

        /// <summary>Read a DMA/PCB register word by its window offset (e.g. 0xC4 = DMA0 dest ptr).</summary>
        public ushort GetRegisterWord(int offset) { return ReadRegisterWord(offset & 0xFF); }

        /// <summary>Write a DMA/PCB register word (used by the DMA controller to update count/pointers).</summary>
        public void SetRegisterWord(int offset, ushort value) { WriteRegisterWord(offset & 0xFF, value); }

        public long AckCount, EoiCount;

        private void HandleEoi(ushort value)
        {
            EoiCount++;
            int isr = ReadRegisterWord(OffsetInService);
            if ((value & 0x8000) != 0)
            {
                // Non-specific EOI: clear the highest-priority in-service bit.
                for (int i = 0; i < 8; i++)
                {
                    if ((isr & (1 << i)) != 0) { isr &= ~(1 << i); break; }
                }
            }
            else
            {
                isr &= ~(1 << (value & 0x07));
            }
            WriteRegisterWord(OffsetInService, (ushort)isr);
        }

        /// <summary>
        /// Advance the integrated timers by the given CPU clocks.  The 80186 timers
        /// count up; on reaching max-count A they set the MC bit and, if the timer's
        /// interrupt is enabled, set the corresponding request bit in the integrated
        /// interrupt controller's request register (offset 0x2E).  iRMX channel map
        /// (roadmap): Timer0 -> bit0, Timer1 -> bit4, Timer2 -> bit5.
        /// </summary>
        public void Tick(int clocks)
        {
            if (clocks <= 0) return;
            TickTimer(OffsetT0Count, OffsetT0MaxA, OffsetT0Mode, 0x0001, clocks);
            TickTimer(OffsetT1Count, OffsetT1MaxA, OffsetT1Mode, 0x0010, clocks);
            TickTimer(OffsetT2Count, OffsetT2MaxA, OffsetT2Mode, 0x0020, clocks);
        }

        /// <summary>
        /// Advance Timer 1 by a number of DMA acknowledges (on the Dove IOP the FDC
        /// DMA channel's DACK' feeds the timer-1 count input); on reaching max-count A
        /// it produces the terminal count that ends the FDC transfer, and sets its MC
        /// bit / interrupt request just as a clocked tick would.
        /// </summary>
        public void PulseTimer1(int acks)
        {
            // Count external (DMA-ack) pulses even though the periodic Tick skips this
            // timer (EXT mode); reaching max-count A wraps to 0 (the terminal count the
            // FDC driver polls) and, if enabled, requests an interrupt.
            int mode = ReadRegisterWord(OffsetT1Mode);
            if ((mode & 0x8000) == 0) return;   // EN not set
            int max = ReadRegisterWord(OffsetT1MaxA);
            if (max == 0) max = 0x10000;
            int count = ReadRegisterWord(OffsetT1Count) + acks;
            while (count >= max)
            {
                count -= max;
                mode |= 0x0020;                 // MC
                if ((mode & 0x2000) != 0)
                    WriteRegisterWord(OffsetIntRequest, (ushort)(ReadRegisterWord(OffsetIntRequest) | 0x0010));
            }
            WriteRegisterWord(OffsetT1Mode, (ushort)mode);
            WriteRegisterWord(OffsetT1Count, (ushort)count);
        }

        /// <summary>Drive timer 1's byte-count to its terminal count (0).  Called when the FDC
        /// DMA transfer terminates on the DMA transfer count (FFC8 drained to 0) BEFORE the DACK
        /// count reached timer-1 max-count A -- the real 80186 DMA TC ends the burst, so the
        /// timer-1 count the FloppyHeadDove driver reads as firstTrack.TotalBytesActuallyTransfered
        /// must be 0 (the residual, exactly like FinalDMACount).  Without this a short burst
        /// (e.g. 512B against a 1536B max-A) leaves timer 1 = 512 -> UpdateOperation:1567
        /// (FinalDMACount + firstTrack.TotalBytesActuallyTransfered # 0) fails the op forever.</summary>
        public void ClearTimer1Count() { WriteRegisterWord(OffsetT1Count, 0); }

        private void TickTimer(int countOff, int maxAOff, int modeOff, int requestBit, int clocks)
        {
            int mode = ReadRegisterWord(modeOff);
            if ((mode & 0x8000) == 0) return;   // EN not set -> not counting
            if ((mode & 0x0004) != 0) return;   // EXT set -> external clock (e.g. FDC DMA
                                                //   acks feeding timer 1); not CPU clocks

            int max = ReadRegisterWord(maxAOff);
            if (max == 0) max = 0x10000;

            int count = ReadRegisterWord(countOff) + clocks;
            while (count >= max)
            {
                count -= max;
                mode |= 0x0020;                 // MC (max count reached)
                if ((mode & 0x2000) != 0)       // INT enabled -> request an interrupt
                {
                    WriteRegisterWord(OffsetIntRequest, (ushort)(ReadRegisterWord(OffsetIntRequest) | requestBit));
                    for (int _b = 0; _b < 16; _b++) if ((requestBit & (1 << _b)) != 0) { TimerFireByReq[_b]++; break; }
                }
                else { for (int _b = 0; _b < 16; _b++) if ((requestBit & (1 << _b)) != 0) { TimerFireByReq[_b]++; break; } }  // reached maxA even if int disabled
                if ((mode & 0x0001) == 0)        // single (not continuous) -> stop
                {
                    mode &= unchecked((ushort)~0x8000);
                    count = 0;
                    break;
                }
            }

            WriteRegisterWord(modeOff, (ushort)mode);
            WriteRegisterWord(countOff, (ushort)count);
        }

        private ushort ReadRegisterWord(int offset)
        {
            offset &= 0xFF;
            return (ushort)(_regs[offset] | (_regs[(offset + 1) & 0xFF] << 8));
        }

        private void WriteRegisterWord(int offset, ushort value)
        {
            offset &= 0xFF;
            _regs[offset] = (byte)value;
            _regs[(offset + 1) & 0xFF] = (byte)(value >> 8);
        }

        // PCB register offsets (offsets from the 0xFF00 base; IOP-TR §2.2).
        private const int OffsetUMCS = 0xA0;
        private const int OffsetReloc = 0xFE;

        // Integrated interrupt controller (iRMX mode).
        private const int OffsetIntVector = 0x20;
        private const int OffsetEoi = 0x22;
        private const int OffsetIntMask = 0x28;
        private const int OffsetInService = 0x2C;
        private const int OffsetIntRequest = 0x2E;

        // Integrated timers (count / max-count A / mode).  T2 has no max-count B,
        // so its mode register is at 0x66 (0x64 unused).
        private const int OffsetT0Count = 0x50, OffsetT0MaxA = 0x52, OffsetT0Mode = 0x56;
        private const int OffsetT1Count = 0x58, OffsetT1MaxA = 0x5A, OffsetT1Mode = 0x5E;
        private const int OffsetT2Count = 0x60, OffsetT2MaxA = 0x62, OffsetT2Mode = 0x66;

        private readonly byte[] _regs = new byte[256];
    }
}
