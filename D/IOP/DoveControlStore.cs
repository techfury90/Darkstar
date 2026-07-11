using System;

namespace D.IOP
{
    /// <summary>
    /// The Dove/6085 CP Writable Control Store (WCS).
    ///
    /// The IOP loads the Central Processor's microcode into this store by streaming
    /// the .db boot file's Cx/Dx ("CP-Write") blocks straight out its I/O ports; the
    /// blocks are never buffered in IOP RAM (which is why the boot appears to stream
    /// through a small window).  Each 48-bit AM2901 microword is written as six byte
    /// lanes, one OUT per lane, over the port window 0x8000-0xDFFF:
    ///
    ///   port = 0x8000 | (lane &lt;&lt; 12) | csAddr        (csAddr = low 12 bits, 4K)
    ///   lane 0 = MSB @0x8000  ...  lane 5 = LSB @0xD000
    ///
    /// (DybrkCP.WriteDybrkControlStore.)  The 48-bit field packing is identical to the
    /// DLion CP (see Darkstar/D/CP/CentralProcessorIO.cs:WriteIOPMicrocodeWord); the
    /// only difference is that Dove writes the lane bytes verbatim -- there is no
    /// end-around ~value complement (the INIA low nibble is pre-complemented in the
    /// .db at build time, by MakeDoveMicroBoot.mesa).
    ///
    /// The active bank is selected by a byte OUT to the daybreakBankRegister (port
    /// 0xE000) before each block; the byte is a DayBreakBankMap code, not the bank
    /// index (DayBreakBankMap = [00, 0C, 0A, 09]).
    /// </summary>
    public sealed class DoveControlStore
    {
        public const int WordsPerBank = 4096;   // csAddr is 12 bits
        public const int NumBanks = 4;          // DayBreakBankMap has 4 entries

        // daybreakBankRegister byte -> logical bank index.
        private static readonly byte[] BankMap = { 0x00, 0x0C, 0x0A, 0x09 };

        private readonly ulong[][] _words;      // [bank][csAddr] raw 48-bit microword
        private int _bank;

        public DoveControlStore()
        {
            _words = new ulong[NumBanks][];
            for (int i = 0; i < NumBanks; i++) _words[i] = new ulong[WordsPerBank];
        }

        /// <summary>Currently selected bank index (0..NumBanks-1).</summary>
        public int Bank { get { return _bank; } }

        /// <summary>Total lane bytes written (diagnostic).</summary>
        public long LaneWrites { get; private set; }

        /// <summary>Highest csAddr touched in each bank (diagnostic: how much loaded).</summary>
        public readonly int[] MaxAddr = new int[NumBanks];

        /// <summary>Select the active bank from a daybreakBankRegister byte code.</summary>
        public void SelectBank(byte bankCode)
        {
            for (int i = 0; i < NumBanks; i++)
            {
                if (BankMap[i] == bankCode) { _bank = i; return; }
            }
            // Undefined code: leave the bank unchanged (matches HW: no decode -> no change).
        }

        /// <summary>
        /// Write one byte-lane of a microword given the raw WCS port (0x8000-0xDFFF).
        /// </summary>
        public void WriteLane(int port, byte value)
        {
            int lane = ((port >> 12) & 0xF) - 8;         // 0x8000->0 .. 0xD000->5
            if (lane < 0 || lane > 5) return;
            int csAddr = port & 0x0FFF;
            _words[_bank][csAddr] = PackLane(_words[_bank][csAddr], lane, value);
            LaneWrites++;
            if (csAddr > MaxAddr[_bank]) MaxAddr[_bank] = csAddr;
        }

        /// <summary>Raw 48-bit microword from the active bank.</summary>
        public ulong GetWord(int csAddr) { return _words[_bank][csAddr & 0x0FFF]; }

        /// <summary>Raw 48-bit microword from a specific bank.</summary>
        public ulong GetWord(int bank, int csAddr) { return _words[bank & 3][csAddr & 0x0FFF]; }

        /// <summary>
        /// Merge one lane byte into a 48-bit AM2901 microword.  Field layout is
        /// identical to the DLion CP's WriteIOPMicrocodeWord (CentralProcessorIO.cs):
        ///   lane 0: bits 47:40   (rA/rB)
        ///   lane 1: bits 39:32   (aS/aF/aD)
        ///   lane 2: bits 31:24   (EP/CIN/EnSU/mem/fS)
        ///   lane 3: FY[0:3] (bits 19:16), INIA[0:3]  (bits 11:8)
        ///   lane 4: FX[0:3] (bits 23:20), INIA[4:7]  (bits  7:4)
        ///   lane 5: FZ[0:3] (bits 15:12), INIA[8:11] (bits  3:0)
        /// Dove writes verbatim (no complement).
        /// </summary>
        public static ulong PackLane(ulong word, int lane, byte value)
        {
            // Daybreak WCS is CONTIGUOUS bytes (ground-truth disassembly, operator answer 9):
            // the .db streams 6 bytes/word MSB-first and lane N drops into bits [47-8N:40-8N].
            // (NOT the DLion interleaved fY/fX/fZ+INIA-nibble split.)
            if (lane < 0 || lane > 5) return word;
            int shift = 40 - 8 * lane;                 // lane0->40 .. lane5->0
            ulong mask = ~(0xFFUL << shift);
            return (word & mask) | ((ulong)value << shift);
        }
    }
}
