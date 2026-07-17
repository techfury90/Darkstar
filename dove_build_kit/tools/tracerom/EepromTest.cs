using System;
using System.IO;
using D.IOP;

// Round-trip test of the 93C46 serial model: drive the EXACT WriteConfigReg bit
// sequence the boot ROM's ReadEEProm (FC5B5) uses, reconstruct the 16-bit value
// from DataOut the way the firmware does, and compare to the loaded image word.
class EepromTest
{
    static I93C46 _e;

    static ushort Read(int addr)
    {
        int bl = (0x80 | addr) & 0xFF;
        _e.WriteConfig(0x0000);            // CS low (reset), as after a previous read
        _e.WriteConfig(0x1000);            // FC5BA: CS=1, clk=0, DI=0
        _e.WriteConfig(0x3000);            // FC5BF: clk=1 (DI=0)
        _e.WriteConfig(0x1000);            // FC5C4: clk=0
        int ax = 0x1000 | 0x8000;          // FC5C9: DI=1 (start bit), clk=0
        for (int i = 0; i < 9; i++)        // FC5CC: cx=9 command bits (start + opcode + addr)
        {
            _e.WriteConfig(ax);            // FC5CF: out (clk=0)
            _e.WriteConfig(ax ^ 0x2000);   // FC5D1: clk=1  (rising edge clocks DI)
            _e.WriteConfig(ax);            // FC5D6: clk=0
            bl <<= 1;                      // FC5DB: shl bl,1
            bool carry = (bl & 0x100) != 0; bl &= 0xFF;
            if (carry) ax |= 0x8000; else ax &= 0x7FFF;   // next DI from bl's shifted-out bit
        }
        int val = 0;
        for (int i = 0; i < 16; i++)       // FC5EB: cx=16 data bits
        {
            _e.WriteConfig(0x3000);        // FC5EE/F1: clk=1 (rising)
            _e.WriteConfig(0x1000);        // FC5F3: clk=0
            bool doBit = _e.DataOut;        // FC5F8: read DO (input port bit 11)
            val = (val << 1) | (doBit ? 1 : 0);   // FC600: rcl bx,1 (MSB first)
        }
        _e.WriteConfig(0x0000);            // CS low
        return (ushort)val;
    }

    static void Main(string[] args)
    {
        string path = args.Length > 0 ? args[0]
            : "C:/Users/techf/Desktop/Darkstar/.claude/worktrees/laughing-gould-251705/dove_build_kit/firmware/config/U128_IOP_sn_071888_08CA_8kCS_3.7mb.bin";
        _e = new I93C46();
        _e.Load(File.ReadAllBytes(path));

        int mismatches = 0;
        for (int a = 0; a < 64; a++)
        {
            ushort got = Read(a);
            ushort exp = _e.Words[a];
            if (got != exp)
            {
                mismatches++;
                if (mismatches <= 10) Console.WriteLine(String.Format("  addr {0,2}: serial={1:X4} image={2:X4}  MISMATCH", a, got, exp));
            }
        }
        Console.WriteLine(mismatches == 0
            ? "SERIAL ROUND-TRIP OK: all 64 words delivered correctly"
            : ("SERIAL BROKEN: " + mismatches + " mismatches"));
    }
}
