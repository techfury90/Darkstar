using System;
using D.IOP;

// Standalone unit tests for the Dove CP Writable Control Store byte-lane packing.
// Build: csc ControlStoreTest.cs ../../../D/IOP/DoveControlStore.cs
class ControlStoreTest
{
    static int _pass = 0, _fail = 0;

    static void Check(string name, bool ok, string detail = "")
    {
        if (ok) { _pass++; }
        else { _fail++; Console.WriteLine("  FAIL: " + name + "  " + detail); }
    }

    static int Main()
    {
        // 1. Lane->port mapping: 0x8000=lane0(MSB) .. 0xD000=lane5(LSB).
        //    Writing byte 0xFF to a single lane must set exactly that lane's byte
        //    (or nibble split for lanes 3-5) and leave the rest zero.
        for (int lane = 0; lane <= 5; lane++)
        {
            var cs = new DoveControlStore();
            int port = 0x8000 | (lane << 12) | 0x123;   // csAddr 0x123
            cs.WriteLane(port, 0xFF);
            ulong w = cs.GetWord(0, 0x123);
            ulong expect = DoveControlStore.PackLane(0, lane, 0xFF);
            Check("lane" + lane + " port-decode", w == expect, "got " + w.ToString("X12") + " expect " + expect.ToString("X12"));
            Check("lane" + lane + " isolated-nonzero", w != 0);
            // Other csAddrs untouched.
            Check("lane" + lane + " other-addr-clean", cs.GetWord(0, 0x124) == 0);
        }

        // 2. Full 6-lane microword assembles the exact 48-bit value, matching the
        //    DLion CentralProcessorIO layout (bytes: MSB..LSB = b0..b5).
        //    Use a recognizable pattern and reconstruct by the documented fields.
        {
            var cs = new DoveControlStore();
            byte[] b = { 0x0D, 0x6D, 0x06, 0xD0, 0x38, 0x03 }; // arbitrary 6 lanes
            for (int lane = 0; lane <= 5; lane++)
                cs.WriteLane(0x8000 | (lane << 12) | 0x000, b[lane]);
            ulong w = cs.GetWord(0, 0);
            // Reconstruct expected from PackLane applied sequentially (reference).
            ulong exp = 0;
            for (int lane = 0; lane <= 5; lane++) exp = DoveControlStore.PackLane(exp, lane, b[lane]);
            Check("6-lane assemble", w == exp, "got " + w.ToString("X12") + " expect " + exp.ToString("X12"));
            // Top byte (lane0) must be b[0].
            Check("6-lane MSB", (byte)(w >> 40) == b[0], "MSB=" + ((byte)(w >> 40)).ToString("X2"));
            Check("6-lane byte1", (byte)(w >> 32) == b[1]);
            Check("6-lane byte2", (byte)(w >> 24) == b[2]);
        }

        // 3. Independent lane writes to the same word accumulate (no clobber).
        {
            var cs = new DoveControlStore();
            cs.WriteLane(0x8000, 0xAB);          // lane0
            cs.WriteLane(0x9000, 0xCD);          // lane1
            ulong w = cs.GetWord(0, 0);
            Check("accumulate lane0", (byte)(w >> 40) == 0xAB);
            Check("accumulate lane1", (byte)(w >> 32) == 0xCD);
        }

        // 4. Bank select via DayBreakBankMap codes [00,0C,0A,09].
        {
            var cs = new DoveControlStore();
            Check("bank default 0", cs.Bank == 0);
            cs.SelectBank(0x0C); Check("bank 0C->1", cs.Bank == 1);
            cs.SelectBank(0x0A); Check("bank 0A->2", cs.Bank == 2);
            cs.SelectBank(0x09); Check("bank 09->3", cs.Bank == 3);
            cs.SelectBank(0x00); Check("bank 00->0", cs.Bank == 0);
            cs.SelectBank(0xFF); Check("bank undefined-no-change", cs.Bank == 0);
        }

        // 5. Writes route to the selected bank only.
        {
            var cs = new DoveControlStore();
            cs.SelectBank(0x0C);                 // bank 1
            cs.WriteLane(0x8000 | 0x010, 0x77);
            Check("write to bank1", (byte)(cs.GetWord(1, 0x010) >> 40) == 0x77);
            Check("bank0 untouched", cs.GetWord(0, 0x010) == 0);
        }

        Console.WriteLine("ControlStore tests: " + _pass + " passed, " + _fail + " failed.");
        return _fail == 0 ? 0 : 1;
    }
}
