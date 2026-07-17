using System;
using D.IOP;
using D.CP;

// Unit tests for the DoveCentralProcessor core sequencer, using hand-built micro-
// programs loaded through the real WCS lane path.
// Build: csc CpTest.cs ../../../D/IOP/DoveControlStore.cs ../../../D/CP/DoveCentralProcessor.cs
//            ../../../D/CP/Microinstruction.cs ../../../D/CP/AM2901.cs
class CpTest
{
    static int _pass = 0, _fail = 0;
    static void Check(string name, bool ok, string detail = "")
    { if (ok) _pass++; else { _fail++; Console.WriteLine("  FAIL: " + name + "  " + detail); } }

    // Pack microword fields into the 48-bit word (layout per Microinstruction ctor).
    static ulong Mi(int rA, int rB, int aS, int aF, int aD, bool ep, bool cin, bool enSU,
                    bool mem, int fSfY, int fSfZ, int fX, int fY, int fZ, int inia)
    {
        return ((ulong)(rA & 0xf) << 44) | ((ulong)(rB & 0xf) << 40) | ((ulong)(aS & 0x7) << 37)
             | ((ulong)(aF & 0x7) << 34) | ((ulong)(aD & 0x3) << 32) | (ep ? 1UL << 31 : 0)
             | (cin ? 1UL << 30 : 0) | (enSU ? 1UL << 29 : 0) | (mem ? 1UL << 28 : 0)
             | ((ulong)(fSfY & 0x3) << 26) | ((ulong)(fSfZ & 0x3) << 24) | ((ulong)(fX & 0xf) << 20)
             | ((ulong)(fY & 0xf) << 16) | ((ulong)(fZ & 0xf) << 12) | ((ulong)(inia & 0xfff));
    }

    // Load one 48-bit word into the control store the way the IOP does (6 lanes).
    static void Load(DoveControlStore cs, int addr, ulong w)
    {
        // Contiguous bytes, MSB-first: lane N -> bits [47-8N : 40-8N].
        for (int L = 0; L < 6; L++)
            cs.WriteLane(0x8000 | (L << 12) | addr, (byte)((w >> (40 - 8 * L)) & 0xff));
    }

    // Field constants.
    const int fyNorm = 1, Byte = 3;
    const int fzNorm = 0, Nibble = 1;
    const int Noop_fX = 8, Noop_fY = 8, Noop0_fZ = 8;
    const int D0 = 7, RplusS = 0;

    static DoveCentralProcessor MakeRunning(DoveControlStore cs)
    {
        var cp = new DoveCentralProcessor(cs);
        cp.SetReset(false);
        cp.WriteCSReg(0x4200);
        return cp;
    }

    static int Main()
    {
        // 0. Lane round-trip: a packed word loads back byte-identical.
        {
            var cs = new DoveControlStore();
            ulong w = Mi(1, 3, 5, 2, 1, false, true, false, false, fyNorm, fzNorm, 9, 4, 5, 0xABC);
            Load(cs, 0x123, w);
            Check("lane round-trip", cs.GetWord(0, 0x123) == w,
                  "got " + cs.GetWord(0, 0x123).ToString("X12") + " want " + w.ToString("X12"));
            var mi = new Microinstruction(cs.GetWord(0, 0x123));
            Check("decode rA", mi.rA == 1); Check("decode rB", mi.rB == 3);
            Check("decode INIA", mi.INIA == 0xABC, "INIA=" + mi.INIA.ToString("X3"));
        }

        // 1. Control gating: no execution until reset released AND run set.
        {
            var cs = new DoveControlStore();
            Load(cs, 0, Mi(0, 0, 0, 0, 1, false, false, false, false, fyNorm, fzNorm, Noop_fX, Noop_fY, Noop0_fZ, 0));
            var cp = new DoveCentralProcessor(cs);
            cp.Execute(5); Check("held in reset", cp.InstructionCount == 0);
            cp.SetReset(false); cp.Execute(5); Check("no run bit", cp.InstructionCount == 0);
            cp.WriteCSReg(0x4200); cp.Execute(5); Check("runs", cp.InstructionCount == 5);
            cp.WriteCSReg(0x0000); cp.Execute(5); Check("halt clears run", cp.InstructionCount == 5);
        }

        // 2. Sequencer: NIA walks 0 -> 5 -> 0x10 -> (self-loop).
        {
            var cs = new DoveControlStore();
            // INIA is stored complemented (low nibble): store target^0x00F to jump to target.
            Load(cs, 0x000, Mi(0, 0, 0, 0, 1, false, false, false, false, fyNorm, fzNorm, Noop_fX, Noop_fY, Noop0_fZ, 0x005 ^ 0x00F));
            Load(cs, 0x005, Mi(0, 0, 0, 0, 1, false, false, false, false, fyNorm, fzNorm, Noop_fX, Noop_fY, Noop0_fZ, 0x010 ^ 0x00F));
            Load(cs, 0x010, Mi(0, 0, 0, 0, 1, false, false, false, false, fyNorm, fzNorm, Noop_fX, Noop_fY, Noop0_fZ, 0x010 ^ 0x00F));
            var cp = MakeRunning(cs);
            cp.Execute(1); Check("nia step1 ->5", cp.CurrentAddress == 0x005, "@" + cp.CurrentAddress.ToString("X3"));
            cp.Execute(1); Check("nia step2 ->10", cp.CurrentAddress == 0x010, "@" + cp.CurrentAddress.ToString("X3"));
            cp.Execute(3); Check("nia self-loop", cp.CurrentAddress == 0x010);
        }

        // 3. ALU + register: nibble constant 5 -> R[3] (aS=D0, aF=R+S, aD=2 writes R[rB]).
        {
            var cs = new DoveControlStore();
            Load(cs, 0, Mi(0, 3, D0, RplusS, 2, false, false, false, false, fyNorm, Nibble, Noop_fX, Noop_fY, 5, 1));
            var cp = MakeRunning(cs);
            cp.Execute(1);
            Check("ALU R[3]=const", cp.ALU.R[3] == 5, "R3=" + cp.ALU.R[3].ToString("X4"));
        }

        // 4. Memory: MDR<- writes Y bus to [MAR] across C1(MAR<-)/C2(MDR<-).
        {
            var cs = new DoveControlStore();
            // C1: MAR<- from R[0] (aS=ZA gives S=R[rA]); simpler: put a constant via nibble
            // into MAR low.  aS=D0 R=D=const, aD=1 Y=f; mem=1 => MAR<- in c1 = (RH[rB]<<16)|Y.
            Load(cs, 0, Mi(0, 0, D0, RplusS, 1, false, false, false, true, fyNorm, Nibble, Noop_fX, Noop_fY, 0x8, 1 ^ 0x00F)); // MAR<-=0x0008, ->1
            // C2: MDR<- writes Y (nibble 0xA) to [MAR].
            Load(cs, 1, Mi(0, 0, D0, RplusS, 1, false, false, false, true, fyNorm, Nibble, Noop_fX, Noop_fY, 0xA, 2 ^ 0x00F)); // MDR<- 0x000A, ->2
            Load(cs, 2, Mi(0, 0, 0, 0, 1, false, false, false, false, fyNorm, fzNorm, Noop_fX, Noop_fY, Noop0_fZ, 2 ^ 0x00F));
            var cp = MakeRunning(cs);
            int wroteAddr = -1; ushort wroteVal = 0;
            cp.WriteWord = (a, v) => { wroteAddr = a; wroteVal = v; };
            cp.ReadWord = a => 0;
            cp.Execute(2);
            Check("MDR<- addr", wroteAddr == 0x0008, "addr=" + wroteAddr.ToString("X"));
            Check("MDR<- data", wroteVal == 0x000A, "val=" + wroteVal.ToString("X"));
        }

        Console.WriteLine("DoveCP tests: " + _pass + " passed, " + _fail + " failed.");
        return _fail == 0 ? 0 : 1;
    }
}
