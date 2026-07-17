using System;
using D.IOP;

namespace DoveTest
{
    // ---- Simple 1 MB flat memory for the CPU tests ----
    class FlatMemory : IPhysicalMemory
    {
        public byte[] Mem = new byte[0x100000];
        public byte ReadByte(int a) { return Mem[a & 0xFFFFF]; }
        public void WriteByte(int a, byte v) { Mem[a & 0xFFFFF] = v; }
        public ushort ReadWord(int a) { return (ushort)(ReadByte(a) | (ReadByte(a + 1) << 8)); }
        public void WriteWord(int a, ushort v) { WriteByte(a, (byte)v); WriteByte(a + 1, (byte)(v >> 8)); }
    }

    class TestIO : IIOBus186
    {
        public int LastOutPort = -1, LastOutVal = -1;
        public int LastInPort = -1;
        public int NextIn = 0;
        public byte ReadByte(ushort p) { LastInPort = p; return (byte)NextIn; }
        public void WriteByte(ushort p, byte v) { LastOutPort = p; LastOutVal = v; }
        public ushort ReadWord(ushort p) { LastInPort = p; return (ushort)NextIn; }
        public void WriteWord(ushort p, ushort v) { LastOutPort = p; LastOutVal = v; }
    }

    class Program
    {
        // Register indices
        const int AX = 0, CX = 1, DX = 2, BX = 3, SP = 4, BP = 5, SI = 6, DI = 7;
        const int ES = 0, CS = 1, SS = 2, DS = 3;
        const int CODEBASE = 0x10000; // CS=0x1000

        static int _pass = 0, _fail = 0;

        static FlatMemory _mem;
        static TestIO _io;
        static I80186Pcb _pcb;
        static i80186 _cpu;

        static i80186 Setup()
        {
            _mem = new FlatMemory();
            _io = new TestIO();
            _pcb = new I80186Pcb();
            _cpu = new i80186(_mem, _io, _pcb);
            _cpu.SetSegNamed(CS, 0x1000);
            _cpu.SetIP(0x0000);
            _cpu.SetSegNamed(SS, 0x2000);
            _cpu.SetReg16Named(SP, 0x0100);
            _cpu.SetSegNamed(DS, 0x3000);
            _cpu.SetSegNamed(ES, 0x3000);
            _cpu.SetFlags(0x0000);
            return _cpu;
        }

        static void Load(int physBase, params int[] bytes)
        {
            for (int i = 0; i < bytes.Length; i++) _mem.WriteByte(physBase + i, (byte)bytes[i]);
        }

        static void Code(params int[] bytes) { Load(CODEBASE, bytes); }

        static void RunUntilHalt(int maxSteps = 100000)
        {
            int steps = 0;
            while (!_cpu.Halted && steps < maxSteps) { _cpu.Execute(); steps++; }
            if (steps >= maxSteps) throw new Exception("runaway (no HLT)");
        }

        static void Check(string name, bool cond)
        {
            if (cond) { _pass++; }
            else { _fail++; Console.WriteLine("  FAIL: " + name + "\n         state: " + _cpu); }
        }

        static void CheckEq(string name, int expected, int actual)
        {
            if (expected == actual) { _pass++; }
            else { _fail++; Console.WriteLine(String.Format("  FAIL: {0}  expected 0x{1:X4} got 0x{2:X4}\n         state: {3}", name, expected, actual, _cpu)); }
        }

        static void Main()
        {
            Console.WriteLine("== 80186 core unit tests ==");

            TestResetState();
            TestMovAndArith();
            TestFlagsDetail();
            TestIncDec();
            TestLogic();
            TestModRMMemory();
            TestSegOverride();
            TestLea();
            TestStack();
            TestPushaPopa();
            TestShiftsRotates();
            TestMulDiv();
            TestStrings();
            TestLoops();
            TestJcc();
            TestCallRet();
            TestFarCallRet();
            TestIntIret();
            Test186Ops();
            TestInOut();
            TestPcbInterception();

            Console.WriteLine(String.Format("\n== {0} passed, {1} failed ==", _pass, _fail));
            Environment.Exit(_fail == 0 ? 0 : 1);
        }

        // --- reset ---
        static void TestResetState()
        {
            var mem = new FlatMemory();
            var pcb = new I80186Pcb();
            var cpu = new i80186(mem, new TestIO(), pcb);
            _cpu = cpu;
            CheckEq("reset CS", 0xFFFF, cpu.GetCS);
            CheckEq("reset IP", 0x0000, cpu.IP);
            CheckEq("reset Flags", 0xF002, cpu.Flags);
            CheckEq("reset InstrAddr", 0xFFFF0, cpu.InstructionAddress);
            CheckEq("reset reloc", 0x20FF, pcb.RelocationRegister);
            int off;
            Check("PCB is I/O mapped", !pcb.IsMemoryMapped);
            Check("PCB window @ I/O 0xFF00", pcb.TryMapIO(0xFF00, out off) && off == 0);
            Check("PCB not memory @ 0xFF00", !pcb.TryMapMemory(0xFF00, out off));
        }

        // --- MOV + ADD basic ---
        static void TestMovAndArith()
        {
            Setup();
            // MOV AX,0x1234 ; ADD AX,0x1111 ; HLT
            Code(0xB8, 0x34, 0x12, 0x05, 0x11, 0x11, 0xF4);
            RunUntilHalt();
            CheckEq("mov/add AX", 0x2345, _cpu.GetAX);

            Setup();
            // MOV AL,0xFF ; ADD AL,0x02 ; HLT  -> 0x01, CF set
            Code(0xB0, 0xFF, 0x04, 0x02, 0xF4);
            RunUntilHalt();
            CheckEq("add8 wrap AL", 0x01, _cpu.GetAX & 0xFF);
            Check("add8 CF set", _cpu.FlagCF);
        }

        // --- detailed flag semantics ---
        static void TestFlagsDetail()
        {
            Setup();
            // MOV AL,0x7F ; ADD AL,1 -> 0x80 : OF=1,SF=1,ZF=0,CF=0,AF=1
            Code(0xB0, 0x7F, 0x04, 0x01, 0xF4);
            RunUntilHalt();
            CheckEq("0x7F+1 result", 0x80, _cpu.GetAX & 0xFF);
            Check("0x7F+1 OF", _cpu.FlagOF);
            Check("0x7F+1 SF", _cpu.FlagSF);
            Check("0x7F+1 !ZF", !_cpu.FlagZF);
            Check("0x7F+1 !CF", !_cpu.FlagCF);
            Check("0x7F+1 AF", _cpu.FlagAF);

            Setup();
            // MOV AL,0x05 ; SUB AL,0x05 -> ZF=1,CF=0
            Code(0xB0, 0x05, 0x2C, 0x05, 0xF4);
            RunUntilHalt();
            Check("sub->0 ZF", _cpu.FlagZF);
            Check("sub->0 !CF", !_cpu.FlagCF);

            Setup();
            // MOV AL,0x00 ; SUB AL,0x01 -> 0xFF, CF=1, SF=1, OF=0
            Code(0xB0, 0x00, 0x2C, 0x01, 0xF4);
            RunUntilHalt();
            CheckEq("0-1 result", 0xFF, _cpu.GetAX & 0xFF);
            Check("0-1 CF", _cpu.FlagCF);
            Check("0-1 SF", _cpu.FlagSF);
            Check("0-1 !OF", !_cpu.FlagOF);

            Setup();
            // CMP: MOV AL,0x10 ; CMP AL,0x20 -> AL unchanged, CF=1 (0x10<0x20)
            Code(0xB0, 0x10, 0x3C, 0x20, 0xF4);
            RunUntilHalt();
            CheckEq("cmp preserves AL", 0x10, _cpu.GetAX & 0xFF);
            Check("cmp CF", _cpu.FlagCF);

            Setup();
            // 16-bit carry out: MOV AX,0xFFFF ; ADD AX,0x0001 -> 0, CF=1, ZF=1
            Code(0xB8, 0xFF, 0xFF, 0x05, 0x01, 0x00, 0xF4);
            RunUntilHalt();
            CheckEq("16-bit wrap", 0x0000, _cpu.GetAX);
            Check("16-bit CF", _cpu.FlagCF);
            Check("16-bit ZF", _cpu.FlagZF);
        }

        static void TestIncDec()
        {
            Setup();
            // STC ; MOV AL,0xFF ; INC AL -> 0x00, ZF=1, AF=1, CF preserved (still 1)
            Code(0xF9, 0xB0, 0xFF, 0xFE, 0xC0, 0xF4);
            RunUntilHalt();
            CheckEq("inc 0xFF", 0x00, _cpu.GetAX & 0xFF);
            Check("inc ZF", _cpu.FlagZF);
            Check("inc preserves CF", _cpu.FlagCF);

            Setup();
            // MOV AX,0x0001 ; DEC AX -> 0, ZF=1
            Code(0xB8, 0x01, 0x00, 0x48, 0xF4);
            RunUntilHalt();
            CheckEq("dec AX", 0x0000, _cpu.GetAX);
            Check("dec ZF", _cpu.FlagZF);
        }

        static void TestLogic()
        {
            Setup();
            // STC ; MOV AL,0xF0 ; AND AL,0x0F -> 0, ZF=1, CF cleared, OF cleared
            Code(0xF9, 0xB0, 0xF0, 0x24, 0x0F, 0xF4);
            RunUntilHalt();
            CheckEq("and result", 0x00, _cpu.GetAX & 0xFF);
            Check("and ZF", _cpu.FlagZF);
            Check("and clears CF", !_cpu.FlagCF);

            Setup();
            // MOV AX,0x00FF ; XOR AX,0x0F0F -> 0x0FF0
            Code(0xB8, 0xFF, 0x00, 0x35, 0x0F, 0x0F, 0xF4);
            RunUntilHalt();
            CheckEq("xor result", 0x0FF0, _cpu.GetAX);
        }

        static void TestModRMMemory()
        {
            Setup();
            // MOV BX,0x0010 ; MOV SI,0x0004 ; MOV AX,0xBEEF ; MOV [BX+SI],AX ; MOV DX,[BX+SI]
            Code(0xBB, 0x10, 0x00,       // MOV BX,0x10
                 0xBE, 0x04, 0x00,       // MOV SI,0x04
                 0xB8, 0xEF, 0xBE,       // MOV AX,0xBEEF
                 0x89, 0x00,             // MOV [BX+SI],AX
                 0x8B, 0x10,             // MOV DX,[BX+SI]
                 0xF4);
            RunUntilHalt();
            CheckEq("modrm store->load DX", 0xBEEF, _cpu.GetDX);
            // DS base 0x30000 + 0x14 = 0x30014
            CheckEq("modrm memory byte lo", 0xEF, _mem.ReadByte(0x30014));
            CheckEq("modrm memory byte hi", 0xBE, _mem.ReadByte(0x30015));

            Setup();
            // disp8 form: MOV DI,0x20 ; MOV byte [DI+4],0x99 ; MOV AL,[DI+4]
            Code(0xBF, 0x20, 0x00,            // MOV DI,0x20
                 0xC6, 0x45, 0x04, 0x99,      // MOV byte [DI+4],0x99
                 0x8A, 0x45, 0x04,            // MOV AL,[DI+4]
                 0xF4);
            RunUntilHalt();
            CheckEq("disp8 modrm", 0x99, _cpu.GetAX & 0xFF);

            Setup();
            // direct address: MOV word [0x0040],0x1234 ; MOV BX,[0x0040]
            Code(0xC7, 0x06, 0x40, 0x00, 0x34, 0x12,   // MOV word [0x40],0x1234
                 0x8B, 0x1E, 0x40, 0x00,               // MOV BX,[0x40]
                 0xF4);
            RunUntilHalt();
            CheckEq("direct16 modrm", 0x1234, _cpu.GetBX);
        }

        static void TestSegOverride()
        {
            Setup();
            // ES base 0x30000, DS base 0x40000 (change DS)
            _cpu.SetSegNamed(DS, 0x4000);
            // MOV BX,0x10 ; MOV byte ES:[BX],0x77 (26 C6 07 77) ; MOV AL, ES:[BX]
            Code(0xBB, 0x10, 0x00,
                 0x26, 0xC6, 0x07, 0x77,   // MOV byte ES:[BX],0x77
                 0x26, 0x8A, 0x07,         // MOV AL,ES:[BX]
                 0xF4);
            RunUntilHalt();
            CheckEq("seg override value", 0x77, _cpu.GetAX & 0xFF);
            CheckEq("seg override wrote ES mem", 0x77, _mem.ReadByte(0x30010));
            CheckEq("seg override left DS mem clear", 0x00, _mem.ReadByte(0x40010));
        }

        static void TestLea()
        {
            Setup();
            // MOV SI,0x0030 ; LEA BX,[SI+2] -> BX=0x32
            Code(0xBE, 0x30, 0x00, 0x8D, 0x5C, 0x02, 0xF4);
            RunUntilHalt();
            CheckEq("lea", 0x0032, _cpu.GetBX);
        }

        static void TestStack()
        {
            Setup();
            // MOV AX,0xCAFE ; PUSH AX ; POP BX
            Code(0xB8, 0xFE, 0xCA, 0x50, 0x5B, 0xF4);
            RunUntilHalt();
            CheckEq("push/pop", 0xCAFE, _cpu.GetBX);
            CheckEq("push/pop SP restored", 0x0100, _cpu.GetSP);
        }

        static void TestPushaPopa()
        {
            Setup();
            // set regs, PUSHA, clobber, POPA
            _cpu.SetReg16Named(AX, 0x1111);
            _cpu.SetReg16Named(CX, 0x2222);
            _cpu.SetReg16Named(DX, 0x3333);
            _cpu.SetReg16Named(BX, 0x4444);
            _cpu.SetReg16Named(BP, 0x5555);
            _cpu.SetReg16Named(SI, 0x6666);
            _cpu.SetReg16Named(DI, 0x7777);
            // PUSHA ; MOV AX,0 ; MOV CX,0 ; POPA ; HLT
            Code(0x60, 0xB8, 0x00, 0x00, 0xB9, 0x00, 0x00, 0x61, 0xF4);
            RunUntilHalt();
            CheckEq("pusha/popa AX", 0x1111, _cpu.GetAX);
            CheckEq("pusha/popa CX", 0x2222, _cpu.GetCX);
            CheckEq("pusha/popa DI", 0x7777, _cpu.GetDI);
            CheckEq("pusha/popa SP", 0x0100, _cpu.GetSP);
        }

        static void TestShiftsRotates()
        {
            Setup();
            // MOV AL,0x81 ; SHL AL,1 -> 0x02, CF=1, OF=1
            Code(0xB0, 0x81, 0xD0, 0xE0, 0xF4);
            RunUntilHalt();
            CheckEq("shl result", 0x02, _cpu.GetAX & 0xFF);
            Check("shl CF", _cpu.FlagCF);

            Setup();
            // MOV AL,0x01 ; SHR AL,1 -> 0x00, CF=1
            Code(0xB0, 0x01, 0xD0, 0xE8, 0xF4);
            RunUntilHalt();
            CheckEq("shr result", 0x00, _cpu.GetAX & 0xFF);
            Check("shr CF", _cpu.FlagCF);

            Setup();
            // MOV AL,0x80 ; SAR AL,1 -> 0xC0 (sign extend)
            Code(0xB0, 0x80, 0xD0, 0xF8, 0xF4);
            RunUntilHalt();
            CheckEq("sar result", 0xC0, _cpu.GetAX & 0xFF);

            Setup();
            // MOV AL,0x01 ; SHL AL,imm 4 (C0 /4 ib) -> 0x10
            Code(0xB0, 0x01, 0xC0, 0xE0, 0x04, 0xF4);
            RunUntilHalt();
            CheckEq("shl imm count", 0x10, _cpu.GetAX & 0xFF);

            Setup();
            // ROL: MOV AL,0x80 ; ROL AL,1 -> 0x01, CF=1
            Code(0xB0, 0x80, 0xD0, 0xC0, 0xF4);
            RunUntilHalt();
            CheckEq("rol result", 0x01, _cpu.GetAX & 0xFF);
            Check("rol CF", _cpu.FlagCF);

            Setup();
            // RCL through carry: STC ; MOV AL,0x00 ; RCL AL,1 -> 0x01, CF=0
            Code(0xF9, 0xB0, 0x00, 0xD0, 0xD0, 0xF4);
            RunUntilHalt();
            CheckEq("rcl brings in carry", 0x01, _cpu.GetAX & 0xFF);
            Check("rcl CF cleared", !_cpu.FlagCF);
        }

        static void TestMulDiv()
        {
            Setup();
            // MOV AL,0x10 ; MOV BL,0x10 ; MUL BL -> AX=0x0100, CF/OF=1
            Code(0xB0, 0x10, 0xB3, 0x10, 0xF6, 0xE3, 0xF4);
            RunUntilHalt();
            CheckEq("mul8", 0x0100, _cpu.GetAX);
            Check("mul8 CF (hi nonzero)", _cpu.FlagCF);

            Setup();
            // IMUL8: MOV AL,0xFF(-1) ; MOV BL,0x02 ; IMUL BL -> AX=0xFFFE(-2)
            Code(0xB0, 0xFF, 0xB3, 0x02, 0xF6, 0xEB, 0xF4);
            RunUntilHalt();
            CheckEq("imul8", 0xFFFE, _cpu.GetAX);

            Setup();
            // DIV8: AX=0x0100, BL=0x10 -> AL=0x10 (quot), AH=0x00 (rem)
            Code(0xB8, 0x00, 0x01, 0xB3, 0x10, 0xF6, 0xF3, 0xF4);
            RunUntilHalt();
            CheckEq("div8 quotient", 0x10, _cpu.GetAX & 0xFF);
            CheckEq("div8 remainder", 0x00, (_cpu.GetAX >> 8) & 0xFF);

            Setup();
            // 16-bit MUL: MOV AX,0x1000 ; MOV BX,0x0010 ; MUL BX -> DX:AX=0x0001:0x0000
            Code(0xB8, 0x00, 0x10, 0xBB, 0x10, 0x00, 0xF7, 0xE3, 0xF4);
            RunUntilHalt();
            CheckEq("mul16 AX", 0x0000, _cpu.GetAX);
            CheckEq("mul16 DX", 0x0001, _cpu.GetDX);
        }

        static void TestStrings()
        {
            Setup();
            // REP STOSB: fill 4 bytes at ES:DI with AL=0xAA
            _cpu.SetReg16Named(AX, 0x00AA);
            _cpu.SetReg16Named(CX, 0x0004);
            _cpu.SetReg16Named(DI, 0x0000);
            // CLD ; REP STOSB ; HLT
            Code(0xFC, 0xF3, 0xAA, 0xF4);
            RunUntilHalt();
            CheckEq("rep stosb count", 0x0000, _cpu.GetCX);
            Check("rep stosb filled",
                _mem.ReadByte(0x30000) == 0xAA && _mem.ReadByte(0x30003) == 0xAA && _mem.ReadByte(0x30004) == 0x00);
            CheckEq("rep stosb DI advanced", 0x0004, _cpu.GetDI);

            Setup();
            // MOVSW: source DS:SI -> dest ES:DI (both base 0x30000). Seed source.
            _mem.WriteByte(0x30010, 0x21); _mem.WriteByte(0x30011, 0x43);
            _cpu.SetReg16Named(SI, 0x0010);
            _cpu.SetReg16Named(DI, 0x0020);
            _cpu.SetReg16Named(CX, 0x0001);
            Code(0xFC, 0xF3, 0xA5, 0xF4); // CLD ; REP MOVSW
            RunUntilHalt();
            Check("rep movsw copied", _mem.ReadByte(0x30020) == 0x21 && _mem.ReadByte(0x30021) == 0x43);

            Setup();
            // REPE CMPSB on equal buffers of length 3 -> ZF=1, CX=0
            for (int i = 0; i < 3; i++) { _mem.WriteByte(0x30010 + i, 0x55); _mem.WriteByte(0x30020 + i, 0x55); }
            _cpu.SetReg16Named(SI, 0x0010);
            _cpu.SetReg16Named(DI, 0x0020);
            _cpu.SetReg16Named(CX, 0x0003);
            Code(0xFC, 0xF3, 0xA6, 0xF4); // CLD ; REPE CMPSB
            RunUntilHalt();
            CheckEq("repe cmpsb consumed CX", 0x0000, _cpu.GetCX);
            Check("repe cmpsb equal ZF", _cpu.FlagZF);

            Setup();
            // REPNE SCASB scanning for 0x07 in {1,2,7,9}; ES:DI, AL=0x07
            _mem.WriteByte(0x30030, 1); _mem.WriteByte(0x30031, 2); _mem.WriteByte(0x30032, 7); _mem.WriteByte(0x30033, 9);
            _cpu.SetReg16Named(AX, 0x0007);
            _cpu.SetReg16Named(DI, 0x0030);
            _cpu.SetReg16Named(CX, 0x0004);
            Code(0xFC, 0xF2, 0xAE, 0xF4); // CLD ; REPNE SCASB
            RunUntilHalt();
            Check("repne scasb found ZF", _cpu.FlagZF);
            CheckEq("repne scasb stopped at index 3", 0x0033, _cpu.GetDI); // DI points past the match (0x30..+3)
        }

        static void TestLoops()
        {
            Setup();
            // sum 1..5 via LOOP: MOV CX,5 ; XOR AX,AX ; loop: ADD AX,CX ; LOOP loop
            Code(0xB9, 0x05, 0x00,        // MOV CX,5
                 0x31, 0xC0,              // XOR AX,AX
                 0x01, 0xC8,              // ADD AX,CX     (loop target = offset 5)
                 0xE2, 0xFC,              // LOOP -4  -> back to offset 5 (ADD AX,CX)
                 0xF4);
            RunUntilHalt();
            CheckEq("loop sum 5+4+3+2+1", 15, _cpu.GetAX);

            Setup();
            // JCXZ taken: MOV CX,0 ; JCXZ +1 ; (skipped) MOV AL,0xFF ; HLT
            Code(0xB9, 0x00, 0x00, 0xE3, 0x02, 0xB0, 0xFF, 0xF4);
            RunUntilHalt();
            CheckEq("jcxz skipped mov", 0x00, _cpu.GetAX & 0xFF);
        }

        static void TestJcc()
        {
            Setup();
            // MOV AL,5 ; CMP AL,5 ; JNE +2 ; MOV AL,0x11 (executed since equal) ; HLT
            Code(0xB0, 0x05, 0x3C, 0x05, 0x75, 0x02, 0xB0, 0x11, 0xF4);
            RunUntilHalt();
            CheckEq("jne not taken (equal)", 0x11, _cpu.GetAX & 0xFF);

            Setup();
            // MOV AL,5 ; CMP AL,6 ; JB set AL=0x22 else 0x33
            // CMP AL,6 (5<6 => CF=1) ; JB +2 -> skip the "MOV AL,0x33"
            Code(0xB0, 0x05, 0x3C, 0x06, 0x72, 0x02, 0xB0, 0x33, 0xB0, 0x22, 0xF4);
            RunUntilHalt();
            CheckEq("jb taken", 0x22, _cpu.GetAX & 0xFF);
        }

        static void TestCallRet()
        {
            Setup();
            // CALL +3 (skip HLT? ) ; MOV AL,0xEE ; HLT ; sub: MOV AL,0x44 ; RET
            // layout: 0:E8 03 00 (call to 6) ; 3:.. ; Let's do:
            // 0: E8 04 00      CALL rel16 -> target = 3 + 4 = 7
            // 3: B0 EE         MOV AL,0xEE
            // 5: F4            HLT
            // 6: 90            NOP (pad)
            // 7: B0 44         MOV AL,0x44   (subroutine)
            // 9: C3            RET
            Code(0xE8, 0x04, 0x00, 0xB0, 0xEE, 0xF4, 0x90, 0xB0, 0x44, 0xC3);
            RunUntilHalt();
            // After call returns to 3, MOV AL,0xEE overwrites 0x44 -> final 0xEE
            CheckEq("call/ret returned to caller", 0xEE, _cpu.GetAX & 0xFF);
            CheckEq("call/ret SP restored", 0x0100, _cpu.GetSP);
        }

        static void TestFarCallRet()
        {
            Setup();
            // Far call to CS=0x1000, off=0x0010; sub sets BX=0x55AA, RETF
            // 0: 9A 10 00 00 10   CALLF 0x1000:0x0010
            // 5: F4               HLT
            Code(0x9A, 0x10, 0x00, 0x00, 0x10, 0xF4);
            // subroutine at physical 0x10010
            Load(0x10010, 0xBB, 0xAA, 0x55, 0xCB); // MOV BX,0x55AA ; RETF
            RunUntilHalt();
            CheckEq("far call/ret BX", 0x55AA, _cpu.GetBX);
            CheckEq("far call/ret SP restored", 0x0100, _cpu.GetSP);
        }

        static void TestIntIret()
        {
            Setup();
            // IVT entry for INT 0x40 at phys 0x100: IP=0x0020, CS=0x1000
            _mem.WriteWord(0x40 * 4, 0x0020);
            _mem.WriteWord(0x40 * 4 + 2, 0x1000);
            // 0: CD 40    INT 0x40
            // 2: B0 EE    MOV AL,0xEE
            // 4: F4       HLT
            Code(0xCD, 0x40, 0xB0, 0xEE, 0xF4);
            // handler at 0x10020: MOV BX,0x1234 ; IRET
            Load(0x10020, 0xBB, 0x34, 0x12, 0xCF);
            RunUntilHalt();
            CheckEq("int handler ran (BX)", 0x1234, _cpu.GetBX);
            CheckEq("iret resumed (AL)", 0xEE, _cpu.GetAX & 0xFF);
            CheckEq("int/iret SP restored", 0x0100, _cpu.GetSP);
        }

        static void Test186Ops()
        {
            Setup();
            // PUSH imm16 then POP: 68 34 12 ; 58 -> AX=0x1234
            Code(0x68, 0x34, 0x12, 0x58, 0xF4);
            RunUntilHalt();
            CheckEq("push imm16/pop", 0x1234, _cpu.GetAX);

            Setup();
            // PUSH imm8 (sign extended) 6A FF -> 0xFFFF ; POP AX
            Code(0x6A, 0xFF, 0x58, 0xF4);
            RunUntilHalt();
            CheckEq("push imm8 sign-ext/pop", 0xFFFF, _cpu.GetAX);

            Setup();
            // IMUL BX,BX,imm8: MOV BX,0x0003 ; IMUL BX,BX,4 (6B DB 04) -> BX=0x0C
            Code(0xBB, 0x03, 0x00, 0x6B, 0xDB, 0x04, 0xF4);
            RunUntilHalt();
            CheckEq("imul imm8", 0x000C, _cpu.GetBX);

            Setup();
            // ENTER 4,0 then LEAVE restores BP/SP
            _cpu.SetReg16Named(BP, 0xABCD);
            // C8 04 00 00 (ENTER 4,0) ; C9 (LEAVE)
            Code(0xC8, 0x04, 0x00, 0x00, 0xC9, 0xF4);
            RunUntilHalt();
            CheckEq("enter/leave restores BP", 0xABCD, _cpu.GetBP);
            CheckEq("enter/leave restores SP", 0x0100, _cpu.GetSP);

            Setup();
            // BOUND in range: bounds [0x30040]=lower 0, [0x30042]=upper 10 ; AX=5 -> no trap
            _mem.WriteWord(0x30040, 0x0000);
            _mem.WriteWord(0x30042, 0x000A);
            _cpu.SetReg16Named(AX, 0x0005);
            _cpu.SetReg16Named(SI, 0x0040);
            // 62 /r reg=AX rm=SI mod00 -> 0x04 ; BOUND AX,[SI]
            Code(0x62, 0x04, 0xB0, 0x77, 0xF4); // then MOV AL,0x77 to prove no trap
            RunUntilHalt();
            CheckEq("bound in-range continued", 0x77, _cpu.GetAX & 0xFF);

            Setup();
            // BOUND out of range -> INT5; set IVT[5] to a handler that sets BX and HLTs
            _mem.WriteWord(0x30040, 0x0000);
            _mem.WriteWord(0x30042, 0x000A);
            _mem.WriteWord(5 * 4, 0x0030);   // handler IP
            _mem.WriteWord(5 * 4 + 2, 0x1000); // handler CS
            _cpu.SetReg16Named(AX, 0x00FF); // 255 > 10 -> out of range
            _cpu.SetReg16Named(SI, 0x0040);
            Code(0x62, 0x04, 0xB0, 0x77, 0xF4);
            Load(0x10030, 0xBB, 0x05, 0x00, 0xF4); // MOV BX,5 ; HLT
            RunUntilHalt();
            CheckEq("bound out-of-range trapped INT5", 0x0005, _cpu.GetBX);
        }

        static void TestInOut()
        {
            Setup();
            // IN AL, 0x42 (canned 0x5A)
            _io.NextIn = 0x5A;
            Code(0xE4, 0x42, 0xF4);
            RunUntilHalt();
            CheckEq("in al,imm8 value", 0x5A, _cpu.GetAX & 0xFF);
            CheckEq("in al,imm8 port", 0x42, _io.LastInPort);

            Setup();
            // MOV AL,0x99 ; OUT 0x21,AL
            Code(0xB0, 0x99, 0xE6, 0x21, 0xF4);
            RunUntilHalt();
            CheckEq("out imm8,al port", 0x21, _io.LastOutPort);
            CheckEq("out imm8,al val", 0x99, _io.LastOutVal);

            Setup();
            // MOV DX,0x1234 ; MOV AL,0x77 ; OUT DX,AL
            Code(0xBA, 0x34, 0x12, 0xB0, 0x77, 0xEE, 0xF4);
            RunUntilHalt();
            CheckEq("out dx,al port", 0x1234, _io.LastOutPort);
            CheckEq("out dx,al val", 0x77, _io.LastOutVal);
        }

        static void TestPcbInterception()
        {
            Setup();
            // OUT to I/O 0xFF20 (a PCB register, interrupt-vector reg) must land in PCB,
            // not the external I/O bus.  Use OUT DX,AX with DX=0xFF20, AX=0x1234.
            Code(0xBA, 0x20, 0xFF, 0xB8, 0x34, 0x12, 0xEF, 0xF4); // MOV DX,0xFF20;MOV AX,0x1234;OUT DX,AX
            RunUntilHalt();
            Check("PCB write not seen on I/O bus", _io.LastOutPort != 0xFF20);
            CheckEq("PCB register stored", 0x1234, _pcb.ReadWord(0x20));

            Setup();
            // Reprogram reloc to memory-map the PCB at 0x000 would collide with RAM; instead
            // verify that writing reloc (0xFFFE) via OUT changes the window base.
            // MOV DX,0xFFFE ; MOV AX,0x11FF (mem-mapped, base 0x1FF00) ; OUT DX,AX
            Code(0xBA, 0xFE, 0xFF, 0xB8, 0xFF, 0x11, 0xEF, 0xF4);
            RunUntilHalt();
            Check("reloc now memory-mapped", _pcb.IsMemoryMapped);
            CheckEq("reloc window base", 0x1FF00, _pcb.WindowBase);
        }
    }
}
