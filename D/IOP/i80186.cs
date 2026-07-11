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

namespace D.IOP
{
    /// <summary>
    /// Emulates the Intel 80186 in real (8086-compatible) mode -- the Dove IOP's
    /// I/O processor.  Implements the full 8086 ISA plus the 80186 additions
    /// (PUSHA/POPA, ENTER/LEAVE, BOUND, PUSH imm, IMUL imm, INS/OUTS, immediate
    /// shift/rotate counts, and the 5-bit shift-count mask).  Protected mode does
    /// not exist on the 80186 and is not modeled.
    ///
    /// The integrated peripherals (PCB) are intercepted here: an access that falls
    /// in the relocation-register-selected 256-byte window is routed to the
    /// <see cref="I80186Pcb"/> instead of the external memory / I/O bus, exactly as
    /// the silicon does.
    /// </summary>
    public class i80186
    {
        public i80186(IPhysicalMemory mem, IIOBus186 io, I80186Pcb pcb)
        {
            _mem = mem;
            _io = io;
            _pcb = pcb;

            BuildParityTable();
            Reset();
        }

        /// <summary>
        /// RESET state per IOP-TR Table 2.2: execution begins at physical 0xFFFF0
        /// (CS=0xFFFF, IP=0), status word = 0xF002, PCB relocation register = 0x20FF
        /// (PCB @ I/O 0xFF00), UMCS = 0xFFFB.
        /// </summary>
        public void Reset()
        {
            for (int i = 0; i < 8; i++) _reg[i] = 0;

            _seg[ES] = 0x0000;
            _seg[CS] = 0xFFFF;
            _seg[SS] = 0x0000;
            _seg[DS] = 0x0000;

            _ip = 0x0000;
            _flags = 0xF002;

            _halted = false;
            _nmiPending = false;
            _hwInterrupts = 0;
            _lastHwVector = -1;

            _pcb.Reset();
        }

        // -------- Register / flag indices --------

        private const int AX = 0, CX = 1, DX = 2, BX = 3, SP = 4, BP = 5, SI = 6, DI = 7;
        private const int ES = 0, CS = 1, SS = 2, DS = 3;

        // Byte-register indices: 0=AL,1=CL,2=DL,3=BL,4=AH,5=CH,6=DH,7=BH
        private const int AL = 0, CL = 1, DL = 2, BL = 3, AH = 4, CH = 5, DH = 6, BH = 7;

        // Flag bit masks.
        private const int CF = 0x0001;
        private const int PF = 0x0004;
        private const int AF = 0x0010;
        private const int ZF = 0x0040;
        private const int SF = 0x0080;
        private const int TF = 0x0100;
        private const int IF = 0x0200;
        private const int DF = 0x0400;
        private const int OF = 0x0800;

        // -------- Public state accessors (debug / integration) --------

        public ushort GetAX { get { return _reg[AX]; } }
        public ushort GetBX { get { return _reg[BX]; } }
        public ushort GetCX { get { return _reg[CX]; } }
        public ushort GetDX { get { return _reg[DX]; } }
        public ushort GetSP { get { return _reg[SP]; } }
        public ushort GetBP { get { return _reg[BP]; } }
        public ushort GetSI { get { return _reg[SI]; } }
        public ushort GetDI { get { return _reg[DI]; } }
        public ushort GetCS { get { return _seg[CS]; } }
        public ushort GetDS { get { return _seg[DS]; } }
        public ushort GetES { get { return _seg[ES]; } }
        public ushort GetSS { get { return _seg[SS]; } }
        public ushort IP { get { return _ip; } }
        public ushort Flags { get { return (ushort)_flags; } }
        public bool Halted { get { return _halted; } }

        // Direct register set for the test harness.
        public void SetReg16Named(int idx, ushort v) { _reg[idx & 7] = v; }
        public ushort GetReg16Named(int idx) { return _reg[idx & 7]; }
        public void SetSegNamed(int idx, ushort v) { _seg[idx & 3] = v; }
        public void SetIP(ushort v) { _ip = v; }
        public void SetFlags(ushort v) { _flags = v | 0xF002; }
        public bool FlagCF { get { return GetFlag(CF); } }
        public bool FlagZF { get { return GetFlag(ZF); } }
        public bool FlagSF { get { return GetFlag(SF); } }
        public bool FlagOF { get { return GetFlag(OF); } }
        public bool FlagPF { get { return GetFlag(PF); } }
        public bool FlagAF { get { return GetFlag(AF); } }
        public bool FlagIF { get { return GetFlag(IF); } }
        public bool FlagDF { get { return GetFlag(DF); } }

        public I80186Pcb Pcb { get { return _pcb; } }

        /// <summary>Count of hardware interrupts (NMI + maskable) serviced since reset.</summary>
        public long HardwareInterruptCount { get { return _hwInterrupts; } }

        /// <summary>The most recently serviced maskable-interrupt vector.</summary>
        public int LastHardwareVector { get { return _lastHwVector; } }

        /// <summary>Physical address of the next instruction (CS:IP).</summary>
        public int InstructionAddress { get { return ((_seg[CS] << 4) + _ip) & 0xFFFFF; } }

        public override string ToString()
        {
            return String.Format(
                "AX={0:X4} BX={1:X4} CX={2:X4} DX={3:X4} SP={4:X4} BP={5:X4} SI={6:X4} DI={7:X4} " +
                "CS={8:X4} DS={9:X4} ES={10:X4} SS={11:X4} IP={12:X4} F={13:X4} [{14}]",
                _reg[AX], _reg[BX], _reg[CX], _reg[DX], _reg[SP], _reg[BP], _reg[SI], _reg[DI],
                _seg[CS], _seg[DS], _seg[ES], _seg[SS], _ip, _flags & 0xFFFF, FlagString());
        }

        private string FlagString()
        {
            return String.Concat(
                GetFlag(OF) ? "O" : "-", GetFlag(DF) ? "D" : "-", GetFlag(IF) ? "I" : "-",
                GetFlag(TF) ? "T" : "-", GetFlag(SF) ? "S" : "-", GetFlag(ZF) ? "Z" : "-",
                GetFlag(AF) ? "A" : "-", GetFlag(PF) ? "P" : "-", GetFlag(CF) ? "C" : "-");
        }

        // -------- Interrupt signalling --------

        /// <summary>Assert the non-maskable interrupt (edge-triggered).</summary>
        public void RaiseNmi() { _nmiPending = true; }

        /// <summary>
        /// Hook the external interrupt controller: when IF is set and this returns a
        /// vector (>= 0), the CPU services it as a maskable hardware interrupt.  The
        /// hook is expected to perform the INTA / priority resolution and return the
        /// vector byte, or -1 if no interrupt is pending.
        /// </summary>
        public Func<int> InterruptAcknowledge;

        /// <summary>Diagnostic hook fired on a software INT n (Opie SVCs): (vector, calling physical address).</summary>
        public Action<int, int> OnSoftwareInterrupt;

        // -------- Main execution --------

        /// <summary>
        /// Executes one instruction (or services one pending interrupt) and returns
        /// an approximate number of 8 MHz CLKOUT cycles consumed.
        /// </summary>
        public int Execute()
        {
            int cycles = ExecuteOne();
            _pcb.Tick(cycles);
            return cycles;
        }

        private int ExecuteOne()
        {
            // Hardware interrupts are sampled at instruction boundaries.
            if (_nmiPending)
            {
                _nmiPending = false;
                _halted = false;
                _hwInterrupts++;
                ServiceInterrupt(2);
                return 45;
            }

            if (GetFlag(IF) && InterruptAcknowledge != null)
            {
                int vector = InterruptAcknowledge();
                if (vector >= 0)
                {
                    _halted = false;
                    _hwInterrupts++;
                    _lastHwVector = vector;
                    ServiceInterrupt(vector);
                    return 45;
                }
            }

            if (_halted)
            {
                // Waiting for an interrupt; burn an idle cycle.
                return 4;
            }

            // Prefix bytes.
            _segOverride = -1;
            int rep = 0;                // 0 none, 1 REP/REPE/REPZ, 2 REPNE/REPNZ
            _cycles = 0;

            while (true)
            {
                byte b = Fetch8();
                switch (b)
                {
                    case 0x26: _segOverride = ES; continue;
                    case 0x2E: _segOverride = CS; continue;
                    case 0x36: _segOverride = SS; continue;
                    case 0x3E: _segOverride = DS; continue;
                    case 0xF0: continue;              // LOCK - no effect in emulation
                    case 0xF2: rep = 2; continue;    // REPNE
                    case 0xF3: rep = 1; continue;    // REP / REPE
                    default:
                        Dispatch(b, rep);
                        return _cycles > 0 ? _cycles : 4;
                }
            }
        }

        private void Dispatch(byte op, int rep)
        {
            // Arithmetic / logic block: 0x00-0x3D where (op & 7) <= 5.
            if (op < 0x40 && (op & 7) <= 5)
            {
                ArithGroup(op);
                return;
            }

            switch (op)
            {
                // ---- segment PUSH/POP (fill-ins in the arithmetic block) ----
                case 0x06: Push16(_seg[ES]); break;
                case 0x07: _seg[ES] = Pop16(); break;
                case 0x0E: Push16(_seg[CS]); break;
                case 0x16: Push16(_seg[SS]); break;
                case 0x17: _seg[SS] = Pop16(); break;
                case 0x1E: Push16(_seg[DS]); break;
                case 0x1F: _seg[DS] = Pop16(); break;

                case 0x27: Daa(); break;
                case 0x2F: Das(); break;
                case 0x37: Aaa(); break;
                case 0x3F: Aas(); break;

                case 0x0F: ServiceInterrupt(6); break;  // invalid opcode on the 80186

                // ---- INC/DEC r16 ----
                case 0x40: case 0x41: case 0x42: case 0x43:
                case 0x44: case 0x45: case 0x46: case 0x47:
                    SetReg16(op & 7, Inc16(GetReg16(op & 7)));
                    break;
                case 0x48: case 0x49: case 0x4A: case 0x4B:
                case 0x4C: case 0x4D: case 0x4E: case 0x4F:
                    SetReg16(op & 7, Dec16(GetReg16(op & 7)));
                    break;

                // ---- PUSH/POP r16 ----
                case 0x50: case 0x51: case 0x52: case 0x53:
                case 0x54: case 0x55: case 0x56: case 0x57:
                    Push16(GetReg16(op & 7));   // 80186 PUSH SP pushes the pre-decrement value
                    break;
                case 0x58: case 0x59: case 0x5A: case 0x5B:
                case 0x5C: case 0x5D: case 0x5E: case 0x5F:
                    SetReg16(op & 7, Pop16());
                    break;

                // ---- 80186 additions ----
                case 0x60: Pusha(); break;
                case 0x61: Popa(); break;
                case 0x62: Bound(); break;
                case 0x68: Push16(Fetch16()); break;                         // PUSH imm16
                case 0x69: ImulImm(true); break;                             // IMUL r16,rm16,imm16
                case 0x6A: Push16((ushort)(sbyte)Fetch8()); break;           // PUSH imm8 (sign-extended)
                case 0x6B: ImulImm(false); break;                            // IMUL r16,rm16,imm8
                case 0x6C: StringOp(rep, 0x6C); break;                       // INSB
                case 0x6D: StringOp(rep, 0x6D); break;                       // INSW
                case 0x6E: StringOp(rep, 0x6E); break;                       // OUTSB
                case 0x6F: StringOp(rep, 0x6F); break;                       // OUTSW

                // ---- Jcc rel8 ----
                case 0x70: case 0x71: case 0x72: case 0x73:
                case 0x74: case 0x75: case 0x76: case 0x77:
                case 0x78: case 0x79: case 0x7A: case 0x7B:
                case 0x7C: case 0x7D: case 0x7E: case 0x7F:
                    Jcc(op & 0x0F);
                    break;

                // ---- grp1: rm,imm ----
                case 0x80: Grp1(8, false); break;    // rm8, imm8
                case 0x81: Grp1(16, false); break;   // rm16, imm16
                case 0x82: Grp1(8, false); break;    // rm8, imm8 (alias)
                case 0x83: Grp1(16, true); break;    // rm16, imm8 sign-extended

                case 0x84: ReadModRM(); { int r = GetReg8(_modrmReg); TestFlags8(ReadRM8() & r); } break;   // TEST rm8,r8
                case 0x85: ReadModRM(); { int r = GetReg16(_modrmReg); TestFlags16(ReadRM16() & r); } break; // TEST rm16,r16
                case 0x86: ReadModRM(); { int a = ReadRM8(); int r = GetReg8(_modrmReg); WriteRM8(r); SetReg8(_modrmReg, a); } break; // XCHG rm8,r8
                case 0x87: ReadModRM(); { int a = ReadRM16(); int r = GetReg16(_modrmReg); WriteRM16(r); SetReg16(_modrmReg, a); } break;

                // ---- MOV ----
                case 0x88: ReadModRM(); WriteRM8(GetReg8(_modrmReg)); break;
                case 0x89: ReadModRM(); WriteRM16(GetReg16(_modrmReg)); break;
                case 0x8A: ReadModRM(); SetReg8(_modrmReg, (byte)ReadRM8()); break;
                case 0x8B: ReadModRM(); SetReg16(_modrmReg, (ushort)ReadRM16()); break;
                case 0x8C: ReadModRM(); WriteRM16(_seg[_modrmReg & 3]); break;         // MOV rm16, sreg
                case 0x8D: ReadModRM(); SetReg16(_modrmReg, (ushort)_modrmOff); break; // LEA
                case 0x8E: ReadModRM(); _seg[_modrmReg & 3] = (ushort)ReadRM16(); break; // MOV sreg, rm16
                case 0x8F: ReadModRM(); WriteRM16(Pop16()); break;                    // POP rm16

                // ---- XCHG AX,r16 / NOP ----
                case 0x90: break; // NOP
                case 0x91: case 0x92: case 0x93: case 0x94:
                case 0x95: case 0x96: case 0x97:
                    { ushort t = _reg[AX]; _reg[AX] = _reg[op & 7]; _reg[op & 7] = t; }
                    break;

                case 0x98: // CBW
                    SetReg8(AH, (byte)((GetReg8(AL) & 0x80) != 0 ? 0xFF : 0x00));
                    break;
                case 0x99: // CWD
                    _reg[DX] = (ushort)((_reg[AX] & 0x8000) != 0 ? 0xFFFF : 0x0000);
                    break;
                case 0x9A: // CALL far ptr16:16
                    {
                        ushort noff = Fetch16();
                        ushort nseg = Fetch16();
                        Push16(_seg[CS]); Push16(_ip);
                        _seg[CS] = nseg; _ip = noff;
                    }
                    break;
                case 0x9B: break; // WAIT
                case 0x9C: Push16((ushort)(_flags | 0xF002)); break; // PUSHF
                case 0x9D: _flags = Pop16() | 0x0002; break;         // POPF
                case 0x9E: // SAHF
                    _flags = (_flags & 0xFF00) | (GetReg8(AH) & 0xD5) | 0x02;
                    break;
                case 0x9F: // LAHF
                    SetReg8(AH, (byte)((_flags & 0xD5) | 0x02));
                    break;

                // ---- MOV AL/AX <-> [disp16] ----
                case 0xA0: { int off = Fetch16(); SetReg8(AL, ReadSeg8(DataSeg(), off)); } break;
                case 0xA1: { int off = Fetch16(); _reg[AX] = ReadSeg16(DataSeg(), off); } break;
                case 0xA2: { int off = Fetch16(); WriteSeg8(DataSeg(), off, GetReg8(AL)); } break;
                case 0xA3: { int off = Fetch16(); WriteSeg16(DataSeg(), off, _reg[AX]); } break;

                case 0xA4: StringOp(rep, 0xA4); break; // MOVSB
                case 0xA5: StringOp(rep, 0xA5); break; // MOVSW
                case 0xA6: StringOp(rep, 0xA6); break; // CMPSB
                case 0xA7: StringOp(rep, 0xA7); break; // CMPSW
                case 0xA8: TestFlags8(GetReg8(AL) & Fetch8()); break;         // TEST AL,imm8
                case 0xA9: TestFlags16(_reg[AX] & Fetch16()); break;          // TEST AX,imm16
                case 0xAA: StringOp(rep, 0xAA); break; // STOSB
                case 0xAB: StringOp(rep, 0xAB); break; // STOSW
                case 0xAC: StringOp(rep, 0xAC); break; // LODSB
                case 0xAD: StringOp(rep, 0xAD); break; // LODSW
                case 0xAE: StringOp(rep, 0xAE); break; // SCASB
                case 0xAF: StringOp(rep, 0xAF); break; // SCASW

                // ---- MOV r,imm ----
                case 0xB0: case 0xB1: case 0xB2: case 0xB3:
                case 0xB4: case 0xB5: case 0xB6: case 0xB7:
                    SetReg8(op & 7, Fetch8());
                    break;
                case 0xB8: case 0xB9: case 0xBA: case 0xBB:
                case 0xBC: case 0xBD: case 0xBE: case 0xBF:
                    SetReg16(op & 7, Fetch16());
                    break;

                // ---- grp2 shifts/rotates ----
                case 0xC0: ReadModRM(); Grp2(8, Fetch8()); break;             // rm8, imm8
                case 0xC1: ReadModRM(); Grp2(16, Fetch8()); break;            // rm16, imm8
                case 0xD0: ReadModRM(); Grp2(8, 1); break;                    // rm8, 1
                case 0xD1: ReadModRM(); Grp2(16, 1); break;                   // rm16, 1
                case 0xD2: ReadModRM(); Grp2(8, GetReg8(CL)); break;          // rm8, CL
                case 0xD3: ReadModRM(); Grp2(16, GetReg8(CL)); break;         // rm16, CL

                // ---- RET / LES / LDS / MOV rm,imm ----
                case 0xC2: { int n = Fetch16(); _ip = Pop16(); _reg[SP] = (ushort)(_reg[SP] + n); } break; // RET near imm16
                case 0xC3: _ip = Pop16(); break;                                                          // RET near
                case 0xC4: ReadModRM(); LoadFarPointer(ES); break;   // LES
                case 0xC5: ReadModRM(); LoadFarPointer(DS); break;   // LDS
                case 0xC6: ReadModRM(); WriteRM8(Fetch8()); break;   // MOV rm8, imm8
                case 0xC7: ReadModRM(); WriteRM16(Fetch16()); break; // MOV rm16, imm16
                case 0xC8: Enter(); break;                           // ENTER
                case 0xC9: Leave(); break;                           // LEAVE
                case 0xCA: { int n = Fetch16(); _ip = Pop16(); _seg[CS] = Pop16(); _reg[SP] = (ushort)(_reg[SP] + n); } break; // RET far imm16
                case 0xCB: { _ip = Pop16(); _seg[CS] = Pop16(); } break;  // RET far
                case 0xCC: ServiceInterrupt(3); break;               // INT3
                case 0xCD:                                           // INT imm8
                    {
                        int v = Fetch8();
                        if (OnSoftwareInterrupt != null)
                            OnSoftwareInterrupt(v, ((_seg[CS] << 4) + ((_ip - 2) & 0xFFFF)) & 0xFFFFF);
                        ServiceInterrupt(v);
                    }
                    break;
                case 0xCE: if (GetFlag(OF)) ServiceInterrupt(4); break; // INTO
                case 0xCF: Iret(); break;                            // IRET

                case 0xD4: Aam(Fetch8()); break;  // AAM
                case 0xD5: Aad(Fetch8()); break;  // AAD
                case 0xD6: SetReg8(AL, (byte)(GetFlag(CF) ? 0xFF : 0x00)); break; // SALC (undoc, harmless)
                case 0xD7: // XLAT
                    SetReg8(AL, ReadSeg8(DataSeg(), (ushort)(_reg[BX] + GetReg8(AL))));
                    break;

                // ---- ESC (no 8087 present) ----
                case 0xD8: case 0xD9: case 0xDA: case 0xDB:
                case 0xDC: case 0xDD: case 0xDE: case 0xDF:
                    ReadModRM(); if (!_modrmIsReg) ReadRM8(); // consume operand, no effect
                    break;

                // ---- loops / JCXZ ----
                case 0xE0: { sbyte d = (sbyte)Fetch8(); _reg[CX]--; if (_reg[CX] != 0 && !GetFlag(ZF)) _ip = (ushort)(_ip + d); } break; // LOOPNE
                case 0xE1: { sbyte d = (sbyte)Fetch8(); _reg[CX]--; if (_reg[CX] != 0 && GetFlag(ZF)) _ip = (ushort)(_ip + d); } break;  // LOOPE
                case 0xE2: { sbyte d = (sbyte)Fetch8(); _reg[CX]--; if (_reg[CX] != 0) _ip = (ushort)(_ip + d); } break;                 // LOOP
                case 0xE3: { sbyte d = (sbyte)Fetch8(); if (_reg[CX] == 0) _ip = (ushort)(_ip + d); } break;                             // JCXZ

                // ---- IN/OUT ----
                case 0xE4: SetReg8(AL, In8(Fetch8())); break;                 // IN AL, imm8
                case 0xE5: _reg[AX] = In16(Fetch8()); break;                  // IN AX, imm8
                case 0xE6: Out8(Fetch8(), GetReg8(AL)); break;                // OUT imm8, AL
                case 0xE7: Out16(Fetch8(), _reg[AX]); break;                  // OUT imm8, AX
                case 0xEC: SetReg8(AL, In8(_reg[DX])); break;                 // IN AL, DX
                case 0xED: _reg[AX] = In16(_reg[DX]); break;                  // IN AX, DX
                case 0xEE: Out8(_reg[DX], GetReg8(AL)); break;                // OUT DX, AL
                case 0xEF: Out16(_reg[DX], _reg[AX]); break;                  // OUT DX, AX

                // ---- CALL / JMP ----
                case 0xE8: { short d = (short)Fetch16(); Push16(_ip); _ip = (ushort)(_ip + d); } break;   // CALL rel16
                case 0xE9: { short d = (short)Fetch16(); _ip = (ushort)(_ip + d); } break;                // JMP rel16
                case 0xEA: { ushort noff = Fetch16(); ushort nseg = Fetch16(); _ip = noff; _seg[CS] = nseg; } break; // JMP far
                case 0xEB: { sbyte d = (sbyte)Fetch8(); _ip = (ushort)(_ip + d); } break;                 // JMP rel8

                // ---- flag / misc ----
                case 0xF1: break;             // undocumented INT1 opcode; treat as NOP
                case 0xF4: _halted = true; break; // HLT
                case 0xF5: SetFlag(CF, !GetFlag(CF)); break; // CMC
                case 0xF6: Grp3(8); break;
                case 0xF7: Grp3(16); break;
                case 0xF8: SetFlag(CF, false); break; // CLC
                case 0xF9: SetFlag(CF, true); break;  // STC
                case 0xFA: SetFlag(IF, false); break; // CLI
                case 0xFB: SetFlag(IF, true); break;  // STI
                case 0xFC: SetFlag(DF, false); break; // CLD
                case 0xFD: SetFlag(DF, true); break;  // STD
                case 0xFE: Grp4(); break;
                case 0xFF: Grp5(); break;

                default:
                    // Unimplemented / invalid opcode.
                    ServiceInterrupt(6);
                    break;
            }
        }

        // -------- Arithmetic/logic block (0x00-0x3D) --------

        private void ArithGroup(byte op)
        {
            int operation = (op >> 3) & 7; // 0=ADD 1=OR 2=ADC 3=SBB 4=AND 5=SUB 6=XOR 7=CMP
            switch (op & 7)
            {
                case 0: ReadModRM(); { int r = Alu8(operation, ReadRM8(), GetReg8(_modrmReg)); if (operation != 7) WriteRM8(r); } break;
                case 1: ReadModRM(); { int r = Alu16(operation, ReadRM16(), GetReg16(_modrmReg)); if (operation != 7) WriteRM16(r); } break;
                case 2: ReadModRM(); { int r = Alu8(operation, GetReg8(_modrmReg), ReadRM8()); if (operation != 7) SetReg8(_modrmReg, (byte)r); } break;
                case 3: ReadModRM(); { int r = Alu16(operation, GetReg16(_modrmReg), ReadRM16()); if (operation != 7) SetReg16(_modrmReg, (ushort)r); } break;
                case 4: { int r = Alu8(operation, GetReg8(AL), Fetch8()); if (operation != 7) SetReg8(AL, (byte)r); } break;
                case 5: { int r = Alu16(operation, _reg[AX], Fetch16()); if (operation != 7) _reg[AX] = (ushort)r; } break;
            }
        }

        private int Alu8(int operation, int a, int b)
        {
            switch (operation)
            {
                case 0: return Add8(a, b, 0);
                case 1: { int r = (a | b) & 0xFF; SetLogicFlags8(r); return r; }
                case 2: return Add8(a, b, GetFlag(CF) ? 1 : 0);
                case 3: return Sub8(a, b, GetFlag(CF) ? 1 : 0);
                case 4: { int r = (a & b) & 0xFF; SetLogicFlags8(r); return r; }
                case 5: return Sub8(a, b, 0);
                case 6: { int r = (a ^ b) & 0xFF; SetLogicFlags8(r); return r; }
                default: return Sub8(a, b, 0); // CMP
            }
        }

        private int Alu16(int operation, int a, int b)
        {
            switch (operation)
            {
                case 0: return Add16(a, b, 0);
                case 1: { int r = (a | b) & 0xFFFF; SetLogicFlags16(r); return r; }
                case 2: return Add16(a, b, GetFlag(CF) ? 1 : 0);
                case 3: return Sub16(a, b, GetFlag(CF) ? 1 : 0);
                case 4: { int r = (a & b) & 0xFFFF; SetLogicFlags16(r); return r; }
                case 5: return Sub16(a, b, 0);
                case 6: { int r = (a ^ b) & 0xFFFF; SetLogicFlags16(r); return r; }
                default: return Sub16(a, b, 0); // CMP
            }
        }

        private void Grp1(int bits, bool signExtendImm)
        {
            ReadModRM();
            int operation = _modrmReg; // reg field selects op
            if (bits == 8)
            {
                int a = ReadRM8();
                int b = Fetch8();
                int r = Alu8(operation, a, b);
                if (operation != 7) WriteRM8(r);
            }
            else
            {
                int a = ReadRM16();
                int b = signExtendImm ? (ushort)(sbyte)Fetch8() : Fetch16();
                int r = Alu16(operation, a, b);
                if (operation != 7) WriteRM16(r);
            }
        }

        // -------- Shift / rotate group --------

        private void Grp2(int bits, int count)
        {
            int op = _modrmReg;
            int val = (bits == 8) ? ReadRM8() : ReadRM16();
            int r = ShiftRotate(op, val, count, bits);
            if (bits == 8) WriteRM8(r); else WriteRM16(r);
        }

        private int ShiftRotate(int op, int val, int count, int bits)
        {
            int mask = (bits == 8) ? 0xFF : 0xFFFF;
            int signBit = (bits == 8) ? 0x80 : 0x8000;
            count &= 0x1F; // 80186 masks shift/rotate counts to 5 bits
            val &= mask;

            if (count == 0)
            {
                return val;
            }

            bool cf = GetFlag(CF);
            switch (op)
            {
                case 0: // ROL
                    for (int i = 0; i < count; i++)
                    {
                        cf = (val & signBit) != 0;
                        val = ((val << 1) | (cf ? 1 : 0)) & mask;
                    }
                    SetFlag(CF, cf);
                    SetFlag(OF, cf ^ ((val & signBit) != 0));
                    break;

                case 1: // ROR
                    for (int i = 0; i < count; i++)
                    {
                        cf = (val & 1) != 0;
                        val = (val >> 1) | (cf ? signBit : 0);
                    }
                    SetFlag(CF, cf);
                    SetFlag(OF, ((val & signBit) != 0) ^ ((val & (signBit >> 1)) != 0));
                    break;

                case 2: // RCL
                    for (int i = 0; i < count; i++)
                    {
                        bool newCf = (val & signBit) != 0;
                        val = ((val << 1) | (cf ? 1 : 0)) & mask;
                        cf = newCf;
                    }
                    SetFlag(CF, cf);
                    SetFlag(OF, cf ^ ((val & signBit) != 0));
                    break;

                case 3: // RCR
                    for (int i = 0; i < count; i++)
                    {
                        bool newCf = (val & 1) != 0;
                        val = (val >> 1) | (cf ? signBit : 0);
                        cf = newCf;
                    }
                    SetFlag(CF, cf);
                    SetFlag(OF, ((val & signBit) != 0) ^ ((val & (signBit >> 1)) != 0));
                    break;

                case 4: // SHL / SAL
                case 6:
                    for (int i = 0; i < count; i++)
                    {
                        cf = (val & signBit) != 0;
                        val = (val << 1) & mask;
                    }
                    SetFlag(CF, cf);
                    SetFlag(OF, cf ^ ((val & signBit) != 0));
                    SetSZP(val, bits);
                    break;

                case 5: // SHR
                    {
                        bool of = (val & signBit) != 0;
                        for (int i = 0; i < count; i++)
                        {
                            cf = (val & 1) != 0;
                            val = val >> 1;
                        }
                        SetFlag(CF, cf);
                        SetFlag(OF, of);
                        SetSZP(val, bits);
                    }
                    break;

                case 7: // SAR
                    for (int i = 0; i < count; i++)
                    {
                        cf = (val & 1) != 0;
                        val = (val >> 1) | (val & signBit);
                    }
                    SetFlag(CF, cf);
                    SetFlag(OF, false);
                    SetSZP(val, bits);
                    break;
            }

            return val & mask;
        }

        // -------- grp3 (TEST/NOT/NEG/MUL/IMUL/DIV/IDIV) --------

        private void Grp3(int bits)
        {
            ReadModRM();
            switch (_modrmReg)
            {
                case 0:
                case 1: // TEST rm, imm
                    if (bits == 8) TestFlags8(ReadRM8() & Fetch8());
                    else TestFlags16(ReadRM16() & Fetch16());
                    break;

                case 2: // NOT (no flags)
                    if (bits == 8) WriteRM8((~ReadRM8()) & 0xFF);
                    else WriteRM16((~ReadRM16()) & 0xFFFF);
                    break;

                case 3: // NEG
                    if (bits == 8) WriteRM8(Sub8(0, ReadRM8(), 0));
                    else WriteRM16(Sub16(0, ReadRM16(), 0));
                    break;

                case 4: // MUL
                    if (bits == 8)
                    {
                        int r = GetReg8(AL) * ReadRM8();
                        _reg[AX] = (ushort)r;
                        bool hi = (r & 0xFF00) != 0;
                        SetFlag(CF, hi); SetFlag(OF, hi);
                    }
                    else
                    {
                        int r = _reg[AX] * ReadRM16();
                        _reg[AX] = (ushort)r; _reg[DX] = (ushort)(r >> 16);
                        bool hi = _reg[DX] != 0;
                        SetFlag(CF, hi); SetFlag(OF, hi);
                    }
                    break;

                case 5: // IMUL
                    if (bits == 8)
                    {
                        int r = (sbyte)GetReg8(AL) * (sbyte)ReadRM8();
                        _reg[AX] = (ushort)r;
                        bool ext = (short)r != (sbyte)r;
                        SetFlag(CF, ext); SetFlag(OF, ext);
                    }
                    else
                    {
                        int r = (short)_reg[AX] * (short)(ushort)ReadRM16();
                        _reg[AX] = (ushort)r; _reg[DX] = (ushort)(r >> 16);
                        bool ext = r != (short)r;
                        SetFlag(CF, ext); SetFlag(OF, ext);
                    }
                    break;

                case 6: // DIV
                    if (bits == 8)
                    {
                        int d = ReadRM8();
                        if (d == 0) { ServiceInterrupt(0); break; }
                        int dividend = _reg[AX];
                        int q = dividend / d, r = dividend % d;
                        if (q > 0xFF) { ServiceInterrupt(0); break; }
                        SetReg8(AL, (byte)q); SetReg8(AH, (byte)r);
                    }
                    else
                    {
                        int d = ReadRM16();
                        if (d == 0) { ServiceInterrupt(0); break; }
                        uint dividend = (uint)((_reg[DX] << 16) | _reg[AX]);
                        uint q = dividend / (uint)d, r = dividend % (uint)d;
                        if (q > 0xFFFF) { ServiceInterrupt(0); break; }
                        _reg[AX] = (ushort)q; _reg[DX] = (ushort)r;
                    }
                    break;

                case 7: // IDIV
                    if (bits == 8)
                    {
                        int d = (sbyte)ReadRM8();
                        if (d == 0) { ServiceInterrupt(0); break; }
                        int dividend = (short)_reg[AX];
                        int q = dividend / d, r = dividend % d;
                        if (q > 127 || q < -128) { ServiceInterrupt(0); break; }
                        SetReg8(AL, (byte)q); SetReg8(AH, (byte)r);
                    }
                    else
                    {
                        int d = (short)(ushort)ReadRM16();
                        if (d == 0) { ServiceInterrupt(0); break; }
                        int dividend = (_reg[DX] << 16) | _reg[AX];
                        int q = dividend / d, r = dividend % d;
                        if (q > 32767 || q < -32768) { ServiceInterrupt(0); break; }
                        _reg[AX] = (ushort)q; _reg[DX] = (ushort)r;
                    }
                    break;
            }
        }

        // -------- grp4 (INC/DEC rm8) / grp5 (INC/DEC/CALL/JMP/PUSH rm16) --------

        private void Grp4()
        {
            ReadModRM();
            switch (_modrmReg)
            {
                case 0: WriteRM8(Inc8(ReadRM8())); break;
                case 1: WriteRM8(Dec8(ReadRM8())); break;
                default: ServiceInterrupt(6); break;
            }
        }

        private void Grp5()
        {
            ReadModRM();
            switch (_modrmReg)
            {
                case 0: WriteRM16(Inc16(ReadRM16())); break;
                case 1: WriteRM16(Dec16(ReadRM16())); break;
                case 2: // CALL near rm16
                    { ushort t = (ushort)ReadRM16(); Push16(_ip); _ip = t; }
                    break;
                case 3: // CALL far [mem]
                    {
                        ushort noff = ReadSeg16(_modrmSeg, _modrmOff);
                        ushort nseg = ReadSeg16(_modrmSeg, _modrmOff + 2);
                        Push16(_seg[CS]); Push16(_ip);
                        _seg[CS] = nseg; _ip = noff;
                    }
                    break;
                case 4: _ip = (ushort)ReadRM16(); break; // JMP near rm16
                case 5: // JMP far [mem]
                    {
                        ushort noff = ReadSeg16(_modrmSeg, _modrmOff);
                        ushort nseg = ReadSeg16(_modrmSeg, _modrmOff + 2);
                        _ip = noff; _seg[CS] = nseg;
                    }
                    break;
                case 6: Push16((ushort)ReadRM16()); break; // PUSH rm16
                default: ServiceInterrupt(6); break;
            }
        }

        // -------- 80186-specific helpers --------

        private void Pusha()
        {
            ushort sp = _reg[SP];
            Push16(_reg[AX]); Push16(_reg[CX]); Push16(_reg[DX]); Push16(_reg[BX]);
            Push16(sp); Push16(_reg[BP]); Push16(_reg[SI]); Push16(_reg[DI]);
        }

        private void Popa()
        {
            _reg[DI] = Pop16(); _reg[SI] = Pop16(); _reg[BP] = Pop16();
            Pop16(); // discarded SP
            _reg[BX] = Pop16(); _reg[DX] = Pop16(); _reg[CX] = Pop16(); _reg[AX] = Pop16();
        }

        private void Bound()
        {
            ReadModRM();
            if (_modrmIsReg) { ServiceInterrupt(6); return; }
            int index = (short)GetReg16(_modrmReg);
            int lower = (short)ReadSeg16(_modrmSeg, _modrmOff);
            int upper = (short)ReadSeg16(_modrmSeg, _modrmOff + 2);
            if (index < lower || index > upper) ServiceInterrupt(5);
        }

        private void ImulImm(bool imm16)
        {
            ReadModRM();
            int src = (short)(ushort)ReadRM16();
            int imm = imm16 ? (short)Fetch16() : (sbyte)Fetch8();
            int r = src * imm;
            SetReg16(_modrmReg, (ushort)r);
            bool ext = r != (short)r;
            SetFlag(CF, ext); SetFlag(OF, ext);
        }

        private void Enter()
        {
            int allocSize = Fetch16();
            int nestingLevel = Fetch8() & 0x1F;
            Push16(_reg[BP]);
            ushort frameTemp = _reg[SP];
            if (nestingLevel > 0)
            {
                for (int i = 1; i < nestingLevel; i++)
                {
                    _reg[BP] = (ushort)(_reg[BP] - 2);
                    Push16(ReadSeg16(SS, _reg[BP]));
                }
                Push16(frameTemp);
            }
            _reg[BP] = frameTemp;
            _reg[SP] = (ushort)(_reg[SP] - allocSize);
        }

        private void Leave()
        {
            _reg[SP] = _reg[BP];
            _reg[BP] = Pop16();
        }

        // -------- String operations --------

        private void StringOp(int rep, int op)
        {
            bool word = (op & 1) != 0 && op != 0x6E;   // handled per-op below
            // Determine element size explicitly per opcode.
            switch (op)
            {
                case 0x6C: case 0x6E: case 0xA4: case 0xA6: case 0xAA: case 0xAC: case 0xAE:
                    word = false; break;
                default:
                    word = true; break;
            }

            int srcSeg = DataSeg();
            int delta = word ? 2 : 1;

            if (rep == 0)
            {
                StringStep(op, word, srcSeg, delta);
                return;
            }

            // REP-prefixed.  MOVS/STOS/LODS/INS/OUTS iterate on CX only; CMPS/SCAS
            // additionally test ZF against the REP flavour.
            bool comparison = (op == 0xA6 || op == 0xA7 || op == 0xAE || op == 0xAF);
            while (_reg[CX] != 0)
            {
                StringStep(op, word, srcSeg, delta);
                _reg[CX]--;
                if (comparison)
                {
                    bool zf = GetFlag(ZF);
                    if (rep == 1 && !zf) break;   // REPE: stop when ZF=0
                    if (rep == 2 && zf) break;    // REPNE: stop when ZF=1
                }
            }
        }

        private void StringStep(int op, bool word, int srcSeg, int delta)
        {
            int dir = GetFlag(DF) ? -delta : delta;
            switch (op)
            {
                case 0xA4: case 0xA5: // MOVS
                    if (word) WriteSeg16(ES, _reg[DI], ReadSeg16(srcSeg, _reg[SI]));
                    else WriteSeg8(ES, _reg[DI], ReadSeg8(srcSeg, _reg[SI]));
                    _reg[SI] = (ushort)(_reg[SI] + dir);
                    _reg[DI] = (ushort)(_reg[DI] + dir);
                    break;

                case 0xAA: case 0xAB: // STOS
                    if (word) WriteSeg16(ES, _reg[DI], _reg[AX]);
                    else WriteSeg8(ES, _reg[DI], GetReg8(AL));
                    _reg[DI] = (ushort)(_reg[DI] + dir);
                    break;

                case 0xAC: case 0xAD: // LODS
                    if (word) _reg[AX] = ReadSeg16(srcSeg, _reg[SI]);
                    else SetReg8(AL, ReadSeg8(srcSeg, _reg[SI]));
                    _reg[SI] = (ushort)(_reg[SI] + dir);
                    break;

                case 0xA6: case 0xA7: // CMPS  (src - dst)
                    if (word) Sub16(ReadSeg16(srcSeg, _reg[SI]), ReadSeg16(ES, _reg[DI]), 0);
                    else Sub8(ReadSeg8(srcSeg, _reg[SI]), ReadSeg8(ES, _reg[DI]), 0);
                    _reg[SI] = (ushort)(_reg[SI] + dir);
                    _reg[DI] = (ushort)(_reg[DI] + dir);
                    break;

                case 0xAE: case 0xAF: // SCAS  (AL/AX - dst)
                    if (word) Sub16(_reg[AX], ReadSeg16(ES, _reg[DI]), 0);
                    else Sub8(GetReg8(AL), ReadSeg8(ES, _reg[DI]), 0);
                    _reg[DI] = (ushort)(_reg[DI] + dir);
                    break;

                case 0x6C: case 0x6D: // INS
                    if (word) WriteSeg16(ES, _reg[DI], In16(_reg[DX]));
                    else WriteSeg8(ES, _reg[DI], In8(_reg[DX]));
                    _reg[DI] = (ushort)(_reg[DI] + dir);
                    break;

                case 0x6E: case 0x6F: // OUTS
                    if (word) Out16(_reg[DX], ReadSeg16(srcSeg, _reg[SI]));
                    else Out8(_reg[DX], ReadSeg8(srcSeg, _reg[SI]));
                    _reg[SI] = (ushort)(_reg[SI] + dir);
                    break;
            }
        }

        // -------- Jcc / conditionals --------

        private void Jcc(int cond)
        {
            sbyte disp = (sbyte)Fetch8();
            if (TestCondition(cond)) _ip = (ushort)(_ip + disp);
        }

        private bool TestCondition(int cond)
        {
            switch (cond)
            {
                case 0x0: return GetFlag(OF);                              // JO
                case 0x1: return !GetFlag(OF);                             // JNO
                case 0x2: return GetFlag(CF);                              // JB/JC
                case 0x3: return !GetFlag(CF);                             // JAE/JNC
                case 0x4: return GetFlag(ZF);                              // JE/JZ
                case 0x5: return !GetFlag(ZF);                             // JNE/JNZ
                case 0x6: return GetFlag(CF) || GetFlag(ZF);               // JBE
                case 0x7: return !(GetFlag(CF) || GetFlag(ZF));            // JA
                case 0x8: return GetFlag(SF);                              // JS
                case 0x9: return !GetFlag(SF);                             // JNS
                case 0xA: return GetFlag(PF);                              // JP/JPE
                case 0xB: return !GetFlag(PF);                             // JNP/JPO
                case 0xC: return GetFlag(SF) != GetFlag(OF);               // JL
                case 0xD: return GetFlag(SF) == GetFlag(OF);               // JGE
                case 0xE: return GetFlag(ZF) || (GetFlag(SF) != GetFlag(OF)); // JLE
                default:  return !GetFlag(ZF) && (GetFlag(SF) == GetFlag(OF)); // JG
            }
        }

        // -------- Interrupts --------

        private void ServiceInterrupt(int vector)
        {
            Push16((ushort)(_flags | 0xF002));
            SetFlag(IF, false);
            SetFlag(TF, false);
            Push16(_seg[CS]);
            Push16(_ip);
            int addr = (vector & 0xFF) * 4;
            _ip = ReadPhys16(addr);
            _seg[CS] = ReadPhys16(addr + 2);
        }

        private void Iret()
        {
            _ip = Pop16();
            _seg[CS] = Pop16();
            _flags = Pop16() | 0x0002;
        }

        private void LoadFarPointer(int segReg)
        {
            SetReg16(_modrmReg, ReadSeg16(_modrmSeg, _modrmOff));
            _seg[segReg] = ReadSeg16(_modrmSeg, _modrmOff + 2);
        }

        // -------- Decimal / ASCII adjust --------

        private void Daa()
        {
            int al = GetReg8(AL);
            bool cf = GetFlag(CF);
            int oldAl = al;
            if ((al & 0x0F) > 9 || GetFlag(AF)) { al += 6; SetFlag(AF, true); } else SetFlag(AF, false);
            if (oldAl > 0x99 || cf) { al += 0x60; SetFlag(CF, true); } else SetFlag(CF, false);
            SetReg8(AL, (byte)al);
            SetSZP(al & 0xFF, 8);
        }

        private void Das()
        {
            int al = GetReg8(AL);
            bool cf = GetFlag(CF);
            int oldAl = al;
            if ((al & 0x0F) > 9 || GetFlag(AF)) { al -= 6; SetFlag(AF, true); } else SetFlag(AF, false);
            if (oldAl > 0x99 || cf) { al -= 0x60; SetFlag(CF, true); } else SetFlag(CF, false);
            SetReg8(AL, (byte)al);
            SetSZP(al & 0xFF, 8);
        }

        private void Aaa()
        {
            if ((GetReg8(AL) & 0x0F) > 9 || GetFlag(AF))
            {
                SetReg8(AL, (byte)(GetReg8(AL) + 6));
                SetReg8(AH, (byte)(GetReg8(AH) + 1));
                SetFlag(AF, true); SetFlag(CF, true);
            }
            else { SetFlag(AF, false); SetFlag(CF, false); }
            SetReg8(AL, (byte)(GetReg8(AL) & 0x0F));
        }

        private void Aas()
        {
            if ((GetReg8(AL) & 0x0F) > 9 || GetFlag(AF))
            {
                SetReg8(AL, (byte)(GetReg8(AL) - 6));
                SetReg8(AH, (byte)(GetReg8(AH) - 1));
                SetFlag(AF, true); SetFlag(CF, true);
            }
            else { SetFlag(AF, false); SetFlag(CF, false); }
            SetReg8(AL, (byte)(GetReg8(AL) & 0x0F));
        }

        private void Aam(int baseVal)
        {
            if (baseVal == 0) { ServiceInterrupt(0); return; }
            int al = GetReg8(AL);
            SetReg8(AH, (byte)(al / baseVal));
            SetReg8(AL, (byte)(al % baseVal));
            SetSZP(GetReg8(AL), 8);
        }

        private void Aad(int baseVal)
        {
            int al = GetReg8(AL) + GetReg8(AH) * baseVal;
            SetReg8(AL, (byte)al);
            SetReg8(AH, 0);
            SetSZP(GetReg8(AL), 8);
        }

        // -------- ModR/M decode --------

        private void ReadModRM()
        {
            byte modrm = Fetch8();
            int mod = (modrm >> 6) & 3;
            _modrmReg = (modrm >> 3) & 7;
            int rm = modrm & 7;

            if (mod == 3)
            {
                _modrmIsReg = true;
                _modrmRm = rm;
                return;
            }

            _modrmIsReg = false;
            int off;
            int seg;

            switch (rm)
            {
                case 0: off = _reg[BX] + _reg[SI]; seg = DS; break;
                case 1: off = _reg[BX] + _reg[DI]; seg = DS; break;
                case 2: off = _reg[BP] + _reg[SI]; seg = SS; break;
                case 3: off = _reg[BP] + _reg[DI]; seg = SS; break;
                case 4: off = _reg[SI]; seg = DS; break;
                case 5: off = _reg[DI]; seg = DS; break;
                case 6:
                    if (mod == 0) { off = Fetch16(); seg = DS; }
                    else { off = _reg[BP]; seg = SS; }
                    break;
                default: off = _reg[BX]; seg = DS; break;
            }

            if (mod == 1) off += (sbyte)Fetch8();
            else if (mod == 2) off += (short)Fetch16();

            _modrmOff = off & 0xFFFF;
            _modrmSeg = (_segOverride >= 0) ? _segOverride : seg;
        }

        private int ReadRM8()
        {
            return _modrmIsReg ? GetReg8(_modrmRm) : ReadSeg8(_modrmSeg, _modrmOff);
        }

        private int ReadRM16()
        {
            return _modrmIsReg ? GetReg16(_modrmRm) : ReadSeg16(_modrmSeg, _modrmOff);
        }

        private void WriteRM8(int value)
        {
            if (_modrmIsReg) SetReg8(_modrmRm, (byte)value);
            else WriteSeg8(_modrmSeg, _modrmOff, (byte)value);
        }

        private void WriteRM16(int value)
        {
            if (_modrmIsReg) SetReg16(_modrmRm, (ushort)value);
            else WriteSeg16(_modrmSeg, _modrmOff, (ushort)value);
        }

        // -------- Register access --------

        private int GetReg8(int i)
        {
            return (i < 4) ? (_reg[i] & 0xFF) : ((_reg[i - 4] >> 8) & 0xFF);
        }

        private void SetReg8(int i, int v)
        {
            v &= 0xFF;
            if (i < 4) _reg[i] = (ushort)((_reg[i] & 0xFF00) | v);
            else _reg[i - 4] = (ushort)((_reg[i - 4] & 0x00FF) | (v << 8));
        }

        private int GetReg16(int i) { return _reg[i]; }
        private void SetReg16(int i, int v) { _reg[i] = (ushort)v; }

        // Default data segment (DS unless overridden).
        private int DataSeg() { return (_segOverride >= 0) ? _segOverride : DS; }

        // -------- Stack --------

        private void Push16(int v)
        {
            _reg[SP] = (ushort)(_reg[SP] - 2);
            WriteSeg16(SS, _reg[SP], v);
        }

        private ushort Pop16()
        {
            ushort v = ReadSeg16(SS, _reg[SP]);
            _reg[SP] = (ushort)(_reg[SP] + 2);
            return v;
        }

        // -------- Instruction fetch --------

        private byte Fetch8()
        {
            byte b = ReadSeg8(CS, _ip);
            _ip = (ushort)(_ip + 1);
            _cycles++;
            return b;
        }

        private ushort Fetch16()
        {
            return (ushort)(Fetch8() | (Fetch8() << 8));
        }

        // -------- Segment-relative memory --------

        private byte ReadSeg8(int seg, int off)
        {
            return MemReadByte(((_seg[seg] << 4) + (off & 0xFFFF)) & 0xFFFFF);
        }

        private ushort ReadSeg16(int seg, int off)
        {
            off &= 0xFFFF;
            int lo = ReadSeg8(seg, off);
            int hi = ReadSeg8(seg, (off + 1) & 0xFFFF);
            return (ushort)(lo | (hi << 8));
        }

        private void WriteSeg8(int seg, int off, int value)
        {
            MemWriteByte(((_seg[seg] << 4) + (off & 0xFFFF)) & 0xFFFFF, (byte)value);
        }

        private void WriteSeg16(int seg, int off, int value)
        {
            off &= 0xFFFF;
            WriteSeg8(seg, off, value & 0xFF);
            WriteSeg8(seg, (off + 1) & 0xFFFF, (value >> 8) & 0xFF);
        }

        private ushort ReadPhys16(int phys)
        {
            int lo = MemReadByte(phys & 0xFFFFF);
            int hi = MemReadByte((phys + 1) & 0xFFFFF);
            return (ushort)(lo | (hi << 8));
        }

        // -------- Physical bus (with PCB interception) --------

        private byte MemReadByte(int phys)
        {
            int off;
            if (_pcb.TryMapMemory(phys, out off)) return _pcb.ReadByte(off);
            return _mem.ReadByte(phys);
        }

        private void MemWriteByte(int phys, byte value)
        {
            int off;
            if (_pcb.TryMapMemory(phys, out off)) { _pcb.WriteByte(off, value); return; }
            _mem.WriteByte(phys, value);
        }

        private byte In8(int port)
        {
            int off;
            if (_pcb.TryMapIO(port, out off)) return _pcb.ReadByte(off);
            return _io.ReadByte((ushort)port);
        }

        private ushort In16(int port)
        {
            int off;
            if (_pcb.TryMapIO(port, out off)) return _pcb.ReadWord(off);
            return _io.ReadWord((ushort)port);
        }

        private void Out8(int port, int value)
        {
            int off;
            if (_pcb.TryMapIO(port, out off)) { _pcb.WriteByte(off, (byte)value); return; }
            _io.WriteByte((ushort)port, (byte)value);
        }

        private void Out16(int port, ushort value)
        {
            int off;
            if (_pcb.TryMapIO(port, out off)) { _pcb.WriteWord(off, value); return; }
            _io.WriteWord((ushort)port, value);
        }

        // -------- Flags / ALU primitives --------

        private bool GetFlag(int mask) { return (_flags & mask) != 0; }
        private void SetFlag(int mask, bool v) { if (v) _flags |= mask; else _flags &= ~mask; }

        private void SetSZP(int r, int bits)
        {
            if (bits == 8)
            {
                r &= 0xFF;
                SetFlag(ZF, r == 0);
                SetFlag(SF, (r & 0x80) != 0);
            }
            else
            {
                r &= 0xFFFF;
                SetFlag(ZF, r == 0);
                SetFlag(SF, (r & 0x8000) != 0);
            }
            SetFlag(PF, _parity[r & 0xFF]);
        }

        private int Add8(int a, int b, int c)
        {
            a &= 0xFF; b &= 0xFF;
            int r = a + b + c;
            int rr = r & 0xFF;
            SetFlag(CF, (r & 0x100) != 0);
            SetFlag(AF, ((a ^ b ^ rr) & 0x10) != 0);
            SetFlag(OF, ((a ^ rr) & (b ^ rr) & 0x80) != 0);
            SetSZP(rr, 8);
            return rr;
        }

        private int Add16(int a, int b, int c)
        {
            a &= 0xFFFF; b &= 0xFFFF;
            int r = a + b + c;
            int rr = r & 0xFFFF;
            SetFlag(CF, (r & 0x10000) != 0);
            SetFlag(AF, ((a ^ b ^ rr) & 0x10) != 0);
            SetFlag(OF, ((a ^ rr) & (b ^ rr) & 0x8000) != 0);
            SetSZP(rr, 16);
            return rr;
        }

        private int Sub8(int a, int b, int c)
        {
            a &= 0xFF; b &= 0xFF;
            int r = a - b - c;
            int rr = r & 0xFF;
            SetFlag(CF, a < b + c);
            SetFlag(AF, ((a ^ b ^ rr) & 0x10) != 0);
            SetFlag(OF, ((a ^ b) & (a ^ rr) & 0x80) != 0);
            SetSZP(rr, 8);
            return rr;
        }

        private int Sub16(int a, int b, int c)
        {
            a &= 0xFFFF; b &= 0xFFFF;
            int r = a - b - c;
            int rr = r & 0xFFFF;
            SetFlag(CF, a < b + c);
            SetFlag(AF, ((a ^ b ^ rr) & 0x10) != 0);
            SetFlag(OF, ((a ^ b) & (a ^ rr) & 0x8000) != 0);
            SetSZP(rr, 16);
            return rr;
        }

        private int Inc8(int a)
        {
            bool cf = GetFlag(CF);
            int r = Add8(a, 1, 0);
            SetFlag(CF, cf); // INC preserves CF
            return r;
        }

        private int Inc16(int a)
        {
            bool cf = GetFlag(CF);
            int r = Add16(a, 1, 0);
            SetFlag(CF, cf);
            return r;
        }

        private int Dec8(int a)
        {
            bool cf = GetFlag(CF);
            int r = Sub8(a, 1, 0);
            SetFlag(CF, cf);
            return r;
        }

        private int Dec16(int a)
        {
            bool cf = GetFlag(CF);
            int r = Sub16(a, 1, 0);
            SetFlag(CF, cf);
            return r;
        }

        private void SetLogicFlags8(int r)
        {
            SetFlag(CF, false); SetFlag(OF, false); SetFlag(AF, false);
            SetSZP(r & 0xFF, 8);
        }

        private void SetLogicFlags16(int r)
        {
            SetFlag(CF, false); SetFlag(OF, false); SetFlag(AF, false);
            SetSZP(r & 0xFFFF, 16);
        }

        private void TestFlags8(int r) { SetLogicFlags8(r & 0xFF); }
        private void TestFlags16(int r) { SetLogicFlags16(r & 0xFFFF); }

        private void BuildParityTable()
        {
            for (int i = 0; i < 256; i++)
            {
                int bits = 0;
                for (int b = 0; b < 8; b++) if ((i & (1 << b)) != 0) bits++;
                _parity[i] = (bits & 1) == 0;
            }
        }

        // -------- State --------

        private readonly ushort[] _reg = new ushort[8];
        private readonly ushort[] _seg = new ushort[4];
        private ushort _ip;
        private int _flags;
        private bool _halted;
        private bool _nmiPending;
        private long _hwInterrupts;
        private int _lastHwVector = -1;

        private int _segOverride;
        private int _cycles;

        // ModR/M decode results.
        private int _modrmReg;
        private int _modrmRm;
        private bool _modrmIsReg;
        private int _modrmSeg;
        private int _modrmOff;

        private readonly bool[] _parity = new bool[256];

        private readonly IPhysicalMemory _mem;
        private readonly IIOBus186 _io;
        private readonly I80186Pcb _pcb;
    }
}
