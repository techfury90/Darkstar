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
using System.Collections.Generic;
using D.IO;

namespace D.IOP
{
    /// <summary>
    /// The Dove IOP's 16-bit I/O bus (IOP-TR §2.4).  Decodes the PCS chip-select
    /// blocks and the system-board I/O registers (map registers, display type).
    /// This is the first-bring-up scaffold: the control surface (PCS1) registers
    /// that the boot ROM needs (LED, reset/control/config regs, host-address PROM,
    /// arbiter strobes) are modelled enough for POST to progress, and the peripheral
    /// controllers (8259/8254/8251/8274/8272/RDC) are permissive stubs.  Every
    /// access is logged so a boot-ROM trace shows exactly which port fidelity is
    /// required next.
    /// </summary>
    public class DoveIOPIO : IIOBus186
    {
        public DoveIOPIO(DoveIOPMemory memory)
        {
            _memory = memory;
            _picMaster = new I8259("master");
            _picSlave = new I8259("slave");
            _picOptions = new I8259("options");
            _pit = new I8254();
            _keyboardUart = new I8251();
            _fdc = new I8272();
            _controlStore = new DoveControlStore();
            _display = new DoveDisplayController(memory);
            // Scan the bitmap out of system DRAM (where the firmware writes it via
            // the IOP map window), not the 80186 address space.
            DoveIOPMemory dmem = memory as DoveIOPMemory;
            if (dmem != null) _display.DisplayReader = dmem.ReadDisplayWord;
            _configEeprom = new I93C46();

            // Rigid disk: behavioral 8X305 controller + AM2942 DMA over a Micropolis-1325 pack
            // (the drive this machine's EEPROM declares).  The backing image is attached by the
            // harness/UI via the Disk accessor; unattached = a blank pack (all sectors unformatted).
            _disk = new Micropolis1325();
            _rdc = new DoveDiskController(_memory, _disk);

            // Per-DOB trace.  0935 on a disk operation means a non-goodCompletion status
            // reached Pilot (ioError -> Space.IOError -> uncaught), so the failing op and its
            // error bytes are the whole question.
            string rdcLogPath = Environment.GetEnvironmentVariable("DOVE_RDC_LOG");
            if (!string.IsNullOrEmpty(rdcLogPath)) _rdc.LogWriter = OpenTrace(rdcLogPath);

            // Floppy Read Data → 80186 DMA channel 0 → main memory.  DMA0 destination
            // is FFC4 (low 16) + FFC6 (upper 4 bits); FFC8 = transfer count.
            _fdc.DmaOut = data =>
            {
                if (_pcb == null) return 0;
                int lo = _pcb.GetRegisterWord(0xC4);
                int hi = _pcb.GetRegisterWord(0xC6);
                int count = _pcb.GetRegisterWord(0xC8);
                int dest = ((hi & 0x000F) << 16) | lo;
                int n = data.Length;
                if (count > 0 && count < n) n = count;         // honor DMA count (bytes)
                for (int i = 0; i < n; i++) _memory.WriteByte(dest + i, data[i]);
                // Advance the DMA0 destination pointer and drain the transfer count,
                // as the real DMA controller does (drivers poll FFC8==0 for completion).
                int nd = dest + n;
                _pcb.SetRegisterWord(0xC4, (ushort)(nd & 0xFFFF));
                _pcb.SetRegisterWord(0xC6, (ushort)((nd >> 16) & 0x000F));
                _pcb.SetRegisterWord(0xC8, (ushort)(count > n ? count - n : 0));
                // The FDC's DACK' feeds 80186 timer 1 (one count per byte); reaching
                // its max-count A wraps the count to 0 -- the terminal count the driver
                // polls (IN FF58) as one of its completion conditions.  (Timer 1's
                // interrupt-enable bit is clear, so no TC interrupt is raised.)
                _pcb.PulseTimer1(n);
                // The transfer terminated on the DMA count (FFC8 fully drained: count<=n).  The real
                // 80186 DMA TC ends the burst here regardless of timer-1's max-count A, so force
                // timer 1 to its terminal count (0).  This makes the head read
                // firstTrack.TotalBytesActuallyTransfered = 0 (residual, like FinalDMACount);
                // otherwise a burst that stops before reaching max-A leaves timer 1 = bytes-
                // transferred (e.g. 512) and UpdateOperation:1567 fails the read forever (the C5/H0/R6
                // 921 boot hang).  Only on a genuine short/under-run (count>n) do we leave the residual.
                if (count > 0 && count <= n) _pcb.ClearTimer1Count();
                // DIAGNOSTIC: ring of the last DMA transfers -- count-programmed vs bytes-delivered.
                // The driver polls FFC8==0 for completion (no TC interrupt), so an UNDER-DELIVERED read
                // (bytes < count, e.g. an unhandled multi-track read) leaves FFC8 non-zero and the driver
                // polls forever.  Capture count-before, delivered, count-after so an under-run is visible.
                if (DmaLog != null)
                {
                    if (DmaLog.Count >= 40) DmaLog.RemoveAt(0);
                    DmaLog.Add("DMA #" + _fdc.ReadCount + " countBefore=" + count + " delivered=" + n
                        + " countAfter=" + (count > n ? count - n : 0) + " dest=0x" + dest.ToString("X5")
                        + (count > n ? "   *** UNDER-RUN: FFC8 stuck at " + (count - n) + " (driver polls forever) ***" : ""));
                }
                // First-60 DMA log WITH the sector, to compare R5(normal,completes) vs R6(compressed,fails)
                // vs R9(normal,fails): does the germ program a different DMA count-before for the failing reads?
                if (DmaFirst != null && DmaFirst.Count < 60 && _fdc.LastReadC == 5 && _fdc.LastReadH == 0
                    && (_fdc.LastReadR == 5 || _fdc.LastReadR == 6 || _fdc.LastReadR == 9))
                {
                    bool allSame = data.Length > 0; for (int i = 1; i < n && i < data.Length; i++) if (data[i] != data[0]) { allSame = false; break; }
                    DmaFirst.Add("C" + _fdc.LastReadC + "/H" + _fdc.LastReadH + "/R" + _fdc.LastReadR + " cntBefore=" + count
                        + " gathered=" + data.Length + " deliv=" + n + " cntAfter=" + (count > n ? count - n : 0)
                        + " firstSecAllSame=" + allSame + "(0x" + (data.Length > 0 ? data[0] : (byte)0).ToString("X2") + ") dest=0x" + dest.ToString("X5")
                        + " | POST-regs ch0cnt(FFC8)=" + _pcb.GetRegisterWord(0xC8) + " ch1cnt(FFD8)=" + _pcb.GetRegisterWord(0xD8)
                        + " t1cnt(FF58)=" + _pcb.GetRegisterWord(0x58) + " t1maxA(FF5A)=" + _pcb.GetRegisterWord(0x5A));
                }
                _lastDmaDest = dest; _lastDmaLen = n;
                if (MinDmaDest < 0 || dest < MinDmaDest) MinDmaDest = dest;
                if (dest + n > MaxDmaDest) MaxDmaDest = dest + n;
                DmaByteTotal += n;
                return n;
            };
            InitDefaultHostProm();

            // The 82586 is a bus master: it reads its command structures straight out of
            // memory and interrupts on slave IR1 when it has posted status.
            _enet = new I82586(memory, RaiseEnetInterrupt);
        }

        public DoveDisplayController Display { get { return _display; } }
        public I93C46 ConfigEeprom { get { return _configEeprom; } }

        /// <summary>Load the 93C46 config EEPROM (U128) image (128 bytes).</summary>
        /// <summary>
        /// Open a trace file that can be READ while the machine runs.  Without FileShare.Read
        /// the obvious thing to do with a trace -- open it and watch -- either fails or makes
        /// the writer fail, and a diagnostic must never be able to disturb the machine.
        /// </summary>
        public static System.IO.StreamWriter OpenTrace(string path)
        {
            try
            {
                // APPEND, not Create: a power cycle builds a fresh machine and reopens the
                // trace, so truncating here silently discards everything logged before the
                // reset -- and the interesting run (partitioning) always follows one.
                var fs = new System.IO.FileStream(path, System.IO.FileMode.Append,
                                                  System.IO.FileAccess.Write,
                                                  System.IO.FileShare.ReadWrite | System.IO.FileShare.Delete);
                return new System.IO.StreamWriter(fs) { AutoFlush = true };
            }
            catch { return null; }      // tracing is never worth failing the run for
        }

        public void LoadConfigEeprom(byte[] image) { _configEeprom.Load(image); }

        /// <summary>
        /// Load the 8-byte host-address (Ethernet MAC) PROM read at ports 0x90-0x9E:
        /// 6 MAC bytes + a rotating-XOR checksum byte + its complement.  Pass the
        /// first 8 bytes of a real 82S123 (U56) image, or any 8 bytes (the checksum
        /// is recomputed to keep the boot-ROM self-test happy).
        /// </summary>
        public void LoadHostProm(byte[] bytes)
        {
            for (int i = 0; i < 6 && i < bytes.Length; i++) _hostProm[i] = bytes[i];
            FinishHostProm();
        }

        private void InitDefaultHostProm()
        {
            // Default MAC using the Xerox OUI (00:00:AA); checksum computed below.
            _hostProm[0] = 0x00; _hostProm[1] = 0x00; _hostProm[2] = 0xAA;
            _hostProm[3] = 0x00; _hostProm[4] = 0x00; _hostProm[5] = 0x01;
            FinishHostProm();
        }

        private void FinishHostProm()
        {
            // Checksum algorithm from the boot ROM (ROMBoot @ FFAA0): checksum starts
            // at byte 0, then for bytes 1..5 checksum = rol((checksum ^ byte), 1).
            int cksum = _hostProm[0];
            for (int i = 1; i < 6; i++)
            {
                cksum ^= _hostProm[i];
                cksum = ((cksum << 1) | (cksum >> 7)) & 0xFF;
            }
            _hostProm[6] = (byte)cksum;
            _hostProm[7] = (byte)~cksum;
        }

        public I8251 KeyboardUart { get { return _keyboardUart; } }
        public I8272 Fdc { get { return _fdc; } }
        public DoveControlStore ControlStore { get { return _controlStore; } }

        /// <summary>WriteCSReg (0xB0) writes — CP run/halt control (0x0200 run bit).</summary>
        public System.Action<ushort> OnWriteCSReg;
        /// <summary>WriteResetReg bit 6 (resetMesaProcessor) changes — true = CP released.</summary>
        public System.Action<bool> OnMesaReset;
        /// <summary>IN 0xB0 (Clear Mesa Interrupt Latch) — IOP acks the CP→IOP doorbell.</summary>
        public System.Action OnReadMesaIntLatch;

        /// <summary>
        /// The CP's SetMPIntIOP microfunction: raise the mesa-processor interrupt on
        /// slave 8259 IR5 (IOP TechRef Table 3.2).  Edge-triggered, but the CP-side latch
        /// holds IR5 high until the IOP reads port 0xB0 -- so a second SetMPIntIOP before
        /// that read produces no new edge.
        /// </summary>
        public void RaiseMesaInterrupt()
        {
            if (!_mesaIntLatch)
            {
                _mesaIntLatch = true;
                _picSlave.RaiseIrq(5);   // rising edge only
            }
        }

        /// <summary>
        /// Drop the CP-&gt;IOP interrupt latch (CP's ClrMPIntIOP, or the IOP's IN 0xB0).
        /// This lets the CP's next SetMPIntIOP present a fresh rising edge on IR5 -- the
        /// mechanism by which a pre-wait ring is absorbed and re-rung.
        /// </summary>
        public void ClearMesaLatch()
        {
            _mesaIntLatch = false;
            _picSlave.LowerIrq(5);
        }
        private bool _mesaIntLatch;

        public I8259 PicMaster { get { return _picMaster; } }
        public I8259 PicSlave { get { return _picSlave; } }
        public I8259 PicOptions { get { return _picOptions; } }
        public I8254 Pit { get { return _pit; } }

        /// <summary>
        /// Advance time-driven devices (the 8254 counters and the display
        /// vertical-retrace generator) by CPU clocks.  VERTRET (VertRetrIntr) fires
        /// once per field on slave-8259 IR0 (vector 0x30 -> master IR5), independent
        /// of the display VIDEO-enable bit (App A: the CMOS DCM/DDC raises retrace
        /// even at VIDEO=0).  The retrace ISR clears the latch by reading port 0xD0
        /// (ClrRetraceIntr), and the firmware counts these to time the 35 s
        /// boot-device-select timeout.
        /// </summary>
        public void Tick(int clocks)
        {
            _pit.Tick(clocks);
            if (_enet != null) _enet.Tick(clocks);
            if (_rdc != null) _rdc.Tick(clocks);

            _retraceCycles += clocks;
            while (_retraceCycles >= _retracePeriod)
            {
                _retraceCycles -= _retracePeriod;
                _retracePending = true;
                _picSlave.RaiseIrq(0);
                _retraceCount++;
            }
        }

        /// <summary>Cycles per display field (~26.3 ms at 8 MHz = ~210k); tunable for bring-up.</summary>
        public int RetracePeriod { get { return _retracePeriod; } set { _retracePeriod = value > 0 ? value : 1; } }
        public long RetraceCount { get { return _retraceCount; } }
        public long RetraceClearReads { get { return _retraceClearReads; } }

        // ---- Interrupt acknowledge (INTA), cascade-aware ----

        /// <summary>
        /// Resolve a pending, unmasked interrupt through the master 8259 and its
        /// cascaded slaves, returning the interrupt vector (or -1 if none pending).
        /// Wired to the CPU's InterruptAcknowledge hook.
        /// </summary>
        public int AcknowledgeInterrupt()
        {
            SyncInternalIrq();
            SyncFdcIrq();
            SyncRdcIrq();
            SyncSlaveIrq();

            int line;
            if (!_picMaster.HasPending(out line)) return -1;

            if (line == 5)   // master IR5 = slave 8259 cascade (i8259SlaveICW3 = 5)
            {
                int sline;
                if (_picSlave.HasPending(out sline))
                {
                    _picMaster.Acknowledge(5);
                    return _picSlave.Acknowledge(sline);
                }
            }

            if (line == 6 && _pcb != null)   // master IR6 = 80186 internal PIC (iRMX slave)
            {
                int v = _pcb.AcknowledgeInternal();
                if (v >= 0)
                {
                    // In auto-EOI mode the cascade line must not stay in service.
                    if (_pcb.AutoEoi) _picMaster.AcknowledgeCascade(6);
                    else _picMaster.Acknowledge(6);
                    return v;
                }
            }

            return _picMaster.Acknowledge(line);
        }

        /// <summary>Connect the 80186's integrated interrupt controller (drives master IR6).</summary>
        public void SetPcb(I80186Pcb pcb) { _pcb = pcb; }

        /// <summary>Master IR6 tracks the 80186 internal PIC's pending state.</summary>
        private void SyncInternalIrq()
        {
            if (_pcb == null) return;
            if (_pcb.InternalInterruptPending) _picMaster.RaiseIrq(6);
            else _picMaster.LowerIrq(6);
        }

        /// <summary>Master IR5 tracks the slave 8259's pending state (cascade, slave ICW3=5).</summary>
        private void SyncSlaveIrq()
        {
            int line;
            if (_picSlave.HasPending(out line)) _picMaster.RaiseIrq(5);
            else _picMaster.LowerIrq(5);
        }

        /// <summary>Slave IR4 tracks the 8272 FDC's INT pin (FDCIntrReq).  Edge-
        /// triggered: a request is latched on the INT rising edge, so a still-asserted
        /// (not-yet-serviced) INT does not re-fire and storm the CPU.</summary>
        private void SyncFdcIrq()
        {
            bool now = _fdc.Interrupt;
            if (now && !_prevFdcInt) _picSlave.RaiseIrq(4);
            if (!now) _picSlave.LowerIrq(4);
            _prevFdcInt = now;
        }
        private bool _prevFdcInt;

        // ---- Rigid Disk Controller ports (0x0200-0x021F) ----
        private bool RdcPort(ushort port) { return port >= 0x0200 && port <= 0x021F; }

        private byte RdcRead(ushort port)
        {
            _rdc.Log = RdcLog; _rdc.HostClock = RdcHostClock;
            return _rdc.ReadReg(port);
        }

        private void RdcWrite(ushort port, ushort value)
        {
            _rdc.Log = RdcLog; _rdc.HostClock = RdcHostClock;
            _rdc.WriteReg(port, value);
        }

        /// <summary>Edge-raise the two RDC interrupts onto the slave 8259: RDiskCtlrIntr=IR3, RDiskDmaIntr'=IR2
        /// (both cascade to master IR5, alongside FDC=IR4).  RDiskCtlrIntr is read-cleared on 0x0214,
        /// RDiskDmaIntr' on 0x0210 -- so each StartDMA/command re-arms a fresh edge (the old stub latched
        /// _rdcDmaInt forever, so IR2 fired exactly once).  Slave bit numbers are INFERRED (spec open Q1).</summary>
        private void SyncRdcIrq()
        {
            bool ctlr = _rdc.CtlrInt, dma = _rdc.DmaInt;
            if (ctlr && !_prevRdcCtlrInt) _picSlave.RaiseIrq(3);
            if (!ctlr) _picSlave.LowerIrq(3);
            _prevRdcCtlrInt = ctlr;
            if (dma && !_prevRdcDmaInt) _picSlave.RaiseIrq(2);
            if (!dma) _picSlave.LowerIrq(2);
            _prevRdcDmaInt = dma;
        }

        /// <summary>Master IR3 tracks the keyboard UART's RxRDY (KbrdInputReq).</summary>
        private void UpdateKeyboardIrq()
        {
            if (_keyboardUart.RxReady) _picMaster.RaiseIrq(3);
            else _picMaster.LowerIrq(3);
        }

        /// <summary>
        /// Deliver a keyboard scancode to the IOP: latches the byte in the 8251 Rx
        /// and asserts KbrdInputReq (master IR3).  For boot-device selection the byte
        /// is raw (0x63=F1/disk .. 0x6C=F10); KEYMO stores it at HexValue (0x3DB2).
        /// </summary>
        /// <summary>
        /// True while a byte is sitting in the 8251 Rx unread.  Injecting another now would
        /// overrun it and lose one, so multi-byte traffic (mouse reports, press/release
        /// pairs, fast typing) has to wait for this to clear.
        /// </summary>
        public bool KeyboardRxReady { get { return _keyboardUart.RxReady; } }

        public void InjectKeyboard(byte scancode)
        {
            _keyboardUart.InjectRx(scancode);
            UpdateKeyboardIrq();
        }

        // ---- PCS1 control-surface ports (IOP-TR Table 2.3) ----
        private const int InputPort = 0x80;      // read: input port / write: WriteCtlReg
        private const int HostProm = 0x90;       // read: host-address PROM / write: hex LED
        private const int RingLatch = 0xA0;      // read: ClrRingLatch / write: ENetAttn
        private const int MesaIntrLatch = 0xB0;  // read: ClrMesaIntr / write: WriteCSReg
        private const int EnetIntrLatch = 0xC0;  // read: ClrENetIntr / write: WriteResetReg
        private const int RetraceLatch = 0xD0;   // read: ClrRetraceIntr / write: WriteConfigReg (EEPROM/LED)
        private const int ArbHoldIOP = 0xF2;     // read: HoldIOPCmd'
        private const int ArbAllowRDC = 0xF4;    // read: AllowRDCCmd'
        private const int ArbAllowPC = 0xF8;     // read: AllowPCCmd'

        // System-board I/O.
        private const int DaybreakBankReg = 0xE000;  // OUT byte: CP control-store bank select
        private const int WcsBase = 0x8000;      // CP Writable Control Store window 0x8000-0xDFFF
        private const int WcsEnd = 0xDFFF;
        private const int MapRegBase = 0xE010;   // E010-E01F : 16 map registers
        private const int MapRegEnd = 0xE01F;
        private const int DisplayTypePort = 0xECCC;

        public byte ReadByte(ushort port)
        {
            Log(true, port, 0);

            if (_display.Handles(port))
            {
                return _display.ReadByte(port);
            }

            if (RdcPort(port)) return RdcRead(port);

            switch (port)
            {
                // 8259 PICs (even = command/IRR/ISR, odd = data/IMR).
                case 0x00: SyncInternalIrq(); SyncSlaveIrq(); return _picMaster.ReadCommand();
                case 0x02: return _picMaster.ReadData();
                case 0x10: return _picSlave.ReadCommand();
                case 0x12: return _picSlave.ReadData();
                case 0x60: return _picOptions.ReadCommand();
                case 0x62: return _picOptions.ReadData();

                // 8254 PIT counters.
                case 0x20: return _pit.ReadCounter(0);
                case 0x22: return _pit.ReadCounter(1);
                case 0x24: return _pit.ReadCounter(2);

                // 8251 keyboard/mouse USART.
                case 0x30: { byte b = _keyboardUart.ReadData(); UpdateKeyboardIrq(); return b; }
                case 0x32: return _keyboardUart.ReadStatus();

                // 8272A floppy disk controller.
                case 0x50: return _fdc.ReadMainStatus();
                case 0x52: return _fdc.ReadData();
                case 0x54: return _fdc.ReadDmaData();

                // i8255 PPI (Burdock/Bindweed umbilical debugger interface): 0x70=Port A, 0x72=Port B,
                // 0x74=Port C, 0x76=control.  No umbilical is connected, so all inputs read 0 -- in particular
                // Port C bit3 (outReady) and bit4 (PC4 loopback) are clear, so the ROM's Bindweed debugger-detect
                // fails (PC4 loopback mismatches + the PC3 IOPAlive poll times out) -> NoDebugger -> StartOPIE.
                case 0x70: return 0x00;
                case 0x72: return 0x00;
                case 0x74: return 0x00;

                case InputPort:
                    // Input port low byte: machine-ID and RS232/modem bits.  b6
                    // (machineIDMask 0x40) = 1 -> Daybreak (6085), 0 -> Daisy; DoveCP.asm
                    // reads this to pick CPType and, if it reads Daisy, SKIPS every
                    // Daybreak CP microcode block in the .db (=> zero WCS writes).  So the
                    // low byte must expose b6 from _inputPortWord.  (b11/b13 are high-byte;
                    // see ReadWord.)
                    return (byte)_inputPortWord;

                case HostProm:
                case 0x92: case 0x94: case 0x96: case 0x98: case 0x9A: case 0x9C: case 0x9E:
                    return ReadHostProm(port);

                // Retrace-interrupt clear-on-read: the display ISR reads 0xD0 each
                // field to reset the latch and deassert slave IR0.
                case RetraceLatch:
                    _retracePending = false;
                    _picSlave.LowerIrq(0);
                    _retraceClearReads++;
                    if (PicLogEnabled && PicLog.Count < 300) PicLog.Add("D0read");
                    return 0x00;

                // MesaIntrLatch (0xB0): reading the CSReg port is "Clear Mesa Interrupt
                // Latch" -- the IOP acknowledging the CP->IOP (SetMPIntIOP) doorbell.  The
                // latch held slave IR5 high; the read drops it.
                case MesaIntrLatch:
                    _mesaIntLatch = false;
                    _picSlave.LowerIrq(5);
                    if (OnReadMesaIntLatch != null) OnReadMesaIntLatch();
                    return 0x00;

                // Other interrupt-latch clear-on-read ports.
                case RingLatch:
                    EnetLatchReads++;
                    return 0x00;

                case EnetIntrLatch:
                    // ClrENetIntr: reading the latch is what drops the 82586's interrupt.
                    // Counting it says whether the driver's ISR is running at all -- if this
                    // stays at zero the interrupt is never being delivered.
                    EnetLatchReads++;
                    if (_enet != null) _enet.InterruptAcknowledges++;
                    _picSlave.LowerIrq(1);
                    return 0x00;

                // Arbiter command read-strobes.  The read IS the command for the RDC-DMA bus grant
                // (AllowRDC, bit 0x04), and returning 0 was fine for every Pilot 14 guest -- which is
                // why the old comment here claimed the value is "never tested by the firmware".
                //
                // That claim is false for at least one guest.  The Pilot 12 Medley installation
                // utility reads this port 13,541,312 times while stalled at MP 0199 -- three orders
                // of magnitude more than any other port -- so it is plainly testing what comes back
                // and waiting for a bit we never set.  DOVE_ARB_F4=<hex> overrides the returned byte
                // so the expected value can be found by sweep rather than assumed.
                case ArbAllowRDC:
                    _allowRDC = true; ArbAllowRdcCount++;
                    return ArbF4Value;
                case ArbHoldIOP:
                case ArbAllowPC:
                    return 0x00;

                case DisplayTypePort:
                    return _displayType;

                default:
                    return _readDefault;
            }
        }

        /// <summary>Diagnostic: high-port (>=0x8000) writes, bucketed by high nibble
        /// (the CP Writable Control Store lanes 0x8000-0xDFFF, IORegion, etc.).</summary>
        public readonly System.Collections.Generic.SortedDictionary<int, long> HighPortWrites
            = new System.Collections.Generic.SortedDictionary<int, long>();

        /// <summary>Diagnostic: the CP microcode-load control sequence (bank/CSReg/reset).</summary>
        public System.Collections.Generic.List<string> CpLoadLog;

        public void WriteByte(ushort port, byte value)
        {
            Log(false, port, value);

            if (RdcPort(port)) { RdcWrite(port, value); return; }

            if (port >= 0x8000)
            {
                int bucket = port & 0xF000;
                long c; HighPortWrites.TryGetValue(bucket, out c); HighPortWrites[bucket] = c + 1;
            }

            // ---- Ethernet control surface (82586 not implemented) ----
            // ENetAttn is the 82586's Channel Attention line: the guest raises it to say a
            // command block is waiting in shared memory.  Counting it separates "the guest
            // never got as far as the hardware" from "it asked and nothing answered".
            if (port == RingLatch || port == RingLatch + 1)
            {
                EnetAttnWrites++;
                if (EnetEnabled) _enet.ChannelAttention();
            }

            // ---- Diagnostic: capture the CP microcode-load control sequence ----
            // Bank register (0xE000), WriteCSReg (0xB0/B1 = CP start/halt), and the
            // resetMesaProcessor bit (0xC0 b6) so we can watch the whole hand-off.
            if (CpLoadLog != null && CpLoadLog.Count < 400)
            {
                if (port == DaybreakBankReg)
                    CpLoadLog.Add("BANK=" + value.ToString("X2"));
                else if (port == MesaIntrLatch || port == MesaIntrLatch + 1)
                    CpLoadLog.Add("CSReg[" + (port & 1) + "]=" + value.ToString("X2"));
                else if (port == EnetIntrLatch && ((value ^ _prevResetReg) & 0x40) != 0)
                    CpLoadLog.Add("MesaReset " + (((value & 0x40) != 0) ? "ASSERT" : "RELEASE"));
            }

            // ---- CP Writable Control Store load ----
            // The .db loader streams CP microcode out the WCS window (0x8000-0xDFFF, one
            // byte-lane per OUT) and selects the bank via a byte to daybreakBankRegister
            // (0xE000) before each block.  Route both into the control store.
            if (port >= WcsBase && port <= WcsEnd)
            {
                _controlStore.WriteLane(port, value);
                return;
            }
            if (port == DaybreakBankReg)
            {
                _controlStore.SelectBank(value);
                return;
            }

            if (port >= MapRegBase && port <= MapRegEnd)
            {
                _memory.SetMapRegister(port - MapRegBase, value);
                return;
            }

            if (_display.Handles(port))
            {
                _display.WriteByte(port, value);
                return;
            }

            switch (port)
            {
                // 8259 PICs (even = command, odd = data).
                case 0x00: if (PicLogEnabled && PicLog.Count < 300) PicLog.Add("Mcmd=" + value.ToString("X2")); _picMaster.WriteCommand(value); return;
                case 0x02: if (PicLogEnabled && PicLog.Count < 300) PicLog.Add("Mdata=" + value.ToString("X2")); _picMaster.WriteData(value); return;
                case 0x10: if (PicLogEnabled && PicLog.Count < 300) PicLog.Add("Scmd=" + value.ToString("X2")); _picSlave.WriteCommand(value); return;
                case 0x12: if (PicLogEnabled && PicLog.Count < 300) PicLog.Add("Sdata=" + value.ToString("X2")); _picSlave.WriteData(value); return;
                case 0x60: _picOptions.WriteCommand(value); return;
                case 0x62: _picOptions.WriteData(value); return;

                // 8254 PIT counters + control word.
                case 0x20: _pit.WriteCounter(0, value); return;
                case 0x22: _pit.WriteCounter(1, value); return;
                case 0x24: _pit.WriteCounter(2, value); return;
                case 0x26: _pit.WriteControl(value); return;

                // 8251 keyboard/mouse USART.
                case 0x30: _keyboardUart.WriteData(value); UpdateKeyboardIrq(); return;
                case 0x32: _keyboardUart.WriteControl(value); UpdateKeyboardIrq(); return;

                // 8272A floppy: 0x50 status is read-only; 0x52 = command FIFO; 0x54 = DMA write.
                case 0x50: return;
                case 0x52: _fdc.WriteData(value); return;
                case 0x54: return;

                // i8255 PPI (Burdock/Bindweed umbilical): 0x70=Port A out, 0x76=control (mode/BSR).  No umbilical
                // attached, so writes drive nothing; capture Port A output for debug only.
                case 0x70: if (DiagUartTxRaw.Count < 40000) DiagUartTxRaw.Add(value); return;
                case 0x76: return;

                case HostProm:                 // hex LED display (low byte)
                    _led = (ushort)((_led & 0xFF00) | value);
                    RecordLed();
                    break;
                case HostProm + 1:
                    _led = (ushort)((_led & 0x00FF) | (value << 8));
                    RecordLed();
                    break;

                case InputPort:                // WriteCtlReg (low byte)
                    _controlReg = (ushort)((_controlReg & 0xFF00) | value);
                    break;
                case InputPort + 1:
                    _controlReg = (ushort)((_controlReg & 0x00FF) | (value << 8));
                    break;

                case EnetIntrLatch:            // WriteResetReg (low byte)
                    _resetReg = (ushort)((_resetReg & 0xFF00) | value);
                    // Bit 3 = resetKeyboardUART: the ROM pulses this before each 8251
                    // (re)programming, returning it to expecting a mode word.
                    if ((value & 0x08) != 0) _keyboardUart.Reset();
                    // Bit 2 = resetFloppyController (active-low): the 0->1 release edge
                    // re-initializes the 8272 (drive-status senses + INT).
                    if ((value & 0x04) != 0 && (_prevResetReg & 0x04) == 0) _fdc.Reset();
                    // Bit 6 = resetMesaProcessor (active-high enable): the reg powers up 0
                    // (CP held in reset); the downloaded loader writes b6=1 to RELEASE the
                    // CP once its microcode is loaded.  Notify on any change.
                    if (((value ^ _prevResetReg) & 0x40) != 0 && OnMesaReset != null)
                        OnMesaReset((value & 0x40) != 0);
                    _prevResetReg = value;
                    break;
                case EnetIntrLatch + 1:
                    _resetReg = (ushort)((_resetReg & 0x00FF) | (value << 8));
                    break;

                case MesaIntrLatch:            // WriteCSReg low byte (Mesa proc / control store)
                    _csReg = (ushort)((_csReg & 0xFF00) | value);
                    if (OnWriteCSReg != null) OnWriteCSReg(_csReg);
                    break;
                case MesaIntrLatch + 1:        // WriteCSReg high byte
                    _csReg = (ushort)((_csReg & 0x00FF) | (value << 8));
                    if (OnWriteCSReg != null) OnWriteCSReg(_csReg);
                    break;

                case RetraceLatch:             // WriteConfigReg (EEPROM + diag LEDs)
                case RetraceLatch + 1:
                    break;

                default:
                    break;
            }
        }

        public ushort ReadWord(ushort port)
        {
            // 16-bit register reads (the ROM reads the input port as a word for the
            // control-store / EEPROM / modem status bits).
            Log(true, port, 0);

            if (_display.Handles(port))
            {
                return (ushort)(_display.ReadByte(port) | (_display.ReadByte(port + 1) << 8));
            }

            if (RdcPort(port)) return RdcRead(port);
            if (port >= 0xF0 && port <= 0xFF)   // arbiter command-strobe (word IN); returned value never tested
            {
                if ((port & 0x04) != 0) { _allowRDC = true; ArbAllowRdcCount++; }
                return 0;
            }

            switch (port)
            {
                case InputPort:
                    // Input port word; bit 11 = config-EEPROM (93C46) data-out.
                    {
                        ushort v = (ushort)(_inputPortWord | (_configEeprom.DataOut ? I93C46.DataOutMask : 0));
                        // The two guest-side gates on CP microcode loading both read through here:
                        // machine-ID bit 6 must be 1 (Daybreak) or ALL CP blocks are skipped, and the
                        // 93C46 csBankConfiguration is clocked out on bit 11.  When the WCS ends up
                        // empty with the block loop spinning (MP 0199), it is because the loader
                        // decided not to stream -- WriteLane stores unconditionally, so zero
                        // occupancy means zero writes -- and these reads are what it decided on.
                        InputPortReads++;
                        if (((v >> 11) & 1) != 0) InputPortB11High++;
                        if (GateLog != null && GateLog.Count < 300)
                            GateLog.Add("in80=" + v.ToString("X4")
                                + " b6=" + ((v >> 6) & 1)
                                + " b11(eeprom)=" + ((v >> 11) & 1)
                                + "  reads=" + InputPortReads);
                        return v;
                    }
                case MesaIntrLatch:
                    // IN AX,0xB0 = Clear Mesa Interrupt Latch (the mesa task reads it as a
                    // WORD).  Same clear-on-read side effect as the byte path.
                    ClearMesaLatch();
                    if (OnReadMesaIntLatch != null) OnReadMesaIntLatch();
                    return 0;
                default:
                    return (ushort)(ReadByteNoLog(port) | (ReadByteNoLog((ushort)(port + 1)) << 8));
            }
        }

        public long WcsWordWrites = 0;   // TEMP: count 16-bit OUTs that land in the WCS window (0x8000-0xDFFF)
        public void WriteWord(ushort port, ushort value)
        {
            if (RdcPort(port)) { RdcWrite(port, value); return; }
            if (port >= WcsBase && port <= WcsEnd) WcsWordWrites++;
            if (port == RetraceLatch)
            {
                // WriteConfigReg (0xD0): drives the 93C46 config EEPROM (CS/clock/DI)
                // and the diagnostic LEDs.
                _configEeprom.WriteConfig(value);
                return;
            }

            WriteByte(port, (byte)value);
            WriteByte((ushort)(port + 1), (byte)(value >> 8));
        }

        // ---- Instrumentation / configuration ----

        /// <summary>The 82586 posts status and interrupts on slave IR1.</summary>
        private void RaiseEnetInterrupt()
        {
            _picSlave.RaiseIrq(1);
            SyncSlaveIrq();
        }

        /// <summary>The Ethernet controller (command completion only -- it moves no packets).</summary>
        public I82586 Ethernet { get { return _enet; } }

        /// <summary>Set false to go back to having no Ethernet controller at all.</summary>
        public bool EnetEnabled = true;

        private I82586 _enet;

        /// <summary>82586 Channel Attention raises, and reads of the ring/interrupt latches.</summary>
        public long EnetAttnWrites;
        public long EnetLatchReads;

        /// <summary>Latest and full history of the 4-digit hex LED (WriteLED @ 0x90) = POST progress.</summary>
        public ushort Led { get { return _led; } }
        public List<ushort> LedHistory { get { return _ledHistory; } }

        // ---- Diagnostic RS232 UART (ports 0x70 TX data, 0x72 RX data, 0x74 status, 0x76 control) ----
        // Used by the microcode-diagnostics disk (130p26403) for its serial console.  Status bit0=RX-ready,
        // bit3=TX-ready.  We keep TX always ready and capture the transmitted bytes; RX is fed from a queue.
        public readonly System.Text.StringBuilder DiagUartTx = new System.Text.StringBuilder();
        public readonly List<byte> DiagUartTxRaw = new List<byte>();
        public readonly Queue<byte> DiagUartRx = new Queue<byte>();
        public byte DiagUartStatusExtra = 0x00;   // extra status bits OR'd into 0x74 (e.g. 0x10) for probing
        public bool DiagUartLoopback = true;       // loop TX back to RX so the UART self-test passes

        /// <summary>Set to a list to record input-port (0x80) reads -- the microcode-load gates.</summary>
        public List<string> GateLog;
        public long InputPortReads;
        /// <summary>How many input-port reads saw bit 11 (EEPROM DO / READY) high.</summary>
        public long InputPortB11High;

        public ushort ControlReg { get { return _controlReg; } }
        public ushort ResetReg { get { return _resetReg; } }

        /// <summary>Value returned by unmodeled read ports (tunable during bring-up).</summary>
        public byte ReadDefault { get { return _readDefault; } set { _readDefault = value; } }

        /// <summary>Value returned by a word read of the input port (0x80).</summary>
        public ushort InputPortWord { get { return _inputPortWord; } set { _inputPortWord = value; } }

        public ushort DisplayType { get { return _displayType; } set { _displayType = (byte)value; } }

        public bool LogEnabled { get { return _logEnabled; } set { _logEnabled = value; } }

        public struct IoAccess { public bool Read; public ushort Port; public ushort Val; }
        public List<IoAccess> AccessLog { get { return _accessLog; } }

        /// <summary>Per-port access counts (helps spot a poll loop stuck on one port).</summary>
        public Dictionary<ushort, int> PortCounts { get { return _portCounts; } }

        private byte ReadHostProm(int port)
        {
            // 90,92,...,9E : 8 bytes of the 48-bit host MAC + checksum + complement.
            int index = (port - 0x90) >> 1;
            return (index >= 0 && index < 8) ? _hostProm[index] : (byte)0xFF;
        }

        private byte ReadByteNoLog(ushort port)
        {
            switch (port)
            {
                case HostProm:
                case 0x92: case 0x94: case 0x96: case 0x98: case 0x9A: case 0x9C: case 0x9E:
                    return ReadHostProm(port);
                default:
                    return _readDefault;
            }
        }

        private void RecordLed()
        {
            if (_ledHistory.Count == 0 || _ledHistory[_ledHistory.Count - 1] != _led)
            {
                _ledHistory.Add(_led);
            }
        }

        private void Log(bool read, ushort port, ushort val)
        {
            int c;
            _portCounts.TryGetValue(port, out c);
            _portCounts[port] = c + 1;

            if (_logEnabled && _accessLog.Count < 200000)
            {
                _accessLog.Add(new IoAccess { Read = read, Port = port, Val = val });
            }
        }

        private readonly DoveIOPMemory _memory;
        private readonly I8259 _picMaster;
        private readonly I8259 _picSlave;
        private readonly I8259 _picOptions;
        private readonly I8254 _pit;
        private readonly I8251 _keyboardUart;
        private readonly I8272 _fdc;
        private readonly DoveControlStore _controlStore;
        public int _lastDmaDest, _lastDmaLen;   // diagnostics for the last FDC DMA transfer
        public System.Collections.Generic.List<string> DmaLog;  // ring of last FDC DMA transfers (count vs delivered)
        public System.Collections.Generic.List<string> DmaFirst; // first 60 FDC DMA transfers WITH the sector (R5 vs R6 vs R9)
        public int MinDmaDest = -1, MaxDmaDest = 0;  // span of IOP linear addrs the floppy load DMA'd into
        public long DmaByteTotal;                    // total bytes DMA'd from the floppy
        private readonly DoveDisplayController _display;
        private readonly I93C46 _configEeprom;
        private readonly byte[] _hostProm = new byte[8];
        private I80186Pcb _pcb;

        // ---- Rigid Disk Controller (RDC): 8x305 command/status @ 0x0214 + AM2942 DMA/FIFO @ 0x0200-0x0216 ----
        // Behavioral 8X305 + AM2942 in DoveDiskController over a Micropolis-1325 backing store.  The DOB
        // round-trips through physical DRAM, ops execute against the pack, and the two completion interrupts
        // (RDiskCtlrIntr = slave IR3, RDiskDmaIntr' = slave IR2) cascade to master IR5 -> IOP ISR.
        private readonly DoveDiskController _rdc;
        private readonly Micropolis1325 _disk;
        /// <summary>
        /// Byte returned by a read of 0xF4 (ArbAllowRDC).  Default 0, which every Pilot 14 guest
        /// accepts; DOVE_ARB_F4 overrides it for guests that test the value.
        /// </summary>
        private static byte ArbF4Value
        {
            get
            {
                if (_arbF4 < 0)
                {
                    string e = Environment.GetEnvironmentVariable("DOVE_ARB_F4");
                    byte v;
                    _arbF4 = (!string.IsNullOrEmpty(e) &&
                              byte.TryParse(e, System.Globalization.NumberStyles.HexNumber, null, out v))
                             ? v : 0x00;
                }
                return (byte)_arbF4;
            }
        }
        private static int _arbF4 = -1;

        private bool _allowRDC;                       // set by the 0xF4 arbiter AllowRDC command
        private bool _prevRdcCtlrInt, _prevRdcDmaInt; // edge state for the slave-8259 raise
        public System.Collections.Generic.List<string> RdcLog;   // diagnostic: every RDC/arbiter access
        public long RdcHostClock;                     // set by the harness for RdcLog timestamps
        public long ArbAllowRdcCount;                 // 0xF4 AllowRDC issue count (the 7.7M-poll)
        /// <summary>The rigid-disk backing store; attach an image via Disk.Load(path) before boot.</summary>
        public Micropolis1325 Disk { get { return _disk; } }
        /// <summary>The behavioral rigid-disk controller (register interface + op execution).</summary>
        public DoveDiskController Rdc { get { return _rdc; } }

        // Display vertical-retrace generator (slave IR0).
        private int _retracePeriod = 210400;   // ~26.3 ms field at 8 MHz
        private int _retraceCycles;
        private bool _retracePending;
        private long _retraceCount;
        private long _retraceClearReads;

        /// <summary>Diagnostic: when enabled, records 8259 command/data writes and 0xD0 reads.</summary>
        public bool PicLogEnabled;
        public System.Collections.Generic.List<string> PicLog = new System.Collections.Generic.List<string>();

        private ushort _led;
        private ushort _controlReg;
        private ushort _resetReg;
        private ushort _csReg;
        private byte _prevResetReg;
        // b6 (0x40, machineIDMask) = 1 -> Daybreak/6085.  Required: DoveCP.asm skips all
        // Daybreak CP microcode blocks in the .db if this reads 0 (Daisy).
        //
        // EVERY OTHER BIT READS 0, and we do not know what they are.  That is fine for the Pilot /
        // ViewPoint boot, which never waits on one, but the XDE/Interlisp installer stalls at MP
        // 0199 polling this port 300+ times with the value constant at 0x0040 -- i.e. waiting on a
        // bit we do not model.  DOVE_INPUT_PORT=<hex> overrides the whole word so the bit can be
        // found by sweep rather than by guessing at the hardware.
        private ushort _inputPortWord = InitialInputPort();
        private static ushort InitialInputPort()
        {
            string e = Environment.GetEnvironmentVariable("DOVE_INPUT_PORT");
            ushort v;
            if (!string.IsNullOrEmpty(e) &&
                ushort.TryParse(e, System.Globalization.NumberStyles.HexNumber, null, out v)) return v;
            return 0x0040;
        }
        private byte _displayType = 0x0000;
        private byte _readDefault = 0x00;
        private bool _eepromDataOut = false;
        private bool _logEnabled = false;

        private readonly List<ushort> _ledHistory = new List<ushort>();
        private readonly List<IoAccess> _accessLog = new List<IoAccess>();
        private readonly Dictionary<ushort, int> _portCounts = new Dictionary<ushort, int>();
    }
}
