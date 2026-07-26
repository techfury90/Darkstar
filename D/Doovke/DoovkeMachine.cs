using System;
using System.IO;
using D.CP;
using D.IOP;

namespace D.Doovke
{
    /// <summary>
    /// The Dove/Daybreak (Xerox 6085) machine.
    ///
    /// Darkstar is the DLion (8010) target; Doovke is Dove/Daybreak.  The two share the
    /// low-level parts (AM2901, Microinstruction, FloppyDisk) but diverge completely at the
    /// CP, the IOP and the display, which is why they are separate build targets rather than
    /// a runtime switch.
    ///
    /// The assembly below is exactly the configuration that boots the Offline Diagnostics
    /// system to its floppy prompt:  the IOP (80186) runs the real Opie boot firmware out of
    /// the boot ROM; the CP (AM2901 microengine) runs the Mesa microcode the IOP streams into
    /// the writable control store; and both share ONE physical DRAM array (DoveIOPMemory's
    /// SystemRaw) with no bridge -- the twisted bus is compensated for in-guest.
    /// </summary>
    public sealed class DoovkeMachine
    {
        private readonly DoveIOPMemory _mem;
        private readonly DoveIOPIO _io;
        private readonly DoveCentralProcessor _cp;
        private readonly I80186Pcb _pcb;
        private readonly i80186 _iop;
        private readonly byte[] _sysRam;
        private readonly int _ramMask;

        /// <summary>CP microinstructions executed per IOP instruction.</summary>
        private const int CpStepsPerIopInstruction = 4;

        public DoveIOPMemory Memory { get { return _mem; } }
        public DoveIOPIO Io { get { return _io; } }
        public DoveCentralProcessor Cp { get { return _cp; } }
        public i80186 Iop { get { return _iop; } }
        public DoveDisplayController Display { get { return _io.Display; } }

        public long IopInstructions { get; private set; }
        public bool Halted { get { return _iop.Halted; } }

        public DoovkeMachine(string bootRomPath, string configEepromPath)
        {
            if (string.IsNullOrEmpty(bootRomPath)) throw new ArgumentNullException("bootRomPath");

            _mem = new DoveIOPMemory(bootRomPath);
            _io = new DoveIOPIO(_mem);

            if (!string.IsNullOrEmpty(configEepromPath))
            {
                _io.LoadConfigEeprom(File.ReadAllBytes(configEepromPath));
            }

            // Vertical-retrace period in IOP clocks.  The boot polls the keyboard off retrace,
            // so this paces the boot-device selection; 20000 is the value the proven boot uses
            // (the 210400 default is the true ~26.3 ms field rate and is far slower).
            _io.RetracePeriod = 20000;

            // The CP takes its microcode from the writable control store the IOP loads.
            _cp = new DoveCentralProcessor(_io.ControlStore);

            // CP <-> DRAM.  The CP is big-endian (Mesa) and addresses memory by 16-bit word.
            _sysRam = _mem.SystemRaw;
            _ramMask = _sysRam.Length - 1;
            _cp.ReadWord = ReadCpWord;
            _cp.WriteWord = WriteCpWord;

            // ---- CP <-> IOP control links ------------------------------------------------
            // The IOP owns the CP's run/reset lines and the two doorbell latches; the CP is
            // otherwise inert.  Without these the microcode loads but the CP never starts.
            //   OnWriteCSReg     : CSReg bit 9 = RUN (bit 8 is the IOP->CP doorbell)
            //   OnMesaReset      : the MesaReset line, asserted low
            //   OnReadMesaIntLatch: IN 0xB0 clears the CP->IOP doorbell latch
            //   OnMesaInterrupt  : CP rings the IOP (SetMPIntIOP)
            _io.OnWriteCSReg = v => _cp.WriteCSReg(v);
            _io.OnMesaReset = release => _cp.SetReset(!release);
            _io.OnReadMesaIntLatch = () => { _cp.MesaInterruptRequest = false; };
            _cp.OnMesaInterrupt = () => _io.RaiseMesaInterrupt();

            _pcb = new I80186Pcb();
            _iop = new i80186(_mem, _io, _pcb);

            // Straps read by the firmware, both already correct by default but called out
            // because the boot depends on them:
            //   _io.InputPortWord  bit 6 = 1  -> machine ID says Daybreak.  If this reads 0
            //                                   the .db loader skips every CP microcode block.
            //   _io.Display.TypeSize bit 0 = 0 -> 19" CRT (1152x861).  bit 0 = 1 is the 15"
            //                                   monitor (832x633); the firmware reads 0xECCC
            //                                   to size the bitmap.
            // Expose them here so the UI can offer a 15"/19" choice later.

            // Hardware-interrupt delivery.  IIOBus186 carries port I/O only, so the CPU takes
            // maskable interrupts ONLY through this callback -- leave it null and every POST
            // step that waits on an IRQ spins forever (the first is the keyboard-UART loopback
            // at FC00:38BF, which OUTs to port 0x30 and waits for the 8251's interrupt).
            _iop.InterruptAcknowledge = () => _io.AcknowledgeInterrupt();

            // The 80186 must be reset to land on the reset vector at 0xFFFF0; without this it
            // runs from an uninitialised state and never reaches POST, the floppy or the CP.
            _iop.Reset();
        }

        private ushort ReadCpWord(int wordAddress)
        {
            int b = (wordAddress << 1) & _ramMask;
            return (ushort)((_sysRam[b] << 8) | _sysRam[(b + 1) & _ramMask]);
        }

        private void WriteCpWord(int wordAddress, ushort value)
        {
            int b = (wordAddress << 1) & _ramMask;
            _sysRam[b] = (byte)(value >> 8);
            _sysRam[(b + 1) & _ramMask] = (byte)value;
        }

        /// <summary>Run one IOP instruction and the CP steps that accompany it.</summary>
        public void Step()
        {
            // Time base for the subsystems that need to know how far the machine has run
            // (memory timing, the RDC's host clock).  Leaving these at zero stalls POST.
            _mem.HostClock = IopInstructions;
            _mem.CurrentPC = _iop.InstructionAddress;
            _io.RdcHostClock = IopInstructions;

            int clocks = _iop.Execute();
            _io.Tick(clocks);
            // Execute() no-ops while the CP is halted, so this is safe before the IOP starts it.
            _cp.Execute(CpStepsPerIopInstruction);
            IopInstructions++;
        }

        public void Run(long iopInstructions)
        {
            for (long i = 0; i < iopInstructions; i++) Step();
        }

        /// <summary>
        /// Deliver a key to the guest.  The boot-device selection at the "Boot device?" stage
        /// and every later prompt (including the Offline Diagnostics floppy prompt) arrive this
        /// way; without one the boot parks waiting for a boot device.
        /// </summary>
        public void InjectKey(byte scanCode)
        {
            _io.InjectKeyboard(scanCode);
        }

        public void LoadFloppy(int drive, string path)
        {
            _io.Fdc.Drives[drive] = new D.IO.FloppyDisk(path);
        }

        public void EjectFloppy(int drive)
        {
            _io.Fdc.Drives[drive] = null;
        }

        /// <summary>
        /// Render the current display into a byte-per-pixel mono frame.
        /// Returns null if the display has not been programmed yet.
        /// </summary>
        public byte[] RenderFrame(out int width, out int height)
        {
            var display = _io.Display;
            display.RenderMono();
            width = display.Width;
            height = display.Height;
            return display.Frame;
        }
    }
}
