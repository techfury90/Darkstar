using System;

namespace D.IOP
{
    /// <summary>
    /// Intel 82586 Ethernet Data Link Controller, as fitted to the Dove/Daybreak IOP board
    /// (82586 DLC + SEEQ 8023A serial interface, IOP-TR sec 6.2).
    ///
    /// The 82586 is a bus-master coprocessor, not a port-mapped device: the driver builds
    /// command blocks in shared memory, rings Channel Attention (a write to IOP port 0xA0),
    /// and the chip walks those structures itself, then posts status and raises INT (slave
    /// IR1, cleared by reading port 0xC0).
    ///
    /// SCP (0xFFFF6) -> ISCP -> SCB -> command block list.  On this board the SCP lives in the
    /// IOP's own boot EPROM so the board can run standalone during diagnostics, which is why
    /// the fixed address falls inside ROM rather than main memory.
    ///
    /// THIS IS THE COMMAND-COMPLETION MODEL ONLY.  It accepts commands, marks them complete
    /// and interrupts -- it does not move packets.  That is deliberate and it is what unblocks
    /// software: without an acknowledging chip, a driver's request can neither succeed nor
    /// fail, so it retries forever and its own timeout never gets to run.  (Measured: the
    /// diagnostics' XNS time request rang Channel Attention once a second indefinitely.  On
    /// real hardware that request times out and the machine prompts for the time by hand.)
    /// Receive is wired to deliver nothing, so a request goes out, no reply arrives, and the
    /// software times out exactly as it did on the real machine.
    ///
    /// Frames are dropped rather than transmitted; the RFA is not walked.  Real packet I/O
    /// goes on top of this, against the existing IPacketInterface.
    /// </summary>
    public class I82586
    {
        /// <summary>Fixed SCP address the chip fetches on the first Channel Attention.</summary>
        public const int ScpAddress = 0xFFFF6;

        /// <summary>
        /// The 82586 drives a 24-bit address, but the board decodes it into the IOP's 20-bit
        /// space -- so pointers out of the SCP/ISCP must be masked, not taken at face value.
        /// The boot EPROM's SCP holds ISCP = 0xF000A0, which is nonsense as a 24-bit address
        /// and is 0x000A0 -- local SRAM, where the IOP-TR says the ISCP lives -- once masked.
        /// Getting this wrong is silent: the busy flag is written somewhere harmless, the
        /// driver never sees initialisation complete, and it rings Channel Attention forever.
        /// </summary>
        private const int AddressMask = 0xFFFFF;

        // SCB status word.
        private const ushort StatCx = 0x8000;    // command executed
        private const ushort StatFr = 0x4000;    // frame received
        private const ushort StatCna = 0x2000;   // command unit left the active state
        private const ushort StatRnr = 0x1000;   // receive unit left the ready state
        private const ushort StatAckMask = 0xF000;
        private const ushort StatRusMask = 0x0070;

        // Command block status word.
        private const ushort CbComplete = 0x8000;
        private const ushort CbBusy = 0x4000;
        private const ushort CbOk = 0x2000;

        // Receive unit status: ready.
        private const ushort RusReady = 0x0040;

        private readonly IPhysicalMemory _memory;
        private readonly Action _raiseInterrupt;

        private bool _initialised;
        private int _scbAddress;
        private int _scbBase;
        private int _iscpAddress;

        public I82586(IPhysicalMemory memory, Action raiseInterrupt)
        {
            _memory = memory;
            _raiseInterrupt = raiseInterrupt;
            Reset();
        }

        public void Reset()
        {
            _initialised = false;
            _scbAddress = 0;
            _scbBase = 0;
            _iscpAddress = 0;
            ChannelAttentions = 0;
            CommandsExecuted = 0;
            FramesDropped = 0;
            InterruptsRaised = 0;
            _interruptCountdown = -1;
        }

        // ---- Diagnostics ----
        public long ChannelAttentions { get; private set; }
        public long CommandsExecuted { get; private set; }
        public long FramesDropped { get; private set; }
        public int ScbAddress { get { return _scbAddress; } }
        public int IscpAddress { get { return _iscpAddress; } }
        public int ScbBase { get { return _scbBase; } }
        public bool Initialised { get { return _initialised; } }

        /// <summary>
        /// Channel Attention: the driver's doorbell.  The first one after reset runs the
        /// configuration fetch; later ones execute whatever is in the SCB.
        /// </summary>
        public void ChannelAttention()
        {
            ChannelAttentions++;
            if (!_initialised) Initialise();
            else ExecuteScb();
            LogAttention();
        }

        /// <summary>
        /// Clocks the chip takes to respond to Channel Attention.  This delay is NOT padding:
        /// the driver's sequence is "ring CA, then wait for the interrupt", and a real 82586
        /// spends tens of microseconds fetching SCP/ISCP, so the interrupt always lands after
        /// the driver is already waiting.  Answering instantly from inside the OUT instruction
        /// makes the 80186 take the interrupt at the very next instruction boundary -- before
        /// the wait is entered -- and a wait that arms rather than checking a latch then blocks
        /// forever.  The classic "works on hardware, hangs under emulation" shape.
        /// </summary>
        public int InterruptDelayClocks = 2000;   // ~250us at 8 MHz

        private int _interruptCountdown = -1;

        private void ScheduleInterrupt()
        {
            _interruptCountdown = InterruptDelayClocks;
        }

        /// <summary>Advance the chip's own timing; called from the IOP's tick.</summary>
        public void Tick(int clocks)
        {
            if (_interruptCountdown < 0) return;
            _interruptCountdown -= clocks;
            if (_interruptCountdown > 0) return;
            _interruptCountdown = -1;
            InterruptsRaised++;
            _raiseInterrupt();
        }

        public long InterruptsRaised { get; private set; }

        /// <summary>
        /// Optional trace of the first attentions: what the chip resolved the pointers to and
        /// what it found in the SCB.  Set DOVE_ENET_LOG to a path to collect it.  Guessing at
        /// this from the datasheet alone is what put the address mask wrong, so make the real
        /// values visible instead.
        /// </summary>
        private void LogAttention()
        {
            if (_log == null)
            {
                if (_logChecked) return;
                _logChecked = true;
                string path = Environment.GetEnvironmentVariable("DOVE_ENET_LOG");
                if (string.IsNullOrEmpty(path)) return;
                _log = new System.IO.StreamWriter(path) { AutoFlush = true };
                _log.WriteLine("SCP@{0:X5}: sysbus={1:X2} iscpPtr={2:X4}{3:X4}",
                    ScpAddress, _memory.ReadByte(ScpAddress),
                    _memory.ReadWord(ScpAddress + 8), _memory.ReadWord(ScpAddress + 6));
            }
            if (ChannelAttentions > 60) return;

            _log.WriteLine(
                "CA#{0}  iscp={1:X5} busy={2:X2}  scb={3:X5}  AS-READ status={4:X4} "
                + "command={5:X4} (cuc={6} ruc={7}) cbl={8:X4}   cmds={9}  int raised={11}/acked={10}",
                ChannelAttentions, _iscpAddress, _memory.ReadByte(_iscpAddress),
                _scbAddress, _lastStatusRead, _lastCommandRead,
                (_lastCommandRead >> 8) & 7, (_lastCommandRead >> 4) & 7,
                _lastCblRead, CommandsExecuted, InterruptAcknowledges, InterruptsRaised);
        }

        private System.IO.StreamWriter _log;
        private bool _logChecked;
        private ushort _lastStatusRead, _lastCommandRead, _lastCblRead;

        /// <summary>Bumped when the driver reads ClrENetIntr -- i.e. its ISR actually ran.</summary>
        public long InterruptAcknowledges;

        /// <summary>
        /// Read SCP -> ISCP -> SCB, then clear the ISCP busy flag and interrupt.  Drivers
        /// wait on that busy byte going to zero as the signal that the chip is alive.
        /// </summary>
        private void Initialise()
        {
            // SCP: sysbus byte at +0, ISCP address at +6 (little-endian, 24 bits used).
            int iscp = (_memory.ReadWord(ScpAddress + 6)
                        | (_memory.ReadWord(ScpAddress + 8) << 16)) & AddressMask;

            // ISCP: busy byte at +0, SCB offset at +2, SCB base at +4.
            int scbOffset = _memory.ReadWord(iscp + 2);
            _scbBase = (_memory.ReadWord(iscp + 4) | (_memory.ReadWord(iscp + 6) << 16))
                       & AddressMask;
            _iscpAddress = iscp;
            _scbAddress = (_scbBase + scbOffset) & AddressMask;

            _memory.WriteByte(iscp, 0);          // busy <- 0: initialisation done
            _initialised = true;

            // Report an idle command unit and interrupt, as the chip does after init.
            _memory.WriteWord(_scbAddress, StatCna);
            ScheduleInterrupt();
        }

        private void ExecuteScb()
        {
            ushort status = _memory.ReadWord(_scbAddress);
            ushort command = _memory.ReadWord(_scbAddress + 2);
            // Keep what the driver actually wrote: the command word is cleared below, so
            // reading it back for the trace would only ever show the zero we just stored.
            _lastStatusRead = status;
            _lastCommandRead = command;
            _lastCblRead = _memory.ReadWord(_scbAddress + 4);

            // Acknowledge: the driver sets the ACK bits for the status bits it has seen.
            status &= (ushort)~(command & StatAckMask);

            int cuc = (command >> 8) & 0x07;    // command unit control
            int ruc = (command >> 4) & 0x07;    // receive unit control

            if (cuc == 1)                        // CU start
            {
                RunCommandList(_memory.ReadWord(_scbAddress + 4));
                status |= (ushort)(StatCx | StatCna);
            }

            if (ruc == 1)                        // RU start
            {
                // Ready, but nothing will ever arrive: no frames are delivered, so the
                // driver's receive path simply stays quiet and its timeouts run normally.
                status = (ushort)((status & ~StatRusMask) | RusReady);
            }

            // Clearing the command word is how the chip says it has taken the command.
            // Leaving it set is precisely what makes a driver spin forever.
            _memory.WriteWord(_scbAddress + 2, 0);
            _memory.WriteWord(_scbAddress, status);
            ScheduleInterrupt();
        }

        /// <summary>
        /// Walk the command block list, marking each block complete.  Blocks are linked by a
        /// 16-bit offset from the SCB base; EL ends the list and S suspends it.
        /// </summary>
        private void RunCommandList(int firstOffset)
        {
            int offset = firstOffset;

            for (int guard = 0; guard < 256; guard++)
            {
                int cb = (_scbBase + offset) & AddressMask;
                ushort cmd = _memory.ReadWord(cb + 2);

                if (((cmd & 0x07)) == 4) FramesDropped++;   // Transmit

                // Complete and OK.  Every command this model supports is a no-op beyond
                // saying it happened -- Configure, IA-Setup and Transmit all just succeed.
                _memory.WriteWord(cb, (ushort)((CbComplete | CbOk) & ~CbBusy));
                CommandsExecuted++;

                if ((cmd & 0x8000) != 0) break;             // EL: end of list
                if ((cmd & 0x4000) != 0) break;             // S: suspend after this block

                int next = _memory.ReadWord(cb + 4);
                if (next == offset) break;                  // self-link: stop rather than spin
                offset = next;
            }
        }
    }
}
