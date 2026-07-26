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
            ChannelAttentions = 0;
            CommandsExecuted = 0;
            FramesDropped = 0;
        }

        // ---- Diagnostics ----
        public long ChannelAttentions { get; private set; }
        public long CommandsExecuted { get; private set; }
        public long FramesDropped { get; private set; }
        public int ScbAddress { get { return _scbAddress; } }
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
        }

        /// <summary>
        /// Read SCP -> ISCP -> SCB, then clear the ISCP busy flag and interrupt.  Drivers
        /// wait on that busy byte going to zero as the signal that the chip is alive.
        /// </summary>
        private void Initialise()
        {
            // SCP: sysbus byte at +0, ISCP address at +6 (little-endian, 24 bits used).
            int iscp = _memory.ReadWord(ScpAddress + 6)
                     | (_memory.ReadWord(ScpAddress + 8) << 16);
            iscp &= 0xFFFFFF;

            // ISCP: busy byte at +0, SCB offset at +2, SCB base at +4.
            int scbOffset = _memory.ReadWord(iscp + 2);
            int scbBase = (_memory.ReadWord(iscp + 4) | (_memory.ReadWord(iscp + 6) << 16))
                          & 0xFFFFFF;
            _scbAddress = (scbBase + scbOffset) & 0xFFFFF;

            _memory.WriteByte(iscp, 0);          // busy <- 0: initialisation done
            _initialised = true;

            // Report an idle command unit and interrupt, as the chip does after init.
            _memory.WriteWord(_scbAddress, StatCna);
            _raiseInterrupt();
        }

        private void ExecuteScb()
        {
            ushort status = _memory.ReadWord(_scbAddress);
            ushort command = _memory.ReadWord(_scbAddress + 2);

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
            _raiseInterrupt();
        }

        /// <summary>
        /// Walk the command block list, marking each block complete.  Blocks are linked by a
        /// 16-bit offset from the SCB base; EL ends the list and S suspends it.
        /// </summary>
        private void RunCommandList(int firstOffset)
        {
            int scbBase = _scbAddress - (_scbAddress & 0xFFFF);
            int offset = firstOffset;

            for (int guard = 0; guard < 256; guard++)
            {
                int cb = (scbBase + offset) & 0xFFFFF;
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
