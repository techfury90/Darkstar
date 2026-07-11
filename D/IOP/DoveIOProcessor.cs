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
    /// The Dove (Xerox 6085 / Daybreak) I/O processor: the 80186 CPU plus its
    /// memory bus (EPROM/SRAM/map window), integrated Peripheral Control Block
    /// (timers, interrupt controller, chip-selects), and the I/O-bus peripheral
    /// chips (8259 x3, 8254, 8251, host-address PROM, ...), wired together with the
    /// interrupt-acknowledge / heartbeat path.
    ///
    /// This is the 80186 counterpart to the DLion's <see cref="IOProcessor"/>.  It
    /// runs the real Opie boot firmware from the boot ROM.
    /// </summary>
    public class DoveIOProcessor : IIOProcessor
    {
        public DoveIOProcessor(string bootRomPath)
        {
            _memory = new DoveIOPMemory(bootRomPath);
            _io = new DoveIOPIO(_memory);
            _pcb = new I80186Pcb();
            _cpu = new i80186(_memory, _io, _pcb);

            // Wire the interrupt-acknowledge / heartbeat path: the I/O bus resolves a
            // pending interrupt through the master 8259 and its cascaded slaves
            // (including the 80186's integrated PIC on IR6), and the CPU vectors to it.
            _io.SetPcb(_pcb);
            _cpu.InterruptAcknowledge = _io.AcknowledgeInterrupt;

            // Seed the Ethernet host-address (MAC) PROM from the configured HostID so
            // the boot ROM's checksum self-test passes with the machine's real address.
            LoadHostProm();

            Reset();
        }

        public void Reset()
        {
            _cpu.Reset();
        }

        /// <summary>Runs one 80186 instruction and advances the timers; returns clocks consumed.</summary>
        public int Execute()
        {
            int clocks = _cpu.Execute();
            _io.Tick(clocks);
            return clocks;
        }

        public i80186 CPU { get { return _cpu; } }
        public DoveIOPMemory Memory { get { return _memory; } }
        public DoveIOPIO IO { get { return _io; } }
        public I80186Pcb Pcb { get { return _pcb; } }

        private void LoadHostProm()
        {
            byte[] mac = new byte[6];
            for (int i = 0; i < 6; i++)
            {
                mac[i] = (byte)(Configuration.HostID >> ((5 - i) * 8));
            }
            _io.LoadHostProm(mac);
        }

        private readonly i80186 _cpu;
        private readonly DoveIOPMemory _memory;
        private readonly DoveIOPIO _io;
        private readonly I80186Pcb _pcb;
    }
}
