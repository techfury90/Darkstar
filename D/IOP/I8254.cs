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

namespace D.IOP
{
    /// <summary>
    /// An Intel 8254 Programmable Interval Timer -- the Dove IOP's external timer
    /// (three counters @ ports 0x20/0x22/0x24 with the control word @ 0x26; IOP-TR
    /// Table 2.3).  Models the control word, LSB/MSB read/write sequencing, the
    /// counter-latch and read-back (count + status) commands, and running counters
    /// (modes 0 and 3) driven by <see cref="Tick"/>.
    ///
    /// The counters are advanced by CPU-cycle ticks; the input frequency ratio is
    /// not modeled cycle-exactly, but the OUT pin toggles and the count decreases,
    /// which is what the boot-ROM timer test verifies (it polls the read-back status
    /// for the OUT bit to go low then high).
    /// </summary>
    public class I8254
    {
        public I8254()
        {
            for (int i = 0; i < 3; i++) _c[i] = new Counter();
        }

        /// <summary>Write the control word (port base+3).</summary>
        public void WriteControl(byte v)
        {
            int sc = (v >> 6) & 3;

            if (sc == 3)
            {
                // Read-back command.
                bool latchCount = (v & 0x20) == 0;   // CNT'=0 -> latch count
                bool latchStatus = (v & 0x10) == 0;  // STAT'=0 -> latch status
                for (int i = 0; i < 3; i++)
                {
                    if ((v & (2 << i)) != 0)
                    {
                        if (latchStatus && !_c[i].statusLatched)
                        {
                            _c[i].statusLatched = true;
                            _c[i].latchedStatus = BuildStatus(i);
                        }
                        if (latchCount && !_c[i].countLatched)
                        {
                            _c[i].countLatched = true;
                            _c[i].latchedCount = _c[i].count;
                        }
                    }
                }
                return;
            }

            Counter c = _c[sc];
            int rw = (v >> 4) & 3;
            if (rw == 0)
            {
                // Counter-latch command.
                if (!c.countLatched) { c.countLatched = true; c.latchedCount = c.count; }
            }
            else
            {
                c.rw = rw;
                c.mode = (v >> 1) & 7;
                c.bcd = (v & 1) != 0;
                c.writeState = 0;
                c.readState = 0;
                c.running = false;
                c.nullCount = true;
                // OUT initial state by mode: mode 0 -> low, others -> high.
                c.output = (c.mode != 0);
            }
        }

        /// <summary>Write a counter's count register (port base+counter).</summary>
        public void WriteCounter(int counter, byte v)
        {
            Counter c = _c[counter];
            switch (c.rw)
            {
                case 1: c.reload = (c.reload & 0xFF00) | v; LoadCounter(c); break;         // LSB only
                case 2: c.reload = (c.reload & 0x00FF) | (v << 8); LoadCounter(c); break;  // MSB only
                case 3:
                    if (c.writeState == 0) { c.reload = (c.reload & 0xFF00) | v; c.writeState = 1; }
                    else { c.reload = (c.reload & 0x00FF) | (v << 8); c.writeState = 0; LoadCounter(c); }
                    break;
            }
        }

        /// <summary>Read a counter (port base+counter): latched status, latched count, or live count.</summary>
        public byte ReadCounter(int counter)
        {
            Counter c = _c[counter];

            if (c.statusLatched)
            {
                c.statusLatched = false;
                return c.latchedStatus;
            }

            int val = c.countLatched ? c.latchedCount : c.count;
            switch (c.rw)
            {
                case 1: c.countLatched = false; return (byte)val;
                case 2: c.countLatched = false; return (byte)(val >> 8);
                default: // LSB then MSB
                    if (c.readState == 0) { c.readState = 1; return (byte)val; }
                    c.readState = 0; c.countLatched = false; return (byte)(val >> 8);
            }
        }

        /// <summary>Advance the running counters by the given number of input clocks.</summary>
        public void Tick(int clocks)
        {
            if (clocks <= 0) return;
            for (int i = 0; i < 3; i++)
            {
                Counter c = _c[i];
                if (!c.running) continue;
                int period = c.reload == 0 ? 0x10000 : c.reload;
                c.phase = (c.phase + clocks) % period;
                c.count = period - c.phase;
                if (c.mode == 3)
                {
                    c.output = c.phase < ((period + 1) / 2);
                }
                else // treat other modes as: OUT asserts when the count expires
                {
                    c.output = c.count == 0;
                }
            }
        }

        /// <summary>Current OUT pin state of a counter (Timer 2's OUT gates the speaker, etc.).</summary>
        public bool Output(int counter) { return _c[counter].output; }

        private void LoadCounter(Counter c)
        {
            c.count = c.reload == 0 ? 0x10000 : c.reload;
            c.phase = 0;
            c.running = true;
            c.nullCount = false;
            c.output = (c.mode != 0);
        }

        private byte BuildStatus(int i)
        {
            Counter c = _c[i];
            return (byte)((c.output ? 0x80 : 0) |
                          (c.nullCount ? 0x40 : 0) |
                          (c.rw << 4) |
                          (c.mode << 1) |
                          (c.bcd ? 1 : 0));
        }

        private class Counter
        {
            public int count;
            public int reload;
            public int phase;
            public int mode;
            public int rw = 3;
            public bool bcd;
            public bool output;
            public bool running;
            public bool nullCount;
            public int writeState;
            public int readState;
            public bool countLatched;
            public int latchedCount;
            public bool statusLatched;
            public byte latchedStatus;
        }

        private readonly Counter[] _c = new Counter[3];
    }
}
