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
    /// An Intel 8259A Programmable Interrupt Controller.  The Dove IOP has three
    /// external 8259As (master @ 0x00/0x02, slave @ 0x10/0x12, options @ 0x60/0x62;
    /// IOP-TR §3.2) cascaded per Fig 3.3, plus the 80186's internal PIC in iRMX mode.
    ///
    /// This models the ICW1-4 initialization sequence, OCW1 (IMR), OCW2 (EOI), and
    /// OCW3 (read-register select + poll), fixed-priority resolution (IR0 highest),
    /// and the INTA vector.  Even port (A0=0) = command; odd port (A0=1) = data.
    /// </summary>
    public class I8259
    {
        public I8259(string name) { _name = name; Reset(); }

        public void Reset()
        {
            _imr = 0xFF;   // all masked
            _irr = 0x00;
            _isr = 0x00;
            _initStep = 0;
            _readReg = 0;  // IRR
            _vectorBase = 0;
            _icw4Needed = false;
            _single = false;
            _autoEoi = false;
            _pollMode = false;
        }

        public string Name { get { return _name; } }
        public byte InterruptMask { get { return _imr; } }
        public byte VectorBase { get { return _vectorBase; } }

        // ---- Programmed I/O ----

        /// <summary>Write to the command port (A0=0): ICW1 / OCW2 / OCW3.</summary>
        public void WriteCommand(byte v)
        {
            if ((v & 0x10) != 0)
            {
                // ICW1: begin initialization.
                _icw4Needed = (v & 0x01) != 0;
                _single = (v & 0x02) != 0;
                _imr = 0x00;
                _isr = 0x00;
                _irr = 0x00;
                _readReg = 0;
                _initStep = 1;   // expect ICW2 next
            }
            else if ((v & 0x08) != 0)
            {
                // OCW3.
                if ((v & 0x02) != 0)
                {
                    _readReg = (v & 0x01);   // 0b10=read IRR, 0b11=read ISR
                }
                _pollMode = (v & 0x04) != 0;
            }
            else
            {
                // OCW2: EOI handling.
                if ((v & 0x20) != 0)
                {
                    if ((v & 0x40) != 0)
                    {
                        // Specific EOI.
                        _isr &= (byte)~(1 << (v & 0x07));
                    }
                    else
                    {
                        // Non-specific EOI: clear highest-priority in-service bit.
                        for (int i = 0; i < 8; i++)
                        {
                            if ((_isr & (1 << i)) != 0) { _isr &= (byte)~(1 << i); break; }
                        }
                    }
                }
            }
        }

        /// <summary>Write to the data port (A0=1): ICW2/3/4 during init, otherwise OCW1 (IMR).</summary>
        public void WriteData(byte v)
        {
            switch (_initStep)
            {
                case 1: // ICW2 = vector base (8086 mode: T7-T3)
                    _vectorBase = (byte)(v & 0xF8);
                    _initStep = _single ? (_icw4Needed ? 3 : 0) : 2;
                    break;
                case 2: // ICW3 = cascade wiring
                    _icw3 = v;
                    _initStep = _icw4Needed ? 3 : 0;
                    break;
                case 3: // ICW4
                    _autoEoi = (v & 0x02) != 0;
                    _initStep = 0;
                    break;
                default: // OCW1 = interrupt mask register
                    _imr = v;
                    break;
            }
        }

        /// <summary>Read the command port (A0=0): IRR or ISR per the last OCW3 select.</summary>
        public byte ReadCommand()
        {
            return _readReg == 1 ? _isr : _irr;
        }

        /// <summary>Read the data port (A0=1): the interrupt mask register.</summary>
        public byte ReadData()
        {
            return _imr;
        }

        /// <summary>Diagnostic accessors for the in-service and request registers.</summary>
        public byte InService { get { return _isr; } }
        public byte Request { get { return _irr; } }

        // ---- Interrupt line handling ----

        /// <summary>Assert an interrupt request line (edge/level both just set IRR here).</summary>
        public void RaiseIrq(int line) { _irr |= (byte)(1 << (line & 7)); }

        /// <summary>Diagnostic: force-clear a mask bit (used to test interrupt delivery).</summary>
        public void ForceUnmask(int line) { _imr &= (byte)~(1 << (line & 7)); }

        /// <summary>
        /// Acknowledge a cascade line auto-EOI: clear the request but do not latch an
        /// in-service bit.  Used for the 80186 internal PIC on master IR6, whose
        /// sub-controller is auto-EOI (the Opie dispatcher never EOIs it), so the
        /// master's cascade line must not stay in service and block the next tick.
        /// </summary>
        public void AcknowledgeCascade(int line) { _irr &= (byte)~(1 << (line & 7)); }

        /// <summary>Deassert a (level) interrupt request line.</summary>
        public void LowerIrq(int line) { _irr &= (byte)~(1 << (line & 7)); }

        /// <summary>
        /// True if an unmasked request of higher priority than anything in service is
        /// pending; if so <paramref name="line"/> is the winning IR line.
        /// </summary>
        public bool HasPending(out int line)
        {
            int ready = _irr & ~_imr;
            for (int i = 0; i < 8; i++)
            {
                int bit = 1 << i;
                if ((_isr & bit) != 0) break;      // a higher/equal priority already in service
                if ((ready & bit) != 0) { line = i; return true; }
            }
            line = -1;
            return false;
        }

        /// <summary>
        /// Acknowledge (INTA) the given line: move it from requesting to in-service and
        /// return its interrupt vector.  In auto-EOI mode the in-service bit is not set.
        /// </summary>
        public int Acknowledge(int line)
        {
            int bit = 1 << (line & 7);
            _irr &= (byte)~bit;
            if (!_autoEoi) _isr |= (byte)bit;
            return _vectorBase + (line & 7);
        }

        private readonly string _name;

        private byte _imr, _irr, _isr;
        private byte _vectorBase, _icw3;
        private int _initStep;    // 0=operational, 1=ICW2, 2=ICW3, 3=ICW4
        private int _readReg;     // 0=IRR, 1=ISR
        private bool _icw4Needed, _single, _autoEoi, _pollMode;
    }
}
