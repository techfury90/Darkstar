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
    /// An Intel 8251A USART -- the Dove IOP's keyboard/mouse serial link (data @
    /// port 0x30, status/command @ 0x32; IOP-TR §9).  Models the mode/command
    /// programming sequence and the TxRDY/RxRDY/TxEMPTY/OE/DSR status bits.
    ///
    /// The boot-ROM self-test (ROMBoot @ FF81C) relies on the keyboard interface
    /// looping transmitted data straight back to the receiver (the 8251 itself has
    /// no loopback bit; the keyboard cable/test hardware provides it): it writes a
    /// byte and expects RxRDY to appear, then writes a second unread byte and
    /// expects an overrun.  So this models internal loopback (Tx -> Rx) and overrun.
    /// Loopback is a self-test artifact; real received keystrokes will drive Rx
    /// directly once the keyboard front-end exists.
    /// </summary>
    public class I8251
    {
        // Status register bits.
        private const int TxRdy = 0x01;
        private const int RxRdy = 0x02;
        private const int TxEmpty = 0x04;
        private const int ParityError = 0x08;
        private const int OverrunError = 0x10;
        private const int FramingError = 0x20;
        private const int SynBrkDet = 0x40;
        private const int Dsr = 0x80;

        public I8251() { Reset(); }

        /// <summary>External reset (the IOP holds the UART in reset via the Reset-Control register).</summary>
        public void Reset()
        {
            _expectMode = true;
            _rxReady = false;
            _overrun = false;
            _rxData = 0;
        }

        /// <summary>Write the control port (0x32): a mode word after reset, then command words.</summary>
        public void WriteControl(byte v)
        {
            if (_expectMode)
            {
                _mode = v;
                // Async modes (baud factor != 0) take no sync characters; the next
                // control write is the command.  (Sync mode is not used by the keyboard link.)
                _expectMode = false;
                return;
            }

            _command = v;
            if ((v & 0x40) != 0)   // IR = internal reset -> back to expecting a mode word
            {
                Reset();
                return;
            }
            if ((v & 0x10) != 0)   // ER = error reset
            {
                _overrun = false;
            }
            _txEnabled = (v & 0x01) != 0;
            _rxEnabled = (v & 0x04) != 0;
        }

        /// <summary>Write the data port (0x30): transmit a byte (looped back to the receiver).</summary>
        public void WriteData(byte v)
        {
            // Loopback: the transmitted byte is immediately received into the
            // single-byte receive buffer.
            if (_rxReady) _overrun = true;  // previous byte not yet read -> overrun
            _rxData = v;
            _rxReady = true;
        }

        /// <summary>
        /// Deliver a byte actually received from the keyboard (as opposed to the
        /// POST loopback in WriteData): latches it into the Rx buffer and asserts
        /// RxRDY, which the IOP wiring turns into KbrdInputReq -> master 8259 IR3.
        /// The boot KEYMO handler reads it raw and stores it at HexValue (0x3DB2).
        /// </summary>
        public void InjectRx(byte v)
        {
            if (_rxReady) _overrun = true;
            _rxData = v;
            _rxReady = true;
        }

        /// <summary>Read the data port (0x30): the received byte; clears RxRDY.</summary>
        public byte ReadData()
        {
            _rxReady = false;
            return _rxData;
        }

        /// <summary>Read the status port (0x32).</summary>
        public byte ReadStatus()
        {
            int s = Dsr | TxEmpty | TxRdy;    // DSR asserted, transmitter idle/ready
            if (_rxReady) s |= RxRdy;
            if (_overrun) s |= OverrunError;
            return (byte)s;
        }

        public byte Mode { get { return _mode; } }
        public byte Command { get { return _command; } }

        /// <summary>True when a received byte is waiting (drives KbrdInputReq -> master 8259 IR3).</summary>
        public bool RxReady { get { return _rxReady; } }

        private bool _expectMode;
        private byte _mode, _command;
        private bool _txEnabled, _rxEnabled;
        private byte _rxData;
        private bool _rxReady;
        private bool _overrun;
    }
}
