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
    /// A 93C46 serial configuration EEPROM (64 x 16-bit) -- the Dove IOP's per-machine
    /// config store (U128: machine serial, memory size, CS-bank count, default boot
    /// device).  Bit-banged by the IOP through the Config register (write port 0xD0,
    /// WriteConfigReg): bit12 = CS/enable, bit13 = clock, bit15 = data-in (DI).  The
    /// data-out (DO) appears on the Input Port (read port 0x80) bit 11.
    ///
    /// Only the READ command (start + opcode 10 + 6-bit address -> 16 bits out,
    /// MSB first, with a leading dummy bit) is needed for boot; writes/erase are
    /// accepted and ignored (the bootstrap never programs it).  The boot ROM's
    /// preboot validates this EEPROM; a bad checksum forces the "dontTrustEEProm"
    /// path (WaitForF9, no display), so a valid image must be loaded.
    /// </summary>
    public class I93C46
    {
        private const int CsBit = 0x1000;    // WriteConfigReg bit 12
        private const int ClockBit = 0x2000; // bit 13
        private const int DataInBit = 0x8000;// bit 15
        public const int DataOutMask = 0x0800; // Input Port bit 11

        public I93C46()
        {
            for (int i = 0; i < _mem.Length; i++) _mem[i] = 0xFFFF;
        }

        /// <summary>Load a 128-byte (64-word, little-endian) image (e.g. a U128 dump).</summary>
        public void Load(byte[] image)
        {
            for (int i = 0; i < 64; i++)
            {
                int lo = (i * 2) < image.Length ? image[i * 2] : 0xFF;
                int hi = (i * 2 + 1) < image.Length ? image[i * 2 + 1] : 0xFF;
                _mem[i] = (ushort)(lo | (hi << 8));
            }
        }

        public ushort ReadWord(int address) { return _mem[address & 0x3F]; }

        /// <summary>Process a WriteConfigReg (port 0xD0) value: CS/clock/DI.</summary>
        public void WriteConfig(int value)
        {
            bool cs = (value & CsBit) != 0;
            bool clock = (value & ClockBit) != 0;
            bool di = (value & DataInBit) != 0;

            if (!cs)
            {
                // Chip deselected: reset the serial state machine.
                _started = false;
                _readMode = false;
                _cmdBits = 0;
                _dataOut = false;
                _clock = false;
                _cs = false;
                return;
            }

            if (cs && !_cs)
            {
                // CS rising: new command.
                _started = false;
                _readMode = false;
                _cmdBits = 0;
                _cmdReg = 0;
                _dataOut = false;
            }
            _cs = cs;

            if (clock && !_clock)
            {
                // Rising clock edge.
                if (_readMode)
                {
                    _outBit--;
                    _dataOut = (_outBit >= 0 && _outBit < 16) && ((_outWord >> _outBit) & 1) != 0;
                }
                else if (!_started)
                {
                    if (di) { _started = true; _cmdBits = 0; _cmdReg = 0; }  // start bit
                }
                else
                {
                    _cmdReg = (_cmdReg << 1) | (di ? 1 : 0);
                    _cmdBits++;
                    if (_cmdBits == 8)   // 2 opcode bits + 6 address bits
                    {
                        int opcode = (_cmdReg >> 6) & 3;
                        int addr = _cmdReg & 0x3F;
                        if (opcode == 2)  // READ (binary 10)
                        {
                            _outWord = _mem[addr];
                            _readMode = true;
                            _outBit = 16;      // leading dummy bit; first read edge -> bit 15
                            _dataOut = false;
                            if (ReadLog != null && ReadLog.Count < 200) ReadLog.Add((addr << 16) | _outWord);
                        }
                        // opcode 0 = special (EWEN/EWDS/ERAL/WRAL), 1 = WRITE, 3 = ERASE:
                        // accepted and ignored (bootstrap is read-only).
                    }
                }
            }
            _clock = clock;
        }

        /// <summary>Current data-out bit, ready to be OR'd into Input Port bit 11.</summary>
        public bool DataOut { get { return _dataOut; } }

        /// <summary>Diagnostic: log of completed READs, each (address&lt;&lt;16)|word.</summary>
        public System.Collections.Generic.List<int> ReadLog;

        public ushort[] Words { get { return _mem; } }

        private readonly ushort[] _mem = new ushort[64];
        private bool _cs, _clock, _started, _readMode, _dataOut;
        private int _cmdReg, _cmdBits, _outWord, _outBit;
    }
}
