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
        /// <summary>
        /// Load a 128-byte 93C46 dump as 64 words, BIG-ENDIAN: the even byte is the word's HIGH
        /// half and the odd byte its LOW half.
        ///
        /// This was assembled the other way round, which byte-swapped every word the guest read.
        /// It went unnoticed because the Pilot 14 boot path never reads the config, but it is fatal
        /// to the Pilot 12 path: InitCPSpecific does %ReadEEProm(eePromMemSize) then AND AL, 0F0H --
        /// the LOW byte's high nibble -- so for byte 25 = 0x22 the correct word 0x0C22 yields
        /// csBankConfiguration 0x20, while the swapped 0x220C yields 0x00.  Zero makes CheckBlock
        /// discard every CP microcode block, so the writable control store stays empty, no CP Start
        /// block is ever acted on, RamBoot.ProcessCPBlock never drains the boot buffer, and
        /// RAMFlpBt's WaitForBuffer spins on bootDataEnd != bootDataStart forever.  One byte order,
        /// four symptoms.
        ///
        /// Big-endian is independently confirmed by the config checksum: read this way the trailing
        /// pair at words 62/63 are exact one's complements (0x5C25 + 0xA3DA = 0xFFFF) across all
        /// eight museum dumps, which does not hold under the swapped reading.
        /// </summary>
        public void Load(byte[] image)
        {
            for (int i = 0; i < 64; i++)
            {
                int hi = (i * 2) < image.Length ? image[i * 2] : 0xFF;
                int lo = (i * 2 + 1) < image.Length ? image[i * 2 + 1] : 0xFF;
                _mem[i] = (ushort)((hi << 8) | lo);
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
                _dataMode = false;
                _cmdBits = 0;
                _dataOut = false;
                _clock = false;
                _cs = false;
                return;      // _ready survives: the guest polls it across CS transitions

            }

            if (cs && !_cs)
            {
                // CS rising: new command.  _ready is deliberately NOT cleared here -- the guest
                // polls status by reasserting CS after a write, so clearing it on the rising edge
                // would make the ready bit unobservable and the spin unbreakable.  It clears when a
                // start bit actually arrives.
                _started = false;
                _readMode = false;
                _dataMode = false;
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
                else if (_dataMode)
                {
                    // WRITE / WRAL: 16 data bits follow the 8 command bits, MSB first.
                    _dataReg = ((_dataReg << 1) | (di ? 1 : 0)) & 0xFFFF;
                    if (++_dataBits == 16)
                    {
                        _dataMode = false;
                        if (_writeEnabled)
                        {
                            if (_pendingWral) { for (int a = 0; a < _mem.Length; a++) _mem[a] = (ushort)_dataReg; }
                            else _mem[_pendingAddr] = (ushort)_dataReg;
                            Dirty = true;
                        }
                        // READY.  Bit 11 is BOTH EEPReadDataMask and EEPStatusReady in HardDefs.asm
                        // -- one pin, two meanings by phase -- and EEPProcs.asm spins on it unbounded
                        // after a write, erase or reset (WriteLoop2 / EraseLoop / ResetLoop, no
                        // timeout).  Leaving it low is why installation utilities hung at MP 0199: a
                        // plain boot only READS the EEPROM, while an installer WRITES it to record the
                        // boot device and configuration.
                        _ready = true;
                        if (WriteLog != null && WriteLog.Count < 200)
                            WriteLog.Add((_pendingAddr << 16) | (_dataReg & 0xFFFF));
                    }
                }
                else if (!_started)
                {
                    if (di) { _started = true; _cmdBits = 0; _cmdReg = 0; _ready = false; }  // start bit
                }
                else
                {
                    _cmdReg = (_cmdReg << 1) | (di ? 1 : 0);
                    _cmdBits++;
                    if (_cmdBits == 8)   // 2 opcode bits + 6 address bits
                    {
                        int opcode = (_cmdReg >> 6) & 3;
                        int addr = _cmdReg & 0x3F;
                        switch (opcode)
                        {
                            case 2:   // READ (10)
                                _outWord = _mem[addr];
                                _readMode = true;
                                _outBit = 16;      // leading dummy bit; first read edge -> bit 15
                                _dataOut = false;
                                if (ReadLog != null && ReadLog.Count < 200) ReadLog.Add((addr << 16) | _outWord);
                                break;

                            case 1:   // WRITE (01) -- 16 data bits follow
                                _pendingAddr = addr; _pendingWral = false;
                                _dataMode = true; _dataBits = 0; _dataReg = 0;
                                break;

                            case 3:   // ERASE (11) -- set the word to all ones
                                if (_writeEnabled) { _mem[addr] = 0xFFFF; Dirty = true; }
                                _ready = true;
                                break;

                            case 0:   // special, selected by the top two address bits
                                switch ((addr >> 4) & 3)
                                {
                                    case 3: _writeEnabled = true; break;    // EWEN (cmd 30H)
                                    case 0: _writeEnabled = false; break;   // EWDS (cmd 00H)
                                    case 2:                                 // ERAL (cmd 20H)
                                        if (_writeEnabled)
                                        {
                                            for (int a = 0; a < _mem.Length; a++) _mem[a] = 0xFFFF;
                                            Dirty = true;
                                        }
                                        _ready = true;
                                        break;
                                    case 1:                                 // WRAL (cmd 10H)
                                        _pendingWral = true;
                                        _dataMode = true; _dataBits = 0; _dataReg = 0;
                                        break;
                                }
                                break;
                        }
                    }
                }
            }
            _clock = clock;
        }

        /// <summary>
        /// Input-port bit 11.  The pin carries READ DATA while a read shifts out and READY/BUSY
        /// status otherwise: EEPReadDataMask and EEPStatusReady are the same 0x0800 mask.
        /// </summary>
        public bool DataOut { get { return _readMode ? _dataOut : (_ready || _dataOut); } }

        /// <summary>True once a write or erase has modified the image, so the host can persist it.</summary>
        public bool Dirty;

        /// <summary>Diagnostic: completed WRITEs, each (address&lt;&lt;16)|word.</summary>
        public System.Collections.Generic.List<int> WriteLog;

        private bool _ready;          // status pin high = not busy
        private bool _writeEnabled;   // EWEN/EWDS latch
        private bool _dataMode;       // shifting the 16 data bits of a WRITE/WRAL
        private bool _pendingWral;
        private int _pendingAddr, _dataBits, _dataReg;

        /// <summary>Diagnostic: log of completed READs, each (address&lt;&lt;16)|word.</summary>
        public System.Collections.Generic.List<int> ReadLog;

        public ushort[] Words { get { return _mem; } }

        private readonly ushort[] _mem = new ushort[64];
        private bool _cs, _clock, _started, _readMode, _dataOut;
        private int _cmdReg, _cmdBits, _outWord, _outBit;
    }
}
