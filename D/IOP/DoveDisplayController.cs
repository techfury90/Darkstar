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

namespace D.IOP
{
    /// <summary>
    /// The Dove (Xerox 6085 / Daybreak) display controller -- the DCM/DDC chip
    /// (Daybreak Tech Ref App A), programmed by the IOP through the mono I/O window
    /// 0xE800-0xEFFF.  Unlike the DLion (whose display is CP-microcode-driven), the
    /// Dove display is entirely IOP-driven and autonomous: the IOP sets the control
    /// register, quadwords-per-line, bitmap origin, border and hardware cursor, and
    /// the DMC scans the mono bitmap out of the low memory bank on its own.
    ///
    /// This models the register file and the scan-out (mono bitmap fetch + hardware
    /// cursor sprite compositing) into a 1-bpp framebuffer.  The boot/MP status code
    /// is drawn by the ROM into the 16x16 cursor sprite, so the cursor path is
    /// modeled from the start (App A 4.5).
    /// </summary>
    public class DoveDisplayController
    {
        public const int WindowBase = 0xE800;
        public const int WindowEnd = 0xEFFF;

        // Register offsets within the window (App A 4.0.x).
        private const int VertCsBase = 0xE800;   // E800-EBFF
        private const int HorzCsBase = 0xEC00;   // EC00-EC7F
        private const int RegControl = 0xEC80;   // display control (W) / status (R)
        private const int RegBorderLo = 0xEC81;
        private const int RegBorderHi = 0xEC82;
        private const int RegCursorXHi = 0xEC83;
        private const int RegCursorXLo = 0xEC84;  // bits3-0 offset, bit7 NCURSOR
        private const int RegCursorYLo = 0xEC85;
        private const int RegCursorYHi = 0xEC86;
        private const int RegQuadwords = 0xEC88;
        private const int RegBitmapLo = 0xEC89;
        private const int RegBitmapHi = 0xEC8A;
        private const int RegTypeSize = 0xECCC;   // controller-type / CRT-size status
        private const int CursorBufBase = 0xED00; // ED00-ED1F (32 bytes)
        private const int SoftResetBase = 0xED60; // ED60-ED7F

        public DoveDisplayController(IPhysicalMemory memory)
        {
            _memory = memory;
            Reset();
        }

        public void Reset()
        {
            for (int i = 0; i < _reg.Length; i++) _reg[i] = 0;
        }

        public bool Handles(int port)
        {
            return port >= WindowBase && port <= WindowEnd;
        }

        // ---- Programmed I/O ----

        public void WriteByte(int port, byte value)
        {
            if (port >= SoftResetBase && port <= WindowEnd)
            {
                // Any write to ED60-ED7F is a soft reset (aligns to first scan line);
                // registers keep their values on a soft reset.
                return;
            }

            if (OnCursorWrite != null && port >= CursorBufBase && port < CursorBufBase + 32)
                OnCursorWrite(port, value);

            _reg[port - WindowBase] = value;

            if (_traceRegisters && IsInterestingRegister(port))
            {
                _regLog.Add(String.Format("{0:X4} <- {1:X2}", port, value));
            }
        }

        public byte ReadByte(int port)
        {
            switch (port)
            {
                case RegTypeSize:
                    // Controller-type / CRT-size status.  bit1: 1 = gate-array DCM,
                    // 0 = CMOS DCM/DDC (App A diff #10).  bit0: 0 = 19", 1 = 15".
                    // _typeSize = 0x00 => CMOS controller + 19" CRT.
                    TypeSizeReads++;
                    return _typeSize;

                case RegControl:  // DDC status (gate-array only); benign constant
                    return 0x00;
                case RegCursorXLo:  // DCC status; benign constant
                    return 0x00;
                case RegQuadwords:  // DMC status; benign constant
                    return 0x00;

                default:
                    return _reg[port - WindowBase];
            }
        }

        // ---- Decoded state ----

        public bool VideoEnabled { get { return (_reg[RegControl - WindowBase] & 0x02) != 0; } }
        public bool NonInterlace { get { return (_reg[RegControl - WindowBase] & 0x01) != 0; } }
        public bool ForcePicture { get { return (_reg[RegControl - WindowBase] & 0x08) != 0; } }
        public int MixFunction { get { return (_reg[RegControl - WindowBase] >> 4) & 0x0F; } }
        public int QuadwordsPerLine { get { return _reg[RegQuadwords - WindowBase] & 0x3F; } }
        public int BorderLow { get { return _reg[RegBorderLo - WindowBase]; } }
        public int BorderHigh { get { return _reg[RegBorderHi - WindowBase]; } }

        /// <summary>Cursor word position (X, in 16-px words).</summary>
        public int CursorWord { get { return _reg[RegCursorXHi - WindowBase]; } }
        /// <summary>Cursor intra-word bit offset (0-15).</summary>
        public int CursorBitOffset { get { return _reg[RegCursorXLo - WindowBase] & 0x0F; } }
        /// <summary>NCURSOR: false = cursor enabled, true = disabled.</summary>
        public bool CursorDisabled { get { return (_reg[RegCursorXLo - WindowBase] & 0x80) != 0; } }
        public int CursorLine
        {
            get { return (_reg[RegCursorYLo - WindowBase]) | ((_reg[RegCursorYHi - WindowBase] & 0x03) << 8); }
        }

        /// <summary>
        /// The bitmap start word address in the low memory bank.  Registers supply
        /// CA05-CA20 (quadword-aligned); the exact 80186-side shift is calibrated
        /// against the firmware (see BitmapOriginShift).
        /// </summary>
        public int BitmapStartWord
        {
            get
            {
                int reg = (_reg[RegBitmapHi - WindowBase] << 8) | _reg[RegBitmapLo - WindowBase];
                return reg << BitmapOriginShift;
            }
        }

        /// <summary>
        /// Left-shift applied to the (EC8A:EC89) origin register to form the word
        /// address.  Tunable during bring-up until scan-out lines up with where the
        /// firmware draws.  Registers map CA05-CA20, so 5 is the nominal value.
        /// </summary>
        public int BitmapOriginShift = 5;

        public int Width { get; private set; }
        public int Height { get; private set; }

        /// <summary>
        /// 1-bpp framebuffer of the last RenderMono(): one byte per pixel (0/1), the
        /// raw bitmap bit.  VIDEO POLARITY: a set bit (1) is WHITE (lit), a clear bit
        /// (0) is BLACK — verified against real boot VRAM (icon-box interior = 0xFFFF
        /// renders white, glyphs = 0x0000 render black, desktopGray 0xBBBB = light
        /// grey).  A surface blitting this must map 1 -> white, 0 -> black.
        /// </summary>
        public byte[] Frame { get { return _frame; } }

        // ---- Scan-out ----

        /// <summary>
        /// Render one mono field from the bitmap in memory plus the hardware cursor
        /// sprite, into <see cref="Frame"/>.  Picture area only (no border frame).
        /// </summary>
        public void RenderMono()
        {
            int quadwords = QuadwordsPerLine;
            if (quadwords <= 0) quadwords = 13;         // default 15"
            int wordsPerLine = quadwords * 4;
            int width = wordsPerLine * 16;
            // Line count follows the monitor: 19" large-format bitmap is 861 lines
            // (17-18 quadwords/line), 15" is 633 (13 quadwords).  Per the Daybreak
            // DDC reference: 15" = 13 qw x 633, 19" = 17 qw x 861.
            int height = _displayLines > 0 ? _displayLines
                       : (quadwords >= 16 ? 861 : 633);

            Width = width;
            Height = height;
            if (_frame == null || _frame.Length != width * height) _frame = new byte[width * height];

            int originByte = BitmapStartWord * 2;
            int lineBytes = wordsPerLine * 2;

            for (int y = 0; y < height; y++)
            {
                int rowBase = originByte + y * lineBytes;
                int px = y * width;
                for (int w = 0; w < wordsPerLine; w++)
                {
                    int word = DisplayReader != null ? DisplayReader(rowBase + w * 2)
                                                     : _memory.ReadWord(rowBase + w * 2);
                    // The bitmap is byte-sequential with MSB = leftmost pixel: the byte
                    // at the lower address supplies pixels 0-7, the next byte pixels
                    // 8-15.  Since the word was read little-endian, its LOW byte is the
                    // lower-address (leftmost) byte -- emit low byte then high byte,
                    // each MSB-first.  (Reading it as one word with bit15 leftmost would
                    // byte-swap every pixel pair and scramble structured content.)
                    int lo = word & 0xFF, hi = (word >> 8) & 0xFF;
                    for (int b = 0; b < 8; b++) _frame[px++] = (byte)((lo >> (7 - b)) & 1);
                    for (int b = 0; b < 8; b++) _frame[px++] = (byte)((hi >> (7 - b)) & 1);
                }
            }

            CompositeCursor(width, height);
        }

        /// <summary>The 32-byte hardware cursor sprite buffer (ED00-ED1F), 16x16 1bpp.</summary>
        public byte[] GetCursorBuffer()
        {
            byte[] b = new byte[32];
            for (int i = 0; i < 32; i++) b[i] = _reg[CursorBufBase - WindowBase + i];
            return b;
        }

        private void CompositeCursor(int width, int height)
        {
            if (CursorDisabled) return;

            int cx = CursorWord * 16 + CursorBitOffset;
            int cy = CursorLine;
            int mix = MixFunction;

            for (int row = 0; row < 16; row++)
            {
                int y = cy + row;
                if (y < 0 || y >= height) continue;

                // Cursor buffer: Intel byte format, 2 bytes per 16-px row.  ED00 even
                // byte = CURD.00 (Mesa MSB)..07; ED01 odd byte = CURD.08..15 (LSB).
                int hi = _reg[CursorBufBase - WindowBase + row * 2];
                int lo = _reg[CursorBufBase - WindowBase + row * 2 + 1];
                int pattern = (hi << 8) | lo;   // Mesa word, MSB = leftmost

                for (int col = 0; col < 16; col++)
                {
                    int x = cx + col;
                    if (x < 0 || x >= width) continue;
                    int c = (pattern >> (15 - col)) & 1;
                    int idx = y * width + x;
                    int v = _frame[idx];
                    _frame[idx] = (byte)MixPixel(mix, v, c);
                }
            }
        }

        // 4-bit cursor/data mixing functions (App A Fig 4.0), V = bitmap, C = cursor.
        private static int MixPixel(int f, int v, int c)
        {
            switch (f & 0x0F)
            {
                case 0x0: return 0;
                case 0x1: return v & (c ^ 1);
                case 0x2: return v & c;
                case 0x3: return v ^ 1;
                case 0x4: return v & (c ^ 1);
                case 0x5: return c ^ 1;
                case 0x6: return v ^ c;
                case 0x7: return v | (c ^ 1);
                case 0x8: return v & c;
                case 0x9: return v ^ c;
                case 0xA: return c;
                case 0xB: return (v ^ 1) | c;
                case 0xC: return v;
                case 0xD: return v | (c ^ 1);
                case 0xE: return v | c;
                default: return 1;
            }
        }

        // ---- Configuration / instrumentation ----

        public void SetCrtLines(int lines) { _displayLines = lines; }
        public byte TypeSize { get { return _typeSize; } set { _typeSize = value; } }
        public bool TraceRegisters { get { return _traceRegisters; } set { _traceRegisters = value; } }
        public List<string> RegisterLog { get { return _regLog; } }

        private bool IsInterestingRegister(int port)
        {
            // The control/geometry/cursor-position registers, not the bulk CS / cursor
            // buffer writes.
            return (port >= RegControl && port <= RegBitmapHi) || port == RegTypeSize;
        }

        /// <summary>
        /// Optional scan-out source for the bitmap.  When set (by DoveIOPIO to the
        /// system-DRAM reader), the DMA reads display memory directly rather than
        /// through the 80186 address space.  Falls back to _memory when null.
        /// </summary>
        public System.Func<int, ushort> DisplayReader;

        /// <summary>Diagnostic hook fired on each write to the cursor-pattern buffer (ED00-1F).</summary>
        public System.Action<int, byte> OnCursorWrite;

        private readonly IPhysicalMemory _memory;
        private readonly byte[] _reg = new byte[0x800];
        private byte[] _frame;
        private int _displayLines = 0;   // 0 = auto (derive from quadwords/monitor)
        private byte _typeSize = 0x00;     // CMOS DCM/DDC (bit1=0), 19" CRT (bit0=0)
        public long TypeSizeReads = 0;     // TEMP: how many times firmware reads 0xECCC (monitor-size strap)
        private bool _traceRegisters;
        private readonly List<string> _regLog = new List<string>();
    }
}
