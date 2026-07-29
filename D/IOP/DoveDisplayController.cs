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

            // Border/control writes as they HAPPEN.  The close-time dump describes one instant, and
            // the instant being judged on screen is usually a different one: at Set Time the border
            // registers read 00/00, where phase cannot matter, while under ViewPoint they hold a
            // real 22/88 stipple.  Without a history there is no way to tell which values were live
            // when a screenshot was taken.
            if (BorderLog != null && BorderLog.Count < 400 &&
                (port == RegBorderLo || port == RegBorderHi || port == RegControl))
            {
                string name = port == RegBorderLo ? "borderLo"
                            : port == RegBorderHi ? "borderHi" : "EC80";
                BorderLog.Add(name + "=" + value.ToString("X2")
                    + "   (lo=" + BorderLow.ToString("X2") + " hi=" + BorderHigh.ToString("X2")
                    + " mix=" + MixFunction.ToString("X") + " nintl=" + (NonInterlace ? 1 : 0) + ")");
            }

            // BORDER PATTERN IS LATCHED BY WRITE ORDER, NOT BY ADDRESS.
            //
            // App A software difference #4 says the two chip generations take the border registers
            // in opposite orders -- gate-array low,high,low,high and standard-cell high,low,high,low
            // -- and gives the remedy as "swap the programmed values".  That only means anything if
            // the chip latches the pair alternately as they arrive rather than by which address was
            // written, so decoding by address makes one guest correct and the other swapped.
            //
            // Measured, both from the border/control write history:
            //   boot ROM   borderLo=BB then borderHi=EE   (gate-array order)
            //   ViewPoint  borderHi=88 then borderLo=22   (standard-cell order, repeatedly)
            // The FIRST-written value is the top pair in both.  No single phase constant can
            // satisfy both, which is why three successive phase theories -- vertical control-store
            // geometry, interlace, and a fixed swap -- each fixed one screen and broke the other.
            if (port == RegBorderLo || port == RegBorderHi)
            {
                if (_borderWriteToggle == 0) _borderTop = value; else _borderBottom = value;
                _borderWriteToggle ^= 1;
            }

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

        /// <summary>Raw display control register (EC80), so callers can watch it change.</summary>
        public byte ControlRegister { get { return _reg[RegControl - WindowBase]; } }

        /// <summary>
        /// One-line summary of the state that decides how the cursor is drawn.  The MP sprite
        /// currently renders white-on-black where the panel should be black-on-white, and that is
        /// entirely a function of the mix: mix A is "C" (cursor bit -> lit) while mix 5 is "C'"
        /// (inverted).  Rather than guess which the ROM programs, read it.
        /// </summary>
        /// <summary>
        /// Run-length summary of the VERTICAL control store (E800-EBFF, one nibble per scan line:
        /// b0 VSYNC, b1 VBLANK, b2 VPIC picture/border', b3 EOF).
        ///
        /// The border is NOT a fixed 32 lines -- TechRef 3.1.1 says the border size is "set by
        /// microcode", and the DDC reference warns the counts are guest-dependent.  Observed: the
        /// border phase that lines up under the boot ROM does not line up under ViewPoint, which is
        /// exactly that warning coming true.  So read the geometry rather than assume it.
        /// </summary>
        /// <summary>Set to a list to record border/control-register writes as they happen.</summary>
        public System.Collections.Generic.List<string> BorderLog;

        public string DescribeVerticalCs()
        {
            var sb = new System.Text.StringBuilder("vertical CS runs (line: code xN): ");
            int n = 0, runStart = 0, prev = -1, emitted = 0;
            for (int line = 0; line <= 1024; line++)
            {
                int code = line < 1024 ? (_reg[VertCsBase - WindowBase + line] & 0x0F) : -2;
                if (code != prev)
                {
                    if (prev >= 0 && emitted < 24)
                    {
                        sb.Append(runStart + ":" + prev.ToString("X") + " x" + (line - runStart) + "  ");
                        emitted++;
                    }
                    runStart = line; prev = code;
                }
                if (code >= 0) n++;
            }
            sb.Append("| nonzero entries=" + NonZeroVertCs());
            return sb.ToString();
        }

        private int NonZeroVertCs()
        {
            int n = 0;
            for (int i = 0; i < 1024; i++) if (_reg[VertCsBase - WindowBase + i] != 0) n++;
            return n;
        }

        /// <summary>
        /// Top border height in lines, taken from the vertical control store when the guest has
        /// programmed one: the leading run of visible, non-picture lines (VBLANK=0, VPIC=0).
        /// Falls back to the nominal 32 when the CS is empty (nothing has programmed it yet).
        /// </summary>
        public int TopBorderLines
        {
            get
            {
                if (NonZeroVertCs() == 0) return BorderPixels;
                int lines = 0;
                for (int i = 0; i < 1024; i++)
                {
                    int code = _reg[VertCsBase - WindowBase + i] & 0x0F;
                    if ((code & 0x02) != 0) continue;        // blanked -- not visible
                    if ((code & 0x04) != 0) break;           // first picture line
                    lines++;
                }
                return lines > 0 && lines < 256 ? lines : BorderPixels;
            }
        }

        public string DescribeCursorState()
        {
            return "EC80=" + ControlRegister.ToString("X2")
                 + " mix=" + MixFunction.ToString("X")
                 + " video=" + (VideoEnabled ? 1 : 0)
                 + " ncursor=" + (CursorDisabled ? 1 : 0)
                 + " rasterX=" + (CursorWord * 16 + CursorBitOffset)
                 + " rasterY=" + CursorLine
                 + " -> visible " + CursorVisibleX + "," + CursorVisibleY
                 + "  border=" + BorderLow.ToString("X2") + "/" + BorderHigh.ToString("X2")
                 + "  qw=" + QuadwordsPerLine;
        }

        public bool VideoEnabled { get { return (_reg[RegControl - WindowBase] & 0x02) != 0; } }
        public bool NonInterlace { get { return (_reg[RegControl - WindowBase] & 0x01) != 0; } }
        public bool ForcePicture { get { return (_reg[RegControl - WindowBase] & 0x08) != 0; } }
        public int MixFunction { get { return (_reg[RegControl - WindowBase] >> 4) & 0x0F; } }
        public int QuadwordsPerLine { get { return _reg[RegQuadwords - WindowBase] & 0x3F; } }
        public int BorderLow { get { return _reg[RegBorderLo - WindowBase]; } }
        public int BorderHigh { get { return _reg[RegBorderHi - WindowBase]; } }

        /// <summary>Border pattern for the first line-pair, i.e. the first of the pair written.</summary>
        public int BorderTop { get { return _borderTop; } }
        /// <summary>Border pattern for the second line-pair.</summary>
        public int BorderBottom { get { return _borderBottom; } }
        private byte _borderTop, _borderBottom;
        private int _borderWriteToggle;

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

        private static int EnvInt(string name, int dflt)
        {
            string e = Environment.GetEnvironmentVariable(name);
            int v;
            return (!string.IsNullOrEmpty(e) && int.TryParse(e, out v)) ? v : dflt;
        }

        /// <summary>Border is 32 lines top and bottom, 32 bits each side (TechRef 3.1.1 / 3.4).</summary>
        public const int BorderPixels = 32;

        /// <summary>
        /// Picture-area size, i.e. the bitmap proper, excluding the border frame.
        /// 15" = 13 quadwords x 633 lines = 832x633; 19" = 18 quadwords x 861 = 1152x861.
        ///
        /// NB the TechRef says "17 quadwords" for the 19" and that is a documentation error --
        /// DsplHdlr.asm has numberQuadWords 13/18, and 18 x 64 = 1152 while 17 x 64 = 1088.
        /// </summary>
        public int PictureWidth { get; private set; }
        public int PictureHeight { get; private set; }

        /// <summary>
        /// Render one mono field: the border frame, the bitmap, and the hardware cursor sprite,
        /// into <see cref="Frame"/>.
        ///
        /// The frame is the VISIBLE area = picture + 32 pixels of border on every side, so
        /// 15" = 896x697 and 19" = 1216x925.  (The 15" figures are given outright in TechRef
        /// 3.1.1 -- 832+64 = 896, 633+64 = 697 -- which is what pins the border arithmetic.)
        /// Rendering the picture alone, as this did before, is why the cursor could never be
        /// placed correctly: the cursor registers are in raster coordinates that count the border.
        /// </summary>
        public void RenderMono()
        {
            int quadwords = QuadwordsPerLine;
            if (quadwords <= 0) quadwords = 13;         // default 15"
            int wordsPerLine = quadwords * 4;
            int picW = wordsPerLine * 16;
            int picH = _displayLines > 0 ? _displayLines
                     : (quadwords >= 16 ? 861 : 633);

            PictureWidth = picW;
            PictureHeight = picH;

            int width = picW + 2 * BorderPixels;
            int height = picH + 2 * BorderPixels;
            Width = width;
            Height = height;
            if (_frame == null || _frame.Length != width * height) _frame = new byte[width * height];

            // ---- border ----
            // EC81 = pattern low, EC82 = pattern high, and the screen alternates TWO lines of
            // low with TWO of high, HIGH AT THE TOP (App A 4.0.4).  The 8-bit pattern repeats
            // horizontally, MSB leftmost.
            //
            // PHASE.  The guest programs the border to continue whatever stipple the picture is
            // using, so a phase error of one pixel or one line shows as a seam on all four sides
            // rather than as a wrong pattern.  Three things can each be off by one and the
            // documentation pins none of them outright:
            //   * which line of the 4-line cycle the visible area starts on (and hence whether the
            //     top pair is high or low -- App A says "high at the top" of the SCREEN, but our
            //     frame origin is the top border, not the top of the raster);
            //   * the horizontal bit phase, which depends on where the visible left edge falls in
            //     the raster (the pattern free-runs across blanking);
            //   * interlace, where the two fields are offset by a half line.
            // DOVE_BORDER_VPHASE (0-3 lines) and DOVE_BORDER_HPHASE (0-7 pixels) tune them without
            // a rebuild.  Calibrating against a screenshot costs one restart; deriving it from
            // blanking widths costs an afternoon and is what the DDC reference warns against for
            // the cursor offsets.
            // The boot ROM and ViewPoint want vertical phases exactly two apart, and it is NOT
            // interlace -- both run interlaced (operator).  The remaining candidate is the top
            // border LINE COUNT, which TechRef 3.1.1 says is "set by microcode" and which the DDC
            // reference warns is guest-dependent: if the two program different counts, our
            // frame-relative pair alignment shifts with them.  DescribeVerticalCs / TopBorderLines
            // are instrumented to settle it; until then this stays a knob rather than a guess.
            int vph = EnvInt("DOVE_BORDER_VPHASE", 0);
            int hph = EnvInt("DOVE_BORDER_HPHASE", 0);
            for (int y = 0; y < height; y++)
            {
                // First-written pattern on the top pair (see WriteByte).
                bool firstPair = ((((y + vph) >> 1) & 1) == 0);
                int pat = firstPair ? BorderTop : BorderBottom;
                bool pictureRow = y >= BorderPixels && y < BorderPixels + picH;
                int row = y * width;
                if (pictureRow)
                {
                    // side borders only
                    for (int x = 0; x < BorderPixels; x++)
                        _frame[row + x] = (byte)((pat >> (7 - ((x + hph) & 7))) & 1);
                    for (int x = width - BorderPixels; x < width; x++)
                        _frame[row + x] = (byte)((pat >> (7 - ((x + hph) & 7))) & 1);
                }
                else
                {
                    for (int x = 0; x < width; x++)
                        _frame[row + x] = (byte)((pat >> (7 - ((x + hph) & 7))) & 1);
                }
            }

            // ---- picture ----
            int originByte = BitmapStartWord * 2;
            int lineBytes = wordsPerLine * 2;

            for (int y = 0; y < picH; y++)
            {
                int rowBase = originByte + y * lineBytes;
                int px = (y + BorderPixels) * width + BorderPixels;
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
            ApplyMixOutsideCursor(width, height);
        }

        /// <summary>The 32-byte hardware cursor sprite buffer (ED00-ED1F), 16x16 1bpp.</summary>
        public byte[] GetCursorBuffer()
        {
            byte[] b = new byte[32];
            for (int i = 0; i < 32; i++) b[i] = _reg[CursorBufBase - WindowBase + i];
            return b;
        }

        /// <summary>
        /// Cursor raster-to-visible offsets.  These are NAMED FIRMWARE CONSTANTS from
        /// DsplHdlr.asm -- xCoordOffset 208/304 (15"/19"), yCoordOffset 32 (both) -- and the
        /// DDC reference is explicit that they must not be derived from the frame geometry.
        ///
        /// The IOP adds them to the Mesa picture-space coordinate before writing EC83-EC86, so
        /// scanning them back out gives a raster coordinate.  Y counts from the top of the
        /// VISIBLE area (i.e. from the top border), which is why yCoordOffset happens to equal
        /// the 32-line border and cancels here; X does not cancel.
        ///
        /// DOVE_CURSOR_XOFF / DOVE_CURSOR_YOFF override them, because the exact reference edge
        /// for X is the one thing the documentation does not state outright and it is far
        /// cheaper to calibrate against a screenshot than to reason about blanking widths.
        /// </summary>
        private int CursorXOffset
        {
            get
            {
                string e = Environment.GetEnvironmentVariable("DOVE_CURSOR_XOFF");
                int v;
                if (!string.IsNullOrEmpty(e) && int.TryParse(e, out v)) return v;
                return QuadwordsPerLine >= 16 ? 304 : 208;      // 19" : 15"
            }
        }
        private int CursorYOffset
        {
            get
            {
                string e = Environment.GetEnvironmentVariable("DOVE_CURSOR_YOFF");
                int v;
                if (!string.IsNullOrEmpty(e) && int.TryParse(e, out v)) return v;
                return 32;
            }
        }

        /// <summary>
        /// Cursor origin in VISIBLE-frame coordinates.  Registers are raster coordinates, and the
        /// frame origin sits BorderPixels outside the picture, so
        ///     visible = raster - firmwareOffset + border.
        /// Both the sprite composite and the mix-exclusion band must use this; when they disagreed
        /// the sprite drew at the corrected position while the mix was suppressed over a 16x16 hole
        /// at the raw position, which showed as a black box tracking the mouse.
        /// </summary>
        public int CursorVisibleX
        {
            get { return CursorWord * 16 + CursorBitOffset - CursorXOffset + BorderPixels; }
        }
        public int CursorVisibleY
        {
            get { return CursorLine - CursorYOffset + BorderPixels; }
        }

        private void CompositeCursor(int width, int height)
        {
            if (CursorDisabled) return;

            int cx = CursorVisibleX;
            int cy = CursorVisibleY;
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

        /// <summary>
        /// The 4-bit mix function (App A Fig 4.0), V = bitmap video, C = cursor video.
        /// The code IS the truth table: bit (V*2 + C) of the nibble is the result, which
        /// makes it the usual raster-op encoding and gives all 16 boolean functions of
        /// two variables exactly once.  (The previous hand-written switch had four
        /// duplicate entries -- 1/4, 2/8, 6/9, 7/D -- and so was missing NOR, V'C,
        /// NAND and XNOR.  Mix 1 = NOR is the one the diagnostics uses.)
        /// </summary>
        private static int MixPixel(int f, int v, int c)
        {
            return (f >> ((v << 1) | c)) & 1;
        }

        /// <summary>
        /// The mix is a function of the whole raster, not just the 16x16 sprite: outside
        /// the cursor the cursor video C is simply 0, so a mix like 1 (NOR) or 3 (V')
        /// inverts the entire picture and F forces it white.  Applying it only where the
        /// sprite happened to be left inverting modes looking like no-ops -- which is why
        /// the boot screen (mix E/C/4, all identity in V when C=0) rendered correctly
        /// while the diagnostics menu (mix 1) came out inverted.
        ///
        /// The sprite region is already mixed with its real C bits by CompositeCursor, so
        /// it is skipped here.
        /// </summary>
        private void ApplyMixOutsideCursor(int width, int height)
        {
            int mix = MixFunction;
            byte m0 = (byte)MixPixel(mix, 0, 0);
            byte m1 = (byte)MixPixel(mix, 1, 0);
            if (m0 == 0 && m1 == 1) return;   // identity in V -- nothing to do

            int cy0 = -1, cy1 = -1, cx0 = 0, cx1 = 0;
            if (!CursorDisabled)
            {
                cx0 = CursorVisibleX; cx1 = cx0 + 16;
                cy0 = CursorVisibleY; cy1 = cy0 + 16;
            }

            for (int y = 0; y < height; y++)
            {
                int row = y * width;
                bool band = y >= cy0 && y < cy1;
                int lo = band ? Math.Max(0, Math.Min(width, cx0)) : width;
                int hi = band ? Math.Max(0, Math.Min(width, cx1)) : width;
                for (int x = 0; x < lo; x++) { int i = row + x; _frame[i] = _frame[i] != 0 ? m1 : m0; }
                for (int x = hi; x < width; x++) { int i = row + x; _frame[i] = _frame[i] != 0 ? m1 : m0; }
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
