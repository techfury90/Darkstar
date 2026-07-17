using System;
using System.IO;
using D.IOP;

namespace DoveDisplayTest
{
    class FlatMemory : IPhysicalMemory
    {
        public byte[] Mem = new byte[0x100000];
        public byte ReadByte(int a) { return Mem[a & 0xFFFFF]; }
        public void WriteByte(int a, byte v) { Mem[a & 0xFFFFF] = v; }
        public ushort ReadWord(int a) { return (ushort)(ReadByte(a) | (ReadByte(a + 1) << 8)); }
        public void WriteWord(int a, ushort v) { WriteByte(a, (byte)v); WriteByte(a + 1, (byte)(v >> 8)); }
    }

    // Verifies the DoveDisplayController scan-out (mono bitmap geometry + bit order)
    // and the hardware-cursor sprite compositing (the MP-code-as-sprite path), by
    // programming the DDC the way the IOP firmware would and rendering a known image.
    class Program
    {
        const int WordsPerLine = 52;   // 13 quadwords * 4 = 832 px
        const int OriginWord = 0x800;  // bitmap start (word address)
        static FlatMemory _mem;

        static void SetPixel(int x, int y)
        {
            if (x < 0 || y < 0) return;
            int wordIdx = OriginWord + y * WordsPerLine + (x >> 4);
            int bit = 15 - (x & 15);
            int addr = wordIdx * 2;
            ushort w = _mem.ReadWord(addr);
            w |= (ushort)(1 << bit);
            _mem.WriteWord(addr, w);
        }

        static void Main()
        {
            _mem = new FlatMemory();
            int W = 832, H = 633;

            // --- Draw a recognizable test image into the bitmap (1 = black) ---
            // 2px border frame.
            for (int x = 0; x < W; x++) { SetPixel(x, 0); SetPixel(x, 1); SetPixel(x, H - 1); SetPixel(x, H - 2); }
            for (int y = 0; y < H; y++) { SetPixel(0, y); SetPixel(1, y); SetPixel(W - 1, y); SetPixel(W - 2, y); }
            // Diagonals.
            for (int i = 0; i < Math.Min(W, H); i++) { SetPixel(i, i); SetPixel(W - 1 - i, i); }
            // A "boot device icon" box in the middle.
            for (int x = 320; x <= 512; x++) { SetPixel(x, 240); SetPixel(x, 392); }
            for (int y = 240; y <= 392; y++) { SetPixel(320, y); SetPixel(512, y); }
            // A grey 50% stipple patch (top-left) to show fine detail.
            for (int y = 40; y < 120; y++)
                for (int x = 40; x < 200; x++)
                    if (((x + y) & 1) == 0) SetPixel(x, y);

            // --- Program the DDC the way the firmware would (init sequence 5.0) ---
            var ddc = new DoveDisplayController(_mem);
            ddc.WriteByte(0xEC88, 13);                       // quadwords per line (15")
            ddc.WriteByte(0xEC89, (byte)((OriginWord >> 5) & 0xFF));         // bitmap start low
            ddc.WriteByte(0xEC8A, (byte)((OriginWord >> 13) & 0xFF));        // bitmap start high
            ddc.WriteByte(0xEC81, 0x00);                     // border low
            ddc.WriteByte(0xEC82, 0x00);                     // border high

            // --- MP-code-as-sprite: rasterize a 16x16 glyph into the cursor buffer ---
            // A distinctive arrow + box so the composite is unmistakable.
            ushort[] cursor = new ushort[16]
            {
                0xFFFF, 0x8001, 0x8001, 0x9C01,
                0x9401, 0x9C01, 0x8001, 0x87E1,
                0x8421, 0x87E1, 0x8001, 0x8181,
                0x8181, 0x81FF, 0x8001, 0xFFFF,
            };
            for (int row = 0; row < 16; row++)
            {
                ddc.WriteByte(0xED00 + row * 2, (byte)(cursor[row] >> 8));    // hi byte (MSB = left)
                ddc.WriteByte(0xED00 + row * 2 + 1, (byte)(cursor[row] & 0xFF));
            }
            int cursorX = 600, cursorY = 300;
            ddc.WriteByte(0xEC83, (byte)(cursorX / 16));      // cursor X word
            ddc.WriteByte(0xEC84, (byte)(cursorX % 16));      // cursor X offset (NCURSOR=0 => enabled)
            ddc.WriteByte(0xEC85, (byte)(cursorY & 0xFF));    // cursor Y low
            ddc.WriteByte(0xEC86, (byte)((cursorY >> 8) & 0x03)); // cursor Y high
            ddc.WriteByte(0xEC80, 0x02 | (0xE << 4));         // VIDEO=1, mix fn E (V+C = overlay)

            // --- Verify decode ---
            Console.WriteLine("DDC state: VIDEO=" + ddc.VideoEnabled + " quadwords=" + ddc.QuadwordsPerLine +
                              " bitmapStartWord=0x" + ddc.BitmapStartWord.ToString("X") +
                              " (expected 0x" + OriginWord.ToString("X") + ")" +
                              " cursor@(" + (ddc.CursorWord * 16 + ddc.CursorBitOffset) + "," + ddc.CursorLine + ")" +
                              " mixFn=" + ddc.MixFunction);

            ddc.RenderMono();
            int set = 0; foreach (var b in ddc.Frame) if (b != 0) set++;
            Console.WriteLine("Rendered " + ddc.Width + "x" + ddc.Height + ", " + set + " pixels set");

            string outPath = Environment.GetEnvironmentVariable("DOVE_FB") ?? "display_test.bin";
            using (var fs = new FileStream(outPath, FileMode.Create))
            using (var bw = new BinaryWriter(fs))
            {
                bw.Write(ddc.Width);
                bw.Write(ddc.Height);
                bw.Write(ddc.Frame);
            }
            Console.WriteLine("Framebuffer written to " + outPath);
        }
    }
}
