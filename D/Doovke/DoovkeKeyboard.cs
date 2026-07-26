using System.Collections.Generic;
using System.Windows.Forms;

namespace D.Doovke
{
    /// <summary>
    /// Host key -> Dove keyboard scan code.
    ///
    /// The Dove keyboard is NOT the DLion's: codes arrive as raw bytes latched into the 8251
    /// UART (DoveIOPIO.InjectKeyboard), and the only encoding confirmed so far is the
    /// boot-device row, 0x63 = F1 .. 0x6C = F10, which KEYMO stores at HexValue (0x3DB2).
    /// The DLion KeyCode enum in D/IOP/Keyboard.cs is a different, Star-keyboard encoding and
    /// must NOT be reused here.
    ///
    /// The rest of the table is pending the real 6085 keymap; until it lands, unmapped keys are
    /// simply not sent (better than sending a wrong byte and having the guest act on it), and
    /// Machine > Send Scan Code lets any byte be delivered by hand.
    /// </summary>
    public static class DoovkeKeyboard
    {
        private static readonly Dictionary<Keys, byte> Map = new Dictionary<Keys, byte>
        {
            // Boot-device function keys -- confirmed.
            { Keys.F1,  0x63 },
            { Keys.F2,  0x64 },
            { Keys.F3,  0x65 },
            { Keys.F4,  0x66 },
            { Keys.F5,  0x67 },
            { Keys.F6,  0x68 },
            { Keys.F7,  0x69 },
            { Keys.F8,  0x6A },
            { Keys.F9,  0x6B },
            { Keys.F10, 0x6C },

            // TODO: alphanumerics, Return, Space, Backspace, arrows and modifiers, from the
            // 6085 keymap.  Add them here as { Keys.X, 0xNN } and they work immediately.
        };

        /// <summary>Translate a host key to a Dove scan code; false if unmapped.</summary>
        public static bool TryMap(Keys key, bool shift, out byte code)
        {
            return Map.TryGetValue(key, out code);
        }

        /// <summary>True once the table covers more than the boot-device row.</summary>
        public static bool HasFullKeymap { get { return Map.Count > 10; } }
    }
}
