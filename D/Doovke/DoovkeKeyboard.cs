using System.Collections.Generic;
using System.Windows.Forms;

namespace D.Doovke
{
    /// <summary>
    /// Host key -> 6085 keyboard WIRE SCAN CODE.
    ///
    /// There are two distinct layers here, and conflating them is what made every key come out
    /// as a different letter:
    ///
    ///   1. the wire scan code -- what the keyboard's 8048 puts on the UART, and therefore what
    ///      Doovke must feed KEYMO.  These are the KBIndexInit indices (KyMoInit.asm), and they
    ///      run in a clean near-QWERTY order.  THIS TABLE.
    ///   2. the KeyStation / bitmap bit -- where KEYMO's KBIndex translation lands it in the
    ///      112-bit down/up map Pilot reads.  Only relevant if you bypass KEYMO and write the
    ///      bitmap directly, in which case note the packing is MSB-first within each byte:
    ///      bit = 0x80 >> (station % 8).
    ///
    /// Sending layer-2 numbers into layer 1 does not fail loudly -- KEYMO happily translates
    /// them, so every keystroke arrives as a real but WRONG key.  Measured: sending P's bit
    /// (27) typed "O", O's bit (41) typed "L", L's bit (42) typed ";" and Q's bit (35) typed
    /// "D", because 27/41/42/35 are the scan codes of o/l/semicolon/d.
    ///
    /// Press = scan, release = scan | 0x80.  The guest polls the down/up bitmap, so a key has
    /// to be HELD across a poll: a press and release delivered back-to-back are never seen.
    ///
    /// The guest tracks the shift keys as ordinary keys and folds case itself, so this table is
    /// deliberately NOT shift-sensitive: send the unshifted code plus a real shift press.
    /// </summary>
    public static class DoovkeKeyboard
    {
        /// <summary>Bit 7 set = release.  A press is the bare scan code.</summary>
        public const byte ReleaseFlag = 0x80;

        // Mouse buttons are scan codes like any other key (movement is the separate
        // 0xFF + dX + dY report).  Point selects, Adjust extends, Menu pops up.
        public const byte ScanPoint = 1;    // host LEFT
        public const byte ScanAdjust = 2;   // host RIGHT
        public const byte ScanMenu = 3;     // host MIDDLE

        private static readonly Dictionary<Keys, byte> Map = new Dictionary<Keys, byte>
        {
            // ---- Letters ----
            { Keys.Q, 19 }, { Keys.W, 20 }, { Keys.E, 21 }, { Keys.R, 22 }, { Keys.T, 23 },
            { Keys.Y, 24 }, { Keys.U, 25 }, { Keys.I, 26 }, { Keys.O, 27 }, { Keys.P, 28 },
            { Keys.A, 33 }, { Keys.S, 34 }, { Keys.D, 35 }, { Keys.F, 36 }, { Keys.G, 37 },
            { Keys.H, 38 }, { Keys.J, 39 }, { Keys.K, 40 }, { Keys.L, 41 },
            { Keys.Z, 47 }, { Keys.X, 48 }, { Keys.C, 49 }, { Keys.V, 50 }, { Keys.B, 51 },
            { Keys.N, 52 }, { Keys.M, 53 },

            // ---- Digits ----
            { Keys.D1, 5 }, { Keys.D2, 6 }, { Keys.D3, 7 }, { Keys.D4, 8 },  { Keys.D5, 9 },
            { Keys.D6, 10 }, { Keys.D7, 11 }, { Keys.D8, 12 }, { Keys.D9, 13 }, { Keys.D0, 14 },
            // The keypad's own scan codes are not in the table we have, so the host keypad is
            // mapped onto the main digit row as a Doovke convenience.  Replace if they turn up.
            { Keys.NumPad1, 5 }, { Keys.NumPad2, 6 }, { Keys.NumPad3, 7 }, { Keys.NumPad4, 8 },
            { Keys.NumPad5, 9 }, { Keys.NumPad6, 10 }, { Keys.NumPad7, 11 }, { Keys.NumPad8, 12 },
            { Keys.NumPad9, 13 }, { Keys.NumPad0, 14 },

            // ---- Symbols ----
            { Keys.OemMinus, 15 },         // Dash
            { Keys.Oemplus, 16 },          // Equal
            { Keys.OemOpenBrackets, 29 },  // LeftBracket
            { Keys.Oem6, 30 },             // RightBracket
            { Keys.Oem1, 42 },             // SemiColon
            { Keys.Oem7, 43 },             // SingleQuote
            { Keys.Oemcomma, 54 },         // Comma
            { Keys.OemPeriod, 55 },        // Period
            { Keys.OemQuestion, 56 },      // Slash
            // DoubleQuote (44) is its own key on this keyboard, with no host equivalent.

            // ---- Control / editing ----
            { Keys.Tab, 17 },           // RightTab
            { Keys.LineFeed, 18 },      // ParaTab
            { Keys.Enter, 31 },         // Return
            { Keys.Space, 61 },
            { Keys.ShiftKey, 45 }, { Keys.LShiftKey, 45 },
            { Keys.RShiftKey, 57 },
            { Keys.CapsLock, 109 },     // Lock
            // No separate Backspace code in the table we have, so host Backspace is mapped to
            // Delete alongside the Delete key, since typing is unusable without it.
            { Keys.Back, 85 }, { Keys.Delete, 85 },

            // ---- Function / Xerox keys ----
            { Keys.Escape, 84 },        // Stop
            { Keys.F12, 86 },           // Undo
            { Keys.PageUp, 87 },        // Again
            { Keys.Home, 88 },          // Find
            { Keys.PageDown, 89 },      // Copy
            { Keys.End, 91 },           // Same / Paste
            { Keys.Insert, 93 },        // Open
            { Keys.Apps, 94 },          // Props
            { Keys.ControlKey, 108 },   // Font

            // ---- Boot-device row ----
            // Codes 99-108 are what the boot ROM's SelectionLoop reads raw as the boot-device
            // icons -- the one mapping confirmed on the machine (F2 = 100 boots the floppy).
            // Under the running system these are Center/Bold/Italic/.../Underline/Font, so
            // several of these duplicate keys that have their own entries above.
            { Keys.F1,  99 }, { Keys.F2, 100 }, { Keys.F3, 101 }, { Keys.F4, 102 }, { Keys.F5, 103 },
            { Keys.F6, 104 }, { Keys.F7, 105 }, { Keys.F8, 106 }, { Keys.F9, 107 }, { Keys.F10, 108 },
        };

        /// <summary>Translate a host key to a wire scan code; false if unmapped.</summary>
        public static bool TryStation(Keys key, out byte scan)
        {
            return Map.TryGetValue(key, out scan);
        }
    }
}
