using System.Collections.Generic;
using System.Windows.Forms;

namespace D.Doovke
{
    /// <summary>
    /// Host key -> 6085 KeyStation number (the LevelIV keyboard, from KeyStations.mesa /
    /// LevelIVKeys.mesa).
    ///
    /// There is no separate "scan code": the byte on the wire IS the station number, with
    /// bit 7 clear for a press and set for a release (KEYMO.asm).  Pilot sees a 112-bit
    /// down/up bitmap where -- inverted from the obvious reading -- down = 0 and up = 1, so
    /// a press CLEARS the station's bit and a release SETS it.  The IOP firmware maintains
    /// that map itself; Doovke only has to deliver well-formed press/release bytes.
    ///
    /// The guest tracks the shift keys as ordinary stations and does its own case folding,
    /// so this table is deliberately NOT shift-sensitive: send the unshifted station plus a
    /// real LeftShift/RightShift press, exactly as the hardware would.
    /// </summary>
    public static class DoovkeKeyboard
    {
        /// <summary>Bit 7 set = release.  A press is the bare station number.</summary>
        public const byte ReleaseFlag = 0x80;

        // Mouse buttons live in the same station space (MouseFace mirrors them into the
        // keyboard bitmap).  Red = select/Point, Blue = menu/Adjust.
        public const byte StationMouseRed = 13;    // Mouse1 -- host LEFT
        public const byte StationMouseBlue = 14;   // Mouse3 -- host RIGHT
        public const byte StationMouseMiddle = 15; // Mouse2 -- host MIDDLE

        private static readonly Dictionary<Keys, byte> Map = new Dictionary<Keys, byte>
        {
            // ---- Letters ----
            { Keys.A, 37 }, { Keys.B, 55 }, { Keys.C, 53 }, { Keys.D, 21 }, { Keys.E, 19 },
            { Keys.F, 51 }, { Keys.G, 66 }, { Keys.H, 68 }, { Keys.I, 39 }, { Keys.J, 54 },
            { Keys.K, 25 }, { Keys.L, 42 }, { Keys.M, 71 }, { Keys.N, 70 }, { Keys.O, 41 },
            { Keys.P, 27 }, { Keys.Q, 35 }, { Keys.R, 64 }, { Keys.S, 36 }, { Keys.T, 65 },
            { Keys.U, 22 }, { Keys.V, 23 }, { Keys.W, 34 }, { Keys.X, 40 }, { Keys.Y, 67 },
            { Keys.Z, 56 },

            // ---- Digits (top row and keypad alike) ----
            { Keys.D0, 24 }, { Keys.D1, 48 }, { Keys.D2, 33 }, { Keys.D3, 32 }, { Keys.D4, 17 },
            { Keys.D5, 16 }, { Keys.D6, 18 }, { Keys.D7, 20 }, { Keys.D8, 69 }, { Keys.D9, 38 },
            { Keys.NumPad0, 24 }, { Keys.NumPad1, 48 }, { Keys.NumPad2, 33 }, { Keys.NumPad3, 32 },
            { Keys.NumPad4, 17 }, { Keys.NumPad5, 16 }, { Keys.NumPad6, 18 }, { Keys.NumPad7, 20 },
            { Keys.NumPad8, 69 }, { Keys.NumPad9, 38 },

            // ---- Symbols ----
            { Keys.OemMinus, 26 },      // Dash
            { Keys.Oemplus, 75 },       // Equal
            { Keys.OemQuestion, 28 },   // Slash
            { Keys.Oemcomma, 43 },      // Comma
            { Keys.OemPeriod, 58 },     // Period
            { Keys.Oem1, 59 },          // SemiColon
            { Keys.Oem7, 44 },          // Quote
            { Keys.Oem6, 45 },          // RightBracket
            { Keys.Oem5, 74 },          // Half  (backslash key -- nearest host position)
            { Keys.Oem3, 61 },          // Para  (backquote key)

            // ---- Editing ----
            { Keys.Space, 73 },
            { Keys.Tab, 49 },
            { Keys.Back, 31 },          // BS
            { Keys.Delete, 62 },
            { Keys.CapsLock, 72 },      // Lock
            { Keys.Enter, 60 },         // NewPara -- the 6085's Return
            { Keys.LineFeed, 50 },      // ParaTab
            { Keys.ShiftKey, 57 }, { Keys.LShiftKey, 57 },
            { Keys.RShiftKey, 76 },

            // ---- Boot-device row ----
            // Stations 99-108 are the typography row (Bold, Italic, Underlined, Superscript,
            // Subscript, Smaller, then four spares).  The boot ROM's SelectionLoop reads them
            // as the boot-device icons, which is the one mapping confirmed on the machine --
            // F2 = station 100 boots the floppy.  Keep the host F-keys here.
            { Keys.F1,  99 }, { Keys.F2, 100 }, { Keys.F3, 101 }, { Keys.F4, 102 }, { Keys.F5, 103 },
            { Keys.F6, 104 }, { Keys.F7, 105 }, { Keys.F8, 106 }, { Keys.F9, 107 }, { Keys.F10, 108 },

            // ---- Xerox named keys ----
            // The 6085 has no F1-F10, so there is no hardware correspondence here; the host
            // keys below are a Doovke convention chosen for familiarity, not a spec.
            { Keys.Escape, 77 },        // Stop
            { Keys.F11, 92 },           // Help
            { Keys.F12, 79 },           // Undo
            { Keys.Home, 90 },          // Find
            { Keys.End, 63 },           // Next
            { Keys.Insert, 46 },        // Open
            { Keys.Apps, 52 },          // Props
            { Keys.PageUp, 91 },        // Again
            { Keys.PageDown, 89 },      // Copy
            { Keys.ControlKey, 47 },    // Special
            { Keys.Menu, 78 },          // Move  (host Alt)
        };

        /// <summary>Translate a host key to a KeyStation number; false if unmapped.</summary>
        public static bool TryStation(Keys key, out byte station)
        {
            return Map.TryGetValue(key, out station);
        }
    }
}
