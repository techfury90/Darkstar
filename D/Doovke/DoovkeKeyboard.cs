using System.Collections.Generic;
using System.Windows.Forms;

namespace D.Doovke
{
    /// <summary>
    /// Host key -> 6085 KeyStation number.
    ///
    /// The 6085 keyboard is **Level V** (LevelVKeys.mesa, 1985), not the Level IV keyboard of
    /// the 8010 Star.  The two agree on the letters and digits, which is what makes them easy
    /// to conflate, but Level V additionally populates the numeric keypad, the -Alt shift
    /// keys, the Japanese input keys and the diagnostic bits, and it reassigns some of the
    /// bits Level IV left spare (74 is LeftBracket here, not Level IV's Half).
    ///
    /// There is no separate "scan code": the byte on the wire IS the station number, with bit
    /// 7 clear for a press and set for a release (KEYMO.asm).  Pilot sees a 112-bit down/up
    /// bitmap in which -- inverted from the obvious reading -- down = 0 and up = 1, so a press
    /// CLEARS the station's bit and a release SETS it.  The IOP firmware maintains that map
    /// itself; Doovke only has to deliver well-formed bytes.
    ///
    /// The guest tracks the shift keys as ordinary stations and does its own case folding, so
    /// this table is deliberately NOT shift-sensitive: a press sends the unshifted station,
    /// plus a real LeftShift/RightShift press, exactly as the hardware would.
    ///
    /// KEYBOARD TYPE, measured 2026-07-26: all EIGHT museum 6085 EEPROM dumps in the build
    /// kit (serials spanning 1986-1988) carry eePromKBType = 1, not the 3 = level5 a 6085 was
    /// expected to report.  The offset is not in doubt -- its neighbours decode sensibly
    /// (RigidSctPerTrk = 16, RigidHdPerCyl = 8) and eePromDispType varies machine-to-machine
    /// while this field does not.  Whether 1 is "level4" in the same namespace as the Mesa
    /// KeyboardType enum, or a wire-level code the IOP translates into the Mesa ordinal, is a
    /// source question and is NOT settled here.
    ///
    /// Either way Doovke does not have to decide: it hands the genuine EEPROM to the genuine
    /// IOP firmware, so whatever MesaUpDn's readKeyboardType does with the value happens for
    /// real.  The EEPROM is deliberately NOT patched -- no real machine says 3, and its
    /// checksum at word 62 is a boot gate (corrupting it alone hangs the machine before the
    /// display is even programmed, measured).
    ///
    /// None of this changes the numbers below: a station number is what the keyboard hardware
    /// puts on the wire, and Level IV and Level V agree on every station Level IV has.  The
    /// Level-V-only stations (keypad, -Alt shifts, DoubleQuote, Japanese keys) are left mapped
    /// because a real Level V keyboard would physically send them; if the guest turns out to
    /// name bits through a Level IV table, those are the ones that would do nothing.
    /// </summary>
    public static class DoovkeKeyboard
    {
        /// <summary>Bit 7 set = release.  A press is the bare station number.</summary>
        public const byte ReleaseFlag = 0x80;

        // Mouse buttons live in the same station space (MouseFace mirrors them into the
        // keyboard bitmap).  Level V names: Point selects, Adjust extends, Menu pops up.
        public const byte StationPoint = 13;   // Mouse1 -- host LEFT
        public const byte StationAdjust = 14;  // Mouse3 -- host RIGHT
        public const byte StationMenu = 15;    // Mouse2 -- host MIDDLE

        private static readonly Dictionary<Keys, byte> Map = new Dictionary<Keys, byte>
        {
            // ---- Letters (shared with Level IV) ----
            { Keys.A, 37 }, { Keys.B, 55 }, { Keys.C, 53 }, { Keys.D, 21 }, { Keys.E, 19 },
            { Keys.F, 51 }, { Keys.G, 66 }, { Keys.H, 68 }, { Keys.I, 39 }, { Keys.J, 54 },
            { Keys.K, 25 }, { Keys.L, 42 }, { Keys.M, 71 }, { Keys.N, 70 }, { Keys.O, 41 },
            { Keys.P, 27 }, { Keys.Q, 35 }, { Keys.R, 64 }, { Keys.S, 36 }, { Keys.T, 65 },
            { Keys.U, 22 }, { Keys.V, 23 }, { Keys.W, 34 }, { Keys.X, 40 }, { Keys.Y, 67 },
            { Keys.Z, 56 },

            // ---- Main digit row (shared with Level IV) ----
            { Keys.D0, 24 }, { Keys.D1, 48 }, { Keys.D2, 33 }, { Keys.D3, 32 }, { Keys.D4, 17 },
            { Keys.D5, 16 }, { Keys.D6, 18 }, { Keys.D7, 20 }, { Keys.D8, 69 }, { Keys.D9, 38 },

            // ---- Numeric keypad (Level V only -- these are NOT the digit-row stations) ----
            { Keys.NumPad0, 98 }, { Keys.NumPad1, 94 }, { Keys.NumPad2, 5 },  { Keys.NumPad3, 6 },
            { Keys.NumPad4, 84 }, { Keys.NumPad5, 85 }, { Keys.NumPad6, 87 }, { Keys.NumPad7, 81 },
            { Keys.NumPad8, 82 }, { Keys.NumPad9, 83 },
            { Keys.Add, 8 }, { Keys.Subtract, 9 }, { Keys.Multiply, 10 }, { Keys.Divide, 11 },
            { Keys.Decimal, 105 },      // KeypadPeriod
            { Keys.NumLock, 12 },       // KeypadClear

            // ---- Symbols ----
            { Keys.OemMinus, 26 },        // Dash
            { Keys.Oemplus, 75 },         // Equal
            { Keys.OemQuestion, 28 },     // Slash
            { Keys.Oemcomma, 43 },        // Comma
            { Keys.OemPeriod, 58 },       // Period
            { Keys.Oem1, 59 },            // SemiColon
            { Keys.Oem7, 44 },            // Quote
            { Keys.OemOpenBrackets, 74 }, // LeftBracket
            { Keys.Oem6, 45 },            // RightBracket
            { Keys.Oem3, 61 },            // OpenQuote
            // SingleQuote (7) and DoubleQuote (108) are separate physical Level V keys with no
            // host equivalent, so they are reachable only via Machine > Send Scan Code.

            // ---- Editing ----
            { Keys.Space, 73 },
            { Keys.Tab, 49 },
            { Keys.Back, 31 },          // BS
            { Keys.Delete, 62 },
            { Keys.CapsLock, 72 },      // Lock
            { Keys.Enter, 60 },         // NewPara -- the 6085's Return
            { Keys.LineFeed, 50 },      // ParaTab

            // ---- Modifiers ----
            { Keys.ShiftKey, 57 }, { Keys.LShiftKey, 57 },
            { Keys.RShiftKey, 76 },
            // LeftShiftAlt (107), RightShiftAlt (111) and Case (3) are Level V additions with
            // no natural host key; host Alt is reserved for releasing the captured mouse.

            // ---- Boot-device row ----
            // Stations 99-108.  The boot ROM's SelectionLoop reads these raw as the boot-device
            // icons, which is the one mapping confirmed on the machine -- F2 = station 100
            // boots the floppy.  Only meaningful at the boot screen: under Pilot these are
            // Bold/Italic/Underline/Superscript/Subscript/Smaller and then keypad and quote
            // keys, so F7-F10 duplicate stations that have their own entries above.
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
        };

        /// <summary>Translate a host key to a KeyStation number; false if unmapped.</summary>
        public static bool TryStation(Keys key, out byte station)
        {
            return Map.TryGetValue(key, out station);
        }
    }
}
