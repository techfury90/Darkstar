using System;
using System.IO;

namespace D.Doovke
{
    /// <summary>
    /// Doovke entry point (Dove/Daybreak = Xerox 6085).  Darkstar remains the DLion target.
    ///
    /// This first cut is headless: it assembles the machine, boots it, optionally injects a
    /// keystroke, and writes the framebuffer out so the screen can be inspected.  The UI
    /// (window + live keyboard + floppy load/eject) grows onto DoovkeMachine from here --
    /// the machine deliberately exposes InjectKey/LoadFloppy/RenderFrame for exactly that.
    /// </summary>
    internal static class Program
    {
        [System.STAThread]
        private static int Main(string[] args)
        {
            string bootRom = null, eeprom = null, floppy = null, fbOut = "doovke_fb.bin";
            long budget = 150000000;
            long pokeAt = 0; byte pokeCode = 0; long keyDelay = 2000000;
            string ipTrace = null; long ipEvery = 1000;
            long ejectAt = -1, changeAt = -1; string changeTo = null;
            bool headless = false; bool keyRelease = false;
            string typeCodes = null; long probeAt = -1, probeGap = 2000000, probeHold = 1000000; int probeFrom = 0, probeTo = -1; int probeDelim = -1;

            for (int i = 0; i < args.Length; i++)
            {
                string a = args[i];
                string next = (i + 1 < args.Length) ? args[i + 1] : null;
                switch (a)
                {
                    case "--rom":    bootRom = next; i++; break;
                    case "--eeprom": eeprom = next; i++; break;
                    case "--floppy": floppy = next; i++; break;
                    case "--fb":     fbOut = next; i++; break;
                    case "--budget": budget = long.Parse(next); i++; break;
                    case "--key-at": pokeAt = long.Parse(next); i++; break;
                    // Scan code is HEX: the boot-device selection byte is 0x63 + icon index.
                    case "--key":       pokeCode = (byte)Convert.ToInt32(next, 16); i++; break;
                    case "--key-delay": keyDelay = long.Parse(next); i++; break;
                    case "--iptrace":       ipTrace = next; i++; break;
                    case "--iptrace-every": ipEvery = long.Parse(next); i++; break;
                    case "--eject-at":  ejectAt = long.Parse(next); i++; break;
                    case "--change-at": changeAt = long.Parse(next); i++; break;
                    case "--change-to": changeTo = next; i++; break;
                    case "--headless":  headless = true; break;
                    case "--key-release": keyRelease = true; break;
                    // Type each station in [from,to] so the guest's echo reveals its own
                    // station->character table, whatever table that turns out to be.
                    case "--probe-at":    probeAt = long.Parse(next); i++; break;
                    case "--probe-from":  probeFrom = int.Parse(next); i++; break;
                    case "--probe-to":    probeTo = int.Parse(next); i++; break;
                    case "--probe-gap":   probeGap = long.Parse(next); i++; break;
                    case "--probe-hold":  probeHold = long.Parse(next); i++; break;
                    case "--type":        typeCodes = next; i++; break;
                    case "--probe-delim": probeDelim = int.Parse(next); i++; break;
                    case "-h":
                    case "--help":   Usage(); return 0;
                    default:
                        Console.Error.WriteLine("unknown argument: " + a);
                        Usage();
                        return 2;
                }
            }

            if (string.IsNullOrEmpty(bootRom))
            {
                Console.Error.WriteLine("--rom is required (Dove IOP boot ROM, 16 KB, loads at 0xFC000)");
                Usage();
                return 2;
            }

            var machine = new DoovkeMachine(bootRom, eeprom);

            // Interactive by default; --headless keeps the batch/regression path.
            if (!headless)
            {
                if (!string.IsNullOrEmpty(floppy)) machine.LoadFloppy(0, floppy);
                // With a key given, queue the boot-device selection so the GUI boots straight
                // from it instead of falling through to the firmware's default (the rigid disk).
                if (pokeCode != 0) machine.ScheduleBootDeviceKey(pokeCode);
                System.Windows.Forms.Application.EnableVisualStyles();
                System.Windows.Forms.Application.SetCompatibleTextRenderingDefault(false);
                System.Windows.Forms.Application.Run(new DoovkeWindow(machine, floppy));
                return 0;
            }

            Console.WriteLine("Doovke -- Dove/Daybreak (Xerox 6085)");
            Console.WriteLine("  boot ROM : " + bootRom);
            Console.WriteLine("  EEPROM   : " + (eeprom ?? "(none)"));

            if (!string.IsNullOrEmpty(floppy))
            {
                machine.LoadFloppy(0, floppy);
                Console.WriteLine("  floppy 0 : " + floppy);
            }

            // Diagnostics: did the IOP ever load/start the CP, and did it touch the floppy?
            machine.Io.CpLoadLog = new System.Collections.Generic.List<string>();
            machine.Io.ConfigEeprom.ReadLog = new System.Collections.Generic.List<int>();

            Console.WriteLine("Running " + budget + " IOP instructions...");
            Console.WriteLine("  reset state: " + machine.Iop);
            // The boot-device selection needs the key pressed TWICE for the diagnostics disk:
            // the SelectionLoop consumes the first press to highlight the icon, the second to boot.
            System.IO.StreamWriter ipw = null;
            if (!string.IsNullOrEmpty(ipTrace)) ipw = new System.IO.StreamWriter(ipTrace);

            // Watch the DDC control register (EC80).  The mix nibble (bits 4-7) is a
            // function of (bitmap, cursor); several of its codes invert, so a change here
            // between the boot screen and a later screen explains a whole-raster flip.
            int lastCtl = -1;
            // The 4-digit hex LED at port 0x90 is the MP code -- the boot's own progress
            // report, and the only thing that says where a failure happened rather than that
            // one did.  Log every transition with the instruction count so a stall or an error
            // code can be placed in time.
            int lastLed = -1;
            // After each probed station, render and count set pixels.  A station the guest
            // acts on moves the screen; one it ignores leaves the count identical.  This finds
            // which stations do anything without having to read the glyphs.
            long nextWatch = long.MaxValue; int watchStation = -1; int lastCount = -1;
            if (probeAt >= 0 && !string.IsNullOrEmpty(typeCodes))
            {
                long at = probeAt;
                Console.WriteLine("  typing scan codes: " + typeCodes);
                foreach (var t in typeCodes.Split(new char[] { ',' }))
                {
                    machine.ScheduleKeystroke((byte)int.Parse(t.Trim()), at, probeHold);
                    at += probeGap;
                }
            }
            else if (probeAt >= 0 && probeTo >= probeFrom)
            {
                long at = probeAt;
                Console.WriteLine("  probe: stations " + probeFrom + ".." + probeTo
                                  + (probeDelim >= 0 ? " delimited by " + probeDelim : "")
                                  + ", every " + probeGap + " instructions from " + probeAt);
                for (int st = probeFrom; st <= probeTo; st++)
                {
                    machine.ScheduleKeystroke((byte)st, at, probeHold); at += probeGap;
                    if (probeDelim >= 0) { machine.ScheduleKeystroke((byte)probeDelim, at, probeHold); at += probeGap; }
                }
                nextWatch = probeAt + probeGap - 1; watchStation = probeFrom;
            }
            bool pressed1 = pokeAt <= 0;
            bool pressed2 = pokeAt <= 0 || keyDelay <= 0;
            while (machine.IopInstructions < budget)
            {
                if (ejectAt >= 0 && machine.IopInstructions == ejectAt)
                {
                    machine.EjectFloppy(0);
                    Console.WriteLine("  floppy 0 EJECTED @IOP instruction " + machine.IopInstructions);
                }
                if (changeAt >= 0 && machine.IopInstructions == changeAt && !string.IsNullOrEmpty(changeTo))
                {
                    machine.ChangeFloppy(0, changeTo);
                    Console.WriteLine("  floppy 0 CHANGE started @IOP instruction " + machine.IopInstructions
                                      + " (drive empty for " + DoovkeMachine.DiskChangeGapSeconds + "s, then -> " + changeTo + ")");
                }
                if (ipw != null && (machine.IopInstructions % ipEvery) == 0)
                    ipw.WriteLine(machine.IopInstructions + " " + machine.Iop.InstructionAddress.ToString("X5"));
                machine.Step();
                if (machine.IopInstructions >= nextWatch && watchStation >= 0)
                {
                    int ww, hh; byte[] fr = machine.RenderFrame(out ww, out hh);
                    int cnt = 0; if (fr != null) foreach (var b in fr) if (b != 0) cnt++;
                    if (lastCount >= 0 && cnt != lastCount)
                        Console.WriteLine("  *** station " + watchStation + " CHANGED the screen: "
                                          + lastCount + " -> " + cnt + " pixels");
                    lastCount = cnt;
                    watchStation++; nextWatch += probeGap;
                    if (watchStation > probeTo) { watchStation = -1; nextWatch = long.MaxValue; }
                }
                if ((machine.IopInstructions & 0x3F) == 0 && machine.Io.Led != lastLed)
                {
                    lastLed = machine.Io.Led;
                    Console.WriteLine(String.Format("  MP {0:X4} @IOP {1} CP {2}",
                        lastLed, machine.IopInstructions, machine.Cp.InstructionCount));
                }
                if ((machine.IopInstructions & 0xFF) == 0)
                {
                    int ctl = machine.Display.ControlRegister;
                    if (ctl != lastCtl)
                    {
                        lastCtl = ctl;
                        Console.WriteLine(String.Format(
                            "  DDC control EC80 = {0:X2}  (mix={1:X1} video={2} nonIlace={3} force={4}) @IOP {5} CP {6}",
                            ctl, (ctl >> 4) & 0xF, (ctl & 2) != 0, (ctl & 1) != 0, (ctl & 8) != 0,
                            machine.IopInstructions, machine.Cp.InstructionCount));
                    }
                }
                if (!pressed1 && machine.IopInstructions >= pokeAt)
                {
                    machine.QueueKey(pokeCode, true); if (keyRelease) machine.QueueKey(pokeCode, false);
                    Console.WriteLine("  station " + pokeCode + " press 1 @IOP instruction " + machine.IopInstructions);
                    pressed1 = true;
                }
                else if (pressed1 && !pressed2 && machine.IopInstructions >= pokeAt + keyDelay)
                {
                    machine.QueueKey(pokeCode, true); if (keyRelease) machine.QueueKey(pokeCode, false);
                    Console.WriteLine("  station " + pokeCode + " press 2 @IOP instruction " + machine.IopInstructions);
                    pressed2 = true;
                }
            }

            if (ipw != null) { ipw.Flush(); ipw.Close(); Console.WriteLine("IP trace -> " + ipTrace); }

            Console.WriteLine();
            Console.WriteLine("Stopped after " + machine.IopInstructions + " IOP instructions"
                              + " (CP executed " + machine.Cp.InstructionCount + ")"
                              + (machine.Halted ? " [IOP halted]" : string.Empty));

            Console.WriteLine("  final state: " + machine.Iop);
            var cpLog = machine.Io.CpLoadLog;
            Console.WriteLine("CP load/control events: " + (cpLog == null ? 0 : cpLog.Count));
            if (cpLog != null && cpLog.Count > 0)
                Console.WriteLine("  " + string.Join("  ", cpLog.GetRange(0, Math.Min(24, cpLog.Count))));
            Console.WriteLine("FDC commands: " + machine.Io.Fdc.CommandCount
                              + "   no-media stalls: " + machine.Io.Fdc.NoMediaStalls
                              + "   FDC resets: " + machine.Io.Fdc.ResetCount);
            var eeLog = machine.Io.ConfigEeprom.ReadLog;
            Console.WriteLine("Config EEPROM reads: " + (eeLog == null ? 0 : eeLog.Count));
            Console.WriteLine("Control store lane writes: " + machine.Io.ControlStore.LaneWrites);

            int w, h;
            byte[] frame = machine.RenderFrame(out w, out h);
            if (frame != null && w > 0)
            {
                using (var fs = new FileStream(fbOut, FileMode.Create))
                using (var bw = new BinaryWriter(fs))
                {
                    bw.Write(w);
                    bw.Write(h);
                    bw.Write(frame);
                }
                int set = 0;
                foreach (var b in frame) if (b != 0) set++;
                Console.WriteLine(string.Format("Framebuffer {0}x{1} -> {2}  ({3} of {4} pixels set)",
                                                w, h, fbOut, set, frame.Length));
            }
            else
            {
                Console.WriteLine("Display not programmed -- no framebuffer to write.");
            }

            return 0;
        }

        private static void Usage()
        {
            Console.WriteLine();
            Console.WriteLine("usage: Doovke --rom <bootrom.bin> [options]");
            Console.WriteLine("  --eeprom <file>   config EEPROM image (93C46)");
            Console.WriteLine("  --floppy <file>   mount an image in drive 0");
            Console.WriteLine("  --budget <n>      IOP instructions to run (default 150000000)");
            Console.WriteLine("  --key-at <n>      inject a keystroke at IOP instruction n");
            Console.WriteLine("  --key <hex>       scan code in HEX (boot device = 0x63 + icon index)");
            Console.WriteLine("  --key-delay <n>   gap before the second press (default 2000000)");
            Console.WriteLine("  --headless        run without the window (batch/regression)");
            Console.WriteLine("  --fb <file>       framebuffer output (default doovke_fb.bin)");
            Console.WriteLine("  --eject-at <n>    eject drive 0 at IOP instruction n");
            Console.WriteLine("  --change-at <n>   start a disk change at n (with --change-to)");
            Console.WriteLine("  --change-to <f>   image to insert after the no-media gap");
        }
    }
}
