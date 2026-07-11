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
    /// Headless bring-up of the Dove IOP: runs the real Opie boot firmware from the
    /// boot ROM and reports where it settles.  This is the in-application counterpart
    /// of the standalone core trace; it is invoked from Program.Main when the machine
    /// type is Daybreak/Daisy, before the (DLion-only) display UI comes up.  It exists
    /// to exercise the 80186 IOP inside the real build until the Dove display / CP
    /// bridge are implemented and the machine can run under the normal UI.
    /// </summary>
    public static class DoveBringup
    {
        public static void Run()
        {
            string romPath = Configuration.DoveBootRom;
            if (string.IsNullOrWhiteSpace(romPath))
            {
                Console.WriteLine("Dove bring-up: no boot ROM configured (set DoveBootRom in the config).");
                return;
            }

            Console.WriteLine("Dove (6085/Daybreak) IOP bring-up");
            Console.WriteLine("Boot ROM: " + romPath);

            DoveIOProcessor iop;
            try
            {
                iop = new DoveIOProcessor(romPath);
            }
            catch (Exception e)
            {
                Console.WriteLine("Dove bring-up failed to initialize: " + e.Message);
                return;
            }

            i80186 cpu = iop.CPU;

            const long budget = 12_000_000;
            const long epoch = 1_000_000;
            var epochAddrs = new HashSet<int>();
            long instr = 0;
            int prevDistinct = -1, stable = 0;
            string outcome = "did not settle within the instruction budget";

            while (instr < budget)
            {
                epochAddrs.Add(cpu.InstructionAddress);

                try
                {
                    iop.Execute();
                }
                catch (Exception e)
                {
                    outcome = "exception: " + e.Message;
                    break;
                }

                instr++;

                if (instr % epoch == 0)
                {
                    int distinct = epochAddrs.Count;
                    if (distinct < 400 && distinct == prevDistinct)
                    {
                        if (++stable >= 2)
                        {
                            outcome = String.Format("reached the Opie scheduler idle loop ({0} addresses around {1:X5})",
                                distinct, cpu.InstructionAddress);
                            break;
                        }
                    }
                    else stable = 0;
                    prevDistinct = distinct;
                    epochAddrs.Clear();
                }
            }

            Console.WriteLine("Result: " + outcome);
            Console.WriteLine(String.Format("Instructions: {0}   Heartbeat interrupts: {1} (last vector 0x{2:X2})",
                instr, cpu.HardwareInterruptCount, cpu.LastHardwareVector));
            Console.WriteLine("Final CPU state: " + cpu);
        }
    }
}
