/*
    BSD 2-Clause License

    Copyright Vulcan Inc. 2017-2018 and Living Computer Museum + Labs 2018
    All rights reserved.
*/

using System;
using System.Collections.Generic;
using D.IO;

namespace D.IOP
{
    /// <summary>
    /// Intel 8272A / NEC uPD765 floppy-disk controller, as wired on the Dove/6085
    /// IOP (IOP-TR section 7).  Presents the programmer-visible registers to the
    /// 80186:
    ///   0x50  Main Status Register (read-only)   -- A0=0
    ///   0x52  Data Register (command/result FIFO) -- A0=1
    ///   0x54  DMA data port (execution-phase byte stream on DMA channel 0)
    /// plus reset via the reset-control register (0xC0 bit 2) and the shared IOP
    /// control register ("Port80", 0x80: motor / drive-select / AllowTmrTC).
    ///
    /// The controller runs the standard three-phase protocol -- Command (host
    /// writes N opcode/parameter bytes), Execution (data transfer, DMA), Result
    /// (host reads the status bytes) -- and raises INT on the slave 8259 IR4 at the
    /// end of each read/write and each async seek/recalibrate.  The 9229 data
    /// separator is analog-only and modelled as a no-op (the media layer holds
    /// already-decoded bytes).
    /// </summary>
    public class I8272
    {
        // ---- Main Status Register bits (Fig 7.7). ----
        private const int RQM = 0x80;   // request for master (byte ready)
        private const int DIO = 0x40;   // 1 = FDC->CPU (result), 0 = CPU->FDC (command)
        private const int NDM = 0x20;   // non-DMA execution
        private const int CB  = 0x10;   // controller busy

        private enum Phase { Command, Execution, Result }
        private Phase _phase = Phase.Command;

        private readonly byte[] _cmd = new byte[9];
        private int _cmdLen, _cmdIdx;
        private readonly byte[] _result = new byte[7];
        private int _resLen, _resIdx;

        private bool _int;                 // INT pin -> slave 8259 IR4
        private int _presentCyl;           // PCN, current head position
        private byte _st0;                 // last ST0 for Sense Interrupt Status
        private int _resetSenses;          // pending post-reset drive-status senses
        private bool _seekPending;         // a seek/recalibrate INT awaits Sense Int Status

        // Specify parameters (retained across reset per the manual).
        private byte _srtHut, _hltNd;

        // Execution-phase data (Read Data / Read ID).
        private byte[] _execData;
        private int _execIdx;

        /// <summary>Optional attached media: 4 double-sided drives (IMD images).</summary>
        public FloppyDisk[] Drives = new FloppyDisk[4];

        // ---- Media presence ------------------------------------------------------------
        // The stock 6085 5.25" drive leaves the READY / disk-change line OPEN (undriven), so
        // ST3.Ready and ST0.ReadyLineChangedDuringCommandExecution NEVER fire on this machine;
        // those guest branches exist defensively only.  Presence is really sensed through index
        // pulses: no diskette => platter not spinning => no index => the 8272 never finds an
        // address mark => the command SIMPLY DOES NOT COMPLETE.  The IOP's software timeout
        // (PFloppy.ASM:32/596/607-621) then fakes a CRC error and forces a retry, which surfaces
        // as iocb.TimeoutOccurred -> FloppyHeadDoveB.mesa:229 RETURN[notReady] -- the first status
        // check, before any ST0/ST3 read.
        //
        // ** This must be a STALL, not a missing-address-mark result. **  A missing AM decodes as
        // recordNotFound (FloppyHeadDoveB.mesa:276), a different path that never reads as a media
        // change, so the guest would keep the old disk's identity.
        //
        // Note the distinction preserved below: NO DISK => stall; disk present but the track is
        // blank/unformatted => the existing missing-AM result, which is correct for that case.
        private bool _noMedia;                       // a media-requiring command hit an empty drive
        public long NoMediaStalls;                   // diagnostic: how many commands stalled

        /// <summary>Only drive 0 physically exists on this FDC (PFloppy.ASM:990); any other
        /// unit select behaves as an empty drive and times out.</summary>
        public bool HasMedia(int unit) { return unit == 0 && Drives[0] != null; }

        private static bool NeedsMedia(int op)
        {
            switch (op)
            {
                case 0x02:   // Read Track
                case 0x05:   // Write Data
                case 0x06:   // Read Data
                case 0x09:   // Write Deleted Data
                case 0x0A:   // Read ID
                case 0x0C:   // Read Deleted Data
                case 0x0D:   // Format Track
                case 0x11: case 0x19: case 0x1D:   // Scan
                    return true;
                default:
                    // Specify / Sense Interrupt / Sense Drive are FDC-internal, and
                    // Recalibrate/Seek are drive-mechanical (track-0 sensing works with no
                    // diskette), so all of those still complete normally on an empty drive.
                    return false;
            }
        }

        /// <summary>Model an empty drive: leave the command in execution forever, with no
        /// interrupt and no result phase, so the IOP's software timeout is what fires.</summary>
        private void StallNoMedia()
        {
            NoMediaStalls++;
            _noMedia = true;
            _phase = Phase.Execution;
            _execData = null; _execIdx = 0;
            // deliberately: no _int, no StartResult -- the command never completes.
        }

        // No constructor reset: the controller powers up idle (RQM=1, no INT).  The
        // boot driver issues the explicit 0xC0-bit2 reset when it wants the
        // drive-status senses + reset interrupt.

        /// <summary>INT pin state -> drive slave 8259 IR4.</summary>
        public bool Interrupt { get { return _int; } }

        /// <summary>
        /// When set, Read Data drives the execution-phase byte stream through this
        /// callback (the 80186 DMA channel 0 → main memory) instead of leaving it for
        /// port-0x54 reads, then the transfer terminates and the result/INT fire.
        /// </summary>
        public System.Func<byte[], int> DmaOut;

        /// <summary>Hard reset (reset-control-register bit 2 low): FDC -> idle, then
        /// posts a drive-status interrupt for each of the 4 drives.  The last Specify
        /// is NOT cleared (manual section 7.3.2).</summary>
        public int ResetCount;

        public void Reset()
        {
            ResetCount++;
            _noMedia = false;      // the timeout handler resets the FDC to recover
            _phase = Phase.Command;
            _cmdIdx = _cmdLen = _resIdx = _resLen = 0;
            _execData = null; _execIdx = 0;
            _presentCyl = 0;
            _resetSenses = 4;      // the reset routine senses all 4 drives until ST0=80H
            _seekPending = false;
            _int = true;           // FDC asserts INT after reset
        }

        // ---- Register access ----

        /// <summary>Read the Main Status Register (port 0x50).</summary>
        public byte ReadMainStatus()
        {
            // Stalled on an empty drive: permanently busy, never ready for a transfer, so the
            // host's poll loop runs out its retries.  Only an FDC reset clears this.
            if (_noMedia) return (byte)CB;
            int s = RQM;
            if (_phase == Phase.Result) s |= DIO | CB;
            else if (_phase == Phase.Execution) s |= CB | DIO;   // exec data reads FDC->CPU
            else if (_cmdIdx > 0) s |= CB;                       // mid-command
            return (byte)s;
        }

        /// <summary>Read the Data Register (port 0x52): a result byte, or an
        /// execution byte in non-DMA mode.</summary>
        public byte ReadData()
        {
            if (_phase == Phase.Result)
            {
                if (_resIdx == 0) _int = false;   // reading the result clears INT
                byte b = _result[_resIdx++];
                _resRead++;                        // count harvested result bytes (per result phase)
                if (_resIdx >= _resLen) GoIdle();
                return b;
            }
            if (_phase == Phase.Execution && _execData != null)
            {
                byte b = _execIdx < _execData.Length ? _execData[_execIdx] : (byte)0;
                _execIdx++;
                if (_execIdx >= _execData.Length) FinishExecution();
                return b;
            }
            return 0xFF;
        }

        /// <summary>Read a byte through the DMA data port (0x54) during execution.</summary>
        public byte ReadDmaData()
        {
            if (_phase == Phase.Execution && _execData != null)
            {
                byte b = _execIdx < _execData.Length ? _execData[_execIdx] : (byte)0;
                _execIdx++;
                if (_execIdx >= _execData.Length) FinishExecution();
                return b;
            }
            return 0xFF;
        }

        /// <summary>Terminal count (from 80186 timer 1) ends the current transfer.</summary>
        public void TerminalCount()
        {
            if (_phase == Phase.Execution) FinishExecution();
        }

        /// <summary>Write the Data Register (port 0x52): the next command byte.</summary>
        public void WriteData(byte v)
        {
            if (_phase != Phase.Command) return;   // ignore during exec/result

            if (_cmdIdx == 0)
            {
                _cmd[0] = v;
                _cmdLen = CommandBytes(v);
                _cmdIdx = 1;
            }
            else if (_cmdIdx < _cmd.Length)
            {
                _cmd[_cmdIdx++] = v;
            }

            if (_cmdIdx >= _cmdLen) Execute();
        }

        // ---- Command byte counts (App B / NumberOfFDCCommandBytes) ----
        private static int CommandBytes(byte opcode)
        {
            switch (opcode & 0x1F)
            {
                case 0x02: return 9;   // Read a Track
                case 0x03: return 3;   // Specify
                case 0x04: return 2;   // Sense Drive Status
                case 0x05: return 9;   // Write Data
                case 0x06: return 9;   // Read Data
                case 0x07: return 2;   // Recalibrate
                case 0x08: return 1;   // Sense Interrupt Status
                case 0x09: return 9;   // Write Deleted Data
                case 0x0A: return 2;   // Read ID
                case 0x0C: return 9;   // Read Deleted Data
                case 0x0D: return 6;   // Format a Track
                case 0x0F: return 3;   // Seek
                case 0x11: case 0x19: case 0x1D: return 9;   // Scan
                default: return 1;     // Invalid
            }
        }

        /// <summary>Optional diagnostic log of executed commands.</summary>
        public System.Collections.Generic.List<string> Log;

        /// <summary>Diagnostic counters (total across the whole run, uncapped).</summary>
        public long CommandCount;
        public long ReadCount;
        /// <summary>Last Read's cyl/head/sector, for trajectory tracing.</summary>
        public int LastReadC = -1, LastReadH = -1, LastReadR = -1;

        /// <summary>
        /// Ring buffer of the LAST N commands.  `Log` above caps at the FIRST 60, which shows the boot's
        /// opening moves but is useless for a stall: the germ issues 115 Read commands, stops requesting
        /// data entirely, and then polls forever (CommandCount climbs into the thousands while ReadCount
        /// stays pinned at 115 -- NB ReadCount counts Reads ISSUED, not sectors delivered, so a frozen
        /// ReadCount means the germ stopped ASKING, not that reads are failing).  This ring answers what
        /// the loop actually consists of.
        /// </summary>
        public string[] RecentLog;
        public long RecentPos;
        /// <summary>Every Read Data command's C/H/R/EOT + outcome (missing-track abnormal vs bytes gathered).
        /// The germ's boot read stalls when it requests a track not in the IMD image (t==null -> ST0=0x40,
        /// ST1=0x01) -> the boot channel sees status != goodCompletion -> cGermDeviceError (921).  This trace
        /// names the exact failing (cylinder,head,sector) so the DMK->IMD geometry gap is visible.</summary>
        public System.Collections.Generic.List<string> ReadTrace;
        /// <summary>Every FDC command (Seek/Sense/Read/...) with the present-cylinder (PCN) and result bytes,
        /// capped to the boot's opening ~950 commands so the C36->C5 BACKWARD-seek transition (~cmd 630) is
        /// captured. Answers: does the firmware issue Seek(0x0F,5)? what is PCN at the C5 ReadData? does the
        /// read return success (ST0=00) or ST2.WC/ST1.ND?</summary>
        public System.Collections.Generic.List<string> CmdTrace;
        private void Recent(string s)
        {
            if (RecentLog == null) return;
            RecentLog[(int)(RecentPos % RecentLog.Length)] = "#" + CommandCount + " " + s;
            RecentPos++;
        }
        /// <summary>The ring, oldest-first, for printing.</summary>
        public System.Collections.Generic.List<string> RecentInOrder()
        {
            var outp = new System.Collections.Generic.List<string>();
            if (RecentLog == null) return outp;
            int n = RecentLog.Length;
            long start = RecentPos > n ? RecentPos - n : 0;
            for (long i = start; i < RecentPos; i++)
            { var v = RecentLog[(int)(i % n)]; if (v != null) outp.Add(v); }
            return outp;
        }

        private static readonly string[] Names = new string[32] {
            "?","?","ReadTrack","Specify","SenseDrive","Write","Read","Recalibrate",
            "SenseInt","WriteDel","ReadID","?","ReadDel","Format","?","Seek",
            "?","ScanEq","?","?","?","?","?","?","?","ScanLo","?","?","?","ScanHi","?","?" };

        // ---- Execution ----
        private void Execute()
        {
            int op = _cmd[0] & 0x1F;
            int prevHarvest = _resRead;   // result bytes the host harvested from the PREVIOUS command's result phase
            CommandCount++;
            if (op == 0x06) { ReadCount++; LastReadC = _cmd[2]; LastReadH = _cmd[3]; LastReadR = _cmd[4]; }
            if (Log != null && Log.Count < 60)
                Log.Add(Names[op] + "(" + BitConverter.ToString(_cmd, 0, _cmdLen) + ")");
            Recent(Names[op] + "(" + BitConverter.ToString(_cmd, 0, _cmdLen) + ")");
            int unit = _cmd.Length > 1 ? (_cmd[1] & 0x03) : 0;
            int head = _cmd.Length > 1 ? ((_cmd[1] >> 2) & 0x01) : 0;

            // Empty drive (or a select of the non-existent drive 1): stall instead of
            // completing, so detection happens via the IOP timeout rather than a status bit.
            if (NeedsMedia(op) && !HasMedia(unit)) { StallNoMedia(); return; }

            switch (op)
            {
                case 0x03:   // Specify -- no result, no INT
                    _srtHut = _cmd[1];
                    _hltNd = _cmd[2];
                    GoIdle();
                    break;

                case 0x07:   // Recalibrate -- head to track 0, async INT, no result
                    _presentCyl = 0;
                    _st0 = (byte)(0x20 | unit);           // SeekEnd | drive
                    _seekPending = true;
                    _int = true;
                    GoIdle();
                    break;

                case 0x0F:   // Seek -- head to NCN, async INT, no result
                    _presentCyl = _cmd[2];
                    _st0 = (byte)(0x20 | (head << 2) | unit);
                    _seekPending = true;
                    _int = true;
                    GoIdle();
                    break;

                case 0x08:   // Sense Interrupt Status -- result ST0, PCN
                    _int = false;
                    if (_resetSenses > 0)
                    {
                        int drv = 4 - _resetSenses;
                        _result[0] = (byte)(0xC0 | drv);  // IC=ReadyChanged | drive
                        _result[1] = 0x00;
                        _resetSenses--;
                        if (Log != null && Log.Count > 0) Log[Log.Count - 1] += "->ST0=" + _result[0].ToString("X2");
                        StartResult(2);
                    }
                    else if (_seekPending)
                    {
                        _result[0] = _st0;
                        _result[1] = (byte)_presentCyl;
                        _seekPending = false;
                        if (Log != null && Log.Count > 0) Log[Log.Count - 1] += "->ST0=" + _result[0].ToString("X2");
                        StartResult(2);
                    }
                    else
                    {
                        // No interrupt pending: the 8272 treats this like an invalid
                        // command and returns ONLY ST0=0x80 (one result byte), not two.
                        _result[0] = 0x80;
                        if (Log != null && Log.Count > 0) Log[Log.Count - 1] += "->ST0=80(1b)";
                        StartResult(1);
                    }
                    break;

                case 0x04:   // Sense Drive Status -- result ST3
                    _result[0] = St3(unit, head);
                    StartResult(1);
                    break;

                case 0x0A:   // Read ID -- result ST0,ST1,ST2,C,H,R,N
                    {
                        FloppyDisk d = Drives[unit];
                        Track t = (d != null && _presentCyl < 77) ? d.GetTrack(_presentCyl, head) : null;
                        byte st1 = (t != null) ? (byte)0 : (byte)0x01;   // MissingAddressMark if no track
                        _st0 = (byte)((t != null ? 0x00 : 0x40) | (head << 2) | unit);
                        _result[0] = _st0; _result[1] = st1; _result[2] = 0;
                        _result[3] = (byte)_presentCyl; _result[4] = (byte)head;
                        _result[5] = 1;                                  // R = first sector
                        _result[6] = (byte)(t != null ? SizeCode(t.SectorSize) : 2);
                        _int = true;
                        StartResult(7);
                    }
                    break;

                case 0x06:   // Read Data -- DMA the sector(s), then result
                    ReadData(unit, head);
                    break;

                default:     // Invalid command
                    _st0 = 0x80;
                    _result[0] = 0x80;
                    StartResult(1);
                    break;
            }
            if (CmdTrace != null && CmdTrace.Count < 950)
            {
                int cc = _cmd.Length > 2 ? _cmd[2] : -1;
                int rr = _cmd.Length > 4 ? _cmd[4] : -1;
                int eot = _cmd.Length > 6 ? _cmd[6] : -1;
                CmdTrace.Add("#" + CommandCount + " op=0x" + op.ToString("X2") + "(" + (op < Names.Length ? Names[op] : "?")
                    + ") cmdC=" + cc + " H=" + head + " R=" + rr + " EOT=" + eot
                    + " | PCN=" + _presentCyl + " ST0=" + _st0.ToString("X2")
                    + " res[0/1/2]=" + _result[0].ToString("X2") + "/" + _result[1].ToString("X2") + "/" + _result[2].ToString("X2")
                    + " resC=" + _result[3] + " resR=" + _result[5]
                    + "  [prevResHarvest=" + prevHarvest + "]");
            }
        }

        // Cmd bytes for Read: C(2) H(3) R(4) N(5) EOT(6) GPL(7) DTL(8).
        private void ReadData(int unit, int head)
        {
            int c = _cmd[2], r = _cmd[4], n = _cmd[5], eot = _cmd[6];
            FloppyDisk d = Drives[unit];
            Track t = (d != null && c < 77) ? d.GetTrack(c, head) : null;

            if (t == null)
            {
                // No track/media: abnormal termination, missing address mark.
                _st0 = (byte)(0x40 | (head << 2) | unit);   // IC=AbnormalTermination
                SetReadResult(0x01, 0x00, c, head, r, n);   // ST1 MissingAddressMark
                if (ReadTrace != null && ReadTrace.Count < 600)
                    ReadTrace.Add("Read#" + CommandCount + " C=" + c + " H=" + head + " R=" + r + " EOT=" + eot + " N=" + n
                        + " -> **MISSING-TRACK ABNORMAL** ST0=" + _st0.ToString("X2") + " ST1=01  [c<77=" + (c < 77) + " getTrack=null]");
                _int = true;
                StartResult(7);
                return;
            }

            // Gather sectors R..EOT on this track into the execution buffer; the
            // DMA/TC handshake decides how many bytes are actually consumed.
            var buf = new List<byte>();
            GatherSectors(d, c, head, r, eot, buf);
            // Multi-Track (MT, bit7 of the command): after the last sector on this
            // head, the read continues on the OTHER head of the SAME cylinder from
            // sector 1 through EOT.  Without this a multi-track read returns only
            // half the requested data; the germ's boot-file inLoad issues MT reads
            // (opcode 0xC6) and, if under-delivered, its DMA count never drains.
            bool mt = (_cmd[0] & 0x80) != 0;
            if (mt) GatherSectors(d, c, head ^ 1, 1, eot, buf);
            if (ReadTrace != null && ReadTrace.Count < 600)
                ReadTrace.Add("Read#" + CommandCount + " C=" + c + " H=" + head + " R=" + r + " EOT=" + eot + " N=" + n
                    + " MT=" + (mt ? 1 : 0) + " -> ok gathered=" + buf.Count + "B");
            _execData = buf.ToArray();
            _execIdx = 0;
            _phase = Phase.Execution;

            _pendC = (byte)c; _pendH = (byte)head; _pendN = (byte)n; _pendUnit = unit;
            _readR = r; _readEot = eot; _pendMt = mt;

            // DMA mode: push the transfer to main memory now (bounded by the DMA
            // count), then terminate with a result that reflects how much was read.
            if (DmaOut != null)
            {
                int sent = DmaOut(_execData);
                _dmaSent = sent > 0 ? sent : _execData.Length;
                FinishExecution();
            }
        }

        // Append sectors [first..last] of (cylinder, head) to buf.  Missing sectors
        // are skipped (their absence shows up as a short buffer, not an exception).
        private void GatherSectors(FloppyDisk d, int c, int head, int first, int last, List<byte> buf)
        {
            int count = (last >= first) ? (last - first + 1) : 1;
            for (int i = 0; i < count; i++)
            {
                Sector sec = null;
                try { sec = d.GetSector(c, head, (first + i) - 1); } catch { }
                if (sec != null) buf.AddRange(sec.Data);
            }
        }

        private static byte SizeCode(int bytes)
        {
            switch (bytes) { case 128: return 0; case 256: return 1; case 512: return 2;
                             case 1024: return 3; case 2048: return 4; default: return 2; }
        }

        private byte _pendR, _pendC, _pendH, _pendN; private int _pendUnit; private bool _pendMt;
        private int _readR, _readEot, _dmaSent;

        private void FinishExecution()
        {
            int secBytes = 128 << (_pendN & 0x07);
            int secsRead = System.Math.Max(1, (_dmaSent + secBytes - 1) / secBytes);
            // Result C/H/R = the sector AFTER the last one transferred (8272 spec).
            int rr = _readR + secsRead;
            int cc = _pendC, hh = _pendH;
            if (rr > _readEot)
            {
                // Past end-of-track.  Multi-Track wraps head0->head1 on the SAME cylinder, then
                // head1->head0 on the NEXT cylinder (8272 MT result rules).  Non-MT just advances
                // the cylinder with the head unchanged.  UpdateOperation feeds ResultBytes[C/H/R]
                // straight into the germ's next disk address, so a non-MT rollover on an MT read
                // (old bug: R9 -> C+1 instead of C5/H1/R1) misdirects the boot read.
                rr = 1;
                if (_pendMt) { if (_pendH == 0) hh = 1; else { hh = 0; cc = _pendC + 1; } }
                else cc = _pendC + 1;
            }
            _st0 = (byte)((_pendH << 2) | _pendUnit);         // termination head = the head actually read
            SetReadResult(0x00, 0x00, cc, hh, rr, _pendN);
            _execData = null; _execIdx = 0; _dmaSent = 0;
            _int = true;
            StartResult(7);
        }

        private void SetReadResult(byte st1, byte st2, int c, int h, int r, int n)
        {
            _result[0] = _st0; _result[1] = st1; _result[2] = st2;
            _result[3] = (byte)c; _result[4] = (byte)h; _result[5] = (byte)r; _result[6] = (byte)n;
        }

        // ST3 (Sense Drive Status): drive is ready + two-sided; track0 if at cyl 0.
        private byte St3(int unit, int head)
        {
            int s = (head << 2) | unit;
            FloppyDisk d = Drives[unit];
            if (d != null) s |= 0x20;         // Ready (media present)
            s |= 0x08;                        // TwoSided
            if (_presentCyl == 0) s |= 0x10;  // Track0
            if (d != null && d.IsWriteProtected) s |= 0x40;
            return (byte)s;
        }

        private void StartResult(int len)
        {
            _resLen = len; _resIdx = 0; _resRead = 0;   // new result phase: reset harvest counter
            _phase = Phase.Result;
        }
        private int _resRead;   // result bytes the host has read in the current/last result phase

        private void GoIdle()
        {
            _phase = Phase.Command;
            _cmdIdx = 0; _cmdLen = 0;
        }
    }
}
