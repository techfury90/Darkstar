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

using D.CP;
using D.Logging;
using System;

namespace D.IO
{
    /// <summary>
    /// Implements the HSIO-L "Large Disk" (Trident) controller of the Large-Capacity
    /// servers.  The register semantics are those of the DLion Trident microcode
    /// (TridentDLion.mc, TridentInitial.mc, TridentBootDLion.mc) with constant values
    /// from Dandelion.dfn:
    ///
    ///   KCmd  = [tag/control byte &lt;&lt; 8 | drive bus]:
    ///           0x8000 CylTag, 0x4000 HdTag, 0x2000 CtlTag (0-&gt;1 edge strobes the
    ///           bus into the drive), 0x0800 ctlTest, 0x0400 firmwareEnable.
    ///           Bus (CtlTag): 0x40 Read gate (also resets Attention), 0x08|0x02
    ///           Reset/Recalibrate, 0x04 head-select enable, 0x01 head advance.
    ///           Bus (CylTag): cylinder number.  Bus (HdTag): head number.
    ///
    ///   KCtl  = [enable nibble | drive-select one-hot nibble | LdSeq/WU/routine]:
    ///           bits 11:8 select the drive (0x8=drive 0 .. 0x1=drive 3), bit 7 loads
    ///           the word sequencer with the routine in bits 3:0, and bits 6:4 select
    ///           the wakeup condition: 1=AnyAttention, 2=IndexFound, 3=SectorFound,
    ///           4=DataRequest, 6=DiskReady, 0=none (firmwareEnable wakes).
    ///
    ///   KStatus (read inverted): 0x01 DiskCheck, 0x08 NotReady, 0x20/0x40 head-select
    ///           probe bits (a T-80 flags an illegal head where a T-300 does not --
    ///           TridentInitial's drive-type test), 0x80 AnyAttention, attention
    ///           one-hot per drive in bits 11:8.
    ///
    ///   KTest:  0x0001 controller present (the Phase0 Trident/SA detection gate, the
    ///           HSIO-e "WriteGate" bit), 0x0200|0x0400 error latches (read inverted:
    ///           1 = no error), 0x1000 firmware-not-busy (tracks ctlTest).
    ///
    ///   KIData/KOData move one word per DataRequest; KStrobe acknowledges a request.
    ///
    /// Data framing is self-paced per word: every transfer loop in TridentDLion.mc
    /// (WrVrfLp/RdLp/DtRdLp/NoopLp) runs the same three beats per word -- KStrobe
    /// (consume/ack), move the word (KOData&lt;-MD / MDR&lt;-KIData), CANCELBR-sleep until
    /// the next DataRequest -- and the hardware generates n+1 DataRequests for an
    /// n-word field (the extra one exits the loop).  The next request therefore
    /// follows the microcode's KStrobe rather than a free-running word clock (the
    /// Disk task only gets one click per 5-click round, ~2.06us, slower than the
    /// real 1.65us word rate).  Only the rotational pulses (index/sector) run at the
    /// true cadence.  Fields frame in sector order header -&gt; label -&gt; data (then the
    /// ECC syndrome readout), and a field that has never been written has no
    /// preamble/sync: a read gate over it produces no DataRequests at all.
    /// </summary>
    public class TridentController : IDiskController
    {
        public TridentController(DSystem system, TridentDrive drive)
        {
            _system = system;
            _drive = drive;

            //
            // The rotational (sector pulse) event runs continuously at the drive's
            // real cadence.
            //
            _system.Scheduler.Schedule(SectorTimeNsec, SectorCallback);
        }

        /// <summary>
        /// A drive spinning up raises Attention once its heads settle at cylinder 0
        /// (power-on sequence complete).  Called when a pack is loaded so the disk
        /// firmware and the diagnostics see the unit announce itself.
        /// </summary>
        public void SignalDrivePowerOn()
        {
            if (_drive.IsLoaded)
            {
                _seekSettled = true;
                _attention |= (1 << 3);     // drive 0
                EvaluateWakeup();
            }
        }

        public void Reset()
        {
            _kCmd = 0;
            _kCtl = 0;
            _driveSelect = -1;
            _wakeup = Wakeup.None;
            _sequencerRoutine = 0;

            _readGate = false;
            _writeGate = false;

            _sectorFound = false;
            _indexFound = false;
            _dataRequest = false;
            _attention = 0;
            _diskCheck = false;
            _illegalHead = false;
            _eccError = false;
            _overrun = false;
            _verifyError = false;
            _seekSettled = false;

            _currentField = Field.None;
            _fieldWordIndex = -1;
            _xferActive = false;
            _xferPending = false;
            _cursor = Field.Header;
            _seqReadSector = -1;
        }

        #region IDiskController (the CP's K functions)

        public void SetKCmd(ushort value)
        {
            ushort lastCmd = _kCmd;
            _kCmd = value;

            if (Log.Enabled) Log.Write(LogComponent.TridentControl, "KCmd<-0x{0:x4}", value);

            int tag = value & TAG_MASK & ~TERM;

            //
            // Tag lines strobe their data into the drive on the 0->1 edge.
            //
            if (RisingEdge(lastCmd, value, CYL_TAG))
            {
                DoSeek(value & 0x3ff);       // setCyl: cylinder in bits 0x03FF
            }

            if (RisingEdge(lastCmd, value, HD_TAG))
            {
                DoHeadSelect(value & 0x1f);  // setHead: head in bits 0x1F
            }

            if (RisingEdge(lastCmd, value, CTL_TAG))
            {
                DoControl(value & 0xff);
            }

            //
            // Read/write gates (bits 0x80/0x40) are meaningful only for the null and
            // ctl tags -- for setCyl/setHead those bits carry cylinder/offset data.
            //
            if (tag == 0 || tag == CTL_TAG)
            {
                SetGates((value & BUS_READ) != 0, (value & BUS_WRITE) != 0);
            }

            EvaluateWakeup();
        }

        public void SetKCtl(ushort value)
        {
            _kCtl = value;

            if (Log.Enabled) Log.Write(LogComponent.TridentControl, "KCtl<-0x{0:x4}", value);

            //
            // Drive select: one-hot in bits 11:8.
            //
            switch ((value >> 8) & 0xf)
            {
                case 0x8: _driveSelect = 0; break;
                case 0x4: _driveSelect = 1; break;
                case 0x2: _driveSelect = 2; break;
                case 0x1: _driveSelect = 3; break;
                default: _driveSelect = -1; break;
            }

            //
            // Wakeup condition select (bits 6:4).
            //
            _wakeup = (Wakeup)((value >> 4) & 0x7);

            //
            // Word sequencer load (bit 7 + count/routine in bits 6:0; the transfer
            // loops write RCnt here -- 0x03 for the 2-word header, 0x0C for the
            // label).  A load marks a field boundary: it ends any transfer in
            // flight, and with a gate already up it re-arms framing for the next
            // field (chained write fields keep the write gate up throughout, so the
            // load is the only boundary signal).
            //
            if ((value & 0x80) != 0)
            {
                _sequencerRoutine = value & 0xf;

                //
                // "The latches are reset by LdSequencer" (TridentDLion.mc): each
                // field op starts with a load, so its error latches reflect only
                // itself -- this is what keeps a NoopLp's compare pollution from
                // leaking into the next op's status.
                //
                _verifyError = false;
                _eccError = false;
                _overrun = false;

                if (Log.Enabled) Log.Write(LogComponent.TridentControl, "Sequencer load 0x{0:x2}", _sequencerRoutine);

                AbortTransfer();
                if (_readGate || _writeGate)
                {
                    ArmTransfer();
                }
            }

            EvaluateWakeup();
        }

        public void SetKOData(ushort value)
        {
            //
            // A word from the microcode.  In WrVrfLp/NoopLp the KStrobe (c1)
            // precedes the KOData push (c3), so by the time the word arrives the
            // request is normally already acked and _fieldWordIndex names the word
            // just consumed; if the push itself is the first sign of service, ack
            // now.
            //
            if (_dataRequest)
            {
                DataRequestServiced();
            }

            if (_fieldWordIndex >= 0 && _currentField != Field.None
                && _fieldWordIndex < FieldLength(_currentField))
            {
                if (_xferWrite && _currentField == Field.Header && _fieldWordIndex == 0)
                {
                    // Hold the cylinder word until the sector number arrives.
                    _xferPendingHdr0 = value;
                }
                else if (_xferWrite && _currentField == Field.Header && _fieldWordIndex == 1)
                {
                    //
                    // The header a format write pushes names its intended target
                    // {cyl, head<<8|sector} -- and the microcode's op can lag our
                    // rotational attribution (each self-paced 271-word sector op
                    // takes longer than the real sector period, so the next op's
                    // sector pulse lands 1-2 sectors later than the microcode
                    // intends).  The pushed header is authoritative: redirect the
                    // transfer to the named sector.
                    //
                    int target = value & 0xff;
                    if (target < TridentDrive.SectorsPerTrack && target != _xferSector)
                    {
                        if (Log.Enabled) Log.Write(LogComponent.TridentControl,
                            "Write redirect sector {0} -> {1}", _xferSector, target);
                        _xferSector = target;
                    }

                    WriteFieldWord(Field.Header, 0, _xferPendingHdr0);
                    WriteFieldWord(Field.Header, 1, value);
                }
                else if (_xferWrite)
                {
                    WriteFieldWord(_currentField, _fieldWordIndex, value);
                }
                else if (_currentField == Field.Header && _fieldWordIndex == 0)
                {
                    // Verify: hold the expected cylinder word until the sector
                    // number arrives.
                    _xferPendingHdr0 = value;
                }
                else if (_currentField == Field.Header && _fieldWordIndex == 1)
                {
                    //
                    // Header verify: the pushed words name the TARGET sector.  On
                    // real hardware FindHdr orients on the index pulse and counts
                    // sector pulses, so the verify always happens AT the target --
                    // a tries=1 op is safe.  Our rotational attribution starts the
                    // op at whatever sector is passing, so redirect to the named
                    // sector (like the write path) and compare against ITS
                    // content: healthy sectors verify first try, genuinely wrong
                    // content still fails honestly.
                    //
                    int target = value & 0xff;
                    if (target < TridentDrive.SectorsPerTrack && target != _xferSector)
                    {
                        if (Log.Enabled) Log.Write(LogComponent.TridentControl,
                            "Verify redirect sector {0} -> {1}", _xferSector, target);
                        _xferSector = target;
                        _xferVirgin = !_drive.IsFieldWritten(target, TridentDrive.FlagHeaderWritten);
                    }

                    ushort disk0 = ReadFieldWord(Field.Header, 0);
                    ushort disk1 = ReadFieldWord(Field.Header, 1);
                    if (disk0 != _xferPendingHdr0 || disk1 != value)
                    {
                        _verifyError = true;

                        if (Log.Enabled) Log.Write(LogComponent.TridentControl,
                            "Header verify mismatch: disk 0x{0:x4},0x{1:x4} != 0x{2:x4},0x{3:x4}",
                            disk0, disk1, _xferPendingHdr0, value);
                    }
                }
                else
                {
                    //
                    // Label/data verify or a NoopLp orientation push: compare and
                    // latch on mismatch.  NoopLp's zero-pushes over live data also
                    // latch, but harmlessly: every field op starts with
                    // LdSequencer, which resets the error latches, and the
                    // microcode only consults verifyError after ops that verify.
                    //
                    ushort disk = ReadFieldWord(_currentField, _fieldWordIndex);
                    if (disk != value)
                    {
                        _verifyError = true;

                        if (Log.Enabled) Log.Write(LogComponent.TridentControl,
                            "Verify mismatch {0}[{1}]: disk 0x{2:x4} != 0x{3:x4}",
                            _currentField, _fieldWordIndex, disk, value);
                    }
                }
            }
        }

        public ushort ReadKIData()
        {
            ushort value = _kIData;

            //
            // RdLp/DtRdLp strobe (c1) before reading KIData (c2), so this is not
            // normally the ack -- but accept it as one if no strobe came.
            //
            if (_dataRequest)
            {
                DataRequestServiced();
            }

            return value;
        }

        public void KStrobe()
        {
            //
            // The per-word acknowledge: "KStrobe is used to reset Data Requests".
            // Every transfer loop strobes once per word, including the final
            // loop-exit iteration.
            //
            if (_dataRequest)
            {
                DataRequestServiced();
            }
        }

        public void ClrKFlags()
        {
            //
            // Reset the sector/index/ECC latches for the next operation.
            //
            _sectorFound = false;
            _indexFound = false;
            _eccError = false;
            _overrun = false;
            _verifyError = false;

            EvaluateWakeup();
        }

        public ushort ReadKStatus()
        {
            //
            // Raw status, returned inverted ("all status bits are inverted on the X
            // bus", as on the Shugart board).  The microcode reads it back with
            // "~KStatus and MASK", which recovers the raw bit, so in RAW terms a set
            // bit means the healthy/true condition:
            //   0x01 CDskCheck : 1 = no disk check
            //   0x02           : seek/recal complete, no error (part of CTstSkEr=0x0B)
            //   0x08 CNotRdy   : 1 = drive ready/online (TestDskRdy branches DskRdy
            //                    when ~KStatus & 8 is nonzero)
            //   0x200/0x400    : head-select probe bits 9&10, read after selecting
            //                    head 5 to tell a T-80 from a T-300 (TridentInitial)
            //   0x80           : any attention; 0xF00 per-drive attention nibble
            //
            bool present = _driveSelect == 0 && _drive.IsLoaded;
            bool ready = present && _drive.SeekComplete;

            //
            // TDC.DiskStatus (Xerox MSB=bit0 numbering; the software reads this
            // back as DeviceStatus = ~KStatus, so a condition is asserted when the
            // corresponding bit below is SET in this raw word / CLEAR on the wire):
            //   0x8000>>u selected[u]   : the unit acknowledged its select line --
            //                             THE presence bit (EILCDCmdsImpl
            //                             .IsUnitPresent checks exactly this)
            //   0x0800>>u attention[u]  : per-unit attention (seek/recal complete,
            //                             power-on)
            //   0x0080   terminatorIn   : daisy chain terminated
            //   0x0040   endOfCylinder  : also raised for an illegal head address
            //                             (the T-80/T-300 head-5 probe)
            //   0x0020   offsetActive
            //   0x0010   diskBusy       : heads in motion
            //   0x0008   notReady
            //   0x0004   readOnly
            //   0x0002   seekTimeout
            //   0x0001   diskCheck
            // (CTstSkEr/CTstRecalEr = 0x0B = notReady|seekTimeout|diskCheck.)
            //
            int raw =
                (present ? 0x8000 >> _driveSelect : 0x0000) |
                ((_attention & 0xf) << 8) |
                (_drive.IsLoaded ? 0x0080 : 0x0000) |
                (_illegalHead ? 0x0040 : 0x0000) |
                (present && !_drive.SeekComplete ? 0x0010 : 0x0000) |
                (!ready ? 0x0008 : 0x0000) |
                (_diskCheck ? 0x0001 : 0x0000);

            return (ushort)~raw;
        }


        public ushort ReadKTest()
        {
            //
            // TDC.ControllerStatus (like KStatus, conditions read asserted when the
            // wire bit is CLEAR): inProgress=0x8000, goodCompletion=0x4000,
            // firmwareEn=0x1000, verifyError=0x0800, eccError=0x0400,
            // overrun=0x0200, idxFound=0x0100.  The low bit (0x0001) is the
            // WriteGate/controller-present bit Phase0's Trident/SA gate dispatches
            // on (set = Trident, uninverted there).
            //
            int value = 0x0001;

            // Idle: not in progress, last completion good.
            value |= 0x8000;
            if (_diskCheck || _eccError || _overrun || _verifyError)
            {
                value |= 0x4000;    // goodCompletion deasserted on error
            }

            if (!_verifyError)
            {
                value |= 0x0800;
            }

            if (!_eccError)
            {
                value |= 0x0400;
            }

            if (!_overrun)
            {
                value |= 0x0200;
            }

            if (!_indexFound)
            {
                value |= 0x0100;
            }

            //
            // Firmware-busy tracks the firmwareEnable bit: Phase0's TridentWait
            // polls until the boot microcode's EndProc drops it.
            //
            if ((_kCmd & FIRMWARE_ENABLE) == 0)
            {
                value |= 0x1000;
            }

            //
            // idxFound/sectorFound are STICKY latches, cleared only by ClrKFlags
            // (TridentDLion FindHdrLp does "ClrKFlags, BRANCH[SecNotFnd, SecFnd]"
            // -- clears, then branches on the latch persisting from the pulse; a
            // read-clear here breaks the sector-count orientation).
            //


            return (ushort)value;
        }

        #endregion

        #region Drive operations

        private void DoSeek(int cylinder)
        {
            if (Log.Enabled) Log.Write(LogComponent.TridentControl, "Seek to cylinder {0}", cylinder);

            if (_driveSelect != 0 || !_drive.IsLoaded)
            {
                if (Log.Enabled) Log.Write(LogComponent.TridentControl, "Seek with no drive selected/loaded.");
                _diskCheck = true;
                return;
            }

            _seqReadSector = -1;

            int drive = _driveSelect;
            if (!_drive.Seek(cylinder, () => SeekDone(drive)))
            {
                _diskCheck = true;
            }
            EvaluateWakeup();
        }

        private void DoHeadSelect(int bus)
        {
            int head = bus & 0x1f;

            if (head != _drive.Head)
            {
                _seqReadSector = -1;
            }

            //
            // An out-of-range head (the T-80/T-300 probe selects head 5) raises the
            // illegal-head status the drive-type test dispatches on.
            //
            _illegalHead = !_drive.SetHead(head);
        }

        private void DoControl(int bus)
        {
            //
            // CRecal=0x0A decomposes into two independent control bits
            // (TridentBootDLion: "Recal & DvcCkRst"):
            //   0x08 DeviceCheckReset -- clears the check conditions (diskCheck,
            //        endOfCylinder/illegal-head) without moving the heads.  The
            //        diagnostics issue this alone (bus=0x008) after the deliberate
            //        illegal head-5 drive-type probe.
            //   0x02 Recalibrate -- restore the heads to cylinder 0.
            //
            if ((bus & BUS_DVCCHKRST) != 0)
            {
                _diskCheck = false;
                _illegalHead = false;
            }

            if ((bus & BUS_REZERO) != 0)
            {
                if (_driveSelect == 0 && _drive.IsLoaded)
                {
                    int drive = _driveSelect;
                    _drive.Recalibrate(() => SeekDone(drive));
                }
                if (Log.Enabled) Log.Write(LogComponent.TridentControl, "Recalibrate (reZero) issued.");
            }

            // (The read gate / attention-reset for a ctl-tag command with the read
            // bit set is handled uniformly in SetGates.)

            if ((bus & BUS_HDADV) != 0)
            {
                //
                // Head advance: step to the next head (end-of-track continuation).
                //
                _drive.SetHead(_drive.Head + 1 >= _drive.Heads ? _drive.Head : _drive.Head + 1);
                if (Log.Enabled) Log.Write(LogComponent.TridentControl, "Head advance -> {0}", _drive.Head);
            }
        }

        private void SeekDone(int drive)
        {
            //
            // Seek/recalibrate completion raises the drive's Attention and settles
            // the heads on-cylinder (so the ready bit is now valid).
            // (Attention bits are one-hot, drive 0 = bit 3 of the nibble, matching
            // the KCtl drive-select encoding.)
            //
            _seekSettled = true;
            _attention |= (1 << (3 - drive));

            EvaluateWakeup();
        }

        private bool SelectedDriveReady()
        {
            return _driveSelect == 0 && _drive.IsLoaded && _drive.SeekComplete;
        }

        private void SetGates(bool read, bool write)
        {
            bool readRising = read && !_readGate;
            bool wasUp = _readGate || _writeGate;
            bool isUp = read || write;

            _readGate = read;
            _writeGate = write;

            //
            // Raising the read circuitry resets the selected drive's Attention
            // (CRstAtt = the read bit, TridentDLion.mc).
            //
            if (readRising && _driveSelect >= 0)
            {
                _attention &= ~(1 << (3 - _driveSelect));
            }

            if (isUp && !wasUp)
            {
                ArmTransfer();
            }
            else if (!isUp && wasUp)
            {
                AbortTransfer();
                _currentField = Field.None;
                _fieldWordIndex = -1;
            }

            if (Log.Enabled) Log.Write(LogComponent.TridentControl, "Gates: read={0} write={1}", read, write);
        }

        #endregion

        #region Rotation and data framing

        /// <summary>
        /// Sector pulse: the drive's servo surface generates 30 sector pulses per
        /// 16.67ms revolution (555.6us apart), independent of any recorded data.
        /// These run on real (emulated) time so rotational timing tests see the
        /// true cadence.  A pulse re-orients field framing: the next field to pass
        /// under the heads is the header.
        /// </summary>
        private void SectorCallback(ulong skewNsec, object context)
        {
            if (_drive.IsLoaded && _drive.SeekComplete)
            {
                _drive.Sector = _drive.Sector + 1;

                _sectorFound = true;
                if (_drive.Sector == 0)
                {
                    _indexFound = true;
                }

                _cursor = Field.Header;

                EvaluateWakeup();
            }

            _system.Scheduler.Schedule(SectorTimeNsec - skewNsec, SectorCallback);
        }

        /// <summary>
        /// Data transfers are self-paced rather than clocked at the real 1.65us/word
        /// rate: the CP's Disk task only runs one click (3 microinstructions) in
        /// every 5-click round (~2.06us), so ANY free-running word clock at the true
        /// rate overruns structurally.  Instead the next DataRequest follows the
        /// microcode's consumption of the previous word (the real hardware absorbs
        /// the same mismatch with the sequencer's buffering).
        ///
        /// A transfer arms when a read/write gate rises (or the sequencer is loaded
        /// with a gate already up) and begins after a short preamble/sync delay,
        /// targeting the field at the rotation cursor.  A read gate over a field
        /// that has never been written finds no sync on real media -- the transfer
        /// never begins and the microcode times out on index pulses, which is the
        /// "unformatted" signature the formatter distinguishes from bad media.
        /// </summary>
        private void ArmTransfer()
        {
            if (_xferPending || _xferActive)
            {
                return;
            }

            _xferPending = true;
            _system.Scheduler.Schedule(SyncDelayNsec, BeginTransferCallback);
        }

        private void BeginTransferCallback(ulong skewNsec, object context)
        {
            _xferPending = false;

            if ((!_readGate && !_writeGate) || _driveSelect != 0 || !_drive.IsLoaded || !_drive.SeekComplete)
            {
                return;
            }

            //
            // Sector attribution: a header op frames whatever sector the rotation
            // has under the heads (that's how the microcode's sector search
            // works).  Label/data ops chain onto the sector whose header the
            // previous op just processed -- self-paced transfers run slower than
            // the real 555us sector period, so re-sampling rotation mid-chain
            // would skew onto later sectors.
            //
            // Sequential reads (the EPROM boot loop, multi-page Pilot reads) read
            // consecutive sectors by arming sectorFound after each one -- correct
            // on real hardware where an op finishes inside its own sector.  Our
            // slower ops would land on N+2, so after a completed full sector read
            // the next header read attributes to N+1 (one-shot; any seek, head
            // change or write resets to rotational).  Header VERIFY ops don't
            // care: the pushed target redirects them regardless.
            //
            Field field = _cursor;
            int sector;
            if (field != Field.Header)
            {
                sector = _xferSector;
            }
            else if (_seqReadSector >= 0)
            {
                sector = (_seqReadSector + 1) % TridentDrive.SectorsPerTrack;
                _seqReadSector = -1;
            }
            else
            {
                sector = _drive.Sector;
            }
            _xferVirgin = false;

            if (field == Field.None)
            {
                //
                // All three fields of this sector already framed: this is the ECC
                // syndrome readout that follows the data field (ECCXfer reads the
                // controller's ECC register via RdLp) -- sourced from the
                // controller, not the pack.  Our ECC is always clean (zero words).
                //
                _xferCeiling = 8;
            }
            else
            {
                //
                // A read gate over a field that has never been written finds no
                // preamble/sync: the deserializer clocks noise, the sequencer still
                // generates its DataRequests (the microcode transfer loop depends
                // on all n+1 of them to exit -- going silent strands task 4 in
                // CANCELBR forever), and the ECC check fails.  Serve the transfer
                // with garbage words and latch eccError at the end: the classic
                // "unformatted media" signature.
                //
                _xferVirgin = !_writeGate && !_drive.IsFieldWritten(sector, FieldFlag(field));

                if (_xferVirgin && Log.Enabled) Log.Write(LogComponent.TridentControl,
                    "Virgin field read {0} c/h/s {1}/{2}/{3}", field, _drive.Cylinder, _drive.Head, sector);

                //
                // The transfer LENGTH is owned by the microcode: each sequencer
                // routine's loop counts its own RCnt of DataRequests (read header
                // 0x03 -> 3 reqs, read label 0x0c -> 11, read data 0x0e -> 257,
                // and format sector-write 0x0b -> 271: all three fields chained
                // as n+1 bursts under one gate).  Requests are served for as long
                // as the microcode keeps strobing; the op boundary (gate drop /
                // sequencer reload) ends the transfer.  The ceiling is only a
                // runaway backstop, sized past a full sector op.
                //
                _xferCeiling = 300;

                // The framing consumed this field's slot in the sector.
                _cursor = NextField(field);
            }

            _xferActive = true;
            _xferField = field;
            _xferSector = sector;
            _xferWordIndex = 0;
            _xferWrite = _writeGate;

            if (Log.Enabled) Log.Write(LogComponent.TridentControl,
                "Transfer begin {0} {1} c/h/s {2}/{3}/{4}",
                _xferWrite ? "write" : "read", field, _drive.Cylinder, _drive.Head, sector);

            RaiseDataRequest();
        }

        /// <summary>
        /// Maps a request index within a transfer to the sector field/word it
        /// frames.  An op may span several fields under one gate: the FORMAT
        /// sector-write (sequencer routine 0x0b) chains header, label and data as
        /// three n+1 bursts in a single continuous operation -- 2 payload words,
        /// an exit slot, 10 payload, exit slot, 256 payload, exit slot (271
        /// requests total).  offset -1 marks a loop-exit slot (no payload; the
        /// microcode does its inter-field housekeeping there).  Single-field ops
        /// never index past their first exit slot before the op boundary ends the
        /// transfer, so the same mapping serves both shapes.
        /// </summary>
        private void MapStreamWord(int index, out Field field, out int offset)
        {
            for (Field seg = _xferField; seg != Field.None; seg = NextField(seg))
            {
                int n = FieldLength(seg);
                if (index < n)
                {
                    field = seg;
                    offset = index;
                    return;
                }
                if (index == n)
                {
                    field = seg;
                    offset = -1;    // this field's loop-exit slot
                    return;
                }
                index -= n + 1;
            }

            field = Field.None;
            offset = -1;
        }

        private void RaiseDataRequest()
        {
            Field field;
            int offset;
            MapStreamWord(_xferWordIndex, out field, out offset);

            if (_xferVirgin)
            {
                // Unformatted field: the read chain is clocking noise.  Anything
                // that can't pass for a valid header works; all-ones avoids the
                // trap that a legitimate cyl 0/head 0/sector 0 header can be
                // all-zeros.
                _kIData = 0xffff;
            }
            else if (!_xferWrite && field != Field.None && offset >= 0)
            {
                _kIData = ReadFieldWord(field, offset);
            }
            else
            {
                // Write transfer, ECC syndrome word, or a loop-exit request slot.
                _kIData = 0;
            }

            _dataRequest = true;
            EvaluateWakeup();
        }

        /// <summary>
        /// The microcode consumed the outstanding request.  Per the transfer loops
        /// the KStrobe (c1) precedes the data movement (c2/c3), so the ack first
        /// records which word the in-flight KOData/KIData traffic belongs to, then
        /// advances.  The next request is paced a sub-click later: the ack can
        /// arrive as several K-function calls within one click (KStrobe then a
        /// KIData read), and the delay keeps that pair from double-advancing while
        /// still outrunning the task's ~2us best-case service rate.
        /// </summary>
        private void DataRequestServiced()
        {
            _dataRequest = false;

            if (!_xferActive)
            {
                EvaluateWakeup();
                return;
            }

            //
            // The emit context for the word being serviced right now (mapped
            // through the sector stream: multi-field format ops roll from header
            // into label into data); persists past transfer completion because
            // the final loop iteration pushes KOData after its strobe.  Exit
            // slots get a None context so their KOData traffic is discarded.
            //
            int offset;
            MapStreamWord(_xferWordIndex, out _currentField, out offset);
            if (offset < 0)
            {
                _currentField = Field.None;
            }
            _fieldWordIndex = offset;

            _xferWordIndex++;

            EvaluateWakeup();

            if (_xferWordIndex >= _xferCeiling)
            {
                if (Log.Enabled) Log.Write(LogComponent.TridentControl,
                    "Transfer ceiling hit: {0} after {1} requests", _xferField, _xferWordIndex);

                EndTransfer();
            }
            else
            {
                _system.Scheduler.Schedule(RequestDelayNsec, NextRequestCallback);
            }
        }

        /// <summary>
        /// Finalize the active transfer (op boundary reached, or the runaway
        /// ceiling).  Writes mark the field recorded regardless of length -- a
        /// short write is a torn field, which real media would also hold.
        /// </summary>
        private void EndTransfer()
        {
            if (_xferWrite && _xferField != Field.None && _xferWordIndex > 0)
            {
                //
                // Mark every field the stream actually reached (a format
                // sector-write rolls through header, label and data in one op).
                //
                Field last;
                int lastOffset;
                MapStreamWord(_xferWordIndex - 1, out last, out lastOffset);
                for (Field seg = _xferField; seg != Field.None; seg = NextField(seg))
                {
                    _drive.MarkFieldWritten(_xferSector, FieldFlag(seg));
                    if (seg == last)
                    {
                        break;
                    }
                }
            }

            if (_xferVirgin && _xferWordIndex > 0)
            {
                // The noise the read chain clocked never checks.
                _eccError = true;
            }

            //
            // A completed full data-field read arms sequential attribution for
            // the next header read; a write ends sequential-read mode.
            //
            if (!_xferWrite && _xferField == Field.Data
                && _xferWordIndex > FieldLength(Field.Data))
            {
                _seqReadSector = _xferSector;
            }
            else if (_xferWrite && _xferWordIndex > 0)
            {
                _seqReadSector = -1;
            }

            _xferActive = false;

            if (Log.Enabled) Log.Write(LogComponent.TridentControl,
                "Transfer end {0} {1} served={2}{3}",
                _xferWrite ? "write" : "read", _xferField, _xferWordIndex,
                _xferVirgin ? " (eccError)" : "");
        }

        private void NextRequestCallback(ulong skewNsec, object context)
        {
            if (_xferActive)
            {
                RaiseDataRequest();
            }
        }

        private void AbortTransfer()
        {
            if (_xferActive)
            {
                EndTransfer();
                _currentField = Field.None;
                _fieldWordIndex = -1;
            }

            if (_dataRequest)
            {
                _dataRequest = false;
                EvaluateWakeup();
            }
        }

        private static Field NextField(Field field)
        {
            switch (field)
            {
                case Field.Header: return Field.Label;
                case Field.Label: return Field.Data;
                default: return Field.None;
            }
        }

        private static int FieldFlag(Field field)
        {
            switch (field)
            {
                case Field.Header: return TridentDrive.FlagHeaderWritten;
                case Field.Label: return TridentDrive.FlagLabelWritten;
                case Field.Data: return TridentDrive.FlagDataWritten;
                default: return 0;
            }
        }

        private static int FieldLength(Field field)
        {
            switch (field)
            {
                case Field.Header: return TridentDrive.HeaderWords;
                case Field.Label: return TridentDrive.LabelWords;
                case Field.Data: return TridentDrive.DataWords;
                default: return 0;
            }
        }

        //
        // Field words address the sector captured when the transfer began: a slow
        // (self-paced) transfer may span the next sector pulse.
        //
        private ushort ReadFieldWord(Field field, int offset)
        {
            if (offset >= FieldLength(field))
            {
                return 0;
            }
            return _drive.ReadWord(_xferSector, FieldBase(field) + offset);
        }

        private void WriteFieldWord(Field field, int offset, ushort value)
        {
            if (offset >= FieldLength(field))
            {
                return;
            }
            _drive.WriteWord(_xferSector, FieldBase(field) + offset, value);
        }

        private static int FieldBase(Field field)
        {
            switch (field)
            {
                case Field.Header: return 0;
                case Field.Label: return TridentDrive.HeaderWords;
                case Field.Data: return TridentDrive.HeaderWords + TridentDrive.LabelWords;
                default: return 0;
            }
        }

        #endregion

        #region Wakeups

        private void EvaluateWakeup()
        {
            bool wake;

            switch (_wakeup)
            {
                case Wakeup.None:
                    //
                    // Default mode: the disk firmware runs while firmwareEnable is
                    // set and sleeps when it clears it (TridentDLion NoSIP: "resets
                    // FirmwareEn bit and goes to sleep").  Pending attention alone
                    // must NOT wake it here -- attention wakes are the armed
                    // CWUAnyAtt condition -- or the CSB idle loop spins forever
                    // against a standing power-on attention.
                    //
                    wake = (_kCmd & FIRMWARE_ENABLE) != 0;
                    break;

                case Wakeup.AnyAttention:
                    wake = _attention != 0;
                    break;

                case Wakeup.IndexFound:
                    wake = _indexFound;
                    break;

                case Wakeup.SectorFound:
                    wake = _sectorFound;
                    break;

                case Wakeup.DataRequest:
                    wake = _dataRequest;
                    break;

                case Wakeup.DiskReady:
                    wake = SelectedDriveReady();
                    break;

                case Wakeup.One:
                    // TDC.KControl wakeup code 7 = "one": constant true.
                    wake = true;
                    break;

                default:
                    wake = false;
                    break;
            }

            if (wake)
            {
                _system.CP.WakeTask(TaskType.Disk);
            }
            else
            {
                _system.CP.SleepTask(TaskType.Disk);
            }
        }

        //
        // TDC.KControl.wakeup: {firmwareEn, anyAttention, idxFound, sectorFound,
        // dataReq, <unused>, rdy, one}.
        //
        private enum Wakeup
        {
            None = 0,           // firmwareEn: wake while KCmd's firmwareEnable is set
            AnyAttention = 1,
            IndexFound = 2,
            SectorFound = 3,
            DataRequest = 4,
            Unused5 = 5,
            DiskReady = 6,
            One = 7,            // constant true
        }

        #endregion

        private static bool RisingEdge(ushort oldValue, ushort newValue, int bit)
        {
            return (oldValue & bit) == 0 && (newValue & bit) != 0;
        }

        private enum Field
        {
            None = 0,
            Header,
            Label,
            Data,
        }

        //
        // KCmd bits (Dandelion.dfn constants, pre-shifted into a 16-bit register:
        // tag byte high, bus low).
        //
        //
        // KCmd command-word layout (TDC.mesa, Xerox MSB=bit0 numbering).
        // Tag field (bits 0-2): null=0, ctl, setHead, setCyl -- the tag reinterprets
        // the low bits (setCyl: cylinder in 0x03FF; setHead: head 0x1F, offset 0x80,
        // forward 0x40), so the read/write gate bits are meaningful ONLY for the
        // null and ctl tags.
        //
        private const int CYL_TAG = 0x8000;         // setCyl
        private const int HD_TAG = 0x4000;          // setHead
        private const int CTL_TAG = 0x2000;         // ctl
        private const int TAG_MASK = 0xe000;

        private const int TERM = 0x1000;            // vestigial (TridentDLion: "not required")
        private const int FIRMWARE_ENABLE = 0x0800;
        private const int TEST_CTL = 0x0400;
        private const int STROBE_LATE = 0x0200;
        private const int STROBE_EARLY = 0x0100;

        //
        // Gate + control bits (low byte).  write=0x80, read=0x40 -- confirmed from
        // TDC.KCtl.write(bit 8)/read(bit 9); the old 0x20 "write" was actually the
        // addressMark bit, so real writes registered as neither gate.
        //
        private const int BUS_WRITE = 0x80;         // write gate (energize write head)
        private const int BUS_READ = 0x40;          // read gate (also resets Attention)
        private const int BUS_ADDRMARK = 0x20;
        private const int BUS_RESETHEADREG = 0x10;
        private const int BUS_DVCCHKRST = 0x08;     // device check reset
        private const int BUS_HEADSELECT = 0x04;
        private const int BUS_REZERO = 0x02;        // recalibrate
        private const int BUS_HDADV = 0x01;         // head advance

        //
        // Rotational timing: 3600rpm = 16.667ms/rev, 30 sectors = 555.6us/sector.
        // These run at the true cadence (a diagnostic timing an index-to-index
        // revolution or counting sector pulses must see real numbers); only the
        // per-word data pacing is decoupled (self-paced by KStrobe).
        //
        private const ulong SectorTimeNsec = 555556;

        //
        // Preamble/sync search time between gate-up and the first data request,
        // standing in for the recorded gap the real read circuitry hunts through.
        //
        private const ulong SyncDelayNsec = 10000;

        //
        // Pacing between the microcode's KStrobe ack and the next data request:
        // long enough that the strobe's companion K-function calls in the same
        // click (KIData read / KOData write) can't double-advance, short enough to
        // never be the transfer bottleneck (the Disk task's click comes ~2us apart).
        //
        private const ulong RequestDelayNsec = 1000;

        //
        // Registers and decoded state
        //
        private ushort _kCmd;
        private ushort _kCtl;
        private int _driveSelect;
        private Wakeup _wakeup;
        private int _sequencerRoutine;

        private bool _readGate;
        private bool _writeGate;

        //
        // Latches
        //
        private bool _sectorFound;
        private bool _indexFound;
        private bool _dataRequest;
        private int _attention;             // one-hot nibble, bit 3 = drive 0
        private bool _diskCheck;
        private bool _illegalHead;
        private bool _eccError;
        private bool _overrun;
        private bool _verifyError;
        private bool _seekSettled;

        //
        // Transfer state.  _cursor is the next field to pass under the heads
        // (reset to Header by each sector pulse, consumed by each framing);
        // _currentField/_fieldWordIndex are the emit context of the word most
        // recently acked (they outlive the transfer for the final loop
        // iteration's trailing KOData push).
        //
        private Field _cursor;
        private bool _xferActive;
        private bool _xferPending;
        private bool _xferWrite;
        private bool _xferVirgin;
        private Field _xferField;
        private int _xferSector;
        private int _xferWordIndex;
        private int _xferCeiling;
        private ushort _xferPendingHdr0;
        private int _seqReadSector = -1;

        private Field _currentField;
        private int _fieldWordIndex;
        private ushort _kIData;

        private DSystem _system;
        private TridentDrive _drive;
    }
}
