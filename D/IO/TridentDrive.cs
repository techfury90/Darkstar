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

using D.Logging;
using System;
using System.IO;

namespace D.IO
{
    public enum TridentDriveType
    {
        Invalid = 0,
        T80 = 4,        // Century Data Systems Trident T-80, ~68MB formatted (DLion format)
        T300 = 5,       // Trident T-300, ~258MB formatted (DLion format)
    }

    /// <summary>
    /// Encapsulates the state, pack data and low-level behavior of a Century Data
    /// Systems Trident T-80/T-300 drive attached to the HSIO-L (Large Disk) controller
    /// of a Large-Capacity server.
    ///
    /// Unlike the SA1000 model (which stores raw track words including address marks),
    /// the Trident pack is stored field-structured, matching the DLion disk format the
    /// HSIO-L's word sequencer frames in hardware: 30 sectors per track, each sector
    /// holding a 2-word header, a 10-word label and a 256-word data field
    /// (TridentDLion.mc / TridentInitial.mc: vSecsPerTrack=30, header count=2,
    /// label count=0A, data count=100).
    ///
    /// Pack image format (v2):
    ///   Byte 0: drive type (TridentDriveType)
    ///   Then sectors in cylinder/head/sector order, each serialized as a flags word
    ///   followed by 2 + 10 + 256 = 268 payload words, little-endian.  The flags word
    ///   records which fields have ever been written (bit 0 header, 1 label, 2 data):
    ///   a field that has never been written has no preamble/sync on real media, so
    ///   a read over it clocks noise and fails its ECC check -- the controller must
    ///   produce garbage + eccError, not clean zeros.  v1 images (no flags word,
    ///   detected by file length) load with nonzero sectors treated as fully written.
    ///
    /// There are no pre-existing Trident pack images anywhere; every pack starts
    /// blank (NewDisk) and is low-level formatted by the rigid disk diagnostics or
    /// the installer inside the emulator.
    /// </summary>
    public class TridentDrive
    {
        public TridentDrive(DSystem system)
        {
            _system = system;

            _type = TridentDriveType.Invalid;
            NewDisk(_type, String.Empty);

            Reset();
        }

        //
        // DLion Trident sector format (words).
        //
        public const int HeaderWords = 2;
        public const int LabelWords = 10;
        public const int DataWords = 256;
        public const int SectorWords = HeaderWords + LabelWords + DataWords;

        public const int SectorsPerTrack = 30;

        //
        // Per-sector written-field flags (in-memory word 0 of each sector record,
        // persisted in v2 pack images).
        //
        public const int FlagHeaderWritten = 1;
        public const int FlagLabelWritten = 2;
        public const int FlagDataWritten = 4;

        private const int RecordWords = 1 + SectorWords;    // flags + payload

        public void Reset()
        {
            _cylinder = 0;
            _head = 0;
            _sector = 0;
            _seekComplete = true;
        }

        public void NewDisk(TridentDriveType type, string path)
        {
            switch (type)
            {
                case TridentDriveType.Invalid:
                    //
                    // As with the SA1000: unloaded drives get a minimal geometry
                    // and always report Not Ready.
                    //
                case TridentDriveType.T80:
                    _cylinders = 815;
                    _heads = 5;
                    break;

                case TridentDriveType.T300:
                    _cylinders = 815;
                    _heads = 19;
                    break;
            }

            _sectors = new ushort[_cylinders * _heads * SectorsPerTrack][];
            _type = type;
            _diskImagePath = path;
        }

        /// <summary>
        /// Creates a new factory-formatted pack of the given type and writes it to
        /// disk.  Real Century Data / CDC packs shipped low-level formatted, with a
        /// valid sector-address header on every sector and an empty defect list; the
        /// field diagnostics can only *re*format such a pack (a truly virgin surface
        /// has no readable cylinder zero for the bad-page table, so the format aborts
        /// with "Bad Page in cylinder zero").  So a "blank" pack here means every
        /// sector carries its address header {cyl, head&lt;&lt;8|sector} over zeroed
        /// label and data -- which the diagnostic's format and the installer's
        /// physical-volume creation then build the Pilot volume on top of.
        /// </summary>
        public static void CreateBlankPack(TridentDriveType type, string path)
        {
            int cylinders;
            int heads;
            switch (type)
            {
                case TridentDriveType.T80: cylinders = 815; heads = 5; break;
                case TridentDriveType.T300: cylinders = 815; heads = 19; break;
                default: throw new InvalidOperationException("Cannot create a pack of an invalid drive type.");
            }

            string tempPath = Path.GetTempFileName();
            using (FileStream fs = new FileStream(tempPath, FileMode.Create, FileAccess.Write))
            {
                fs.WriteByte((byte)type);

                byte[] record = new byte[RecordWords * 2];
                for (int c = 0; c < cylinders; c++)
                {
                    for (int h = 0; h < heads; h++)
                    {
                        for (int s = 0; s < SectorsPerTrack; s++)
                        {
                            Array.Clear(record, 0, record.Length);

                            // Flags word: header/label/data all formatted.
                            ushort flags = FlagHeaderWritten | FlagLabelWritten | FlagDataWritten;
                            record[0] = (byte)flags;
                            record[1] = (byte)(flags >> 8);

                            // Header word 0 = cylinder, word 1 = head<<8 | sector
                            // (the Pilot DiskAddress layout the controller frames).
                            ushort hdr0 = (ushort)c;
                            ushort hdr1 = (ushort)((h << 8) | s);
                            record[2] = (byte)hdr0;
                            record[3] = (byte)(hdr0 >> 8);
                            record[4] = (byte)hdr1;
                            record[5] = (byte)(hdr1 >> 8);

                            // Label and data fields stay zeroed.
                            fs.Write(record, 0, record.Length);
                        }
                    }
                }
            }

            File.Copy(tempPath, path, true /* overwrite */);
            File.Delete(tempPath);
        }

        public void Load(string path)
        {
            try
            {
                using (FileStream fs = new FileStream(path, FileMode.Open, FileAccess.Read))
                {
                    byte type = (byte)fs.ReadByte();
                    if (type != (int)TridentDriveType.T80 && type != (int)TridentDriveType.T300)
                    {
                        throw new InvalidOperationException("Unsupported Trident drive type.");
                    }

                    NewDisk((TridentDriveType)type, path);

                    //
                    // v2 records carry a leading flags word; v1 records are bare
                    // payload.  Distinguish by file length.
                    //
                    long dataLength = fs.Length - 1;
                    bool v2;
                    if (dataLength == (long)_sectors.Length * RecordWords * 2)
                    {
                        v2 = true;
                    }
                    else if (dataLength == (long)_sectors.Length * SectorWords * 2)
                    {
                        v2 = false;
                    }
                    else
                    {
                        throw new InvalidOperationException("Trident pack image size does not match drive geometry.");
                    }

                    int recordWords = v2 ? RecordWords : SectorWords;
                    byte[] buffer = new byte[recordWords * 2];
                    for (int i = 0; i < _sectors.Length; i++)
                    {
                        int read = fs.Read(buffer, 0, buffer.Length);
                        if (read < buffer.Length)
                        {
                            throw new InvalidOperationException("Short read on Trident pack load.");
                        }

                        //
                        // Only materialize sectors that contain data; a blank
                        // (all-zero) sector stays null to keep memory usage sane
                        // on a T-300.
                        //
                        bool empty = true;
                        for (int b = 0; b < buffer.Length; b++)
                        {
                            if (buffer[b] != 0)
                            {
                                empty = false;
                                break;
                            }
                        }

                        if (!empty)
                        {
                            ushort[] sector = new ushort[RecordWords];
                            if (v2)
                            {
                                for (int w = 0; w < RecordWords; w++)
                                {
                                    sector[w] = (ushort)(buffer[w * 2] | (buffer[w * 2 + 1] << 8));
                                }
                            }
                            else
                            {
                                // v1: no flags word; nonzero content means the
                                // sector was written by an earlier build.
                                sector[0] = FlagHeaderWritten | FlagLabelWritten | FlagDataWritten;
                                for (int w = 0; w < SectorWords; w++)
                                {
                                    sector[1 + w] = (ushort)(buffer[w * 2] | (buffer[w * 2 + 1] << 8));
                                }
                            }
                            _sectors[i] = sector;
                        }
                    }
                }
            }
            catch (Exception e)
            {
                // Don't leave a partial pack loaded.
                _type = TridentDriveType.Invalid;
                NewDisk(_type, String.Empty);

                throw e;
            }
        }

        public void Save()
        {
            if (!string.IsNullOrEmpty(_diskImagePath))
            {
                Save(_diskImagePath);
            }
        }

        public void Save(string path)
        {
            // Commit to a temporary file first (same pattern as SA1000Drive).
            string tempPath = Path.GetTempFileName();

            using (FileStream fs = new FileStream(tempPath, FileMode.Create, FileAccess.Write))
            {
                fs.WriteByte((byte)_type);

                byte[] zero = new byte[RecordWords * 2];
                byte[] buffer = new byte[RecordWords * 2];
                for (int i = 0; i < _sectors.Length; i++)
                {
                    ushort[] sector = _sectors[i];
                    if (sector == null)
                    {
                        fs.Write(zero, 0, zero.Length);
                    }
                    else
                    {
                        for (int w = 0; w < RecordWords; w++)
                        {
                            buffer[w * 2] = (byte)sector[w];
                            buffer[w * 2 + 1] = (byte)(sector[w] >> 8);
                        }
                        fs.Write(buffer, 0, buffer.Length);
                    }
                }
            }

            File.Copy(tempPath, path, true /* overwrite */);
            File.Delete(tempPath);
            _diskImagePath = path;
        }

        public string ImagePath
        {
            get { return _diskImagePath; }
        }

        public TridentDriveType Type
        {
            get { return _type; }
        }

        public int Cylinders
        {
            get { return _cylinders; }
        }

        public int Heads
        {
            get { return _heads; }
        }

        public bool IsLoaded
        {
            get { return _type != TridentDriveType.Invalid; }
        }

        public int Cylinder
        {
            get { return _cylinder; }
        }

        public int Head
        {
            get { return _head; }
        }

        /// <summary>
        /// The sector currently passing under the heads (advanced by the controller's
        /// rotation timing).
        /// </summary>
        public int Sector
        {
            get { return _sector; }
            set { _sector = value % SectorsPerTrack; }
        }

        /// <summary>
        /// True while the heads are settled (not seeking).  The drive also reports
        /// not-ready briefly after head-offset changes on the real hardware; we do
        /// not model offsets.
        /// </summary>
        public bool SeekComplete
        {
            get { return _seekComplete; }
        }

        public bool SetHead(int head)
        {
            if (head >= _heads)
            {
                //
                // Selecting a nonexistent head: TridentInitial.mc uses exactly this
                // (select head 5) to distinguish a T-80 from a T-300; the drive
                // reports the illegal address, it does not wrap.
                //
                if (Log.Enabled) Log.Write(LogComponent.TridentControl, "Head select {0} out of range ({1} heads)", head, _heads);
                return false;
            }

            _head = head;
            return true;
        }

        /// <summary>
        /// Starts a seek to the given cylinder.  Returns false (device check) for an
        /// out-of-range address.  Completion is signalled to the controller after the
        /// physical seek time (Century manual: ~6ms adjacent, ~55ms full stroke --
        /// modeled as the linear ramp ContrAlto uses: 6.0 + 0.602 * delta ms).
        /// </summary>
        public bool Seek(int destCylinder, Action onComplete)
        {
            if (destCylinder >= _cylinders)
            {
                if (Log.Enabled) Log.Write(LogComponent.TridentControl, "Seek to cylinder {0} out of range", destCylinder);
                return false;
            }

            if (destCylinder == _cylinder)
            {
                // Zero-length seeks still cycle SeekComplete/attention; give them a
                // short settle time.
                _seekComplete = false;
                _destCylinder = destCylinder;
                _onSeekComplete = onComplete;
                _system.Scheduler.Schedule((ulong)(1.0 * Conversion.MsecToNsec), SeekCompleteCallback);
            }
            else
            {
                _seekComplete = false;
                _destCylinder = destCylinder;
                _onSeekComplete = onComplete;

                ulong seekDuration = (ulong)((6.0 + 0.602 * Math.Abs(_cylinder - destCylinder)) * Conversion.MsecToNsec);
                _system.Scheduler.Schedule(seekDuration, SeekCompleteCallback);
            }

            return true;
        }

        /// <summary>
        /// Recalibrate: restore to cylinder 0.
        /// </summary>
        public bool Recalibrate(Action onComplete)
        {
            return Seek(0, onComplete);
        }

        public ushort ReadWord(int sector, int wordOffset)
        {
            ushort[] s = _sectors[SectorIndex(sector)];
            return s == null ? (ushort)0 : s[1 + wordOffset];
        }

        public void WriteWord(int sector, int wordOffset, ushort value)
        {
            MaterializeSector(sector)[1 + wordOffset] = value;
        }

        /// <summary>
        /// True if the given field of the sector under the heads has ever been
        /// written.  A never-written field has no preamble/sync recorded, so a read
        /// over it clocks noise and fails its ECC check (the virgin-media signature
        /// that distinguishes "unformatted" from valid data).
        /// </summary>
        public bool IsFieldWritten(int sector, int fieldFlag)
        {
            ushort[] s = _sectors[SectorIndex(sector)];
            return s != null && (s[0] & fieldFlag) != 0;
        }

        public void MarkFieldWritten(int sector, int fieldFlag)
        {
            MaterializeSector(sector)[0] |= (ushort)fieldFlag;
        }

        private ushort[] MaterializeSector(int sector)
        {
            int index = SectorIndex(sector);
            ushort[] s = _sectors[index];
            if (s == null)
            {
                s = new ushort[RecordWords];
                _sectors[index] = s;
            }
            return s;
        }

        private int SectorIndex(int sector)
        {
            return (_cylinder * _heads + _head) * SectorsPerTrack + sector;
        }

        private void SeekCompleteCallback(ulong skewNsec, object context)
        {
            _cylinder = _destCylinder;
            _seekComplete = true;

            if (Log.Enabled) Log.Write(LogComponent.TridentControl, "Seek to {0} complete.", _cylinder);

            if (_onSeekComplete != null)
            {
                Action complete = _onSeekComplete;
                _onSeekComplete = null;
                complete();
            }
        }

        //
        // Geometry (T-80: 815 x 5, T-300: 815 x 19; both 30 sectors/track in the
        // DLion format).
        //
        private TridentDriveType _type;
        private int _cylinders;
        private int _heads;

        //
        // Pack data: one ushort[RecordWords] per sector (word 0 = written-field
        // flags, then the payload), lazily allocated (null = virgin sector).
        //
        private ushort[][] _sectors;

        //
        // Position
        //
        private int _cylinder;
        private int _head;
        private int _sector;

        //
        // Seek state
        //
        private bool _seekComplete;
        private int _destCylinder;
        private Action _onSeekComplete;

        private string _diskImagePath;

        private DSystem _system;
    }
}
