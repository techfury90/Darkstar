using System;
using System.IO;

namespace D.IO
{
    /// <summary>
    /// Backing store for the Xerox 6085 rigid disk as configured by THIS machine's
    /// EEPROM: a Micropolis 1325 declared as 960 cylinders x 8 heads x 16 sectors/track,
    /// 512 bytes/sector = 62,914,560 bytes (60 MiB) of data.
    ///
    /// Every Pilot sector carries THREE fields: a header (CHS, implicit in the addressing),
    /// a 10-word (20-byte) label, and 512 bytes of data.  The controller verifies the stored
    /// label against a label Pilot supplies on reads, so the label MUST be persisted per sector
    /// (a data-only image silently fails every verify).  We keep the data image a clean flat
    /// 62,914,560-byte file (inspectable as a raw disk) and the labels in a parallel sidecar.
    ///
    /// A sector is "formatted" once Format (or WriteLabelAndData) has written it; before that it
    /// is blank.  A blank/virgin image is a valid first-bring-up state -- Offline Diagnostics'
    /// Format runs before any verify-read.  Reads of an unformatted sector report blank.
    /// </summary>
    public sealed class Micropolis1325
    {
        public const int Cylinders = 960;
        public const int Heads = 8;
        public const int SectorsPerTrack = 16;
        public const int SectorBytes = 512;
        public const int LabelWords = 10;                       // 20 bytes
        public const int TotalSectors = Cylinders * Heads * SectorsPerTrack;   // 122,880
        public const long DataBytes = (long)TotalSectors * SectorBytes;         // 62,914,560

        // Lazy per-sector allocation: null slot = unformatted/blank.  A blank pack is all-null,
        // so a fresh image costs nothing until Format/Write touches a sector.
        private readonly byte[][] _data = new byte[TotalSectors][];
        private readonly ushort[][] _label = new ushort[TotalSectors][];

        private string _dataPath, _labelPath;

        /// <summary>Format/write activity, so a format in progress is visible.  Distinct counts
        /// how much of the pack has been touched at least once -- a real format makes several
        /// passes over the whole surface, so the raw count runs well past the pack size.</summary>
        public long SectorsFormatted, DistinctSectorsFormatted, SectorsWritten;

        public Micropolis1325() { }

        /// <summary>page number = sector + spt*(head + heads*cyl); the linear sector index.</summary>
        public static int Page(int cyl, int head, int sector)
        {
            return sector + SectorsPerTrack * (head + Heads * cyl);
        }

        public bool InRange(int cyl, int head, int sector)
        {
            return cyl >= 0 && cyl < Cylinders && head >= 0 && head < Heads
                && sector >= 0 && sector < SectorsPerTrack;
        }

        public bool IsFormatted(int page) { return page >= 0 && page < TotalSectors && _data[page] != null; }

        /// <summary>Returns the stored 512-byte data for a page, or null if unformatted.</summary>
        public byte[] ReadData(int page)
        {
            return (page >= 0 && page < TotalSectors) ? _data[page] : null;
        }

        /// <summary>Returns the stored 10-word label for a page, or null if unformatted.</summary>
        public ushort[] ReadLabel(int page)
        {
            return (page >= 0 && page < TotalSectors) ? _label[page] : null;
        }

        /// <summary>Write both fields for a page (Format / WriteLabelAndData). Copies the inputs.</summary>
        public void WriteSector(int page, byte[] data, ushort[] label)
        {
            if (page < 0 || page >= TotalSectors) return;
            var d = new byte[SectorBytes];
            if (data != null) Array.Copy(data, d, Math.Min(data.Length, SectorBytes));
            _data[page] = d;
            var l = new ushort[LabelWords];
            if (label != null) Array.Copy(label, l, Math.Min(label.Length, LabelWords));
            _label[page] = l;
        }

        /// <summary>Write only the 512-byte data (WriteData), leaving/creating the label.</summary>
        public void WriteData(int page, byte[] data)
        {
            SectorsWritten++;
            if (page < 0 || page >= TotalSectors) return;
            var d = new byte[SectorBytes];
            if (data != null) Array.Copy(data, d, Math.Min(data.Length, SectorBytes));
            _data[page] = d;
            if (_label[page] == null) _label[page] = new ushort[LabelWords];
        }

        /// <summary>Initialize a sector to a blank formatted state (Format): zero data + zero label.</summary>
        public void FormatSector(int page)
        {
            if (page < 0 || page >= TotalSectors) return;
            SectorsFormatted++;
            if (_data[page] == null) DistinctSectorsFormatted++;
            _data[page] = new byte[SectorBytes];
            _label[page] = new ushort[LabelWords];
        }

        // ---- persistence ------------------------------------------------------
        // The data file is a flat 62,914,560-byte raw image (0 = unformatted, indistinguishable
        // from a formatted-zero sector on disk -- we keep an in-memory formatted bitmap only
        // while running).  Labels ride in a sidecar ".labels".  Load of a missing file = blank pack.

        public void Load(string dataPath)
        {
            _dataPath = dataPath;
            _labelPath = dataPath + ".labels";
            if (File.Exists(dataPath))
            {
                using (var fs = new FileStream(dataPath, FileMode.Open, FileAccess.Read))
                {
                    for (int p = 0; p < TotalSectors; p++)
                    {
                        var buf = new byte[SectorBytes];
                        int off = 0, n;
                        while (off < SectorBytes && (n = fs.Read(buf, off, SectorBytes - off)) > 0) off += n;
                        if (off == 0) break;                    // short file: rest stays blank
                        bool nonZero = false; for (int i = 0; i < off; i++) if (buf[i] != 0) { nonZero = true; break; }
                        if (nonZero) _data[p] = buf;            // treat all-zero as unformatted (blank)
                    }
                }
            }
            if (_labelPath != null && File.Exists(_labelPath))
            {
                var raw = File.ReadAllBytes(_labelPath);
                int stride = LabelWords * 2;
                for (int p = 0; p < TotalSectors && (p + 1) * stride <= raw.Length; p++)
                {
                    var lab = new ushort[LabelWords]; bool nz = false;
                    for (int w = 0; w < LabelWords; w++)
                    {
                        int b = p * stride + w * 2;
                        lab[w] = (ushort)((raw[b] << 8) | raw[b + 1]);       // big-endian on disk
                        if (lab[w] != 0) nz = true;
                    }
                    if (nz || _data[p] != null) _label[p] = lab;
                }
            }
        }

        public void Save() { if (_dataPath != null) Save(_dataPath); }

        public void Save(string dataPath)
        {
            string tmp = dataPath + ".tmp";
            using (var fs = new FileStream(tmp, FileMode.Create, FileAccess.Write))
            {
                var zero = new byte[SectorBytes];
                for (int p = 0; p < TotalSectors; p++)
                    fs.Write(_data[p] ?? zero, 0, SectorBytes);
            }
            if (File.Exists(dataPath)) File.Delete(dataPath);
            File.Move(tmp, dataPath);

            string lp = dataPath + ".labels", ltmp = lp + ".tmp";
            using (var fs = new FileStream(ltmp, FileMode.Create, FileAccess.Write))
            {
                for (int p = 0; p < TotalSectors; p++)
                {
                    var lab = _label[p];
                    for (int w = 0; w < LabelWords; w++)
                    {
                        ushort v = lab != null ? lab[w] : (ushort)0;
                        fs.WriteByte((byte)(v >> 8)); fs.WriteByte((byte)v);
                    }
                }
            }
            if (File.Exists(lp)) File.Delete(lp);
            File.Move(ltmp, lp);
        }
    }
}
