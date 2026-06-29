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
using System.Collections.Generic;

namespace D.Display
{
    public class DisplayController
    {
        public DisplayController(DSystem system)
        {
            _system = system;
            
            _lostSyncEvent = new Event(_lostSyncInterval, null, LostSyncCallback);

            _fifo = new Queue<ushort>(16);

        }

        public void Reset()
        {
            _displayOn = false;
            _blank = false;
            _picture = false;
            _invert = false;
            _oddLine = false;

            _scanline = 0;

            _fifo.Clear();

            if (_system.Display != null)
            {
                _system.Display.Clear();
            }
        }

        public bool DisplayOn
        {
            get { return _displayOn; }
        }

        public void ClrDpRq()
        {
            //
            // Put the display task to sleep
            //
            _system.CP.SleepTask(TaskType.Display);
        }

        public void SetDCtlFifo(ushort value)
        {
            // if (Log.Enabled) Log.Write(LogType.Verbose, LogComponent.DisplayControl, "DCtlFIFO<-0x{0:x4}", value);
            if (_fifo.Count < 16)
            {
                _fifo.Enqueue(value);
            }
            else
            {
                if (Log.Enabled) Log.Write(LogType.Error, LogComponent.DisplayControl, "DCtlFIFO: FIFO overflow, word dropped.");
            }
        }

        public void SetDCtl(ushort value)
        {
            bool displayOn = _displayOn;
            _displayOn = (value & 0x01) != 0;
            _blank = (value & 0x02) != 0;
            _picture = (value & 0x04) != 0;
            _invert = (value & 0x08) != 0;

            if ((value & 0x20) != 0)
            {
                // Vertical Sync -- back to the top of the screen
                _scanline = 0;
                _oddLine = (value & 0x10) != 0;
                _syncPresent = true;
                _captureFirstPicture = true;     // re-measure where the picture starts this field (see border phasing)
                _system.Display.Render();
            }

            if ((value & 0x40) == 0)
            {
                // Clear control fifo
                _fifo.Clear();
            }

            if (!displayOn && _displayOn)
            {
                // Kick off the horizontal retrace callback since we're turning the display on.                
                _system.Scheduler.Schedule(_horizontalRetraceDelay, HorizontalRetraceCallback);

                _system.Scheduler.Cancel(_lostSyncEvent);
                _lostSyncEvent = _system.Scheduler.Schedule(_lostSyncInterval, LostSyncCallback);
            }
            else if (!_displayOn)
            {
                //
                // Put the display task to sleep.
                //
                _system.CP.SleepTask(TaskType.Display);
            }

            if (Log.Enabled) Log.Write(LogType.Verbose, LogComponent.DisplayControl, "DCtl<-0x{0:x4}: On={1} Blank={2} Picture={3} Invert={4} Odd={5}"
                , value,
                _displayOn,
                _blank,
                _picture,
                _invert,
                _oddLine);
        }

        public void SetDBorder(ushort value)
        {
            _displayBorder = value;
            if (Log.Enabled) Log.Write(LogType.Verbose, LogComponent.DisplayControl, "DBorder<-0x{0:x4}", value);
        }

        /// <summary>
        /// Invoked at the end of every scanline: Wake display task, update state, schedule next callback
        /// as necessary.
        /// </summary>
        /// <param name="skewNsec"></param>
        /// <param name="context"></param>
        private void HorizontalRetraceCallback(ulong skewNsec, object context)
        {
            int visibleOffset = _oddLine ? 37 : 36;
            int effectiveScanline = _scanline - visibleOffset;

            //
            // Border phase: the border pattern's 2-scanline blocks must line up with the picture's stipple.
            // The picture begins a guest-programmed number of sync + top-border lines below the vertical-sync
            // reset, and that distance differs between guest OSes (e.g. ViewPoint and Interlisp-D program
            // different vertical-sync lengths -- 16/17 vs 13/14 lines), so it cannot be a fixed constant.
            // Instead, measure where the picture actually starts this field and phase the border off that, so
            // the border tracks whatever vertical geometry the guest sets up.  Border-only lines (top/bottom
            // border, display off) reuse the previous field's measurement.
            // The two interlace fields start their picture at different _scanline values (one extra sync
            // line on the odd field), so they must be measured separately -- otherwise a field's top/bottom
            // border, which reuses the previous field's measurement, would be phased off the *other* parity
            // and tear by one line right where the active region begins.
            int field = _oddLine ? 1 : 0;
            if (_picture && _captureFirstPicture)
            {
                _firstPictureScanline[field] = _scanline;
                _captureFirstPicture = false;
            }
            int borderPhaseLine = _scanline - _firstPictureScanline[field] + _borderPhaseAdjust;

            //
            // Render this scanline, if there's anything to do.
            //
            if (_blank)
            {
                // Render blank scanline (no border, no picture)
                for (int i = 0; i < _scanlineData.Length; i++)
                {
                    _scanlineData[i] = 0;
                }
            }
            else
            {
                if (_picture)
                {
                    // Normal line : 32 bits of border pattern, 1024 bits of display, 32 bits of border pattern
                    // Border pattern: low byte on lines 4n, 4n+1; high byte on 4n+2, 4n+3 (phased off the
                    // measured picture start, see borderPhaseLine above).
                    int patternByte = (borderPhaseLine & 0x2) == 0 ? _displayBorder & 0xff : _displayBorder >> 8;
                    ushort patternWord = (ushort)(patternByte | (patternByte << 8));

                    _scanlineData[0] = patternWord;
                    _scanlineData[1] = patternWord;

                    if (_fifo.Count > 0)
                    {
                        // Grab first segment from FIFO
                        ushort fifoWord = _fifo.Dequeue();
                        int lastWord = fifoWord >> 10;
                        int lineNumber = fifoWord & 0x3ff;
                        bool valid = false;
                        for (int word = 0; word < 64; word++)
                        {
                            _scanlineData[word + 2] = _system.MemoryController.DebugMemory.ReadWord((lineNumber << 6) | word, out valid);

                            // Grab next segment if this isn't the last word in the scanline.
                            if (word != 63 && word == lastWord && _fifo.Count > 0)
                            {                                
                                fifoWord = _fifo.Dequeue();
                                lastWord = fifoWord >> 10;
                                lineNumber = fifoWord & 0x3ff;
                            }
                        }
                    }
                    else
                    {
                        // Blank out display words, nothing in the FIFO.
                        for (int i = 2; i < 64; i++)
                        {
                            _scanlineData[i] = 0;
                        }
                    }

                    _scanlineData[66] = patternWord;
                    _scanlineData[67] = patternWord;
                }
                else
                {
                    // Just display the border pattern everywhere:
                    // low byte on lines 4n, 4n+1; high byte on 4n+2, 4n+3 (phased off the measured
                    // picture start, see borderPhaseLine above).
                    int patternByte = (borderPhaseLine & 0x2) == 0 ? _displayBorder & 0xff : _displayBorder >> 8;
                    ushort patternWord = (ushort)(patternByte | (patternByte << 8));

                    for (int i = 0; i < _scanlineData.Length; i++)
                    {
                        _scanlineData[i] = patternWord;
                    }
                }
            }            

            if (effectiveScanline > 0 && effectiveScanline < 860)
            {
                // Render to screen
                _system.Display.DrawScanline(effectiveScanline, _scanlineData, _invert);
            }

            // Move to next scanline
            _scanline += 2;

            //
            // Schedule next retrace as long as the display is still on.
            //
            if (_displayOn)
            {
                _system.Scheduler.Schedule(_horizontalRetraceDelay, HorizontalRetraceCallback);

                //
                // End of scanline: Wake up the display task.
                //
                _system.CP.WakeTask(TaskType.Display);
            }
        }

        private void LostSyncCallback(ulong skewNsec, object context)
        {
            if (_syncPresent)
            {
                //
                // Got sync, keep the display alive and reschedule ourselves.
                _lostSyncEvent = _system.Scheduler.Schedule(_lostSyncInterval, LostSyncCallback);
            }
            else
            {
                // No sync since the last callback, blank the display and stop the sync callback.
                _system.Display.Clear();
            }

            _syncPresent = false;
        }

        // Control bits
        private bool _displayOn;
        private bool _blank;
        private bool _picture;
        private bool _invert;
        private bool _oddLine;

        // Border bitmap
        private ushort _displayBorder;

        // Border phasing.  The border pattern is lined up with the picture's stipple by measuring where the
        // picture starts rather than assuming it: _firstPictureScanline[field] is the _scanline of the first
        // picture line seen since the last vertical sync, measured separately for the even (0) and odd (1)
        // interlace fields (they differ by the odd field's extra sync line).  This is guest- and OS-dependent,
        // so it must be measured rather than assumed.  _captureFirstPicture latches the measurement once per
        // field.  _borderPhaseAdjust is a one-time global nudge for the stipple's own phase: +2 swaps the two
        // border bytes, +1 shifts by one scanline.
        private int[] _firstPictureScanline = { 36, 37 };
        private bool _captureFirstPicture;
        private const int _borderPhaseAdjust = 2;

        // Control FIFO.  Max 16 entries.
        private Queue<ushort> _fifo;

        // Scanline
        private int _scanline;
        private ushort[] _scanlineData = new ushort[64 + 4];     // 1024 bits picture, 32 bits border on either side

        private bool _syncPresent;

        private DSystem _system;

        //
        // Timing and events
        //        
        private readonly ulong _horizontalRetraceDelay = (ulong)(28.8 * Conversion.UsecToNsec);       // 28.8uS 

        private Event _lostSyncEvent;
        private readonly ulong _lostSyncInterval = (ulong)(52.91 * Conversion.MsecToNsec);            // 53ms (one frame time)
    }
}
