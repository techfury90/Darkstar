using System;
using System.Runtime.InteropServices;
using System.Threading;
using System.Windows.Forms;
using SDL2;

namespace D.Doovke
{
    /// <summary>
    /// Interactive front end for the Dove/Daybreak machine.
    ///
    /// Forked from Darkstar's DWindow: a WinForms host with a Panel whose handle SDL renders
    /// into (SDL_CreateWindowFrom), an ARGB8888 streaming texture, and an SDL message-loop
    /// thread.  What differs is the source of pixels (DoveDisplayController's 1-byte-per-pixel
    /// mono frame instead of the DLion display) and the keyboard encoding.
    /// </summary>
    public sealed class DoovkeWindow : Form
    {
        private DoovkeMachine _machine;
        private readonly object _machineLock = new object();

        // ---- display ----
        private readonly Panel _displayBox;
        private IntPtr _sdlWindow = IntPtr.Zero;
        private IntPtr _sdlRenderer = IntPtr.Zero;
        private IntPtr _displayTexture = IntPtr.Zero;
        private int[] _32bppDisplayBuffer;
        private int _displayWidth = 1152;
        private int _displayHeight = 861;
        private readonly ReaderWriterLockSlim _textureLock = new ReaderWriterLockSlim();
        private Thread _sdlThread;
        private volatile bool _sdlRunning;

        /// <summary>
        /// Display refresh interval.  The Dove display is INTERLACED, like the DLion's: the
        /// ~26.3 ms / ~38 Hz figure that falls out of DoveIOPIO's retrace period (210400 clocks
        /// at 8 MHz) is the FRAME rate, i.e. both fields.  A field therefore arrives every
        /// ~13.2 ms (~76 Hz), and that is the cadence the screen should be redrawn at.
        /// </summary>
        private const int RefreshIntervalMs = 13;

        // ---- machine ----
        private Thread _machineThread;
        private volatile bool _machineRunning;
        private volatile bool _paused;
        private const int BatchSteps = 50000;   // steps per lock acquisition

        private readonly System.Windows.Forms.Timer _refreshTimer;
        private readonly StatusStrip _statusStrip;
        private readonly ToolStripStatusLabel _statusLabel;
        private string _floppyPath;

        public DoovkeWindow(DoovkeMachine machine, string initialFloppy)
        {
            _machine = machine;
            _floppyPath = initialFloppy;

            Text = "Doovke - Xerox 6085 (Dove/Daybreak)";
            StartPosition = FormStartPosition.CenterScreen;
            KeyPreview = true;

            _displayBox = new Panel
            {
                Dock = DockStyle.Fill,
                BackColor = System.Drawing.Color.Black,
                TabStop = false,
            };
            Controls.Add(_displayBox);

            _statusLabel = new ToolStripStatusLabel("starting...");
            _statusStrip = new StatusStrip();
            _statusStrip.Items.Add(_statusLabel);
            Controls.Add(_statusStrip);

            Controls.Add(BuildMenu());

            _32bppDisplayBuffer = new int[_displayWidth * _displayHeight];

            _refreshTimer = new System.Windows.Forms.Timer { Interval = RefreshIntervalMs };
            _refreshTimer.Tick += (s, e) => { PollMouse(); UpdateAndRender(); };

            Load += OnWindowLoad;
            FormClosing += (s, e) => Shutdown();
        }

        /// <summary>
        /// Size the window so the display panel is exactly the framebuffer's size, i.e. 1:1 with
        /// no scaling.  SDL stretches the texture to the panel, so the panel has to be the exact
        /// pixel size or the image is resampled.  The menu and status bars sit outside it.
        /// </summary>
        private void SizeToDisplay()
        {
            int chrome = 0;
            if (MainMenuStrip != null) chrome += MainMenuStrip.Height;
            if (_statusStrip != null) chrome += _statusStrip.Height;

            ClientSize = new System.Drawing.Size(_displayWidth, _displayHeight + chrome);

            // If that would not fit on the screen, fall back to the largest size that does --
            // still 1:1 for as much of the display as is visible, and the window is resizable.
            var wa = Screen.FromControl(this).WorkingArea;
            if (Width > wa.Width || Height > wa.Height)
            {
                Width = Math.Min(Width, wa.Width);
                Height = Math.Min(Height, wa.Height);
            }
        }

        private MenuStrip BuildMenu()
        {
            var menu = new MenuStrip();

            var file = new ToolStripMenuItem("&File");
            file.DropDownItems.Add("&Load Floppy...", null, (s, e) => OnLoadFloppy());
            file.DropDownItems.Add("&Change Floppy...", null, (s, e) => OnChangeFloppy());
            file.DropDownItems.Add("&Eject Floppy", null, (s, e) => { lock (_machineLock) _machine.EjectFloppy(0); });
            file.DropDownItems.Add(new ToolStripSeparator());
            file.DropDownItems.Add("E&xit", null, (s, e) => Close());
            menu.Items.Add(file);

            var mach = new ToolStripMenuItem("&Machine");
            var pause = new ToolStripMenuItem("&Pause") { CheckOnClick = true };
            pause.CheckedChanged += (s, e) => _paused = pause.Checked;
            mach.DropDownItems.Add(pause);
            mach.DropDownItems.Add(new ToolStripSeparator());

            // Boot-device selection takes a raw byte: 0x63 = F1 .. 0x6C = F10.
            var boot = new ToolStripMenuItem("Boot &Device Key");
            for (int i = 0; i < 10; i++)
            {
                byte code = (byte)(0x63 + i);
                boot.DropDownItems.Add("F" + (i + 1) + "  (station " + code + ")", null,
                                       (s, e) => { SendKey(code, true); SendKey(code, false); });
            }
            mach.DropDownItems.Add(boot);
            mach.DropDownItems.Add("Send Scan &Code...", null, (s, e) => OnSendScanCode());
            mach.DropDownItems.Add(new ToolStripSeparator());
            mach.DropDownItems.Add("&Reset (Power Cycle)", null, (s, e) => PowerCycle(null));
            // Boot-device keys are only accepted during the selection screen early in the boot,
            // so choosing a device after the fact means power-cycling first.  These do both.
            var bootFrom = new ToolStripMenuItem("Power Cycle && &Boot From");
            for (int i = 0; i < 10; i++)
            {
                byte code = (byte)(0x63 + i);
                bootFrom.DropDownItems.Add("F" + (i + 1) + "  (0x" + code.ToString("X2") + ")", null,
                                           (s, e) => PowerCycle(code));
            }
            mach.DropDownItems.Add(bootFrom);
            menu.Items.Add(mach);

            MainMenuStrip = menu;
            return menu;
        }

        // ---- SDL setup (fork of DWindow-IO) ---------------------------------------------

        private void OnWindowLoad(object sender, EventArgs e)
        {
            if (SDL.SDL_Init(SDL.SDL_INIT_VIDEO) < 0)
                throw new InvalidOperationException("SDL_Init failed.");

            _sdlWindow = SDL.SDL_CreateWindowFrom(_displayBox.Handle);
            if (_sdlWindow == IntPtr.Zero)
                throw new InvalidOperationException("SDL_CreateWindowFrom failed.");

            _sdlRenderer = SDL.SDL_CreateRenderer(_sdlWindow, -1, SDL.SDL_RendererFlags.SDL_RENDERER_ACCELERATED);
            if (_sdlRenderer == IntPtr.Zero)
                _sdlRenderer = SDL.SDL_CreateRenderer(_sdlWindow, -1, SDL.SDL_RendererFlags.SDL_RENDERER_SOFTWARE);
            if (_sdlRenderer == IntPtr.Zero)
                throw new InvalidOperationException("SDL_CreateRenderer failed.");

            CreateDisplayTexture();

            _sdlRunning = true;
            _sdlThread = new Thread(SDLMessageLoopThread) { IsBackground = true, Name = "Doovke SDL" };
            _sdlThread.Start();

            SizeToDisplay();
            StartMachine();
            _refreshTimer.Start();
        }

        private void CreateDisplayTexture()
        {
            _textureLock.EnterWriteLock();
            try
            {
                SDL.SDL_SetHint(SDL.SDL_HINT_RENDER_SCALE_QUALITY, "nearest");
                if (_displayTexture != IntPtr.Zero) SDL.SDL_DestroyTexture(_displayTexture);
                _displayTexture = SDL.SDL_CreateTexture(
                    _sdlRenderer,
                    SDL.SDL_PIXELFORMAT_ARGB8888,
                    (int)SDL.SDL_TextureAccess.SDL_TEXTUREACCESS_STREAMING,
                    _displayWidth,
                    _displayHeight);
                if (_displayTexture == IntPtr.Zero)
                    throw new InvalidOperationException("SDL_CreateTexture failed.");
            }
            finally { _textureLock.ExitWriteLock(); }
        }

        private void SDLMessageLoopThread()
        {
            while (_sdlRunning)
            {
                SDL.SDL_Event e;
                if (SDL.SDL_WaitEventTimeout(out e, 100) != 0)
                {
                    if (e.type == SDL.SDL_EventType.SDL_QUIT) break;
                }
            }
        }

        /// <summary>Snapshot the machine's framebuffer, expand mono to ARGB, and present.</summary>
        private void UpdateAndRender()
        {
            byte[] frame; int w, h; long instr; bool changing; bool haveDisk;
            lock (_machineLock)
            {
                frame = _machine.RenderFrame(out w, out h);
                instr = _machine.IopInstructions;
                changing = _machine.DiskChangeInProgress;
                haveDisk = _machine.Io.Fdc.Drives[0] != null;
            }

            if (frame != null && w > 0 && h > 0)
            {
                if (w != _displayWidth || h != _displayHeight)
                {
                    _displayWidth = w; _displayHeight = h;
                    _32bppDisplayBuffer = new int[w * h];
                    CreateDisplayTexture();
                    SizeToDisplay();
                }

                // Mono (1 byte per pixel, non-zero = lit) -> opaque black/white ARGB.
                unchecked
                {
                    int white = (int)0xFFFFFFFF;
                    int black = (int)0xFF000000;
                    for (int i = 0; i < _32bppDisplayBuffer.Length; i++)
                        _32bppDisplayBuffer[i] = frame[i] != 0 ? white : black;
                }
                RenderDisplay();
            }

            if (_machineFault != null)
            {
                _statusLabel.Text = "MACHINE STOPPED -- " + _machineFault;
                return;
            }
            _statusLabel.Text = string.Format("IOP {0:N0}   CP {1:N0}   floppy: {2}{3}{4}",
                instr, _machine.Cp.InstructionCount,
                haveDisk ? System.IO.Path.GetFileName(_floppyPath ?? "(image)") : "(empty)",
                changing ? "   [changing disk]" : string.Empty,
                _paused ? "   [paused]" : string.Empty);
        }

        private void RenderDisplay()
        {
            if (_displayTexture == IntPtr.Zero) return;
            _textureLock.EnterReadLock();
            try
            {
                IntPtr bits; int pitch;
                if (SDL.SDL_LockTexture(_displayTexture, IntPtr.Zero, out bits, out pitch) != 0) return;
                Marshal.Copy(_32bppDisplayBuffer, 0, bits, _32bppDisplayBuffer.Length);
                SDL.SDL_UnlockTexture(_displayTexture);

                SDL.SDL_RenderCopy(_sdlRenderer, _displayTexture, IntPtr.Zero, IntPtr.Zero);
                SDL.SDL_RenderPresent(_sdlRenderer);
            }
            finally { _textureLock.ExitReadLock(); }
        }

        // ---- machine thread -------------------------------------------------------------

        private void StartMachine()
        {
            _machineRunning = true;
            _machineThread = new Thread(MachineLoop) { IsBackground = true, Name = "Doovke machine" };
            _machineThread.Start();
        }

        private void MachineLoop()
        {
            while (_machineRunning)
            {
                if (_paused) { Thread.Sleep(20); continue; }
                try
                {
                    lock (_machineLock)
                    {
                        for (int i = 0; i < BatchSteps && _machineRunning && !_paused; i++) _machine.Step();
                    }
                }
                catch (Exception e)
                {
                    // This thread is a background thread: an unhandled exception here would
                    // terminate the process with no explanation.  Stop and report instead.
                    _machineFault = e.GetType().Name + ": " + e.Message;
                    _machineRunning = false;
                }
            }
        }

        /// <summary>
        /// Power-cycle: tear the machine down and build a fresh one from the same ROM/EEPROM,
        /// remounting whatever image is in the drive.  Rebuilding rather than poking a reset line
        /// is what makes it a true cold start -- memory, devices and the control store all clear.
        /// If a boot-device key is given, it is queued for the selection screen (twice, as the
        /// firmware requires).
        /// </summary>
        private void PowerCycle(byte? bootKey)
        {
            _machineRunning = false;
            var t = _machineThread;
            if (t != null) t.Join(2000);

            lock (_machineLock)
            {
                var fresh = new DoovkeMachine(_machine.BootRomPath, _machine.ConfigEepromPath);
                if (!string.IsNullOrEmpty(_floppyPath)) fresh.LoadFloppy(0, _floppyPath);
                if (bootKey.HasValue) fresh.ScheduleBootDeviceKey(bootKey.Value);
                _machine = fresh;
            }

            StartMachine();
        }

        private void Shutdown()
        {
            _refreshTimer.Stop();
            _machineRunning = false;
            var mt = _machineThread; if (mt != null) mt.Join(2000);

            _sdlRunning = false;
            var ev = new SDL.SDL_Event { type = SDL.SDL_EventType.SDL_QUIT };
            SDL.SDL_PushEvent(ref ev);
            var st = _sdlThread; if (st != null) st.Join(1000);

            if (_displayTexture != IntPtr.Zero) SDL.SDL_DestroyTexture(_displayTexture);
            if (_sdlRenderer != IntPtr.Zero) SDL.SDL_DestroyRenderer(_sdlRenderer);
            if (_sdlWindow != IntPtr.Zero) SDL.SDL_DestroyWindow(_sdlWindow);
        }

        // ---- keyboard --------------------------------------------------------------------

        protected override void OnKeyDown(KeyEventArgs e)
        {
            // Alt releases a captured mouse, the same gesture Darkstar uses.
            if (e.Alt) { ReleaseMouse(); e.Handled = true; e.SuppressKeyPress = true; return; }

            byte station;
            if (DoovkeKeyboard.TryStation(e.KeyCode, out station))
            {
                // Auto-repeat resends KeyDown without an intervening KeyUp; the guest holds a
                // down/up bitmap, so repeats are redundant and only cost UART bandwidth.
                if (_keysDown.Add(e.KeyCode)) SendKey(station, true);
                e.Handled = true;
                e.SuppressKeyPress = true;
            }
            base.OnKeyDown(e);
        }

        protected override void OnKeyUp(KeyEventArgs e)
        {
            byte station;
            if (DoovkeKeyboard.TryStation(e.KeyCode, out station))
            {
                _keysDown.Remove(e.KeyCode);
                SendKey(station, false);
                e.Handled = true;
                e.SuppressKeyPress = true;
            }
            base.OnKeyUp(e);
        }

        /// <summary>Release every held key -- on focus loss, so nothing sticks down.</summary>
        private void ReleaseAllKeys()
        {
            foreach (var k in new System.Collections.Generic.List<Keys>(_keysDown))
            {
                byte station;
                if (DoovkeKeyboard.TryStation(k, out station)) SendKey(station, false);
            }
            _keysDown.Clear();
        }

        protected override void OnDeactivate(EventArgs e)
        {
            ReleaseAllKeys();
            ReleaseMouse();
            base.OnDeactivate(e);
        }

        private void SendKey(byte station, bool down)
        {
            lock (_machineLock) _machine.QueueKey(station, down);
        }

        // ---- mouse -----------------------------------------------------------------------
        //
        // Driven by polling rather than by mouse events: SDL owns the display panel's window
        // handle, so whether a given WM_MOUSEMOVE surfaces as a WinForms event or is consumed
        // by SDL is not something worth depending on.  Cursor.Position and Control.MouseButtons
        // read the real state regardless of who receives the messages.
        //
        // Motion is relative: while captured, the host pointer is warped back to the centre of
        // the display each poll and the distance it moved becomes the delta.

        private void PollMouse()
        {
            // Track the host buttons every poll so capture can trigger on the EDGE of a
            // press.  Level-triggering re-grabbed the pointer on any poll where a button
            // happened to be down -- which is every poll while a menu item is being clicked
            // or a dialog dragged, making the whole UI unusable.
            bool anyDown = Control.MouseButtons != MouseButtons.None;
            bool pressEdge = anyDown && !_hostButtonsWereDown;
            _hostButtonsWereDown = anyDown;

            if (!_mouseCaptured)
            {
                // Press inside the display captures, as in Darkstar -- but only when this
                // window is genuinely the one being used.  A modal dialog makes ActiveForm
                // the dialog, and an open menu drops down OVER the display area, so without
                // these the pointer gets grabbed the moment a menu is clicked.
                if (pressEdge
                    && Form.ActiveForm == this && ContainsFocus && !AnyMenuOpen()
                    && _displayBox.ClientRectangle.Contains(_displayBox.PointToClient(Cursor.Position)))
                {
                    CaptureMouse();
                }
                return;
            }

            var centre = _displayBox.PointToScreen(new System.Drawing.Point(
                _displayBox.ClientSize.Width / 2, _displayBox.ClientSize.Height / 2));
            var now = Cursor.Position;
            int dx = now.X - centre.X, dy = now.Y - centre.Y;
            if (dx != 0 || dy != 0)
            {
                // Display pixels are host pixels: the panel is sized 1:1 to the framebuffer.
                lock (_machineLock) _machine.QueueMouseMotion(dx, dy);
                Cursor.Position = centre;
            }

            var b = Control.MouseButtons;
            bool left = (b & MouseButtons.Left) != 0;
            bool right = (b & MouseButtons.Right) != 0;
            bool middle = (b & MouseButtons.Middle) != 0;

            // Straight through: the guest systems implement Point+Adjust chording for Menu
            // themselves, so synthesising it here would double up.
            SetButton(DoovkeKeyboard.ScanPoint, left, ref _pointDown);
            SetButton(DoovkeKeyboard.ScanAdjust, right, ref _adjustDown);
            SetButton(DoovkeKeyboard.ScanMenu, middle, ref _menuDown);
        }

        /// <summary>True while any menu drop-down is showing (it overlays the display area).</summary>
        private bool AnyMenuOpen()
        {
            if (MainMenuStrip == null) return false;
            foreach (ToolStripItem item in MainMenuStrip.Items)
            {
                ToolStripMenuItem mi = item as ToolStripMenuItem;
                if (mi != null && mi.DropDown != null && mi.DropDown.Visible) return true;
            }
            return false;
        }

        private void SetButton(byte scan, bool want, ref bool have)
        {
            if (want == have) return;
            have = want;
            lock (_machineLock) _machine.QueueMouseButton(scan, want);
        }

        private void CaptureMouse()
        {
            if (_mouseCaptured) return;
            _mouseCaptured = true;
            Cursor.Position = _displayBox.PointToScreen(new System.Drawing.Point(
                _displayBox.ClientSize.Width / 2, _displayBox.ClientSize.Height / 2));
            Cursor.Hide();
        }

        private void ReleaseMouse()
        {
            if (!_mouseCaptured) return;
            _mouseCaptured = false;
            // Let go of any button the guest still thinks is down.
            SetButton(DoovkeKeyboard.ScanPoint, false, ref _pointDown);
            SetButton(DoovkeKeyboard.ScanAdjust, false, ref _adjustDown);
            SetButton(DoovkeKeyboard.ScanMenu, false, ref _menuDown);
            Cursor.Show();
        }

        private readonly System.Collections.Generic.HashSet<Keys> _keysDown =
            new System.Collections.Generic.HashSet<Keys>();
        private bool _mouseCaptured;
        private bool _hostButtonsWereDown;
        private volatile string _machineFault;
        private bool _pointDown, _adjustDown, _menuDown;

        // ---- menu handlers ---------------------------------------------------------------

        private string PickImage()
        {
            using (var dlg = new OpenFileDialog())
            {
                dlg.Filter = "Floppy images (*.imd;*.img;*.dsk)|*.imd;*.img;*.dsk|All files (*.*)|*.*";
                return dlg.ShowDialog(this) == DialogResult.OK ? dlg.FileName : null;
            }
        }

        private void OnLoadFloppy()
        {
            string path = PickImage();
            if (path == null || !ImageIsReadable(path)) return;
            lock (_machineLock) _machine.LoadFloppy(0, path);
            _floppyPath = path;
        }

        private void OnChangeFloppy()
        {
            string path = PickImage();
            if (path == null || !ImageIsReadable(path)) return;
            // Goes through a real no-media gap so the guest latches the change.
            lock (_machineLock) _machine.ChangeFloppy(0, path);
            _floppyPath = path;
        }

        /// <summary>
        /// Report an unreadable image up front rather than letting it fail later on the
        /// machine thread, where the drive would just silently stay empty.
        /// </summary>
        private bool ImageIsReadable(string path)
        {
            string err = DoovkeMachine.ValidateImage(path);
            if (err == null) return true;
            string nl = Environment.NewLine;
            MessageBox.Show(this, "Cannot read this disk image:" + nl + nl + err + nl + nl
                            + "The drive has been left as it was.",
                            "Bad disk image", MessageBoxButtons.OK, MessageBoxIcon.Warning);
            return false;
        }

        private void OnSendScanCode()
        {
            using (var dlg = new Form())
            {
                dlg.Text = "Send Scan Code";
                dlg.FormBorderStyle = FormBorderStyle.FixedDialog;
                dlg.ClientSize = new System.Drawing.Size(260, 90);
                dlg.StartPosition = FormStartPosition.CenterParent;
                var lbl = new Label { Text = "Scan code (hex):", Left = 10, Top = 14, Width = 100 };
                var box = new TextBox { Left = 115, Top = 11, Width = 120, Text = "63" };
                var ok = new Button { Text = "Send", Left = 100, Top = 45, DialogResult = DialogResult.OK };
                dlg.Controls.Add(lbl); dlg.Controls.Add(box); dlg.Controls.Add(ok);
                dlg.AcceptButton = ok;
                if (dlg.ShowDialog(this) != DialogResult.OK) return;
                byte code;
                try { code = Convert.ToByte(box.Text.Trim(), 16); }
                catch { MessageBox.Show(this, "Not a hex byte: " + box.Text); return; }
                lock (_machineLock) _machine.QueueRaw(code);
            }
        }
    }
}
