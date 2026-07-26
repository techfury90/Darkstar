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
            ClientSize = new System.Drawing.Size(_displayWidth / 2, _displayHeight / 2 + 48);
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
            _refreshTimer.Tick += (s, e) => UpdateAndRender();

            Load += OnWindowLoad;
            FormClosing += (s, e) => Shutdown();
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
                boot.DropDownItems.Add("F" + (i + 1) + "  (0x" + code.ToString("X2") + ")", null,
                                       (s, e) => SendKey(code));
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
                lock (_machineLock)
                {
                    for (int i = 0; i < BatchSteps && _machineRunning && !_paused; i++) _machine.Step();
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
            byte code;
            if (DoovkeKeyboard.TryMap(e.KeyCode, e.Shift, out code))
            {
                SendKey(code);
                e.Handled = true;
                e.SuppressKeyPress = true;
            }
            base.OnKeyDown(e);
        }

        private void SendKey(byte code)
        {
            lock (_machineLock) _machine.InjectKey(code);
        }

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
            if (path == null) return;
            lock (_machineLock) _machine.LoadFloppy(0, path);
            _floppyPath = path;
        }

        private void OnChangeFloppy()
        {
            string path = PickImage();
            if (path == null) return;
            // Goes through a real no-media gap so the guest latches the change.
            lock (_machineLock) _machine.ChangeFloppy(0, path);
            _floppyPath = path;
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
                SendKey(code);
            }
        }
    }
}
