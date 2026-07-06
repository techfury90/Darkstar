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
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Net;
using System.Net.Sockets;
using System.Threading;

namespace D.IOP
{
    /// <summary>
    /// The IOP's "Printer" port (SysDefs.asm: PrinterBase, 0x88/0x89): an i8251A USART
    /// clocked by i8253 counter 0.  Despite the register names, this is the TTY /
    /// maintenance console port -- it is driven by the IOP firmware's TTYTask on behalf
    /// of Pilot's TTY head, and on a headless server (e.g. Services 11.x) it is the
    /// System Administrator's terminal.  (The LSEP print-engine UART is a different
    /// 8251A at 0x90-0x92, and the Z80-SIO RS-232-C data-comms port is at 0x98-0x9B.)
    ///
    /// The host side of the line is exposed as a TCP listener on the loopback address
    /// (port set by Configuration.TTYConsolePort, 0 disables): connect with a telnet
    /// client to interact with the console.  Basic telnet IAC negotiation is absorbed
    /// so a stock telnet client works in character-at-a-time mode.
    ///
    /// The USART emulation covers what TTYTask.asm uses: the mode/command write
    /// sequencing (including the Reset0/1/2 dance), Rx/Tx enables, DTR/RTS, error
    /// reset, and the status bits TxRDY/RxRDY/TxEMPTY/DSR.  Errors are never reported
    /// (a virtual line is clean), and DSR is always asserted (a terminal is always
    /// connected).  The TxRDY/RxRDY *pin* states are exposed via TxRequest/RxRequest
    /// for the interrupt-status register at port 0xE9 that TTYTask polls.
    /// </summary>
    public class Printer : IIOPDevice
    {
        public Printer()
        {
            Reset();
            StartConsoleServer();
        }

        public void Reset()
        {
            _controlState = ControlState.Mode;
            _mode = 0;
            _command = 0;
            _breakDetected = false;

            byte discard;
            while (_rxQueue.TryDequeue(out discard))
            {
            }
        }

        /// <summary>
        /// State of the USART's TxRDY pin (true = ready), polled by the IOP main loop
        /// via the interrupt status register (0xE9, PtrTxReqMask, active low there).
        /// The pin is TxEnable && CTS && buffer-empty; the virtual line always has CTS
        /// asserted and an empty buffer, so it reduces to the Tx enable.
        /// </summary>
        public bool TxRequest
        {
            get { return (_command & CMD_TXEN) != 0; }
        }

        /// <summary>
        /// State of the USART's RxRDY pin (true = character available), polled via the
        /// interrupt status register (0xE9, PtrRxReqMask, active low there).
        /// </summary>
        public bool RxRequest
        {
            get { return (_command & CMD_RXEN) != 0 && !_rxQueue.IsEmpty; }
        }

        public int[] ReadPorts
        {
            get { return _readPorts; }
        }

        public int[] WritePorts
        {
            get { return _writePorts; }
        }

        public byte ReadPort(int port)
        {
            byte value = 0;
            switch (port)
            {
                case 0x88:
                    //
                    // Rx data.  TTYTask only reads this after RxRDY; if read blind
                    // (diagnostics, no terminal) return the last transmitted byte,
                    // preserving the loopback behavior the rigid diags expect.
                    //
                    if ((_command & CMD_RXEN) == 0 || !_rxQueue.TryDequeue(out value))
                    {
                        value = _lastTxData;
                    }
                    if (Log.Enabled) Log.Write(LogComponent.IOPPrinter, "TTY data read 0x{0:x2}", value);
                    break;

                case 0x89:
                    //
                    // Status: TxRDY and TxEMPTY always (host output is instantaneous),
                    // DSR always (a terminal is always attached), no line errors,
                    // RxRDY when an input byte is queued, BRKDET while a received
                    // break is pending (cleared by the Error Reset command).
                    //
                    value = (byte)(STATUS_TXRDY | STATUS_TXEMPTY | STATUS_DSR |
                        (RxRequest ? STATUS_RXRDY : 0x00) |
                        (_breakDetected ? STATUS_BRKDET : 0x00));
                    break;
            }

            return value;
        }

        public void WritePort(int port, byte data)
        {
            switch (port)
            {
                case 0x88:
                    if (Log.Enabled) Log.Write(LogComponent.IOPPrinter, "TTY data write 0x{0:x2}", data);
                    _lastTxData = data;
                    SendToConsole(data);
                    break;

                case 0x89:
                    WriteControl(data);
                    break;
            }
        }

        /// <summary>
        /// The 8251's control write sequencing: after reset the first control write is
        /// the mode; in sync modes the next one or two writes are sync characters; all
        /// further writes are commands until a command with IR (internal reset) starts
        /// the sequence over.  TTYTask's Reset0/Reset1/Reset2 (0x80, 0x00, 0x40) dance
        /// lands in the command state from any starting state.
        /// </summary>
        private void WriteControl(byte data)
        {
            switch (_controlState)
            {
                case ControlState.Mode:
                    _mode = data;
                    if ((data & MODE_BAUD_MASK) == 0)
                    {
                        // Sync mode: one sync char if SCS (bit 7) set, else two.
                        _controlState = (data & MODE_SCS) != 0 ? ControlState.Sync2 : ControlState.Sync1;
                    }
                    else
                    {
                        // Async mode.
                        _controlState = ControlState.Command;
                    }
                    if (Log.Enabled) Log.Write(LogComponent.IOPPrinter, "TTY mode 0x{0:x2}", data);
                    break;

                case ControlState.Sync1:
                    _controlState = ControlState.Sync2;
                    break;

                case ControlState.Sync2:
                    _controlState = ControlState.Command;
                    break;

                case ControlState.Command:
                    _command = data;
                    if (Log.Enabled) Log.Write(LogComponent.IOPPrinter, "TTY command 0x{0:x2}", data);

                    if ((data & CMD_IR) != 0)
                    {
                        // Internal reset: next control write is a mode.
                        _command = 0;
                        _controlState = ControlState.Mode;
                        _breakDetected = false;
                    }

                    if ((data & CMD_ERESET) != 0)
                    {
                        // Error reset: clears the error/break status bits.
                        _breakDetected = false;
                    }

                    // Send-break and hunt mode are ignored.
                    break;
            }
        }

        #region Host console (TCP)

        private void StartConsoleServer()
        {
            if (_serverStarted || Configuration.TTYConsolePort == 0)
            {
                return;
            }
            _serverStarted = true;

            try
            {
                _listener = new TcpListener(IPAddress.Loopback, Configuration.TTYConsolePort);
                _listener.Start();
            }
            catch (SocketException e)
            {
                Console.WriteLine("TTY console: unable to listen on port {0} ({1}); console disabled.",
                    Configuration.TTYConsolePort, e.Message);
                _listener = null;
                return;
            }

            Console.WriteLine("TTY console: telnet to localhost:{0} for the maintenance terminal.",
                Configuration.TTYConsolePort);

            _acceptThread = new Thread(AcceptLoop);
            _acceptThread.IsBackground = true;
            _acceptThread.Start();

            _writeThread = new Thread(WriteLoop);
            _writeThread.IsBackground = true;
            _writeThread.Start();
        }

        private void AcceptLoop()
        {
            while (true)
            {
                TcpClient client = null;
                try
                {
                    client = _listener.AcceptTcpClient();
                    client.NoDelay = true;

                    NetworkStream stream = client.GetStream();

                    //
                    // Ask the telnet client for character-at-a-time mode with no local
                    // echo (the console echoes): IAC WILL ECHO, IAC WILL SGA.
                    //
                    stream.Write(new byte[] { 0xff, 0xfb, 0x01, 0xff, 0xfb, 0x03 }, 0, 6);

                    lock (_txLock)
                    {
                        _stream = stream;
                    }
                    _txEvent.Set();     // flush any backlogged output (e.g. a herald)

                    ClientReadLoop(stream);
                }
                catch (Exception)
                {
                    // Client or listener trouble; drop the connection and re-accept.
                }
                finally
                {
                    lock (_txLock)
                    {
                        _stream = null;
                    }
                    if (client != null)
                    {
                        client.Close();
                    }
                }
            }
        }

        /// <summary>
        /// Moves bytes from the connected client into the Rx queue, absorbing telnet
        /// IAC negotiation so a stock telnet client can be used.
        /// </summary>
        private void ClientReadLoop(NetworkStream stream)
        {
            byte[] buffer = new byte[512];
            TelnetState telnet = TelnetState.Data;

            while (true)
            {
                int read = stream.Read(buffer, 0, buffer.Length);
                if (read <= 0)
                {
                    return;     // client disconnected
                }

                for (int i = 0; i < read; i++)
                {
                    byte b = buffer[i];
                    switch (telnet)
                    {
                        case TelnetState.Data:
                            if (b == 0xff)
                            {
                                telnet = TelnetState.Iac;
                            }
                            else
                            {
                                _rxQueue.Enqueue(b);
                            }
                            break;

                        case TelnetState.Iac:
                            if (b == 0xff)
                            {
                                _rxQueue.Enqueue(b);        // escaped 0xff
                                telnet = TelnetState.Data;
                            }
                            else if (b == 0xf3)
                            {
                                // IAC BREAK: surface as a received line break -- a null
                                // character with the break-detected status bit, which
                                // TTYTask reports to Pilot.  (Send via telnet's
                                // "send brk" / PuTTY's Special Command > Break.)
                                _breakDetected = true;
                                _rxQueue.Enqueue(0x00);
                                telnet = TelnetState.Data;
                            }
                            else if (b == 0xfa)
                            {
                                telnet = TelnetState.Subneg;
                            }
                            else if (b >= 0xfb)
                            {
                                telnet = TelnetState.IacOption;     // WILL/WONT/DO/DONT
                            }
                            else
                            {
                                telnet = TelnetState.Data;          // 2-byte command
                            }
                            break;

                        case TelnetState.IacOption:
                            telnet = TelnetState.Data;              // swallow option byte
                            break;

                        case TelnetState.Subneg:
                            if (b == 0xff)
                            {
                                telnet = TelnetState.SubnegIac;
                            }
                            break;

                        case TelnetState.SubnegIac:
                            telnet = b == 0xf0 ? TelnetState.Data : TelnetState.Subneg;
                            break;
                    }
                }
            }
        }

        /// <summary>
        /// Queues a transmitted byte for the console client.  Never blocks the
        /// emulation thread; when no client is connected, output is buffered
        /// (bounded) and flushed on the next connect.
        /// </summary>
        private void SendToConsole(byte data)
        {
            lock (_txLock)
            {
                _txQueue.Enqueue(data);
                while (_txQueue.Count > MaxTxBacklog)
                {
                    _txQueue.Dequeue();
                }
            }
            _txEvent.Set();
        }

        private void WriteLoop()
        {
            while (true)
            {
                _txEvent.WaitOne();

                byte[] pending = null;
                NetworkStream stream;
                lock (_txLock)
                {
                    stream = _stream;
                    if (stream != null && _txQueue.Count > 0)
                    {
                        pending = _txQueue.ToArray();
                        _txQueue.Clear();
                    }
                }

                if (pending != null)
                {
                    try
                    {
                        stream.Write(pending, 0, pending.Length);
                    }
                    catch (Exception)
                    {
                        // Client went away mid-write; the accept loop will clean up.
                    }
                }
            }
        }

        #endregion

        private enum ControlState
        {
            Mode = 0,
            Sync1,
            Sync2,
            Command,
        }

        private enum TelnetState
        {
            Data = 0,
            Iac,
            IacOption,
            Subneg,
            SubnegIac,
        }

        // i8251 mode bits
        private const byte MODE_BAUD_MASK = 0x03;   // 00 = sync mode
        private const byte MODE_SCS = 0x80;         // single character sync

        // i8251 command bits
        private const byte CMD_TXEN = 0x01;
        private const byte CMD_DTR = 0x02;
        private const byte CMD_RXEN = 0x04;
        private const byte CMD_SBRK = 0x08;
        private const byte CMD_ERESET = 0x10;
        private const byte CMD_RTS = 0x20;
        private const byte CMD_IR = 0x40;

        // i8251 status bits
        private const byte STATUS_TXRDY = 0x01;
        private const byte STATUS_RXRDY = 0x02;
        private const byte STATUS_TXEMPTY = 0x04;
        private const byte STATUS_BRKDET = 0x40;
        private const byte STATUS_DSR = 0x80;

        // USART state
        private ControlState _controlState;
        private byte _mode;
        private byte _command;
        private byte _lastTxData;
        private volatile bool _breakDetected;

        // Host console plumbing
        private const int MaxTxBacklog = 8192;
        private readonly ConcurrentQueue<byte> _rxQueue = new ConcurrentQueue<byte>();
        private readonly Queue<byte> _txQueue = new Queue<byte>();
        private readonly object _txLock = new object();
        private readonly AutoResetEvent _txEvent = new AutoResetEvent(false);
        private bool _serverStarted;
        private TcpListener _listener;
        private NetworkStream _stream;
        private Thread _acceptThread;
        private Thread _writeThread;

        private readonly int[] _readPorts = new int[]
           {
                0x88,       // data
                0x89,       // status
           };

        private readonly int[] _writePorts = new int[]
            {
                0x88,       // data
                0x89,       // mode/commands
            };
    }
}
