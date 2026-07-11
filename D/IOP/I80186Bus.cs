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

namespace D.IOP
{
    /// <summary>
    /// The 80186's physical memory bus.  Addresses are 20-bit physical
    /// (segment*16 + offset, masked to 1 MB); the Dove memory bus decodes
    /// EPROM / SRAM / the map-register window into CP VM behind this.
    ///
    /// The CPU core intercepts the integrated Peripheral Control Block (PCB)
    /// window itself before an access reaches this bus, so implementations here
    /// only see "external" memory.
    /// </summary>
    public interface IPhysicalMemory
    {
        byte ReadByte(int address);
        void WriteByte(int address, byte value);

        // Word accesses may be unaligned; the default 8086/186 behaviour is two
        // successive byte accesses with the offset wrapping inside the segment,
        // but at the physical-bus level we present a simple little-endian pair.
        ushort ReadWord(int address);
        void WriteWord(int address, ushort value);
    }

    /// <summary>
    /// The 80186's 16-bit I/O bus.  Ports are 16-bit; the Dove I/O bus decodes
    /// the PCS chip-select blocks (PCS0..PCS6) behind this.  The CPU core
    /// intercepts the PCB window (when I/O-mapped) itself before an access
    /// reaches this bus.
    /// </summary>
    public interface IIOBus186
    {
        byte ReadByte(ushort port);
        void WriteByte(ushort port, byte value);
        ushort ReadWord(ushort port);
        void WriteWord(ushort port, ushort value);
    }
}
