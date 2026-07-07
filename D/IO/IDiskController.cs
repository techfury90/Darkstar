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

namespace D.IO
{
    /// <summary>
    /// The CP's disk controller interface: the "K" I/O functions the Dandelion
    /// microcode uses to talk to whichever rigid disk controller is present.
    /// The Shugart SA1000 controller (standard HSIO board) and the Trident
    /// Large Disk controller (HSIO-L board, Large-Capacity servers) share these
    /// CP functions but implement very different register semantics behind them.
    /// The boot microcode probes which controller is present via KTest (see
    /// Phase0.mc "Trident/SA": the low KTest bit distinguishes HSIO-L from the
    /// SA1000 controller) and branches accordingly.
    /// </summary>
    public interface IDiskController
    {
        void Reset();

        ushort ReadKIData();

        ushort ReadKStatus();

        ushort ReadKTest();

        void KStrobe();

        void ClrKFlags();

        void SetKOData(ushort value);

        void SetKCtl(ushort value);

        void SetKCmd(ushort value);
    }
}
