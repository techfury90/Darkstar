#!/usr/bin/env python3
"""Regenerate the harness's floppy-boot EEPROM from the museum U128 dump.

Why this script exists instead of a committed .bin:
    .gitignore excludes dove_build_kit/ because the Xerox firmware ROMs and the
    93C46 EEPROM dumps are copyrighted museum artifacts -- runtime inputs, not
    source.  eeprom_floppy.bin is a 3-byte edit of one of those dumps (it still
    carries the museum machine's serial number), so committing it would smuggle
    an EEPROM dump into the repo through the back door.  The DERIVATION is ours
    and is safe to track; the artifact is not.

Usage (from the repo root):
    python dove_build_kit/tools/tracerom/make_eeprom_floppy.py

Produces dove_build_kit/tools/tracerom/eeprom_floppy.bin, which the harness
wants as DOVE_EEPROM (see HANDOFF.md section 0).
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, "..", "..", "firmware", "config",
                   "U128_IOP_sn_071888_08CA_8kCS_3.7mb.bin")
DST = os.path.join(HERE, "eeprom_floppy.bin")

# The 3 bytes that differ from the stock U128 dump: {offset: (expect, want)}.
# 42 selects the default boot device (0x00 -> 0x04 = boot floppy rather than
# rigid disk); 125 and 127 are the config checksum the IOP verifies at boot.
# Direction verified against the real artifact:
#   cmp -l firmware/config/U128_IOP_sn_071888_08CA_8kCS_3.7mb.bin tools/tracerom/eeprom_floppy.bin
EDITS = {42: (0x00, 0x04), 125: (0x1D, 0x25), 127: (0xE2, 0xDA)}

def main():
    if not os.path.exists(SRC):
        sys.exit("missing museum U128 dump: %s\n"
                 "dove_build_kit/ is gitignored -- restore the kit first." % SRC)

    data = bytearray(open(SRC, "rb").read())
    if len(data) != 128:
        sys.exit("expected a 128-byte 93C46 image, got %d bytes" % len(data))

    for off, (expect, want) in sorted(EDITS.items()):
        if data[off] != expect:
            sys.exit("byte %d is 0x%02X, expected 0x%02X -- wrong source image?"
                     % (off, data[off], expect))
        data[off] = want

    open(DST, "wb").write(bytes(data))
    print("wrote %s (%d bytes)" % (DST, len(data)))
    print("expected md5: ce6685895a6a9eb68597759a6e389de4")

if __name__ == "__main__":
    main()
