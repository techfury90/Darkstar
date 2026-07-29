#!/usr/bin/env python3
"""Decode a DOVE_MP_TRACE capture into the MP codes the panel displayed.

On Daybreak the maintenance-panel code IS the 16x16 mouse cursor sprite: the ROM
draws four 5x7 digits into the 32-byte buffer at ED00-ED1F.  DOVE_MP_TRACE logs
that buffer, raw, on every change with a CP instruction count; this turns it back
into digits.

    python3 mpdecode.py mptrace.txt [min-hold-cpi]

Layout: rows 0-6 are the top two digits, rows 8-14 the bottom two; the HIGH byte
of each row is the left digit and the LOW byte the right.  Glyphs are 5x7 shifted
left 3 within the byte, so normalise with >>3.

Decode offline rather than in the emulator.  The opcode-level hook that preceded
this (@WRMP, zESC alpha 0x77) reported 259 posts that never happened and missed
every code that was actually on screen -- the sprite is the panel, and the bytes
are the bytes.

NOTE the panel blinks 8888 BETWEEN digits of a spelled-out message, so a
diagnostic reads as 9999 8888 7649 8888 0223 ... -- filter 8888 to recover the
payload.  A 76xx code after 9999 means a basic-workstation-software problem, and
the groups that follow spell a file name two digits per letter (A=01): 0223 1919
2021 0606 = B W S S T U F F.
"""
import sys, re

# Several digits have two glyph forms in this ROM; both are listed.
FONT = {
    (0x0E,0x11,0x11,0x11,0x11,0x11,0x0E): '0',
    (0x04,0x0C,0x04,0x04,0x04,0x04,0x0E): '1',
    (0x04,0x0C,0x14,0x04,0x04,0x04,0x1F): '1',
    (0x0E,0x0C,0x14,0x04,0x04,0x04,0x1F): '1',
    (0x0E,0x11,0x01,0x02,0x04,0x08,0x1F): '2',
    (0x0E,0x11,0x02,0x04,0x08,0x10,0x1F): '2',
    (0x0E,0x11,0x01,0x06,0x01,0x11,0x0E): '3',
    (0x12,0x12,0x12,0x1F,0x02,0x02,0x02): '4',
    (0x1F,0x10,0x1E,0x01,0x01,0x11,0x0E): '5',
    (0x06,0x08,0x10,0x1E,0x11,0x11,0x0E): '6',
    (0x0E,0x11,0x10,0x1E,0x11,0x11,0x0E): '6',
    (0x1F,0x01,0x02,0x04,0x08,0x08,0x08): '7',
    (0x0E,0x11,0x11,0x0E,0x11,0x11,0x0E): '8',
    (0x0E,0x11,0x11,0x0E,0x11,0x11,0x1F): '8',
    (0x0E,0x11,0x11,0x0F,0x01,0x11,0x0E): '9',
    (0,0,0,0,0,0,0):                      ' ',
}

def digits(b):
    out = []
    for base, idx in ((0,0), (0,1), (8,0), (8,1)):
        pat = tuple(b[(base+r)*2+idx] for r in range(7))
        if any(p & 0xE0 for p in pat):      # left-shifted within the byte
            pat = tuple(p >> 3 for p in pat)
        out.append(FONT.get(pat, '?'))
    return "".join(out)

def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "mptrace.txt"
    hold = int(sys.argv[2]) if len(sys.argv) > 2 else 60000
    rows = []
    for line in open(path, encoding="utf-8", errors="replace"):
        m = re.match(r"CPi=(\d+) (.+)", line.strip())
        if m:
            rows.append((int(m.group(1)), [int(x, 16) for x in m.group(2).split("-")]))
    if not rows:
        print("no sprite samples in", path); return

    # Only states that PERSIST are real codes; the rest are mid-redraw, since the
    # ROM rewrites the sprite a byte at a time.
    prev, seq = None, []
    for i, (cpi, b) in enumerate(rows):
        nxt = rows[i+1][0] if i+1 < len(rows) else cpi + 10**9
        if nxt - cpi > hold:
            d = digits(b)
            if d != prev:
                seq.append((cpi, nxt - cpi, d))
                prev = d
    print("%d sprite changes, %d held states\n" % (len(rows), len(seq)))
    for cpi, dur, d in seq:
        print("  CPi %13d  held %12d  ->  MP %s" % (cpi, dur, d))

    payload = [d for _, _, d in seq if d != "8888"]
    if "9999" in payload:
        i = payload.index("9999")
        msg = payload[i:]
        print("\ndiagnostic message (8888 separators removed):")
        print("  " + " ".join(msg[:20]))
        letters = "".join(
            chr(64 + int(p[k:k+2])) if p[k:k+2].isdigit() and 1 <= int(p[k:k+2]) <= 26 else "."
            for p in msg[2:] for k in (0, 2))
        print("  as letters (A=01): " + letters[:40])

main()
