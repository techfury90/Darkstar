# Dove (Xerox 6085) germ-boot trace harness — handoff

Standalone `csc` harness that boots the emulated Daybreak **IOP (80186) + CP
(AM2901 microengine)** far enough to run the ViewPoint germ, with heavy
instrumentation printed at the end of the run. Used to diagnose the MP-0900
`sCodeTrap` wedge (see memory `darkstar-dove-milestone2-cp-bridge`, OQ53–OQ61).

`TraceRom.cs` is **not** part of the product build — it's a throwaway driver
that `#include`s the real emulator sources and dumps trace logs. It is kept here
so it survives across sessions (it was previously only in the ephemeral
scratchpad). Everything is diagnostic-only; the real emulator behavior lives in
`D/CP/*` and `D/IOP/*`.

---

## 0. Prerequisites

- **Compiler:** Roslyn `csc.exe`. On this machine:
  `C:\Program Files\Microsoft Visual Studio\18\Community\MSBuild\Current\Bin\Roslyn\csc.exe`
  (Git-Bash path: `/c/Program\ Files/Microsoft\ Visual\ Studio/18/Community/MSBuild/Current/Bin/Roslyn/csc.exe`).
- **Inputs (durable):**
  - Boot ROM (positional arg): `dove_build_kit/firmware/boot_rom/dove_iop_V2_K_merged_load_at_FC000.bin` (repo-relative).
  - EEPROM config (`DOVE_EEPROM`): `eeprom_floppy.bin` (in this dir).
  - Germ floppy (`DOVE_FLOPPY`): `C:/Users/techf/Desktop/Darkstar/130P26405_6085_Xerox_ViewPoint_Installer_#1.imd` (outside the repo — an IMD ImageDisk of the 6085 ViewPoint installer disk #1).
- Run from the **repo root** (`.../worktrees/<name>/`) so the `D/...` source paths resolve.

---

## 1. Build

Compile the real emulator sources + `TraceRom.cs` into `tracerom.exe`. From the repo root:

```bash
CSC="/c/Program Files/Microsoft Visual Studio/18/Community/MSBuild/Current/Bin/Roslyn/csc.exe"
OUT="dove_build_kit/tools/tracerom"          # or any scratch dir you can write

"$CSC" -nologo -out:"$OUT/tracerom.exe" -target:exe \
  D/IOP/i80186.cs D/IOP/I80186Bus.cs D/IOP/I80186Pcb.cs \
  D/IOP/I8259.cs D/IOP/I8254.cs D/IOP/I8251.cs D/IOP/I93C46.cs D/IOP/I8272.cs \
  D/IOP/DoveControlStore.cs D/IOP/DoveDisplayController.cs D/IOP/DoveIOPMemory.cs D/IOP/DoveIOPIO.cs \
  D/CP/DoveCentralProcessor.cs D/CP/Microinstruction.cs D/CP/AM2901.cs \
  D/IO/FloppyDisk.cs \
  "$OUT/TraceRom.cs"
```

Check for `: error` in the output; a clean build produces `tracerom.exe`.
**If you edit `D/CP/*` or `D/IOP/*`, you must rebuild** — the exe statically
links the sources, so a stale exe silently traces old behavior.

---

## 2. Run

```bash
OUT="dove_build_kit/tools/tracerom"
IMD="C:/Users/techf/Desktop/Darkstar/130P26405_6085_Xerox_ViewPoint_Installer_#1.imd"
BIN="dove_build_kit/firmware/boot_rom/dove_iop_V2_K_merged_load_at_FC000.bin"

DOVE_BUDGET=17000000 DOVE_POKE_AT=12000000 DOVE_POKE_DELAY=0 DOVE_POKE_CODE=64 \
  DOVE_EEPROM="$OUT/eeprom_floppy.bin" DOVE_FLOPPY="$IMD" \
  "$OUT/tracerom.exe" "$BIN" > trace.txt 2>&1
```

A full 17M-budget run takes **~6 minutes**. All output goes to stdout; redirect
to a file and grep it (sections below).

### Environment variables

| Var | Default | Meaning |
|---|---|---|
| `DOVE_BUDGET` | — | Total instruction budget. **Budget is IOP-step-based**: the CP (germ) doesn't start until the boot poke at IOP ~12M, so a budget < ~13M never boots the germ (you'll see MP `0000`/`0199` only). `17000000` reaches the MP-0900 wedge (CP ~10.4M instrs). |
| `DOVE_POKE_AT` | — | IOP instruction count at which the boot poke fires (the "power/boot button"). `12000000`. |
| `DOVE_POKE_DELAY` | — | `0`. |
| `DOVE_POKE_CODE` | — | Poke value. `64`. |
| `DOVE_EEPROM` | — | EEPROM config blob (128 B). Selects floppy boot. |
| `DOVE_FLOPPY` | — | Germ floppy IMD. |
| `DOVE_LOOPTRACE_FROM` | `0` | Start CPi for the microinstruction-level `LoopTrace` (captures the next **520 microwords**). Set to the CPi window you care about. |
| `DOVE_IBLOG_FROM` / `DOVE_IBLOG_TO` | `2226` / `2360` | CPi window for the IB `LoadIB`/`Readib`/`IBDisp` trace. |
| `DOVE_XFER_FROM` | `10000000` | Start CPi for `XferChainLog` (register-writing `MAR<-` microwords). Set to `99999999` to disable it. |

---

## 3. Timeline / CPi milestones (17M budget, current baseline)

The CP instruction counter (`CPi`) starts at 0 when the germ CP begins running
(after the IOP poke). Key points on the current MP-0900 baseline:

| CPi | Event |
|---|---|
| ~1250–1770 | germ executing real page 0x4B1 (mesa Start-family code) |
| ~1470–1543 | **the SD[7] LOOPHOLE desync** (`C1 F8 73` WriteWDC + `CE 39 1B`) — the @LIW word-cross |
| 1576 | germ zero-fills `SD[7]` (`real 0x4820E/F` ← `[0,0]`) |
| 1621 / 1666 | germ installs `SD[6]`=`[0B5D,06F4]` / `SD[3]`=`[0B5D,06FF]` (these work) |
| 1827 | `GetRealPage` aGMF (`17 F8 09`) reads `map[vp 0x100]` |
| **2150** | **first CodeTrap**: reads `SD[7]` (null) → escalates to `SD[6]` sControlTrap → XFER to real 0x499B6 (GermWorldError) |
| ~10.4M | the wedge: GermWorldError handler re-CodeTraps forever |

MP-code progress (IOP hex LED) ends at **`0900`** on the baseline; the fix's
success signal is MP advancing **past 0900** (and the desktop staying drawn).

---

## 4. Reading the output — key sections

Grep section headers with `grep -E '^=== ' trace.txt`. The important ones:

| Grep | What it shows |
|---|---|
| `MP-code history` | IOP MP-code (hex-LED) progression — the single best "how far did it boot" signal. |
| `ONE 0900-LOOP ITERATION` | `LoopTrace` — microinstruction trace (`@addr cN CPi= R5= RH5= pc16= ibPtr= ibF= … <op decode>`). Windowed by `DOVE_LOOPTRACE_FROM`. |
| `IB _ib\[0\]/refill trace` | `IbLog` — `LoadIB`/`<-ib reads`/`IBDisp dispatched`/`IBDisp TRAP` with IB state. Windowed by `DOVE_IBLOG_FROM/TO`. **This is the tool for the @LIW word-cross.** |
| `SD TRAP-TABLE READS` | `SdReadLog` — every `<-MD` of `SD[n]` (real 0x48200+). Shows the CodeTrap(SD7)→ControlTrap(SD6) escalation and the faulting `R5`. |
| `SD-INSTALL WRITES` | every store to the SD table — confirms `SD[7]` never gets `[0B5D,0x391B]`. |
| `MAP-ARRAY READS` | `MapArrRead` — `<-MD` of the map array `0x400C0-0x40101` (GetState inputs). |
| `aGMF / FindStartOfIORegion Map<-` | `MapReadLog` — every aGMF (`@8E5`) with call-site `R5`. |
| `XFER-CHAIN` | `XferChainLog` — register-writing `MAR<-` microwords with Rold/Y/F/splice + candidate write-back models. |
| `GERM MESA OPCODE STREAM` | `OpLog` — dispatched mesa opcodes. |
| `CURSOR-SPRITE MP-code snapshots` | DDC cursor-sprite frames — non-blank frames = germ is drawing (desktop rendered). |

Address decoders you'll need constantly:
- **byte-PC** = `(R5 << 1) | pc16`; real code word = `(RH5 & 0xf) << 16 | R5`.
- **map-word → real page** = `((w & 0x1F) << 8) | (w >> 8)`.
- **CP word W ↔ phys byte** = `phys = 2*W`; **IOP linear L (mapreg8=5)** = `phys 0xA0000 + L` (128 KB pages).

---

## 5. Current diagnosis (where the fix goes)

Root cause is confirmed and banked in memory (`darkstar-dove-milestone2-cp-bridge`,
OQ53–OQ61). Short version:

- The wedge = infinite **CodeTrap → ControlTrap** loop. `sCodeTrap` (`SD[7]`) is
  **null**; it should be `[0B5D, 0x391B]` (installed at germ src `:1099`).
- The install is the LOOPHOLE-build `CE 39 1B`, in the `C1 F8 73` (WriteWDC)
  region at word `0xB1D7–0xB1DA`. It **desyncs across a word boundary** — the
  `zESC` alpha / next-word immediate byte isn't resident.
- **Fix lives in the not-empty refill / `pc16` advance** (the 3-byte-IB invariant
  / `RefillNE` top-to-3 across a word boundary). It is **NOT** the
  `RefillE`/split-2901/address-capture path — edits there collapse the germ to
  MP 0200 (tried and reverted; the capture is retained in the baseline).

Confirming trace for the next pass:
```bash
DOVE_BUDGET=17000000 DOVE_POKE_AT=12000000 DOVE_POKE_DELAY=0 DOVE_POKE_CODE=64 \
  DOVE_IBLOG_FROM=1470 DOVE_IBLOG_TO=1545 DOVE_LOOPTRACE_FROM=1470 DOVE_XFER_FROM=99999999 \
  DOVE_EEPROM="$OUT/eeprom_floppy.bin" DOVE_FLOPPY="$IMD" "$OUT/tracerom.exe" "$BIN" > f8.txt 2>&1
```
then read the `IbLog` + `LoopTrace` around the `F8 73` dispatch (CPi ~1500–1545).
Success = MP advances past `0900` and `SD[7]` gets `[0B5D,0x391B]` in `SD-INSTALL WRITES`.

---

## 6. Regression: the DLion CP unit tests (don't skip after a CP edit)

The CP change must keep the DLion regression green — memory refers to this as
**"14/14 CpTest"**. The unit-test harnesses live alongside this file
(`CpTest.cs`, `ControlStoreTest.cs`, `DisplayTest.cs`, `EepromTest.cs`,
`Test186.cs`). Each is a standalone `csc` main that exercises one subsystem; build
the same way (real sources + the one test `.cs`) and run. `CpTest` is the CP
microengine regression — run it after any `D/CP/*` change and confirm all cases pass
before trusting a germ-boot result.
