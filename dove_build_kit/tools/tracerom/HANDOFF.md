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

## 0a. READ THE SOURCES — they are already in this kit

**The de-blobbed Daybreak microcode is in `dove_build_kit/daybreak_ucode/uc/` (55 `.mc` files).**
Read it. `BBInit.mc`, `BandBLT.mc`, `Floyd.mc`, `TextBlt.mc`, `LoadStore.mc`, `Misc.mc` answer
opcode / dispatch / slot questions in minutes.

Any older instruction to "route source questions through the operator" is **withdrawn, and it was
actively harmful**: it put a human round-trip between a lying probe and the file that disproved it.
That is how the phantom "the germ reaches BitBlt at depth 1" survived a whole session and became the
premise of a handoff. `LoadStore.mc:404` says `0x76` is `@SGDB`, not a BLT — one `grep` would have
killed it on day one.

**The one genuine gap is the defs layer**: no `Set[hbs.N]` / `SetLabel[...]` exists anywhere in the
kit. That is the only thing worth asking the operator for (`Extensions.dfn`, `BandBlt.dfn`,
`TmMacroTablesDaybreak`, under `DaybreakMicrocode/Private/`). Known so far, from the operator's Duke
(15.2) `Extensions.dfn`: `SetLabel[HowBigStack, 0030]` — table base `0x0030`, corroborated by the
trace (`0x0030 | (~2 & 0xF) = 0x003D` = the measured `bbNormEntry`). Don't adopt Duke's slot names.

Still the museum's lane: the Xerox **Mesa/Pilot** trees and anything not already de-blobbed here.

## 0. Prerequisites

- **Compiler:** Roslyn `csc.exe`. On this machine:
  `C:\Program Files\Microsoft Visual Studio\18\Community\MSBuild\Current\Bin\Roslyn\csc.exe`
  (Git-Bash path: `/c/Program\ Files/Microsoft\ Visual\ Studio/18/Community/MSBuild/Current/Bin/Roslyn/csc.exe`).
- **Inputs (durable):**
  - Boot ROM (positional arg): `dove_build_kit/firmware/boot_rom/dove_iop_V2_K_merged_load_at_FC000.bin` (repo-relative).
  - EEPROM config (`DOVE_EEPROM`): `eeprom_floppy.bin` (in this dir). **Not in git** — it is a 3-byte
    edit of the museum U128 dump (and still carries that machine's serial), so it is an EEPROM dump
    and `.gitignore` deliberately excludes it. Regenerate it from the kit:
    `python dove_build_kit/tools/tracerom/make_eeprom_floppy.py`
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

**★ 2026-07-17. Everything the old §5 said here (SD[7]-null / `C1 F8 73` word-cross / "fix lives in
the not-empty refill") is REFUTED — do not chase it.** So is the BitBlt/`HowBigStack`/stack-depth
thread: measured, the germ's `aBITBLT` dispatch is CORRECT (`sp=2` → `~2=0x0D` → microstore `0x03D`
= `hbs.2` = `bbNormEntry`, and `bbGetArg` reads the bbTable with `UWidth=0x10`). See memory
`dove-bitblt-howbigstack`.

**THE LIVE ROOT LEAD: `Map<-` is executing in c2, and ~all of it is being dropped.**

```
LoadMap microwords by executed cycle:   c1=324   c2=992007   c3=0
```

- `Map<-` is **c1-only** in the microcode: 19 of 19 `Map <- ...` in `daybreak_ucode/uc/*.mc` are
  annotated `,c1`; **zero** at `,c2`/`,c3`. So the emulator's `case 1:` gate is right, and every
  c2 hit is a map reference that **never happens**.
- The **DLion reference asserts this is impossible** — `D/CP/CentralProcessor.cs` case 2 and case 3
  each `throw new InvalidOperationException("Map<- in c2"/"c3")`. **The Dove port dropped that
  assertion.** Worse, `Microinstruction.cs:286` has `MarMapMDR = mem || LoadMap`, so a `mem=0`
  `Map<-` word still enters the memory block, and `case 2:` does an **unconditional `MDR<-`** — it
  never checks `mi.mem`. Each dropped `Map<-` therefore becomes a **wild store to a stale MAR**
  (~992k of them; the spin repeatedly writes `0x020C` to real word `0x00005`).
- **THE FIRST OFFENCE (this is where to start):**
  ```
  *** Map<- IN c2 @A4E CPi=15340  mem=0 rB=5 RH5=84 Y=B1FF
      -> would MDR<- stale MAR 4B100 = B1FF   pCall/Ret2 Map<- RH5,,push Q<- R5
  ```
  That is **inside `Start`'s `zRET` window**: zRET at CPi 15,325 → `sControlTrap` (SD[06] read) at
  15,383. The failing microword maps the **code pointer** (`RH5,,R5`) for the XFER. Its translation
  is dropped ⟹ the transfer reads a map entry that was never loaded ⟹ ControlTrap ⟹
  `GermWorldError` ⟹ the 247,891-iteration map set-ref spin (`@49C`/`@062`/`@140`) that eats 99.9%
  of the run. **One chain, not two wounds.**

**NEXT (in order):**
1. Why is the click phase +1 at CPi 15340? The detector only sees `Map<-` words, so the phase may
   break slightly earlier — walk back from `@A4E` (the `pCall/Ret2` return-cycle bookkeeping around
   `zRET`/XFER is the prime suspect).
2. Independently: `case 2:` must not write memory when `mem=0`. That is wrong regardless of phase
   and is what converts a phase error into a million wild stores. (Fix it *after* 1, so the
   first-offence signal stays loud.)
3. Do **not** "fix" this by honouring `LoadMap` in c2 — that is the "make the dispatch land where I
   want" mistake that produced `c2c301c`. The microcode is unanimous that `Map<-` is c1.

Repro:
```bash
DOVE_BUDGET=17000000 DOVE_POKE_AT=12000000 DOVE_POKE_DELAY=0 DOVE_POKE_CODE=64 \
  DOVE_XFER_FROM=99999999 DOVE_EEPROM="$OUT/eeprom_floppy.bin" DOVE_FLOPPY="$IMD" \
  "$OUT/tracerom.exe" "$BIN" > t.txt 2>&1
grep -A9 "FIRST Map<- OUTSIDE c1" t.txt      # the first offence + the c1/c2/c3 tally
```
Also useful: `DOVE_LOOP_ADDR=C61 DOVE_LOOP_FROM=20000` (one spin iteration, microword-level),
`DOVE_HIST_FROM=20000` (microword histogram), `DOVE_SPINMAP_FROM=<CPi>` (every honoured `Map<-`
with `MAPA`, the resolved entry, and `Q`). NB `MAPA<-` fires exactly **once** per boot (`MAPA<-4`
⇒ base `0x40000`, `InitDaybreak.mc:154`) and the base is **correct** — that branch is ruled out.
A map word of `0x0000` is **never-written**; vacant would be `0x60`.

---

## 6. Regression: the DLion CP unit tests (don't skip after a CP edit)

The CP change must keep the DLion regression green — memory refers to this as
**"14/14 CpTest"**. The unit-test harnesses live alongside this file
(`CpTest.cs`, `ControlStoreTest.cs`, `DisplayTest.cs`, `EepromTest.cs`,
`Test186.cs`). Each is a standalone `csc` main that exercises one subsystem; build
the same way (real sources + the one test `.cs`) and run. `CpTest` is the CP
microengine regression — run it after any `D/CP/*` change and confirm all cases pass
before trusting a germ-boot result.
