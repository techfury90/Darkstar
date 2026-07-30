# Handoff — Medley/Interlisp installer reaches MP 0990, then posts 0935

Written 2026-07-30. Supersedes nothing; `HANDOFF-0934.md` still holds the launch recipe and
asset paths, and its "Practical" section is still correct.

---

## The prompt

> Doovke (Xerox 6085 / Dove emulator, forked from Darkstar) now boots the Medley/Interlisp
> **installer**: its CP microcode loads, it reaches **MP 0990 = running**, executes ~600 M CP
> instructions of visibly correct work including 68 rigid-disk operations and 669 floppy
> reads, and then posts **MP 0935 = `cCantTeledebug`** and sits in the debugger-substitute
> display loop for ever.
>
> ViewPoint 2.0 boots to its logon sheet on the same build and is the regression baseline.
>
> Find what raises the uncaught error behind that 0935. Read §"Do not re-chase" first — a
> long list is already excluded by measurement — and §"Instrument traps" before writing any
> new instrument, because five of mine produced convincing but wrong output.

---

## What works now

- **Medley microcode loads.** Root cause was port 0x80 **bit 5**: the downloaded `.db` loader
  gates CP WriteData blocks on `AND AX,0060 / CMP AX,0060` — bits 5 **and** 6 — and at 0x0040
  it silently *dumped* every block. Fix `2596a4d`. Signature of success: `byteOUTs=23574`
  (3,929 microinstructions × 6 lanes) over ports `8000..DF58`, bank0 occupancy 3929.
- **ViewPoint 2.0** boots to logon, re-verified after every change tonight. Snapshot
  `vp20-verified-bit5`.
- **Six timing corrections**, all of which were running fast — see
  `memory/doovke-timing-hierarchy.md`. The scheduler tick alone was 9.4× fast *and*
  instruction-paced.
- **8-inch DLion media is refused** with a message naming the machine (`b686804`).
- **UI lock hold is time-bounded** (`c40cf46`) — a long-standing intermittent where the
  window stopped responding whenever the emulator got slower.

## Repro

```powershell
$root = "C:\Users\techf\Desktop\Darkstar\.claude\worktrees\optimistic-easley-b9fe49"
$dk   = "C:\Users\techf\Desktop\Darkstar"
$rom  = "$root\dove_build_kit\firmware\boot_rom\dove_iop_V2_K_merged_load_at_FC000.bin"
$env:DOVE_MP_TRACE = "$root\mp.txt"      # panel, sampled from the cursor sprite
$env:DOVE_CP_STATE = "$root\state.txt"   # everything else, written on close
$fd = "$dk\1186_floppies_imd\010_master\010_001_installation_utility.imd"
$argline = '--rom "' + $rom + '" --eeprom "' + $dk + '\eeprom_lisp.bin" --floppy "' + $fd +
           '" --rigid "' + $dk + '\dove_rigid_lisp.img" --key 64'
Start-Process -FilePath "$root\D\bin\Doovke\Release\Doovke.exe" -ArgumentList $argline -WorkingDirectory $root
```

Enter a date at the Set Time prompt; 0935 follows. **Close with the window control** (or
`CloseMainWindow()`) — `Stop-Process` skips `FormClosing` and loses every dump. **Close Doovke
before building**: it holds the exe and MSBuild fails with `MSB3026`, silently leaving a stale
binary.

Displayed sequence (the sprite is ground truth):
`900 → 920 → 910 → 920 → 910 → 920 → 930 → 940 → 970 → 990` … ~605 M instructions … `935`.
That run-up is **healthy** — it matches ViewPoint's almost exactly.

## Do not re-chase — excluded by measurement

| | finding |
|---|---|
| CP traps | none of any kind. `trapCode=0`, no stack/IB-empty traps; the only `ErrTrap` entry is the boot init trap at CPi 12. If a trap is behind this it is a **Mesa** trap. |
| Mesa-bus I/O | zero unmodelled reads; every read is 0x41/0x42 and handled |
| 8254 programming | both builds program it byte-identically; counter 0 = `0x0C35` = 3125 |
| DDC generation | `ECCC` forced to gate-array, read 42,751 times, same failure at the same GFI/globalLink/PC |
| ST3 Ready | media-gated vs always-asserted: identical 669 reads, same failure |
| floppy path | 669 reads, all "ok", zero abnormal terminations, perfectly sequential C/H/R |
| MDS switch | works — banks change across `aWRMDS` (0x06 → 0x00), read-backs match |
| the report path | GFI 217 —`aPO` alpha 0D→ GFI 705 (`DebuggerSubstituteImpl`, a RESPONDING PORT) → GFI 644 —`@WRMP`→ panel. All healthy. GFI 705's loop is `ShowCodeInMP`'s 400,000-iteration display dwell (935 re-posted every 21.6 M instructions). **The machine is reporting, not hung.** |
| rigid-disk contents | same 0935 on three packs: ViewPoint-installed, unformatted, formatted-empty. (⚠ unformatted is its own known-bad case, so it proves less than it looks.) |
| guest geometry | a Micropolis 1325 is physically 1024 cylinders; Xerox formats 960. Asserted now; `OutOfImageAccesses` measured **0**. Real, not causal. |
| six timing corrections | none moved it — see the timing memory |

## ★Instrument traps — read before writing any instrument

**The machine does not stop at the fault.** It posts 0935 and then runs the display loop for
*billions* of instructions. A ring that keeps its **tail** keeps the report loop; a **first-N
cap** fills long before the fault. Four instruments measured the wrong window that way and a
fifth mis-printed (a summary emitting the *first* 200 runs plus the last, so two non-adjacent
runs read as a call edge and invented a "2184 → 705" that does not exist). **Everything now
honours `DiagFrozen`, set at the fault.**

**The MP code is NOT `_alu.R[0]` at the `@WRMP` dispatch.** Checked against the sprite by CPi,
R0 read 940 as `18984`, 970 as `2`, 990 as `2`, and 935 as `938 then 990` — it manufactured an
**MP 938 that never happened**, and a whole reframe got built on it. The **cursor sprite is
the only ground truth**.

**The ESC alpha from `_ib[_ibPtr & 1]` is wrong ~2 in 10** and can echo the opcode back
(reported `C1` for `0C`, `F8` for `79`). Read the alpha from **memory** at the dispatch PC,
locating the known-correct `_ibFront` value in the two words.

**GFI must be resolved at dispatch**, not at snapshot: `[L-2]` read later attributes a
dispatch to whoever has since been handed that frame.

**Cross-run address substitution does not work** — the macro PC is a physical word address and
demand paging places code differently each run.

## Live leads

1. **`DiskChannelResident:196` — a 1-second naked-notify guard on RDC completions**, where a
   lost completion becomes a permanent hang instead of a retry (operator's §3 audit). We know
   the last RDC operation ends ~6,800 IOP instructions before the post and that we never
   report an error in any of 68 DOBs. A completion the guest never *sees* would look exactly
   like this.
2. **`CourierImplM:126`** spins `UNTIL LowHalf[pulses] # 0` — the only named consumer that
   fails on a *stopped* clock rather than a mis-rated one.
3. **GMT drift**, passive and free: boot ViewPoint, compare the desktop clock against the host
   after a while. Rate error shows up as proportional drift and validates the timing stack.
4. **What are GFI 217, 644, 705** in a Pilot 12 build? No Pilot 12 source, so the archive is
   the only route. This Pilot's `ShowInMP` responder re-shows only the code — it does **not**
   cycle `gfi`/`pc` like `GermWorldError` — so there is no traceback in the MP stream.
5. `DoveDiskController.OpCount` increments on `(value & 3) == 0` at port 0x214, i.e. it
   *assumes* a write of 0 starts a command. The final trace ends on one, so it may be counting
   terminations. Verify before trusting op counts.

## Environment switches added tonight

`DOVE_RETRACE=<cycles>` · `DOVE_PIT_CPINSTR=1` (legacy CP-paced 8254) · `DOVE_PIT_DIV=<n>` ·
`DOVE_CP_STEPS=<n>` · `DOVE_ST3_MEDIA=1` (media-gated ST3 Ready) · `DOVE_UART74_TXREADY=1` ·
`DOVE_INPUT_PORT=<bare hex>` · `DOVE_RDC_LOG=<path>`

See `memory/doovke-medley-0935-open.md` and `memory/doovke-timing-hierarchy.md` for the full
record.
