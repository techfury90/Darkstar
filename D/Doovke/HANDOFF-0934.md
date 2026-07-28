# Doovke: MP 0934 after the germ inLoads BWSDove

State as of 2026-07-28. Branch `doovke` on the `fork` remote.

## Where the machine is

The Xerox 6085 emulator boots ViewPoint 2.0 **from its own rigid disk**. It loads the
Utility Pilot boot file, runs the Set Time utility, accepts the date, then inLoads
`BWSDove` (the real ViewPoint boot file) — and Pilot rejects the StartList with MP 0934.

Two root causes were found and fixed getting here, both committed:

* `72debf5` — the IOP firmware issues **one `StartDMA` per 512-byte page**, not one per
  operation. We transferred a single sector per operation and reported the whole run
  complete, so 31 of every 32 pages of every boot file vanished silently. This is what
  made ViewPoint fail to install for weeks.
* `14bd745` — the RDC DMA indexed the DRAM array directly and could not reach the IOP's
  separate 16 KB local SRAM, where the boot ROM keeps its DOB and boot buffer. We read an
  all-zero DOB (decoding as a phantom restore) instead of the ROM's read. This is what
  made the disk unbootable (MP 0149).

## The open problem, precisely

The germ issues 21 transfers to load `BWSDove`. **The final two never reach CP memory.**

* every one of the 426 boot-file pages is read off the platter cleanly — no errors, no
  nil DMA addresses, correct sector counts
* destinations resolve: the CP page map has virtual 3488–3517 mapped contiguously to real
  0x1220–0x1233 (3498 → real 0x122A)
* data from earlier transfers **is** resident and byte-exact (master p200 verified)
* master p405 (the StartList Header, virtual 3498) and p425 are **absent from the entire
  4 MB array**, in either byte order, at any alignment
* so Pilot reads virtual 3498, finds the *previous* boot's StartList still sitting there
  (`lastVMPage = 1499`, the Utility Pilot's), compares it against a 3517-page image, and
  raises 0934

Interval probe across the file: `100=YES 200=YES 250=YES 300=no* 357=YES 366=YES 380=YES
390=YES 405=no 425=no`. (`300` is almost certainly a false negative — the probe needs an
exact 16-byte match and a running Pilot writes to loaded pages.)

## Proven — do not re-check

* **The boot file on disk is byte-identical to the floppy master.** 2800 of 2802 pages;
  the two differences are `Set Boot User`'s own patches to the BootFile header (filePage 1)
  and the StartList header (filePage 406). Master extracted from install disks #2–#6.
* The installed file carries **one extra leading page** (a description string), so
  master page N = disk filePage N+1. The germ accounts for this and reads from filePage 1.
* `countData = 417` matches the master. `lastBootLoadedPage = lastVMPage = 3517`, so there
  is **no demand-paged remainder** — Pilot never reading past filePage 426 is correct.
* `MakeBootable` on the `pilot` path writes **no label chain links**. It patches two header
  *data* pages with `op 3`. `bootCL = 0000/0000` throughout `0x0103` is correct.
* The `0xF`-prefixed DMA targets are the **boot ROM's** reads into its SRAM buffer
  (positions 0–194 of the stream). Correctly routed; not implicated.

## Refuted by measurement — do not re-chase

An off-by-one from the extra leading page · word-vs-byte granularity
(`WordOrPageOpieAddressMask`) · the 887-page shuffle corrupting the map · a zero-count or
differently-computed final run · `transferCleanup` freeing pending ring slots (the ring is
depth 2 and `transferWait` drains both; `xfer[19]` is drained four times before EXIT) ·
boot-file expiry (operator: irrelevant on real hardware) · the label-verify width (widened
to 10 words — still 0 rejections in 41 `op 2` reads, because the channel's null
`bootChainLink` matches the disk's null) · `BadAddressReturnNil` in `%ConvertAddress`
(no zero DMA addresses anywhere) · a stale/absent `Set Boot User` (re-run in the ordering
`TemporaryBooting` specifies; changed nothing).

## The next measurement

**Instrument the write path directly.** Do not reconstruct addresses from the trace
afterwards — that is where four separate wrong conclusions came from today (see below).

In `DoveDiskController.WrMem`, log the target address and first bytes for every sector of
a boot-file read, at the moment it is written. The emulator then reports where it actually
put the data, with no attribution arithmetic. One boot answers whether the final two
transfers write at all, and if so, where.

## Method warnings, earned the hard way

Every wrong conclusion today had the same shape: derive an address or format from memory
or from log-line pairing, get plausible-looking bytes back, and read meaning into them.

1. Read the disk FCB out of the shared DRAM array — it lives in the separate 16 KB local
   SRAM. Returned fill bytes that looked exactly like an uninitialised structure.
2. Read `w2=0000` in a DOB as "1 sector" when it meant "this DOB is entirely empty".
3. A condition-word watch that reported **pre-store** values (`DoveIOPMemory` fires
   `OnSramWrite` before committing the byte), making a working handler look broken.
4. A page-map dump against a base that held no map; then a DMA-address attribution that
   paired each DMA line with the *preceding* op, when sector 0's DMA comes *before* its op
   — every derived address one page off.

**Carry a control that must pass, and build it before the measurement.** Items 3 and 4
were caught only because a control failed loudly; items 1 and 2 were reported upstream as
findings before being retracted.

**Sampling time is part of the measurement.** A dump fired at `OUT 0214h` (go-idle) shows
state *before* the handler's completion path and made a fully working disk handler look
broken in three separate ways. The `PARKED` sampler exists for this reason. Likewise a
frozen `CS:IP` only means something against a moving baseline — Opie's `SystemIdle` is a
busy loop, so an idle machine still advances.

## Also worth logging in the same boot

`%ConvertAddress` (`IOPLMap.asm:165`, the Opie SVC that `DoDiskDMA` calls right before it
programs 0x0208/0x020A) resolves a Mesa virtual page by reading the **CP's page map as
ordinary memory**, through a pointer pair `mesaPageMapSegment:mesaPageMapOffset`:

```
MOV BX, mesaPageMapOffset      ; "stored by IOPInit"
ADD BX, AX                     ; index = page x 2   ("assume no overflow")
MOV AX, mesaPageMapSegment
CMP AX, Null / JE BadAddressReturnNil
MOV DS, AX / MOV BX, [BX]      ; the entry
```

In the B2 ROM the **only** writer of that pair is a conditional-assembly test harness
(`HandInit.asm:90-91`, a 32-word `FakeMesaPageMapSeg`). The real setter lives in the
RAM-resident system downloaded from the `.db` and is not in the source tree we have. So
"the CP map is correct" and "the IOP resolves virtual 3498 correctly" are two different
assertions, and only the second governs the DMA target. Dump those two words at the germ's
first inload transfer:

* still the fake stub, or `Null` -> `ConvertAddress` is reading a 32-word placeholder
* a plausible pointer -> read `[offset + 3488*2]` through it, the way the IOP does, and
  compare against the CP-side map value (virtual 3498 -> real 0x122A)
* correct entry and correct conversion -> the loss is downstream, and the `WrMem` log shows it

Note the 8 IOP map registers (`%EstablishAccess`) are a **different** mechanism — they let
the 80186 dereference Mesa memory via `ES:DI`, which is how `DiskDove.asm` reaches the IOCB
and DOB. They do not touch the DMA target. If they were wrong the DOBs would be corrupt,
and they demonstrably are not.

## Practical

### Paths (all verified present)

```
WORKTREE  C:\Users\techf\Desktop\Darkstar\.claude\worktrees\optimistic-easley-b9fe49
DARKSTAR  C:\Users\techf\Desktop\Darkstar
exe       WORKTREE\D\bin\Doovke\Release\Doovke.exe
ROM       WORKTREE\dove_build_kit\firmware\boot_rom\dove_iop_V2_K_merged_load_at_FC000.bin
EEPROM    WORKTREE\dove_build_kit\tools\tracerom\eeprom_floppy.bin
rigid     DARKSTAR\dove_rigid.img            (+ .labels and .formatted sidecars)
snapshots DARKSTAR\packs\<name>.img          (+ .labels, .formatted)
floppies  DARKSTAR\Viewpoint_2.0_imd\        (83 .imd images)
master    DARKSTAR\reference\BWSDove.boot    1,434,624 bytes = 2802 pages
```

`reference\BWSDove.boot` is the byte-exact `BWSDove` extracted from install disks #2-#6.
It is the source of truth for every claim here about the boot file. Compare it against the
installed file with a **+1 page shift** (the installed file has an extra leading
description page): master page N == disk filePage N+1.

### Snapshots

* **`vp20-installed`** - the good pack. Complete ViewPoint 2.0 install, boot files dense,
  boots to Set Time and then 0934. **Start here.**
* `baseline` - formatted and partitioned, no software. For a fresh install.
* `preSetBoot`, `repro-setbootuser` - intermediate states.
* `vp20-install-failed` - historical, boot files 3% full (pre-DMA-fix). Reference only.

```
powershell -File "WORKTREE\D\Doovke\pack.ps1" restore vp20-installed
powershell -File "WORKTREE\D\Doovke\pack.ps1" save   <name>
powershell -File "WORKTREE\D\Doovke\pack.ps1" list
```

Doovke must be **closed** for either - the script refuses otherwise, because a torn copy
looks exactly like an emulator bug. Closing the window saves the pack (`FormClosing ->
SaveRigidDisk`); there is also a 2-minute autosave and a **File > Save Rigid Disk Now**.

### Launching - boot from the rigid disk (the 0934 repro)

PowerShell. `Start-Process` with **one pre-quoted argument string** - passing an array to
`-ArgumentList` shreds paths containing spaces and commas, which the floppy names do.

```powershell
$root  = "C:\Users\techf\Desktop\Darkstar\.claude\worktrees\optimistic-easley-b9fe49"
$rigid = "C:\Users\techf\Desktop\Darkstar\dove_rigid.img"
Get-Process Doovke -ErrorAction SilentlyContinue | Stop-Process -Force
Start-Sleep -Seconds 3
& "$root\D\Doovke\pack.ps1" restore vp20-installed
if (Test-Path "$root\rdc_trace.txt") { Move-Item -Force "$root\rdc_trace.txt" "$root\rdc_trace_prev.txt" }
$env:DOVE_RDC_LOG = "$root\rdc_trace.txt"
$rom = "$root\dove_build_kit\firmware\boot_rom\dove_iop_V2_K_merged_load_at_FC000.bin"
$ee  = "$root\dove_build_kit\tools\tracerom\eeprom_floppy.bin"
$argline = '--rom "' + $rom + '" --eeprom "' + $ee + '" --rigid "' + $rigid + '"'
Start-Process -FilePath "$root\D\bin\Doovke\Release\Doovke.exe" -ArgumentList $argline -WorkingDirectory $root
```

**No `--floppy`.** The machine boots the rigid disk, shows Set Time, and after you enter a
date it inLoads BWSDove and stops at 0934.

### Launching - with the installer floppy

Add `--floppy` and `--key 64` (0x64 = 100 = **F2**, the boot-floppy key, pressed at boot):

```powershell
$fd = "C:\Users\techf\Desktop\Darkstar\Viewpoint_2.0_imd\130P26405_6085_Xerox_ViewPoint_Installer_#1.imd"
$argline = '--rom "' + $rom + '" --eeprom "' + $ee + '" --floppy "' + $fd + '" --rigid "' + $rigid + '" --key 64'
```

Swap disks from the File menu. The set is 20 disks: microcode on #3, the boot file on
#4-8, ViewPoint content from #9. A `Set Boot User` runs before the disk 9 prompt and again
at the end.

### Driving it

* At Set Time enter a date with year **1995** (known-good). Expiry is not a factor - the
  operator confirms boot-file expiration is irrelevant on real hardware.
* Keys are wire scan codes (`D/Doovke/DoovkeKeyboard.cs`); F1-F10 = 99-108, F2 = floppy.
* The mouse captures on click; menus release it.

### Building

```powershell
& "C:\Program Files\Microsoft Visual Studio\18\Community\MSBuild\Current\Bin\MSBuild.exe" `
  "$root\D\Doovke.csproj" -p:Configuration=Release -v:m -nologo
```

**Grep the output for `error`, not `error CS`.** If Doovke is running it holds
`bin\Doovke\Release\Doovke.exe` and the copy fails with `MSB3027` - the compile succeeds,
`bin` keeps the old binary, and you silently test a stale build. Stop Doovke first and
check the exe timestamp afterwards.

### Tracing

* `DOVE_RDC_LOG=<path>` - RDC trace. **Appends** across power cycles by design, so rotate
  it between runs or you will analyse two sessions as one.
* `DOVE_RDC_PORTLOG=<n>` - port-level trace of 0x0200-0x0216, default 4000 entries.
* Line kinds: `DOB op=` (one per executed DOB), `DOB:` (all 34 words), `DMA words=`,
  `XFER DONE` (a completed multi-sector stream), `PORT R/W`, `FCB (...)`, `PROBE`, `CPMAP`.
* A new session is identifiable by the `@IOP<clock>` counter resetting.
