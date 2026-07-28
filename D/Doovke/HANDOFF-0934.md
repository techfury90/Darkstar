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

## Practical

* Snapshots: `pack.ps1 save <name>` / `restore <name>` (Doovke must be closed).
  `vp20-installed` is the good pack; also `baseline` (partitioned, empty),
  `preSetBoot`, `vp20-install-failed` (historical, boot files 3% full).
* Trace: `DOVE_RDC_LOG=<path>`; appends. Port-level tracing is bounded by
  `DOVE_RDC_PORTLOG` (default 4000).
* Launch: EEPROM **must** be `dove_build_kit/tools/tracerom/eeprom_floppy.bin`; ROM is
  `firmware/boot_rom/dove_iop_V2_K_merged_load_at_FC000.bin`; `--key 64` is F2 (boot
  floppy). Use `Start-Process` with a single pre-quoted argument string — an array
  `-ArgumentList` shreds the floppy paths, which contain spaces and commas.
* Set the clock to **1995** at the Set Time prompt (known-good; today's date also works —
  expiry is not a factor).
* Build errors: grep for `error`, not `error CS` — the `MSB3027` "file locked by Doovke"
  copy failure hides otherwise and you silently run a stale binary.
