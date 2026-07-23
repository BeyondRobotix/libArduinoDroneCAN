# Host tests for parameter storage

Compiles `src/storage.cpp` on the host and runs its H7 log-structured path
against a RAM model of the flash peripheral. No hardware required.

```powershell
pwsh test/native/run_tests.ps1
```

Requires MSVC (Visual Studio 2022 Build Tools or Community); `run_tests.ps1`
finds it via `vswhere` and sets `INCLUDE`/`LIB` itself.

## Why this exists

The H7 storage path is a two-sector log with compaction and power-loss
recovery. Most of its interesting states — a full log, an interrupted sector
switch, a stale `FULL` sector awaiting erase — take thousands of writes or a
precisely timed power cut to reach on a bench node. They are one line each
here.

## What the model enforces

`flash_model.cpp` reserves the real `0x08000000` address range from the OS,
so `storage.cpp`'s memory-mapped reads work unmodified, and it aborts on any
access that the H7 flash controller would reject:

- writes must be 32-byte aligned and a whole number of 32-byte flash words
- **a flash word may be programmed only once per erase** — this is the ECC
  constraint that rules out in-place state-machine updates on H7, and the
  reason sector states occupy three separate padded slots

An abort here means the code would have faulted or corrupted flash on target.

It also counts erases, which is how `test_write_path_never_erases` asserts
the central property of the design: in steady state, saving a parameter
performs exactly one 32-byte write and zero erases.

## Simulating power loss

`arm_power_loss(n)` fails the *n*th flash word write from that point and
rejects everything afterwards; `reboot()` in the test file then clears both
the simulated brown-out and `storage.cpp`'s cached log position, which on
target is only cleared by a reset. `storage.cpp` exposes
`dronecan_storage_test_reset_cache()` for that, compiled only under
`-DSTORAGE_HOST_TEST`.

## Coverage

The L4/G4/F4 erase-and-rewrite path is not covered here — it is unchanged
apart from the added signature word, and its behaviour is verified by the
HIL suite at the mono-repo root.
