# Host tests for parameter storage

Compiles `src/storage.cpp` on the host and runs it against a RAM model of the
flash peripheral. No hardware required.

The suite is built and run **twice**, once per flash geometry, because the
algorithm depends on properties that differ between the families:

| | page / sector | record | records per page |
|---|---|---|---|
| H7 (H743, H723) | 128 KB | 32 B | 4093 |
| L4 (L431) | 2 KB | 8 B | 250 |

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
access the flash controller would reject:

- writes must be aligned to, and a whole number of, the family's write unit
- **a flash word may be programmed only once per erase** — on H7 this is the
  ECC constraint that rules out in-place state-machine updates, and the reason
  sector states occupy three separate padded slots. It is modelled on L4 too,
  which is stricter than that hardware requires but keeps one shared rule

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

Both families run the same ten scenarios. What is *not* covered here is the
integration with the DroneCAN layer — parameter persistence over the bus,
the coalescing window, and flush-on-restart are covered by the HIL suite at
the mono-repo root (`tests/test_node.py`).
