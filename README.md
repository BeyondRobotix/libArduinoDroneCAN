# libArduinoDroneCAN

DroneCAN library for Arduino on STM32, bundling everything needed to send and receive DroneCAN messages from an STM32-based Beyond Robotix node (or any STM32 Arduino target).

## Contents

- `src/` — Beyond Robotix DroneCAN wrapper: the `DroneCAN` class, parameter storage, firmware update, and board-specific CAN drivers (L4 bxCAN, H7 FDCAN).
- `dronecan/` — DSDL-generated message types (`uavcan.protocol.*`, `uavcan.equipment.*`, `dronecan.*`). Regenerate via `tools/dronecan/` in the [mono workspace](https://github.com/BeyondRobotix/ArduinoDroneCAN_Mono).
- `libcanard/` — upstream libcanard CAN transport.
- `acanfd-stm32/` — FDCAN driver for H7, vendored from [pierremolinaro/acanfd-stm32](https://github.com/pierremolinaro/acanfd-stm32). Examples and extras dropped; source + license retained.

## Usage

Add to your `platformio.ini`:

```ini
lib_deps = https://github.com/BeyondRobotix/libArduinoDroneCAN.git#v1.0.0
```

Pin to a specific tag for reproducible builds. To track main, drop the `#vX.Y.Z`. After bumping the pin, force a refresh with `pio pkg uninstall --library libArduinoDroneCAN -e <env>` or delete the project's `.pio/libdeps/`.

## Supported targets

- MicroNode (STM32L431, bxCAN) — `-DCANL431`
- CoreNode (STM32H743, FDCAN) — `-DCANH7 -DACANFD`
- MicroNodePlus (STM32H723, FDCAN) — `-DACANFD` plus the H723 variant identifiers

The Beyond Robotix [br-stm32 platform](https://github.com/BeyondRobotix/br_platformio_hwdef) sets these defines automatically per board.

## Consumers

- [Arduino-DroneCAN](https://github.com/BeyondRobotix/Arduino-DroneCAN) — reference application + HIL test suite.
- [BR_bootloader](https://github.com/BeyondRobotix/BR_bootloader) — PlatformIO + Arduino bootloader; uses this library for the DroneCAN firmware-update server.

## Releases

Manual cut-a-tag: edit, commit, `git tag vX.Y.Z`, push tags. Consumers bump their `lib_deps` pin.
