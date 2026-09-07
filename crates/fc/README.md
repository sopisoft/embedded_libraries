# fc

Board-independent composition for the conventional fixed-wing controller: CRSF input, failsafe, attitude estimation, mode selection, and actuator commands.

`Config` owns the RC mapping, estimator, controller, mixer, servo ranges, output mapping, failsafe timeout, and GS-1502 switch settings. Output channels are named by `OutputChannel`; the board layer writes the returned pulse widths to hardware.

The controller starts in failsafe and returns to neutral surfaces, minimum throttle, and `Gs1502Position::Retracted` when RC input is stale. By default, CH6 selects `Retracted` below 1,600 µs and `Extended` at or above it.

The complete Pico 2 firmware and wiring are in [`rp2350-examples`](../rp2350-examples/README.md).

```bash
cargo test -p fc --lib
```
