# Embedded Libraries

`no_std` Rust crates for embedded sensing, estimation, RC links, and fixed-wing control. `imu-viz` is the host-side visualization tool.

| Area | Crates |
| --- | --- |
| Sensors and I/O | `elrs`, `gs1502`, `lis3mdl`, `lps25hb`, `mcp3208`, `mis2500`, `pwm`, `tsd10` |
| Estimation and navigation | `ahrs`, `eskf`, `imu`, `kinematics`, `navigation` |
| Control | `airframe`, `control`, `fc`, `indi`, `stabilization`, `tecs` |
| Tools and firmware | `imu-viz`, `rp2350-examples` |

Each crate README documents its public API and hardware constraints.

```bash
cargo fmt-check
cargo lint
cargo test --workspace
cargo rp2350-check
cargo rp2350-clippy
```

Flash an RP2350 example through the configured `probe-rs` runner:

```bash
cargo rp2350 --example rp2350
```

書き込み時に `Target device did not respond` になる場合は、Pico 2へ3V3/GND/SWDIO/SWCLKを接続し、デバッグプローブが対象基板へ接続されていることを確認してください。
