# imu-viz

`imu-viz` is a small desktop viewer for the human-readable `defmt` logs emitted by
`crates/imu/examples/rp235x_stemma_qt_9dof_lps25hb.rs`.

It launches the target command, parses `state ...` lines from stdout/stderr, and
shows:

- a real-time 3D orientation view
- roll / pitch / yaw history in degrees
- relative altitude history
- recent raw logs

## Run

Without arguments, `imu-viz` starts the default fusion preset
(`rp235x_stemma_qt_9dof_lps25hb`):

```bash
cargo run -p imu-viz
```

The default spawned command is:

```bash
cargo run -p imu --example rp235x_stemma_qt_9dof_lps25hb --target thumbv8m.main-none-eabihf
```

To launch the IMU-only preset instead:

```bash
cargo run -p imu-viz -- --mode imu
```

You can also provide another command to visualize:

```bash
cargo run -p imu-viz -- cargo run -p imu --example rp235x_stemma_qt_9dof_lps25hb --target thumbv8m.main-none-eabihf
```

## Expected Log Format

The viewer looks for lines in this format:

```text
state t_ms=<elapsed_ms> quat=(<quat_w>, <quat_x>, <quat_y>, <quat_z>) euler_deg=(<roll>, <pitch>, <yaw>) velocity_m_s=(<vx>, <vy>, <vz>) altitude_m=<altitude>
```

Example:

```text
state t_ms=1200 quat=(0.9931, 0.0102, -0.0016, 0.1164) euler_deg=(1.1, -0.2, 13.4) velocity_m_s=(0.02, -0.01, 0.00) altitude_m=0.15
```

## Controls

- `follow attitude plot`: auto-follow the roll / pitch / yaw graph
- `follow altitude plot`: auto-follow the altitude graph
If auto-follow is disabled, you can pan and zoom the plots manually. Parsed `state ...` lines are summarized again in the log panel.

## Notes

- Altitude is still relative. Without a barometer or other external reference,
  long-term absolute altitude will drift.
- The orientation view assumes `x` points into depth and `z` points up in the initial camera view.
