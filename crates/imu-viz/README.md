# imu-viz

Desktop viewer for the USB CDC telemetry from `rp235x_stemma_qt_9dof_lps25hb`. It displays orientation, attitude history, relative altitude, and recent logs.

```bash
cargo rp2350 --example rp235x_stemma_qt_9dof_lps25hb
cargo run -p imu-viz -- --port /dev/ttyACM0
```

Expected input:

```text
state t_ms=<ms> quat=(<w>, <x>, <y>, <z>) velocity_m_s=(<vx>, <vy>, <vz>) altitude_m=<m> mag_enabled=<bool> mag_ready=<bool>
```

Plots can follow incoming data or be panned and zoomed. Altitude remains relative and drifts without an external reference.
