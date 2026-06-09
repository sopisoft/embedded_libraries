# Navigation Examples

- `inertial_navigation`: the smallest useful IMU prediction + correction workflow
- `fixed_wing_dead_reckoning`: aircraft-oriented propagation from attitude, airspeed, and wind

Read `inertial_navigation` first if you want to understand the state layout.
Read `fixed_wing_dead_reckoning` first if your application is an RC aircraft.
When you are ready to wire a real 9-DoF board, move to
`imu/examples/rp235x_stemma_qt_9dof.rs`, which combines sensor drivers,
attitude fusion, and relative altitude estimation on RP2350 hardware.
