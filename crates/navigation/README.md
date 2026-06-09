# navigation

Lightweight navigation and dead-reckoning layers.

## What This Crate Contains

- `InertialNavigator`
- `FixedWingNavigator`

## Intended Use

Use this crate for compact navigation logic when you do not need a full Kalman filter.

Typical inputs:

- body-frame acceleration
- attitude estimate
- airspeed or groundspeed

## Examples

- `examples/inertial_navigation.rs`
- `examples/fixed_wing_dead_reckoning.rs`
