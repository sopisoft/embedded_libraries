# tecs

Total Energy Control System helpers for fixed-wing altitude and airspeed hold.

## What This Crate Contains

- `TecsController`
- `TecsConfig`
- `TecsTarget`
- `TecsState`
- `TecsOutput`

## Control Split

This crate follows the standard TECS idea:

- throttle tracks total aircraft energy
- pitch tracks the balance between altitude energy and speed energy

That makes it useful for:

- altitude hold
- airspeed hold
- coordinated tradeoff between climb and speed

## Notes

- TECS is most useful when you have a real or well-estimated airspeed input
- The implementation here is intentionally compact and sensor-agnostic
- It is meant to compose with `stabilization` and `airframe`, not replace them

## Example

- `examples/altitude_hold.rs`
