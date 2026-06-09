# control

Control primitives for embedded vehicles.

## What This Crate Contains

- RC input shaping:
  `apply_deadband`, `apply_expo`, `apply_dual_rate`, `shape_rc_command`
- Time-based PID through `PidController`
- Fixed-wing surface mixers:
  `ConventionalTailMixer`, `ElevonMixer`, `VTailMixer`

## Typical Use

Use this crate when you need the low-level control blocks but do not yet want a full aircraft pipeline.

Examples:

- shape stick input before control
- run a rate or attitude PID
- mix roll / pitch / yaw / throttle into surfaces

## Example

- `examples/fixed_wing_actuation.rs`

This example walks through input shaping, PID, and output mixing with beginner-oriented comments.
