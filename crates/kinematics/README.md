# kinematics

Motion-state and propagation helpers.

## What This Crate Contains

- `PlanarMotion` and `SpatialMotion`
- `MotionState2` and `MotionState3`
- `FixedWingState`
- `coordinated_turn_rate`

## Intended Use

Use this crate for deterministic state propagation that does not need a probabilistic filter.

Examples:

- simple dead reckoning
- body-rate integration
- fixed-wing turn-rate calculations

## Example

- `examples/motion_models.rs`
