# stabilization

Cascaded attitude and rate control helpers.

## What This Crate Contains

- `CascadeAxis`
- `CascadeAttitudeController`
- `CascadeOutputs`
- `AxisErrorMode`

## Architecture

The crate follows the common flight-control structure:

- outer loop controls attitude
- inner loop controls angular rate

This is the usual building block for fixed-wing attitude hold and many multirotor controllers.

## Example

- `examples/fixed_wing_attitude_hold.rs`
