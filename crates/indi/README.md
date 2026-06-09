# indi

`no_std` Incremental Nonlinear Dynamic Inversion controllers for embedded aircraft and other small vehicles.

## What This Crate Contains

- `IndiAxis` for one control axis
- `IndiRateController` for roll, pitch, and yaw
- `IndiAttitudeController` for attitude hold over the rate loop
- `ControlEffectiveness` and `IndiAllocator` for 3xN actuator allocation
- low-pass filtering for measured angular acceleration
- optional actuator feedback as the INDI actuator reference
- rate-error to desired-angular-acceleration conversion
- incremental actuator updates with saturation and slew limiting
- optional angular-acceleration estimation from successive gyro rates

## Scope

The simple path is diagonal roll/pitch/yaw control through `IndiRateController`.
For coupled surfaces or over-actuated layouts, use `IndiAllocator` with a 3xN
effectiveness matrix.

## Typical Flow

1. identify or estimate `control_effectiveness` for each axis
2. create one `IndiAxis` per controlled axis
3. feed desired and measured body rates every control step
4. send the returned actuator command to your mixer or servo output layer

## Examples

- `examples/rate_control.rs`
- `examples/control_allocation.rs`
