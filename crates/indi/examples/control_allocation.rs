use fugit::MicrosDurationU32;
use glam::Vec3;
use indi::{ControlEffectiveness, IndiAllocator, IndiAllocatorConfig};
use nalgebra::SMatrix;

fn main() {
    // This example allocates a desired angular acceleration across four
    // normalized actuators:
    //
    // 0: left aileron
    // 1: right aileron
    // 2: elevator
    // 3: rudder
    //
    // Each column is the angular acceleration produced by +1.0 actuator
    // command: [roll, pitch, yaw].
    let effectiveness = ControlEffectiveness::new(
        SMatrix::<f32, 3, 4>::from_row_slice(&[
            18.0, -18.0, 0.0, 0.0, 0.0, 0.0, 15.0, 0.0, 1.0, -1.0, 0.0, 10.0,
        ]),
        0.2,
    );

    let mut allocator = IndiAllocator::new(IndiAllocatorConfig::normalized(effectiveness, 20.0));

    let dt = MicrosDurationU32::from_millis(20);
    let desired_angular_accel = Vec3::new(5.0, -2.0, 1.0);
    let measured_angular_accel = Vec3::new(1.0, 0.0, 0.2);

    let output = allocator
        .update(desired_angular_accel, measured_angular_accel, dt)
        .unwrap();

    println!("actuator delta: {:?}", output.actuator_delta);
    println!("actuator cmd  : {:?}", output.actuator);
}
