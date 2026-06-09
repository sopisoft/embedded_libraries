use control::PidController;
use fugit::MicrosDurationU32;
use tecs::{TecsConfig, TecsController, TecsState, TecsTarget};

fn main() {
    // This example shows the smallest useful TECS setup for a fixed-wing aircraft.
    //
    // TECS splits the job into two coordinated commands:
    // - throttle controls total energy,
    // - pitch controls the balance between altitude energy and speed energy.
    //
    // In practice:
    // - if the aircraft is low, TECS tends to add throttle and pitch up,
    // - if the aircraft is slow, TECS tends to add throttle and lower the nose.

    let mut total_energy_pid = PidController::new(0.004, 0.001, 0.0);
    total_energy_pid.set_output_limits(-0.35, 0.45);
    total_energy_pid.set_integral_limits(-40.0, 40.0);

    let mut balance_pid = PidController::new(0.003, 0.0005, 0.0);
    balance_pid.set_output_limits(-0.20, 0.20);
    balance_pid.set_integral_limits(-40.0, 40.0);

    let config = TecsConfig::new(
        0.0,
        0.45,
        1.0,
        -20.0_f32.to_radians(),
        0.0,
        20.0_f32.to_radians(),
        1.0,
    );
    let mut tecs = TecsController::new(total_energy_pid, balance_pid, config);

    // Command a modest climb while also asking for a slightly faster cruise speed.
    let target = TecsTarget::new(125.0, 18.0);

    // Measured state from your estimator and airspeed source.
    let state = TecsState::new(100.0, 16.5);

    let output = tecs.update(target, state, MicrosDurationU32::from_millis(20));

    println!("Throttle command: {:.3}", output.throttle);
    println!("Pitch command [deg]: {:.2}", output.pitch_rad.to_degrees());
    println!(
        "Energy errors: potential={:.2} kinetic={:.2} total={:.2}",
        output.potential_energy_error, output.kinetic_energy_error, output.total_energy_error
    );
}
