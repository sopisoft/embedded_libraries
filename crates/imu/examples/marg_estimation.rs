use fugit::MicrosDurationU32;
use imu::{AccelGyroSample, MargEstimator, MargSample, Vector3};

fn main() {
    // This example demonstrates the intended layering:
    // 1. your sensor drivers produce one `MargSample`,
    // 2. `imu::MargEstimator` fuses attitude and relative altitude,
    // 3. higher-level control code consumes the result.

    let mut estimator = MargEstimator::new(0.08);
    let dt = MicrosDurationU32::from_millis(10);

    // Pretend the board is sitting flat on the desk:
    // - accelerometer sees +1 g on body Z,
    // - gyroscope sees no rotation,
    // - magnetometer points roughly along body X.
    let sample = MargSample::new(
        AccelGyroSample::without_temperature(Vector3::new(0.0, 0.0, 9.80665), Vector3::ZERO),
        Vector3::X,
    );

    let estimate = estimator.update_marg(sample, dt);
    println!(
        "Euler [deg]: roll={:.2}, pitch={:.2}, yaw={:.2}",
        estimate.euler.x.to_degrees(),
        estimate.euler.y.to_degrees(),
        estimate.euler.z.to_degrees()
    );
    println!(
        "Relative altitude={:.4} m, vertical speed={:.4} m/s",
        estimate.relative_altitude_m, estimate.vertical_speed_m_s
    );

    // Replace the synthetic sample with a real sensor sample in your firmware
    // loop and keep the estimator update call unchanged.
}
