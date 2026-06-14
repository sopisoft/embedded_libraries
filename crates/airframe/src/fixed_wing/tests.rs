#[cfg(feature = "cascade-pid")]
mod cascade {
    use crate::{Attitude, Vector3};
    use control::{ConventionalTailMixer, ElevonMixer, PidController, VTailMixer};
    use fugit::MicrosDurationU32;
    use pwm::{ServoRange, ServoSet};
    use stabilization::{AxisErrorMode, CascadeAttitudeController, CascadeAxis};

    use super::super::{
        AttitudeHoldLimits, ElevonController, ElevonServoMap, FixedWingController, ServoMap,
        VTailController, VTailServoMap,
    };
    use crate::PilotCommand;

    fn make_axis() -> CascadeAxis {
        let mut attitude = PidController::new(4.0, 0.0, 0.0);
        attitude.set_output_limits(-10.0, 10.0);

        let mut rate = PidController::new(0.5, 0.0, 0.0);
        rate.set_output_limits(-1.0, 1.0);

        CascadeAxis::new(attitude, rate, 3.0)
    }

    fn make_controller<const N: usize>() -> FixedWingController<N> {
        let base = ServoRange::default();
        let servos = ServoSet::new([base; N]);

        let mut roll = CascadeAxis::new(
            PidController::new(4.0, 0.0, 0.0),
            PidController::new(0.5, 0.0, 0.0),
            2.0,
        );
        roll.attitude_pid.set_output_limits(-2.0, 2.0);
        roll.rate_pid.set_output_limits(-1.0, 1.0);

        let mut pitch = CascadeAxis::new(
            PidController::new(4.0, 0.0, 0.0),
            PidController::new(0.5, 0.0, 0.0),
            2.0,
        );
        pitch.attitude_pid.set_output_limits(-2.0, 2.0);
        pitch.rate_pid.set_output_limits(-1.0, 1.0);

        let mut yaw = CascadeAxis::new(
            PidController::new(2.0, 0.0, 0.0),
            PidController::new(0.3, 0.0, 0.0),
            1.0,
        )
        .with_error_mode(AxisErrorMode::WrappedAngle);
        yaw.rate_pid.set_output_limits(-1.0, 1.0);

        FixedWingController::new(
            CascadeAttitudeController::new(roll, pitch, yaw),
            ConventionalTailMixer::new(),
            servos,
            ServoMap::conventional_5ch(),
            AttitudeHoldLimits::default(),
        )
    }

    #[test]
    fn manual_mode_maps_throttle_as_normalized_output() {
        let controller = make_controller::<5>();
        let output = controller.update_manual(PilotCommand {
            roll: 0.0,
            pitch: 0.0,
            yaw: 0.0,
            throttle: 1.0,
            flaps: 0.0,
            attitude_hold_enabled: false,
        });
        assert!(output.pulses[4].as_micros() >= 1_990);
    }

    #[test]
    fn attitude_hold_mode_generates_surface_commands() {
        let mut controller = make_controller::<5>();
        let output = controller.update_attitude_hold(
            PilotCommand {
                roll: 0.5,
                pitch: 0.2,
                yaw: 0.1,
                throttle: 0.5,
                flaps: 0.0,
                attitude_hold_enabled: true,
            },
            Attitude::new(0.0, 0.0, 0.0),
            Vector3::ZERO,
            MicrosDurationU32::from_millis(10),
        );
        assert!(output.surfaces.left_aileron.abs() > 0.0);
        assert!(output.surfaces.elevator.abs() > 0.0);
    }

    #[test]
    fn elevon_controller_generates_differential_surface_commands() {
        let base = ServoRange::default();
        let servos = ServoSet::new([base; 3]);
        let mut controller = ElevonController::new(
            CascadeAttitudeController::new(make_axis(), make_axis(), make_axis()),
            ElevonMixer::new(),
            servos,
            ElevonServoMap::three_channel(),
            AttitudeHoldLimits::default(),
        );
        let output = controller.update_attitude_hold(
            PilotCommand {
                roll: 0.4,
                pitch: 0.1,
                yaw: 0.0,
                throttle: 0.5,
                flaps: 0.0,
                attitude_hold_enabled: true,
            },
            Attitude::new(0.0, 0.0, 0.0),
            Vector3::ZERO,
            MicrosDurationU32::from_millis(10),
        );
        assert_ne!(output.surfaces.left_elevon, output.surfaces.right_elevon);
    }

    #[test]
    fn vtail_controller_generates_tail_split() {
        let base = ServoRange::default();
        let servos = ServoSet::new([base; 4]);
        let mut controller = VTailController::new(
            CascadeAttitudeController::new(make_axis(), make_axis(), make_axis()),
            VTailMixer::new(),
            servos,
            VTailServoMap::four_channel(),
            AttitudeHoldLimits::default(),
        );
        let output = controller.update_attitude_hold(
            PilotCommand {
                roll: 0.2,
                pitch: 0.1,
                yaw: 0.3,
                throttle: 0.5,
                flaps: 0.0,
                attitude_hold_enabled: true,
            },
            Attitude::new(0.0, 0.0, 0.0),
            Vector3::ZERO,
            MicrosDurationU32::from_millis(10),
        );
        assert_ne!(output.surfaces.left_tail, output.surfaces.right_tail);
    }
}

#[cfg(feature = "indi")]
mod indi_backend {
    use crate::{Attitude, Vector3};
    use control::ConventionalTailMixer;
    use fugit::MicrosDurationU32;
    use indi::{IndiAttitudeConfig, IndiAttitudeController, IndiAxisConfig, IndiRateController};
    use pwm::{ServoRange, ServoSet};

    use super::super::{AttitudeHoldLimits, FixedWingController, ServoMap};
    use crate::PilotCommand;

    fn make_indi_controller<const N: usize>() -> FixedWingController<N, IndiAttitudeController> {
        let axis = IndiAxisConfig {
            acceleration_filter_tau_s: 0.0,
            actuator_filter_tau_s: 0.0,
            ..IndiAxisConfig::symmetric(20.0, 6.0, 30.0, 50.0)
        };
        FixedWingController::new(
            IndiAttitudeController::new(
                IndiRateController::from_configs(axis, axis, axis),
                IndiAttitudeConfig::fixed_wing_default(),
            ),
            ConventionalTailMixer::new(),
            ServoSet::new([ServoRange::default(); N]),
            ServoMap::conventional_5ch(),
            AttitudeHoldLimits::default(),
        )
    }

    #[test]
    fn indi_backend_generates_surface_commands() {
        let mut controller = make_indi_controller::<5>();
        let output = controller.update_attitude_hold(
            PilotCommand {
                roll: 0.4,
                pitch: -0.2,
                yaw: 0.1,
                throttle: 0.5,
                flaps: 0.0,
                attitude_hold_enabled: true,
            },
            Attitude::new(0.0, 0.0, 0.0),
            Vector3::ZERO,
            MicrosDurationU32::from_millis(20),
        );
        assert!(output.surfaces.left_aileron.abs() > 0.0);
        assert!(output.surfaces.elevator.abs() > 0.0);
        assert!(output.surfaces.rudder.abs() > 0.0);
    }
}
