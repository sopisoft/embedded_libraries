use super::super::{
    ControlAxes, ConventionalTailMixer, ElevonMixer, Normalized, SignedNormalized, VTailMixer,
};

fn axes(roll: f32, pitch: f32, yaw: f32, throttle: f32, flaps: f32) -> ControlAxes {
    ControlAxes::new(
        SignedNormalized::saturated(roll),
        SignedNormalized::saturated(pitch),
        SignedNormalized::saturated(yaw),
        Normalized::saturated(throttle),
        Normalized::saturated(flaps),
    )
}

#[test]
fn normalized_commands_reject_invalid_values() {
    assert!(SignedNormalized::new(1.1).is_none());
    assert!(Normalized::new(-0.1).is_none());
    assert_eq!(
        SignedNormalized::saturated(f32::NAN),
        SignedNormalized::ZERO
    );
    assert_eq!(Normalized::saturated(2.0).get(), 1.0);
}

#[test]
fn conventional_mixer_splits_ailerons() {
    let mixer = ConventionalTailMixer::new();
    let outputs = mixer.mix(axes(0.5, 0.0, 0.0, 0.0, 0.0));
    assert!(outputs.left_aileron.get() > 0.0);
    assert!(outputs.right_aileron.get() < 0.0);
}

#[test]
fn elevon_mixer_combines_pitch_and_roll() {
    let mixer = ElevonMixer::new();
    let outputs = mixer.mix(axes(0.3, 0.4, 0.0, 0.0, 0.0));
    assert!(outputs.left_elevon.get() > outputs.right_elevon.get());
}

#[test]
fn vtail_mixer_combines_pitch_and_yaw() {
    let mixer = VTailMixer::new();
    let outputs = mixer.mix(axes(0.0, 0.4, 0.2, 0.0, 0.0));
    assert!(outputs.left_tail.get() > outputs.right_tail.get());
}
