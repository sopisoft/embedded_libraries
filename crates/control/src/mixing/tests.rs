use super::super::{ControlAxes, ConventionalTailMixer, ElevonMixer, VTailMixer};

#[test]
fn conventional_mixer_splits_ailerons() {
    let mixer = ConventionalTailMixer::new();
    let outputs = mixer.mix(ControlAxes::new(0.5, 0.0, 0.0, 0.0, 0.0));
    assert!(outputs.left_aileron > 0.0);
    assert!(outputs.right_aileron < 0.0);
}

#[test]
fn elevon_mixer_combines_pitch_and_roll() {
    let mixer = ElevonMixer::new();
    let outputs = mixer.mix(ControlAxes::new(0.3, 0.4, 0.0, 0.0, 0.0));
    assert!(outputs.left_elevon > outputs.right_elevon);
}

#[test]
fn vtail_mixer_combines_pitch_and_yaw() {
    let mixer = VTailMixer::new();
    let outputs = mixer.mix(ControlAxes::new(0.0, 0.4, 0.2, 0.0, 0.0));
    assert!(outputs.left_tail > outputs.right_tail);
}
