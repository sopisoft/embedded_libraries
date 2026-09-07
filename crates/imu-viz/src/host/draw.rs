use eframe::egui;
use eframe::egui::{Color32, Pos2, Stroke, Vec2};
use glam::{Mat3, Vec3};

pub(crate) fn draw_reference_axes(painter: &egui::Painter, center: Pos2, scale: f32, view: Mat3) {
    for axis in [Vec3::X, Vec3::Y, Vec3::Z] {
        let tip = project(center, view, axis * scale * 0.9);
        painter.line_segment([center, tip], Stroke::new(1.0_f32, Color32::from_gray(90)));
    }
}

pub(crate) fn draw_body_axis(
    painter: &egui::Painter,
    center: Pos2,
    scale: f32,
    axis: Vec3,
    color: Color32,
    label: &str,
) {
    let tip = project(center, Mat3::IDENTITY, axis * scale);
    painter.line_segment([center, tip], Stroke::new(3.0_f32, color));
    painter.circle_filled(tip, 4.0, color);
    painter.text(
        tip + Vec2::new(6.0, -6.0),
        egui::Align2::LEFT_BOTTOM,
        label,
        egui::FontId::proportional(14.0),
        color,
    );
}

pub(crate) fn draw_body_cube(painter: &egui::Painter, center: Pos2, scale: f32, basis: Mat3) {
    let half = scale * 0.45;
    let vertices = [
        Vec3::new(-half, -half, -half),
        Vec3::new(half, -half, -half),
        Vec3::new(half, half, -half),
        Vec3::new(-half, half, -half),
        Vec3::new(-half, -half, half),
        Vec3::new(half, -half, half),
        Vec3::new(half, half, half),
        Vec3::new(-half, half, half),
    ];
    let edges = [
        (0, 1),
        (1, 2),
        (2, 3),
        (3, 0),
        (4, 5),
        (5, 6),
        (6, 7),
        (7, 4),
        (0, 4),
        (1, 5),
        (2, 6),
        (3, 7),
    ];
    let projected: Vec<_> = vertices
        .into_iter()
        .map(|vertex| project(center, Mat3::IDENTITY, basis * vertex))
        .collect();

    for (start, end) in edges {
        painter.line_segment(
            [projected[start], projected[end]],
            Stroke::new(2.0_f32, Color32::from_gray(170)),
        );
    }
}

pub(crate) fn project(center: Pos2, view_rotation: Mat3, point: Vec3) -> Pos2 {
    let camera = view_rotation * point;
    let perspective = (1.0 / (1.0 + camera.z * 0.003)).clamp(0.65, 1.35);
    Pos2::new(
        center.x + camera.x * perspective,
        center.y - camera.y * perspective,
    )
}
