use eframe::egui;
use eframe::egui::{Color32, Sense, Stroke, Vec2};
use glam::{Mat3, Vec3};

use super::app::ImuVizApp;
use super::draw::{draw_body_axis, draw_body_cube, draw_reference_axes};

impl ImuVizApp {
    fn orientation_view_rotation() -> Mat3 {
        Mat3::from_cols(
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
        )
    }

    fn draw_orientation_view(&self, ui: &mut egui::Ui) {
        let desired_size = Vec2::new(ui.available_width(), 360.0);
        let (rect, _) = ui.allocate_exact_size(desired_size, Sense::hover());
        let painter = ui.painter_at(rect);

        painter.rect_filled(rect, 8.0, Color32::from_rgb(14, 18, 24));
        painter.rect_stroke(
            rect,
            8.0,
            Stroke::new(1.0_f32, Color32::from_gray(70)),
            egui::StrokeKind::Inside,
        );

        let center = rect.center();
        let scale = rect.width().min(rect.height()) * 0.22;
        let basis = self.current_basis();
        let orientation_view = Self::orientation_view_rotation();

        draw_reference_axes(&painter, center, scale, orientation_view);
        draw_body_cube(&painter, center, scale, orientation_view * basis);
        draw_body_axis(
            &painter,
            center,
            scale,
            orientation_view * (basis * Vec3::X),
            Color32::from_rgb(255, 90, 90),
            "X",
        );
        draw_body_axis(
            &painter,
            center,
            scale,
            orientation_view * (basis * Vec3::Y),
            Color32::from_rgb(80, 220, 120),
            "Y",
        );
        draw_body_axis(
            &painter,
            center,
            scale,
            orientation_view * (basis * Vec3::Z),
            Color32::from_rgb(90, 160, 255),
            "Z",
        );

        painter.text(
            rect.left_top() + Vec2::new(12.0, 12.0),
            egui::Align2::LEFT_TOP,
            "3D Orientation",
            egui::FontId::proportional(18.0),
            Color32::WHITE,
        );
        painter.text(
            rect.left_bottom() + Vec2::new(12.0, -12.0),
            egui::Align2::LEFT_BOTTOM,
            "X: depth  Y: horizontal  Z: up",
            egui::FontId::proportional(13.0),
            Color32::from_gray(190),
        );
    }

    pub(crate) fn draw_top_panel(&mut self, ui: &mut egui::Ui) {
        ui.heading("IMU Real-Time Visualizer");
        ui.label(format!("port: {}", self.port.path));

        if let Some(sample) = self.current() {
            ui.horizontal_wrapped(|ui| {
                ui.monospace(format!(
                    "elapsed: {:.1}s",
                    sample.elapsed_ms as f32 / 1000.0
                ));
                ui.separator();
                ui.monospace(format!("roll: {:.2} deg", sample.roll_deg));
                ui.separator();
                ui.monospace(format!("pitch: {:.2} deg", sample.pitch_deg));
                ui.separator();
                ui.monospace(format!("yaw: {:.2} deg", sample.yaw_deg));
                ui.separator();
                ui.monospace(format!(
                    "velocity: ({:.2}, {:.2}, {:.2}) m/s",
                    sample.velocity_world_m_s.x,
                    sample.velocity_world_m_s.y,
                    sample.velocity_world_m_s.z
                ));
                ui.separator();
                ui.monospace(format!("altitude: {:.3} m", sample.altitude_m));
            });
        } else {
            ui.label("waiting for sensor samples...");
        }

        ui.horizontal_wrapped(|ui| {
            ui.label(if self.connected {
                "state: connected"
            } else {
                "state: disconnected"
            });
            ui.separator();
            ui.label(format!("samples: {}", self.samples.len()));
        });

        ui.horizontal_wrapped(|ui| {
            ui.monospace(format!("yaw: {}", self.status.yaw.label()));
            ui.separator();
            ui.monospace(format!("mag: {}", self.status.magnetometer.label()));
        });

        ui.horizontal_wrapped(|ui| {
            ui.checkbox(&mut self.follow_attitude_plot, "follow attitude plot");
            ui.separator();
            ui.checkbox(&mut self.follow_motion_plot, "follow motion plot");
        });
    }

    pub(crate) fn draw_logs_panel(&self, ui: &mut egui::Ui) {
        ui.heading("Logs");
        egui::ScrollArea::vertical()
            .stick_to_bottom(true)
            .show(ui, |ui| {
                for line in &self.logs {
                    ui.monospace(line);
                }
            });
    }

    pub(crate) fn draw_central_panel(&mut self, ui: &mut egui::Ui) {
        self.draw_orientation_view(ui);
        ui.separator();
        self.draw_attitude_plot(ui);
        ui.separator();
        self.draw_motion_plot(ui);
    }
}
