use eframe::egui;
use eframe::egui::Color32;
use egui_plot::{Legend, Line, Plot, PlotPoints};

use super::app::ImuVizApp;

impl ImuVizApp {
    fn attitude_points<F>(&self, map: F) -> PlotPoints<'static>
    where
        F: Fn(super::Sample) -> f32,
    {
        self.samples
            .iter()
            .map(|sample| [sample.elapsed_ms as f64 / 1000.0, map(*sample) as f64])
            .collect()
    }

    pub(crate) fn draw_attitude_plot(&mut self, ui: &mut egui::Ui) {
        Plot::new("attitude_plot")
            .legend(Legend::default())
            .height(220.0)
            .x_axis_label("Time [s]")
            .y_axis_label("Angle [deg]")
            .allow_drag(true)
            .allow_scroll(true)
            .allow_zoom(true)
            .show(ui, |plot_ui| {
                if self.follow_attitude_plot {
                    plot_ui.set_auto_bounds(egui::Vec2b::TRUE);
                }
                plot_ui.line(
                    Line::new("roll [deg]", self.attitude_points(|sample| sample.roll_deg))
                        .color(Color32::from_rgb(0, 200, 255)),
                );
                plot_ui.line(
                    Line::new(
                        "pitch [deg]",
                        self.attitude_points(|sample| sample.pitch_deg),
                    )
                    .color(Color32::from_rgb(255, 210, 0)),
                );
                plot_ui.line(
                    Line::new("yaw [deg]", self.attitude_points(|sample| sample.yaw_deg))
                        .color(Color32::from_rgb(255, 0, 180)),
                );
            });
    }

    pub(crate) fn draw_motion_plot(&mut self, ui: &mut egui::Ui) {
        Plot::new("motion_plot")
            .legend(Legend::default())
            .height(220.0)
            .x_axis_label("Time [s]")
            .y_axis_label("Altitude [m] / Velocity [m/s]")
            .allow_drag(true)
            .allow_scroll(true)
            .allow_zoom(true)
            .show(ui, |plot_ui| {
                if self.follow_motion_plot {
                    plot_ui.set_auto_bounds(egui::Vec2b::TRUE);
                }
                plot_ui.line(
                    Line::new("altitude [m]", self.attitude_points(|sample| sample.altitude_m))
                        .color(Color32::from_rgb(0, 220, 120)),
                );
                plot_ui.line(
                    Line::new(
                        "vx [m/s]",
                        self.attitude_points(|sample| sample.velocity_world_m_s.x),
                    )
                    .color(Color32::from_rgb(255, 90, 90)),
                );
                plot_ui.line(
                    Line::new(
                        "vy [m/s]",
                        self.attitude_points(|sample| sample.velocity_world_m_s.y),
                    )
                    .color(Color32::from_rgb(255, 210, 0)),
                );
                plot_ui.line(
                    Line::new(
                        "vz [m/s]",
                        self.attitude_points(|sample| sample.velocity_world_m_s.z),
                    )
                    .color(Color32::from_rgb(90, 160, 255)),
                );
            });
    }
}
