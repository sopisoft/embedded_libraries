use std::collections::VecDeque;

use eframe::egui;
use eframe::egui::Color32;
use egui_plot::{Legend, Line, Plot, PlotPoint, PlotPoints};

use super::Sample;
use super::app::ImuVizApp;

pub(crate) struct PlotCache {
    roll: Vec<PlotPoint>,
    pitch: Vec<PlotPoint>,
    yaw: Vec<PlotPoint>,
    altitude: Vec<PlotPoint>,
    velocity_x: Vec<PlotPoint>,
    velocity_y: Vec<PlotPoint>,
    velocity_z: Vec<PlotPoint>,
}

impl PlotCache {
    fn from_samples(samples: &VecDeque<Sample>) -> Self {
        fn points<F>(samples: &VecDeque<Sample>, map: F) -> Vec<PlotPoint>
        where
            F: Fn(Sample) -> f32,
        {
            samples
                .iter()
                .map(|sample| PlotPoint::new(sample.elapsed_ms as f64 / 1000.0, map(*sample)))
                .collect()
        }

        Self {
            roll: points(samples, |sample| sample.roll_deg),
            pitch: points(samples, |sample| sample.pitch_deg),
            yaw: points(samples, |sample| sample.yaw_deg),
            altitude: points(samples, |sample| sample.altitude_m),
            velocity_x: points(samples, |sample| sample.velocity_world_m_s.x),
            velocity_y: points(samples, |sample| sample.velocity_world_m_s.y),
            velocity_z: points(samples, |sample| sample.velocity_world_m_s.z),
        }
    }
}

impl ImuVizApp {
    fn plot_cache(&mut self) -> &PlotCache {
        if self.plot_cache.is_none() {
            self.plot_cache = Some(PlotCache::from_samples(&self.samples));
        }
        self.plot_cache.as_ref().unwrap()
    }

    pub(crate) fn draw_attitude_plot(&mut self, ui: &mut egui::Ui) {
        let follow = self.follow_attitude_plot;
        let cache = self.plot_cache();
        Plot::new("attitude_plot")
            .legend(Legend::default())
            .height(220.0)
            .x_axis_label("Time [s]")
            .y_axis_label("Angle [deg]")
            .allow_drag(true)
            .allow_scroll(true)
            .allow_zoom(true)
            .show(ui, |plot_ui| {
                if follow {
                    plot_ui.set_auto_bounds(egui::Vec2b::TRUE);
                }
                plot_ui.line(
                    Line::new("roll [deg]", PlotPoints::Borrowed(&cache.roll))
                        .color(Color32::from_rgb(0, 200, 255)),
                );
                plot_ui.line(
                    Line::new("pitch [deg]", PlotPoints::Borrowed(&cache.pitch))
                        .color(Color32::from_rgb(255, 210, 0)),
                );
                plot_ui.line(
                    Line::new("yaw [deg]", PlotPoints::Borrowed(&cache.yaw))
                        .color(Color32::from_rgb(255, 0, 180)),
                );
            });
    }

    pub(crate) fn draw_motion_plot(&mut self, ui: &mut egui::Ui) {
        let follow = self.follow_motion_plot;
        let cache = self.plot_cache();
        Plot::new("motion_plot")
            .legend(Legend::default())
            .height(220.0)
            .x_axis_label("Time [s]")
            .y_axis_label("Altitude [m] / Velocity [m/s]")
            .allow_drag(true)
            .allow_scroll(true)
            .allow_zoom(true)
            .show(ui, |plot_ui| {
                if follow {
                    plot_ui.set_auto_bounds(egui::Vec2b::TRUE);
                }
                plot_ui.line(
                    Line::new("altitude [m]", PlotPoints::Borrowed(&cache.altitude))
                        .color(Color32::from_rgb(0, 220, 120)),
                );
                plot_ui.line(
                    Line::new("vx [m/s]", PlotPoints::Borrowed(&cache.velocity_x))
                        .color(Color32::from_rgb(255, 90, 90)),
                );
                plot_ui.line(
                    Line::new("vy [m/s]", PlotPoints::Borrowed(&cache.velocity_y))
                        .color(Color32::from_rgb(255, 210, 0)),
                );
                plot_ui.line(
                    Line::new("vz [m/s]", PlotPoints::Borrowed(&cache.velocity_z))
                        .color(Color32::from_rgb(90, 160, 255)),
                );
            });
    }
}
