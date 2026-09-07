use std::collections::VecDeque;
use std::sync::mpsc::Receiver;
use std::sync::{
    Arc,
    atomic::{AtomicBool, Ordering},
};
use std::time::Duration;

use eframe::egui;
use glam::Mat3;

use super::plots::PlotCache;
use super::{AppEvent, FeatureStatus, HISTORY_LIMIT, LOG_LIMIT, PortConfig, Sample};

pub(crate) struct ImuVizApp {
    pub(crate) port: PortConfig,
    pub(crate) rx: Receiver<AppEvent>,
    pub(crate) stop: Arc<AtomicBool>,
    pub(crate) samples: VecDeque<Sample>,
    pub(crate) logs: VecDeque<String>,
    pub(crate) connected: bool,
    pub(crate) follow_attitude_plot: bool,
    pub(crate) follow_motion_plot: bool,
    pub(crate) status: FeatureStatus,
    pub(crate) plot_cache: Option<PlotCache>,
}

impl ImuVizApp {
    pub(crate) fn new(port: PortConfig, rx: Receiver<AppEvent>, stop: Arc<AtomicBool>) -> Self {
        Self {
            status: FeatureStatus::unknown(),
            port,
            rx,
            stop,
            samples: VecDeque::with_capacity(HISTORY_LIMIT),
            logs: VecDeque::with_capacity(LOG_LIMIT),
            connected: true,
            follow_attitude_plot: true,
            follow_motion_plot: true,
            plot_cache: None,
        }
    }

    pub(crate) fn current(&self) -> Option<Sample> {
        self.samples.back().copied()
    }

    pub(crate) fn current_basis(&self) -> Mat3 {
        Mat3::from_quat(self.current().unwrap_or_default().orientation)
    }

    pub(crate) fn poll_events(&mut self) {
        while let Ok(event) = self.rx.try_recv() {
            match event {
                AppEvent::Log(line) => self.push_log(line),
                AppEvent::Sample(sample, status) => {
                    self.status = status;
                    self.plot_cache = None;
                    if self.samples.len() == HISTORY_LIMIT {
                        self.samples.pop_front();
                    }
                    self.samples.push_back(sample);
                }
                AppEvent::Disconnected(reason) => {
                    self.push_log(format!("disconnected: {reason}"));
                    self.connected = false;
                }
            }
        }
    }

    fn push_log(&mut self, line: String) {
        if self.logs.len() == LOG_LIMIT {
            self.logs.pop_front();
        }
        self.logs.push_back(line);
    }
}

impl Drop for ImuVizApp {
    fn drop(&mut self) {
        self.stop.store(false, Ordering::Relaxed);
    }
}

impl eframe::App for ImuVizApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        self.poll_events();
        ctx.request_repaint_after(Duration::from_millis(16));

        egui::TopBottomPanel::top("top_panel").show(ctx, |ui| self.draw_top_panel(ui));
        egui::SidePanel::right("logs_panel")
            .resizable(true)
            .default_width(360.0)
            .show(ctx, |ui| self.draw_logs_panel(ui));
        egui::CentralPanel::default().show(ctx, |ui| self.draw_central_panel(ui));
    }
}
