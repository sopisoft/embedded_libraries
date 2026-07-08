use std::collections::VecDeque;
use std::process::Child;
use std::sync::mpsc::Receiver;
use std::time::Duration;

use eframe::egui;
use glam::Mat3;

use super::{AppEvent, FeatureStatus, HISTORY_LIMIT, LOG_LIMIT, LaunchConfig, Sample};

pub(crate) struct ImuVizApp {
    pub(crate) launch: LaunchConfig,
    pub(crate) rx: Receiver<AppEvent>,
    pub(crate) child: Option<Child>,
    pub(crate) samples: VecDeque<Sample>,
    pub(crate) logs: VecDeque<String>,
    pub(crate) child_running: bool,
    pub(crate) follow_attitude_plot: bool,
    pub(crate) follow_motion_plot: bool,
    pub(crate) status: FeatureStatus,
}

impl ImuVizApp {
    pub(crate) fn new(launch: LaunchConfig, rx: Receiver<AppEvent>, child: Child) -> Self {
        Self {
            status: FeatureStatus::from_launch(&launch),
            launch,
            rx,
            child: Some(child),
            samples: VecDeque::with_capacity(HISTORY_LIMIT),
            logs: VecDeque::with_capacity(LOG_LIMIT),
            child_running: true,
            follow_attitude_plot: true,
            follow_motion_plot: true,
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
                AppEvent::Log(line) => {
                    self.status.update_from_log(&line);
                    self.push_log(line);
                }
                AppEvent::Sample(sample) => {
                    if self.samples.len() == HISTORY_LIMIT {
                        self.samples.pop_front();
                    }
                    self.samples.push_back(sample);
                }
            }
        }

        if let Some(child) = self.child.as_mut() {
            if self.child_running {
                match child.try_wait() {
                    Ok(Some(status)) => {
                        self.push_log(format!("child exited: {status}"));
                        self.child_running = false;
                    }
                    Ok(None) => {}
                    Err(error) => {
                        self.push_log(format!("wait error: {error}"));
                        self.child_running = false;
                    }
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
        if let Some(child) = self.child.as_mut() {
            let _ = child.kill();
        }
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
