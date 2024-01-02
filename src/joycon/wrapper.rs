use std::{env, sync::mpsc};

use crate::settings;

use super::controller_manager::{Controller, ControllerManager};
#[cfg(target_os = "linux")]
use super::linux_integration;
use super::{
    communication::ServerStatus, integration::joycon_thread, test_integration::test_controllers,
    wiimote_integration::wiimote_thread, ChannelData, Communication, Status,
};

pub struct Wrapper {
    status_rx: mpsc::Receiver<Vec<Status>>,
    server_rx: mpsc::Receiver<ServerStatus>,
}
impl Wrapper {
    pub fn new(settings: settings::Handler) -> Self {
        let (status_tx, status_rx) = mpsc::channel();
        let (server_tx, server_rx) = mpsc::channel();
        let (tx, rx) = mpsc::channel();

        {
            let settings = settings.clone();
            std::thread::spawn(move || {
                Communication::start(rx, status_tx, server_tx, settings);
            });
        }

        {
            let tx = tx.clone();
            if env::args().any(|a| &a == "test") {
                std::thread::spawn(move || test_controllers(tx));
            }
        }

        // evdev integration
        #[cfg(target_os = "linux")]
        {
            let tx = tx.clone();
            let settings = settings.clone();
            std::thread::spawn(move || linux_integration::spawn_thread(tx, settings));
        }

        std::thread::spawn(move || Self::spawn_controller_thread(tx, settings));

        Self {
            status_rx,
            server_rx,
        }
    }
    pub fn poll_status(&self) -> Option<Vec<Status>> {
        self.status_rx.try_iter().last()
    }
    pub fn poll_server(&self) -> Option<ServerStatus> {
        self.server_rx.try_iter().last()
    }

    fn spawn_controller_thread(tx: mpsc::Sender<ChannelData>, settings: settings::Handler) {
        let manager = ControllerManager::get_instance();
        let devices = {
            let lock = manager.lock();
            match lock {
                Ok(manager) => manager.new_devices_receiver(),
                Err(_) => return,
            }
        };
        for controller in &devices {
            let tx = tx.clone();
            let settings = settings.clone();
            std::thread::spawn(move || Self::forward_to_integration(controller, tx, settings));
        }
    }

    fn forward_to_integration(
        controller: Controller,
        tx: mpsc::Sender<ChannelData>,
        settings: settings::Handler,
    ) {
        match controller {
            Controller::JoyCon(joy_con) => joycon_thread(joy_con, tx, settings),
            Controller::Wiimote(wiimote) => wiimote_thread(wiimote, tx, settings),
        }
    }
}
