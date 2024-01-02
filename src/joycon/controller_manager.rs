use hidapi::{DeviceInfo, HidApi, HidResult};
use joycon_rs::joycon::JoyConDevice;
use wiimote_rs::prelude::{prepare_wiimote_connections, WiimoteDevice};

use std::collections::{HashMap, HashSet};
use std::sync::{Arc, Mutex, MutexGuard, Once};
use std::thread::JoinHandle;
use std::time::Duration;

#[derive(Debug, PartialEq, Eq, Hash, Clone)]
pub struct WiimoteSerialNumber(pub String);

#[derive(Clone)]
pub enum Controller {
    JoyCon(Arc<Mutex<JoyConDevice>>),
    Wiimote(Arc<Mutex<WiimoteDevice>>),
}

impl Controller {
    pub fn disconnected(&self) {
        match self {
            Self::JoyCon(device) => {
                let mut device = Self::lock_ignore_err(device);
                if device.is_connected() {
                    device.forget_device();
                }
            }
            Self::Wiimote(device) => {
                let mut device = Self::lock_ignore_err(device);
                if device.is_connected() {
                    device.disconnected();
                }
            }
        }
    }

    pub fn reconnect(&self, hid_api: &HidApi, device_info: &DeviceInfo) -> bool {
        match self {
            Self::JoyCon(device) => {
                let mut device = Self::lock_ignore_err(device);
                if !device.is_connected() {
                    if let Ok(hid_device) = device_info.open_device(hid_api) {
                        device.reset_device(hid_device);
                        return true;
                    }
                }
                false
            }
            Self::Wiimote(device) => {
                let mut device = Self::lock_ignore_err(device);
                !device.is_connected() && device.reconnect(device_info, hid_api).is_ok()
            }
        }
    }

    fn lock_ignore_err<T>(arc: &Arc<Mutex<T>>) -> MutexGuard<T> {
        match arc.lock() {
            Ok(guard) => guard,
            Err(e) => e.into_inner(),
        }
    }
}

/// Periodically checks for connections / disconnections of Joy-Cons and Wii remotes.
pub struct ControllerManager {
    devices: HashMap<String, Controller>,
    hid_api: Option<HidApi>,
    scan_thread: Option<JoinHandle<()>>,
    scan_interval: Duration,
    new_devices_receiver: crossbeam_channel::Receiver<Controller>,
}

impl ControllerManager {
    /// Get the Wii remote manager instance.
    ///
    /// # Panics
    ///
    /// Panics if `HidApi` failes to initialize or detect devices.
    pub fn get_instance() -> Arc<Mutex<Self>> {
        static mut SINGLETON: Option<Arc<Mutex<ControllerManager>>> = None;
        static ONCE: Once = Once::new();

        unsafe {
            ONCE.call_once(|| {
                let instance = Self::new().unwrap();

                SINGLETON = Some(instance);
            });

            SINGLETON.clone().unwrap_or_else(|| unreachable!())
        }
    }

    fn new() -> HidResult<Arc<Mutex<Self>>> {
        Self::with_interval(Duration::from_millis(100))
    }

    fn with_interval(interval: Duration) -> HidResult<Arc<Mutex<Self>>> {
        let (new_sender, new_devices_receiver) = crossbeam_channel::unbounded();

        let manager = {
            let mut manager = Self {
                devices: HashMap::new(),
                hid_api: None,
                scan_thread: None,
                scan_interval: interval,
                new_devices_receiver,
            };

            // Immediately scan for devices
            let new_devices = manager.scan()?;
            for new_device in new_devices {
                let _ = new_sender.send(new_device);
            }

            Arc::new(Mutex::new(manager))
        };

        let scan_thread = {
            let manager = Arc::downgrade(&manager);

            std::thread::spawn(move || {
                while let Some(manager) = manager.upgrade() {
                    let interval = {
                        let mut manager = match manager.lock() {
                            Ok(m) => m,
                            Err(m) => m.into_inner(),
                        };

                        if let Ok(new_devices) = manager.scan() {
                            let send_result = new_devices
                                .into_iter()
                                .try_for_each(|device| new_sender.send(device));
                            if send_result.is_err() {
                                // Channel is disconnected, end scan thread
                                return;
                            }
                        }

                        manager.scan_interval
                    };

                    std::thread::sleep(interval);
                }
            })
        };

        if let Ok(mut manager) = manager.lock() {
            manager.scan_thread = Some(scan_thread);
        }

        Ok(manager)
    }

    pub fn set_scan_interval(&mut self, interval: Duration) {
        self.scan_interval = interval;
    }

    /// Scan the Wii remotes connected to your computer.
    ///
    /// # Errors
    ///
    /// Returns an error if `HidApi` failes to initialize or detect devices.
    fn scan(&mut self) -> HidResult<Vec<Controller>> {
        prepare_wiimote_connections();

        let hid_api = if let Some(hid_api) = &mut self.hid_api {
            hid_api.refresh_devices()?;
            hid_api
        } else {
            self.hid_api = Some(HidApi::new()?);
            self.hid_api.as_mut().unwrap_or_else(|| unreachable!())
        };

        let detected_devices = hid_api
            .device_list()
            .filter(|&device_info| {
                JoyConDevice::check_type_of_device(device_info).is_ok()
                    || WiimoteDevice::get_wiimote_device_type(device_info).is_ok()
            })
            .filter_map(|device_info| {
                device_info
                    .serial_number()
                    .map(ToString::to_string)
                    .map(|serial| (device_info, serial))
            })
            .collect::<Vec<_>>();

        // Disconnected devices
        for removed_serial in self
            .devices
            .keys()
            .cloned()
            .collect::<HashSet<_>>()
            .difference(&detected_devices.iter().map(|(_, s)| s.clone()).collect())
        {
            if let Some(device) = self.devices.get(removed_serial) {
                device.disconnected();
            }
        }

        let mut new_devices = Vec::new();
        for (device_info, serial) in detected_devices {
            if self.devices.contains_key(&serial) {
                let previous_device = self.devices.get(&serial).unwrap_or_else(|| unreachable!());

                // Reconnected device
                previous_device.reconnect(hid_api, device_info);
            } else {
                // New device
                let is_wiimote = WiimoteDevice::get_wiimote_device_type(device_info).is_ok();
                if is_wiimote {
                    if let Ok(device) = WiimoteDevice::new(device_info, hid_api) {
                        let controller = Controller::Wiimote(Arc::new(Mutex::new(device)));
                        new_devices.push(controller.clone());
                        self.devices.insert(serial.clone(), controller);
                    }
                } else if let Ok(device) = JoyConDevice::new(device_info, hid_api) {
                    let controller = Controller::JoyCon(Arc::new(Mutex::new(device)));
                    new_devices.push(controller.clone());
                    self.devices.insert(serial.clone(), controller);
                }
            }
        }

        Ok(new_devices)
    }

    /// Channel to receive newly connected controllers.
    #[must_use]
    pub fn new_devices_receiver(&self) -> crossbeam_channel::Receiver<Controller> {
        self.new_devices_receiver.clone()
    }
}
