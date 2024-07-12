use crate::joycon::communication::ChannelData;
use crate::joycon::imu::JoyconAxisData;
use crate::joycon::{Battery, ChannelInfo, JoyconDesign, JoyconDesignType};
use crate::settings;
use std::sync::{mpsc, Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};
use wiimote_rs::input::{ButtonData, InputReport, WiimoteData};
use wiimote_rs::output::{DataReporingMode, OutputReport, PlayerLedFlags};
use wiimote_rs::prelude::*;

use super::ImuData;

struct CalibrationData {
    calibrated: bool,
    start: Instant,
    start_offset: Duration,
    data: Vec<MotionPlusData>,
    calibration: Option<MotionPlusCalibration>,
}

impl CalibrationData {
    const CALIBRATION_START_DELAY: Duration = Duration::from_secs(2);
    const CALIBRATION_COUNT: usize = 16;

    fn new(start: Instant, motion_plus_calibration: Option<MotionPlusCalibration>) -> Self {
        Self {
            calibrated: false,
            start,
            start_offset: Self::CALIBRATION_START_DELAY,
            data: Vec::with_capacity(Self::CALIBRATION_COUNT),
            calibration: motion_plus_calibration,
        }
    }

    fn start_calibration_delayed(&mut self) {
        self.calibrated = false;
        self.start_offset = self.start.elapsed() + Self::CALIBRATION_START_DELAY;
    }

    fn push_data(&mut self, motion_plus_data: MotionPlusData, d: &Arc<Mutex<WiimoteDevice>>) {
        if !self.calibrated && self.start.elapsed() > self.start_offset {
            if self.data.is_empty() {
                println!("Starting calibration");
            }
            self.data.push(motion_plus_data);
            if self.data.len() == Self::CALIBRATION_COUNT {
                if let Some(new_calibration) = Self::calibrate_motion_plus(d, &self.data) {
                    self.calibration.replace(new_calibration);
                    self.calibrated = true;
                }
                self.data.clear();
                println!("Calibrated motion plus");
            }
        }
    }

    fn calibrate_motion_plus(
        d: &Arc<Mutex<WiimoteDevice>>,
        calibration_data: &[MotionPlusData],
    ) -> Option<MotionPlusCalibration> {
        d.lock()
            .unwrap()
            .motion_plus()
            .unwrap()
            .calibrate_zero_values(calibration_data)
    }
}

const fn convert_battery(battery: u8) -> Battery {
    match battery {
        ..=10 => Battery::Empty,
        11..=20 => Battery::Critical,
        21..=40 => Battery::Low,
        41..=70 => Battery::Medium,
        71.. => Battery::Full,
    }
}

fn wiimote_listen_loop(
    d: &Arc<Mutex<WiimoteDevice>>,
    tx: &mpsc::Sender<ChannelData>,
    serial_number: String,
    accelerometer_calibration: &AccelerometerCalibration,
    motion_plus_calibration: Option<MotionPlusCalibration>,
    settings: &settings::Handler,
) {
    let mut last_battery = None;
    let mut last_status_request: Option<Instant> = None;
    let status_check_interval = Duration::from_secs(10);
    let start = Instant::now();

    let mut motion_plus_calibration = CalibrationData::new(start, motion_plus_calibration);

    set_reporting_mode_accelerometer_and_extension(d);

    loop {
        let request_status = last_status_request.map_or(true, |last_status_request| {
            last_status_request.elapsed() > status_check_interval
        });
        if request_status {
            last_status_request = Some(Instant::now());
            let status_request = OutputReport::StatusRequest;
            _ = d.lock().unwrap().write(&status_request);
        }

        let result = d.lock().unwrap().read_timeout(100);
        match result {
            Ok(report) => {
                if let InputReport::StatusInformation(status) = report {
                    // If this report is received when not requested, the application 'MUST'
                    // send report 0x12 to change the data reporting mode, otherwise no further data reports will be received.
                    set_reporting_mode_accelerometer_and_extension(d);
                    let battery_level = convert_battery(status.battery_level());
                    if Some(battery_level) != last_battery {
                        last_battery = Some(battery_level);
                        tx.send(ChannelData::new(
                            serial_number.clone(),
                            ChannelInfo::Battery(battery_level),
                        ))
                        .unwrap();
                    }
                } else if let InputReport::DataReport(0x35, wiimote_data) = &report {
                    let buttons = wiimote_data.buttons();
                    if buttons.contains(ButtonData::A | ButtonData::B) {
                        println!("A and B pressed, starting calibration soon...");
                        motion_plus_calibration.start_calibration_delayed();
                    }
                    if buttons.contains(ButtonData::UP | ButtonData::B) {
                        println!("UP and B pressed, resetting position...");
                        tx.send(ChannelData::new(serial_number.clone(), ChannelInfo::Reset))
                            .unwrap();
                    }

                    let gyro_scale = settings.load().joycon_scale_get(&serial_number);

                    if let Some((imu_data, motion_plus_data)) = get_axis_data(
                        wiimote_data,
                        accelerometer_calibration,
                        &motion_plus_calibration,
                        gyro_scale,
                    ) {
                        tx.send(ChannelData::new(
                            serial_number.clone(),
                            ChannelInfo::ImuData(ImuData::SingleEntry(imu_data)),
                        ))
                        .unwrap();

                        motion_plus_calibration.push_data(motion_plus_data, d);
                    }
                }
            }
            Err(WiimoteError::Disconnected) => {
                tx.send(ChannelData::new(serial_number, ChannelInfo::Disconnected))
                    .unwrap();
                // Break out of listen loop and wait for reconnection
                return;
            }
            _ => {}
        }
    }
}

fn get_axis_data(
    wiimote_data: &WiimoteData,
    accelerometer_calibration: &AccelerometerCalibration,
    motion_plus_calibration: &CalibrationData,
    gyro_scale: f64,
) -> Option<(JoyconAxisData, MotionPlusData)> {
    if let Some(calibration) = &motion_plus_calibration.calibration {
        let accelerometer_data = AccelerometerData::from_normal_reporting(&wiimote_data.data);
        let (x, y, z) = accelerometer_calibration.get_acceleration(&accelerometer_data);

        let mut motion_plus_buffer = [0u8; 6];
        motion_plus_buffer.copy_from_slice(&wiimote_data.data[5..11]);

        if let Ok(motion_plus_data) = MotionPlusData::try_from(motion_plus_buffer) {
            let (yaw, roll, pitch) = calibration.get_angular_velocity(&motion_plus_data);

            let imu_data = JoyconAxisData {
                accel_x: z, // wiimote laying flat
                accel_y: x, // wiimote laying with the left side up
                accel_z: y, // wiimote pointing downwards

                // Starting from an upright position, the wiimote's axes are:
                gyro_x: -yaw * gyro_scale, // around forward axis
                gyro_y: -pitch * gyro_scale, // around left/right axis
                gyro_z: roll * gyro_scale, // around upward axis
            };
            return Some((imu_data, motion_plus_data));
        }
    }
    None
}

fn set_reporting_mode_accelerometer_and_extension(d: &Arc<Mutex<WiimoteDevice>>) -> bool {
    let reporting_mode = OutputReport::DataReportingMode(DataReporingMode {
        continuous: true,
        mode: 0x35, // Core Buttons and Accelerometer with 16 Extension Bytes
    });
    d.lock().unwrap().write(&reporting_mode).is_ok()
}

fn wiimote_thread(
    d: Arc<Mutex<WiimoteDevice>>,
    tx: mpsc::Sender<ChannelData>,
    settings: settings::Handler,
) {
    loop {
        if match d.lock() {
            Ok(d) => d,
            Err(d) => d.into_inner(),
        }
        .is_connected()
        {
            let led_report = OutputReport::PlayerLed(PlayerLedFlags::LED_2 | PlayerLedFlags::LED_3);
            d.lock().unwrap().write(&led_report).unwrap();

            let (identifier, motion_plus_type, accelerometer_calibration, motion_plus_calibration) = {
                let wiimote = d.lock().unwrap();
                if let Some(motion_plus) = wiimote.motion_plus() {
                    motion_plus.initialize(&wiimote).unwrap();
                    motion_plus
                        .change_mode(&wiimote, MotionPlusMode::Active)
                        .unwrap();
                }
                (
                    wiimote.identifier().to_owned(),
                    wiimote.motion_plus().map(MotionPlus::motion_plus_type),
                    wiimote.accelerometer_calibration().clone(),
                    wiimote.motion_plus().map(MotionPlus::calibration),
                )
            };

            let design = JoyconDesign {
                color: "#FFFFFF".to_owned(),
                design_type: match motion_plus_type {
                    None => JoyconDesignType::Wiimote,
                    Some(MotionPlusType::Builtin) => JoyconDesignType::WiimotePlus,
                    Some(MotionPlusType::External) => JoyconDesignType::WiimoteExternalMotionPlus,
                },
            };
            tx.send(ChannelData {
                serial_number: identifier.clone(),
                info: ChannelInfo::Connected(design),
            })
            .unwrap();

            wiimote_listen_loop(
                &d,
                &tx,
                identifier,
                &accelerometer_calibration,
                motion_plus_calibration,
                &settings,
            );
        }
        // Wiimote was disconnected, check for reconnection after 1 second
        thread::sleep(Duration::from_millis(1000));
    }
}

pub fn spawn_wiimote_thread(tx: mpsc::Sender<ChannelData>, settings: settings::Handler) {
    let manager = WiimoteManager::get_instance();
    let devices = {
        let lock = manager.lock();
        match lock {
            Ok(manager) => manager.new_devices_receiver(),
            Err(_) => return,
        }
    };
    for d in devices.iter() {
        let tx = tx.clone();
        let settings = settings.clone();
        thread::spawn(move || wiimote_thread(d, tx, settings));
    }
}
