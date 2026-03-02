use anyhow::{Context, Result};
use comms::messages::LidarScan;
use rplidar_driver::{Health, RplidarDevice, RplidarDriver, ScanOptions};
use rplidar_driver::rpos_drv::RposError;
use serialport::{SerialPort, TTYPort};
use std::time::Duration;
use tokio::sync::mpsc;
use tracing::{error, info, warn};

#[derive(Debug, Clone)]
pub struct LidarConfig {
    pub id: String,
    pub device: String,
}

pub struct LidarScanner {
    config: LidarConfig,
    // Use concrete TTYPort type for Linux serial communication
    driver: Option<RplidarDevice<TTYPort>>,
}

impl LidarScanner {
    pub fn new(config: LidarConfig) -> Result<Self> {
        let driver = Self::open_device(&config)?;
        Ok(Self {
            config,
            driver: Some(driver),
        })
    }

    fn open_device(config: &LidarConfig) -> Result<RplidarDevice<TTYPort>> {
        info!("Opening LiDAR device {} at {}", config.id, config.device);

        // Open serial port as TTYPort (Linux-specific)
        let builder = serialport::new(&config.device, 115200)
            .timeout(Duration::from_millis(1000));
        let mut port = TTYPort::open(&builder)
            .with_context(|| format!("Failed to open serial port {}", config.device))?;

        // CRITICAL: Clear DTR to allow motor to spin
        // Reference: SLAMTEC SDK src/arch/linux/net_serial.cpp clearDTR()
        // The RPLiDAR A1M8 motor will not spin if DTR is high
        port.write_data_terminal_ready(false)
            .context("Failed to clear DTR signal")?;
        info!("DTR cleared for {} - motor should start spinning", config.id);

        // Give motor time to start
        std::thread::sleep(Duration::from_millis(500));

        let mut driver = RplidarDevice::with_stream(port);

        // Get device info
        match driver.get_device_info() {
            Ok(info) => {
                info!(
                    "LiDAR {} - Model: {}, Firmware: {}.{}, Hardware: {}",
                    config.id,
                    info.model,
                    info.firmware_version >> 8,
                    info.firmware_version & 0xff,
                    info.hardware_version
                );
            }
            Err(e) => {
                warn!("Failed to get device info for {}: {}", config.id, e);
            }
        }

        // Check device health
        match driver.get_device_health() {
            Ok(Health::Healthy) => {
                info!("LiDAR {} is healthy", config.id);
            }
            Ok(Health::Warning(code)) => {
                warn!("LiDAR {} health warning: {:04X}", config.id, code);
            }
            Ok(Health::Error(code)) => {
                error!("LiDAR {} health error: {:04X}", config.id, code);
                return Err(anyhow::anyhow!(
                    "LiDAR {} unhealthy with error code {:04X}",
                    config.id,
                    code
                ));
            }
            Err(e) => {
                warn!("Failed to get device health for {}: {}", config.id, e);
            }
        }

        // Check motor control support and start motor
        match driver.check_motor_ctrl_support() {
            Ok(true) => {
                info!("Starting motor for {} with PWM 600", config.id);
                driver
                    .set_motor_pwm(600)
                    .context("Failed to start motor")?;
                std::thread::sleep(Duration::from_millis(500));
            }
            Ok(false) => {
                info!("Motor control not supported for {} - motor should be externally powered", config.id);
            }
            Err(e) => {
                warn!("Failed to check motor control support for {}: {}", config.id, e);
            }
        }

        // Get scan modes (for logging)
        if let Ok(modes) = driver.get_all_supported_scan_modes() {
            info!("LiDAR {} supported scan modes:", config.id);
            for mode in modes {
                info!(
                    "  Mode {}: {} (max: {:.2}m)",
                    mode.id, mode.name, mode.max_distance
                );
            }
        }

        // Start scanning - use standard mode (mode 0 for A1)
        let scan_options = ScanOptions::with_mode(0);
        let actual_mode = driver
            .start_scan_with_options(&scan_options)
            .context("Failed to start scan")?;

        info!("LiDAR {} started scanning in mode: {}", config.id, actual_mode.name);

        Ok(driver)
    }

    pub fn run_loop(mut self, tx: mpsc::Sender<LidarScan>) -> Result<()> {
        info!("Starting scan loop for {}", self.config.id);

        let mut consecutive_errors = 0;
        const MAX_CONSECUTIVE_ERRORS: u32 = 10;

        loop {
            match self.read_scan() {
                Ok(scan) => {
                    consecutive_errors = 0;
                    if let Err(e) = tx.blocking_send(scan) {
                        error!("Failed to send scan for {}: {}", self.config.id, e);
                        break;
                    }
                }
                Err(e) => {
                    consecutive_errors += 1;

                    // Don't log timeout errors at warn level - they're expected
                    if e.to_string().contains("OperationTimeout") {
                        continue;
                    }

                    warn!("Scan error for {} ({}): {}", self.config.id, consecutive_errors, e);

                    if consecutive_errors >= MAX_CONSECUTIVE_ERRORS {
                        error!(
                            "Too many consecutive errors for {} - attempting reconnect",
                            self.config.id
                        );
                        consecutive_errors = 0;

                        // Attempt reconnect
                        if let Err(e) = self.reconnect() {
                            error!("Reconnect failed for {}: {}", self.config.id, e);
                            std::thread::sleep(Duration::from_secs(5));
                        }
                    } else {
                        std::thread::sleep(Duration::from_millis(100));
                    }
                }
            }
        }

        Ok(())
    }

    fn read_scan(&mut self) -> Result<LidarScan> {
        let driver = self
            .driver
            .as_mut()
            .context("Device not initialized")?;

        // Grab a full 360° scan
        let scan_points = match driver.grab_scan() {
            Ok(points) => points,
            Err(RposError::OperationTimeout) => {
                return Err(anyhow::anyhow!("OperationTimeout"));
            }
            Err(e) => {
                return Err(anyhow::anyhow!("Scan error: {:?}", e));
            }
        };

        // Filter invalid points and convert to our format
        let valid_points: Vec<_> = scan_points
            .into_iter()
            .filter(|p| p.is_valid() && p.distance() > 0.0)
            .collect();

        info!(
            "LiDAR {} grabbed scan with {} valid points",
            self.config.id,
            valid_points.len()
        );

        // Convert to ranges and intensities
        let mut ranges = Vec::with_capacity(valid_points.len());
        let mut intensities = Vec::with_capacity(valid_points.len());

        for point in valid_points {
            ranges.push(point.distance()); // Already in meters
            intensities.push(point.quality); // Field access, not method
        }

        // RPLiDAR A1M8 specifications
        let angle_min = 0.0;
        let angle_max = 2.0 * std::f32::consts::PI;
        let angle_increment = if !ranges.is_empty() {
            angle_max / ranges.len() as f32
        } else {
            0.0
        };

        Ok(LidarScan {
            sensor_id: self.config.id.clone(),
            angle_min,
            angle_max,
            angle_increment,
            range_min: 0.15, // RPLiDAR A1M8 minimum range
            range_max: 12.0, // RPLiDAR A1M8 maximum range
            ranges,
            intensities,
        })
    }

    fn reconnect(&mut self) -> Result<()> {
        warn!("Attempting to reconnect LiDAR {}", self.config.id);

        // Close existing device if any
        if let Some(mut driver) = self.driver.take() {
            let _ = driver.stop();
            let _ = driver.stop_motor();
        }

        // Wait a bit before reconnecting
        std::thread::sleep(Duration::from_secs(2));

        // Try to reopen
        match Self::open_device(&self.config) {
            Ok(driver) => {
                self.driver = Some(driver);
                info!("LiDAR {} reconnected successfully", self.config.id);
                Ok(())
            }
            Err(e) => {
                error!("Failed to reconnect LiDAR {}: {}", self.config.id, e);
                Err(e)
            }
        }
    }
}

impl Drop for LidarScanner {
    fn drop(&mut self) {
        if let Some(mut driver) = self.driver.take() {
            info!("Stopping scan for {}", self.config.id);
            let _ = driver.stop();
            let _ = driver.stop_motor();
        }
    }
}
