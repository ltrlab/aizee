// AIZEE Motor Control - Main Control Loop

mod motor;
mod robstride;

use anyhow::{Context, Result};
use comms::{
    ArmJointsPayload, CommandMessage, CommandSubscriber, DrivePayload, MotorTelemetry,
    TelemetryMessage, TelemetryPublisher,
};
use motor::{Motor, MotorConfig, MotorGroup, MotorState};
use robstride::MotorModel;
use socketcan::{CanSocket, EmbeddedFrame, Frame, Socket, SocketOptions};
use std::cell::RefCell;
use std::collections::HashMap;
use std::process::Command;
use std::time::{Duration, Instant};
use tokio::time::interval;
use tracing::{debug, error, info, warn};

#[derive(Debug, Clone, serde::Deserialize)]
struct Config {
    motors: MotorsConfig,
    control: ControlConfig,
    can: CanConfig,
    network: NetworkConfig,
}

#[derive(Debug, Clone, serde::Deserialize)]
struct MotorsConfig {
    wheels: Vec<MotorConfigYaml>,
    swivel: MotorConfigYaml,
    arm: Vec<MotorConfigYaml>,
}

#[derive(Debug, Clone, serde::Deserialize)]
struct MotorConfigYaml {
    id: String,
    can_id: u8,
    #[serde(default = "default_can_bus")]
    can_bus: String,  // Optional, defaults to "can1" for backward compatibility
    #[serde(rename = "type")]
    motor_type: String,
    #[serde(default)]
    min_position: Option<f32>,
    #[serde(default)]
    max_position: Option<f32>,
    max_velocity: f32,
    max_torque: f32,
}

fn default_can_bus() -> String {
    "can1".to_string()
}

#[derive(Debug, Clone, serde::Deserialize)]
struct ControlConfig {
    arm_frequency: f32,
    base_frequency: f32,
    telemetry_rate: f32,
    watchdog_timeout: f32,
}

#[derive(Debug, Clone, serde::Deserialize)]
struct CanConfig {
    #[serde(default)]
    interface: String,  // Legacy single interface (deprecated)
    #[serde(default)]
    interfaces: HashMap<String, String>,  // New: map of bus name -> interface name
}

#[derive(Debug, Clone, serde::Deserialize)]
struct NetworkConfig {
    #[serde(alias = "jetson")]  // Backward compatibility
    device: DeviceConfig,
}

#[derive(Debug, Clone, serde::Deserialize)]
struct DeviceConfig {
    #[serde(default)]
    ip: String,
    #[serde(default)]
    hostname: String,
    zmq: ZmqConfig,
}

#[derive(Debug, Clone, serde::Deserialize)]
struct ZmqConfig {
    command_sub: String,
    telemetry_pub: String,
}

fn load_config() -> Result<Config> {
    let config_path = std::env::var("AIZEE_CONFIG").unwrap_or_else(|_| "config/hardware.yaml".to_string());
    let config_str = std::fs::read_to_string(&config_path)
        .with_context(|| format!("Failed to read config from {}", config_path))?;
    let config: Config = serde_yaml::from_str(&config_str)?;
    Ok(config)
}

fn parse_motor_model(type_str: &str) -> MotorModel {
    match type_str {
        "ROBSTRIDE04" => MotorModel::Model04,
        "ROBSTRIDE03" => MotorModel::Model03,
        "ROBSTRIDE02" => MotorModel::Model02,
        "ROBSTRIDE00" => MotorModel::Model00,
        _ => {
            warn!("Unknown motor type {}, defaulting to Model03", type_str);
            MotorModel::Model03
        }
    }
}

fn yaml_to_motor_config(yaml: &MotorConfigYaml) -> MotorConfig {
    MotorConfig {
        id: yaml.id.clone(),
        can_id: yaml.can_id,
        can_bus: yaml.can_bus.clone(),
        model: parse_motor_model(&yaml.motor_type),
        min_position: yaml.min_position,
        max_position: yaml.max_position,
        max_velocity: yaml.max_velocity,
        max_torque: yaml.max_torque,
    }
}

struct ControlSystem {
    can_sockets: HashMap<String, CanSocket>,  // bus_name -> socket
    can_interfaces: HashMap<String, String>,  // bus_name -> interface_name (for socket recovery)
    base_group: MotorGroup,
    arm_group: MotorGroup,
    command_sub: CommandSubscriber,
    telemetry_pub: TelemetryPublisher,
    motor_id_map: HashMap<String, u8>, // motor_id -> can_id
    motor_bus_map: HashMap<String, String>, // motor_id -> can_bus
    emergency_stop: bool,
    last_base_control: Instant,
    control_tick: u64,
    battery_voltage: Option<f32>, // Avg VBUS across fresh per-motor readings
    last_vbus_request: Instant,   // Track when we last requested VBUS
    last_battery_voltage_update: Option<Instant>, // Track freshness of battery reading
    // Per-motor VBUS readings (CAN id -> (volts, when received)).  Robstrides
    // each report their own input voltage; we round-robin queries across
    // ALL motors and average the fresh values for the published
    // battery_voltage.  Used to be single-source from base_group.first(),
    // which broke when the wheels were retired (2026-05-30).
    motor_vbus: HashMap<u8, (f32, Instant)>,
    vbus_query_idx: usize,        // Round-robin cursor over all motors
    dropped_frames: RefCell<HashMap<String, u64>>, // Track dropped frames per CAN bus
    last_buffer_warning: RefCell<HashMap<String, Instant>>, // Rate-limit buffer warnings
    consecutive_tx_errors: RefCell<HashMap<String, u32>>, // Track consecutive errors for recovery
    last_error_log: Instant, // Rate-limit "group has errors" log flood
    last_hold_log: Instant,  // Rate-limit "command timeout hold" log
    last_socket_recovery: Instant, // Rate-limit socket recovery to prevent storm
    can_recovery_level: HashMap<String, u32>, // Current recovery escalation level per bus (0=socket, 1=link, 2=exit)
    can_recovery_attempts: HashMap<String, u32>, // Attempts at current level per bus
    total_recoveries: u32, // Total recovery attempts across all levels (exit at 10)
    last_link_recovery: Instant, // Rate-limit link-level recovery
}

impl ControlSystem {
    fn new(config: Config) -> Result<Self> {
        // Initialize CAN sockets (supports multiple buses)
        let mut can_sockets = HashMap::new();

        // Build interface map (supports both legacy single interface and new multi-interface)
        let mut interfaces = config.can.interfaces.clone();
        if !config.can.interface.is_empty() && interfaces.is_empty() {
            // Legacy mode: single interface on can1
            interfaces.insert("can1".to_string(), config.can.interface.clone());
        }

        // Open all CAN interfaces
        for (bus_name, interface_name) in &interfaces {
            let socket = CanSocket::open(interface_name)
                .with_context(|| format!("Failed to open CAN interface {} for bus {}", interface_name, bus_name))?;

            socket.set_nonblocking(true)
                .with_context(|| format!("Failed to set CAN socket {} to non-blocking mode", bus_name))?;

            // Disable loopback: by default Linux CAN copies every TX frame back to the
            // RX queue for other applications. We're the only app on this bus — disabling
            // loopback eliminates kernel-side frame copying overhead.
            if let Err(e) = socket.set_loopback(false) {
                warn!("Failed to disable CAN loopback on {}: {} (continuing)", bus_name, e);
            }

            info!("CAN interface {} opened for bus {} (non-blocking, loopback off)", interface_name, bus_name);
            can_sockets.insert(bus_name.clone(), socket);
        }

        let can_interfaces = interfaces.clone();

        // Create motor groups
        let watchdog_timeout = Duration::from_secs_f32(config.control.watchdog_timeout);

        // Base group: drive wheels only.  Swivel used to live here at index 2;
        // it now sits at the head of the arm group so the firmware speaks one
        // unified `arm_joints` command instead of swivel-vs-arm split.
        let mut base_motors = Vec::new();
        for wheel in &config.motors.wheels {
            base_motors.push(Motor::new(yaml_to_motor_config(wheel), watchdog_timeout));
        }

        let base_group = MotorGroup::new(
            "base".to_string(),
            base_motors,
            config.control.base_frequency,
        );

        // Arm group: swivel (index 0) + gantry joints, driven at arm_frequency.
        let mut arm_motors: Vec<Motor> = Vec::with_capacity(config.motors.arm.len() + 1);
        arm_motors.push(Motor::new(
            yaml_to_motor_config(&config.motors.swivel),
            watchdog_timeout,
        ));
        for m in &config.motors.arm {
            arm_motors.push(Motor::new(yaml_to_motor_config(m), watchdog_timeout));
        }

        let arm_group = MotorGroup::new(
            "arm".to_string(),
            arm_motors,
            config.control.arm_frequency,
        );

        // Build motor ID and bus maps
        let mut motor_id_map = HashMap::new();
        let mut motor_bus_map = HashMap::new();
        for wheel in &config.motors.wheels {
            motor_id_map.insert(wheel.id.clone(), wheel.can_id);
            motor_bus_map.insert(wheel.id.clone(), wheel.can_bus.clone());
        }
        motor_id_map.insert(config.motors.swivel.id.clone(), config.motors.swivel.can_id);
        motor_bus_map.insert(config.motors.swivel.id.clone(), config.motors.swivel.can_bus.clone());
        for arm_motor in &config.motors.arm {
            motor_id_map.insert(arm_motor.id.clone(), arm_motor.can_id);
            motor_bus_map.insert(arm_motor.id.clone(), arm_motor.can_bus.clone());
        }

        // Initialize ZeroMQ
        let command_sub = CommandSubscriber::new(&config.network.device.zmq.command_sub)?;
        let telemetry_pub = TelemetryPublisher::new(&config.network.device.zmq.telemetry_pub)?;

        info!("Control system initialized");
        Ok(Self {
            can_sockets,
            can_interfaces,
            base_group,
            arm_group,
            command_sub,
            telemetry_pub,
            motor_id_map,
            motor_bus_map,
            emergency_stop: false,
            last_base_control: Instant::now(),
            control_tick: 0,
            battery_voltage: None,
            last_vbus_request: Instant::now(),
            last_battery_voltage_update: None,
            motor_vbus: HashMap::new(),
            vbus_query_idx: 0,
            dropped_frames: RefCell::new(HashMap::new()),
            last_buffer_warning: RefCell::new(HashMap::new()),
            consecutive_tx_errors: RefCell::new(HashMap::new()),
            last_error_log: Instant::now(),
            last_hold_log: Instant::now(),
            last_socket_recovery: Instant::now(),
            can_recovery_level: HashMap::new(),
            can_recovery_attempts: HashMap::new(),
            total_recoveries: 0,
            last_link_recovery: Instant::now(),
        })
    }

    fn find_motor_mut(&mut self, motor_id: &str) -> Option<&mut Motor> {
        self.base_group
            .motors
            .iter_mut()
            .find(|m| m.config.id == motor_id)
            .or_else(|| {
                self.arm_group
                    .motors
                    .iter_mut()
                    .find(|m| m.config.id == motor_id)
            })
    }

    /// Look up motor model by CAN ID (searches both groups)
    fn model_for_can_id(&self, can_id: u8) -> MotorModel {
        for motor in &self.base_group.motors {
            if motor.config.can_id == can_id {
                return motor.config.model;
            }
        }
        for motor in &self.arm_group.motors {
            if motor.config.can_id == can_id {
                return motor.config.model;
            }
        }
        MotorModel::Model03 // fallback
    }

    /// Write a CAN frame with retry on ENOBUFS/EAGAIN.
    ///
    /// The gs_usb adapter has only 10 TX URBs. When all are in-flight, writes return
    /// ENOBUFS (105) or EAGAIN (11). Instead of dropping the frame and sleeping, we
    /// retry with a short backoff — the USB round-trip is ~1ms, so a URB slot will
    /// free up quickly. This matches what other projects (OpenArm, K-Scale) do:
    /// never drop a control frame, just wait for bus capacity.
    ///
    /// Only truly unrecoverable errors (persistent after all retries) increment the
    /// consecutive error counter for escalating recovery.
    fn safe_write_frame(&self, socket: &CanSocket, frame: &socketcan::CanFrame, bus_name: &str) -> Result<()> {
        const MAX_RETRIES: u32 = 10;
        const RETRY_BACKOFF_US: u64 = 150; // ~1 CAN frame time at 1Mbps

        for attempt in 0..=MAX_RETRIES {
            match socket.write_frame(frame) {
                Ok(_) => {
                    // Success — reset consecutive error counter
                    self.consecutive_tx_errors.borrow_mut().insert(bus_name.to_string(), 0);
                    return Ok(());
                }
                Err(e) => {
                    let errno = e.raw_os_error().unwrap_or(0);
                    // ENOBUFS (105) or EAGAIN (11) — TX queue full, retry after backoff
                    if (errno == 105 || errno == 11) && attempt < MAX_RETRIES {
                        std::thread::sleep(Duration::from_micros(RETRY_BACKOFF_US));
                        continue;
                    }

                    // All retries exhausted or non-transient error — count as failure
                    let consecutive = {
                        let mut errors = self.consecutive_tx_errors.borrow_mut();
                        let count = errors.entry(bus_name.to_string()).or_insert(0);
                        *count += 1;
                        *count
                    };

                    let mut dropped = self.dropped_frames.borrow_mut();
                    *dropped.entry(bus_name.to_string()).or_insert(0) += 1;

                    // Rate-limit warnings to once per second
                    let now = Instant::now();
                    let should_warn = {
                        let warnings = self.last_buffer_warning.borrow();
                        warnings.get(bus_name)
                            .map(|last| now.duration_since(*last) >= Duration::from_secs(1))
                            .unwrap_or(true)
                    };

                    if should_warn {
                        let dropped_count = *dropped.get(bus_name).unwrap_or(&0);
                        warn!("CAN {} TX failed after {} retries (errno {}, consecutive: {}, total dropped: {})",
                              bus_name, attempt, errno, consecutive, dropped_count);
                        self.last_buffer_warning.borrow_mut().insert(bus_name.to_string(), now);
                    }

                    return Ok(());
                }
            }
        }
        Ok(())
    }

    /// Write a CAN frame through safe_write_frame (tracking errors) but discard the result.
    /// Use this instead of `let _ = socket.write_frame(...)` so TX errors are counted toward
    /// recovery detection.
    fn tracked_write_frame(&self, socket: &CanSocket, frame: &socketcan::CanFrame, bus_name: &str) {
        let _ = self.safe_write_frame(socket, frame, bus_name);
    }

    /// Level 0 recovery: close and reopen the CAN socket.
    /// Flushes kernel socket buffers but does NOT reset gs_usb TX echo ID pool.
    fn recover_socket(&mut self, bus_name: &str) -> bool {
        let interface_name = match self.can_interfaces.get(bus_name) {
            Some(name) => name.clone(),
            None => return false,
        };

        let error_count = self.consecutive_tx_errors.borrow()
            .get(bus_name).copied().unwrap_or(0);
        warn!("CAN {} Level 0 recovery: socket reopen ({} consecutive TX errors)",
              bus_name, error_count);

        // Drop old socket (closes fd, frees kernel socket buffers)
        self.can_sockets.remove(bus_name);
        std::thread::sleep(Duration::from_millis(50));

        match CanSocket::open(&interface_name) {
            Ok(socket) => {
                if let Err(e) = socket.set_nonblocking(true) {
                    error!("Failed to set recovered socket {} to non-blocking: {}", bus_name, e);
                    return false;
                }
                let _ = socket.set_loopback(false);
                self.can_sockets.insert(bus_name.to_string(), socket);
                self.consecutive_tx_errors.borrow_mut().insert(bus_name.to_string(), 0);
                info!("CAN {} Level 0 recovery complete", bus_name);
                true
            }
            Err(e) => {
                error!("Failed to reopen CAN socket {}: {}", bus_name, e);
                false
            }
        }
    }

    /// Level 1 recovery: ip link down/up to reset gs_usb TX echo ID pool.
    /// Calls `sudo ip link set <iface> down`, reconfigures bitrate/txqueuelen, then up.
    /// All motors on this bus are set to Disabled (user must re-enable).
    fn recover_link(&mut self, bus_name: &str) -> bool {
        let interface_name = match self.can_interfaces.get(bus_name) {
            Some(name) => name.clone(),
            None => return false,
        };

        warn!("CAN {} Level 1 recovery: ip link down/up (resetting USB-CAN echo IDs)", bus_name);

        // Close socket first
        self.can_sockets.remove(bus_name);
        std::thread::sleep(Duration::from_millis(50));

        // ip link set down — calls ndo_stop() → usb_kill_anchored_urbs()
        let down = Command::new("sudo")
            .args(["ip", "link", "set", &interface_name, "down"])
            .output();
        match down {
            Ok(output) if output.status.success() => {
                info!("CAN {} interface {} brought down", bus_name, interface_name);
            }
            Ok(output) => {
                error!("Failed to bring down {}: {}", interface_name,
                       String::from_utf8_lossy(&output.stderr));
                return false;
            }
            Err(e) => {
                error!("Failed to run ip link set {} down: {}", interface_name, e);
                return false;
            }
        }

        std::thread::sleep(Duration::from_millis(200));

        // Reconfigure bitrate
        let config_type = Command::new("sudo")
            .args(["ip", "link", "set", &interface_name, "type", "can",
                   "bitrate", "1000000", "restart-ms", "100"])
            .output();
        if let Ok(output) = config_type {
            if !output.status.success() {
                warn!("CAN {} bitrate config warning: {}", bus_name,
                      String::from_utf8_lossy(&output.stderr));
            }
        }

        // Set txqueuelen
        let txq = Command::new("sudo")
            .args(["ip", "link", "set", &interface_name, "txqueuelen", "128"])
            .output();
        if let Ok(output) = txq {
            if !output.status.success() {
                warn!("CAN {} txqueuelen config warning: {}", bus_name,
                      String::from_utf8_lossy(&output.stderr));
            }
        }

        // ip link set up — calls ndo_open() → reinitializes TX context pool
        let up = Command::new("sudo")
            .args(["ip", "link", "set", &interface_name, "up"])
            .output();
        match up {
            Ok(output) if output.status.success() => {
                info!("CAN {} interface {} brought up", bus_name, interface_name);
            }
            Ok(output) => {
                error!("Failed to bring up {}: {}", interface_name,
                       String::from_utf8_lossy(&output.stderr));
                return false;
            }
            Err(e) => {
                error!("Failed to run ip link set {} up: {}", interface_name, e);
                return false;
            }
        }

        std::thread::sleep(Duration::from_millis(200));

        // Reopen socket
        match CanSocket::open(&interface_name) {
            Ok(socket) => {
                if let Err(e) = socket.set_nonblocking(true) {
                    error!("Failed to set recovered socket {} to non-blocking: {}", bus_name, e);
                    return false;
                }
                let _ = socket.set_loopback(false);
                self.can_sockets.insert(bus_name.to_string(), socket);
                self.consecutive_tx_errors.borrow_mut().insert(bus_name.to_string(), 0);

                // Set all motors on this bus to Disabled — user must re-enable
                for motor in self.base_group.motors.iter_mut()
                    .chain(self.arm_group.motors.iter_mut())
                {
                    if motor.config.can_bus == bus_name {
                        motor.state = MotorState::Disabled;
                        motor.feedback = None;
                    }
                }

                info!("CAN {} Level 1 recovery complete — all motors on bus set to Disabled", bus_name);
                true
            }
            Err(e) => {
                error!("Failed to reopen CAN socket {} after link recovery: {}", bus_name, e);
                false
            }
        }
    }

    /// Escalating CAN recovery for stuck TX errors.
    ///
    /// Level 0 — Socket reopen (transient issues): 2 attempts, 10s cooldown.
    /// Level 1 — ip link down/up (gs_usb echo corruption): 2 attempts, 30s cooldown.
    /// Level 2 — Exit (USB-level corruption): process exits, systemd restarts with USB reset.
    ///
    /// KEY INSIGHT: If we reach this function again after a "successful" recovery, it means
    /// the fix didn't actually work (ENOBUFS resumed). So attempts always increment —
    /// success/fail of the operation itself doesn't matter for escalation.
    fn recover_can_if_needed(&mut self) {
        const RECOVERY_THRESHOLD: u32 = 50;
        const SOCKET_COOLDOWN: Duration = Duration::from_secs(10);
        const LINK_COOLDOWN: Duration = Duration::from_secs(30);
        const MAX_SOCKET_ATTEMPTS: u32 = 2;
        const MAX_LINK_ATTEMPTS: u32 = 2;

        let buses_to_recover: Vec<String> = {
            let errors = self.consecutive_tx_errors.borrow();
            errors.iter()
                .filter(|(_, &count)| count >= RECOVERY_THRESHOLD)
                .map(|(bus, _)| bus.clone())
                .collect()
        };

        for bus_name in buses_to_recover {
            let level = self.can_recovery_level.get(&bus_name).copied().unwrap_or(0);
            let attempts = self.can_recovery_attempts.get(&bus_name).copied().unwrap_or(0);

            // Level 2+ = exit immediately
            if level >= 2 {
                error!("CAN {} recovery exhausted (level={}) — exiting for systemd restart with USB reset",
                       bus_name, level);
                std::process::exit(1);
            }

            // Check cooldown for current level
            match level {
                0 => {
                    if self.last_socket_recovery.elapsed() < SOCKET_COOLDOWN {
                        continue;
                    }
                }
                1 => {
                    if self.last_link_recovery.elapsed() < LINK_COOLDOWN {
                        continue;
                    }
                }
                _ => {}
            }

            // Check if current level is exhausted → escalate BEFORE attempting
            match level {
                0 if attempts >= MAX_SOCKET_ATTEMPTS => {
                    warn!("CAN {} Level 0 exhausted ({} attempts), escalating to Level 1 (ip link down/up)",
                          bus_name, attempts);
                    self.can_recovery_level.insert(bus_name.clone(), 1);
                    self.can_recovery_attempts.insert(bus_name.clone(), 0);
                    // Fall through to try Level 1 immediately (if cooldown allows)
                    if self.last_link_recovery.elapsed() < LINK_COOLDOWN {
                        continue;
                    }
                    // Try Level 1 now
                    let success = self.recover_link(&bus_name);
                    self.last_link_recovery = Instant::now();
                    self.can_recovery_attempts.insert(bus_name.clone(), 1);
                    if success {
                        info!("CAN {} Level 1 recovery succeeded on first try", bus_name);
                    }
                }
                1 if attempts >= MAX_LINK_ATTEMPTS => {
                    warn!("CAN {} Level 1 exhausted ({} attempts), escalating to Level 2 (exit)",
                          bus_name, attempts);
                    self.can_recovery_level.insert(bus_name.clone(), 2);
                    error!("CAN {} recovery exhausted — exiting for systemd restart with USB reset", bus_name);
                    std::process::exit(1);
                }
                0 => {
                    // Level 0: socket reopen
                    self.recover_socket(&bus_name);
                    self.last_socket_recovery = Instant::now();
                    // Always increment — if we're back here, it didn't really work
                    let new_attempts = attempts + 1;
                    self.can_recovery_attempts.insert(bus_name.clone(), new_attempts);
                    warn!("CAN {} Level 0 attempt {}/{}", bus_name, new_attempts, MAX_SOCKET_ATTEMPTS);
                }
                1 => {
                    // Level 1: ip link down/up
                    self.recover_link(&bus_name);
                    self.last_link_recovery = Instant::now();
                    let new_attempts = attempts + 1;
                    self.can_recovery_attempts.insert(bus_name.clone(), new_attempts);
                    warn!("CAN {} Level 1 attempt {}/{}", bus_name, new_attempts, MAX_LINK_ATTEMPTS);
                }
                _ => {}
            }
        }
    }

    /// Look up CAN ID and model for a motor by name
    fn can_id_and_model(&self, motor_id: &str) -> Option<(u8, MotorModel)> {
        for motor in &self.base_group.motors {
            if motor.config.id == motor_id {
                return Some((motor.config.can_id, motor.config.model));
            }
        }
        for motor in &self.arm_group.motors {
            if motor.config.id == motor_id {
                return Some((motor.config.can_id, motor.config.model));
            }
        }
        None
    }

    /// Get CAN socket for a motor by ID
    fn get_can_socket(&self, motor_id: &str) -> Option<&CanSocket> {
        let bus = self.motor_bus_map.get(motor_id)?;
        self.can_sockets.get(bus)
    }

    /// Get CAN socket for a motor's bus name
    fn get_can_socket_by_bus(&self, bus: &str) -> Option<&CanSocket> {
        self.can_sockets.get(bus)
    }

    /// Send zero-force keepalive frames to all enabled/running base motors
    fn send_keepalives(&self) {
        for motor in &self.base_group.motors {
            if motor.state == MotorState::Enabled || motor.state == MotorState::Running {
                if let Some(socket) = self.get_can_socket_by_bus(&motor.config.can_bus) {
                    let pos = motor.feedback.as_ref().map(|f| f.position).unwrap_or(0.0);
                    let keepalive = robstride::build_control_frame(
                        motor.config.can_id, motor.config.model, pos, 0.0, 0.0, 0.0, 0.0,
                    );
                    self.tracked_write_frame(socket, &keepalive, &motor.config.can_bus);
                }
            }
        }
        // Arm motors also need keepalives during the blocking enable sequence —
        // the hardware watchdog (~100ms) fires if arm motors get no CAN frames
        // while later motors in the enable list are timing out (up to ~300ms each
        // for unconnected motors exhausting their 3 retries).
        for motor in &self.arm_group.motors {
            if motor.state == MotorState::Enabled || motor.state == MotorState::Running {
                if let Some(socket) = self.get_can_socket_by_bus(&motor.config.can_bus) {
                    let pos = motor.feedback.as_ref().map(|f| f.position).unwrap_or(0.0);
                    let keepalive = robstride::build_control_frame(
                        motor.config.can_id, motor.config.model, pos, 0.0, 0.0, 0.0, 0.0,
                    );
                    self.tracked_write_frame(socket, &keepalive, &motor.config.can_bus);
                }
            }
        }
    }

    /// Sleep while sending keepalives to active motors every 5ms
    fn sleep_with_keepalives(&self, duration_ms: u64) {
        let start = Instant::now();
        let target = Duration::from_millis(duration_ms);
        while start.elapsed() < target {
            self.send_keepalives();
            std::thread::sleep(std::time::Duration::from_millis(5));
        }
    }

    fn enable_motor(&mut self, motor_id: &str) -> Result<()> {
        if let Some((can_id, model)) = self.can_id_and_model(motor_id) {
            // Get CAN socket and bus name for this motor
            let socket = self.get_can_socket(motor_id)
                .ok_or_else(|| anyhow::anyhow!("CAN bus not found for motor {}", motor_id))?;
            let bus_name = self.motor_bus_map.get(motor_id)
                .ok_or_else(|| anyhow::anyhow!("Bus name not found for motor {}", motor_id))?
                .clone(); // Clone to avoid borrow conflict

            // First send disable to clear any existing fault state
            let disable_frame = robstride::build_disable_frame(can_id);
            self.safe_write_frame(socket, &disable_frame, &bus_name)?;
            self.sleep_with_keepalives(50);
            info!("Sent disable (fault clear) for motor {}", motor_id);

            // Drain any pending CAN frames
            loop {
                match socket.read_frame() {
                    Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => break,
                    _ => continue,
                }
            }

            // Send enable command with retries
            let mut enabled = false;
            let mut confirmed_position = 0.0f32;
            for attempt in 1..=3 {
                let frame = robstride::build_enable_frame(can_id);
                self.safe_write_frame(socket, &frame, &bus_name)?;
                info!("Sent enable for motor {} (attempt {})", motor_id, attempt);

                // Brief wait for motor to process enable — NO keepalives here!
                // Keepalives flood the bus with responses from enabled motors,
                // preventing us from seeing the enable response from the target.
                // Enabled motors survive ~100ms without keepalives (hardware watchdog).
                std::thread::sleep(Duration::from_millis(2));

                // Drain stale frames before polling for the enable response
                loop {
                    match socket.read_frame() {
                        Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => break,
                        _ => continue,
                    }
                }

                // Re-send enable in case the first was lost during bus contention
                let frame = robstride::build_enable_frame(can_id);
                self.safe_write_frame(socket, &frame, &bus_name)?;

                // Poll for response — clean bus, no keepalive noise
                let mut got_run_mode = false;
                let poll_start = Instant::now();
                while poll_start.elapsed() < Duration::from_millis(50) && !got_run_mode {
                    match socket.read_frame() {
                        Ok(frame) => {
                            let arb_id_raw = match frame.id() {
                                socketcan::Id::Extended(id) => id.as_raw(),
                                _ => continue,
                            };
                            let msg_type = ((arb_id_raw >> 24) & 0x1F) as u8;
                            let resp_motor_id = ((arb_id_raw >> 8) & 0xFF) as u8;
                            let mode_bits = ((arb_id_raw >> 22) & 0x03) as u8;

                            // Log all feedback frames during enable poll for debugging
                            if msg_type == 2 {
                                info!("  enable poll: motor=0x{:02X} mode={} (want 0x{:02X})",
                                      resp_motor_id, mode_bits, can_id);
                            }

                            if msg_type != 2 { continue; }
                            if resp_motor_id != can_id { continue; }

                            let error_bits = ((arb_id_raw >> 16) & 0x1F) as u8;

                            // Parse position from feedback to use for keepalive
                            let data = frame.data();
                            if data.len() >= 2 {
                                let angle_raw = u16::from_be_bytes([data[0], data[1]]);
                                confirmed_position = (angle_raw as f32 / 65535.0) * (8.0 * std::f32::consts::PI) - (4.0 * std::f32::consts::PI);
                            }

                            info!("  Enable response: motor=0x{:02X}, mode={}, errors=0x{:02X}, pos={:.3}",
                                  resp_motor_id, mode_bits, error_bits, confirmed_position);

                            if mode_bits == 2 {
                                got_run_mode = true;
                            }
                        }
                        Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                            std::thread::sleep(Duration::from_millis(1));
                        }
                        Err(e) => return Err(e.into()),
                    }
                }

                if got_run_mode {
                    info!("Motor {} entered Run mode at pos={:.3}", motor_id, confirmed_position);

                    // Send zero-force keepalives to stabilize the motor.
                    // IMPORTANT: Do NOT apply Kd during enable keepalives — it causes
                    // motor faults due to interaction with the motor's internal enable
                    // transition. Kd braking is applied in the main loop Enabled state
                    // (~200ms later) which is safe.
                    //
                    // Initialize still_running = true: we just confirmed Run mode entry
                    // and are sending valid zero-force control frames. We'll update it
                    // if we see a frame indicating the motor left Run mode.
                    let mut still_running = true;
                    for _ in 0..50 {
                        let keepalive = robstride::build_control_frame(can_id, model, confirmed_position, 0.0, 0.0, 0.0, 0.0);
                        self.tracked_write_frame(socket, &keepalive, &bus_name);
                        self.send_keepalives(); // keep other motors alive too
                        std::thread::sleep(std::time::Duration::from_millis(2));
                        // Drain receive buffer each iteration to prevent overflow from
                        // other motors' feedback responses, and track whether the target
                        // motor stays in Run mode.
                        loop {
                            match socket.read_frame() {
                                Ok(frame) => {
                                            let arb_id_raw = match frame.id() {
                                        socketcan::Id::Extended(id) => id.as_raw(),
                                        _ => continue,
                                    };
                                    if ((arb_id_raw >> 24) & 0x1F) as u8 != 2 { continue; }
                                    let resp_motor_id = ((arb_id_raw >> 8) & 0xFF) as u8;
                                    if resp_motor_id != can_id { continue; }
                                    let mode_bits = ((arb_id_raw >> 22) & 0x03) as u8;
                                    let data = frame.data();
                                    if data.len() >= 2 {
                                        let angle_raw = u16::from_be_bytes([data[0], data[1]]);
                                        confirmed_position = (angle_raw as f32 / 65535.0) * (8.0 * std::f32::consts::PI) - (4.0 * std::f32::consts::PI);
                                    }
                                    still_running = mode_bits == 2;
                                }
                                Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => break,
                                Err(_) => break,
                            }
                        }
                    }

                    // Final drain for any frames that arrived after the last iteration
                    loop {
                        match socket.read_frame() {
                            Ok(frame) => {
                                let arb_id_raw = match frame.id() {
                                    socketcan::Id::Extended(id) => id.as_raw(),
                                    _ => continue,
                                };
                                if ((arb_id_raw >> 24) & 0x1F) as u8 != 2 { continue; }
                                let resp_motor_id = ((arb_id_raw >> 8) & 0xFF) as u8;
                                if resp_motor_id != can_id { continue; }
                                let mode_bits = ((arb_id_raw >> 22) & 0x03) as u8;
                                let data = frame.data();
                                if data.len() >= 2 {
                                    let angle_raw = u16::from_be_bytes([data[0], data[1]]);
                                    confirmed_position = (angle_raw as f32 / 65535.0) * (8.0 * std::f32::consts::PI) - (4.0 * std::f32::consts::PI);
                                }
                                still_running = mode_bits == 2;
                            }
                            Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => break,
                            Err(_) => break,
                        }
                    }

                    if still_running {
                        info!("Motor {} confirmed stable in Run mode at pos={:.3}", motor_id, confirmed_position);
                        enabled = true;
                        break;
                    } else {
                        warn!("Motor {} faulted after enable (attempt {}), retrying...", motor_id, attempt);
                        self.tracked_write_frame(socket, &disable_frame, &bus_name);
                        self.sleep_with_keepalives(100);
                    }
                } else {
                    warn!("Motor {} did NOT enter Run mode on attempt {}", motor_id, attempt);
                    self.tracked_write_frame(socket, &disable_frame, &bus_name);
                    self.sleep_with_keepalives(50);
                }
            }

            if !enabled {
                warn!("Motor {} failed to stay in Run mode after 3 attempts — may need power cycle", motor_id);
            }

            if let Some(motor) = self.find_motor_mut(motor_id) {
                motor.state = if enabled { MotorState::Enabled } else { MotorState::Error };
                motor.last_command_time = Instant::now();
                // Reset feedback timestamp — enable_motor reads CAN frames directly
                // without calling update_feedback(), so last_feedback_time goes stale
                // during the blocking enable sequence (~280ms per motor). Without this
                // reset, check_safety() trips the 500ms feedback timeout immediately.
                motor.last_feedback_time = Instant::now();
                motor.consecutive_errors = 0;
                // Use the ACTUAL confirmed position from the enable response
                // This prevents stale feedback from overwriting with the old position
                motor.target_position = confirmed_position;
                motor.target_velocity = 0.0;
                motor.position_initialized = true; // Prevent re-init from stale feedback
                // Populate synthetic feedback so this motor appears in telemetry immediately.
                // enable_motor() reads CAN frames directly without calling update_feedback(),
                // leaving motor.feedback = None. publish_telemetry() skips motors with no
                // feedback, so the motor is invisible to teleop. For the swivel this creates
                // a circular dependency: teleop only sends Swivel commands after seeing the
                // motor as "enabled" in telemetry, but it never appears without feedback.
                if enabled {
                    motor.feedback = Some(robstride::MotorFeedback {
                        motor_id: can_id,
                        mode: robstride::MotorMode::Run,
                        position: confirmed_position,
                        velocity: 0.0,
                        torque: 0.0,
                        temperature: 0.0,
                        error: robstride::MotorError::from_bits(0),
                        timestamp: std::time::Instant::now(),
                    });
                }
                info!("Motor {} state set to {:?}, target_pos={:.3}", motor_id, motor.state, confirmed_position);
            }
            Ok(())
        } else {
            Err(anyhow::anyhow!("Motor {} not found", motor_id))
        }
    }

    fn disable_motor(&mut self, motor_id: &str) -> Result<()> {
        if let Some(can_id) = self.motor_id_map.get(motor_id) {
            let socket = self.get_can_socket(motor_id)
                .ok_or_else(|| anyhow::anyhow!("CAN bus not found for motor {}", motor_id))?;
            let bus_name = self.motor_bus_map.get(motor_id)
                .ok_or_else(|| anyhow::anyhow!("Bus name not found for motor {}", motor_id))?
                .clone(); // Clone to avoid borrow conflict
            let frame = robstride::build_disable_frame(*can_id);
            self.safe_write_frame(socket, &frame, &bus_name)?;
            if let Some(motor) = self.find_motor_mut(motor_id) {
                motor.state = MotorState::Disabled;
                motor.last_command_time = Instant::now(); // Update command time
                info!("Disabled motor {}", motor_id);
            }
            Ok(())
        } else {
            Err(anyhow::anyhow!("Motor {} not found", motor_id))
        }
    }

    fn handle_command(&mut self, cmd: CommandMessage) -> Result<()> {
        debug!("Handling command: {:?}", cmd);

        // Allow ClearFault, ClearEmergencyStop, and TriggerFault through even during e-stop
        if self.emergency_stop {
            match &cmd {
                CommandMessage::ClearEmergencyStop
                | CommandMessage::ClearFault { .. }
                | CommandMessage::TriggerFault { .. } => {}
                _ => {
                    warn!("Emergency stop active, ignoring command");
                    return Ok(());
                }
            }
        }

        match cmd {
            CommandMessage::Drive { linear, angular, kp, kd } => {
                // Differential drive for two wheels only (swivel handled separately)
                // Negate both to match controller direction
                let linear = -linear;
                let angular = -angular;
                let left_vel = linear - angular;
                let right_vel = linear + angular;

                // Default gains if not provided (backwards compatibility)
                let drive_kp = if kp == 0.0 && kd == 0.0 { 0.0 } else { kp };
                let drive_kd = if kp == 0.0 && kd == 0.0 { 3.0 } else { kd };

                if let Some(left_motor) = self.base_group.motors.first_mut() {
                    info!("Setting {} velocity to {:.3} rad/s (Kp={:.1}, Kd={:.1})",
                          left_motor.config.id, left_vel, drive_kp, drive_kd);
                    left_motor.set_velocity_target(left_vel)?;
                    left_motor.kp = drive_kp;
                    left_motor.kd = drive_kd;
                }
                if let Some(right_motor) = self.base_group.motors.get_mut(1) {
                    info!("Setting {} velocity to {:.3} rad/s (Kp={:.1}, Kd={:.1})",
                          right_motor.config.id, right_vel, drive_kp, drive_kd);
                    right_motor.set_velocity_target(right_vel)?;
                    right_motor.kp = drive_kp;
                    right_motor.kd = drive_kd;
                }
            }
            CommandMessage::ArmJoints {
                positions,
                velocities,
                kp,
                kd,
                torques,
            } => {
                let num_arm_motors = self.arm_group.motors.len();
                if positions.len() < num_arm_motors {
                    return Err(anyhow::anyhow!(
                        "Expected at least {} positions, got {}",
                        num_arm_motors,
                        positions.len()
                    ));
                }

                for (i, motor) in self.arm_group.motors.iter_mut().enumerate() {
                    let vel = velocities.get(i).copied().unwrap_or(0.0);
                    let kp_val = kp.get(i).copied().unwrap_or(30.0);
                    let kd_val = kd.get(i).copied().unwrap_or(1.0);
                    let torque_ff = torques.get(i).copied().unwrap_or(0.0);
                    if let Err(e) = motor.set_position_target(positions[i], vel, kp_val, kd_val, torque_ff) {
                        // Soft-limit violation: hold the last valid target but refresh
                        // last_command_time so the watchdog doesn't fire for the whole
                        // arm group just because one joint hit its limit.
                        warn!("Motor {} rejected pos {:.3}: {} — holding last target",
                              motor.config.id, positions[i], e);
                        motor.last_command_time = std::time::Instant::now();
                    }
                }
            }
            CommandMessage::Bundle { arm_joints, drive } => {
                // Drive first (lowest stakes), then arm.  Each sub-payload
                // re-uses the existing variant handlers so any future change
                // to Drive/ArmJoints semantics carries through automatically.
                if let Some(DrivePayload { linear, angular, kp, kd }) = drive {
                    self.handle_command(CommandMessage::Drive { linear, angular, kp, kd })?;
                }
                if let Some(ArmJointsPayload {
                    positions, velocities, kp, kd, torques,
                }) = arm_joints {
                    self.handle_command(CommandMessage::ArmJoints {
                        positions, velocities, kp, kd, torques,
                    })?;
                }
            }
            CommandMessage::Enable { motor_ids } => {
                let mut enabled_motors: Vec<(String, u8, MotorModel)> = Vec::new();
                for motor_id in &motor_ids {
                    // Skip motors that are already enabled or running — re-enabling a
                    // running motor blocks the main loop for ~170ms per motor, starving
                    // the command timeout watchdog and triggering emergency_stop_all().
                    let already_active = self.find_motor_mut(motor_id)
                        .map(|m| matches!(m.state, MotorState::Enabled | MotorState::Running))
                        .unwrap_or(false);
                    if already_active {
                        info!("Motor {} already active, skipping enable", motor_id);
                        if let Some((cid, cmodel)) = self.can_id_and_model(motor_id) {
                            enabled_motors.push((motor_id.clone(), cid, cmodel));
                        }
                        continue;
                    }

                    // Send keepalives to already-enabled motors before enabling next one
                    for (mid, cid, cmodel) in &enabled_motors {
                        let mid_bus = self.motor_bus_map.get(mid).cloned().unwrap_or_default();
                        if let Some(socket) = self.get_can_socket(mid) {
                            let keepalive = robstride::build_control_frame(*cid, *cmodel, 0.0, 0.0, 0.0, 0.0, 0.0);
                            self.tracked_write_frame(socket, &keepalive, &mid_bus);
                        }
                    }
                    // Outer retry loop: attempt enable up to 3 times, each with its own
                    // 3-attempt internal retry, giving up to 9 total enable attempts.
                    // IMPORTANT: Don't propagate errors — a CAN TX error on one motor
                    // must not abort the entire enable sequence or skip the post-enable
                    // feedback timestamp refresh (which would cause a timeout death spiral).
                    for outer_attempt in 1..=3 {
                        if let Err(e) = self.enable_motor(motor_id) {
                            warn!("Motor {} enable error (attempt {}): {} — continuing", motor_id, outer_attempt, e);
                            break; // Skip further retries for this motor, try next one
                        }
                        let is_enabled = self.find_motor_mut(motor_id)
                            .map(|m| m.state == MotorState::Enabled)
                            .unwrap_or(false);
                        if is_enabled {
                            break;
                        }
                        if outer_attempt < 3 {
                            warn!("Motor {} outer enable attempt {} failed, retrying...", motor_id, outer_attempt);
                        }
                    }
                    if let Some((cid, cmodel)) = self.can_id_and_model(motor_id) {
                        enabled_motors.push((motor_id.clone(), cid, cmodel));
                    }
                }

                // Refresh feedback timestamps for all just-enabled motors.
                //
                // The sequential enable takes ~165ms per motor, so after enabling
                // all 9 motors the first motor's last_feedback_time is ~1320ms stale.
                // check_safety() uses a 500ms feedback timeout, so it trips on the
                // first 7 motors and sets them back to Error — even though they are
                // fine. This happens because our stabilization drain loop discards
                // non-target-motor frames without calling update_feedback(), so
                // last_feedback_time never advances during subsequent enables.
                //
                // Resetting here is equivalent to what enable_motor() does
                // individually; it just extends the grace period to cover the
                // full batch enable duration rather than a single motor's.
                let now = Instant::now();
                for motor in self.base_group.motors.iter_mut()
                    .chain(self.arm_group.motors.iter_mut())
                {
                    if matches!(motor.state, MotorState::Enabled | MotorState::Running) {
                        motor.last_feedback_time = now;
                    }
                }
            }
            CommandMessage::Disable { motor_ids } => {
                // Don't bail on the first unknown motor — teleop sends a
                // canonical list that may include motors absent from the
                // active config (e.g. wheels after 2026-05-30 arm-only
                // separation).  Log and continue so the real motors still
                // get disabled.
                for motor_id in motor_ids {
                    if !self.motor_id_map.contains_key(&motor_id) {
                        debug!("Disable: skipping unknown motor '{}' (not in config)", motor_id);
                        continue;
                    }
                    if let Err(e) = self.disable_motor(&motor_id) {
                        warn!("Disable {} failed: {} (continuing with remaining motors)", motor_id, e);
                    }
                }
            }
            CommandMessage::EmergencyStop => {
                info!("EMERGENCY STOP");
                self.emergency_stop = true;
                self.base_group.emergency_stop_all();
                self.arm_group.emergency_stop_all();
            }
            CommandMessage::ZeroPosition { motor_ids } => {
                // Software-only zero: set target to current feedback position.
                // Do NOT send CAN zero_pos — it causes a massive torque spike
                // (47-85 Nm on RS04) that can re-accelerate or fault the motor.
                for motor_id in &motor_ids {
                    if let Some(motor) = self.find_motor_mut(motor_id) {
                        let current_pos = motor.feedback.as_ref()
                            .map(|f| f.position).unwrap_or(motor.target_position);
                        motor.target_position = current_pos;
                        motor.target_velocity = 0.0;
                        info!("Software zero for {}: target_pos set to fb_pos={:.3}", motor_id, current_pos);
                    }
                }
            }
            CommandMessage::MechZero { motor_ids, save } => {
                // Hardware mechanical zero per Robstride protocol (msg type 6).
                // Refuses unless every named motor is currently disabled — the
                // ZeroPos command is documented to produce a torque spike on a
                // running motor, and silently rewriting the zero under load is
                // unsafe.
                let mut blocked: Vec<String> = Vec::new();
                for motor_id in &motor_ids {
                    match self.find_motor_mut(motor_id) {
                        Some(motor) if motor.state != MotorState::Idle
                            && motor.state != MotorState::Disabled => {
                            blocked.push(format!("{}={}", motor_id, motor.state.as_str()));
                        }
                        None => blocked.push(format!("{}=unknown", motor_id)),
                        _ => {}
                    }
                }
                if !blocked.is_empty() {
                    warn!("MechZero refused — disable motors first. Active: {}", blocked.join(", "));
                } else {
                    info!("MechZero: writing hardware zero to {} motor(s){}",
                          motor_ids.len(),
                          if save { " (will SaveConfig)" } else { "" });
                    for motor_id in &motor_ids {
                        let can_id = match self.find_motor_mut(motor_id).map(|m| m.config.can_id) {
                            Some(id) => id,
                            None => continue,
                        };
                        let bus_name = match self.motor_bus_map.get(motor_id).cloned() {
                            Some(b) => b,
                            None => {
                                warn!("MechZero: no bus mapping for {}", motor_id);
                                continue;
                            }
                        };
                        let pos_before = self
                            .find_motor_mut(motor_id)
                            .and_then(|m| m.feedback.as_ref().map(|f| f.position));
                        let socket = match self.can_sockets.get(&bus_name) {
                            Some(s) => s,
                            None => {
                                warn!("MechZero: no CAN socket for bus {}", bus_name);
                                continue;
                            }
                        };
                        let zero_frame = robstride::build_zero_pos_frame(can_id);
                        if let Err(e) = self.safe_write_frame(socket, &zero_frame, &bus_name) {
                            error!("MechZero: write ZeroPos to {} failed: {}", motor_id, e);
                            continue;
                        }
                        std::thread::sleep(Duration::from_millis(20));
                        if save {
                            let save_frame = robstride::build_save_config_frame(can_id);
                            if let Err(e) = self.safe_write_frame(socket, &save_frame, &bus_name) {
                                error!("MechZero: write SaveConfig to {} failed: {}", motor_id, e);
                                continue;
                            }
                            std::thread::sleep(Duration::from_millis(10));
                        }
                        info!("MechZero {}: ok (pos_before={:?}){}",
                              motor_id, pos_before,
                              if save { " saved" } else { "" });
                    }
                }
            }
            CommandMessage::ClearFault { motor_ids } => {
                for motor_id in &motor_ids {
                    if let Some(motor) = self.find_motor_mut(motor_id) {
                        if motor.state != MotorState::Error {
                            info!("Motor {} not in Error state, skipping clear_fault", motor_id);
                            continue;
                        }
                    }
                    info!("Attempting fault recovery for motor {}", motor_id);
                    match self.enable_motor(motor_id) {
                        Ok(()) => {
                            if let Some(motor) = self.find_motor_mut(motor_id) {
                                // Reset feedback timestamp since enable_motor reads
                                // CAN frames internally without calling update_feedback
                                motor.last_feedback_time = Instant::now();
                                if motor.state == MotorState::Enabled {
                                    info!("Motor {} fault cleared successfully", motor_id);
                                    motor.fault_info = None;
                                } else {
                                    // enable_motor already set state to Error
                                    if let Some(ref mut fi) = motor.fault_info {
                                        fi.recovery_attempts += 1;
                                        warn!("Motor {} fault recovery failed (attempt {})",
                                              motor_id, fi.recovery_attempts);
                                    }
                                }
                            }
                        }
                        Err(e) => {
                            warn!("Motor {} fault recovery error: {}", motor_id, e);
                            if let Some(motor) = self.find_motor_mut(motor_id) {
                                if let Some(ref mut fi) = motor.fault_info {
                                    fi.recovery_attempts += 1;
                                }
                            }
                        }
                    }
                }
            }
            CommandMessage::TriggerFault { motor_ids } => {
                for motor_id in &motor_ids {
                    if let Some(motor) = self.find_motor_mut(motor_id) {
                        warn!("Triggering simulated fault on motor {}", motor_id);
                        motor.state = MotorState::Error;
                        motor.fault_info = Some(motor::FaultInfo {
                            error: robstride::MotorError::from_bits(0),
                            mode_at_fault: robstride::MotorMode::Run,
                            recovery_attempts: 0,
                        });
                    } else {
                        warn!("Motor {} not found for trigger_fault", motor_id);
                    }
                }
            }
            CommandMessage::ClearEmergencyStop => {
                if self.emergency_stop {
                    info!("Clearing emergency stop");
                    self.emergency_stop = false;
                } else {
                    info!("Emergency stop was not active");
                }
            }
        }
        Ok(())
    }

    /// Send control commands to arm motors only (called at arm_frequency, e.g. 1kHz)
    fn send_arm_commands(&mut self) -> Result<()> {
        for motor in &self.arm_group.motors {
            if motor.state == MotorState::Running {
                let bus_name = motor.config.can_bus.clone();
                if let Some(socket) = self.get_can_socket_by_bus(&bus_name) {
                    let frame = robstride::build_control_frame(
                        motor.config.can_id,
                        motor.config.model,
                        motor.target_position,
                        motor.target_velocity,
                        motor.kp,
                        motor.kd,
                        motor.target_torque,
                    );
                    self.safe_write_frame(socket, &frame, &bus_name)?;
                }
            } else if motor.state == MotorState::Enabled {
                // Send zero-force position-hold frame to prevent hardware watchdog from firing
                // while waiting for the first ArmJoints command after enable.
                let bus_name = motor.config.can_bus.clone();
                if let Some(socket) = self.get_can_socket_by_bus(&bus_name) {
                    let pos = motor.feedback.as_ref().map(|f| f.position).unwrap_or(motor.target_position);
                    let frame = robstride::build_control_frame(
                        motor.config.can_id, motor.config.model, pos, 0.0, 0.0, 0.0, 0.0,
                    );
                    self.tracked_write_frame(socket, &frame, &bus_name);
                }
            }
        }
        Ok(())
    }

    /// Send control commands to base motors only (called at base_frequency, e.g. 100Hz).
    /// Drive wheels only — swivel moved to the arm group.
    fn send_base_commands(&mut self) -> Result<()> {
        self.control_tick += 1;

        for motor in self.base_group.motors.iter() {
            if motor.state == MotorState::Enabled || motor.state == MotorState::Running {
                let bus_name = motor.config.can_bus.clone();
                if let Some(socket) = self.get_can_socket_by_bus(&bus_name) {
                    let fb_pos = motor.feedback.as_ref().map(|f| f.position).unwrap_or(0.0);

                    // Drive wheels: velocity control.  Kp=0 for pure velocity
                    // control (avoids position wrapping at ±4π).  ROBSTRIDE
                    // control law: torque = Kp*(pos_err) + Kd*(vel_err) + ff.
                    let frame = robstride::build_control_frame(
                        motor.config.can_id,
                        motor.config.model,
                        fb_pos,                  // Echo feedback position (ignored with Kp=0)
                        motor.target_velocity,   // Velocity target
                        motor.kp,                // Tunable position gain (default 0.0)
                        motor.kd,                // Tunable velocity tracking gain (default 3.0)
                        0.0,                     // No feedforward torque
                    );

                    self.safe_write_frame(socket, &frame, &bus_name)?;

                    if self.control_tick % 50 == 0 && motor.state == MotorState::Running {
                        info!(
                            "  RUN {}: tgt_vel={:.3} fb_vel={:.3} fb_pos={:.3} Kp={:.1} Kd={:.1}",
                            motor.config.id, motor.target_velocity,
                            motor.feedback.as_ref().map(|f| f.velocity).unwrap_or(-999.0),
                            fb_pos, motor.kp, motor.kd,
                        );
                    }
                }
            }
        }

        // NOTE: No auto-transition from Enabled → Running here. Motors transition
        // to Running when they receive an actual command (Drive/ArmJoints).
        // Auto-transitioning caused a watchdog death spiral historically when
        // swivel lived in this group: it entered Running but no commands were
        // flowing, so 100 ms later the command timeout fired emergency_stop_all().
        // Enabled motors already receive zero-force keepalive frames above.

        // Request battery voltage (VBUS) periodically (10Hz = every 100ms).
        // Round-robin across ALL motors (base + arm) so the published bus
        // voltage is an average rather than a single-motor reading.  Used to
        // query only base_group.first(), which silently broke when the wheels
        // were removed (2026-05-30) and left base_group empty/swivel-only.
        if self.last_vbus_request.elapsed() >= Duration::from_millis(100) {
            let total = self.base_group.motors.len() + self.arm_group.motors.len();
            if total > 0 {
                let idx = self.vbus_query_idx % total;
                let (bus_name, can_id, motor_id) = if idx < self.base_group.motors.len() {
                    let m = &self.base_group.motors[idx];
                    (m.config.can_bus.clone(), m.config.can_id, m.config.id.clone())
                } else {
                    let m = &self.arm_group.motors[idx - self.base_group.motors.len()];
                    (m.config.can_bus.clone(), m.config.can_id, m.config.id.clone())
                };
                if let Some(socket) = self.get_can_socket_by_bus(&bus_name) {
                    let frame = robstride::build_read_param_frame(can_id, robstride::params::VBUS);
                    self.safe_write_frame(socket, &frame, &bus_name)?;
                    self.last_vbus_request = Instant::now();
                    self.vbus_query_idx = self.vbus_query_idx.wrapping_add(1);
                    debug!("Requested VBUS from motor {} (CAN ID 0x{:02X}) [round-robin {}/{}]",
                           motor_id, can_id, idx, total);
                }
            }
        }

        Ok(())
    }

    fn read_feedback(&mut self) -> Result<()> {
        // Read all available CAN frames from all buses
        let mut frame_count = 0;

        for (_bus_name, socket) in &self.can_sockets {
            loop {
                match socket.read_frame() {
                Ok(frame) => {
                    frame_count += 1;

                    // Extract motor ID from arbitration ID
                    // Response format: host_id (bits 0-7) | motor_id (bits 8-15) | msg_type (bits 24-31)
                    let arb_id_raw = match frame.id() {
                        socketcan::Id::Extended(id) => id.as_raw(),
                        _ => {
                            debug!("Skipping standard frame");
                            continue;
                        }
                    };
                    // Process Feedback frames (msg_type == 2) and ReadParam responses (msg_type == 17)
                    let msg_type = ((arb_id_raw >> 24) & 0x1F) as u8;
                    let motor_can_id = ((arb_id_raw >> 8) & 0xFF) as u8;

                    // Handle ReadParam responses (battery voltage)
                    if msg_type == 17 {
                        match robstride::parse_read_param_response(&frame) {
                            Ok((param_id, value)) => {
                                if param_id == robstride::params::VBUS {
                                    let now = Instant::now();
                                    self.motor_vbus.insert(motor_can_id, (value, now));
                                    // Average across motors with a reading in the
                                    // last 2 s (round-robin polls every ~100ms; 2 s
                                    // gives every motor multiple chances to be heard
                                    // before falling out of the average).
                                    let cutoff = Duration::from_secs(2);
                                    let fresh: Vec<f32> = self.motor_vbus.values()
                                        .filter(|(_, t)| now.duration_since(*t) < cutoff)
                                        .map(|(v, _)| *v)
                                        .collect();
                                    if !fresh.is_empty() {
                                        let avg = fresh.iter().sum::<f32>() / fresh.len() as f32;
                                        self.battery_voltage = Some(avg);
                                        self.last_battery_voltage_update = Some(now);
                                        debug!("Battery voltage avg: {:.2}V (motor 0x{:02X}={:.2}V, n={})",
                                               avg, motor_can_id, value, fresh.len());
                                    }
                                } else {
                                    debug!("Read param response: 0x{:04X} = {:.3}", param_id, value);
                                }
                            }
                            Err(e) => {
                                warn!("Failed to parse read param response: {}", e);
                            }
                        }
                        continue;
                    }

                    // Only process Feedback frames (msg_type == 2)
                    if msg_type != 2 {
                        debug!("Skipping non-feedback frame: arb_id=0x{:08X}, msg_type={}", arb_id_raw, msg_type);
                        continue;
                    }
                    debug!("Received feedback frame: arb_id=0x{:08X}, motor_id=0x{:02X}", arb_id_raw, motor_can_id);

                    // Try to find motor in base group
                    if let Some(motor) = self.base_group.find_motor_mut(motor_can_id) {
                        match robstride::parse_feedback(&frame, motor.config.model) {
                            Ok(feedback) => {
                                if feedback.error.has_error() {
                                    warn!("⚠ MOTOR ERROR {} [0x{:02X}]: mode={:?}, pos={:.3}, vel={:.3}, err={:?}",
                                          motor.config.id, motor_can_id, feedback.mode, feedback.position, feedback.velocity, feedback.error);
                                } else {
                                    info!("← {} [0x{:02X}]: mode={:?}, pos={:.3}, vel={:.3}, torque={:.3}",
                                          motor.config.id, motor_can_id, feedback.mode, feedback.position, feedback.velocity, feedback.torque);
                                }
                                motor.update_feedback(feedback);
                            }
                            Err(e) => {
                                warn!("Failed to parse feedback for motor {}: {}", motor.config.id, e);
                            }
                        }
                        continue;
                    }

                    // Try to find motor in arm group
                    if let Some(motor) = self.arm_group.find_motor_mut(motor_can_id) {
                        match robstride::parse_feedback(&frame, motor.config.model) {
                            Ok(feedback) => {
                                debug!("Parsed feedback for arm motor {}: pos={:.3}, vel={:.3}", motor.config.id, feedback.position, feedback.velocity);
                                motor.update_feedback(feedback);
                            }
                            Err(e) => {
                                debug!("Failed to parse feedback for arm motor {}: {}", motor.config.id, e);
                            }
                        }
                    } else {
                        debug!("No motor found for CAN ID 0x{:02X}", motor_can_id);
                    }
                }
                    Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                        break; // Move to next CAN bus
                    }
                    Err(e) => return Err(e.into()),
                }
            }
        }

        if frame_count > 0 {
            debug!("Read {} frames this cycle", frame_count);
        }
        Ok(())
    }

    fn publish_telemetry(&mut self) -> Result<()> {
        let mut msg = TelemetryMessage::new();

        let all_motors = self.base_group.motors.iter().chain(self.arm_group.motors.iter());
        for motor in all_motors {
            if let Some(feedback) = &motor.feedback {
                let error_str = if feedback.error.has_error() {
                    Some(format!("{:?}", feedback.error))
                } else if let Some(ref fi) = motor.fault_info {
                    // Report fault info even if current feedback has no error flags
                    Some(format!("fault: {:?} (recovery_attempts: {})", fi.error, fi.recovery_attempts))
                } else {
                    None
                };

                msg.add_motor(
                    motor.config.id.clone(),
                    MotorTelemetry {
                        position: feedback.position,
                        velocity: feedback.velocity,
                        torque: feedback.torque,
                        temperature: feedback.temperature,
                        state: motor.state.as_str().to_string(),
                        mode: feedback.mode.as_str().to_string(),
                        error: error_str,
                    },
                );
            }
        }

        // Include battery voltage only if fresh (within 5 s of last VBUS response)
        let batt_fresh = self.last_battery_voltage_update
            .map(|t| t.elapsed() < Duration::from_secs(5))
            .unwrap_or(false);
        msg.battery_voltage = if batt_fresh { self.battery_voltage } else { None };
        msg.emergency_stop = self.emergency_stop;

        self.telemetry_pub.publish(&msg)?;
        Ok(())
    }

    fn check_safety(&mut self) {
        // 2000ms timeout: CAN TX queue overflow can cause transient feedback gaps lasting
        // hundreds of ms. A 500ms timeout was causing a death spiral: timeout → Error state →
        // stop sending control frames → motor never recovers. 2s gives ~800 arm cycles
        // worth of retries before giving up on a truly dead motor.
        let feedback_timeout = Duration::from_millis(2000);

        // Check for command timeouts — hold position instead of emergency stopping.
        // Motors stay Running with Kp/Kd gains so the arm doesn't drop under gravity.
        let base_timeout = self.base_group.has_timeouts();
        let arm_timeout = self.arm_group.has_timeouts();
        if base_timeout {
            self.base_group.hold_timed_out();
        }
        if arm_timeout {
            self.arm_group.hold_timed_out();
        }
        if (base_timeout || arm_timeout) && self.last_hold_log.elapsed() >= Duration::from_secs(5) {
            warn!("Command timeout — holding position (base={}, arm={})", base_timeout, arm_timeout);
            self.last_hold_log = Instant::now();
        }

        // Check for feedback heartbeat timeouts (motor stopped responding on CAN)
        for motor in &mut self.base_group.motors {
            if motor.is_feedback_timeout(feedback_timeout) && motor.state != MotorState::Error {
                warn!("Motor {} feedback timeout (no CAN response for {:?})",
                      motor.config.id, feedback_timeout);
                motor.state = MotorState::Error;
                motor.fault_info = Some(motor::FaultInfo {
                    error: robstride::MotorError::from_bits(0),
                    mode_at_fault: motor.feedback.as_ref()
                        .map(|f| f.mode).unwrap_or(robstride::MotorMode::Unknown),
                    recovery_attempts: 0,
                });
            }
        }
        for motor in &mut self.arm_group.motors {
            if motor.is_feedback_timeout(feedback_timeout) && motor.state != MotorState::Error {
                warn!("Motor {} feedback timeout (no CAN response for {:?})",
                      motor.config.id, feedback_timeout);
                motor.state = MotorState::Error;
                motor.fault_info = Some(motor::FaultInfo {
                    error: robstride::MotorError::from_bits(0),
                    mode_at_fault: motor.feedback.as_ref()
                        .map(|f| f.mode).unwrap_or(robstride::MotorMode::Unknown),
                    recovery_attempts: 0,
                });
            }
        }

        // Check for errors (rate-limited logging — once per second)
        if (self.base_group.has_errors() || self.arm_group.has_errors())
            && self.last_error_log.elapsed() >= Duration::from_secs(1)
        {
            let base_err: Vec<_> = self.base_group.motors.iter()
                .filter(|m| m.state == MotorState::Error)
                .map(|m| m.config.id.as_str()).collect();
            let arm_err: Vec<_> = self.arm_group.motors.iter()
                .filter(|m| m.state == MotorState::Error)
                .map(|m| m.config.id.as_str()).collect();
            if !base_err.is_empty() {
                error!("Base group errors: {:?}", base_err);
            }
            if !arm_err.is_empty() {
                error!("Arm group errors: {:?}", arm_err);
            }
            self.last_error_log = Instant::now();
        }
    }
}

#[tokio::main]
async fn main() -> Result<()> {
    // Initialize logging
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| tracing_subscriber::EnvFilter::new("info")),
        )
        .init();

    info!("AIZEE Motor Control Starting...");

    // Load configuration
    let config = load_config()?;
    info!("Configuration loaded");

    // Initialize control system
    let mut system = ControlSystem::new(config.clone())?;

    // Control loop timers
    let arm_period = Duration::from_nanos(system.arm_group.control_period_ns());
    let base_period = Duration::from_nanos(system.base_group.control_period_ns());
    let telemetry_period = Duration::from_secs_f32(1.0 / config.control.telemetry_rate);

    let mut arm_interval = interval(arm_period);
    arm_interval.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
    let mut base_interval = interval(base_period);
    base_interval.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
    let mut telemetry_interval = interval(telemetry_period);

    info!("Control loops started:");
    info!("  Arm: {:.1} Hz", system.arm_group.control_frequency);
    info!("  Base: {:.1} Hz", system.base_group.control_frequency);
    info!("  Telemetry: {:.1} Hz", config.control.telemetry_rate);
    info!("Entering main control loop...");

    let mut loop_counter = 0u64;
    loop {
        loop_counter += 1;
        if loop_counter == 1 {
            info!("First loop iteration starting...");
        }
        if loop_counter % 1000 == 0 {
            info!("Main loop heartbeat: {} iterations", loop_counter);
        }
        tokio::select! {
            _ = arm_interval.tick() => {
                // Arm control loop (1 kHz) - arm motors only
                if let Err(e) = system.read_feedback() {
                    error!("Failed to read feedback: {}", e);
                }
                if let Err(e) = system.send_arm_commands() {
                    error!("Failed to send arm commands: {}", e);
                }
                system.check_safety();

                // Check for commands on every arm iteration (1kHz rate)
                match system.command_sub.recv_command() {
                    Ok(Some(cmd)) => {
                        info!("✓ Received command: {:?}", cmd);
                        if let Err(e) = system.handle_command(cmd) {
                            error!("Failed to handle command: {}", e);
                        }
                    }
                    Ok(None) => {}
                    Err(e) => {
                        error!("Failed to receive command: {}", e);
                    }
                }
            }
            _ = base_interval.tick() => {
                // Base control loop (100 Hz) - base motors only
                if let Err(e) = system.send_base_commands() {
                    error!("Failed to send base commands: {}", e);
                }
                system.recover_can_if_needed();
            }
            _ = telemetry_interval.tick() => {
                // Telemetry publishing (50 Hz)
                if let Err(e) = system.publish_telemetry() {
                    error!("Failed to publish telemetry: {}", e);
                }
            }
        }
    }
}
