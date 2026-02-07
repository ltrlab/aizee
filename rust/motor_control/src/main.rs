// AIZEE Motor Control - Main Control Loop

mod motor;
mod robstride;

use anyhow::{Context, Result};
use comms::{CommandMessage, CommandSubscriber, MotorTelemetry, TelemetryMessage, TelemetryPublisher};
use motor::{Motor, MotorConfig, MotorGroup, MotorState};
use robstride::{MotorModel, RunMode};
use socketcan::{CanSocket, EmbeddedFrame, Socket};
use std::collections::HashMap;
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
    #[serde(rename = "type")]
    motor_type: String,
    #[serde(default)]
    min_position: Option<f32>,
    #[serde(default)]
    max_position: Option<f32>,
    max_velocity: f32,
    max_torque: f32,
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
    interface: String,
}

#[derive(Debug, Clone, serde::Deserialize)]
struct NetworkConfig {
    jetson: JetsonConfig,
}

#[derive(Debug, Clone, serde::Deserialize)]
struct JetsonConfig {
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
        model: parse_motor_model(&yaml.motor_type),
        min_position: yaml.min_position,
        max_position: yaml.max_position,
        max_velocity: yaml.max_velocity,
        max_torque: yaml.max_torque,
    }
}

struct ControlSystem {
    can_socket: CanSocket,
    base_group: MotorGroup,
    arm_group: MotorGroup,
    command_sub: CommandSubscriber,
    telemetry_pub: TelemetryPublisher,
    motor_id_map: HashMap<String, u8>, // motor_id -> can_id
    emergency_stop: bool,
    last_base_control: Instant,
    control_tick: u64,
}

impl ControlSystem {
    fn new(config: Config) -> Result<Self> {
        // Initialize CAN socket
        let can_socket = CanSocket::open(&config.can.interface)
            .with_context(|| format!("Failed to open CAN interface {}", config.can.interface))?;

        // Set to non-blocking mode to avoid blocking tokio runtime
        can_socket.set_nonblocking(true)
            .context("Failed to set CAN socket to non-blocking mode")?;

        info!("CAN interface {} opened (non-blocking)", config.can.interface);

        // Create motor groups
        let watchdog_timeout = Duration::from_secs_f32(config.control.watchdog_timeout);

        let mut base_motors = Vec::new();
        for wheel in &config.motors.wheels {
            base_motors.push(Motor::new(yaml_to_motor_config(wheel), watchdog_timeout));
        }
        base_motors.push(Motor::new(yaml_to_motor_config(&config.motors.swivel), watchdog_timeout));

        let base_group = MotorGroup::new(
            "base".to_string(),
            base_motors,
            config.control.base_frequency,
        );

        let arm_motors: Vec<Motor> = config
            .motors
            .arm
            .iter()
            .map(|m| Motor::new(yaml_to_motor_config(m), watchdog_timeout))
            .collect();

        let arm_group = MotorGroup::new(
            "arm".to_string(),
            arm_motors,
            config.control.arm_frequency,
        );

        // Build motor ID map
        let mut motor_id_map = HashMap::new();
        for wheel in &config.motors.wheels {
            motor_id_map.insert(wheel.id.clone(), wheel.can_id);
        }
        motor_id_map.insert(config.motors.swivel.id.clone(), config.motors.swivel.can_id);
        for arm_motor in &config.motors.arm {
            motor_id_map.insert(arm_motor.id.clone(), arm_motor.can_id);
        }

        // Initialize ZeroMQ
        let command_sub = CommandSubscriber::new(&config.network.jetson.zmq.command_sub)?;
        let telemetry_pub = TelemetryPublisher::new(&config.network.jetson.zmq.telemetry_pub)?;

        info!("Control system initialized");
        Ok(Self {
            can_socket,
            base_group,
            arm_group,
            command_sub,
            telemetry_pub,
            motor_id_map,
            emergency_stop: false,
            last_base_control: Instant::now(),
            control_tick: 0,
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

    /// Send zero-force keepalive frames to all enabled/running base motors
    fn send_keepalives(&self) {
        for motor in &self.base_group.motors {
            if motor.state == MotorState::Enabled || motor.state == MotorState::Running {
                let pos = motor.feedback.as_ref().map(|f| f.position).unwrap_or(0.0);
                let keepalive = robstride::build_control_frame(
                    motor.config.can_id, motor.config.model, pos, 0.0, 0.0, 0.0, 0.0,
                );
                let _ = self.can_socket.write_frame(&keepalive);
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

            // First send disable to clear any existing fault state
            let disable_frame = robstride::build_disable_frame(can_id);
            self.can_socket.write_frame(&disable_frame)?;
            self.sleep_with_keepalives(50);
            info!("Sent disable (fault clear) for motor {}", motor_id);

            // Drain any pending CAN frames
            loop {
                match self.can_socket.read_frame() {
                    Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => break,
                    _ => continue,
                }
            }

            // Send enable command with retries
            let mut enabled = false;
            let mut confirmed_position = 0.0f32;
            for attempt in 1..=3 {
                let frame = robstride::build_enable_frame(can_id);
                self.can_socket.write_frame(&frame)?;
                info!("Sent enable for motor {} (attempt {})", motor_id, attempt);

                // Brief wait then check for response
                self.sleep_with_keepalives(10);

                // Read response frames to check if motor entered Run mode
                let mut got_run_mode = false;
                for _ in 0..20 {
                    // Send keepalives while polling for response
                    self.send_keepalives();
                    match self.can_socket.read_frame() {
                        Ok(frame) => {
                            let arb_id_raw = match frame.id() {
                                socketcan::Id::Extended(id) => id.as_raw(),
                                _ => continue,
                            };
                            let msg_type = ((arb_id_raw >> 24) & 0x1F) as u8;
                            if msg_type != 2 { continue; }
                            let resp_motor_id = ((arb_id_raw >> 8) & 0xFF) as u8;
                            if resp_motor_id != can_id { continue; }

                            let mode_bits = ((arb_id_raw >> 22) & 0x03) as u8;
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
                                break;
                            }
                        }
                        Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                            std::thread::sleep(std::time::Duration::from_millis(2));
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
                    for _ in 0..50 {
                        let keepalive = robstride::build_control_frame(can_id, model, confirmed_position, 0.0, 0.0, 0.0, 0.0);
                        let _ = self.can_socket.write_frame(&keepalive);
                        self.send_keepalives(); // keep other motors alive too
                        std::thread::sleep(std::time::Duration::from_millis(2));
                    }

                    // Verify motor is still in Run mode after settling
                    let mut still_running = false;
                    // Drain and check latest feedback
                    loop {
                        match self.can_socket.read_frame() {
                            Ok(frame) => {
                                let arb_id_raw = match frame.id() {
                                    socketcan::Id::Extended(id) => id.as_raw(),
                                    _ => continue,
                                };
                                let msg_type = ((arb_id_raw >> 24) & 0x1F) as u8;
                                if msg_type != 2 { continue; }
                                let resp_motor_id = ((arb_id_raw >> 8) & 0xFF) as u8;
                                if resp_motor_id != can_id { continue; }
                                let mode_bits = ((arb_id_raw >> 22) & 0x03) as u8;
                                // Update confirmed_position from latest feedback
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
                        self.can_socket.write_frame(&disable_frame)?;
                        self.sleep_with_keepalives(100);
                    }
                } else {
                    warn!("Motor {} did NOT enter Run mode on attempt {}", motor_id, attempt);
                    self.can_socket.write_frame(&disable_frame)?;
                    self.sleep_with_keepalives(50);
                }
            }

            if !enabled {
                warn!("Motor {} failed to stay in Run mode after 3 attempts — may need power cycle", motor_id);
            }

            if let Some(motor) = self.find_motor_mut(motor_id) {
                motor.state = if enabled { MotorState::Enabled } else { MotorState::Error };
                motor.last_command_time = Instant::now();
                motor.consecutive_errors = 0;
                // Use the ACTUAL confirmed position from the enable response
                // This prevents stale feedback from overwriting with the old position
                motor.target_position = confirmed_position;
                motor.target_velocity = 0.0;
                motor.position_initialized = true; // Prevent re-init from stale feedback
                info!("Motor {} state set to {:?}, target_pos={:.3}", motor_id, motor.state, confirmed_position);
            }
            Ok(())
        } else {
            Err(anyhow::anyhow!("Motor {} not found", motor_id))
        }
    }

    fn disable_motor(&mut self, motor_id: &str) -> Result<()> {
        if let Some(can_id) = self.motor_id_map.get(motor_id) {
            let frame = robstride::build_disable_frame(*can_id);
            self.can_socket.write_frame(&frame)?;
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
            CommandMessage::Drive { linear, angular } => {
                // Simple differential drive for two wheels
                let left_vel = linear - angular;
                let right_vel = linear + angular;

                if let Some(left_motor) = self.base_group.motors.first_mut() {
                    info!("Setting {} velocity to {:.3} rad/s", left_motor.config.id, left_vel);
                    left_motor.set_velocity_target(left_vel)?;
                }
                if let Some(right_motor) = self.base_group.motors.get_mut(1) {
                    info!("Setting {} velocity to {:.3} rad/s", right_motor.config.id, right_vel);
                    right_motor.set_velocity_target(right_vel)?;
                }
            }
            CommandMessage::ArmJoints {
                positions,
                velocities,
                kp,
                kd,
            } => {
                let num_arm_motors = self.arm_group.motors.len();
                if positions.len() != num_arm_motors {
                    return Err(anyhow::anyhow!(
                        "Expected {} positions, got {}",
                        num_arm_motors,
                        positions.len()
                    ));
                }

                for (i, motor) in self.arm_group.motors.iter_mut().enumerate() {
                    let vel = velocities.get(i).copied().unwrap_or(0.0);
                    let kp_val = kp.get(i).copied().unwrap_or(30.0);
                    let kd_val = kd.get(i).copied().unwrap_or(1.0);
                    motor.set_position_target(positions[i], vel, kp_val, kd_val)?;
                }
            }
            CommandMessage::Enable { motor_ids } => {
                let mut enabled_motors: Vec<(u8, MotorModel)> = Vec::new();
                for motor_id in &motor_ids {
                    // Send keepalives to already-enabled motors before enabling next one
                    for &(cid, cmodel) in &enabled_motors {
                        let keepalive = robstride::build_control_frame(cid, cmodel, 0.0, 0.0, 0.0, 0.0, 0.0);
                        let _ = self.can_socket.write_frame(&keepalive);
                    }
                    self.enable_motor(motor_id)?;
                    if let Some((cid, cmodel)) = self.can_id_and_model(motor_id) {
                        enabled_motors.push((cid, cmodel));
                    }
                }
            }
            CommandMessage::Disable { motor_ids } => {
                for motor_id in motor_ids {
                    self.disable_motor(&motor_id)?;
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

    fn send_control_commands(&mut self) -> Result<()> {
        self.control_tick += 1;

        // Send control commands to arm motors
        for motor in &self.arm_group.motors {
            if motor.state == MotorState::Running {
                let frame = robstride::build_control_frame(
                    motor.config.can_id,
                    motor.config.model,
                    motor.target_position,
                    motor.target_velocity,
                    motor.kp,
                    motor.kd,
                    motor.target_torque,
                );
                self.can_socket.write_frame(&frame)?;
            }
        }

        // Send position commands to base motors (integrate velocity)
        // ROBSTRIDE motors need changing position targets to move
        // Use actual elapsed time for velocity integration (this function is called
        // at arm loop rate ~1kHz, not base rate ~100Hz)
        let now = Instant::now();
        let dt = now.duration_since(self.last_base_control).as_secs_f32();
        self.last_base_control = now;
        // Clamp dt to avoid jumps on first iteration or timing glitches
        let dt = dt.clamp(0.0001, 0.1);
        for motor in &mut self.base_group.motors {
            // Send control frames for both Enabled and Running motors
            // Motors need continuous control frames after enable to stay active
            if motor.state == MotorState::Enabled || motor.state == MotorState::Running {
                // Always sync target_position from latest feedback when available
                // This prevents stale targets from causing jerks
                if let Some(ref fb) = motor.feedback {
                    if !motor.position_initialized {
                        motor.target_position = fb.position;
                        motor.position_initialized = true;
                        info!("  → Initialized {} target position to {:.3} rad",
                              motor.config.id, motor.target_position);
                    }
                }

                if motor.state == MotorState::Enabled {
                    // Hold position with gentle damping while waiting for commands
                    let hold_pos = motor.feedback.as_ref().map(|f| f.position).unwrap_or(motor.target_position);
                    let frame = robstride::build_control_frame(
                        motor.config.can_id,
                        motor.config.model,
                        hold_pos,  // Track feedback position
                        0.0,       // Target velocity = 0
                        0.0,       // Zero Kp - no position force
                        0.5,       // Kd=0.5 - gentle damping, matches Running state
                        0.0,       // No torque
                    );
                    self.can_socket.write_frame(&frame)?;
                } else {
                    // Running state: integrate velocity and apply gains
                    motor.target_position += motor.target_velocity * dt;

                    let frame = robstride::build_control_frame(
                        motor.config.can_id,
                        motor.config.model,
                        motor.target_position,   // Integrated position target
                        motor.target_velocity,   // Velocity feedforward
                        1.0,   // Kp - gentle position tracking
                        0.5,   // Kd - velocity damping
                        0.0,   // No feedforward torque
                    );
                    self.can_socket.write_frame(&frame)?;

                    // Log Running state every 500 ticks (~500ms at 1kHz) for diagnostics
                    if self.control_tick % 500 == 0 {
                        info!("  RUN {}: tgt_pos={:.3} tgt_vel={:.3} fb_pos={:.3} dt={:.4}",
                              motor.config.id, motor.target_position, motor.target_velocity,
                              motor.feedback.as_ref().map(|f| f.position).unwrap_or(-999.0), dt);
                    }
                }
            }
        }

        Ok(())
    }

    fn read_feedback(&mut self) -> Result<()> {
        // Read all available CAN frames
        let mut frame_count = 0;
        loop {
            match self.can_socket.read_frame() {
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
                    // Only process Feedback frames (msg_type == 2)
                    let msg_type = ((arb_id_raw >> 24) & 0x1F) as u8;
                    if msg_type != 2 {
                        debug!("Skipping non-feedback frame: arb_id=0x{:08X}, msg_type={}", arb_id_raw, msg_type);
                        continue;
                    }
                    let motor_can_id = ((arb_id_raw >> 8) & 0xFF) as u8;
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
                    if frame_count > 0 {
                        debug!("Read {} frames this cycle", frame_count);
                    }
                    break;
                }
                Err(e) => return Err(e.into()),
            }
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

        self.telemetry_pub.publish(&msg)?;
        Ok(())
    }

    fn check_safety(&mut self) {
        // 500ms timeout accounts for enable_motor() blocking the control loop (~200-400ms)
        let feedback_timeout = Duration::from_millis(500);

        // Check for command timeouts
        if self.base_group.has_timeouts() {
            warn!("Base group timeout detected");
            self.base_group.emergency_stop_all();
        }
        if self.arm_group.has_timeouts() {
            warn!("Arm group timeout detected");
            self.arm_group.emergency_stop_all();
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

        // Check for errors (logging only)
        if self.base_group.has_errors() {
            error!("Base group has errors");
        }
        if self.arm_group.has_errors() {
            error!("Arm group has errors");
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
    let mut base_interval = interval(base_period);
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
                // Arm control loop (1 kHz)
                if let Err(e) = system.read_feedback() {
                    error!("Failed to read feedback: {}", e);
                }
                if let Err(e) = system.send_control_commands() {
                    error!("Failed to send control commands: {}", e);
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
                    Ok(None) => {
                        // No command available (normal - timeout)
                    }
                    Err(e) => {
                        error!("Failed to receive command: {}", e);
                    }
                }
            }
            _ = base_interval.tick() => {
                // Base control loop (100 Hz)
                // Feedback is already read in arm loop
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
