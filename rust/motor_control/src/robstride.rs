// ROBSTRIDE Motor CAN Protocol Implementation
// Based on RS03-EN protocol specification

use anyhow::{anyhow, Result};
use socketcan::{CanFrame, EmbeddedFrame, ExtendedId};

/// Motor command types
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum MotorMsg {
    Info = 0,
    Control = 1,
    Feedback = 2,
    Enable = 3,
    Disable = 4,
    ZeroPos = 6,
    SetID = 7,
    ReadParam = 17,
    WriteParam = 18,
    SaveConfig = 22,
}

/// Motor operating modes
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum RunMode {
    Operation = 0,
    Position = 1,
    Speed = 2,
    Current = 3,
}

/// Motor operating mode reported in feedback
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MotorMode {
    Reset = 0,       // Disabled / in reset state
    Calibration = 1, // Calibrating
    Run = 2,         // Enabled and running
    Unknown = 3,     // Reserved
}

impl MotorMode {
    pub fn from_bits(bits: u8) -> Self {
        match bits & 0x03 {
            0 => MotorMode::Reset,
            1 => MotorMode::Calibration,
            2 => MotorMode::Run,
            _ => MotorMode::Unknown,
        }
    }

    pub fn is_running(&self) -> bool {
        *self == MotorMode::Run
    }

    pub fn as_str(&self) -> &'static str {
        match self {
            MotorMode::Reset => "reset",
            MotorMode::Calibration => "calibration",
            MotorMode::Run => "run",
            MotorMode::Unknown => "unknown",
        }
    }
}

/// Motor error flags
#[derive(Debug, Clone, Copy)]
pub struct MotorError {
    pub undervoltage: bool,
    pub overcurrent: bool,
    pub overtemp: bool,
    pub magnetic_encoding_fault: bool,
    pub hall_encoding_fault: bool,
    pub uncalibrated: bool,
}

impl MotorError {
    pub fn from_bits(error_bits: u8) -> Self {
        Self {
            undervoltage: (error_bits & 0x01) != 0,
            overcurrent: (error_bits & 0x02) != 0,
            overtemp: (error_bits & 0x04) != 0,
            magnetic_encoding_fault: (error_bits & 0x08) != 0,
            hall_encoding_fault: (error_bits & 0x10) != 0,
            uncalibrated: (error_bits & 0x20) != 0,
        }
    }

    pub fn has_error(&self) -> bool {
        self.undervoltage
            || self.overcurrent
            || self.overtemp
            || self.magnetic_encoding_fault
            || self.hall_encoding_fault
            || self.uncalibrated
    }
}

/// Motor model types with different scaling factors
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MotorModel {
    Model04, // High torque (240 Nm range)
    Model03, // Medium torque (34 Nm range)
    Model02, // Low torque
    Model00, // Micro motor (wrist/gripper, ~2 Nm)
}

impl MotorModel {
    /// Get velocity scaling range for MIT feedback/control (rad/s)
    pub fn velocity_range(&self) -> (f32, f32) {
        let max = self.mit_velocity_max();
        (-max, max)
    }

    /// Get torque scaling range for MIT feedback/control (Nm)
    pub fn torque_range(&self) -> (f32, f32) {
        let max = self.mit_torque_max();
        (-max, max)
    }

    /// MIT velocity max (rad/s) - model-specific
    pub fn mit_velocity_max(&self) -> f32 {
        match self {
            MotorModel::Model04 => 15.0,
            MotorModel::Model03 => 50.0,
            MotorModel::Model02 => 44.0,
            MotorModel::Model00 => 30.0,
        }
    }

    /// MIT torque max (Nm) - model-specific
    pub fn mit_torque_max(&self) -> f32 {
        match self {
            MotorModel::Model04 => 120.0,
            MotorModel::Model03 => 60.0,
            MotorModel::Model02 => 17.0,
            MotorModel::Model00 => 2.0,
        }
    }

    /// MIT Kp max (Nm/rad) - model-specific
    pub fn mit_kp_max(&self) -> f32 {
        match self {
            MotorModel::Model04 | MotorModel::Model03 => 5000.0,
            MotorModel::Model02 => 500.0,
            MotorModel::Model00 => 100.0,
        }
    }

    /// MIT Kd max (Nm*s/rad) - model-specific
    pub fn mit_kd_max(&self) -> f32 {
        match self {
            MotorModel::Model04 | MotorModel::Model03 => 100.0,
            MotorModel::Model02 => 5.0,
            MotorModel::Model00 => 5.0,
        }
    }
}

/// Motor feedback state
#[derive(Debug, Clone)]
pub struct MotorFeedback {
    pub motor_id: u8,
    pub mode: MotorMode,    // Motor operating mode (Reset/Calibration/Run)
    pub position: f32,      // radians
    pub velocity: f32,      // rad/s
    pub torque: f32,        // Nm
    pub temperature: f32,   // °C
    pub error: MotorError,
    pub timestamp: std::time::Instant,
}

/// Parameter IDs for read/write operations
#[allow(dead_code)]
pub mod params {
    pub const RUN_MODE: u16 = 0x7005;
    pub const IQ_REF: u16 = 0x7006;
    pub const SPD_REF: u16 = 0x700A;
    pub const LIMIT_TORQUE: u16 = 0x700B;
    pub const CUR_KP: u16 = 0x7010;
    pub const CUR_KI: u16 = 0x7011;
    pub const CUR_FIT_GAIN: u16 = 0x7014;
    pub const LOC_REF: u16 = 0x7016;
    pub const LIMIT_SPD: u16 = 0x7017;
    pub const LIMIT_CUR: u16 = 0x7018;
    pub const MECHPOS: u16 = 0x7019;
    pub const IQF: u16 = 0x701A;
    pub const MECHVEL: u16 = 0x701B;
    pub const VBUS: u16 = 0x701C;  // Battery voltage (float, in Volts)
    pub const LOC_KP: u16 = 0x701E;
    pub const SPD_KP: u16 = 0x701F;
    pub const SPD_KI: u16 = 0x7020;
    pub const SPD_FILT_GAIN: u16 = 0x7021;
}

const HOST_CAN_ID: u8 = 0xAA;

/// Build CAN arbitration ID
fn build_arb_id(motor_id: u8, msg_type: MotorMsg) -> ExtendedId {
    let id_raw = (motor_id as u32) | ((HOST_CAN_ID as u32) << 8) | ((msg_type as u32) << 24);
    ExtendedId::new(id_raw).expect("Invalid extended CAN ID")
}

/// Parse feedback from CAN frame
pub fn parse_feedback(frame: &CanFrame, model: MotorModel) -> Result<MotorFeedback> {
    let data = frame.data();
    if data.len() != 8 {
        return Err(anyhow!("Invalid frame length: {}", data.len()));
    }

    let arb_id = frame.id();
    let arb_id_raw = match arb_id {
        socketcan::Id::Extended(id) => id.as_raw(),
        _ => return Err(anyhow!("Expected extended ID")),
    };

    // In responses, motor swaps fields: motor_id is bits 15:8, host_id is bits 7:0
    let motor_id = ((arb_id_raw >> 8) & 0xFF) as u8;
    let error_bits = ((arb_id_raw >> 16) & 0x1F) as u8;
    let mode_bits = ((arb_id_raw >> 22) & 0x03) as u8;
    let mode = MotorMode::from_bits(mode_bits);

    // Parse angle (bytes 0-1, big-endian)
    let angle_raw = u16::from_be_bytes([data[0], data[1]]);
    let position = (angle_raw as f32 / 65535.0) * (8.0 * std::f32::consts::PI) - (4.0 * std::f32::consts::PI);

    // Parse velocity (bytes 2-3)
    let vel_raw = u16::from_be_bytes([data[2], data[3]]);
    let (vel_min, vel_max) = model.velocity_range();
    let velocity = (vel_raw as f32 / 65535.0) * (vel_max - vel_min) + vel_min;

    // Parse torque (bytes 4-5)
    let torque_raw = u16::from_be_bytes([data[4], data[5]]);
    let (torque_min, torque_max) = model.torque_range();
    let torque = (torque_raw as f32 / 65535.0) * (torque_max - torque_min) + torque_min;

    // Parse temperature (bytes 6-7)
    let temp_raw = u16::from_be_bytes([data[6], data[7]]);
    let temperature = temp_raw as f32 / 10.0;

    Ok(MotorFeedback {
        motor_id,
        mode,
        position,
        velocity,
        torque,
        temperature,
        error: MotorError::from_bits(error_bits),
        timestamp: std::time::Instant::now(),
    })
}

/// Build enable command frame
pub fn build_enable_frame(motor_id: u8) -> CanFrame {
    let arb_id = build_arb_id(motor_id, MotorMsg::Enable);
    CanFrame::new(socketcan::Id::Extended(arb_id), &[0u8; 8]).expect("Failed to create frame")
}

/// Build disable command frame
pub fn build_disable_frame(motor_id: u8) -> CanFrame {
    let arb_id = build_arb_id(motor_id, MotorMsg::Disable);
    CanFrame::new(socketcan::Id::Extended(arb_id), &[0u8; 8]).expect("Failed to create frame")
}

/// Build a fault-CLEAR frame — the RobStride/CyberGear type-4 "stop" with
/// `data[0] = 0x01`. The plain disable (data[0]=0) is a stop that does NOT clear
/// latched faults, so a motor that has tripped (e.g. overcurrent) stays fault-locked
/// and refuses to re-enter Run mode until a real clear or a power-cycle. This variant
/// clears the latch. It leaves the motor disabled — re-enable afterward.
pub fn build_clear_fault_frame(motor_id: u8) -> CanFrame {
    let arb_id = build_arb_id(motor_id, MotorMsg::Disable);
    let mut data = [0u8; 8];
    data[0] = 0x01; // clear-fault flag on the stop frame
    CanFrame::new(socketcan::Id::Extended(arb_id), &data).expect("Failed to create frame")
}

/// Build zero position command frame
pub fn build_zero_pos_frame(motor_id: u8) -> CanFrame {
    let arb_id = build_arb_id(motor_id, MotorMsg::ZeroPos);
    let mut data = [0u8; 8];
    data[0] = 0x01; // Required per Robstride protocol
    CanFrame::new(socketcan::Id::Extended(arb_id), &data).expect("Failed to create frame")
}

/// Build SaveConfig command frame — persists current parameter set (including
/// the mechanical zero just written) to motor flash.
pub fn build_save_config_frame(motor_id: u8) -> CanFrame {
    let arb_id = build_arb_id(motor_id, MotorMsg::SaveConfig);
    CanFrame::new(socketcan::Id::Extended(arb_id), &[0u8; 8]).expect("Failed to create frame")
}

/// Build read parameter frame
pub fn build_read_param_frame(motor_id: u8, param_id: u16) -> CanFrame {
    let arb_id = build_arb_id(motor_id, MotorMsg::ReadParam);
    let mut data = [0u8; 8];
    data[0..2].copy_from_slice(&param_id.to_le_bytes());
    CanFrame::new(socketcan::Id::Extended(arb_id), &data).expect("Failed to create frame")
}

/// Parse read parameter response
/// Returns (param_id, value) from the response frame
pub fn parse_read_param_response(frame: &CanFrame) -> Result<(u16, f32)> {
    let data = frame.data();
    if data.len() != 8 {
        return Err(anyhow!("Invalid parameter response length: {}", data.len()));
    }

    // Parameter ID in bytes 0-1 (little-endian)
    let param_id = u16::from_le_bytes([data[0], data[1]]);

    // Value in bytes 4-7 (float, little-endian)
    let value = f32::from_le_bytes([data[4], data[5], data[6], data[7]]);

    Ok((param_id, value))
}

/// Build write parameter frame
pub fn build_write_param_frame(motor_id: u8, param_id: u16, value: f32) -> CanFrame {
    let arb_id = build_arb_id(motor_id, MotorMsg::WriteParam);
    let mut data = [0u8; 8];
    data[0..2].copy_from_slice(&param_id.to_le_bytes());
    data[4..8].copy_from_slice(&value.to_le_bytes());
    CanFrame::new(socketcan::Id::Extended(arb_id), &data).expect("Failed to create frame")
}

/// Build write parameter frame for run mode (uses byte instead of float)
pub fn build_write_run_mode_frame(motor_id: u8, mode: RunMode) -> CanFrame {
    let arb_id = build_arb_id(motor_id, MotorMsg::WriteParam);
    let mut data = [0u8; 8];
    data[0..2].copy_from_slice(&params::RUN_MODE.to_le_bytes());
    data[4] = mode as u8;
    CanFrame::new(socketcan::Id::Extended(arb_id), &data).expect("Failed to create frame")
}

/// Build control frame for position/velocity/torque control (MIT mode)
///
/// ROBSTRIDE protocol: 4 × 16-bit fields in data bytes, torque in arb_id.
/// - Data: position(u16) | velocity(u16) | kp(u16) | kd(u16)  [big-endian]
/// - Arb ID bits 8-23: torque (encoded u16)
/// - Arb ID bits 0-7: motor_id
/// - Arb ID bits 24-28: message type (1 = control)
///
/// Scaling is model-specific (RS-03/04 use different ranges than RS-02).
pub fn build_control_frame(
    motor_id: u8,
    model: MotorModel,
    position: f32,
    velocity: f32,
    kp: f32,
    kd: f32,
    torque: f32,
) -> CanFrame {
    let pos_max = 4.0 * std::f32::consts::PI;
    let vel_max = model.mit_velocity_max();
    let kp_max = model.mit_kp_max();
    let kd_max = model.mit_kd_max();
    let torque_max = model.mit_torque_max();

    // Encode position: [-4π, 4π] → [0, 0xFFFF] (signed, midpoint = 0x7FFF)
    let pos_u16 = ((position.clamp(-pos_max, pos_max) / pos_max + 1.0) * 0x7FFF as f32)
        .clamp(0.0, 0xFFFF as f32) as u16;

    // Encode velocity: [-vel_max, vel_max] → [0, 0xFFFF] (signed)
    let vel_u16 = ((velocity.clamp(-vel_max, vel_max) / vel_max + 1.0) * 0x7FFF as f32)
        .clamp(0.0, 0xFFFF as f32) as u16;

    // Encode Kp: [0, kp_max] → [0, 0xFFFF] (unsigned)
    let kp_u16 = (kp.clamp(0.0, kp_max) / kp_max * 0xFFFF as f32)
        .clamp(0.0, 0xFFFF as f32) as u16;

    // Encode Kd: [0, kd_max] → [0, 0xFFFF] (unsigned)
    let kd_u16 = (kd.clamp(0.0, kd_max) / kd_max * 0xFFFF as f32)
        .clamp(0.0, 0xFFFF as f32) as u16;

    // Encode torque: [-torque_max, torque_max] → [0, 0xFFFF] (signed, goes in arb_id)
    let torque_u16 = ((torque.clamp(-torque_max, torque_max) / torque_max + 1.0) * 0x7FFF as f32)
        .clamp(0.0, 0xFFFF as f32) as u16;

    // Build CAN arbitration ID: msg_type(5) | torque_u16(16) | motor_id(8)
    let arb_id_raw = ((MotorMsg::Control as u32) << 24) | ((torque_u16 as u32) << 8) | (motor_id as u32);
    let arb_id = ExtendedId::new(arb_id_raw).expect("Invalid extended CAN ID");

    // Pack data as 4 big-endian u16: position, velocity, kp, kd
    let mut data = [0u8; 8];
    data[0..2].copy_from_slice(&pos_u16.to_be_bytes());
    data[2..4].copy_from_slice(&vel_u16.to_be_bytes());
    data[4..6].copy_from_slice(&kp_u16.to_be_bytes());
    data[6..8].copy_from_slice(&kd_u16.to_be_bytes());

    CanFrame::new(socketcan::Id::Extended(arb_id), &data).expect("Failed to create frame")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_arb_id_construction() {
        let arb_id = build_arb_id(1, MotorMsg::Enable);
        let raw = arb_id.as_raw();
        assert_eq!(raw & 0xFF, 1); // Motor ID
        assert_eq!((raw >> 8) & 0xFF, HOST_CAN_ID as u32); // Host ID
        assert_eq!((raw >> 24) & 0xFF, MotorMsg::Enable as u32); // Msg type
    }

    #[test]
    fn test_motor_error_parsing() {
        let error = MotorError::from_bits(0b00010110);
        assert!(!error.undervoltage);
        assert!(error.overcurrent);
        assert!(error.overtemp);
        assert!(!error.magnetic_encoding_fault);
        assert!(error.hall_encoding_fault);
        assert!(!error.uncalibrated);
        assert!(error.has_error());
    }
}
