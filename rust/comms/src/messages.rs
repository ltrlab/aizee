// Message definitions for ZeroMQ communication

use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// Command message types
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type")]
pub enum CommandMessage {
    #[serde(rename = "drive")]
    Drive {
        linear: f32,
        angular: f32,
        #[serde(default)]
        swivel: f32,
    },
    #[serde(rename = "arm_joints")]
    ArmJoints {
        positions: Vec<f32>,
        velocities: Vec<f32>,
        #[serde(default)]
        kp: Vec<f32>,
        #[serde(default)]
        kd: Vec<f32>,
    },
    #[serde(rename = "enable")]
    Enable {
        motor_ids: Vec<String>,
    },
    #[serde(rename = "disable")]
    Disable {
        motor_ids: Vec<String>,
    },
    #[serde(rename = "emergency_stop")]
    EmergencyStop,
    #[serde(rename = "zero_position")]
    ZeroPosition {
        motor_ids: Vec<String>,
    },
    #[serde(rename = "clear_fault")]
    ClearFault {
        motor_ids: Vec<String>,
    },
    #[serde(rename = "trigger_fault")]
    TriggerFault {
        motor_ids: Vec<String>,
    },
    #[serde(rename = "clear_emergency_stop")]
    ClearEmergencyStop,
}

/// Individual motor telemetry
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MotorTelemetry {
    pub position: f32,
    pub velocity: f32,
    pub torque: f32,
    pub temperature: f32,
    pub state: String,
    pub mode: String,
    pub error: Option<String>,
}

/// LiDAR scan data
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LidarScan {
    pub sensor_id: String,        // "lidar_front" or "lidar_back"
    pub angle_min: f32,           // radians
    pub angle_max: f32,           // radians
    pub angle_increment: f32,     // radians
    pub range_min: f32,           // meters (0.15 for A1M8)
    pub range_max: f32,           // meters (12.0 for A1M8)
    pub ranges: Vec<f32>,         // meters (360 points typical)
    pub intensities: Vec<u8>,     // signal quality 0-255
}

/// Telemetry message published to ZeroMQ
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TelemetryMessage {
    pub timestamp: f64,
    pub motors: HashMap<String, MotorTelemetry>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub battery_voltage: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub lidar_scans: Option<Vec<LidarScan>>,
}

impl TelemetryMessage {
    pub fn new() -> Self {
        Self {
            timestamp: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_secs_f64(),
            motors: HashMap::new(),
            battery_voltage: None,
            lidar_scans: None,
        }
    }

    pub fn add_motor(&mut self, id: String, telemetry: MotorTelemetry) {
        self.motors.insert(id, telemetry);
    }
}

impl Default for TelemetryMessage {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_command_serialization() {
        let cmd = CommandMessage::Drive {
            linear: 0.5,
            angular: 0.2,
        };
        let json = serde_json::to_string(&cmd).unwrap();
        assert!(json.contains("\"type\":\"drive\""));
        assert!(json.contains("\"linear\":0.5"));
    }

    #[test]
    fn test_arm_joints_deserialization() {
        let json = r#"{"type":"arm_joints","positions":[0.1,0.5,-0.3],"velocities":[0.0,0.0,0.0]}"#;
        let cmd: CommandMessage = serde_json::from_str(json).unwrap();
        match cmd {
            CommandMessage::ArmJoints { positions, .. } => {
                assert_eq!(positions.len(), 3);
                assert_eq!(positions[0], 0.1);
            }
            _ => panic!("Wrong command type"),
        }
    }

    #[test]
    fn test_telemetry_message() {
        let mut msg = TelemetryMessage::new();
        msg.add_motor(
            "test_motor".to_string(),
            MotorTelemetry {
                position: 1.5,
                velocity: 0.5,
                torque: 2.1,
                temperature: 45.0,
                state: "running".to_string(),
                mode: "run".to_string(),
                error: None,
            },
        );
        let json = serde_json::to_string(&msg).unwrap();
        assert!(json.contains("test_motor"));
    }
}
