// Motor state management and control

use crate::robstride::{MotorError, MotorFeedback, MotorMode, MotorModel};
use anyhow::{anyhow, Result};
use std::time::{Duration, Instant};

/// Motor state machine
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MotorState {
    Idle,
    Enabling,
    Enabled,
    Running,
    Error,
    Disabled,
}

impl MotorState {
    pub fn as_str(&self) -> &'static str {
        match self {
            MotorState::Idle => "idle",
            MotorState::Enabling => "enabling",
            MotorState::Enabled => "enabled",
            MotorState::Running => "running",
            MotorState::Error => "error",
            MotorState::Disabled => "disabled",
        }
    }
}

/// Structured fault information for diagnostics and telemetry
#[derive(Debug, Clone)]
pub struct FaultInfo {
    pub error: MotorError,
    pub mode_at_fault: MotorMode,
    pub recovery_attempts: u32,
}

/// Motor configuration from hardware.yaml
#[derive(Debug, Clone)]
pub struct MotorConfig {
    pub id: String,
    pub can_id: u8,
    pub can_bus: String,  // CAN bus name (e.g., "can1", "can2")
    pub model: MotorModel,
    pub min_position: Option<f32>,
    pub max_position: Option<f32>,
    pub max_velocity: f32,
    pub max_torque: f32,
}

/// Motor controller with state tracking
pub struct Motor {
    pub config: MotorConfig,
    pub state: MotorState,
    pub feedback: Option<MotorFeedback>,
    pub last_command_time: Instant,
    pub command_timeout: Duration,
    pub target_position: f32,
    pub target_velocity: f32,
    pub target_torque: f32,
    pub kp: f32,
    pub kd: f32,
    pub consecutive_errors: u32,
    pub position_initialized: bool,
    pub fault_info: Option<FaultInfo>,
    pub last_feedback_time: Instant,
}

impl Motor {
    pub fn new(config: MotorConfig, command_timeout: Duration) -> Self {
        Self {
            config,
            state: MotorState::Idle,
            feedback: None,
            last_command_time: Instant::now(),
            command_timeout,
            target_position: 0.0,
            target_velocity: 0.0,
            target_torque: 0.0,
            kp: 0.0,
            kd: 0.0,
            consecutive_errors: 0,
            position_initialized: false,
            fault_info: None,
            last_feedback_time: Instant::now(),
        }
    }

    /// Update motor state with new feedback
    pub fn update_feedback(&mut self, feedback: MotorFeedback) {
        self.last_feedback_time = Instant::now();

        // Mode transition detection: Run → Reset means the motor faulted
        // Only detect faults when motor is expected to be active (Enabled/Running)
        if let Some(prev) = &self.feedback {
            if prev.mode == MotorMode::Run && feedback.mode == MotorMode::Reset
                && matches!(self.state, MotorState::Enabled | MotorState::Running)
            {
                if self.state != MotorState::Error {
                    tracing::warn!(
                        "Motor {} mode transition Run→Reset (hardware fault detected)",
                        self.config.id
                    );
                    self.state = MotorState::Error;
                    self.fault_info = Some(FaultInfo {
                        error: feedback.error,
                        mode_at_fault: MotorMode::Run,
                        recovery_attempts: 0,
                    });
                }
            }
        }

        // Check for errors in feedback
        if feedback.error.has_error() {
            self.consecutive_errors += 1;
            if self.consecutive_errors >= 10 {
                // Only enter Error state after sustained errors (10 consecutive frames)
                if self.state != MotorState::Error {
                    tracing::warn!(
                        "Motor {} entering Error state after {} consecutive errors: {:?}",
                        self.config.id,
                        self.consecutive_errors,
                        feedback.error
                    );
                    self.state = MotorState::Error;
                    self.fault_info = Some(FaultInfo {
                        error: feedback.error,
                        mode_at_fault: feedback.mode,
                        recovery_attempts: 0,
                    });
                }
            } else {
                tracing::debug!(
                    "Motor {} transient error ({}/10): {:?}",
                    self.config.id,
                    self.consecutive_errors,
                    feedback.error
                );
            }
        } else {
            // Clear error counter on good feedback
            if self.consecutive_errors > 0 {
                tracing::debug!(
                    "Motor {} error cleared after {} bad frames",
                    self.config.id,
                    self.consecutive_errors
                );
            }
            self.consecutive_errors = 0;
        }

        self.feedback = Some(feedback);
    }

    /// Set target position with velocity and gains
    pub fn set_position_target(
        &mut self,
        position: f32,
        velocity: f32,
        kp: f32,
        kd: f32,
    ) -> Result<()> {
        // Validate soft limits
        if let Some(min_pos) = self.config.min_position {
            if position < min_pos {
                return Err(anyhow!(
                    "Position {} below minimum {}",
                    position,
                    min_pos
                ));
            }
        }
        if let Some(max_pos) = self.config.max_position {
            if position > max_pos {
                return Err(anyhow!(
                    "Position {} above maximum {}",
                    position,
                    max_pos
                ));
            }
        }

        // Validate velocity limit
        if velocity.abs() > self.config.max_velocity {
            return Err(anyhow!(
                "Velocity {} exceeds maximum {}",
                velocity,
                self.config.max_velocity
            ));
        }

        self.target_position = position;
        self.target_velocity = velocity;
        self.kp = kp;
        self.kd = kd;
        self.last_command_time = Instant::now();

        if self.state == MotorState::Enabled {
            self.state = MotorState::Running;
        }

        Ok(())
    }

    /// Set target velocity
    pub fn set_velocity_target(&mut self, velocity: f32) -> Result<()> {
        if velocity.abs() > self.config.max_velocity {
            return Err(anyhow!(
                "Velocity {} exceeds maximum {}",
                velocity,
                self.config.max_velocity
            ));
        }

        self.target_velocity = velocity;
        self.last_command_time = Instant::now();

        if self.state == MotorState::Enabled {
            self.state = MotorState::Running;
        }

        Ok(())
    }

    /// Check if motor has timed out (no command received)
    /// Only Running motors can timeout - Idle/Disabled motors are ignored
    pub fn is_timeout(&self) -> bool {
        matches!(self.state, MotorState::Running)
            && self.last_command_time.elapsed() > self.command_timeout
    }

    /// Check if motor feedback has timed out (no CAN response received)
    /// Only active motors (Enabled/Running) can have feedback timeouts
    pub fn is_feedback_timeout(&self, timeout: Duration) -> bool {
        matches!(self.state, MotorState::Enabled | MotorState::Running)
            && self.feedback.is_some()
            && self.last_feedback_time.elapsed() > timeout
    }

    /// Check if motor position is within soft limits
    pub fn is_within_limits(&self) -> bool {
        if let Some(feedback) = &self.feedback {
            if let Some(min_pos) = self.config.min_position {
                if feedback.position < min_pos {
                    return false;
                }
            }
            if let Some(max_pos) = self.config.max_position {
                if feedback.position > max_pos {
                    return false;
                }
            }
        }
        true
    }

    /// Emergency stop - zero all commands
    pub fn emergency_stop(&mut self) {
        self.target_position = self.feedback.as_ref().map(|f| f.position).unwrap_or(0.0);
        self.target_velocity = 0.0;
        self.target_torque = 0.0;
        self.state = MotorState::Enabled;
        tracing::warn!("Motor {} emergency stop", self.config.id);
    }

    /// Get current position (from feedback)
    pub fn position(&self) -> Option<f32> {
        self.feedback.as_ref().map(|f| f.position)
    }

    /// Get current velocity (from feedback)
    pub fn velocity(&self) -> Option<f32> {
        self.feedback.as_ref().map(|f| f.velocity)
    }

    /// Get current torque (from feedback)
    pub fn torque(&self) -> Option<f32> {
        self.feedback.as_ref().map(|f| f.torque)
    }

    /// Get motor temperature
    pub fn temperature(&self) -> Option<f32> {
        self.feedback.as_ref().map(|f| f.temperature)
    }
}

/// Motor group for coordinated control
pub struct MotorGroup {
    pub name: String,
    pub motors: Vec<Motor>,
    pub control_frequency: f32, // Hz
}

impl MotorGroup {
    pub fn new(name: String, motors: Vec<Motor>, control_frequency: f32) -> Self {
        Self {
            name,
            motors,
            control_frequency,
        }
    }

    /// Find motor by CAN ID
    pub fn find_motor_mut(&mut self, can_id: u8) -> Option<&mut Motor> {
        self.motors.iter_mut().find(|m| m.config.can_id == can_id)
    }

    /// Check if any motor has errors
    pub fn has_errors(&self) -> bool {
        self.motors.iter().any(|m| m.state == MotorState::Error)
    }

    /// Check if any motor has timed out
    pub fn has_timeouts(&self) -> bool {
        self.motors.iter().any(|m| m.is_timeout())
    }

    /// Emergency stop all motors
    pub fn emergency_stop_all(&mut self) {
        for motor in &mut self.motors {
            motor.emergency_stop();
        }
    }

    /// Get control period in nanoseconds
    pub fn control_period_ns(&self) -> u64 {
        (1_000_000_000.0 / self.control_frequency) as u64
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn test_motor() -> Motor {
        let config = MotorConfig {
            id: "test_motor".to_string(),
            can_id: 1,
            model: MotorModel::Model03,
            min_position: Some(-1.57),
            max_position: Some(1.57),
            max_velocity: 5.0,
            max_torque: 8.0,
        };
        Motor::new(config, Duration::from_millis(100))
    }

    #[test]
    fn test_position_limits() {
        let mut motor = test_motor();
        motor.state = MotorState::Enabled;

        // Valid position
        assert!(motor.set_position_target(1.0, 0.0, 1.0, 0.1).is_ok());

        // Below minimum
        assert!(motor.set_position_target(-2.0, 0.0, 1.0, 0.1).is_err());

        // Above maximum
        assert!(motor.set_position_target(2.0, 0.0, 1.0, 0.1).is_err());
    }

    #[test]
    fn test_velocity_limits() {
        let mut motor = test_motor();
        motor.state = MotorState::Enabled;

        // Valid velocity
        assert!(motor.set_velocity_target(4.0).is_ok());

        // Exceeds maximum
        assert!(motor.set_velocity_target(6.0).is_err());
    }

    #[test]
    fn test_timeout() {
        let mut motor = test_motor();
        motor.command_timeout = Duration::from_millis(10);
        motor.last_command_time = Instant::now() - Duration::from_millis(20);
        assert!(motor.is_timeout());
    }
}
