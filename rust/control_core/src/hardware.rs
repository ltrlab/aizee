//! Hardware-independent receiver-local feedback and command interface.
use crate::protocol::JointCommand;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Default, Serialize, Deserialize)]
pub struct JointFeedback {
    pub position: f64,
    pub velocity: f64,
    pub effort: f64,
    pub temperature: f64,
    pub sequence: u64,
    /// Actual sample time in the controller's monotonic clock domain.
    pub sampled_at_ms: u64,
    pub fault: bool,
}

#[derive(Debug, Clone, Copy)]
pub struct HardwareError;

/// Implementations must bound their work, preserve measurement age and fill
/// every feedback slot. Command validation happens before apply(). An adapter
/// must report partial writes as errors; it cannot promise CAN-bus atomicity.
pub trait Hardware {
    fn read(&mut self, now_ms: u64, feedback: &mut [JointFeedback]) -> Result<(), HardwareError>;
    fn enable(&mut self) -> Result<(), HardwareError>;
    fn disable(&mut self) -> Result<(), HardwareError>;
    fn apply(&mut self, commands: &[JointCommand]) -> Result<(), HardwareError>;
}

/// Ideal command-following test device. This is NOT a dynamics or braking model.
pub struct Loopback {
    pub enabled: bool,
    pub frozen: bool,
    pub fail_read: bool,
    pub fail_write: bool,
    pub motor_fault: bool,
    pub writes: u64,
    /// Successful enable() calls; physical enable permits are single-use.
    pub enables: u64,
    pub feedback: Vec<JointFeedback>,
    /// Per joint, a drive that has stopped answering: its reading is not updated (empty: none).
    pub quiet: Vec<bool>,
    observed: Vec<JointFeedback>,
}

impl Loopback {
    pub fn new(positions: &[f64]) -> Self {
        let feedback: Vec<_> = positions
            .iter()
            .map(|&position| JointFeedback {
                position,
                temperature: 25.0,
                ..JointFeedback::default()
            })
            .collect();
        Self {
            enabled: false,
            frozen: false,
            fail_read: false,
            fail_write: false,
            motor_fault: false,
            writes: 0,
            enables: 0,
            quiet: Vec::new(),
            observed: feedback.clone(),
            feedback,
        }
    }
}

impl Hardware for Loopback {
    fn read(&mut self, now_ms: u64, output: &mut [JointFeedback]) -> Result<(), HardwareError> {
        if self.fail_read || output.len() != self.feedback.len() {
            return Err(HardwareError);
        }
        for i in 0..self.feedback.len() {
            if !self.frozen && !self.quiet.get(i).copied().unwrap_or(false) {
                let joint = &mut self.feedback[i];
                joint.sequence = joint.sequence.checked_add(1).ok_or(HardwareError)?;
                joint.sampled_at_ms = now_ms;
                joint.fault = self.motor_fault;
                self.observed[i] = *joint;
            }
        }
        output.copy_from_slice(&self.observed);
        Ok(())
    }
    fn enable(&mut self) -> Result<(), HardwareError> {
        if self.fail_write {
            return Err(HardwareError);
        }
        self.enabled = true;
        self.enables += 1;
        Ok(())
    }
    fn disable(&mut self) -> Result<(), HardwareError> {
        self.enabled = false;
        for joint in &mut self.feedback {
            joint.velocity = 0.0;
            joint.effort = 0.0;
        }
        if self.fail_write {
            Err(HardwareError)
        } else {
            Ok(())
        }
    }
    fn apply(&mut self, commands: &[JointCommand]) -> Result<(), HardwareError> {
        if self.fail_write || !self.enabled || commands.len() != self.feedback.len() {
            return Err(HardwareError);
        }
        self.writes += 1;
        for (joint, command) in self.feedback.iter_mut().zip(commands) {
            joint.position = command.position;
            joint.velocity = command.velocity;
            joint.effort = command.effort;
        }
        Ok(())
    }
}
