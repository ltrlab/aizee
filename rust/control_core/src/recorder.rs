//! A receiver's high-rate record: every tick's time, each joint's measured position,
//! velocity and effort, and the whole command its drive was given. Portable (no CAN), so
//! the Linux service and tests share it.
use crate::{hardware::JointFeedback, protocol::JointCommand};

/// Every tick's time and, per joint, measured position, velocity and effort and commanded
/// position; then, per joint, the rest of the drive's command: velocity, effort, kp and kd.
/// The second block lets a client replay exactly what each drive was told (an actuator
/// response fit needs the feed-forward and gains, not only the position). Columns of the
/// first block keep their places from before the second existed. Each status response
/// carries the rows recorded since the previous response. The buffer is allocated once;
/// recording a tick never allocates.
pub struct Recorder {
    joints: usize,
    width: usize,
    rows: Vec<f64>,
    written: u64,
    reported: u64,
}
impl Recorder {
    pub const CAPACITY: usize = 256;
    /// Measured position, velocity, effort and commanded position.
    pub const MEASURED_FIELDS: usize = 4;
    /// Commanded velocity, effort, kp and kd.
    pub const COMMAND_FIELDS: usize = 4;

    pub fn new(joints: usize) -> Self {
        let width = 1 + (Self::MEASURED_FIELDS + Self::COMMAND_FIELDS) * joints;
        Self {
            joints,
            width,
            rows: vec![0.0; Self::CAPACITY * width],
            written: 0,
            reported: 0,
        }
    }

    pub fn record(&mut self, t_ms: f64, feedback: &[JointFeedback], commands: &[JointCommand]) {
        let start = (self.written % Self::CAPACITY as u64) as usize * self.width;
        let row = &mut self.rows[start..start + self.width];
        row[0] = t_ms;
        let (measured, commanded) = row[1..].split_at_mut(Self::MEASURED_FIELDS * self.joints);
        for (i, (f, c)) in feedback.iter().zip(commands).enumerate() {
            measured[Self::MEASURED_FIELDS * i..Self::MEASURED_FIELDS * (i + 1)]
                .copy_from_slice(&[f.position, f.velocity, f.effort, c.position]);
            commanded[Self::COMMAND_FIELDS * i..Self::COMMAND_FIELDS * (i + 1)]
                .copy_from_slice(&[c.velocity, c.effort, c.kp, c.kd]);
        }
        self.written += 1;
    }

    /// The rows not yet reported, oldest first, and how many were overwritten unreported.
    pub fn take(&mut self) -> (Vec<&[f64]>, u64) {
        let oldest = self.written.saturating_sub(Self::CAPACITY as u64);
        let dropped = oldest.saturating_sub(self.reported);
        let first = self.reported.max(oldest);
        self.reported = self.written;
        let rows = (first..self.written)
            .map(|n| {
                let start = (n % Self::CAPACITY as u64) as usize * self.width;
                &self.rows[start..start + self.width]
            })
            .collect();
        (rows, dropped)
    }
}
