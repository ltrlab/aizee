//! A receiver's high-rate record: every tick's time and, per joint, measured
//! position, velocity and effort and commanded position. Portable (no CAN), so the
//! Linux service and tests share it.
use crate::{hardware::JointFeedback, protocol::JointCommand};

/// Every tick's time and, per joint, measured position, velocity and effort and
/// commanded position, so a client can analyse motion at the receiver's rate. Each
/// status response carries the rows recorded since the previous response. The
/// buffer is allocated once; recording a tick never allocates.
pub struct Recorder {
    width: usize,
    rows: Vec<f64>,
    written: u64,
    reported: u64,
}
impl Recorder {
    pub const CAPACITY: usize = 256;

    pub fn new(joints: usize) -> Self {
        let width = 1 + 4 * joints;
        Self {
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
        for (i, (f, c)) in feedback.iter().zip(commands).enumerate() {
            row[1 + 4 * i..5 + 4 * i]
                .copy_from_slice(&[f.position, f.velocity, f.effort, c.position]);
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
