//! Acceleration-limited independent-joint stop; no jerk or synchronization claim.
use crate::protocol::JointCommand;

#[derive(Debug, Clone, Copy, Default)]
pub struct JointStop {
    position: f64,
    velocity: f64,
    acceleration: f64,
    pub duration_s: f64,
    pub end_position: f64,
}

impl JointStop {
    pub fn new(position: f64, velocity: f64, acceleration: f64) -> Option<Self> {
        if !position.is_finite()
            || !velocity.is_finite()
            || !acceleration.is_finite()
            || acceleration <= 0.0
        {
            return None;
        }
        let duration_s = velocity.abs() / acceleration;
        let end_position = position + 0.5 * velocity * duration_s;
        if !duration_s.is_finite() || !end_position.is_finite() {
            return None;
        }
        Some(Self {
            position,
            velocity,
            acceleration,
            duration_s,
            end_position,
        })
    }

    /// No trajectory: stay at `position` (a gripper keeps its last target).
    pub fn hold(position: f64) -> Self {
        Self {
            position,
            end_position: position,
            ..Self::default()
        }
    }

    pub fn sample(&self, elapsed_s: f64, kp: f64, kd: f64) -> JointCommand {
        let t = elapsed_s.clamp(0.0, self.duration_s);
        let a = -self.velocity.signum() * self.acceleration;
        JointCommand {
            position: if t >= self.duration_s {
                self.end_position
            } else {
                self.position + self.velocity * t + 0.5 * a * t * t
            },
            velocity: if t >= self.duration_s {
                0.0
            } else {
                self.velocity + a * t
            },
            effort: 0.0,
            kp,
            kd,
        }
    }
}
