//! Acceleration-limited independent-joint stop; no jerk or synchronization claim.
use crate::protocol::JointCommand;

#[derive(Debug, Clone, Copy, Default)]
pub struct JointStop {
    position: f64,
    velocity: f64,
    acceleration: f64,
    /// Held through the stop: the last command's feed-forward, which holds the joint against
    /// gravity (without it a joint held out sags by gravity / kp, past its tracking bound).
    effort: f64,
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
            effort: 0.0,
            duration_s,
            end_position,
        })
    }

    /// Brake to rest within `lower..=upper`: at `acceleration`, or harder where that would carry
    /// the joint past an edge, to stop at it (the drive brakes as hard as its torque allows and
    /// the hold then pulls the joint back). A joint past an edge, or moving on past it, holds at it.
    pub fn within(
        position: f64,
        velocity: f64,
        acceleration: f64,
        lower: f64,
        upper: f64,
    ) -> Option<Self> {
        if !lower.is_finite() || !upper.is_finite() || lower > upper {
            return None;
        }
        let plan = Self::new(position, velocity, acceleration)?;
        if (lower..=upper).contains(&plan.end_position) {
            return Some(plan);
        }
        let edge = if plan.end_position > upper {
            upper
        } else {
            lower
        };
        let room = (edge - position) * velocity.signum();
        if velocity != 0.0 && room > 0.0 {
            if let Some(harder) = Self::new(position, velocity, velocity * velocity / (2.0 * room))
            {
                return Some(Self {
                    end_position: harder.end_position.clamp(lower, upper),
                    ..harder
                });
            }
        }
        Some(Self::hold(position.clamp(lower, upper)))
    }

    /// Keep commanding `effort` while braking and after.
    pub fn with_effort(self, effort: f64) -> Self {
        Self { effort, ..self }
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
            effort: self.effort,
            kp,
            kd,
        }
    }
}
