//! Receiver-enforced coordinate boxes certified by the local collision supervisor.
//! A box must include all controlled joints and their configured stopping reserve
//! (none for grippers, which do not brake).
//! Geometry computation stays outside the periodic loop; configuration determines
//! the trusted local producer and covered collision scope. This is not authentication.
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CollisionPolicy {
    pub world_revision: String,
    pub scope: String,
    pub max_verdict_ms: u64,
    pub reaction_ms: u64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MotionEnvelope {
    /// Single-use receiver offer requested BEFORE computing this verdict.
    pub offer: u64,
    pub world_revision: String,
    pub scope: String,
    pub lower: Vec<f64>,
    pub upper: Vec<f64>,
}

pub fn reserve(
    policy: &CollisionPolicy,
    config: &crate::Config,
    joint: &crate::JointLimits,
) -> f64 {
    if joint.gripper {
        return 0.0;
    }
    let reaction =
        (policy.max_verdict_ms + config.feedback_timeout_ms + policy.reaction_ms) as f64 / 1000.0;
    joint.max_tracking_error
        + joint.max_velocity * reaction
        + joint.max_velocity.powi(2) / (2.0 * joint.stop_acceleration)
}
