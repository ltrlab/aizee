//! Receiver-enforced coordinate boxes certified by the local collision supervisor.
//! A box must include all controlled joints and their stopping reserve (none for
//! grippers, which do not brake). The reserve toward a limit is taken at the fastest
//! the joint may approach it from where it is: its speed cap in open travel, less
//! near the limit (a braking curve the receiver enforces on feedback), so a joint
//! can use its travel up to the stopping margin of its approach floor.
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

/// Near a limit a joint may still approach it this fast (or at its cap, if lower):
/// slower readings are drive noise (servoed joints read up to about 0.2 rad/s at
/// rest, and a slowly moving wrist spiked higher). Its stopping distance is part of
/// the margin every joint keeps at each end.
pub const APPROACH_FLOOR_RAD_S: f64 = 0.5;

/// The time a joint may keep moving before the receiver reacts: the verdict's
/// lifetime, the feedback timeout and the reaction allowance.
pub fn reaction_s(policy: &CollisionPolicy, config: &crate::Config) -> f64 {
    (policy.max_verdict_ms + config.feedback_timeout_ms + policy.reaction_ms) as f64 / 1000.0
}

/// The fastest a joint may move toward a limit `distance` away: its speed cap in open
/// travel; nearer, slow enough that after the reaction time and braking at its stop
/// acceleration it stops with its tracking bound still in hand; never below the
/// approach floor (or its cap). Grippers keep their cap: they do not brake.
pub fn approach_speed(
    policy: &CollisionPolicy,
    config: &crate::Config,
    joint: &crate::JointLimits,
    distance: f64,
) -> f64 {
    let cap = joint.max_velocity;
    if joint.gripper {
        return cap;
    }
    let floor = APPROACH_FLOOR_RAD_S.min(cap);
    let room = distance - joint.max_tracking_error;
    if !(room > 0.0) {
        return floor;
    }
    // v * reaction + v^2 / (2 a) = room
    let reaction = reaction_s(policy, config);
    let a = joint.stop_acceleration;
    let v = a * ((reaction * reaction + 2.0 * room / a).sqrt() - reaction);
    v.clamp(floor, cap)
}

/// The time the receiver may take to react to a joint approaching a limit too fast: it checks
/// feedback every tick, so the feedback timeout and the reaction allowance (the collision
/// verdict's lifetime bounds a box, not a limit).
pub fn limit_reaction_s(policy: &CollisionPolicy, config: &crate::Config) -> f64 {
    (config.feedback_timeout_ms + policy.reaction_ms) as f64 / 1000.0
}

/// The fastest a joint may move toward a limit `distance` away and still stop at it: after the
/// limit reaction time, braking at its stop acceleration; its cap in open travel; never below
/// the approach floor (or its cap). The receiver enforces it on feedback while it drives the
/// arm, so a joint uses its travel up to the floor's stopping margin. Grippers keep their cap.
pub fn limit_speed(
    policy: &CollisionPolicy,
    config: &crate::Config,
    joint: &crate::JointLimits,
    distance: f64,
) -> f64 {
    let cap = joint.max_velocity;
    if joint.gripper {
        return cap;
    }
    let floor = APPROACH_FLOOR_RAD_S.min(cap);
    if !(distance > 0.0) {
        return floor;
    }
    // v * reaction + v^2 / (2 a) = distance
    let reaction = limit_reaction_s(policy, config);
    let a = joint.stop_acceleration;
    let v = a * ((reaction * reaction + 2.0 * distance / a).sqrt() - reaction);
    v.clamp(floor, cap)
}

/// How far a joint moving toward a limit `distance` away may travel before it stops: the limit
/// reaction and braking distances at its limit speed there. It equals the distance on the curve
/// and exceeds it only within the floor's stopping margin, where no target may be.
pub fn limit_reserve(
    policy: &CollisionPolicy,
    config: &crate::Config,
    joint: &crate::JointLimits,
    distance: f64,
) -> f64 {
    if joint.gripper {
        return 0.0;
    }
    let v = limit_speed(policy, config, joint, distance);
    v * limit_reaction_s(policy, config) + v * v / (2.0 * joint.stop_acceleration)
}

/// The stopping reserve a box keeps toward a limit `distance` from the joint's extreme
/// position (feedback or target): its tracking bound, then the reaction and braking
/// distances at its approach speed there. In open travel this is the reserve at the cap;
/// nearer the limit it equals the distance (the box reaches the limit); within the
/// floor's stopping margin it exceeds it, which no box can hold.
pub fn reserve_toward(
    policy: &CollisionPolicy,
    config: &crate::Config,
    joint: &crate::JointLimits,
    distance: f64,
) -> f64 {
    if joint.gripper {
        return 0.0;
    }
    let v = approach_speed(policy, config, joint, distance);
    joint.max_tracking_error
        + v * reaction_s(policy, config)
        + v * v / (2.0 * joint.stop_acceleration)
}
