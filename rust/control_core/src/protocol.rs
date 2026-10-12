//! Guarded protocol v1. Times and offers belong to the receiver's boot epoch.
use serde::{Deserialize, Serialize};

pub const PROTOCOL_VERSION: u32 = 1;

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Identity {
    pub model: String,
    pub calibration: String,
    pub collision_policy: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Authority {
    pub epoch: String,
    pub session: String,
    pub lease: u64,
    pub sequence: u64,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct JointCommand {
    pub position: f64,
    pub velocity: f64,
    pub effort: f64,
    pub kp: f64,
    pub kd: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case", deny_unknown_fields)]
pub enum Request {
    Status,
    /// Starts the freshness window BEFORE the client can form a motion command.
    Offer,
    Acquire {
        epoch: String,
        offer: u64,
        session: String,
        group: String,
        identity: Identity,
    },
    Renew {
        authority: Authority,
        offer: u64,
    },
    /// Zero-gain backdrive at measured positions. Never writes a mechanical zero.
    Idle {
        authority: Authority,
    },
    Arm {
        authority: Authority,
        #[serde(default)]
        collision: Option<crate::collision::MotionEnvelope>,
    },
    Move {
        authority: Authority,
        identity: Identity,
        joints: Vec<JointCommand>,
        #[serde(default)]
        collision: Option<crate::collision::MotionEnvelope>,
        /// Per joint, effort added only while the receiver ramps to this command (its
        /// interpolation period), to accelerate the joint to the command's velocity; never
        /// held through a stop or after the ramp. Empty: none.
        #[serde(default)]
        ramp_efforts: Vec<f64>,
    },
    /// Collision-disabled latest target. The previous reply's one-use offer bounds
    /// dispatch age; an accepted update returns the next offer without another RPC.
    DirectUpdate {
        authority: Authority,
        identity: Identity,
        offer: u64,
        /// Expiry relative to offer issuance, conservatively shortened for source age.
        valid_for_ms: u64,
        joints: Vec<JointCommand>,
        #[serde(default)]
        ramp_efforts: Vec<f64>,
        /// Commit these bounded gains for subsequent receiver-owned holds/stops.
        /// Position-only, settled feedback and the unchanged target are required.
        #[serde(default)]
        update_hold_gains: bool,
    },
    Stop {
        authority: Authority,
    },
    Release {
        authority: Authority,
    },
    Reset {
        authority: Authority,
    },
    /// Operator disable: remove torque authority now and release ownership.
    Disable {
        authority: Authority,
    },
    /// Always available regardless of ownership, versioned identities or lease.
    EmergencyStop,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum State {
    Disarmed,
    /// Enabled with zero gains; commands follow measured positions.
    Idle,
    Holding,
    Executing,
    Stopping,
    Stopped,
    Faulted,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Reason {
    Ok,
    Busy,
    WrongEpoch,
    WrongGroup,
    WrongIdentity,
    InvalidSession,
    InvalidOffer,
    OfferExpired,
    WrongOwner,
    WrongLease,
    LeaseExpired,
    CommandExpired,
    Sequence,
    InvalidCommand,
    InvalidState,
    FeedbackStale,
    FeedbackInvalid,
    HardwareFault,
    TrackingError,
    LimitViolation,
    ClockRegression,
    RequestedStop,
    Released,
    EmergencyStop,
    CollisionRequired,
    CollisionExpired,
    CollisionViolation,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Offer {
    pub id: u64,
    pub issued_at_ms: u64,
    pub expires_at_ms: u64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Lease {
    pub session: String,
    pub id: u64,
    pub expires_at_ms: u64,
    pub last_sequence: u64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AppliedTarget {
    pub sequence: u64,
    pub accepted_at_ms: u64,
    pub applied_at_ms: u64,
}

/// The last check that tripped, faulted or refused the receiver: which joint, which check (for
/// example `speed` or `temperature`), the reading and the limit it broke, on the receiver clock.
/// A status's reason alone could not say which of a joint's limits a live session broke (2026-10-12).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Trip {
    pub reason: Reason,
    pub joint: String,
    pub check: String,
    pub value: f64,
    pub limit: f64,
    pub at_ms: u64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Status {
    pub protocol_version: u32,
    pub epoch: String,
    pub group: String,
    pub identity: Identity,
    pub state: State,
    pub joint_names: Vec<String>,
    pub reason: Reason,
    pub now_ms: u64,
    pub lease: Option<Lease>,
    pub command_expires_at_ms: Option<u64>,
    pub collision_expires_at_ms: Option<u64>,
    pub joints: Vec<JointCommand>,
    /// How fast each joint may move [down, up] under a motion collision bound
    /// (collision::Bound::Motion), for a client sizing its boxes: they only fall until the next Move.
    pub speed_bounds: Vec<[f64; 2]>,
    /// Where each joint's reference comes to rest without a new command: the end of the ramp in force,
    /// or of its stop. A client's box covers the way there, as the receiver's check of a command does.
    pub reference_ends: Vec<f64>,
    /// Last target successfully written by the receiver, on the receiver clock.
    /// An acknowledgement may precede this when service writes are deferred.
    pub last_applied_target: Option<AppliedTarget>,
    /// The last trip, fault or refusal a joint's check caused; kept until the next one.
    pub last_trip: Option<Trip>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Reply {
    pub accepted: bool,
    pub reason: Reason,
    pub offer: Option<Offer>,
    pub status: Status,
}
