//! Robot-independent guarded controller. No sockets, CAN types or wall clock.
//!
//! The host MUST call tick() independently of request traffic and give both
//! tick() and handle() the same receiver-local monotonic time. Wire parsing and
//! status serialization belong outside that periodic path. Physical adapters
//! enforce physical pre-enable gates; collision envelopes expire in this core.
pub mod can_adapter;
pub mod collision;
pub mod hardware;
pub mod protocol;
pub mod recorder;
pub mod reference;
pub mod robstride;
#[cfg(all(target_os = "linux", feature = "linux-service"))]
pub mod service;
pub mod service_config;
pub mod stop;
pub mod watchdog;
pub mod watchdog_trial;

use hardware::{Hardware, JointFeedback};
use protocol::*;
use reference::MOVING_READINGS;

/// A joint over its allowed speed faults after MOVING_READINGS samples in a row, or at
/// once beyond this multiple of it. Drives report speed spikes: on Minerva a servoed
/// wrist-roll joint moving slowly read +1.53 rad/s against its 0.8 cap. The allowed
/// speed is the cap, or, while the arm is driven under a collision policy, the joint's
/// approach speed toward the limit it is moving to (collision::approach_speed).
const OVERSPEED_FACTOR: f64 = 3.0;
/// Box edges are compared with this tolerance: near a limit the reserve equals the
/// distance to it, so the box edge is the limit itself, up to rounding.
const EDGE_EPS: f64 = 1e-9;
use serde::{Deserialize, Serialize};
use stop::JointStop;

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum JointUnit {
    Radians,
    Metres,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct JointLimits {
    pub name: String,
    pub unit: JointUnit,
    pub lower: f64,
    pub upper: f64,
    pub max_velocity: f64,
    pub max_effort: f64,
    pub max_kp: f64,
    pub max_kd: f64,
    pub stop_acceleration: f64,
    pub hold_kp: f64,
    pub hold_kd: f64,
    pub max_tracking_error: f64,
    pub max_temperature: f64,
    /// Grippers touch and hold objects. They keep no stopping reserve at their
    /// ends, do not brake on a stop (they keep their last target, so a grasp is
    /// not released) and do not fault on tracking error while stalled on an
    /// object. Position, velocity, effort, gain and temperature limits still apply.
    #[serde(default)]
    pub gripper: bool,
    /// Under a motion collision bound, the tracking bound at rest: it grows with the joint's speed
    /// bound to max_tracking_error at its cap (a servo lags in proportion to its speed), so a joint
    /// at rest keeps a tighter box (collision::tracking_bound). None: max_tracking_error throughout.
    #[serde(default)]
    pub rest_tracking_error: Option<f64>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DirectPolicy {
    pub command_ms: u64,
    pub velocity_timeout_ms: u64,
    pub limit_reaction_ms: u64,
    /// Position targets retain bounded holding/gravity effort, but clients must
    /// not supply velocity feedforward or acceleration effort. Receiver-owned
    /// braking from measured motion remains independent of this command policy.
    #[serde(default)]
    pub position_only: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Config {
    pub group: String,
    pub identity: Identity,
    pub lease_ms: u64,
    pub feedback_timeout_ms: u64,
    pub settled_velocity: f64,
    pub joints: Vec<JointLimits>,
    #[serde(default)]
    pub collision: Option<collision::CollisionPolicy>,
    #[serde(default)]
    pub direct: Option<DirectPolicy>,
    /// Commands arrive this often (ms). A command moving at `velocity` is ramped over it, tick by
    /// tick, from where the previous one pointed (position - velocity x period) to its position,
    /// with that velocity fed forward: smooth motion from a periodic stream. 0: applied as sent.
    #[serde(default)]
    pub interpolation_ms: u64,
}

impl Config {
    fn limit_reaction_s(&self) -> Option<f64> {
        self.direct
            .as_ref()
            .map(|p| p.limit_reaction_ms)
            .or_else(|| self.collision.as_ref().map(|p| p.reaction_ms))
            .map(|ms| (self.feedback_timeout_ms + ms) as f64 / 1000.0)
    }

    pub fn validate(&self) -> Result<(), &'static str> {
        if let Some(policy) = &self.direct {
            if self.collision.is_some()
                || self.identity.collision_policy != "disabled-v1"
                || policy.command_ms == 0
                || policy.command_ms > self.lease_ms
                || policy.velocity_timeout_ms == 0
                || policy.velocity_timeout_ms > policy.command_ms
                || self.interpolation_ms > policy.velocity_timeout_ms
                || policy.limit_reaction_ms > 1000
            {
                return Err("invalid direct policy, collision identity or freshness budget");
            }
        }
        if let Some(policy) = &self.collision {
            if !valid_id(&policy.world_revision)
                || !matches!(policy.scope.as_str(), "all" | "inter_branch")
                || policy.max_verdict_ms == 0
                || policy.max_verdict_ms > self.lease_ms
                || policy.reaction_ms > 1000
            {
                return Err("invalid collision policy, scope or freshness budget");
            }
            // A motion bound covers the reference's ramp: commands must be ramped.
            if policy.bound == collision::Bound::Motion && self.interpolation_ms == 0 {
                return Err("a motion collision bound needs interpolated commands");
            }
        }
        if !valid_id(&self.group)
            || !valid_id(&self.identity.model)
            || !valid_id(&self.identity.calibration)
            || !valid_id(&self.identity.collision_policy)
        {
            return Err(
                "group and revision identifiers must contain 1..256 non-whitespace characters",
            );
        }
        if self.lease_ms == 0
            || self.lease_ms > 1000
            || self.interpolation_ms > self.lease_ms
            || self.feedback_timeout_ms == 0
            || self.feedback_timeout_ms > 1000
            || self.joints.is_empty()
            || self.joints.len() > 256
            || !self.settled_velocity.is_finite()
            || self.settled_velocity < 0.0
        {
            return Err("invalid timing, joint count or settling threshold");
        }
        for (index, j) in self.joints.iter().enumerate() {
            if !valid_id(&j.name)
                || self.joints[..index]
                    .iter()
                    .any(|other| other.name == j.name)
                || !j.lower.is_finite()
                || !j.upper.is_finite()
                || j.lower >= j.upper
                || [
                    j.max_velocity,
                    j.max_effort,
                    j.max_kp,
                    j.max_kd,
                    j.stop_acceleration,
                    j.max_tracking_error,
                    j.max_temperature,
                ]
                .iter()
                .any(|x| !x.is_finite() || *x <= 0.0)
                || !j.hold_kp.is_finite()
                || j.hold_kp < 0.0
                || j.hold_kp > j.max_kp
                || !j.hold_kd.is_finite()
                || j.hold_kd < 0.0
                || j.hold_kd > j.max_kd
                || j.rest_tracking_error
                    .is_some_and(|rest| !(rest > 0.0 && rest <= j.max_tracking_error))
            {
                return Err("invalid joint configuration");
            }
        }
        Ok(())
    }
}

fn valid_id(value: &str) -> bool {
    !value.trim().is_empty() && value.len() <= 256
}

pub struct Controller {
    config: Config,
    epoch: String,
    state: State,
    reason: Reason,
    now_ms: u64,
    lease: Option<Lease>,
    // Keep revoked strings alive until non-periodic acquisition cleans them up.
    retired_lease: Option<Lease>,
    command_deadline: Option<u64>,
    collision_deadline: Option<u64>,
    collision_active: bool,
    collision_lower: Vec<f64>,
    collision_upper: Vec<f64>,
    next_offer: u64,
    offers: [Option<Offer>; 16],
    feedback: Vec<JointFeedback>,
    previous_feedback: Vec<JointFeedback>,
    /// New feedback samples in a row over settled_velocity, per joint. Drives report
    /// speed noise at rest, so one fast reading is not a backdriven arm.
    fast_readings: Vec<u8>,
    /// New feedback samples in a row over each joint's max_velocity.
    overspeed: Vec<u8>,
    /// Under a motion collision bound, how fast each joint may move [down, up]: its overshoot over the
    /// speed its reference is commanded at that way, which after a slower command decays at its stop
    /// acceleration (the joint brakes from the faster one), never below its approach floor.
    speed_bounds: Vec<[f64; 2]>,
    /// When the speed bounds last decayed.
    speed_bounds_at_ms: u64,
    commands: Vec<JointCommand>,
    /// The last accepted Move, which `commands` ramps toward while Executing.
    targets: Vec<JointCommand>,
    /// Its ramp efforts: added to each joint's effort only while it ramps.
    ramp_efforts: Vec<f64>,
    target_at_ms: u64,
    target_sequence: Option<u64>,
    last_applied_target: Option<AppliedTarget>,
    stops: Vec<JointStop>,
    stop_started_ms: u64,
}

impl Controller {
    pub fn new(config: Config, epoch: String) -> Result<Self, &'static str> {
        config.validate()?;
        if !valid_id(&epoch) {
            return Err("a unique nonempty boot epoch is required");
        }
        let count = config.joints.len();
        let resting = config
            .joints
            .iter()
            .map(|joint| collision::speed_bounds(joint, 0.0))
            .collect();
        Ok(Self {
            config,
            epoch,
            state: State::Disarmed,
            reason: Reason::Ok,
            now_ms: 0,
            lease: None,
            retired_lease: None,
            command_deadline: None,
            collision_deadline: None,
            collision_active: false,
            collision_lower: vec![0.0; count],
            collision_upper: vec![0.0; count],
            next_offer: 0,
            offers: std::array::from_fn(|_| None),
            feedback: vec![JointFeedback::default(); count],
            previous_feedback: vec![JointFeedback::default(); count],
            fast_readings: vec![0; count],
            overspeed: vec![0; count],
            speed_bounds: resting,
            speed_bounds_at_ms: 0,
            commands: vec![
                JointCommand {
                    position: 0.0,
                    velocity: 0.0,
                    effort: 0.0,
                    kp: 0.0,
                    kd: 0.0
                };
                count
            ],
            targets: vec![
                JointCommand {
                    position: 0.0,
                    velocity: 0.0,
                    effort: 0.0,
                    kp: 0.0,
                    kd: 0.0
                };
                count
            ],
            ramp_efforts: vec![0.0; count],
            target_at_ms: 0,
            target_sequence: None,
            last_applied_target: None,
            stops: vec![JointStop::default(); count],
            stop_started_ms: 0,
        })
    }

    fn interpolation_s(&self) -> f64 {
        self.config.interpolation_ms as f64 / 1000.0
    }

    /// Ramp the reference toward the last Move: from where its velocity says the stream came from
    /// to its position, over one interpolation period, with its velocity fed forward and its ramp
    /// effort added (to accelerate the joint to that velocity); then hold it, without either.
    fn follow_targets(&mut self) {
        let period = self.interpolation_s();
        let elapsed = (self.now_ms.saturating_sub(self.target_at_ms)) as f64 / 1000.0;
        for (((command, target), limit), ramp) in self
            .commands
            .iter_mut()
            .zip(&self.targets)
            .zip(&self.config.joints)
            .zip(&self.ramp_efforts)
        {
            *command = *target;
            if self.config.direct.as_ref().is_some_and(|p| {
                self.now_ms.saturating_sub(self.target_at_ms) >= p.velocity_timeout_ms
            }) {
                command.velocity = 0.0;
            }
            if period <= 0.0 {
                continue; // No interpolation: applied as sent (no ramp, so no ramp effort).
            }
            if elapsed < period {
                command.position = (target.position - target.velocity * (period - elapsed))
                    .clamp(limit.lower, limit.upper);
                command.effort += ramp;
            } else {
                command.velocity = 0.0;
            }
        }
    }

    pub fn status(&self) -> Status {
        Status {
            protocol_version: PROTOCOL_VERSION,
            last_applied_target: self.last_applied_target.clone(),
            epoch: self.epoch.clone(),
            group: self.config.group.clone(),
            identity: self.config.identity.clone(),
            state: self.state,
            reason: self.reason,
            now_ms: self.now_ms,
            joint_names: self
                .config
                .joints
                .iter()
                .map(|joint| joint.name.clone())
                .collect(),
            lease: self.lease.clone(),
            command_expires_at_ms: self.command_deadline,
            collision_expires_at_ms: self.collision_deadline,
            joints: self.commands.clone(),
            speed_bounds: self.speed_bounds.clone(),
            reference_ends: (0..self.commands.len())
                .map(|i| match self.state {
                    State::Executing => self.targets[i].position,
                    State::Stopping => self.stops[i].end_position,
                    _ => self.commands[i].position,
                })
                .collect(),
        }
    }

    /// The joint commands in force (the last validated targets, or the braking or hold samples).
    pub fn commands(&self) -> &[JointCommand] {
        &self.commands
    }

    pub fn feedback(&self) -> &[JointFeedback] {
        &self.feedback
    }

    fn active(&self) -> bool {
        matches!(
            self.state,
            State::Holding | State::Executing | State::Stopping | State::Stopped
        )
    }

    /// Hardware enabled and receiving periodic commands, including zero-gain Idle.
    fn powered(&self) -> bool {
        self.active() || self.state == State::Idle
    }

    fn zero_gain_at_measured(&mut self) {
        for (command, feedback) in self.commands.iter_mut().zip(&self.feedback) {
            *command = JointCommand {
                position: feedback.position,
                velocity: 0.0,
                effort: 0.0,
                kp: 0.0,
                kd: 0.0,
            };
        }
    }

    /// Remove torque authority without a braking trajectory. Keeps `reason`.
    fn disarm(&mut self, hardware: &mut impl Hardware) -> Result<(), Reason> {
        self.command_deadline = None;
        self.collision_deadline = None;
        self.collision_active = false;
        self.zero_gain_at_measured();
        if hardware.disable().is_err() {
            self.fault(Reason::HardwareFault, hardware);
            return Err(Reason::HardwareFault);
        }
        self.state = State::Disarmed;
        Ok(())
    }

    fn fault(&mut self, reason: Reason, hardware: &mut impl Hardware) {
        self.state = State::Faulted;
        self.reason = reason;
        self.revoke();
        self.command_deadline = None;
        self.collision_deadline = None;
        self.collision_active = false;
        // Best effort. An adapter failure is never interpreted as proof of stop.
        if hardware.disable().is_err() {
            self.reason = Reason::HardwareFault;
        }
    }

    fn revoke(&mut self) {
        if let Some(lease) = self.lease.take() {
            self.retired_lease = Some(lease);
        }
    }

    /// How fast joint `i` may move at `position` in the direction of `velocity`: its cap,
    /// or, while driven under a collision policy, its approach speed toward that limit.
    fn allowed_speed(&self, i: usize, position: f64, velocity: f64) -> f64 {
        let limit = &self.config.joints[i];
        match self.config.limit_reaction_s() {
            Some(reaction) if self.active() => {
                let distance = if velocity > 0.0 {
                    limit.upper - position
                } else {
                    position - limit.lower
                };
                collision::limit_speed_with_reaction(limit, distance, reaction)
            }
            _ => limit.max_velocity,
        }
    }

    /// Whether boxes cover the motion commanded (collision::Bound::Motion), with the joints' speed
    /// bounds enforced.
    fn motion_bound(&self) -> bool {
        self.config
            .collision
            .as_ref()
            .is_some_and(|policy| policy.bound == collision::Bound::Motion)
    }

    /// How fast joint `i` may keep moving: its allowed speed and, while driven under a motion bound,
    /// its speed bound. Readings over it in a row trip (one is drive noise); only its allowed speed
    /// trips at once (a drive spiked to 1.53 rad/s on a slowly moving wrist).
    fn sustained_speed(&self, i: usize, position: f64, velocity: f64) -> f64 {
        let allowed = self.allowed_speed(i, position, velocity);
        if self.motion_bound() && self.active() {
            allowed.min(self.speed_bounds[i][collision::direction(velocity)])
        } else {
            allowed
        }
    }

    /// How far joint `i` may trail or lead its reference: its tracking bound, or under a motion bound
    /// while driven, the share of it for its speed bound (collision::tracking_bound).
    fn tracking_bound(&self, i: usize) -> f64 {
        let limit = &self.config.joints[i];
        if self.motion_bound() && self.active() {
            let [down, up] = self.speed_bounds[i];
            collision::tracking_bound(limit, down.max(up))
        } else {
            limit.max_tracking_error
        }
    }

    /// The joints at rest: each speed bound back to its approach floor.
    fn rest_speed_bounds(&mut self) {
        for (bounds, limit) in self.speed_bounds.iter_mut().zip(&self.config.joints) {
            *bounds = collision::speed_bounds(limit, 0.0);
        }
    }

    /// Each speed bound follows the reference's commanded speed up at once and down no faster than
    /// the joint's stop acceleration, the braking it is qualified for.
    fn decay_speed_bounds(&mut self, now_ms: u64) {
        let dt = now_ms.saturating_sub(self.speed_bounds_at_ms) as f64 / 1000.0;
        self.speed_bounds_at_ms = now_ms;
        for ((bounds, limit), command) in self
            .speed_bounds
            .iter_mut()
            .zip(&self.config.joints)
            .zip(&self.commands)
        {
            let commanded = collision::speed_bounds(limit, command.velocity);
            for (bound, floor) in bounds.iter_mut().zip(commanded) {
                *bound = floor.max(*bound - limit.stop_acceleration * dt);
            }
        }
    }

    fn stale(&self, f: &JointFeedback) -> bool {
        self.now_ms - f.sampled_at_ms >= self.config.feedback_timeout_ms
    }

    /// Invalid feedback, a drive reporting a fault, or no drive answering at all (the bus is gone):
    /// nothing can be held, so these cut torque (a fault).
    fn feedback_error(&self) -> Option<Reason> {
        for f in &self.feedback {
            if f.sequence == 0
                || f.sampled_at_ms > self.now_ms
                || [f.position, f.velocity, f.effort, f.temperature]
                    .iter()
                    .any(|v| !v.is_finite())
            {
                return Some(Reason::FeedbackInvalid);
            }
            if f.fault {
                return Some(Reason::HardwareFault);
            }
        }
        if self.feedback.iter().all(|f| self.stale(f)) {
            return Some(Reason::FeedbackStale);
        }
        None
    }

    /// Some drives, not all, have stopped answering: a trip, holding the arm on those that still do
    /// (each quiet one keeps its last command). The left gripper went quiet for over 30 ms with the
    /// left arm raised, and cutting every joint's torque dropped the arm (2026-09-30).
    fn stale_error(&self) -> Option<Reason> {
        self.feedback
            .iter()
            .any(|f| self.stale(f))
            .then_some(Reason::FeedbackStale)
    }

    /// A joint past a limit, over its allowed speed, effort or temperature, its state known: while
    /// the arm is held this trips it to a braking hold (trip), rather than dropping it.
    fn limit_error(&self) -> Option<Reason> {
        for (i, ((f, limit), &overspeed)) in self
            .feedback
            .iter()
            .zip(&self.config.joints)
            .zip(&self.overspeed)
            .enumerate()
        {
            if f.position < limit.lower
                || f.position > limit.upper
                || overspeed >= MOVING_READINGS
                || f.velocity.abs()
                    > OVERSPEED_FACTOR * self.allowed_speed(i, f.position, f.velocity)
                || f.effort.abs() > limit.max_effort
                || f.temperature > limit.max_temperature
            {
                return Some(Reason::LimitViolation);
            }
        }
        None
    }

    /// A limit, tracking or collision-box trip with the joints' state known. While the arm is held
    /// (holding or executing) it is braked to a hold that keeps its feed-forward, and authority is
    /// revoked as for any stop: the client acquires again and re-arms it where it is held, to lower
    /// it. Cutting torque dropped a raised arm: right J2 fell from horizontal on a near-limit
    /// overspeed (2026-09-29). A stop in progress carries on; nothing is held in Idle or Disarmed,
    /// which fault as before.
    fn trip(&mut self, reason: Reason, hardware: &mut impl Hardware) {
        match self.state {
            State::Holding | State::Executing => self.start_stop(reason, hardware),
            State::Stopping | State::Stopped => {}
            _ => self.fault(reason, hardware),
        }
    }

    /// Where a stop brings joint `i` to rest: within its limits less its approach floor's stopping
    /// margin, where the receiver accepts a target, so a held arm can be re-armed and lowered from
    /// there; and within the collision box in force. The limits alone where those leave no room.
    fn hold_band(&self, i: usize) -> (f64, f64) {
        let limit = &self.config.joints[i];
        let margin = self.config.limit_reaction_s().map_or(0.0, |reaction| {
            if limit.gripper {
                return 0.0;
            }
            let v = collision::limit_speed_with_reaction(limit, 0.0, reaction);
            v * reaction + v * v / (2.0 * limit.stop_acceleration)
        });
        let (mut lower, mut upper) = (limit.lower + margin, limit.upper - margin);
        if lower > upper {
            (lower, upper) = (limit.lower, limit.upper);
        }
        if self.collision_active {
            let low = lower.max(self.collision_lower[i]);
            let high = upper.min(self.collision_upper[i]);
            if low <= high {
                (lower, upper) = (low, high);
            }
        }
        (lower, upper)
    }

    fn start_stop(&mut self, reason: Reason, hardware: &mut impl Hardware) {
        self.revoke();
        self.command_deadline = None;
        self.reason = reason;
        if self.state == State::Idle {
            // Zero-gain backdrive has no trajectory to brake; losing it disables.
            // A disable failure has already latched a hardware fault.
            let _ = self.disarm(hardware);
            return;
        }
        if !self.active() {
            return;
        }
        // Once braking begins, repeated requests must not restart the trajectory.
        if matches!(self.state, State::Stopping | State::Stopped) {
            return;
        }
        if let Some(error) = self.feedback_error() {
            self.fault(error, hardware);
            return;
        }
        for i in 0..self.stops.len() {
            let (f, command) = (self.feedback[i], self.commands[i]);
            // The effort that holds the joint: the last Move's, without the ramp effort that was
            // accelerating it (a stop that kept that would push on in its direction).
            let held = if self.state == State::Executing {
                self.targets[i].effort
            } else {
                command.effort
            };
            let limit = &self.config.joints[i];
            if limit.gripper {
                // No braking trajectory: keep holding the last validated target.
                self.stops[i] = JointStop::hold(command.position);
                continue;
            }
            if self.stale(&f) {
                // Where it is and how fast it moves are unknown: it keeps the command in force.
                self.stops[i] = JointStop::hold(command.position).with_effort(held);
                continue;
            }
            // Brake from at most the speed cap: a faster reading is a drive spike unless it
            // repeats (which trips), and braking from a spike would carry the joint on.
            let speed = f.velocity.clamp(-limit.max_velocity, limit.max_velocity);
            let acceleration = limit.stop_acceleration;
            // At the stop acceleration, or harder where that would carry the joint out of its
            // hold band (it stops at the band's edge rather than faulting and dropping).
            let (lower, upper) = self.hold_band(i);
            let Some(plan) = JointStop::within(f.position, speed, acceleration, lower, upper)
            else {
                self.fault(Reason::InvalidCommand, hardware);
                return;
            };
            // The last command's feed-forward holds the joint against gravity through the stop.
            self.stops[i] = plan.with_effort(held);
        }
        // Keep enforcing the spatial box during stopping and subsequent hold.
        self.collision_deadline = None;
        self.state = State::Stopping;
        self.stop_started_ms = self.now_ms;
    }

    /// Run on the controller schedule even with zero client messages.
    pub fn tick(&mut self, now_ms: u64, hardware: &mut impl Hardware) {
        self.tick_inner(now_ms, hardware, true);
    }

    fn tick_inner(&mut self, now_ms: u64, hardware: &mut impl Hardware, drive: bool) {
        if now_ms < self.now_ms {
            self.fault(Reason::ClockRegression, hardware);
            return;
        }
        self.now_ms = now_ms;
        self.previous_feedback.copy_from_slice(&self.feedback);
        if hardware.read(now_ms, &mut self.feedback).is_err() {
            self.fault(Reason::HardwareFault, hardware);
            return;
        }
        // An unchanged sample cannot be given a younger timestamp or new values.
        for (current, old) in self.feedback.iter().zip(&self.previous_feedback) {
            if current.sequence < old.sequence
                || (current.sequence == old.sequence && current != old)
                || current.sampled_at_ms < old.sampled_at_ms
            {
                self.fault(Reason::FeedbackInvalid, hardware);
                return;
            }
        }
        self.decay_speed_bounds(now_ms);
        // Count each joint's new samples over its settled speed and its speed cap.
        for (i, (current, old)) in self
            .feedback
            .iter()
            .zip(&self.previous_feedback)
            .enumerate()
        {
            if current.sequence != old.sequence {
                let speed = current.velocity.abs();
                self.fast_readings[i] = if speed > self.config.settled_velocity {
                    self.fast_readings[i].saturating_add(1)
                } else {
                    0
                };
                self.overspeed[i] =
                    if speed > self.sustained_speed(i, current.position, current.velocity) {
                        self.overspeed[i].saturating_add(1)
                    } else {
                        0
                    };
            }
        }
        if let Some(error) = self.feedback_error() {
            self.fault(error, hardware);
            return;
        }
        if let Some(error) = self.limit_error().or_else(|| self.stale_error()) {
            self.trip(error, hardware);
        }
        if self.state == State::Faulted {
            return;
        }
        if self.state == State::Executing {
            self.follow_targets();
        }
        if self.collision_active
            && self.active()
            && self.feedback.iter().zip(&self.commands).enumerate().any(
                |(i, (feedback, command))| {
                    feedback.position < self.collision_lower[i]
                        || feedback.position > self.collision_upper[i]
                        || command.position < self.collision_lower[i]
                        || command.position > self.collision_upper[i]
                },
            )
        {
            self.trip(Reason::CollisionViolation, hardware);
        }
        if self.active()
            && self
                .feedback
                .iter()
                .zip(&self.commands)
                .zip(&self.config.joints)
                .enumerate()
                .any(|(i, ((f, c), l))| {
                    !l.gripper && (f.position - c.position).abs() > self.tracking_bound(i)
                })
        {
            self.trip(Reason::TrackingError, hardware);
        }
        if self.state == State::Faulted {
            return;
        }
        if self
            .collision_deadline
            .is_some_and(|deadline| now_ms >= deadline)
        {
            self.start_stop(Reason::CollisionExpired, hardware);
        } else if self
            .lease
            .as_ref()
            .is_some_and(|lease| now_ms >= lease.expires_at_ms)
        {
            self.start_stop(Reason::LeaseExpired, hardware);
        } else if self
            .command_deadline
            .is_some_and(|deadline| now_ms >= deadline)
        {
            // Renewing ownership does not renew an old motion or hold command.
            self.start_stop(Reason::CommandExpired, hardware);
        }
        if self.state == State::Stopping {
            let elapsed = (now_ms - self.stop_started_ms) as f64 / 1000.0;
            let mut complete = true;
            for ((command, stop), limit) in self
                .commands
                .iter_mut()
                .zip(&self.stops)
                .zip(&self.config.joints)
            {
                *command = stop.sample(elapsed, limit.hold_kp, limit.hold_kd);
                complete &= elapsed >= stop.duration_s;
            }
            if complete
                && self
                    .feedback
                    .iter()
                    .all(|f| f.velocity.abs() <= self.config.settled_velocity)
            {
                self.state = State::Stopped;
            }
        }
        if self.state == State::Idle {
            // Zero-gain commands follow the backdriven pose, never an old target.
            for (command, feedback) in self.commands.iter_mut().zip(&self.feedback) {
                command.position = feedback.position;
            }
        }
        if drive && self.powered() {
            if hardware.apply(&self.commands).is_err() {
                self.fault(Reason::HardwareFault, hardware);
            } else {
                self.note_applied();
            }
        }
    }

    fn note_applied(&mut self) {
        if self.state != State::Executing {
            return;
        }
        if let Some(sequence) = self.target_sequence {
            if self.last_applied_target.as_ref().is_none_or(|last| {
                last.sequence != sequence || last.accepted_at_ms != self.target_at_ms
            }) {
                self.last_applied_target = Some(AppliedTarget {
                    sequence,
                    accepted_at_ms: self.target_at_ms,
                    applied_at_ms: self.now_ms,
                });
            }
        }
    }

    fn authorize(&self, authority: &Authority) -> Result<(), Reason> {
        if authority.epoch != self.epoch {
            return Err(Reason::WrongEpoch);
        }
        let Some(lease) = &self.lease else {
            return Err(Reason::WrongOwner);
        };
        if authority.session != lease.session {
            return Err(Reason::WrongOwner);
        }
        if authority.lease != lease.id {
            return Err(Reason::WrongLease);
        }
        if self.now_ms >= lease.expires_at_ms {
            return Err(Reason::LeaseExpired);
        }
        if authority.sequence <= lease.last_sequence {
            return Err(Reason::Sequence);
        }
        Ok(())
    }

    fn consume_sequence(&mut self, authority: &Authority) {
        if let Some(lease) = &mut self.lease {
            lease.last_sequence = authority.sequence;
        }
    }

    fn issue_offer(&mut self) -> Result<Offer, Reason> {
        let id = self.next_offer.checked_add(1).ok_or(Reason::InvalidOffer)?;
        let expires_at_ms = self
            .now_ms
            .checked_add(self.config.lease_ms)
            .ok_or(Reason::InvalidOffer)?;
        self.next_offer = id;
        let offer = Offer {
            id,
            issued_at_ms: self.now_ms,
            expires_at_ms,
        };
        self.offers[id as usize % 16] = Some(offer.clone());
        Ok(offer)
    }

    fn consume_offer(&mut self, id: u64) -> Result<Offer, Reason> {
        let slot = &mut self.offers[id as usize % 16];
        if slot.as_ref().is_none_or(|offer| offer.id != id) {
            return Err(Reason::InvalidOffer);
        }
        let offer = slot.take().ok_or(Reason::InvalidOffer)?;
        if self.now_ms >= offer.expires_at_ms {
            return Err(Reason::OfferExpired);
        }
        Ok(offer)
    }

    /// Validates a command's collision box and consumes its offer: Some((verdict deadline, offer)), or None
    /// with no collision policy.
    fn validate_collision(
        &mut self,
        envelope: Option<&collision::MotionEnvelope>,
        commands: Option<&[JointCommand]>,
    ) -> Result<Option<(u64, Offer)>, Reason> {
        let Some(policy) = &self.config.collision else {
            return if envelope.is_none() {
                Ok(None)
            } else {
                Err(Reason::InvalidCommand)
            };
        };
        let envelope = envelope.ok_or(Reason::CollisionRequired)?;
        if envelope.world_revision != policy.world_revision || envelope.scope != policy.scope {
            return Err(Reason::WrongIdentity);
        }
        if envelope.lower.len() != self.feedback.len()
            || envelope.upper.len() != self.feedback.len()
        {
            return Err(Reason::InvalidCommand);
        }
        let motion = policy.bound == collision::Bound::Motion;
        for (i, (feedback, limit)) in self.feedback.iter().zip(&self.config.joints).enumerate() {
            let (low, high) = (envelope.lower[i], envelope.upper[i]);
            // The reference ramps from where the command's velocity says the stream came from.
            let (target, start, speed) =
                commands.map_or((feedback.position, feedback.position, 0.0), |values| {
                    let command = &values[i];
                    (
                        command.position,
                        command.position - command.velocity * self.interpolation_s(),
                        command.velocity,
                    )
                });
            let (lo, hi, reserve_low, reserve_high) = if motion {
                // The reference the receiver drives from now on: the command in force (it ramps on to
                // its target before this one takes over), then this command's ramp. The joint stays
                // within its tracking bound of it and its speed bound, or trips and stops within its
                // reserve. A gripper may trail its command (stalled on an object): it keeps its
                // measured position in the box too.
                let reference = if self.active() {
                    self.commands[i].position
                } else {
                    feedback.position
                };
                let (mut lo, mut hi) = (
                    reference.min(target).min(start),
                    reference.max(target).max(start),
                );
                if limit.gripper {
                    lo = lo.min(feedback.position);
                    hi = hi.max(feedback.position);
                }
                let commanded = collision::speed_bounds(limit, speed);
                let [down, up] = self.speed_bounds[i];
                let [below, above] = collision::motion_reserve(
                    policy,
                    &self.config,
                    limit,
                    [down.max(commanded[0]), up.max(commanded[1])],
                );
                (lo, hi, below, above)
            } else {
                let lo = feedback.position.min(target).min(start);
                let hi = feedback.position.max(target).max(start);
                // The box holds each side's stopping reserve at the fastest the joint may approach
                // that limit, from wherever the joint is or is driven.
                (
                    lo,
                    hi,
                    collision::reserve_toward(policy, &self.config, limit, lo - limit.lower),
                    collision::reserve_toward(policy, &self.config, limit, limit.upper - hi),
                )
            };
            // The reference the receiver drives (the command's ramp) must stop before the limit on
            // its limit curve. The measured position is held to the limits, the curve and the
            // tracking bound every tick; a joint settling a hair past a command at the edge of its
            // usable range is not refused (right J1's sweep was, 2026-09-29).
            let (driven_lo, driven_hi) = (target.min(start), target.max(start));
            let stop_low =
                collision::limit_reserve(policy, &self.config, limit, driven_lo - limit.lower);
            let stop_high =
                collision::limit_reserve(policy, &self.config, limit, limit.upper - driven_hi);
            if !low.is_finite()
                || !high.is_finite()
                || low >= high
                || !reserve_low.is_finite()
                || !reserve_high.is_finite()
                || low < limit.lower - EDGE_EPS
                || high > limit.upper + EDGE_EPS
                || driven_lo - stop_low < limit.lower - EDGE_EPS
                || driven_hi + stop_high > limit.upper + EDGE_EPS
                || low > (lo - reserve_low).max(limit.lower) + EDGE_EPS
                || high < (hi + reserve_high).min(limit.upper) - EDGE_EPS
            {
                return Err(Reason::CollisionViolation);
            }
        }
        let lifetime = policy.max_verdict_ms;
        let offer = self.consume_offer(envelope.offer)?;
        let deadline = offer
            .issued_at_ms
            .checked_add(lifetime)
            .ok_or(Reason::InvalidOffer)?;
        if self.now_ms >= deadline {
            return Err(Reason::CollisionExpired);
        }
        Ok(Some((deadline, offer)))
    }

    /// An accepted command whose collision verdict consumed a fresh offer renews the lease with that offer, as
    /// Renew does: its sender computed the verdict after the offer was issued, so it is alive and
    /// authorized. A teleop cycle then makes two requests (offer, move), not four (offer, renew, offer, move),
    /// each waiting for a receiver tick. An older offer never shortens the lease.
    fn renew_with(&mut self, offer: &Offer) {
        if let Some(lease) = &mut self.lease {
            if offer.expires_at_ms > lease.expires_at_ms {
                lease.id = offer.id;
                lease.expires_at_ms = offer.expires_at_ms;
            }
        }
    }

    fn install_collision(
        &mut self,
        envelope: Option<&collision::MotionEnvelope>,
        deadline: Option<u64>,
    ) {
        self.collision_deadline = deadline;
        self.collision_active = envelope.is_some();
        if let Some(envelope) = envelope {
            self.collision_lower.copy_from_slice(&envelope.lower);
            self.collision_upper.copy_from_slice(&envelope.upper);
        }
    }

    /// Dispatch from a single owner thread. Parsing/status copies are not RT work.
    pub fn handle(&mut self, now_ms: u64, request: Request, hardware: &mut impl Hardware) -> Reply {
        self.handle_inner(now_ms, request, hardware, true)
    }

    /// Linux receiver path: acknowledge validated targets, apply them on tick().
    /// Arm initializes its hold immediately; emergency/fault disable is immediate.
    /// Polling or flooding requests must not increase the periodic CAN rate.
    pub fn handle_deferred(
        &mut self,
        now_ms: u64,
        request: Request,
        hardware: &mut impl Hardware,
    ) -> Reply {
        self.handle_inner(now_ms, request, hardware, false)
    }

    fn handle_inner(
        &mut self,
        now_ms: u64,
        request: Request,
        hardware: &mut impl Hardware,
        apply_now: bool,
    ) -> Reply {
        self.tick_inner(now_ms, hardware, apply_now);
        let mut offered = None;
        let result = if now_ms < self.now_ms {
            Err(Reason::ClockRegression)
        } else {
            self.dispatch(request, hardware, &mut offered, apply_now)
        };
        Reply {
            accepted: result.is_ok(),
            reason: result.err().unwrap_or(Reason::Ok),
            offer: offered,
            status: self.status(),
        }
    }

    fn dispatch(
        &mut self,
        request: Request,
        hardware: &mut impl Hardware,
        offered: &mut Option<Offer>,
        apply_now: bool,
    ) -> Result<(), Reason> {
        // Normalize only this explicit wire operation into the existing whole-batch
        // command validator. Ordinary Move cannot bypass a direct policy's deadlines.
        let (request, direct_offer) = match request {
            Request::DirectUpdate {
                authority,
                identity,
                offer,
                valid_for_ms,
                joints,
                ramp_efforts,
            } => {
                if self.config.direct.is_none() {
                    return Err(Reason::InvalidCommand);
                }
                if valid_for_ms == 0
                    || valid_for_ms > self.config.direct.as_ref().unwrap().command_ms
                {
                    return Err(Reason::InvalidCommand);
                }
                (
                    Request::Move {
                        authority,
                        identity,
                        joints,
                        collision: None,
                        ramp_efforts,
                    },
                    Some((offer, valid_for_ms)),
                )
            }
            other => (other, None),
        };
        match request {
            Request::DirectUpdate { .. } => unreachable!("normalized above"),
            Request::Status => {}
            Request::Offer => {
                *offered = Some(self.issue_offer()?);
            }
            Request::Acquire {
                epoch,
                offer,
                session,
                group,
                identity,
            } => {
                if epoch != self.epoch {
                    return Err(Reason::WrongEpoch);
                }
                if group != self.config.group {
                    return Err(Reason::WrongGroup);
                }
                if identity != self.config.identity {
                    return Err(Reason::WrongIdentity);
                }
                if !valid_id(&session) {
                    return Err(Reason::InvalidSession);
                }
                if self.lease.is_some() {
                    return Err(Reason::Busy);
                }
                if self.state == State::Stopping {
                    return Err(Reason::InvalidState);
                }
                let offer = self.consume_offer(offer)?;
                self.retired_lease = None;
                self.lease = Some(Lease {
                    session,
                    id: offer.id,
                    expires_at_ms: offer.expires_at_ms,
                    last_sequence: 0,
                });
            }
            Request::Renew { authority, offer } => {
                self.authorize(&authority)?;
                let next = self.consume_offer(offer)?;
                let lease = self.lease.as_mut().ok_or(Reason::WrongOwner)?;
                if next.expires_at_ms <= lease.expires_at_ms {
                    return Err(Reason::InvalidOffer);
                }
                lease.id = next.id;
                lease.expires_at_ms = next.expires_at_ms;
                lease.last_sequence = authority.sequence;
            }
            Request::Idle { authority } => {
                self.authorize(&authority)?;
                // Braking and faults finish first; Idle is not a recovery path.
                if !matches!(
                    self.state,
                    State::Disarmed
                        | State::Idle
                        | State::Holding
                        | State::Executing
                        | State::Stopped
                ) {
                    return Err(Reason::InvalidState);
                }
                if let Some(error) = self
                    .feedback_error()
                    .or_else(|| self.limit_error())
                    .or_else(|| self.stale_error())
                {
                    return Err(error);
                }
                let entering = self.state != State::Idle;
                let enabling = self.state == State::Disarmed;
                // The receiver captures measured positions; clients supply no pose.
                self.zero_gain_at_measured();
                // Enabling or dropping gains is immediate. A refresh cannot raise
                // the periodic write rate on the deferred receiver path.
                if (enabling && hardware.enable().is_err())
                    || ((entering || apply_now) && hardware.apply(&self.commands).is_err())
                {
                    self.fault(Reason::HardwareFault, hardware);
                    return Err(Reason::HardwareFault);
                }
                self.consume_sequence(&authority);
                self.collision_active = false;
                self.collision_deadline = None;
                self.state = State::Idle;
                self.reason = Reason::Ok;
                self.command_deadline = self.lease.as_ref().map(|l| l.expires_at_ms);
            }
            Request::Arm {
                authority,
                collision,
            } => {
                self.authorize(&authority)?;
                if !matches!(self.state, State::Disarmed | State::Idle | State::Stopped) {
                    return Err(Reason::InvalidState);
                }
                if let Some(error) = self
                    .feedback_error()
                    .or_else(|| self.limit_error())
                    .or_else(|| self.stale_error())
                {
                    return Err(error);
                }
                if self.fast_readings.iter().any(|&n| n >= MOVING_READINGS) {
                    return Err(Reason::InvalidState);
                }
                // From a stop the arm is re-armed where the stop holds it: that target and its
                // feed-forward, inside the hold band (captured from the measured pose instead, a
                // raised arm would sag by its gravity / kp, past its tracking bound).
                let resuming = self.state == State::Stopped;
                let held = resuming.then(|| self.commands.clone());
                let verdict = self.validate_collision(collision.as_ref(), held.as_deref())?;
                self.install_collision(
                    collision.as_ref(),
                    verdict.as_ref().map(|(deadline, _)| *deadline),
                );
                if !resuming {
                    // Capture measured positions, never an old command or a zero default.
                    for ((command, feedback), limit) in self
                        .commands
                        .iter_mut()
                        .zip(&self.feedback)
                        .zip(&self.config.joints)
                    {
                        *command = JointCommand {
                            position: feedback.position,
                            velocity: 0.0,
                            effort: 0.0,
                            kp: limit.hold_kp,
                            kd: limit.hold_kd,
                        };
                    }
                }
                // Idle already consumed the adapter's enable permit; never re-enable.
                let enabling = self.state == State::Disarmed;
                if (enabling && hardware.enable().is_err())
                    || hardware.apply(&self.commands).is_err()
                {
                    self.fault(Reason::HardwareFault, hardware);
                    return Err(Reason::HardwareFault);
                }
                self.consume_sequence(&authority);
                if let Some((_, offer)) = &verdict {
                    self.renew_with(offer);
                }
                self.rest_speed_bounds();
                self.state = State::Holding;
                self.reason = Reason::Ok;
                self.command_deadline = self.lease.as_ref().map(|l| l.expires_at_ms);
            }
            Request::Move {
                authority,
                identity,
                joints,
                collision,
                ramp_efforts,
            } => {
                self.authorize(&authority)?;
                if self.config.direct.is_some() != direct_offer.is_some() {
                    return Err(Reason::InvalidCommand);
                }
                if !matches!(self.state, State::Holding | State::Executing) {
                    return Err(Reason::InvalidState);
                }
                if identity != self.config.identity {
                    return Err(Reason::WrongIdentity);
                }
                if joints.len() != self.commands.len()
                    || !(ramp_efforts.is_empty() || ramp_efforts.len() == joints.len())
                {
                    return Err(Reason::InvalidCommand);
                }
                // Enforce the selected control law across the complete batch,
                // including grippers, before consuming its offer or sequence.
                if self.config.direct.as_ref().is_some_and(|p| p.position_only)
                    && (joints.iter().any(|command| command.velocity != 0.0)
                        || ramp_efforts.iter().any(|effort| *effort != 0.0))
                {
                    return Err(Reason::InvalidCommand);
                }
                // Validate the ENTIRE batch before any target changes.
                for (command, limit) in joints.iter().zip(&self.config.joints) {
                    if [
                        command.position,
                        command.velocity,
                        command.effort,
                        command.kp,
                        command.kd,
                    ]
                    .iter()
                    .any(|v| !v.is_finite())
                        || command.position < limit.lower
                        || command.position > limit.upper
                        || command.velocity.abs() > limit.max_velocity
                        || command.effort.abs() > limit.max_effort
                        || command.kp < 0.0
                        || command.kp > limit.max_kp
                        || command.kd < 0.0
                        || command.kd > limit.max_kd
                    {
                        return Err(Reason::InvalidCommand);
                    }
                    if let Some(policy) = &self.config.direct {
                        if !limit.gripper {
                            let reaction =
                                (self.config.feedback_timeout_ms + policy.limit_reaction_ms) as f64
                                    / 1000.0;
                            let v = collision::limit_speed_with_reaction(limit, 0.0, reaction);
                            let margin = v * reaction + v * v / (2.0 * limit.stop_acceleration);
                            let distance = if command.velocity > 0.0 {
                                limit.upper - command.position
                            } else {
                                command.position - limit.lower
                            };
                            let start =
                                command.position - command.velocity * self.interpolation_s();
                            if command.position.min(start) < limit.lower + margin - EDGE_EPS
                                || command.position.max(start) > limit.upper - margin + EDGE_EPS
                                || command.velocity.abs()
                                    > collision::limit_speed_with_reaction(
                                        limit, distance, reaction,
                                    ) + EDGE_EPS
                            {
                                return Err(Reason::InvalidCommand);
                            }
                        }
                    }
                }
                // A ramp effort is finite and, with the command's own, within the effort limit.
                for ((command, limit), ramp) in joints
                    .iter()
                    .zip(&self.config.joints)
                    .zip(ramp_efforts.iter().chain(std::iter::repeat(&0.0)))
                {
                    if !ramp.is_finite() || (command.effort + ramp).abs() > limit.max_effort {
                        return Err(Reason::InvalidCommand);
                    }
                }
                let direct = if let Some((id, valid_for_ms)) = direct_offer {
                    let offer = self.consume_offer(id)?;
                    let deadline = offer
                        .issued_at_ms
                        .saturating_add(valid_for_ms)
                        .min(offer.expires_at_ms);
                    if self.now_ms >= deadline {
                        return Err(Reason::OfferExpired);
                    }
                    Some((deadline, offer))
                } else {
                    None
                };
                let verdict = self.validate_collision(collision.as_ref(), Some(&joints))?;
                self.install_collision(
                    collision.as_ref(),
                    verdict.as_ref().map(|(deadline, _)| *deadline),
                );
                for ((bounds, limit), command) in self
                    .speed_bounds
                    .iter_mut()
                    .zip(&self.config.joints)
                    .zip(&joints)
                {
                    let commanded = collision::speed_bounds(limit, command.velocity);
                    for (bound, floor) in bounds.iter_mut().zip(commanded) {
                        *bound = bound.max(floor);
                    }
                }
                self.targets.copy_from_slice(&joints);
                if ramp_efforts.is_empty() {
                    self.ramp_efforts.fill(0.0);
                } else {
                    self.ramp_efforts.copy_from_slice(&ramp_efforts);
                }
                self.target_at_ms = self.now_ms;
                self.target_sequence = Some(authority.sequence);
                self.follow_targets();
                self.consume_sequence(&authority);
                if let Some((_, offer)) = &verdict {
                    self.renew_with(offer);
                }
                if let Some((_, offer)) = &direct {
                    self.renew_with(offer);
                }
                self.command_deadline = direct
                    .as_ref()
                    .map(|(deadline, _)| *deadline)
                    .or_else(|| self.lease.as_ref().map(|l| l.expires_at_ms));
                self.state = State::Executing;
                if apply_now && hardware.apply(&self.commands).is_err() {
                    self.fault(Reason::HardwareFault, hardware);
                    return Err(Reason::HardwareFault);
                }
                if apply_now {
                    self.note_applied();
                }
                if direct.is_some() {
                    *offered = Some(self.issue_offer()?);
                }
            }
            Request::Stop { authority } => {
                self.authorize(&authority)?;
                self.start_stop(Reason::RequestedStop, hardware);
            }
            Request::Release { authority } => {
                self.authorize(&authority)?;
                self.start_stop(Reason::Released, hardware);
            }
            Request::Reset { authority } => {
                self.authorize(&authority)?;
                if !matches!(self.state, State::Faulted | State::Stopped) {
                    return Err(Reason::InvalidState);
                }
                if let Some(error) = self
                    .feedback_error()
                    .or_else(|| self.limit_error())
                    .or_else(|| self.stale_error())
                {
                    return Err(error);
                }
                if self.fast_readings.iter().any(|&n| n >= MOVING_READINGS) {
                    return Err(Reason::InvalidState);
                }
                if hardware.disable().is_err() {
                    self.fault(Reason::HardwareFault, hardware);
                    return Err(Reason::HardwareFault);
                }
                self.consume_sequence(&authority);
                self.state = State::Disarmed;
                self.collision_active = false;
                self.collision_deadline = None;
                self.reason = Reason::Ok;
            }
            Request::Disable { authority } => {
                self.authorize(&authority)?;
                // Operator intent: no braking trajectory and no automatic rearm.
                // The next session must acquire again from Disarmed.
                self.revoke();
                if self.state == State::Faulted {
                    // Only Reset clears a fault; still retry removing torque.
                    if hardware.disable().is_err() {
                        self.reason = Reason::HardwareFault;
                        return Err(Reason::HardwareFault);
                    }
                    return Ok(());
                }
                self.reason = Reason::Ok;
                self.disarm(hardware)?;
            }
            Request::EmergencyStop => {
                self.fault(Reason::EmergencyStop, hardware);
            }
        }
        Ok(())
    }
}
