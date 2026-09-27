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
}

impl Config {
    pub fn validate(&self) -> Result<(), &'static str> {
        if let Some(policy) = &self.collision {
            if !valid_id(&policy.world_revision)
                || !matches!(policy.scope.as_str(), "all" | "inter_branch")
                || policy.max_verdict_ms == 0
                || policy.max_verdict_ms > self.lease_ms
                || policy.reaction_ms > 1000
            {
                return Err("invalid collision policy, scope or freshness budget");
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
    commands: Vec<JointCommand>,
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
            stops: vec![JointStop::default(); count],
            stop_started_ms: 0,
        })
    }

    pub fn status(&self) -> Status {
        Status {
            protocol_version: PROTOCOL_VERSION,
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
        }
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

    fn feedback_error(&self) -> Option<Reason> {
        for (f, limit) in self.feedback.iter().zip(&self.config.joints) {
            if f.sequence == 0
                || f.sampled_at_ms > self.now_ms
                || [f.position, f.velocity, f.effort, f.temperature]
                    .iter()
                    .any(|v| !v.is_finite())
            {
                return Some(Reason::FeedbackInvalid);
            }
            if self.now_ms - f.sampled_at_ms >= self.config.feedback_timeout_ms {
                return Some(Reason::FeedbackStale);
            }
            if f.fault {
                return Some(Reason::HardwareFault);
            }
            if f.position < limit.lower
                || f.position > limit.upper
                || f.velocity.abs() > limit.max_velocity
                || f.effort.abs() > limit.max_effort
                || f.temperature > limit.max_temperature
            {
                return Some(Reason::LimitViolation);
            }
        }
        None
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
        for (((f, limit), stop), command) in self
            .feedback
            .iter()
            .zip(&self.config.joints)
            .zip(&mut self.stops)
            .zip(&self.commands)
        {
            if limit.gripper {
                // No braking trajectory: keep holding the last validated target.
                *stop = JointStop::hold(command.position);
                continue;
            }
            let Some(plan) = JointStop::new(f.position, f.velocity, limit.stop_acceleration) else {
                self.fault(Reason::InvalidCommand, hardware);
                return;
            };
            if plan.end_position < limit.lower || plan.end_position > limit.upper {
                self.fault(Reason::LimitViolation, hardware);
                return;
            }
            *stop = plan;
        }
        if self.collision_active
            && self.stops.iter().enumerate().any(|(i, stop)| {
                stop.end_position < self.collision_lower[i]
                    || stop.end_position > self.collision_upper[i]
            })
        {
            self.fault(Reason::CollisionViolation, hardware);
            return;
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
        if let Some(error) = self.feedback_error() {
            self.fault(error, hardware);
            return;
        }
        for ((fast, current), old) in self
            .fast_readings
            .iter_mut()
            .zip(&self.feedback)
            .zip(&self.previous_feedback)
        {
            if current.sequence != old.sequence {
                *fast = if current.velocity.abs() > self.config.settled_velocity {
                    fast.saturating_add(1)
                } else {
                    0
                };
            }
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
            self.fault(Reason::CollisionViolation, hardware);
            return;
        }
        if self.active()
            && self
                .feedback
                .iter()
                .zip(&self.commands)
                .zip(&self.config.joints)
                .any(|((f, c), l)| {
                    !l.gripper && (f.position - c.position).abs() > l.max_tracking_error
                })
        {
            self.fault(Reason::TrackingError, hardware);
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
        if drive && self.powered() && hardware.apply(&self.commands).is_err() {
            self.fault(Reason::HardwareFault, hardware);
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

    fn validate_collision(
        &mut self,
        envelope: Option<&collision::MotionEnvelope>,
        commands: Option<&[JointCommand]>,
    ) -> Result<Option<u64>, Reason> {
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
        for (i, (feedback, limit)) in self.feedback.iter().zip(&self.config.joints).enumerate() {
            let (low, high) = (envelope.lower[i], envelope.upper[i]);
            let target = commands.map_or(feedback.position, |values| values[i].position);
            let reserve = collision::reserve(policy, &self.config, limit);
            if !low.is_finite()
                || !high.is_finite()
                || low >= high
                || !reserve.is_finite()
                || low < limit.lower
                || high > limit.upper
                || low > feedback.position.min(target) - reserve
                || high < feedback.position.max(target) + reserve
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
        Ok(Some(deadline))
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
        match request {
            Request::Status => {}
            Request::Offer => {
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
                *offered = Some(offer);
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
                if let Some(error) = self.feedback_error() {
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
                if !matches!(self.state, State::Disarmed | State::Idle) {
                    return Err(Reason::InvalidState);
                }
                if let Some(error) = self.feedback_error() {
                    return Err(error);
                }
                if self.fast_readings.iter().any(|&n| n >= MOVING_READINGS) {
                    return Err(Reason::InvalidState);
                }
                let deadline = self.validate_collision(collision.as_ref(), None)?;
                self.install_collision(collision.as_ref(), deadline);
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
                // Idle already consumed the adapter's enable permit; never re-enable.
                let enabling = self.state == State::Disarmed;
                if (enabling && hardware.enable().is_err())
                    || hardware.apply(&self.commands).is_err()
                {
                    self.fault(Reason::HardwareFault, hardware);
                    return Err(Reason::HardwareFault);
                }
                self.consume_sequence(&authority);
                self.state = State::Holding;
                self.reason = Reason::Ok;
                self.command_deadline = self.lease.as_ref().map(|l| l.expires_at_ms);
            }
            Request::Move {
                authority,
                identity,
                joints,
                collision,
            } => {
                self.authorize(&authority)?;
                if !matches!(self.state, State::Holding | State::Executing) {
                    return Err(Reason::InvalidState);
                }
                if identity != self.config.identity {
                    return Err(Reason::WrongIdentity);
                }
                if joints.len() != self.commands.len() {
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
                        || (self.config.collision.is_some()
                            && (command.velocity != 0.0 || command.effort != 0.0))
                    {
                        return Err(Reason::InvalidCommand);
                    }
                }
                let deadline = self.validate_collision(collision.as_ref(), Some(&joints))?;
                self.install_collision(collision.as_ref(), deadline);
                self.commands.copy_from_slice(&joints);
                self.consume_sequence(&authority);
                self.command_deadline = self.lease.as_ref().map(|l| l.expires_at_ms);
                self.state = State::Executing;
                if apply_now && hardware.apply(&self.commands).is_err() {
                    self.fault(Reason::HardwareFault, hardware);
                    return Err(Reason::HardwareFault);
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
                if let Some(error) = self.feedback_error() {
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
