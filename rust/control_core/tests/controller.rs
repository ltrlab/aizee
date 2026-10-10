use robot_control::{
    hardware::Loopback, protocol::*, stop::JointStop, Config, Controller, JointLimits, JointUnit,
};

fn identity() -> Identity {
    Identity {
        model: "fixture-model".into(),
        calibration: "synthetic".into(),
        collision_policy: "fixture-policy".into(),
    }
}
fn config(count: usize) -> Config {
    Config {
        direct: None,
        collision: None,
        group: "arbitrary-group".into(),
        identity: identity(),
        lease_ms: 100,
        feedback_timeout_ms: 20,
        interpolation_ms: 0,
        settled_velocity: 0.001,
        joints: (0..count)
            .map(|i| JointLimits {
                name: format!("axis-{i}"),
                unit: JointUnit::Radians,
                lower: -3.0,
                upper: 3.0,
                max_velocity: 2.0,
                max_effort: 5.0,
                max_kp: 20.0,
                max_kd: 5.0,
                stop_acceleration: 2.0,
                hold_kp: 10.0,
                hold_kd: 1.0,
                max_tracking_error: 0.5,
                max_temperature: 70.0,
                gripper: false,
                rest_tracking_error: None,
            })
            .collect(),
    }
}
fn fixture(count: usize) -> (Controller, Loopback) {
    (
        Controller::new(config(count), "boot-a".into()).unwrap(),
        Loopback::new(&vec![0.0; count]),
    )
}
fn offer(c: &mut Controller, h: &mut Loopback, now: u64) -> u64 {
    c.handle(now, Request::Offer, h).offer.unwrap().id
}
fn claim(c: &mut Controller, h: &mut Loopback, now: u64) -> Reply {
    let id = offer(c, h, now);
    c.handle(
        now,
        Request::Acquire {
            epoch: "boot-a".into(),
            offer: id,
            session: "client-a".into(),
            group: "arbitrary-group".into(),
            identity: identity(),
        },
        h,
    )
}
fn auth(c: &Controller, sequence: u64) -> Authority {
    let status = c.status();
    let lease = status.lease.unwrap();
    Authority {
        epoch: status.epoch,
        session: lease.session,
        lease: lease.id,
        sequence,
    }
}
fn armed(count: usize) -> (Controller, Loopback) {
    let (mut c, mut h) = fixture(count);
    assert!(claim(&mut c, &mut h, 0).accepted);
    assert!(
        c.handle(
            1,
            Request::Arm {
                collision: None,
                authority: auth(&c, 1)
            },
            &mut h
        )
        .accepted
    );
    (c, h)
}
fn joints(count: usize, q: f64, v: f64) -> Vec<JointCommand> {
    vec![
        JointCommand {
            position: q,
            velocity: v,
            effort: 0.0,
            kp: 10.0,
            kd: 1.0
        };
        count
    ]
}
fn move_request(c: &Controller, sequence: u64, count: usize, q: f64, v: f64) -> Request {
    Request::Move {
        collision: None,
        authority: auth(c, sequence),
        identity: identity(),
        ramp_efforts: Vec::new(),
        joints: joints(count, q, v),
    }
}

fn collision_fixture() -> (Controller, Loopback) {
    let mut cfg = config(2);
    cfg.collision = Some(robot_control::collision::CollisionPolicy {
        world_revision: "fixed-world".into(),
        scope: "inter_branch".into(),
        max_verdict_ms: 40,
        reaction_ms: 10,
        bound: Default::default(),
    });
    let mut c = Controller::new(cfg, "boot-a".into()).unwrap();
    let mut h = Loopback::new(&[0.0, 0.0]);
    assert!(claim(&mut c, &mut h, 0).accepted);
    (c, h)
}

fn direct_armed() -> (Controller, Loopback) {
    direct_armed_with_timeout(10)
}

fn direct_armed_with_timeout(timeout: u64) -> (Controller, Loopback) {
    direct_armed_with_policy(timeout, false)
}

fn direct_armed_with_policy(timeout: u64, position_only: bool) -> (Controller, Loopback) {
    let mut cfg = config(2);
    cfg.identity.collision_policy = "disabled-v1".into();
    cfg.direct = Some(robot_control::DirectPolicy {
        command_ms: 30,
        velocity_timeout_ms: timeout,
        limit_reaction_ms: 10,
        position_only,
    });
    direct_armed_config(cfg)
}

fn direct_armed_config(cfg: Config) -> (Controller, Loopback) {
    let mut c = Controller::new(cfg, "boot-a".into()).unwrap();
    let mut h = Loopback::new(&[0.0, 0.0]);
    let id = offer(&mut c, &mut h, 0);
    let identity = c.status().identity;
    assert!(
        c.handle(
            0,
            Request::Acquire {
                epoch: "boot-a".into(),
                offer: id,
                session: "client-a".into(),
                group: "arbitrary-group".into(),
                identity
            },
            &mut h
        )
        .accepted
    );
    assert!(
        c.handle(
            1,
            Request::Arm {
                authority: auth(&c, 1),
                collision: None
            },
            &mut h
        )
        .accepted
    );
    (c, h)
}

fn direct_request(c: &Controller, sequence: u64, offer: u64, q: f64, v: f64) -> Request {
    Request::DirectUpdate {
        authority: auth(c, sequence),
        identity: c.status().identity,
        offer,
        valid_for_ms: 30,
        joints: joints(2, q, v),
        ramp_efforts: vec![],
    }
}

#[test]
fn direct_position_only_is_explicit_and_old_policy_keeps_feedforward() {
    let legacy = serde_json::json!({
        "command_ms": 30, "velocity_timeout_ms": 10, "limit_reaction_ms": 10
    });
    let policy: robot_control::DirectPolicy = serde_json::from_value(legacy.clone()).unwrap();
    assert!(!policy.position_only);
    let mut selected = legacy;
    selected["position_only"] = true.into();
    let policy: robot_control::DirectPolicy = serde_json::from_value(selected.clone()).unwrap();
    assert!(policy.position_only);
    selected["position_only"] = "true".into();
    assert!(serde_json::from_value::<robot_control::DirectPolicy>(selected).is_err());

    let (mut c, mut h) = direct_armed();
    let id = offer(&mut c, &mut h, 2);
    let mut request = direct_request(&c, 2, id, 0.01, 0.1);
    if let Request::DirectUpdate { ramp_efforts, .. } = &mut request {
        *ramp_efforts = vec![0.2, -0.2];
    }
    assert!(c.handle(3, request, &mut h).accepted);
    assert_eq!(c.status().joints[0].velocity, 0.1);
}

#[test]
fn position_only_rejects_whole_nonzero_feedforward_batches_without_consuming_authority() {
    for deferred in [false, true] {
        for gripper in [false, true] {
            for velocity in [false, true] {
                for nonzero in [-f64::EPSILON, f64::EPSILON] {
                    let mut cfg = config(2);
                    cfg.identity.collision_policy = "disabled-v1".into();
                    cfg.joints[1].gripper = gripper;
                    cfg.direct = Some(robot_control::DirectPolicy {
                        command_ms: 30,
                        velocity_timeout_ms: 10,
                        limit_reaction_ms: 10,
                        position_only: true,
                    });
                    let (mut c, mut h) = direct_armed_config(cfg);
                    let id = offer(&mut c, &mut h, 2);
                    let initial = c.handle(3, direct_request(&c, 2, id, 0.01, 0.0), &mut h);
                    assert!(initial.accepted);
                    let before = c.status();
                    let lease = before.lease.as_ref().unwrap();
                    let next = initial.offer.unwrap().id;
                    let mut invalid = direct_request(&c, 3, next, 0.02, 0.0);
                    if let Request::DirectUpdate {
                        joints,
                        ramp_efforts,
                        ..
                    } = &mut invalid
                    {
                        // First joint is valid and moves, but the final joint
                        // rejects the entire batch, even for tiny signed inputs.
                        if velocity {
                            joints[1].velocity = nonzero;
                        } else {
                            *ramp_efforts = vec![0.0, nonzero];
                        }
                    }
                    let writes = h.writes;
                    let reply = if deferred {
                        c.handle_deferred(4, invalid, &mut h)
                    } else {
                        c.handle(4, invalid, &mut h)
                    };
                    assert!(!reply.accepted);
                    assert_eq!(reply.reason, Reason::InvalidCommand);
                    assert!(reply.offer.is_none());
                    assert_eq!(reply.status.joints, before.joints);
                    assert_eq!(
                        reply.status.command_expires_at_ms,
                        before.command_expires_at_ms
                    );
                    let after_lease = reply.status.lease.unwrap();
                    assert_eq!(after_lease.id, lease.id);
                    assert_eq!(after_lease.expires_at_ms, lease.expires_at_ms);
                    assert_eq!(after_lease.last_sequence, lease.last_sequence);
                    assert_eq!(h.feedback[0].position, 0.01);
                    assert_eq!(h.feedback[1].position, 0.01);
                    if deferred {
                        assert_eq!(h.writes, writes);
                    }
                    // Correction can use exactly the rejected sequence/offer;
                    // the bad request neither consumed nor renewed authority.
                    assert!(
                        c.handle(5, direct_request(&c, 3, next, 0.02, 0.0), &mut h)
                            .accepted
                    );
                }
            }
        }
    }
}

#[test]
fn position_only_accepts_gravity_and_gains_without_weakening_expiry_or_plain_move_rejection() {
    let (mut c, mut h) = direct_armed_with_policy(10, true);
    let mut id = offer(&mut c, &mut h, 2);
    for (now, sequence, ramp) in [(3, 2, vec![]), (8, 3, vec![0.0, -0.0])] {
        let mut request = direct_request(&c, sequence, id, 0.01 * sequence as f64, -0.0);
        if let Request::DirectUpdate {
            joints,
            ramp_efforts,
            ..
        } = &mut request
        {
            joints[0].effort = 1.2;
            joints[1].effort = -0.8;
            *ramp_efforts = ramp;
        }
        let reply = c.handle_deferred(now, request, &mut h);
        assert!(reply.accepted, "{:?}", reply.reason);
        id = reply.offer.unwrap().id;
        c.tick(now + 1, &mut h);
        let status = c.status();
        assert_eq!(status.joints[0].position, 0.01 * sequence as f64);
        assert_eq!(status.joints[0].effort, 1.2);
        assert_eq!(status.joints[1].effort, -0.8);
        assert!(status
            .joints
            .iter()
            .all(|j| j.velocity == 0.0 && j.kp == 10.0 && j.kd == 1.0));
        assert_eq!(status.last_applied_target.unwrap().sequence, sequence);
    }
    let before = c.status();
    for velocity in [0.0, 0.1] {
        let mut request = move_request(&c, 4, 2, 0.04, velocity);
        if let Request::Move { identity, .. } = &mut request {
            *identity = c.status().identity;
        }
        assert_eq!(c.handle(10, request, &mut h).reason, Reason::InvalidCommand);
    }
    assert_eq!(
        c.status().command_expires_at_ms,
        before.command_expires_at_ms
    );
    assert_eq!(c.status().lease.unwrap().last_sequence, 3);
    c.tick(before.command_expires_at_ms.unwrap(), &mut h);
    assert_eq!(c.status().reason, Reason::CommandExpired);
    assert!(c.status().lease.is_none());
    assert!(c.handle(34, Request::EmergencyStop, &mut h).accepted);
    assert!(!h.enabled);
}

#[test]
fn position_only_does_not_disable_receiver_owned_braking_from_measured_motion() {
    let (mut c, mut h) = direct_armed_with_policy(10, true);
    let id = offer(&mut c, &mut h, 2);
    let mut request = direct_request(&c, 2, id, 0.01, 0.0);
    if let Request::DirectUpdate { joints, .. } = &mut request {
        joints[0].effort = 1.2;
    }
    assert!(c.handle(3, request, &mut h).accepted);
    h.feedback[0].velocity = 0.2;
    let reply = c.handle_deferred(
        4,
        Request::Stop {
            authority: auth(&c, 3),
        },
        &mut h,
    );
    assert!(reply.accepted);
    assert_eq!(reply.status.reason, Reason::RequestedStop);
    assert_eq!(reply.status.state, State::Stopping);
    assert!(reply.status.lease.is_none());
    c.tick(5, &mut h);
    let command = c.status().joints[0];
    assert!(command.position > 0.01);
    assert!(command.velocity > 0.0 && command.velocity < 0.2);
    assert_eq!(command.effort, 1.2);
}

#[test]
fn direct_updates_chain_one_rpc_and_keep_receiver_issued_deadlines() {
    let (mut c, mut h) = direct_armed();
    let mut id = offer(&mut c, &mut h, 2);
    for n in 1..100 {
        let now = 2 + n * 10;
        let request = direct_request(&c, n + 1, id, 0.0, 0.0);
        let reply = c.handle(now, request, &mut h);
        assert!(reply.accepted, "{:?}", reply.reason);
        assert_eq!(reply.status.command_expires_at_ms, Some(now + 20));
        assert!(
            reply.status.lease.unwrap().expires_at_ms < reply.offer.as_ref().unwrap().expires_at_ms
        );
        id = reply.offer.unwrap().id;
    }
}

#[test]
fn direct_replay_invalid_batch_and_delayed_offer_cannot_change_targets_or_deadlines() {
    let (mut c, mut h) = direct_armed();
    let id = offer(&mut c, &mut h, 2);
    let request = direct_request(&c, 2, id, 0.01, 0.0);
    let accepted = c.handle(3, request.clone(), &mut h);
    assert!(accepted.accepted);
    assert!(!c.handle(4, request, &mut h).accepted);
    assert!(
        !c.handle(5, direct_request(&c, 3, id, 0.0, 0.0), &mut h)
            .accepted
    );
    let next = accepted.offer.unwrap().id;
    assert_eq!(
        c.handle(6, direct_request(&c, 3, next, 4.0, 0.0), &mut h)
            .reason,
        Reason::InvalidCommand
    );
    assert_eq!(c.status().command_expires_at_ms, Some(32));
    assert_eq!(c.status().lease.unwrap().last_sequence, 2);
    assert!(
        c.handle(7, direct_request(&c, 3, next, 0.01, 0.0), &mut h)
            .accepted
    );
    // Fresh lease renewal must not extend the old direct target.
    let fresh = offer(&mut c, &mut h, 20);
    assert!(
        c.handle(
            20,
            Request::Renew {
                authority: auth(&c, 4),
                offer: fresh
            },
            &mut h
        )
        .accepted
    );
    assert_eq!(c.status().command_expires_at_ms, Some(33));
    c.tick(33, &mut h);
    assert_eq!(c.status().reason, Reason::CommandExpired);

    let (mut c, mut h) = direct_armed();
    let old = offer(&mut c, &mut h, 2);
    assert_eq!(
        c.handle(32, direct_request(&c, 2, old, 0.0, 0.0), &mut h)
            .reason,
        Reason::OfferExpired
    );
    assert_eq!(c.status().lease.unwrap().last_sequence, 1);
}

#[test]
fn direct_velocity_expires_without_another_request_and_plain_move_is_rejected() {
    let (mut c, mut h) = direct_armed();
    let id = offer(&mut c, &mut h, 2);
    assert!(
        c.handle(3, direct_request(&c, 2, id, 0.01, 0.1), &mut h)
            .accepted
    );
    assert_eq!(c.status().joints[0].velocity, 0.1);
    c.tick(13, &mut h);
    assert_eq!(c.status().joints[0].velocity, 0.0);
    let mut request = move_request(&c, 3, 2, 0.01, 0.0);
    if let Request::Move { identity, .. } = &mut request {
        *identity = c.status().identity;
    }
    assert_eq!(c.handle(14, request, &mut h).reason, Reason::InvalidCommand);
    assert!(c.handle(15, Request::EmergencyStop, &mut h).accepted);
    assert!(c.status().lease.is_none());
}

#[test]
fn direct_mode_cannot_bypass_collision_policy_or_joint_limit_margin() {
    let (mut c, mut h) = armed(2);
    let id = offer(&mut c, &mut h, 2);
    assert_eq!(
        c.handle(3, direct_request(&c, 2, id, 0.0, 0.0), &mut h)
            .reason,
        Reason::InvalidCommand
    );
    let (mut c, mut h) = direct_armed();
    let id = offer(&mut c, &mut h, 2);
    assert_eq!(
        c.handle(3, direct_request(&c, 2, id, 2.99, 0.0), &mut h)
            .reason,
        Reason::InvalidCommand
    );
    assert_eq!(c.status().lease.unwrap().last_sequence, 1);
}

#[test]
fn direct_deferred_writes_keep_feedforward_across_a_period_and_report_application() {
    let (mut c, mut h) = direct_armed_with_timeout(15);
    let id = offer(&mut c, &mut h, 2);
    let before = h.writes;
    let first = c.handle_deferred(3, direct_request(&c, 2, id, 0.01, 0.1), &mut h);
    assert!(first.accepted && first.status.last_applied_target.is_none());
    assert_eq!(h.writes, before);
    c.tick(5, &mut h);
    assert_eq!(c.status().last_applied_target.unwrap().applied_at_ms, 5);
    c.tick(13, &mut h); // Service writes before accepting the next 100 Hz update.
    assert_eq!(c.status().joints[0].velocity, 0.1);
    let next = c.handle_deferred(
        13,
        direct_request(&c, 3, first.offer.unwrap().id, 0.02, 0.1),
        &mut h,
    );
    assert!(next.accepted);
    assert_eq!(next.status.last_applied_target.unwrap().sequence, 2);
    c.tick(15, &mut h);
    let applied = c.status().last_applied_target.unwrap();
    assert_eq!(
        (
            applied.sequence,
            applied.accepted_at_ms,
            applied.applied_at_ms
        ),
        (3, 13, 15)
    );
    c.tick(28, &mut h);
    assert_eq!(c.status().joints[0].velocity, 0.0);
}

#[test]
fn direct_shortened_source_window_and_feedback_timeout_survive_transport_stalls() {
    let (mut c, mut h) = direct_armed();
    let id = offer(&mut c, &mut h, 2);
    let mut request = direct_request(&c, 2, id, 0.0, 0.0);
    if let Request::DirectUpdate { valid_for_ms, .. } = &mut request {
        *valid_for_ms = 7;
    }
    assert_eq!(c.handle(9, request, &mut h).reason, Reason::OfferExpired);
    assert_eq!(c.status().lease.unwrap().last_sequence, 1);
    let id = offer(&mut c, &mut h, 10);
    assert!(
        c.handle(10, direct_request(&c, 2, id, 0.0, 0.0), &mut h)
            .accepted
    );
    h.frozen = true;
    c.tick(31, &mut h);
    assert_eq!(c.status().reason, Reason::FeedbackStale);
    assert!(!h.enabled);
}

fn envelope(
    c: &mut Controller,
    h: &mut Loopback,
    now: u64,
) -> robot_control::collision::MotionEnvelope {
    robot_control::collision::MotionEnvelope {
        offer: offer(c, h, now),
        world_revision: "fixed-world".into(),
        scope: "inter_branch".into(),
        lower: vec![-2.5; 2],
        upper: vec![2.5; 2],
    }
}

#[test]
fn collision_policy_requires_a_verdict_before_arm_or_move() {
    let (mut c, mut h) = collision_fixture();
    assert_eq!(
        c.handle(
            1,
            Request::Arm {
                authority: auth(&c, 1),
                collision: None
            },
            &mut h
        )
        .reason,
        Reason::CollisionRequired
    );
    assert!(!h.enabled);
    let checked = envelope(&mut c, &mut h, 2);
    assert!(
        c.handle(
            2,
            Request::Arm {
                authority: auth(&c, 1),
                collision: Some(checked)
            },
            &mut h
        )
        .accepted
    );
    let writes = h.writes;
    assert_eq!(
        c.handle_deferred(3, move_request(&c, 2, 2, 0.1, 0.), &mut h)
            .reason,
        Reason::CollisionRequired
    );
    assert_eq!(h.writes, writes);
    assert_eq!(c.status().joints[0].position, 0.);
}

#[test]
fn collision_expiry_survives_lease_renewal_and_stops_without_client_messages() {
    let (mut c, mut h) = collision_fixture();
    let checked = envelope(&mut c, &mut h, 1);
    assert!(
        c.handle(
            1,
            Request::Arm {
                authority: auth(&c, 1),
                collision: Some(checked)
            },
            &mut h
        )
        .accepted
    );
    let checked = envelope(&mut c, &mut h, 2);
    assert!(
        c.handle(
            2,
            Request::Move {
                authority: auth(&c, 2),
                identity: identity(),
                ramp_efforts: Vec::new(),
                joints: joints(2, 0.1, 0.0),
                collision: Some(checked)
            },
            &mut h
        )
        .accepted
    );
    let renewal = offer(&mut c, &mut h, 20);
    assert!(
        c.handle(
            20,
            Request::Renew {
                authority: auth(&c, 3),
                offer: renewal
            },
            &mut h
        )
        .accepted
    );
    assert_eq!(c.status().collision_expires_at_ms, Some(42));
    // Position-only setpoints still require braking from measured velocity.
    for sample in &mut h.feedback {
        sample.velocity = 0.2;
    }
    c.tick(42, &mut h);
    assert_eq!(c.status().reason, Reason::CollisionExpired);
    assert_eq!(c.status().state, State::Stopping);
    assert!(c.status().lease.is_none());
    for now in 43..160 {
        c.tick(now, &mut h);
    }
    assert_eq!(c.status().state, State::Stopped);
    assert!((c.status().joints[0].position - 0.11).abs() < 1e-10);
    // Pushed out of the box while held, the joint is held (pulled back), not dropped.
    h.feedback[0].position = 2.6;
    c.tick(160, &mut h);
    assert_eq!(c.status().state, State::Stopped);
    assert!(h.enabled);
    assert!((c.status().joints[0].position - 0.11).abs() < 1e-10);
}

#[test]
fn collision_offer_cannot_be_delayed_reused_or_given_a_new_lifetime() {
    let (mut c, mut h) = collision_fixture();
    let late = envelope(&mut c, &mut h, 0);
    assert_eq!(
        c.handle(
            40,
            Request::Arm {
                authority: auth(&c, 1),
                collision: Some(late)
            },
            &mut h
        )
        .reason,
        Reason::CollisionExpired
    );
    let checked = envelope(&mut c, &mut h, 41);
    assert!(
        c.handle(
            41,
            Request::Arm {
                authority: auth(&c, 1),
                collision: Some(checked.clone())
            },
            &mut h
        )
        .accepted
    );
    assert_eq!(
        c.handle_deferred(
            42,
            Request::Move {
                authority: auth(&c, 2),
                identity: identity(),
                ramp_efforts: Vec::new(),
                joints: joints(2, 0.1, 0.),
                collision: Some(checked)
            },
            &mut h
        )
        .reason,
        Reason::InvalidOffer
    );
    assert_eq!(c.status().collision_expires_at_ms, Some(81));
}

#[test]
fn collision_envelopes_validate_scope_world_dimensions_finiteness_and_stopping_reserve() {
    for failure in 0..7 {
        let (mut c, mut h) = collision_fixture();
        let mut checked = envelope(&mut c, &mut h, 1);
        match failure {
            0 => checked.world_revision = "other-world".into(),
            1 => checked.scope = "all".into(),
            2 => {
                checked.upper.pop();
            }
            3 => checked.lower[0] = f64::NAN,
            4 => checked.lower[0] = -0.1, // contains current pose but lacks stopping reserve
            5 => checked.upper[1] = 0.1,
            _ => checked.upper[0] = 4., // outside operational limits
        }
        let result = c.handle(
            1,
            Request::Arm {
                authority: auth(&c, 1),
                collision: Some(checked),
            },
            &mut h,
        );
        assert!(!result.accepted);
        assert!(!h.enabled);
        assert_eq!(h.writes, 0);
    }
}

#[test]
fn deferred_requests_do_not_accelerate_periodic_writes_or_defer_emergency_stop() {
    let (mut c, mut h) = armed(1);
    let before = h.writes;
    let request = move_request(&c, 2, 1, 0.1, 0.0);
    assert!(c.handle_deferred(2, request, &mut h).accepted);
    for now in 3..10 {
        assert!(c.handle_deferred(now, Request::Status, &mut h).accepted);
    }
    assert_eq!(h.writes, before);
    assert_eq!(h.feedback[0].position, 0.0);
    c.tick(10, &mut h);
    assert_eq!(h.writes, before + 1);
    assert_eq!(h.feedback[0].position, 0.1);
    c.handle_deferred(11, Request::EmergencyStop, &mut h);
    assert!(!h.enabled);
    assert_eq!(h.writes, before + 1);
}

#[test]
fn acquire_never_enables_and_owner_is_exclusive() {
    let (mut c, mut h) = fixture(3);
    assert!(claim(&mut c, &mut h, 0).accepted);
    assert!(!h.enabled);
    assert_eq!(claim(&mut c, &mut h, 1).reason, Reason::Busy);
    assert_eq!(c.status().state, State::Disarmed);
}

#[test]
fn delayed_offer_and_acquire_cannot_start_new_window() {
    let (mut c, mut h) = fixture(1);
    let id = offer(&mut c, &mut h, 0);
    let request = Request::Acquire {
        epoch: "boot-a".into(),
        offer: id,
        session: "client".into(),
        group: "arbitrary-group".into(),
        identity: identity(),
    };
    assert_eq!(c.handle(100, request, &mut h).reason, Reason::OfferExpired);
    assert!(!h.enabled);
    assert!(c.status().lease.is_none());
}

#[test]
fn permit_expiry_is_issuance_time_not_arrival_time() {
    let (mut c, mut h) = fixture(1);
    let id = offer(&mut c, &mut h, 0);
    let r = c.handle(
        80,
        Request::Acquire {
            epoch: "boot-a".into(),
            offer: id,
            session: "client-a".into(),
            group: "arbitrary-group".into(),
            identity: identity(),
        },
        &mut h,
    );
    assert!(r.accepted);
    assert_eq!(r.status.lease.unwrap().expires_at_ms, 100);
    c.handle(
        81,
        Request::Arm {
            collision: None,
            authority: auth(&c, 1),
        },
        &mut h,
    );
    let queued = move_request(&c, 2, 1, 0.1, 0.0);
    assert!(!c.handle(100, queued, &mut h).accepted);
    assert_eq!(c.status().reason, Reason::LeaseExpired);
}

#[test]
fn rejects_duplicates_reordering_wrong_owner_and_previous_boot() {
    let (mut c, mut h) = armed(2);
    let request = move_request(&c, 4, 2, 0.1, 0.0);
    assert!(c.handle(2, request.clone(), &mut h).accepted);
    assert_eq!(c.handle(3, request, &mut h).reason, Reason::Sequence);
    let old = move_request(&c, 3, 2, 0.2, 0.0);
    assert_eq!(c.handle(4, old, &mut h).reason, Reason::Sequence);
    let mut wrong = auth(&c, 5);
    wrong.session = "client-b".into();
    assert_eq!(
        c.handle(
            5,
            Request::Arm {
                collision: None,
                authority: wrong.clone()
            },
            &mut h
        )
        .reason,
        Reason::WrongOwner
    );
    wrong.epoch = "previous-boot".into();
    assert_eq!(
        c.handle(
            6,
            Request::Arm {
                authority: wrong,
                collision: None
            },
            &mut h
        )
        .reason,
        Reason::WrongEpoch
    );
    assert_eq!(c.status().joints[0].position, 0.1);
}

#[test]
fn entire_batch_and_identity_are_validated_before_mutation() {
    let (mut c, mut h) = armed(2);
    let before = c.status().joints;
    for bad in [f64::NAN, f64::INFINITY, 4.0] {
        let mut values = joints(2, 0.1, 0.0);
        values[1].position = bad;
        let r = c.handle(
            2,
            Request::Move {
                collision: None,
                authority: auth(&c, 2),
                identity: identity(),
                ramp_efforts: Vec::new(),
                joints: values,
            },
            &mut h,
        );
        assert_eq!(r.reason, Reason::InvalidCommand);
        assert_eq!(c.status().joints, before);
        assert_eq!(h.feedback[0].position, 0.0);
    }
    let bad_length = move_request(&c, 2, 1, 0.1, 0.0);
    assert_eq!(
        c.handle(2, bad_length, &mut h).reason,
        Reason::InvalidCommand
    );
    let mut changed = identity();
    changed.calibration = "other".into();
    let r = c.handle(
        2,
        Request::Move {
            collision: None,
            authority: auth(&c, 2),
            identity: changed,
            joints: joints(2, 0.1, 0.0),
            ramp_efforts: Vec::new(),
        },
        &mut h,
    );
    assert_eq!(r.reason, Reason::WrongIdentity);
    assert_eq!(c.status().lease.unwrap().last_sequence, 1);
}

#[test]
fn rejected_motion_does_not_renew_deadline() {
    let (mut c, mut h) = armed(1);
    let r = move_request(&c, 2, 1, 10.0, 0.0);
    assert!(!c.handle(90, r, &mut h).accepted);
    c.tick(100, &mut h);
    assert_eq!(c.status().reason, Reason::LeaseExpired);
    assert!(c.status().lease.is_none());
}

#[test]
fn renewal_does_not_extend_old_motion_and_invalidates_old_permit() {
    let (mut c, mut h) = armed(1);
    let old_command = move_request(&c, 3, 1, 0.1, 0.0);
    let id = offer(&mut c, &mut h, 50);
    let r = c.handle(
        60,
        Request::Renew {
            authority: auth(&c, 2),
            offer: id,
        },
        &mut h,
    );
    assert!(r.accepted);
    assert_eq!(r.status.lease.unwrap().expires_at_ms, 150);
    assert_eq!(r.status.command_expires_at_ms, Some(100));
    assert_eq!(c.handle(70, old_command, &mut h).reason, Reason::WrongLease);
    c.tick(100, &mut h);
    assert_eq!(c.status().reason, Reason::CommandExpired);
    assert!(c.status().lease.is_none());
}

#[test]
fn new_motion_after_renewal_gets_only_new_offer_deadline() {
    let (mut c, mut h) = armed(1);
    let id = offer(&mut c, &mut h, 50);
    c.handle(
        55,
        Request::Renew {
            authority: auth(&c, 2),
            offer: id,
        },
        &mut h,
    );
    let r = move_request(&c, 3, 1, 0.1, 0.0);
    assert!(c.handle(60, r, &mut h).accepted);
    c.tick(110, &mut h);
    assert_eq!(c.status().state, State::Executing);
    c.tick(150, &mut h);
    assert_eq!(c.status().reason, Reason::LeaseExpired);
}

#[test]
fn expiry_stops_locally_and_the_owner_re_arms_where_the_stop_holds() {
    let (mut c, mut h) = armed(2);
    let r = move_request(&c, 2, 2, 0.1, 0.2);
    assert!(c.handle(2, r, &mut h).accepted);
    c.tick(100, &mut h);
    assert_eq!(c.status().state, State::Stopping);
    c.tick(150, &mut h);
    assert!((c.status().joints[0].velocity - 0.1).abs() < 1e-10);
    c.tick(200, &mut h);
    c.tick(201, &mut h);
    assert_eq!(c.status().state, State::Stopped);
    assert!((c.status().joints[0].position - 0.11).abs() < 1e-10);
    // A new owner re-arms it where the stop holds it: no new enable (the permit is single-use),
    // no limp moment; then it may move it again.
    assert!(claim(&mut c, &mut h, 202).accepted);
    let reply = c.handle(
        203,
        Request::Arm {
            collision: None,
            authority: auth(&c, 1),
        },
        &mut h,
    );
    assert!(reply.accepted, "{:?}", reply.reason);
    assert_eq!(c.status().state, State::Holding);
    assert!((c.status().joints[0].position - 0.11).abs() < 1e-10);
    assert!(h.enabled);
    assert_eq!(h.enables, 1);
    let r = move_request(&c, 2, 2, 0.0, 0.0);
    assert!(c.handle(204, r, &mut h).accepted);
    // Reset still disarms a stopped arm; Arm then enables it from Disarmed.
    let (mut c, mut h) = armed(2);
    let r = move_request(&c, 2, 2, 0.1, 0.2);
    assert!(c.handle(2, r, &mut h).accepted);
    for now in [100, 150, 200, 201] {
        c.tick(now, &mut h);
    }
    assert_eq!(c.status().state, State::Stopped);
    assert!(claim(&mut c, &mut h, 202).accepted);
    assert!(
        c.handle(
            204,
            Request::Reset {
                authority: auth(&c, 1)
            },
            &mut h
        )
        .accepted
    );
    assert!(!h.enabled);
    assert_eq!(c.status().state, State::Disarmed);
    assert!(
        c.handle(
            205,
            Request::Arm {
                collision: None,
                authority: auth(&c, 2)
            },
            &mut h
        )
        .accepted
    );
}

#[test]
fn emergency_stop_bypasses_ownership_and_never_auto_resumes() {
    let (mut c, mut h) = armed(1);
    let queued = move_request(&c, 2, 1, 0.1, 0.0);
    assert!(c.handle(2, Request::EmergencyStop, &mut h).accepted);
    assert!(!h.enabled);
    assert_eq!(c.status().state, State::Faulted);
    assert!(!c.handle(3, queued, &mut h).accepted);
    c.tick(200, &mut h);
    assert_eq!(c.status().state, State::Faulted);
    assert!(!h.enabled);
}

#[test]
fn frozen_feedback_and_hardware_failures_revoke_authority() {
    for kind in 0..4 {
        let (mut c, mut h) = armed(1);
        match kind {
            0 => h.frozen = true,
            1 => h.fail_read = true,
            2 => h.fail_write = true,
            _ => h.motor_fault = true,
        }
        c.tick(21, &mut h);
        assert_eq!(c.status().state, State::Faulted, "fault kind {kind}");
        assert_eq!(
            c.status().reason,
            if kind == 0 {
                Reason::FeedbackStale
            } else {
                Reason::HardwareFault
            }
        );
        assert!(c.status().lease.is_none());
        assert!(!h.enabled);
    }
}

#[test]
fn one_overspeed_reading_is_drive_noise_but_a_run_of_them_trips_to_a_hold() {
    let (mut c, mut h) = armed(1);
    h.feedback[0].velocity = 2.5; // Over the 2.0 rad/s cap once...
    c.tick(2, &mut h);
    h.feedback[0].velocity = 0.0;
    c.tick(3, &mut h);
    assert_eq!(c.status().state, State::Holding);
    for now in 4..7 {
        h.feedback[0].velocity = 2.5; // ...then three samples in a row.
        c.tick(now, &mut h);
    }
    // Braked to a hold, not dropped: torque stays on and authority is revoked, as for a stop.
    assert_eq!(c.status().reason, Reason::LimitViolation);
    assert_eq!(c.status().state, State::Stopping);
    assert!(c.status().lease.is_none());
    assert!(h.enabled);
}

#[test]
fn a_reading_far_over_the_speed_cap_trips_at_once() {
    let (mut c, mut h) = armed(1);
    h.feedback[0].velocity = 6.5; // Over three times the 2.0 rad/s cap.
    c.tick(2, &mut h);
    assert_eq!(c.status().reason, Reason::LimitViolation);
    assert_eq!(c.status().state, State::Stopping);
    assert!(h.enabled);
}

#[test]
fn a_trip_holds_a_raised_joint_with_its_feed_forward_and_its_owner_lowers_it() {
    // Right J2, 2026-09-29: raised near horizontal with 19 Nm of gravity fed forward, a run of
    // readings over its speed near a limit cut its torque and it fell. Now it is braked to a
    // hold that keeps the feed-forward; a new owner re-arms it there and moves it down.
    let (mut c, mut h) = armed(1);
    let mut raised = joints(1, 1.5, 0.0);
    raised[0].effort = 4.0; // Within the fixture's 5.0 limit.
    let r = Request::Move {
        collision: None,
        authority: auth(&c, 2),
        identity: identity(),
        ramp_efforts: Vec::new(),
        joints: raised,
    };
    assert!(c.handle(2, r, &mut h).accepted);
    for now in 3..6 {
        h.feedback[0].velocity = 2.5;
        c.tick(now, &mut h);
    }
    assert_eq!(c.status().state, State::Stopping);
    assert!(h.enabled);
    for now in 6..2000 {
        c.tick(now, &mut h);
    }
    assert_eq!(c.status().state, State::Stopped);
    let held = c.status().joints[0];
    assert!(h.enabled);
    assert_eq!(held.effort, 4.0);
    assert_eq!((held.kp, held.kd), (10.0, 1.0));
    assert!(claim(&mut c, &mut h, 2000).accepted);
    let reply = c.handle(
        2001,
        Request::Arm {
            collision: None,
            authority: auth(&c, 1),
        },
        &mut h,
    );
    assert!(reply.accepted, "{:?}", reply.reason);
    assert_eq!(c.status().joints[0], held);
    let r = move_request(&c, 2, 1, held.position - 0.01, 0.0);
    assert!(c.handle(2002, r, &mut h).accepted);
    assert_eq!(c.status().state, State::Executing);
}

#[test]
fn lost_feedback_or_a_drive_fault_still_cuts_torque() {
    let (mut c, mut h) = armed(1);
    h.motor_fault = true;
    c.tick(2, &mut h);
    assert_eq!(c.status().state, State::Faulted);
    assert!(!h.enabled);
}

#[test]
fn braking_starts_from_at_most_the_speed_cap() {
    // Braking from a lone 2.5 rad/s spike would carry the joint 1.56 rad on;
    // from the 2.0 rad/s cap it is 1.0 rad at 2 rad/s^2.
    let (mut c, mut h) = armed(1);
    h.feedback[0].velocity = 2.5;
    let reply = c.handle(
        2,
        Request::Stop {
            authority: auth(&c, 2),
        },
        &mut h,
    );
    assert!(reply.accepted, "{:?}", reply.reason);
    assert_eq!(c.status().state, State::Stopping);
    for now in 3..1500 {
        c.tick(now, &mut h);
    }
    assert_eq!(c.status().state, State::Stopped);
    assert!((c.status().joints[0].position - 1.0).abs() < 1e-9);
}

#[test]
fn clock_regression_latches_a_fault_and_a_stop_that_would_overrun_brakes_harder() {
    let (mut c, mut h) = armed(1);
    c.tick(0, &mut h);
    assert_eq!(c.status().reason, Reason::ClockRegression);
    assert!(!h.enabled);
    // At 2 rad/s^2 a joint at 2.99 moving 1 rad/s would stop 0.25 rad on, past the limit at 3:
    // it brakes harder, to stop at the limit, rather than cutting torque.
    let (mut c, mut h) = armed(1);
    let r = move_request(&c, 2, 1, 2.99, 1.0);
    c.handle(2, r, &mut h);
    c.tick(100, &mut h);
    assert_eq!(c.status().reason, Reason::LeaseExpired);
    assert_eq!(c.status().state, State::Stopping);
    assert!(h.enabled);
    for now in 101..200 {
        c.tick(now, &mut h);
    }
    assert!(c.status().joints[0].position <= 3.0);
    assert!((c.status().joints[0].position - 3.0).abs() < 1e-9);
}

#[test]
fn config_rejects_duplicate_names_nonfinite_and_invalid_gains() {
    let mut c = config(2);
    c.joints[1].name = c.joints[0].name.clone();
    assert!(c.validate().is_err());
    c = config(1);
    c.joints[0].stop_acceleration = f64::NAN;
    assert!(c.validate().is_err());
    c = config(1);
    c.joints[0].hold_kp = 21.0;
    assert!(c.validate().is_err());
}

#[test]
fn stopping_equations_cover_both_directions_and_zero() {
    for velocity in [-2.0, 0.0, 2.0] {
        let stop = JointStop::new(0.5, velocity, 4.0).unwrap();
        let end = stop.sample(100.0, 1.0, 1.0);
        assert_eq!(end.velocity, 0.0);
        assert_eq!(end.position, 0.5 + velocity * velocity.abs() / 8.0);
        assert_eq!(stop.sample(0.0, 1.0, 1.0).velocity, velocity);
    }
    assert!(JointStop::new(0.0, 1.0, 0.0).is_none());
}

fn idle_request(c: &Controller, sequence: u64) -> Request {
    Request::Idle {
        authority: auth(c, sequence),
    }
}

fn disable_request(c: &Controller, sequence: u64) -> Request {
    Request::Disable {
        authority: auth(c, sequence),
    }
}

#[test]
fn idle_is_zero_gain_backdrive_that_follows_the_measured_pose() {
    let (mut c, mut h) = fixture(2);
    assert!(claim(&mut c, &mut h, 0).accepted);
    h.feedback[0].position = 0.4;
    let reply = c.handle(1, idle_request(&c, 1), &mut h);
    assert!(reply.accepted);
    assert_eq!(reply.status.state, State::Idle);
    assert!(h.enabled);
    assert_eq!(h.enables, 1);
    assert_eq!(reply.status.joints[0].position, 0.4);
    assert!(reply
        .status
        .joints
        .iter()
        .all(|j| j.kp == 0.0 && j.kd == 0.0 && j.velocity == 0.0 && j.effort == 0.0));
    // Backdriving far beyond the tracking bound is not a fault with zero gains.
    h.feedback[1].position = -1.2;
    c.tick(2, &mut h);
    let status = c.status();
    assert_eq!(status.state, State::Idle);
    assert_eq!(status.joints[1].position, -1.2);
    assert_eq!(status.joints[1].kp, 0.0);
}

#[test]
fn enable_from_idle_holds_measured_pose_without_a_second_enable() {
    let (mut c, mut h) = fixture(1);
    assert!(claim(&mut c, &mut h, 0).accepted);
    assert!(c.handle(1, idle_request(&c, 1), &mut h).accepted);
    h.feedback[0].position = 0.25;
    c.tick(2, &mut h);
    let reply = c.handle(
        3,
        Request::Arm {
            collision: None,
            authority: auth(&c, 2),
        },
        &mut h,
    );
    assert!(reply.accepted);
    assert_eq!(reply.status.state, State::Holding);
    assert_eq!(h.enables, 1);
    assert_eq!(reply.status.joints[0].position, 0.25);
    assert_eq!(reply.status.joints[0].kp, 10.0);
}

#[test]
fn enable_from_idle_is_refused_while_backdriven() {
    let (mut c, mut h) = fixture(1);
    assert!(claim(&mut c, &mut h, 0).accepted);
    assert!(c.handle(1, idle_request(&c, 1), &mut h).accepted);
    // Backdriven: every new sample reads fast (Idle's zero-gain apply resets the loopback).
    for now in 2..5 {
        h.feedback[0].velocity = 0.5;
        c.tick(now, &mut h);
    }
    h.feedback[0].velocity = 0.5;
    let reply = c.handle(
        5,
        Request::Arm {
            collision: None,
            authority: auth(&c, 2),
        },
        &mut h,
    );
    assert_eq!(reply.reason, Reason::InvalidState);
    assert_eq!(c.status().state, State::Idle);
    assert!(h.enabled);
}

#[test]
fn one_fast_reading_while_idle_is_drive_noise_not_a_backdriven_arm() {
    let (mut c, mut h) = fixture(1);
    assert!(claim(&mut c, &mut h, 0).accepted);
    assert!(c.handle(1, idle_request(&c, 1), &mut h).accepted);
    h.feedback[0].velocity = 0.5;
    c.tick(2, &mut h);
    h.feedback[0].velocity = 0.0;
    c.tick(3, &mut h);
    h.feedback[0].velocity = 0.5;
    c.tick(4, &mut h);
    let reply = c.handle(
        5,
        Request::Arm {
            collision: None,
            authority: auth(&c, 2),
        },
        &mut h,
    );
    assert!(reply.accepted, "{:?}", reply.reason);
    assert_eq!(c.status().state, State::Holding);
}

#[test]
fn move_requires_enable_after_idle() {
    let (mut c, mut h) = fixture(1);
    assert!(claim(&mut c, &mut h, 0).accepted);
    assert!(c.handle(1, idle_request(&c, 1), &mut h).accepted);
    let reply = c.handle(2, move_request(&c, 2, 1, 0.1, 0.0), &mut h);
    assert_eq!(reply.reason, Reason::InvalidState);
    assert_eq!(c.status().state, State::Idle);
}

#[test]
fn idle_owner_loss_disables_instead_of_holding() {
    let (mut c, mut h) = fixture(1);
    assert!(claim(&mut c, &mut h, 0).accepted);
    assert!(c.handle(1, idle_request(&c, 1), &mut h).accepted);
    c.tick(99, &mut h);
    assert_eq!(c.status().state, State::Idle);
    c.tick(100, &mut h);
    let status = c.status();
    assert_eq!(status.state, State::Disarmed);
    assert_eq!(status.reason, Reason::LeaseExpired);
    assert!(status.lease.is_none());
    assert!(!h.enabled);
    let writes = h.writes;
    c.tick(150, &mut h);
    assert_eq!(h.writes, writes);
}

#[test]
fn idle_must_be_refreshed_like_any_command() {
    let (mut c, mut h) = fixture(1);
    assert!(claim(&mut c, &mut h, 0).accepted);
    assert!(c.handle(1, idle_request(&c, 1), &mut h).accepted);
    let next = offer(&mut c, &mut h, 50);
    assert!(
        c.handle(
            50,
            Request::Renew {
                authority: auth(&c, 2),
                offer: next
            },
            &mut h
        )
        .accepted
    );
    c.tick(100, &mut h);
    assert_eq!(c.status().state, State::Disarmed);
    assert_eq!(c.status().reason, Reason::CommandExpired);
    assert!(!h.enabled);
}

#[test]
fn deferred_idle_refresh_cannot_accelerate_periodic_writes() {
    let (mut c, mut h) = fixture(1);
    assert!(claim(&mut c, &mut h, 0).accepted);
    assert!(c.handle_deferred(1, idle_request(&c, 1), &mut h).accepted);
    let writes = h.writes;
    assert_eq!(writes, 1);
    for sequence in 2..6 {
        assert!(
            c.handle_deferred(2, idle_request(&c, sequence), &mut h)
                .accepted
        );
    }
    assert_eq!(h.writes, writes);
    c.tick(3, &mut h);
    assert_eq!(h.writes, writes + 1);
}

#[test]
fn idle_from_hold_drops_gains_now_and_clears_collision_bounds() {
    let (mut c, mut h) = collision_fixture();
    let checked = envelope(&mut c, &mut h, 1);
    assert!(
        c.handle(
            1,
            Request::Arm {
                authority: auth(&c, 1),
                collision: Some(checked)
            },
            &mut h
        )
        .accepted
    );
    let writes = h.writes;
    let reply = c.handle_deferred(2, idle_request(&c, 2), &mut h);
    assert!(reply.accepted);
    assert_eq!(reply.status.state, State::Idle);
    assert!(reply.status.collision_expires_at_ms.is_none());
    assert!(reply
        .status
        .joints
        .iter()
        .all(|j| j.kp == 0.0 && j.kd == 0.0));
    assert_eq!(h.writes, writes + 1);
    assert_eq!(h.enables, 1);
    // Hand motion outside the former collision box is not a collision fault.
    h.feedback[0].position = 2.8;
    c.tick(3, &mut h);
    assert_eq!(c.status().state, State::Idle);
}

#[test]
fn disable_removes_torque_immediately_and_releases_ownership() {
    for moving in [false, true] {
        let (mut c, mut h) = armed(2);
        let mut sequence = 2;
        if moving {
            assert!(
                c.handle(2, move_request(&c, sequence, 2, 0.2, 0.0), &mut h)
                    .accepted
            );
            sequence += 1;
            assert_eq!(c.status().state, State::Executing);
        }
        let reply = c.handle(3, disable_request(&c, sequence), &mut h);
        assert!(reply.accepted);
        assert_eq!(reply.status.state, State::Disarmed);
        assert!(reply.status.lease.is_none());
        assert!(reply.status.command_expires_at_ms.is_none());
        assert!(reply
            .status
            .joints
            .iter()
            .all(|j| j.kp == 0.0 && j.kd == 0.0));
        assert!(!h.enabled);
        let writes = h.writes;
        c.tick(4, &mut h);
        assert_eq!(h.writes, writes);
        // A new owner must explicitly start again; nothing re-arms by itself.
        assert!(claim(&mut c, &mut h, 5).accepted);
        assert!(!h.enabled);
        assert_eq!(c.status().state, State::Disarmed);
    }
}

#[test]
fn idle_and_disable_never_clear_a_fault() {
    let (mut c, mut h) = armed(1);
    assert!(c.handle(2, Request::EmergencyStop, &mut h).accepted);
    assert!(claim(&mut c, &mut h, 3).accepted);
    assert_eq!(
        c.handle(4, idle_request(&c, 1), &mut h).reason,
        Reason::InvalidState
    );
    assert!(!h.enabled);
    let reply = c.handle(5, disable_request(&c, 2), &mut h);
    assert!(reply.accepted);
    assert_eq!(reply.status.state, State::Faulted);
    assert_eq!(reply.status.reason, Reason::EmergencyStop);
    assert!(reply.status.lease.is_none());
    assert!(!h.enabled);
}

#[test]
fn idle_and_disable_require_the_current_owner() {
    let (mut c, mut h) = fixture(1);
    assert!(claim(&mut c, &mut h, 0).accepted);
    let mut stale = auth(&c, 1);
    stale.session = "someone-else".into();
    let reply = c.handle(
        1,
        Request::Idle {
            authority: stale.clone(),
        },
        &mut h,
    );
    assert_eq!(reply.reason, Reason::WrongOwner);
    assert!(!h.enabled);
    assert_eq!(
        c.handle(2, Request::Disable { authority: stale }, &mut h)
            .reason,
        Reason::WrongOwner
    );
    assert!(c.status().lease.is_some());
}

fn with_gripper(mut cfg: Config) -> Config {
    let last = cfg.joints.len() - 1;
    cfg.joints[last].gripper = true;
    cfg
}

#[test]
fn gripper_config_field_defaults_to_an_ordinary_joint() {
    let text = r#"{"name":"a","unit":"radians","lower":-1,"upper":1,"max_velocity":1,"max_effort":1,
        "max_kp":1,"max_kd":1,"stop_acceleration":1,"hold_kp":1,"hold_kd":1,"max_tracking_error":1,
        "max_temperature":70}"#;
    let joint: JointLimits = serde_json::from_str(text).unwrap();
    assert!(!joint.gripper);
}

#[test]
fn a_gripper_needs_no_stopping_reserve_at_its_ends() {
    let mut cfg = with_gripper(config(2));
    cfg.collision = Some(robot_control::collision::CollisionPolicy {
        world_revision: "fixed-world".into(),
        scope: "inter_branch".into(),
        max_verdict_ms: 40,
        reaction_ms: 10,
        bound: Default::default(),
    });
    for (gripper_low, accepted) in [(0.0, true), (0.01, false)] {
        let mut c = Controller::new(cfg.clone(), "boot-a".into()).unwrap();
        let mut h = Loopback::new(&[0.0, 0.0]);
        assert!(claim(&mut c, &mut h, 0).accepted);
        let mut checked = envelope(&mut c, &mut h, 1);
        // The gripper's box may start exactly at its position; it must still contain it.
        checked.lower[1] = gripper_low;
        let reply = c.handle(
            1,
            Request::Arm {
                authority: auth(&c, 1),
                collision: Some(checked),
            },
            &mut h,
        );
        assert_eq!(reply.accepted, accepted);
    }
}

#[test]
fn a_stopped_gripper_keeps_its_last_target_while_the_arm_brakes() {
    let mut c = Controller::new(with_gripper(config(2)), "boot-a".into()).unwrap();
    let mut h = Loopback::new(&[0.0, 0.0]);
    assert!(claim(&mut c, &mut h, 0).accepted);
    assert!(
        c.handle(
            1,
            Request::Arm {
                collision: None,
                authority: auth(&c, 1)
            },
            &mut h
        )
        .accepted
    );
    let r = move_request(&c, 2, 2, 0.1, 0.2);
    assert!(c.handle(2, r, &mut h).accepted);
    c.tick(100, &mut h);
    assert_eq!(c.status().state, State::Stopping);
    c.tick(150, &mut h);
    let joints = c.status().joints;
    assert!((joints[0].velocity - 0.1).abs() < 1e-10); // Braking at 2 rad/s^2.
    assert_eq!((joints[1].position, joints[1].velocity), (0.1, 0.0)); // Holding its target.
    c.tick(200, &mut h);
    c.tick(201, &mut h);
    assert_eq!(c.status().state, State::Stopped);
    assert_eq!(c.status().joints[1].position, 0.1);
}

#[test]
fn a_gripper_stalled_on_an_object_does_not_fault_but_an_arm_joint_does() {
    for gripper_stalled in [true, false] {
        let mut c = Controller::new(with_gripper(config(2)), "boot-a".into()).unwrap();
        let mut h = Loopback::new(&[0.0, 0.0]);
        assert!(claim(&mut c, &mut h, 0).accepted);
        assert!(
            c.handle(
                1,
                Request::Arm {
                    collision: None,
                    authority: auth(&c, 1)
                },
                &mut h
            )
            .accepted
        );
        let r = move_request(&c, 2, 2, 1.0, 0.0);
        assert!(c.handle(2, r, &mut h).accepted);
        // The jaws (or an arm joint) stop short of the target, 1 rad behind.
        let stalled = if gripper_stalled { 1 } else { 0 };
        h.feedback[stalled].position = 0.0;
        c.tick(3, &mut h);
        let status = c.status();
        if gripper_stalled {
            assert_eq!(status.state, State::Executing);
        } else {
            // Braked to a hold (at once: it was not moving), not dropped.
            assert_eq!(
                (status.state, status.reason),
                (State::Stopped, Reason::TrackingError)
            );
            assert!(h.enabled);
        }
    }
}

// Braking curves: in the fixture's collision policy a joint (limits +-3, cap 2 rad/s,
// 2 rad/s^2) may approach a limit only as fast as it can still stop at it, reacting within
// the feedback timeout and reaction allowance (30 ms), down to 0.5 rad/s, whose stopping
// margin (0.0775 rad) is all it keeps at each end.

fn arm_at(position: f64, upper: f64) -> (Controller, Loopback, Reply) {
    let (mut c, mut h) = collision_fixture();
    h.feedback[0].position = position;
    c.tick(1, &mut h);
    let mut checked = envelope(&mut c, &mut h, 1);
    checked.upper[0] = upper;
    let reply = c.handle(
        1,
        Request::Arm {
            authority: auth(&c, 1),
            collision: Some(checked),
        },
        &mut h,
    );
    (c, h, reply)
}

#[test]
fn limit_speed_is_the_cap_in_open_travel_and_stops_the_joint_at_the_limit() {
    use robot_control::collision::{limit_reserve, limit_speed, APPROACH_FLOOR_RAD_S};
    let mut cfg = config(1);
    let policy = robot_control::collision::CollisionPolicy {
        world_revision: "fixed-world".into(),
        scope: "inter_branch".into(),
        max_verdict_ms: 40,
        reaction_ms: 10,
        bound: Default::default(),
    };
    cfg.collision = Some(policy.clone());
    let joint = &cfg.joints[0];
    assert_eq!(limit_speed(&policy, &cfg, joint, 3.0), 2.0);
    // One radian from the limit: under the cap, and it stops exactly at the limit.
    let v = limit_speed(&policy, &cfg, joint, 1.0);
    assert!(v > APPROACH_FLOOR_RAD_S && v < 2.0, "{v}");
    assert!((limit_reserve(&policy, &cfg, joint, 1.0) - 1.0).abs() < 1e-9);
    // Within the floor's stopping margin the reserve is that margin: more than the distance.
    assert_eq!(
        limit_speed(&policy, &cfg, joint, 0.05),
        APPROACH_FLOOR_RAD_S
    );
    assert!((limit_reserve(&policy, &cfg, joint, 0.05) - 0.0775).abs() < 1e-9);
    let mut gripper = joint.clone();
    gripper.gripper = true;
    assert_eq!(limit_speed(&policy, &cfg, &gripper, 0.01), 2.0);
    assert_eq!(limit_reserve(&policy, &cfg, &gripper, 0.01), 0.0);
}

#[test]
fn a_joint_near_its_limit_arms_with_a_box_reaching_the_limit() {
    let (c, h, reply) = arm_at(2.0, 3.0);
    assert!(reply.accepted, "{:?}", reply.reason);
    assert_eq!(c.status().state, State::Holding);
    assert!(h.enabled);
    // A box stopping short of the limit lacks the reserve.
    let (_, h, reply) = arm_at(2.0, 2.9);
    assert!(!reply.accepted);
    assert_eq!(reply.reason, Reason::CollisionViolation);
    assert!(!h.enabled);
}

#[test]
fn a_joint_uses_its_travel_up_to_the_floors_stopping_margin() {
    let (_, h, reply) = arm_at(2.95, 3.0); // 0.05 rad from the limit; the margin is 0.0775.
    assert!(!reply.accepted);
    assert_eq!(reply.reason, Reason::CollisionViolation);
    assert!(!h.enabled);
    let (_, _, reply) = arm_at(2.9, 3.0); // 0.1 rad: on the curve, which stops it at the limit.
    assert!(reply.accepted, "{:?}", reply.reason);
}

#[test]
fn approaching_a_limit_faster_than_the_curve_allows_trips_but_leaving_it_does_not() {
    // 0.6 rad from the upper limit a joint may approach it at about 1.49 rad/s.
    let (mut c, mut h, reply) = arm_at(2.4, 3.0);
    assert!(reply.accepted, "{:?}", reply.reason);
    for now in 2..5 {
        h.feedback[0].velocity = -1.8; // Away from it, within the 2 rad/s cap.
        c.tick(now, &mut h);
    }
    assert_eq!(c.status().state, State::Holding);
    for now in 5..8 {
        h.feedback[0].velocity = 1.8; // Toward it, three readings in a row.
        c.tick(now, &mut h);
    }
    assert_eq!(c.status().reason, Reason::LimitViolation);
    assert_eq!(c.status().state, State::Stopping);
    assert!(h.enabled);
    // It is braked to rest short of the floor's stopping margin (0.0775 rad), where targets are
    // accepted again, and inside the box.
    for now in 8..3000 {
        c.tick(now, &mut h);
    }
    assert_eq!(c.status().state, State::Stopped);
    let held = c.status().joints[0].position;
    assert!(held <= 3.0 - 0.0775 + 1e-9, "{held}");
    assert!(held > 2.4);
}

fn interpolated_fixture() -> (Controller, Loopback) {
    let mut cfg = config(2);
    cfg.interpolation_ms = 40;
    cfg.collision = Some(robot_control::collision::CollisionPolicy {
        world_revision: "fixed-world".into(),
        scope: "inter_branch".into(),
        max_verdict_ms: 100, // Longer than a ramp, so the verdict stays fresh through these tests.
        reaction_ms: 10,
        bound: Default::default(),
    });
    let mut c = Controller::new(cfg, "boot-a".into()).unwrap();
    let mut h = Loopback::new(&[0.0, 0.0]);
    assert!(claim(&mut c, &mut h, 0).accepted);
    let checked = envelope(&mut c, &mut h, 1);
    assert!(
        c.handle(
            1,
            Request::Arm {
                authority: auth(&c, 1),
                collision: Some(checked),
            },
            &mut h,
        )
        .accepted
    );
    (c, h)
}

#[allow(clippy::too_many_arguments)]
fn move_with(
    c: &mut Controller,
    h: &mut Loopback,
    now: u64,
    sequence: u64,
    q: f64,
    v: f64,
    effort: f64,
    upper: f64,
) -> Reply {
    let mut checked = envelope(c, h, now);
    checked.upper = vec![upper; 2];
    let mut commanded = joints(2, q, v);
    for joint in commanded.iter_mut() {
        joint.effort = effort;
    }
    c.handle(
        now,
        Request::Move {
            collision: Some(checked),
            authority: auth(c, sequence),
            identity: identity(),
            ramp_efforts: Vec::new(),
            joints: commanded,
        },
        h,
    )
}

#[test]
fn velocity_and_effort_feed_forward_are_accepted_with_a_collision_box() {
    let (mut c, mut h) = interpolated_fixture();
    let reply = move_with(&mut c, &mut h, 2, 2, 0.2, 1.0, 1.5, 2.5);
    assert!(reply.accepted, "{:?}", reply.reason);
    let status = c.status();
    assert_eq!(status.state, State::Executing);
    assert_eq!(
        (status.joints[0].velocity, status.joints[0].effort),
        (1.0, 1.5)
    );
}

#[test]
fn a_move_ramps_from_where_the_stream_came_from_then_holds() {
    let (mut c, mut h) = interpolated_fixture();
    // 0.4 rad at 1 rad/s over the 40 ms period: the reference starts at 0.36.
    assert!(move_with(&mut c, &mut h, 10, 2, 0.4, 1.0, 0.0, 2.5).accepted);
    assert!((c.status().joints[0].position - 0.36).abs() < 1e-9);
    c.tick(30, &mut h);
    let halfway = c.status().joints[0];
    assert!((halfway.position - 0.38).abs() < 1e-9 && halfway.velocity == 1.0);
    c.tick(50, &mut h);
    let there = c.status().joints[0];
    assert!((there.position - 0.4).abs() < 1e-9 && there.velocity == 0.0);
}

#[test]
fn a_ramp_that_would_start_within_the_stopping_margin_is_refused() {
    let (mut c, mut h) = interpolated_fixture();
    // Ends 0.1 rad from the limit, but ramps from 2.98 (moving away at 2 rad/s over 40 ms).
    let reply = move_with(&mut c, &mut h, 2, 2, 2.9, -2.0, 0.0, 3.0);
    assert!(!reply.accepted);
    assert_eq!(reply.reason, Reason::CollisionViolation);
}

#[test]
fn a_stop_keeps_the_last_commands_feed_forward_holding_the_joint() {
    // A joint held out against gravity by 3 Nm of feed-forward: without it, stopping would drop
    // the load onto the position loop and sag the joint past its tracking bound.
    let (mut c, mut h) = interpolated_fixture();
    assert!(move_with(&mut c, &mut h, 2, 2, 0.0, 0.0, 3.0, 2.5).accepted);
    let reply = c.handle(
        3,
        Request::Stop {
            authority: auth(&c, 3),
        },
        &mut h,
    );
    assert!(reply.accepted, "{:?}", reply.reason);
    c.tick(4, &mut h);
    assert_eq!(c.status().state, State::Stopped);
    assert_eq!(c.status().joints[0].effort, 3.0);
}

#[test]
fn joint_limits_read_from_json_are_exactly_the_values_written() {
    // The live receiver reads its configuration from JSON, and the collision envelope carries the
    // same limits as binary floats. serde_json's default float parser can be 1 ULP off: it read
    // right J1's upper limit 1.6313617420196533 as ...531, and the receiver refused an envelope at
    // exactly its limits as a collision violation (2026-09-29).
    for text in [
        "1.6313617420196533",
        "-0.9554215860366821",
        "-0.8491166424751282",
    ] {
        let parsed: f64 = serde_json::from_str(text).unwrap();
        assert_eq!(parsed, text.parse::<f64>().unwrap(), "{text}");
    }
}

#[test]
fn a_joint_settling_past_a_command_at_the_edge_of_its_range_can_still_be_commanded() {
    // The command is at the edge of the usable range (0.1 rad from the limit, outside the
    // floor's 0.0775 stopping margin); the joint has settled 0.03 rad past it, inside that margin.
    let (mut c, mut h) = interpolated_fixture();
    assert!(move_with(&mut c, &mut h, 2, 2, 2.9, 0.0, 0.0, 3.0).accepted);
    h.feedback[0].position = 2.93;
    c.tick(3, &mut h);
    let reply = move_with(&mut c, &mut h, 4, 3, 2.9, 0.0, 0.0, 3.0);
    assert!(reply.accepted, "{:?}", reply.reason);
    // A command there is still refused.
    let reply = move_with(&mut c, &mut h, 5, 4, 2.95, 0.0, 0.0, 3.0);
    assert_eq!(reply.reason, Reason::CollisionViolation);
}

fn move_with_ramp(
    c: &mut Controller,
    h: &mut Loopback,
    now: u64,
    sequence: u64,
    ramp: Vec<f64>,
) -> Reply {
    let checked = envelope(c, h, now);
    let mut commanded = joints(2, 0.4, 1.0);
    for joint in commanded.iter_mut() {
        joint.effort = 1.0;
    }
    c.handle(
        now,
        Request::Move {
            collision: Some(checked),
            authority: auth(c, sequence),
            identity: identity(),
            ramp_efforts: ramp,
            joints: commanded,
        },
        h,
    )
}

#[test]
fn a_ramp_effort_accelerates_only_the_ramp_and_is_never_held() {
    // An acceleration's effort belongs to the ramp that changes the velocity: held after it, or
    // through a stop, it would push the joint on in that direction.
    let (mut c, mut h) = interpolated_fixture();
    assert!(move_with_ramp(&mut c, &mut h, 10, 2, vec![2.0, 0.0]).accepted);
    assert_eq!(c.status().joints[0].effort, 3.0); // Its own 1 Nm, and 2 accelerating it.
    assert_eq!(c.status().joints[1].effort, 1.0);
    c.tick(30, &mut h);
    assert_eq!(c.status().joints[0].effort, 3.0);
    c.tick(50, &mut h); // The ramp is over: only its own.
    assert_eq!(c.status().joints[0].effort, 1.0);
    // A stop in the middle of a ramp holds the joint with its own effort, not the ramp's.
    assert!(move_with_ramp(&mut c, &mut h, 60, 3, vec![2.0, 0.0]).accepted);
    assert!(
        c.handle(
            61,
            Request::Stop {
                authority: auth(&c, 4)
            },
            &mut h
        )
        .accepted
    );
    for now in 62..600 {
        c.tick(now, &mut h);
    }
    assert_eq!(c.status().joints[0].effort, 1.0);
    // Refused: a ramp effort for some joints only, or over the effort limit with the command's own.
    let (mut c, mut h) = interpolated_fixture();
    let short = move_with_ramp(&mut c, &mut h, 10, 2, vec![2.0]);
    assert_eq!(short.reason, Reason::InvalidCommand);
    let over = move_with_ramp(&mut c, &mut h, 11, 3, vec![4.5, 0.0]); // 1 + 4.5 over the 5 Nm limit.
    assert_eq!(over.reason, Reason::InvalidCommand);
    let bad = move_with_ramp(&mut c, &mut h, 12, 4, vec![f64::NAN, 0.0]);
    assert_eq!(bad.reason, Reason::InvalidCommand);
}

#[test]
fn a_quiet_drive_among_answering_ones_holds_the_arm_until_it_answers() {
    // The left gripper stopped answering for over 30 ms with the left arm raised; cutting every
    // joint's torque dropped the arm (2026-09-30). Now the arm is held on the drives that answer,
    // the quiet one keeps its last command, and it is re-armed only once every drive answers.
    let (mut c, mut h) = armed(2);
    let mut raised = joints(2, 1.0, 0.0);
    raised[0].effort = 4.0;
    let r = Request::Move {
        collision: None,
        authority: auth(&c, 2),
        identity: identity(),
        ramp_efforts: Vec::new(),
        joints: raised,
    };
    assert!(c.handle(2, r, &mut h).accepted);
    c.tick(3, &mut h); // Both readings reach the command...
    h.quiet = vec![false, true]; // ...then the second drive stops answering.
    for now in 4..30 {
        c.tick(now, &mut h);
    }
    let status = c.status();
    assert!(matches!(status.state, State::Stopping | State::Stopped));
    assert_eq!(status.reason, Reason::FeedbackStale);
    assert!(h.enabled && status.lease.is_none());
    assert_eq!(status.joints[0].effort, 4.0); // Held against its load.
    assert_eq!(status.joints[1].position, 1.0); // The quiet one keeps its command.
                                                // While it is quiet the arm stays held: no re-arm.
    assert!(claim(&mut c, &mut h, 30).accepted);
    let refused = c.handle(
        31,
        Request::Arm {
            collision: None,
            authority: auth(&c, 1),
        },
        &mut h,
    );
    assert_eq!(refused.reason, Reason::FeedbackStale);
    assert!(h.enabled);
    // It answers again: re-armed where it is held.
    h.quiet = Vec::new();
    c.tick(32, &mut h);
    let reply = c.handle(
        33,
        Request::Arm {
            collision: None,
            authority: auth(&c, 2),
        },
        &mut h,
    );
    assert!(reply.accepted, "{:?}", reply.reason);
    assert_eq!(c.status().state, State::Holding);
    // Every drive quiet at once (the bus) still cuts torque.
    h.quiet = vec![true, true];
    for now in 34..60 {
        c.tick(now, &mut h);
    }
    assert_eq!(c.status().state, State::Faulted);
    assert!(!h.enabled);
}

#[test]
fn a_move_bound_to_a_fresh_offer_renews_the_lease_with_it() {
    // A teleop cycle makes two requests: an offer, then the move whose verdict binds it. The move renews the
    // lease as Renew would, so the cycle needs no separate offer and renewal.
    let (mut c, mut h) = collision_fixture();
    let checked = envelope(&mut c, &mut h, 1);
    let r = c.handle(
        1,
        Request::Arm {
            authority: auth(&c, 1),
            collision: Some(checked),
        },
        &mut h,
    );
    assert!(r.accepted);
    let first = r.status.lease.unwrap();
    assert_eq!(first.expires_at_ms, 101); // The arm's offer, issued at 1.
    let mut sequence = 2;
    for now in (30..=300).step_by(30) {
        let checked = envelope(&mut c, &mut h, now);
        let r = c.handle(
            now,
            Request::Move {
                authority: auth(&c, sequence),
                identity: identity(),
                ramp_efforts: Vec::new(),
                joints: joints(2, 0.0, 0.0),
                collision: Some(checked),
            },
            &mut h,
        );
        assert!(r.accepted, "{:?}", r.reason);
        assert_eq!(r.status.lease.as_ref().unwrap().expires_at_ms, now + 100);
        assert_eq!(r.status.command_expires_at_ms, Some(now + 100));
        sequence += 1;
        c.tick(now + 1, &mut h);
    }
    // Moves alone kept it 300 ms past the first lease, and the lease id rotated as with Renew.
    assert_eq!(c.status().state, State::Executing);
    let lease = c.status().lease.unwrap();
    assert_ne!(lease.id, first.id);
    let stale = Authority {
        epoch: c.status().epoch,
        session: lease.session.clone(),
        lease: first.id,
        sequence,
    };
    let checked = envelope(&mut c, &mut h, 305);
    let refused = c.handle(
        305,
        Request::Move {
            authority: stale,
            identity: identity(),
            ramp_efforts: Vec::new(),
            joints: joints(2, 0.0, 0.0),
            collision: Some(checked),
        },
        &mut h,
    );
    assert_eq!(refused.reason, Reason::WrongLease);
    // A move bound to an offer older than the lease's expiry is still accepted, but never shortens it.
    let older = offer(&mut c, &mut h, 306);
    let fresh = offer(&mut c, &mut h, 310);
    let r = c.handle(
        310,
        Request::Renew {
            authority: auth(&c, sequence + 1),
            offer: fresh,
        },
        &mut h,
    );
    assert!(r.accepted);
    let r = c.handle(
        311,
        Request::Move {
            authority: auth(&c, sequence + 2),
            identity: identity(),
            ramp_efforts: Vec::new(),
            joints: joints(2, 0.0, 0.0),
            collision: Some(robot_control::collision::MotionEnvelope {
                offer: older,
                world_revision: "fixed-world".into(),
                scope: "inter_branch".into(),
                lower: vec![-2.5; 2],
                upper: vec![2.5; 2],
            }),
        },
        &mut h,
    );
    assert!(r.accepted, "{:?}", r.reason);
    assert_eq!(r.status.lease.unwrap().expires_at_ms, 410);
}

fn motion_fixture() -> (Controller, Loopback) {
    let mut cfg = config(1);
    cfg.interpolation_ms = 20;
    cfg.collision = Some(robot_control::collision::CollisionPolicy {
        world_revision: "fixed-world".into(),
        scope: "inter_branch".into(),
        max_verdict_ms: 80,
        reaction_ms: 10,
        bound: robot_control::collision::Bound::Motion,
    });
    let mut c = Controller::new(cfg, "boot-a".into()).unwrap();
    let mut h = Loopback::new(&[0.0]);
    assert!(claim(&mut c, &mut h, 0).accepted);
    (c, h)
}

fn boxed(
    c: &mut Controller,
    h: &mut Loopback,
    now: u64,
    low: f64,
    high: f64,
) -> robot_control::collision::MotionEnvelope {
    robot_control::collision::MotionEnvelope {
        offer: offer(c, h, now),
        world_revision: "fixed-world".into(),
        scope: "inter_branch".into(),
        lower: vec![low],
        upper: vec![high],
    }
}

#[test]
fn a_motion_box_covers_the_reference_ramp_and_a_stop_from_the_speed_bound() {
    // Tracking bound 0.5, stop acceleration 2, limit reaction 30 ms (20 ms feedback, 10 reaction).
    let reserve = |speed: f64| 0.5 + speed * 0.03 + speed * speed / 4.0;
    let (mut c, mut h) = motion_fixture();
    // At rest a joint may move at its approach floor, 0.5 rad/s.
    let rest = reserve(0.5);
    let arm = |c: &mut Controller, h: &mut Loopback, now: u64, edge: f64, sequence: u64| {
        let collision = Some(boxed(c, h, now, -edge, edge));
        c.handle(
            now,
            Request::Arm {
                authority: auth(c, sequence),
                collision,
            },
            h,
        )
    };
    assert_eq!(
        arm(&mut c, &mut h, 1, rest - 0.01, 1).reason,
        Reason::CollisionViolation
    );
    assert!(arm(&mut c, &mut h, 2, rest + 0.001, 2).accepted);
    assert_eq!(c.status().speed_bounds, vec![[0.5, 0.5]]);
    // A move up at 1 rad/s ramps from 0 to 0.02: its box holds that ramp, the tracking bound and above
    // it a stop from 1.5 rad/s (a servo overshoots the speed it follows, and its readings are noisy);
    // below it, one from the floor.
    let fast = reserve(1.5);
    let mut sequence = 3;
    let mut moved = |c: &mut Controller, h: &mut Loopback, now: u64, low: f64, high: f64| {
        let collision = Some(boxed(c, h, now, low, high));
        let mut joints = joints(1, 0.02, 1.0);
        joints[0].kp = 10.0;
        sequence += 1;
        c.handle(
            now,
            Request::Move {
                authority: auth(c, sequence),
                identity: identity(),
                joints,
                collision,
                ramp_efforts: Vec::new(),
            },
            h,
        )
    };
    assert_eq!(
        moved(&mut c, &mut h, 3, -rest + 0.01, 0.02 + fast).reason,
        Reason::CollisionViolation
    );
    assert_eq!(
        moved(&mut c, &mut h, 4, -rest, 0.02 + fast - 0.01).reason,
        Reason::CollisionViolation
    );
    assert!(moved(&mut c, &mut h, 5, -rest - 0.001, 0.02 + fast + 0.001).accepted);
    assert_eq!(c.status().speed_bounds, vec![[0.5, 1.5]]);
    // The reference ramps on to 0.02 and rests there.
    assert_eq!(c.status().reference_ends, vec![0.02]);
    // Once the ramp ends the bound falls at the joint's stop acceleration, no faster.
    for now in 6..=75 {
        c.tick(now, &mut h);
    }
    let bound = c.status().speed_bounds[0][1];
    assert!(
        (1.5 - 2.0 * 0.051..=1.5 - 2.0 * 0.049).contains(&bound),
        "{bound}"
    );
    assert_eq!(c.status().state, State::Executing);
}

#[test]
fn under_a_motion_bound_a_run_of_readings_over_the_speed_bound_trips_but_one_does_not() {
    let (mut c, mut h) = motion_fixture();
    let collision = Some(boxed(&mut c, &mut h, 1, -2.9, 2.9));
    assert!(
        c.handle(
            1,
            Request::Arm {
                authority: auth(&c, 1),
                collision
            },
            &mut h
        )
        .accepted
    );
    h.feedback[0].velocity = 0.6; // Over the 0.5 rad/s bound at rest, well under the 2 rad/s cap...
    c.tick(2, &mut h);
    h.feedback[0].velocity = 0.0;
    c.tick(3, &mut h);
    assert_eq!(c.status().state, State::Holding);
    for now in 4..7 {
        h.feedback[0].velocity = 0.6; // ...then three samples in a row.
        c.tick(now, &mut h);
    }
    assert_eq!(c.status().reason, Reason::LimitViolation);
    assert_eq!(c.status().state, State::Stopping);
    assert!(h.enabled);
    // Without a motion bound the same run is under the cap.
    let (mut c, mut h) = armed(1);
    for now in 2..6 {
        h.feedback[0].velocity = 0.6;
        c.tick(now, &mut h);
    }
    assert_eq!(c.status().state, State::Holding);
}

#[test]
fn under_a_motion_bound_a_joint_at_rest_keeps_its_rest_tracking_error() {
    let mut cfg = config(1);
    cfg.interpolation_ms = 20;
    cfg.joints[0].rest_tracking_error = Some(0.25); // Half its 0.5 bound.
    cfg.collision = Some(robot_control::collision::CollisionPolicy {
        world_revision: "fixed-world".into(),
        scope: "inter_branch".into(),
        max_verdict_ms: 80,
        reaction_ms: 10,
        bound: robot_control::collision::Bound::Motion,
    });
    let joint = cfg.joints[0].clone();
    // Its share of the bound grows from its approach floor (0.5) to its cap (2.0).
    use robot_control::collision::tracking_bound;
    assert_eq!(tracking_bound(&joint, 0.5), 0.25);
    assert_eq!(tracking_bound(&joint, 1.25), 0.375);
    assert_eq!(tracking_bound(&joint, 2.5), 0.5);
    let mut c = Controller::new(cfg, "boot-a".into()).unwrap();
    let mut h = Loopback::new(&[0.0]);
    assert!(claim(&mut c, &mut h, 0).accepted);
    // At rest the box keeps the rest error and a stop from the floor: 0.25 + 0.015 + 0.0625.
    let rest = 0.25 + 0.5 * 0.03 + 0.25 / 4.0;
    let collision = Some(boxed(&mut c, &mut h, 1, -rest + 0.005, rest));
    let refused = c.handle(
        1,
        Request::Arm {
            authority: auth(&c, 1),
            collision,
        },
        &mut h,
    );
    assert_eq!(refused.reason, Reason::CollisionViolation);
    let collision = Some(boxed(&mut c, &mut h, 2, -rest - 0.001, rest + 0.001));
    assert!(
        c.handle(
            2,
            Request::Arm {
                authority: auth(&c, 2),
                collision
            },
            &mut h
        )
        .accepted
    );
    // Trailing its reference by 0.3 at rest trips it (within its whole 0.5 bound, but not at rest).
    h.feedback[0].position = 0.3;
    c.tick(3, &mut h);
    assert_eq!(c.status().reason, Reason::TrackingError);
    // Braked to a hold (at rest, at once), not dropped.
    assert!(matches!(c.status().state, State::Stopping | State::Stopped));
    assert!(h.enabled);
}
