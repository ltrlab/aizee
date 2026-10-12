//! One test process isolates allocator instrumentation from other test threads.
use robot_control::{hardware::Loopback, protocol::*, Config, Controller, JointLimits, JointUnit};
use std::alloc::{GlobalAlloc, Layout, System};
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};

struct Audit;
static WATCH: AtomicBool = AtomicBool::new(false);
static OPERATIONS: AtomicUsize = AtomicUsize::new(0);
unsafe impl GlobalAlloc for Audit {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        if WATCH.load(Ordering::Relaxed) {
            OPERATIONS.fetch_add(1, Ordering::Relaxed);
        }
        System.alloc(layout)
    }
    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        if WATCH.load(Ordering::Relaxed) {
            OPERATIONS.fetch_add(1, Ordering::Relaxed);
        }
        System.dealloc(ptr, layout)
    }
    unsafe fn realloc(&self, ptr: *mut u8, layout: Layout, size: usize) -> *mut u8 {
        if WATCH.load(Ordering::Relaxed) {
            OPERATIONS.fetch_add(1, Ordering::Relaxed);
        }
        System.realloc(ptr, layout, size)
    }
}
#[global_allocator]
static ALLOCATOR: Audit = Audit;

#[test]
fn periodic_execution_expiry_stop_and_fault_do_not_touch_allocator() {
    let identity = Identity {
        model: "m".into(),
        calibration: "c".into(),
        collision_policy: "p".into(),
    };
    let config = Config {
        direct: None,
        collision: Some(robot_control::collision::CollisionPolicy {
            world_revision: "fixed".into(),
            scope: "all".into(),
            max_verdict_ms: 40,
            reaction_ms: 10,
            bound: Default::default(),
        }),
        group: "g".into(),
        identity: identity.clone(),
        lease_ms: 100,
        feedback_timeout_ms: 20,
        interpolation_ms: 0,
        settled_velocity: 0.001,
        joints: vec![JointLimits {
            name: "q".into(),
            unit: JointUnit::Radians,
            lower: -2.0,
            upper: 2.0,
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
        }],
    };
    let mut c = Controller::new(config, "boot".into()).unwrap();
    let mut h = Loopback::new(&[0.0]);
    let offer = c.handle(0, Request::Offer, &mut h).offer.unwrap().id;
    c.handle(
        0,
        Request::Acquire {
            epoch: "boot".into(),
            offer,
            session: "s".into(),
            group: "g".into(),
            identity: identity.clone(),
        },
        &mut h,
    );
    // An arm or move whose verdict binds a fresh offer renews the lease with it: the lease id follows.
    let authority = |c: &Controller, sequence| Authority {
        epoch: "boot".into(),
        session: "s".into(),
        lease: c.status().lease.map_or(offer, |lease| lease.id),
        sequence,
    };
    let collision_offer = c.handle(1, Request::Offer, &mut h).offer.unwrap().id;
    let envelope = |offer| robot_control::collision::MotionEnvelope {
        offer,
        world_revision: "fixed".into(),
        scope: "all".into(),
        lower: vec![-1.99],
        upper: vec![1.99],
    };
    assert!(
        c.handle(
            1,
            Request::Arm {
                collision: Some(envelope(collision_offer)),
                authority: authority(&c, 1),
            },
            &mut h,
        )
        .accepted
    );
    let collision_offer = c.handle(2, Request::Offer, &mut h).offer.unwrap().id;
    assert!(
        c.handle(
            2,
            Request::Move {
                collision: Some(envelope(collision_offer)),
                authority: authority(&c, 2),
                identity,
                joints: vec![JointCommand {
                    position: 0.1,
                    velocity: 0.0,
                    effort: 0.0,
                    kp: 10.0,
                    kd: 1.0,
                }],
                ramp_efforts: Vec::new(),
            },
            &mut h,
        )
        .accepted
    );
    WATCH.store(true, Ordering::SeqCst);
    for now in 3..300 {
        if now == 42 {
            h.feedback[0].velocity = 0.2;
        }
        c.tick(now, &mut h);
    }
    h.motor_fault = true;
    c.tick(300, &mut h);
    WATCH.store(false, Ordering::SeqCst);
    assert_eq!(OPERATIONS.load(Ordering::SeqCst), 0);
    assert_eq!(c.status().state, State::Faulted);
    // The fault's trip was recorded inside the watched ticks, without allocating.
    assert_eq!(c.status().last_trip.unwrap().check, "drive_fault");
}
