use robot_control::{
    can_adapter::{CanHardware, CanIo, Diagnostic, Motor, Received},
    hardware::{Hardware, HardwareError, JointFeedback},
    protocol::JointCommand,
    robstride::{self, Frame, Ranges},
};
use std::collections::VecDeque;

fn ranges() -> Ranges {
    Ranges {
        position: 12.5,
        velocity: 44.0,
        effort: 17.0,
        kp: 500.0,
        kd: 5.0,
    }
}
fn command() -> JointCommand {
    JointCommand {
        position: 0.0,
        velocity: 0.0,
        effort: 0.0,
        kp: 250.0,
        kd: 2.5,
    }
}
#[derive(Default)]
struct Bus {
    rx: VecDeque<Received>,
    tx: Vec<(usize, Frame)>,
    fail_at: Option<usize>,
    attempts: usize,
}
impl CanIo for Bus {
    fn receive(&mut self, _: u64) -> Result<Option<Received>, HardwareError> {
        Ok(self.rx.pop_front())
    }
    fn send(&mut self, bus: usize, frame: Frame) -> Result<(), HardwareError> {
        self.attempts += 1;
        if self.fail_at == Some(self.attempts) {
            return Err(HardwareError);
        }
        self.tx.push((bus, frame));
        Ok(())
    }
}
fn motor(name: &str, bus: usize) -> Motor {
    Motor {
        name: name.into(),
        bus,
        id: 4,
        ranges: ranges(),
        scale: 1.0,
        zero: 0.0,
    }
}
fn feedback(bus: usize, stamp: u64) -> Received {
    Received {
        bus,
        sampled_at_ms: stamp,
        frame: Frame {
            id: 0x028004aa,
            data: [0x80, 0, 0x80, 0, 0x80, 0, 0x01, 0x2c],
        },
    }
}

#[test]
fn codec_golden_vectors_use_full_16_bit_span() {
    assert_eq!(
        robstride::control(4, ranges(), command()).unwrap(),
        Frame {
            id: 0x017fff04,
            data: [0x7f, 0xff, 0x7f, 0xff, 0x7f, 0xff, 0x7f, 0xff]
        }
    );
    let mut c = command();
    c.position = 12.5;
    c.velocity = 44.0;
    c.effort = 17.0;
    c.kp = 500.0;
    c.kd = 5.0;
    assert_eq!(
        robstride::control(4, ranges(), c).unwrap(),
        Frame {
            id: 0x01ffff04,
            data: [255; 8]
        }
    );
    c.position = -12.5;
    c.velocity = -44.0;
    c.effort = -17.0;
    c.kp = 0.0;
    c.kd = 0.0;
    assert_eq!(
        robstride::control(4, ranges(), c).unwrap(),
        Frame {
            id: 0x01000004,
            data: [0; 8]
        }
    );
    c.position = f64::NAN;
    assert!(robstride::control(4, ranges(), c).is_err());
    c.position = 12.50001;
    assert!(robstride::control(4, ranges(), c).is_err());
    let f = robstride::feedback(feedback(0, 0).frame, 0xaa, ranges()).unwrap();
    assert_eq!(f.temperature, 30.0);
    assert_eq!(f.mode, 2);
    assert_eq!(f.error_bits, 0);
    assert!(robstride::feedback(
        Frame {
            id: 0x028004bb,
            ..feedback(0, 0).frame
        },
        0xaa,
        ranges()
    )
    .is_err());
    assert_eq!(
        robstride::motion_mode(4, 0xaa).data,
        [5, 0x70, 0, 0, 0, 0, 0, 0]
    );
}

#[test]
fn identical_ids_on_different_buses_and_source_age_are_preserved() {
    let mut bus = Bus::default();
    bus.rx.push_back(feedback(1, 5));
    let mut hw =
        CanHardware::new(bus, vec![motor("left", 0), motor("right", 1)], 0xaa, false).unwrap();
    let mut samples = [JointFeedback::default(); 2];
    hw.read(10, &mut samples).unwrap();
    assert_eq!(samples[0].sequence, 0);
    assert_eq!(samples[1].sequence, 1);
    assert_eq!(samples[1].sampled_at_ms, 5);
    let old = samples;
    hw.read(100, &mut samples).unwrap();
    assert_eq!(samples, old);
    hw.io.rx.push_back(feedback(1, 4));
    assert!(hw.read(101, &mut samples).is_err());
}

#[test]
fn transmission_transform_preserves_power_and_stiffness() {
    let mut m = motor("geared", 0);
    m.scale = -0.02;
    m.zero = 1.0;
    let c = JointCommand {
        position: 0.1,
        velocity: 0.2,
        effort: 10.0,
        kp: 100.0,
        kd: 5.0,
    };
    let raw = m.to_motor(c);
    assert_eq!(raw.position, -4.0);
    assert_eq!(raw.velocity, -10.0);
    assert!((raw.effort * raw.velocity - c.effort * c.velocity).abs() < 1e-12);
    assert!((raw.kp - 0.04).abs() < 1e-12);
}

#[test]
fn observer_cannot_transmit_and_invalid_batch_sends_nothing() {
    let mut hw = CanHardware::new(
        Bus::default(),
        vec![motor("left", 0), motor("right", 1)],
        0xaa,
        false,
    )
    .unwrap();
    assert!(hw.enable().is_err());
    hw.disable().unwrap();
    assert!(hw.probe(0, 0x7019).is_err());
    assert_eq!(hw.io.attempts, 0);
    hw.transmit = true;
    hw.allow_virtual_enable();
    hw.enable().unwrap();
    hw.io.tx.clear();
    let mut invalid = command();
    invalid.position = 100.0;
    assert!(hw.apply(&[command(), invalid]).is_err());
    assert!(hw.io.tx.is_empty());
}

#[test]
fn partial_enable_and_disable_report_failures_without_skipping_other_motors() {
    let bus = Bus {
        fail_at: Some(3),
        ..Bus::default()
    };
    let mut hw =
        CanHardware::new(bus, vec![motor("left", 0), motor("right", 1)], 0xaa, true).unwrap();
    hw.allow_virtual_enable();
    assert!(hw.enable().is_err());
    assert!(hw.enabled);
    hw.io.fail_at = Some(4);
    assert!(hw.disable().is_err());
    assert!(!hw.enabled);
    assert_eq!(hw.io.attempts, 5); // Tried BOTH disable frames even after first failed.
    assert_eq!(hw.io.tx.last().unwrap().0, 1);
}

#[test]
fn frame_flood_is_bounded_and_unknown_host_does_not_refresh_samples() {
    let mut bus = Bus::default();
    let mut rx = feedback(0, 0);
    rx.frame.id = 0x028004bb;
    bus.rx.extend(std::iter::repeat_n(rx, 1000));
    let mut hw = CanHardware::new(bus, vec![motor("one", 0)], 0xaa, false).unwrap();
    let mut output = [JointFeedback::default()];
    assert!(hw.read(1, &mut output).is_err());
    assert_eq!(hw.rx_frames, 128);
    assert_eq!(output[0].sequence, 0);
}

#[test]
fn diagnostics_cannot_enable_clear_faults_or_write_and_version_is_not_feedback() {
    let mut hw = CanHardware::new(Bus::default(), vec![motor("one", 0)], 0xaa, true).unwrap();
    hw.diagnostic(0, Diagnostic::Identity).unwrap();
    hw.diagnostic(0, Diagnostic::StopFeedback).unwrap();
    hw.diagnostic(0, Diagnostic::Version).unwrap();
    hw.diagnostic(0, Diagnostic::Parameter { parameter: 0x7028 })
        .unwrap();
    assert!(!hw.enabled);
    assert_eq!(
        hw.io.tx[0].1,
        Frame {
            id: 0xaa04,
            data: [0; 8]
        }
    );
    assert_eq!(
        hw.io.tx[1].1,
        Frame {
            id: 0x0400aa04,
            data: [0; 8]
        }
    );
    assert_eq!(
        hw.io.tx[2].1,
        Frame {
            id: 0x0400aa04,
            data: [0, 0xc4, 0, 0, 0, 0, 0, 0]
        }
    );
    assert_eq!(
        hw.io.tx[3].1,
        Frame {
            id: 0x1100aa04,
            data: [0x28, 0x70, 0, 0, 0, 0, 0, 0]
        }
    );
    assert!(hw
        .diagnostic(0, Diagnostic::Parameter { parameter: 0x200b })
        .is_err());
    assert!(hw.diagnostic(1, Diagnostic::Identity).is_err());
    let version = Frame {
        id: 0x020004aa,
        data: [0, 0xc4, 0x56, 0, 3, 1, 41, 0],
    };
    assert!(robstride::feedback(version, 0xaa, ranges()).is_err());
    hw.io.rx.push_back(Received {
        bus: 0,
        frame: version,
        sampled_at_ms: 1,
    });
    let mut samples = [JointFeedback::default()];
    hw.read(1, &mut samples).unwrap();
    assert_eq!(samples[0].sequence, 0);
    hw.io.rx.push_back(feedback(0, 2));
    hw.read(2, &mut samples).unwrap();
    assert_eq!(samples[0].sequence, 1);
    hw.enabled = true;
    assert!(hw.diagnostic(0, Diagnostic::StopFeedback).is_err());
    hw.enabled = false;
    hw.transmit = false;
    assert!(hw.diagnostic(0, Diagnostic::Identity).is_err());
    assert_eq!(hw.io.tx.len(), 4);
}

#[test]
fn a_drive_that_leaves_run_mode_while_driven_reads_as_quiet_not_as_a_failure() {
    let mut hw = CanHardware::new(Bus::default(), vec![motor("one", 0)], 0xaa, true).unwrap();
    hw.allow_virtual_enable();
    hw.enable().unwrap();
    let mut output = [JointFeedback::default()];
    hw.io.rx.push_back(feedback(0, 10));
    hw.read(10, &mut output).unwrap();
    assert_eq!((output[0].sequence, output[0].sampled_at_ms), (1, 10));
    // It reboots (or its own watchdog disables it): Reset mode, no fault, and it has moved.
    let mut reset = feedback(0, 70);
    reset.frame.id = 0x020004aa;
    reset.frame.data[0] = 0x90;
    hw.io.rx.push_back(reset);
    hw.read(70, &mut output).unwrap();
    assert_eq!((output[0].sequence, output[0].sampled_at_ms), (1, 10));
    assert!(hw
        .dropped()
        .unwrap()
        .starts_with("one: left Run mode while driven"));
    assert!(hw.failure().is_none());
    // A drive that reports a fault as it leaves Run mode still fails the read, cutting torque, named.
    let mut faulted = feedback(0, 80);
    faulted.frame.id = 0x020104aa;
    hw.io.rx.push_back(faulted);
    assert!(hw.read(80, &mut output).is_err());
    assert_eq!(
        hw.failure(),
        Some("one: left Run mode while driven (mode 0), reporting undervoltage")
    );
}

#[test]
fn fault_bits_are_named() {
    assert_eq!(robstride::fault_names(0), "no fault");
    assert_eq!(
        robstride::fault_names(0b000011),
        "undervoltage, overcurrent"
    );
    assert_eq!(robstride::fault_names(0b100000), "uncalibrated");
}

#[test]
fn a_drive_that_never_reaches_run_mode_after_enable_still_fails() {
    let mut hw = CanHardware::new(Bus::default(), vec![motor("one", 0)], 0xaa, true).unwrap();
    hw.allow_virtual_enable();
    hw.enable().unwrap();
    let mut output = [JointFeedback::default()];
    let mut reset = feedback(0, 60);
    reset.frame.id = 0x020004aa;
    hw.io.rx.push_back(reset);
    assert!(hw.read(60, &mut output).is_err());
    assert!(hw
        .failure()
        .unwrap()
        .contains("not in Run mode 50 ms after enable"));
    assert!(hw.dropped().is_none());
}
