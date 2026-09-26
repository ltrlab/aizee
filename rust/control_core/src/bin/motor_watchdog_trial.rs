#[cfg(all(target_os = "linux", feature = "linux-service"))]
fn main() -> Result<(), Box<dyn std::error::Error>> {
    use robot_control::{
        service::{linux_can::LinuxCan, Mode, ServiceConfig},
        watchdog_trial::{self, Clock, Identity},
    };
    use serde::Deserialize;
    use std::{
        fs,
        sync::atomic::{AtomicBool, Ordering},
        time::{Duration, Instant},
    };
    static STOP: AtomicBool = AtomicBool::new(false);
    extern "C" fn stop(_: libc::c_int) {
        STOP.store(true, Ordering::Relaxed);
    }
    #[derive(Deserialize)]
    #[serde(deny_unknown_fields)]
    struct Config {
        service: ServiceConfig,
        motor_identity: Identity,
        #[serde(default)]
        jog_radians: Option<f64>,
        #[serde(default)]
        breakaway: Option<watchdog_trial::Breakaway>,
    }
    struct RealClock(Instant);
    impl Clock for RealClock {
        fn now_ms(&self) -> u64 {
            self.0.elapsed().as_millis() as u64
        }
        fn wait_ms(&mut self, ms: u64) {
            std::thread::sleep(Duration::from_millis(ms));
        }
        fn interrupted(&self) -> bool {
            STOP.load(Ordering::Relaxed)
        }
    }
    let args: Vec<String> = std::env::args().skip(1).collect();
    if args.len() != 2
        || ![
            "--allow-zero-torque-trial",
            "--allow-bounded-jog",
            "--allow-breakaway",
        ]
        .contains(&args[1].as_str())
    {
        return Err(
            "usage: motor_watchdog_trial CONFIG.json --allow-zero-torque-trial|--allow-bounded-jog|--allow-breakaway"
                .into(),
        );
    }
    let config: Config = serde_json::from_reader(fs::File::open(&args[0])?)?;
    config.service.validate()?;
    let jog = args[1] == "--allow-bounded-jog";
    let breakaway = args[1] == "--allow-breakaway";
    if jog != config.jog_radians.is_some()
        || breakaway != config.breakaway.is_some()
        || (jog && breakaway)
        || config.service.mode
            != if jog {
                Mode::BoundedJogTrial
            } else if breakaway {
                Mode::BreakawayTrial
            } else {
                Mode::ZeroTorqueTrial
            }
        || config.service.buses.len() != 1
        || config.service.motors[0].bus != 0
    {
        return Err("requires one motor on one identified physical CAN bus".into());
    }
    unsafe {
        libc::signal(libc::SIGINT, stop as *const () as libc::sighandler_t);
        libc::signal(libc::SIGTERM, stop as *const () as libc::sighandler_t);
    }
    let start = Instant::now();
    let mut io = LinuxCan::open(
        &config.service.buses,
        config.service.mode,
        &config.service.conflicting_services,
        start,
    )?;
    let result = if let Some(experiment) = config.breakaway {
        watchdog_trial::run_breakaway(
            &mut io,
            &mut RealClock(start),
            &config.service.motors[0],
            config.service.host_id,
            &config.motor_identity,
            experiment,
        )
    } else {
        watchdog_trial::run_with_jog(
            &mut io,
            &mut RealClock(start),
            &config.service.motors[0],
            config.service.host_id,
            &config.motor_identity,
            config.jog_radians,
        )
    };
    println!("{}", serde_json::to_string_pretty(&result)?);
    if !result.success {
        return Err("commissioning trial failed; inspect report".into());
    }
    Ok(())
}
#[cfg(not(target_os = "linux"))]
fn main() {
    eprintln!("motor_watchdog_trial requires Linux SocketCAN");
    std::process::exit(2);
}
