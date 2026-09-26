use super::{Bus, Mode};
use crate::{
    can_adapter::{CanIo, Received},
    hardware::HardwareError,
    robstride::Frame,
};
use serde::Serialize;
use socketcan::{
    CanFrame, CanSocket, EmbeddedFrame, Frame as SocketFrame, Id, Socket, SocketOptions,
};
use std::{
    fs::{self, File, OpenOptions},
    io,
    os::fd::AsRawFd,
    path::Path,
    time::{Duration, Instant, SystemTime},
};

#[derive(Clone, Copy, Default, Serialize)]
pub struct Observation {
    pub bus: usize,
    pub id: u32,
    pub data: [u8; 8],
    pub received_at_ms: u64,
}
pub struct LinuxCan {
    sockets: Vec<CanSocket>,
    _locks: Vec<File>,
    clock: Instant,
    wall_start: SystemTime,
    start_offset: Duration,
    next: usize,
    pub observations: Vec<Observation>,
    observed: usize,
}
impl LinuxCan {
    pub fn open(
        buses: &[Bus],
        mode: Mode,
        conflicting_services: &[String],
        clock: Instant,
    ) -> Result<Self, Box<dyn std::error::Error>> {
        let mut sockets = Vec::new();
        let mut locks = Vec::new();
        for bus in buses {
            let sys = Path::new("/sys/class/net").join(&bus.interface);
            // Check the kernel interface kind, not a name prefix alone.
            let output = std::process::Command::new("ip")
                .args(["-details", "-json", "link", "show", &bus.interface])
                .output()?;
            if !output.status.success() {
                return Err("CAN interface is absent".into());
            }
            let links: serde_json::Value = serde_json::from_slice(&output.stdout)?;
            let kind = links[0]["linkinfo"]["info_kind"].as_str().unwrap_or("");
            if mode == Mode::VirtualCan {
                if kind != "vcan" || sys.join("device").exists() {
                    return Err("virtual mode requires a kernel vcan interface".into());
                }
            } else {
                let mut serial = None;
                let device = sys.join("device").canonicalize()?;
                for ancestor in device.ancestors() {
                    if ancestor.join("serial").is_file() {
                        serial = Some(
                            fs::read_to_string(ancestor.join("serial"))?
                                .trim()
                                .to_owned(),
                        );
                        break;
                    }
                }
                if kind != "can" || serial != bus.usb_serial {
                    return Err("physical CAN serial mismatch".into());
                }
                for name in conflicting_services {
                    let result = std::process::Command::new("systemctl")
                        .args(["show", name, "--property=MainPID", "--value"])
                        .output()?;
                    if !result.status.success()
                        || String::from_utf8_lossy(&result.stdout).trim() != "0"
                    {
                        return Err(
                            "a legacy motor service is running or its state is unknown".into()
                        );
                    }
                }
            }
            if mode != Mode::Observe
                && !links[0]["flags"]
                    .as_array()
                    .is_some_and(|v| v.iter().any(|f| f == "UP"))
            {
                return Err(
                    "CAN interface must already be up; service never changes link state".into(),
                );
            }
            // flock prevents two guarded owners, including observer vs transmitter.
            // Legacy services must remain disabled because they do not take this lock.
            use std::os::unix::fs::OpenOptionsExt;
            let lock = OpenOptions::new()
                .create(true)
                .read(true)
                .write(true)
                .truncate(false)
                .custom_flags(libc::O_NOFOLLOW)
                .open(format!("/tmp/robot-control-{}.lock", bus.interface))?;
            if unsafe { libc::flock(lock.as_raw_fd(), libc::LOCK_EX | libc::LOCK_NB) } != 0 {
                return Err("CAN interface already owned by another guarded process".into());
            }
            locks.push(lock);
            let socket = CanSocket::open(&bus.interface)?;
            socket.set_nonblocking(true)?;
            socket.set_recv_timestamp(true)?;
            socket.set_error_filter_accept_all()?;
            socket.set_recv_own_msgs(false)?;
            socket.set_loopback(mode == Mode::VirtualCan)?;
            sockets.push(socket);
        }
        let wall_start = SystemTime::now();
        let start_offset = clock.elapsed();
        Ok(Self {
            sockets,
            _locks: locks,
            clock,
            wall_start,
            start_offset,
            next: 0,
            observations: vec![Observation::default(); 32],
            observed: 0,
        })
    }
}
impl CanIo for LinuxCan {
    fn receive(&mut self, now_ms: u64) -> Result<Option<Received>, HardwareError> {
        for _ in 0..self.sockets.len() {
            let bus = self.next;
            self.next = (self.next + 1) % self.sockets.len();
            match self.sockets[bus].read_frame_with_timestamp() {
                Ok((frame, stamp)) => {
                    let elapsed = self.clock.elapsed();
                    let wall = SystemTime::now();
                    let expected = self.wall_start + elapsed.saturating_sub(self.start_offset);
                    let shift = wall
                        .duration_since(expected)
                        .unwrap_or_else(|e| e.duration());
                    if shift > Duration::from_millis(5) {
                        return Err(HardwareError);
                    }
                    let age = wall.duration_since(stamp).map_err(|_| HardwareError)?;
                    // A frame may arrive after this tick began. Dating it at tick
                    // start is conservative and avoids future samples at ms boundaries.
                    let sampled = (elapsed.checked_sub(age).ok_or(HardwareError)?.as_millis()
                        as u64)
                        .min(now_ms);
                    let id = match frame.id() {
                        Id::Extended(id) => id.as_raw(),
                        _ => return Err(HardwareError),
                    };
                    if frame.is_error_frame() || frame.is_remote_frame() || frame.data().len() != 8
                    {
                        return Err(HardwareError);
                    }
                    let data: [u8; 8] = frame.data().try_into().map_err(|_| HardwareError)?;
                    self.observations[self.observed % 32] = Observation {
                        bus,
                        id,
                        data,
                        received_at_ms: sampled,
                    };
                    self.observed = self.observed.wrapping_add(1);
                    return Ok(Some(Received {
                        bus,
                        frame: Frame { id, data },
                        sampled_at_ms: sampled,
                    }));
                }
                Err(e) if e.kind() == io::ErrorKind::WouldBlock => {}
                Err(_) => return Err(HardwareError),
            }
        }
        Ok(None)
    }
    fn send(&mut self, bus: usize, frame: Frame) -> Result<(), HardwareError> {
        let id = socketcan::ExtendedId::new(frame.id).ok_or(HardwareError)?;
        let wire = CanFrame::new(Id::Extended(id), &frame.data).ok_or(HardwareError)?;
        self.sockets
            .get(bus)
            .ok_or(HardwareError)?
            .write_frame(&wire)
            .map_err(|_| HardwareError)
    }
}
