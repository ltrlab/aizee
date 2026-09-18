//! Opt-in raw boundary trace. Disk writes never run on the control thread.
use serde::Serialize;
use std::fs::OpenOptions;
use std::io::{BufWriter, Write};
use std::sync::{mpsc, Arc, atomic::{AtomicU64, Ordering}};
use std::thread::JoinHandle;
use std::time::Instant;
use socketcan::EmbeddedFrame;

#[derive(Serialize)]
struct Event {
    schema_version: u32,
    sequence: u64,
    time_s: f64,
    kind: String,
    payload: serde_json::Value,
}

pub struct TraceRecorder {
    sender: Option<mpsc::SyncSender<Event>>,
    worker: Option<JoinHandle<()>>,
    sequence: AtomicU64,
    dropped: Arc<AtomicU64>,
    started: Instant,
}

impl TraceRecorder {
    pub fn from_env() -> std::io::Result<Self> {
        let mut result = Self { sender: None, worker: None, sequence: AtomicU64::new(0),
            dropped: Arc::new(AtomicU64::new(0)), started: Instant::now() };
        if let Some(path) = std::env::var_os("ROBOT_PLATFORM_TRACE") {
            let file = OpenOptions::new().write(true).create_new(true).open(path)?;
            let (sender, receiver) = mpsc::sync_channel::<Event>(8192);
            let dropped = result.dropped.clone();
            result.worker = Some(std::thread::spawn(move || {
                let mut out = BufWriter::new(file);
                let write_result = (|| -> std::io::Result<()> {
                    for event in receiver {
                        serde_json::to_writer(&mut out, &event)?;
                        out.write_all(b"\n")?;
                    }
                    serde_json::to_writer(&mut out, &serde_json::json!({"kind": "trace_end",
                        "dropped_events": dropped.load(Ordering::Relaxed)}))?;
                    out.write_all(b"\n")?;
                    out.flush()
                })();
                if let Err(error) = write_result {
                    eprintln!("TRACE INCOMPLETE: {}", error);
                }
            }));
            result.sender = Some(sender);
            result.record("trace_start", serde_json::json!({"pid": std::process::id(),
                "clock_domain": "process_monotonic", "timestamp_provenance": "host userspace boundary",
                "utc_start": std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).ok().map(|v| v.as_secs_f64()),
                "on_wire_timing": false}));
        }
        Ok(result)
    }

    pub fn enabled(&self) -> bool { self.sender.is_some() }

    pub fn record(&self, kind: &str, payload: serde_json::Value) {
        if let Some(sender) = &self.sender {
            let event = Event { schema_version: 1,
                sequence: self.sequence.fetch_add(1, Ordering::Relaxed) + 1,
                time_s: self.started.elapsed().as_secs_f64(), kind: kind.into(), payload };
            if sender.try_send(event).is_err() {
                let count = self.dropped.fetch_add(1, Ordering::Relaxed) + 1;
                if count == 1 || count % 10000 == 0 {
                    eprintln!("TRACE INCOMPLETE: {} events dropped or writer failed", count);
                }
            }
        }
    }

    pub fn can(&self, kind: &str, frame: &socketcan::CanFrame, bus: &str) {
        if self.enabled() {
            let id = match frame.id() {
                socketcan::Id::Standard(id) => id.as_raw() as u32,
                socketcan::Id::Extended(id) => id.as_raw(),
            };
            self.record(kind, serde_json::json!({"bus": bus, "arbitration_id": id,
                "extended": frame.is_extended(), "data": frame.data()}));
        }
    }
}

impl Drop for TraceRecorder {
    fn drop(&mut self) {
        self.sender.take();
        if let Some(worker) = self.worker.take() { let _ = worker.join(); }
    }
}
