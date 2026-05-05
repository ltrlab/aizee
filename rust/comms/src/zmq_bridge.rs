// ZeroMQ pub/sub bridge implementation

use crate::messages::{CommandMessage, TelemetryMessage};
use anyhow::{Context, Result};
use std::time::Duration;

/// Receive command messages from teleop (PULL socket)
pub struct CommandSubscriber {
    socket: zmq::Socket,
}

impl CommandSubscriber {
    pub fn new(endpoint: &str) -> Result<Self> {
        let context = zmq::Context::new();
        // Use PULL instead of SUB - guarantees delivery, no slow joiner problem
        let socket = context.socket(zmq::PULL)?;

        // Bind as the receiver (standard PUSH-PULL pattern)
        socket.bind(endpoint)?;

        // Non-blocking: return EAGAIN immediately if no message available.
        // rcvtimeo=0 prevents the arm control loop (which calls recv_command on
        // every 1 kHz tick) from stalling for 100 ms waiting for a ZMQ message,
        // which was throttling the arm CAN output to ~10 Hz instead of 1 kHz.
        if let Err(e) = socket.set_rcvtimeo(0) {
            tracing::warn!("Could not set receive timeout: {}", e);
        }

        tracing::info!("Command receiver (PULL) bound to {}", endpoint);

        Ok(Self { socket })
    }

    /// Receive next command (non-blocking).  Wire format is msgpack
    /// (`#[serde(tag = "type")]` enums encode as a map with a `type`
    /// field, which msgpack handles natively).
    pub fn recv_command(&mut self) -> Result<Option<CommandMessage>> {
        match self.socket.recv_bytes(0) {
            Ok(bytes) => {
                tracing::debug!("Received {} bytes on ZMQ socket", bytes.len());
                let cmd: CommandMessage = rmp_serde::from_slice(&bytes)
                    .context("Failed to deserialize msgpack command")?;
                tracing::info!("Parsed command: {:?}", cmd);
                Ok(Some(cmd))
            }
            Err(zmq::Error::EAGAIN) => {
                Ok(None)
            }
            Err(e) => {
                tracing::error!("ZMQ receive error: {}", e);
                Err(e.into())
            }
        }
    }
}

/// Publish telemetry messages
pub struct TelemetryPublisher {
    socket: zmq::Socket,
}

impl TelemetryPublisher {
    pub fn new(endpoint: &str) -> Result<Self> {
        let context = zmq::Context::new();
        let socket = context.socket(zmq::PUB)?;
        socket.bind(endpoint)?;

        tracing::info!("Telemetry publisher bound to {}", endpoint);

        Ok(Self { socket })
    }

    /// Publish a telemetry message.  Wire format is msgpack with named
    /// fields (`to_vec_named`) — every Python consumer reconstructs a
    /// dict shaped like the old JSON form via `unpack_msg(...)`.
    pub fn publish(&mut self, msg: &TelemetryMessage) -> Result<()> {
        let bytes = rmp_serde::to_vec_named(msg)
            .context("Failed to serialize telemetry to msgpack")?;
        self.socket.send(&bytes, 0)?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::messages::{MotorTelemetry, TelemetryMessage};
    use std::thread;

    #[test]
    #[ignore] // Requires ZeroMQ sockets, run manually
    fn test_pub_sub() {
        // Publisher
        let mut publisher = TelemetryPublisher::new("tcp://127.0.0.1:15556").unwrap();

        // Subscriber in separate thread
        let handle = thread::spawn(|| {
            let mut subscriber = CommandSubscriber::new("tcp://127.0.0.1:15556").unwrap();
            thread::sleep(Duration::from_millis(100)); // Wait for connection

            for _ in 0..5 {
                if let Ok(Some(_msg)) = subscriber.recv_command() {
                    return true;
                }
                thread::sleep(Duration::from_millis(10));
            }
            false
        });

        thread::sleep(Duration::from_millis(50)); // Wait for subscriber to connect

        let mut msg = TelemetryMessage::new();
        msg.add_motor(
            "test".to_string(),
            MotorTelemetry {
                position: 0.0,
                velocity: 0.0,
                torque: 0.0,
                temperature: 25.0,
                error: None,
            },
        );
        publisher.publish(&msg).unwrap();

        // Note: This test may fail due to timing; it's primarily for manual verification
    }
}
