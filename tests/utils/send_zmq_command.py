#!/usr/bin/env python3
"""Minimal ZMQ PUB test"""
import zmq
import time
import json

context = zmq.Context()

# Try both CONNECT (current) and BIND (alternative)
pub = context.socket(zmq.PUB)
pub.bind("tcp://*:5554")  # Use different port to avoid conflicts
print("Publisher BOUND to tcp://*:5554")

time.sleep(1)  # Allow socket to establish

for i in range(10):
    msg = {"type": "enable", "motor_ids": ["test"], "count": i}
    json_msg = json.dumps(msg)
    pub.send_string(json_msg)
    print(f"Sent {i}: {json_msg}")
    time.sleep(0.5)

pub.close()
context.term()
print("Done")
