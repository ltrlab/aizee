#!/usr/bin/env python3
"""
Test subscriber for AIZEE camera node

Subscribes to camera stream and displays received frames for testing.
"""

import argparse
import base64
import json
import time
from io import BytesIO

import cv2
import numpy as np
import zmq
from PIL import Image


def main():
    parser = argparse.ArgumentParser(description='Test AIZEE Camera Subscriber')
    parser.add_argument(
        '--zmq-endpoint',
        type=str,
        default='tcp://192.168.0.2:5557',
        help='ZeroMQ endpoint to subscribe to'
    )
    parser.add_argument(
        '--display',
        action='store_true',
        help='Display frames in window (requires X11)'
    )
    args = parser.parse_args()

    print(f"Connecting to camera at {args.zmq_endpoint}...")

    # Create ZMQ subscriber
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.connect(args.zmq_endpoint)
    socket.subscribe("")  # Subscribe to all messages

    print("Connected! Waiting for frames...")
    time.sleep(1)  # Give time for subscription to propagate

    frame_count = 0
    start_time = time.time()
    last_stats_time = start_time

    try:
        while True:
            # Receive message
            message_json = socket.recv_string()
            message = json.loads(message_json)

            frame_count += 1
            current_time = time.time()

            # Print statistics every 5 seconds
            if current_time - last_stats_time >= 5.0:
                elapsed = current_time - last_stats_time
                fps = frame_count / elapsed
                print(f"Received {frame_count} frames in {elapsed:.1f}s ({fps:.1f} fps)")
                print(f"  Camera: {message['camera_id']}")
                print(f"  Frame #: {message['frame_number']}")
                print(f"  Color: {message['color']['width']}x{message['color']['height']}")
                if 'infrared' in message:
                    print(f"  Infrared: {message['infrared']['width']}x{message['infrared']['height']}")

                # Decode and show image size
                color_data = base64.b64decode(message['color']['data'])
                print(f"  Color JPEG size: {len(color_data)} bytes")

                frame_count = 0
                last_stats_time = current_time

            # Optionally display frames
            if args.display:
                # Decode color image
                color_data = base64.b64decode(message['color']['data'])
                color_img = Image.open(BytesIO(color_data))
                color_np = np.array(color_img)

                # Convert RGB to BGR for OpenCV
                color_bgr = cv2.cvtColor(color_np, cv2.COLOR_RGB2BGR)

                # Display
                cv2.imshow('Camera Feed', color_bgr)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        if args.display:
            cv2.destroyAllWindows()
        socket.close()
        context.term()
        print("Disconnected")


if __name__ == '__main__':
    main()
