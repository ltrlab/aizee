import can
import time

bus = can.Bus(interface='socketcan', channel='can1', bitrate=1000000)

found_motors = []

print('Scanning all IDs 1-127...')
for motor_id in range(1, 128):
    # Send enable command
    can_id = motor_id | (0xAA << 8) | (3 << 24)
    msg = can.Message(arbitration_id=can_id, is_extended_id=True, data=bytes([0]*8))
    bus.send(msg)
    
    # Listen for response
    start = time.time()
    while time.time() - start < 0.05:
        try:
            response = bus.recv(timeout=0.03)
            if response:
                resp_motor_id = (response.arbitration_id >> 8) & 0xFF
                if resp_motor_id == motor_id and motor_id not in found_motors:
                    found_motors.append(motor_id)
                    print(f'✓ Found motor at CAN ID {motor_id} (0x{motor_id:02X})')
                    # Disable it
                    can_id = motor_id | (0xAA << 8) | (4 << 24)
                    msg = can.Message(arbitration_id=can_id, is_extended_id=True, data=bytes([0]*8))
                    bus.send(msg)
                    break
        except:
            pass
    
    if motor_id % 20 == 0:
        print(f'  ...scanned up to ID {motor_id}')

print(f'\nFound {len(found_motors)} motor(s): {found_motors}')
bus.shutdown()
