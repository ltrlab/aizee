#!/usr/bin/env python3
"""
ROS 2 **SCServo Sync Driver** with *Interactive Centering Menu*
==============================================================

Changes
-------
1. After auto‑scan (or explicit `joint_map`) the node prints an **interactive
   menu** (CLI) that lets the operator:
   * list detected servos
   * pick one
   * read its *Min Angle Limit* and *Max Angle Limit* registers
   * command the servo to the midpoint of those limits (centering)
2. Menu runs **before** spinning the ROS executor so normal pub/sub still
   works afterward; non‑interactive deployments can bypass it with
   `--ros-args -p interactive:=false`.
3. Logger messages remain f‑string single‑arg (rclpy ≤ 0.8 safe).

### New parameters
```yaml
interactive: true   # if false, skips CLI and starts ROS immediately
```

### Extra control‑table addresses (STS/SMS)
```
ADDR_MIN_ANGLE_LIMIT = 9   # 2 bytes
ADDR_MAX_ANGLE_LIMIT = 11  # 2 bytes
```
"""
import math, threading, sys
from typing import Dict, List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

try:
    from scservo_sdk import (
        PortHandler, PacketHandler, GroupSyncWrite, GroupSyncRead,
        COMM_SUCCESS, SCS_LOBYTE, SCS_HIBYTE
    )
except ImportError as e:
    raise SystemExit('Install SDK first:  pip install feetech-servo-sdk') from e

# Control‑table addresses (STS/SMS)
ADDR_TORQUE_ENABLE     = 40
ADDR_GOAL_POSITION     = 42   # 2 bytes
ADDR_PRESENT_POSITION  = 56   # 2 bytes
ADDR_MIN_ANGLE_LIMIT   = 9    # 2 bytes
ADDR_MAX_ANGLE_LIMIT   = 11   # 2 bytes

# Conversion helpers
_RAD2DEG = 180.0 / math.pi
_DEG2RAD = math.pi / 180.0
_PULSE_MAX, _RANGE_DEG = 4095, 240.0
_SCALE = _PULSE_MAX / _RANGE_DEG

def rad_to_pulse(rad: float) -> int:
    deg = max(-_RANGE_DEG/2, min(_RANGE_DEG/2, rad * _RAD2DEG))
    return int((_RANGE_DEG/2 + deg) * _SCALE)

def pulse_to_rad(pulse: int) -> float:
    deg = (pulse / _SCALE) - _RANGE_DEG/2
    return deg * _DEG2RAD

#───────────────────────────────────────────────────────────────
class SCServoSyncDriver(Node):
    def __init__(self):
        super().__init__('scservo_sync_driver')
        # Params
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 1_000_000)
        self.declare_parameter('protocol_end', 0)
        self.declare_parameter('joint_map', [])
        self.declare_parameter('joint_ids', [])
        self.declare_parameter('scan_range', '1:16')
        self.declare_parameter('state_poll_hz', 20.0)
        self.declare_parameter('interactive', True)

        port      = self.get_parameter('serial_port').value
        baud      = int(self.get_parameter('baudrate').value)
        proto_end = int(self.get_parameter('protocol_end').value)
        poll_hz   = float(self.get_parameter('state_poll_hz').value)
        self.interactive = bool(self.get_parameter('interactive').value)

        # Serial
        self.port = PortHandler(port)
        if not self.port.openPort():
            raise SystemExit(f'Cannot open {port}')
        if not self.port.setBaudRate(baud):
            raise SystemExit(f'Cannot set baudrate {baud}')
        self.packet = PacketHandler(proto_end)
        self.get_logger().info(f'Port {port} opened @ {baud} baud')

        # Discover servos
        self.joint_names, self.ids = self._discover_servos()
        active = ', '.join(f'{n}(id={i})' for n,i in self.ids.items())
        self.get_logger().info(f'Active servos: {active}')

        # Sync helpers
        self._gs_write = GroupSyncWrite(self.port, self.packet, ADDR_GOAL_POSITION, 2)
        self._gs_read  = GroupSyncRead (self.port, self.packet, ADDR_PRESENT_POSITION, 2)
        for sid in self.ids.values():
            self._gs_read.addParam(sid)

        # ROS I/O
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        self.cmd_sub   = self.create_subscription(JointState, '/joint_group_cmd', self._cmd_cb, 10)
        self.state_pub = self.create_publisher  (JointState, '/joint_states', qos)
        self.diag_pub  = self.create_publisher  (DiagnosticArray, '/diagnostics', 10)

        self._lock = threading.Lock()
        self.create_timer(1.0/poll_hz, self._poll_loop)

        # Optional interactive menu before spinning
        if self.interactive:
            self._interactive_menu()

    #──────── discover
    def _discover_servos(self):
        jm=self.get_parameter('joint_map').value
        if jm:
            if isinstance(jm[0], str):
                import yaml; jm=[yaml.safe_load(s) for s in jm]
            names=[j['name'] for j in jm]; ids={j['name']:int(j['id']) for j in jm}
            return names,ids
        jids=self.get_parameter('joint_ids').value
        if jids:
            names=[f'servo_{i:02d}' for i in jids]
            return names,dict(zip(names,map(int,jids)))
        start,end=map(int,(self.get_parameter('scan_range').value or '1:16').split(':'))
        self.get_logger().info(f'Scanning IDs {start}–{end}')
        names,ids=[],{}
        for sid in range(start,end+1):
            _,res,err=self.packet.ping(self.port,sid)
            if res==COMM_SUCCESS and err==0:
                nm=f'servo_{sid:02d}'; names.append(nm); ids[nm]=sid
        if not ids: raise SystemExit('No servos detected')
        return names,ids

    #──────── interactive CLI
    def _interactive_menu(self):
        while True:
            print('\n===== SCServo Menu =====')
            for idx,name in enumerate(self.joint_names,1):
                print(f'{idx}. {name} (ID {self.ids[name]})')
            print('0. Continue to ROS spin')
            choice=input('Select servo to center (or 0): ')
            if not choice.isdigit():
                continue
            choice=int(choice)
            if choice==0:
                break
            if 1<=choice<=len(self.joint_names):
                sel=self.joint_names[choice-1]
                self._center_servo(sel)
            else:
                print('Invalid choice')

    def _center_servo(self,name:str):
        sid=self.ids[name]
        # read min/max angle limit (2 bytes each)
        min_raw,res1,err1=self.packet.read2ByteTxRx(self.port,sid,ADDR_MIN_ANGLE_LIMIT)
        max_raw,res2,err2=self.packet.read2ByteTxRx(self.port,sid,ADDR_MAX_ANGLE_LIMIT)
        if res1!=COMM_SUCCESS or res2!=COMM_SUCCESS or err1!=0 or err2!=0:
            print(f'Error reading angle limits for {name}')
            return
        center=(min_raw+max_raw)//2
        print(f'{name}: min={min_raw}, max={max_raw}, centering at={center}')
        # send goal position
        self.packet.write2ByteTxRx(self.port,sid,ADDR_GOAL_POSITION,center)
        # simple wait
        import time; time.sleep(0.5)

    #──────── command callback
    def _cmd_cb(self,msg:JointState):
        with self._lock:
            self._gs_write.clearParam()
            for name,rad in zip(msg.name,msg.position):
                sid=self.ids.get(name);  
                if sid is None: continue
                pulse=rad_to_pulse(rad)
                self._gs_write.addParam(sid,[SCS_LOBYTE(pulse),SCS_HIBYTE(pulse)])
            if self._gs_write.param:
                res=self._gs_write.txPacket()
                if res!=COMM_SUCCESS:
                    self.get_logger().error(f'SyncWrite error: {self.packet.getTxRxResult(res)}')

    #──────── poll
    def _poll_loop(self):
        js=JointState(); js.header.stamp=self.get_clock().now().to_msg()
        diag=DiagnosticArray(header=js.header)
        with self._lock:
            res=self._gs_read.txRxPacket()
            if res!=COMM_SUCCESS:
                self.get_logger().error(f'SyncRead error: {self.packet.getTxRxResult(res)}'); return
            for name in self.joint_names:
                sid=self.ids[name]
                if not self._gs_read.isAvailable(sid,ADDR_PRESENT_POSITION,2):
                    diag.status.append(DiagnosticStatus(name=f'Servo {name}',level=DiagnosticStatus.ERROR,message='No data'))
                    continue
                pulse=self._gs_read.getData(sid,ADDR_PRESENT_POSITION,2)
                js.name.append(name); js.position.append(pulse_to_rad(pulse))
                st=DiagnosticStatus(name=f'Servo {name}',level=DiagnosticStatus.OK,message='OK')
                st.values.append(KeyValue(key='pulse',value=str(pulse)))
                diag.status.append(st)
        self.state_pub.publish(js); self.diag_pub.publish(diag)

    def destroy_node(self):
        self.port.closePort(); super().destroy_node()

#───────────────────────────────────────────────────────────────

def main():
    rclpy.init()
    node=SCServoSyncDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node(); rclpy.shutdown()

if __name__=='__main__':
    main()
