#!/usr/bin/env python3
# ────────────────────────────────────────────────────────────────
#  scservo_sync_driver.py  –  Jetson hardware bridge for MoveIt Servo
# ----------------------------------------------------------------
#  © 2024  Your Lab
#  Licence: MIT
# ────────────────────────────────────────────────────────────────
"""
Run on the Jetson that is wired to Feetech SC/SMS/ST servos.

Publishes:
    • /joint_states          (sensor_msgs/JointState)
    • /servo/status          (std_msgs/String  “OK” / “ERR”)

Subscribes:
    • /servo_node/delta_joint_cmds   (control_msgs/msg/JointJog)
    • /arm_group_controller/joint_trajectory  (trajectory_msgs/JointTrajectory)

Parameters (ros2 launch … or `ros2 run … --ros-args -p …`):

    serial_port:      /dev/ttyUSB0
    baud:             1000000
    ids:              [1,2,3,4,5,6]
    servo_names:      [shoulder_joint, …]      # same length as ids
    sign_correction:  [+1,+1,-1,+1,+1,-1]      #  +1 normal, -1 reversed
    home_offset:      [0,0,0,0,0,0]            # rad added *after* sign
    rated_torque:     2.50                     # Nm, used for “effort”
    state_poll_hz:    30.0
    interactive:      true                     # centring CLI

The sign/offset mapping applied is:

    rad_servo =  sign * (rad_robot + home_offset)
"""
# ----------------------------------------------------------------
import math, threading, queue
from typing import Dict, List

import rclpy
from rclpy.node      import Node
from rclpy.qos       import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg     import JointState
from std_msgs.msg        import Header
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg    import JointJog
from std_msgs.msg        import String

try:
    # pip install scservo_sdk  (same API as dynamixel‑sdk)
    from scservo_sdk import (
        PortHandler, PacketHandler, GroupSyncWrite, GroupSyncRead,
        COMM_SUCCESS, SCS_LOBYTE, SCS_HIBYTE
    )
except ImportError as e:
    raise SystemExit('🛑  `pip install feetech-servo-sdk` first!') from e

# ───────── Servo control‑table (STS/SMS series) ─────────
ADDR_GOAL_POSITION    = 42        # 2 bytes
ADDR_PRESENT_POSITION = 56        # 2 bytes
ADDR_PRESENT_SPEED    = 58        # 2 bytes
ADDR_PRESENT_LOAD     = 60        # 2 bytes
LEN_PRESENT_BLOCK     = 6         # pos+speed+load

# ───────── helper conversions ─────────
_PULSE_MAX, _RANGE_DEG = 4095, 240.0   # ±120 deg
_SCALE      = _PULSE_MAX / _RANGE_DEG
_RAD2DEG    = 180.0 / math.pi
_DEG2RAD    = math.pi  / 180.0

def rad_to_pulse(rad: float) -> int:
    """Clamp to ±120 deg and convert to Feetech pulse (0‑4095)."""
    deg = max(-_RANGE_DEG/2, min(_RANGE_DEG/2, rad * _RAD2DEG))
    return int((_RANGE_DEG/2 + deg) * _SCALE)

def pulse_to_rad(p: int) -> float:
    return ((p / _SCALE) - _RANGE_DEG/2) * _DEG2RAD

def speed_raw_to_rads(raw: int) -> float:
    """
    Present Speed register:
        bit10  → direction (1 = ‑)
        0‑9    → magnitude 0‑1023  (0.24 RPM each)
    """
    mag = 0.24 * (raw & 0x03FF)        # RPM
    sign = -1 if raw & 0x0400 else +1
    return sign * mag * 2*math.pi / 60.0

def load_raw_to_nm(raw: int, rated_nm: float) -> float:
    """Present Load raw (same sign convention) → Nm."""
    pct = (raw & 0x03FF) / 1023.0
    sign = -1 if raw & 0x0400 else +1
    return sign * pct * rated_nm

# ────────────────────────────────────────────────────────────────
class SCServoDriver(Node):
    def __init__(self):
        super().__init__('scservo_driver')

        # ───── parameters
        self.declare_parameters(
            '', [('serial_port',     '/dev/ttyUSB0'),
                 ('baud',            1_000_000),
                 ('ids',             [1, 2, 3, 4, 5, 6, 7]),
                 ('servo_names',     ['shoulder_joint','upper_arm_joint',
                                       'upper_elbow_joint','lower_elbow_joint',
                                       'forearm_joint','wrist_joint', 'cutter_end_joint']),
                 ('sign_correction', [-1, -1,  -1,  1, 1,  1,  1]),
                 ('home_offset',     [0.3, 0.26, 2.2, 0.5, 0.4, -0.85, 0.15]),
                 ('rated_torque',    2.5),
                 ('state_poll_hz',   30.0),
                 ('interactive',     True)]
        )
        port_name     = self.get_parameter('serial_port').value
        baud          = int(self.get_parameter('baud').value)
        self.ids: List[int]          = list(map(int, self.get_parameter('ids').value))
        names         = self.get_parameter('servo_names').value
        signs         = list(map(int,  self.get_parameter('sign_correction').value))
        offsets       = list(map(float,self.get_parameter('home_offset').value))
        self.rated_nm = float(self.get_parameter('rated_torque').value)
        poll_hz       = float(self.get_parameter('state_poll_hz').value)
        self.interactive = bool(self.get_parameter('interactive').value)

        assert len(names)==len(self.ids)==len(signs)==len(offsets), \
            'ids, servo_names, sign_correction, home_offset  must have equal length'

        # lookup tables
        self.name_to_id: Dict[str,int]  = dict(zip(names, self.ids))
        self.id_to_name: Dict[int,str]  = dict(zip(self.ids, names))
        self.sign:        Dict[int,int] = dict(zip(self.ids, signs))
        self.offset:      Dict[int,float] = dict(zip(self.ids, offsets))

        # remember last robot‑space position [rad]
        self.last_pos_robot: Dict[str,float] = {n:0.0 for n in names}

        # ───── serial
        self.port   = PortHandler(port_name)
        if not self.port.openPort():
            raise SystemExit(f'Cannot open {port_name}')
        if not self.port.setBaudRate(baud):
            raise SystemExit(f'Cannot set baudrate {baud}')
        self.packet = PacketHandler(0)
        self.get_logger().info(f'🟢 Serial open {port_name}@{baud}')

        # ───── SDK helpers
        self.w_sync = GroupSyncWrite(self.port, self.packet,
                                     ADDR_GOAL_POSITION, 2)
        self.r_sync = GroupSyncRead (self.port, self.packet,
                                     ADDR_PRESENT_POSITION, LEN_PRESENT_BLOCK)
        for sid in self.ids:
            self.r_sync.addParam(sid)

        # ───── ROS I/O
        qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.BEST_EFFORT)
        self.state_pub  = self.create_publisher(JointState, '/joint_states', qos)
        self.status_pub = self.create_publisher(String,      '/servo/status', 1)

        self.create_subscription(JointJog,
            '/servo_node/delta_joint_cmds', self._jog_cb, 10)

        self.create_subscription(JointTrajectory,
            '/arm_group_controller/joint_trajectory', self._traj_cb, 10)

        # polling loop → joint_states
        self.create_timer(1.0/poll_hz, self._poll)

        # background worker for trajectory points
        self._goal_q: queue.Queue = queue.Queue()
        threading.Thread(target=self._exec_loop, daemon=True).start()

        # optional centring CLI
        if self.interactive:
            threading.Thread(target=self._menu, daemon=True).start()

    # ───────── interactive centring menu (optional) ─────────
    def _menu(self):
        import time
        while rclpy.ok():
            print('\n===== Servo Centering Menu =====')
            for i,sid in enumerate(self.ids,1):
                print(f'{i}. {self.id_to_name[sid]}  (ID {sid})')
            print('0. continue')
            sel=input('Select servo to center › ')
            if not sel.isdigit(): continue
            idx=int(sel)
            if idx==0: break
            if 1<=idx<=len(self.ids):
                sid=self.ids[idx-1]
                # read angle limits
                min_raw,_ ,_ = self.packet.read2ByteTxRx(self.port,sid,9)
                max_raw,_ ,_ = self.packet.read2ByteTxRx(self.port,sid,11)
                center=(min_raw+max_raw)//2
                self.packet.write2ByteTxRx(self.port,sid,ADDR_GOAL_POSITION,center)
                print(f'Centered {self.id_to_name[sid]}')
            time.sleep(0.3)

    # ───────── JointJog from MoveIt Servo─────────
    def _jog_cb(self, msg: JointJog):
        self.w_sync.clearParam()
        for name, d in zip(msg.joint_names, msg.displacements):
            sid = self.name_to_id.get(name)
            if sid is None: continue
            # integrate delta in *robot* space
            self.last_pos_robot[name] += d
            robot = self.last_pos_robot[name]

            # map to servo space
            servo_rad = self.sign[sid] * (robot + self.offset[sid])
            pulse     = rad_to_pulse(servo_rad)
            self.w_sync.addParam(sid, [SCS_LOBYTE(pulse), SCS_HIBYTE(pulse)])

        if self.w_sync.param:
            res=self.w_sync.txPacket()
            self.status_pub.publish(String(data='OK' if res==COMM_SUCCESS else 'ERR'))

    # ───────── Trajectory follower (optional) ─────────
    def _traj_cb(self, msg: JointTrajectory):
        if not msg.points: return
        point: JointTrajectoryPoint = msg.points[-1]
        goal={}
        for name, rad in zip(msg.joint_names, point.positions):
            sid=self.name_to_id.get(name)
            if sid is None: continue
            self.last_pos_robot[name] = rad          # overwrite
            servo_rad = self.sign[sid]*(rad + self.offset[sid])
            goal[sid] = rad_to_pulse(servo_rad)
        self._goal_q.put(goal)

    def _exec_loop(self):
        while rclpy.ok():
            goal=self._goal_q.get()
            self.w_sync.clearParam()
            for sid,pulse in goal.items():
                self.w_sync.addParam(sid,[SCS_LOBYTE(pulse),SCS_HIBYTE(pulse)])
            res=self.w_sync.txPacket()
            self.status_pub.publish(String(data='OK' if res==COMM_SUCCESS else 'ERR'))

    # ───────── poll → /joint_states ─────────
    def _poll(self):
        if self.r_sync.txRxPacket()!=COMM_SUCCESS:
            return
        now = self.get_clock().now().to_msg()
        stamp = self.get_clock().now().to_msg()
        js    = JointState(header=Header(stamp=stamp))

        for sid in self.ids:
            if not self.r_sync.isAvailable(sid, ADDR_PRESENT_POSITION, LEN_PRESENT_BLOCK):
                continue
            pos_raw   = self.r_sync.getData(sid, ADDR_PRESENT_POSITION, 2)
            speed_raw = self.r_sync.getData(sid, ADDR_PRESENT_SPEED,    2)
            load_raw  = self.r_sync.getData(sid, ADDR_PRESENT_LOAD,     2)

            # servo → robot space
            rad_servo  = pulse_to_rad(pos_raw)
            rad_robot  = self.sign[sid]*(rad_servo - self.offset[sid])
            vel_robot  = self.sign[sid]*speed_raw_to_rads(speed_raw)
            effort_nm  = self.sign[sid]*load_raw_to_nm(load_raw, self.rated_nm)

            name = self.id_to_name[sid]
            self.last_pos_robot[name] = rad_robot

            js.name.append(name)
            js.position.append(rad_robot)
            js.velocity.append(vel_robot)
            js.effort.append(effort_nm)

        self.state_pub.publish(js)

    # ───────── shutdown cleanly ─────────
    def destroy_node(self):
        self.port.closePort()
        super().destroy_node()

# ────────────────────────────────────────────────────────────────
def main():
    rclpy.init()
    node=SCServoDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':
    main()
