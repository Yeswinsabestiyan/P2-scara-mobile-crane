#!/usr/bin/env python3
"""
scara_teleop.py  -  drive the whole robot from the keyboard.

Run it in its OWN terminal (it needs the keyboard):
    ros2 run scara_arm_description scara_teleop.py

It owns /joint_states (including the virtual base and object joints defined by
the sim_world URDF), so do not run joint_state_publisher_gui or the other demo
nodes with it. Use teleop.launch.py for robot_state_publisher + RViz.

Keys
  W / S   drive forward / back
  A / D   turn left / right
  I / K   crane column up / down
  J / L   arm shoulder (joint1) left / right
  U / O   arm elbow (joint2)
  T / G   slider (joint3) up / down
  R / F   wrist (joint4) rotate
  SPACE   close / open gripper (grabs the object if it is between the fingers)
  N       respawn the object in front of the robot
  H       arm back to home pose
  1/2/3   speed mode: SLOW / NORMAL / FAST
  Q       quit

Geometry constants must match urdf/scara_arm.urdf.xacro.
"""

import json
import math
import os
import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

# ---- geometry (keep in sync with the URDF) ---------------------------------
L1, L2 = 0.30, 0.25
WHEEL_R = 0.06
WHEEL_TRACK = 0.34
GRIPPER_Z_BASE = 0.41       # gripper_base world z when lift = 0 and joint3 = 0
HELD_Z = -0.04              # held object centre, relative to gripper_base
OBJ_SIZE = 0.04
OBJ_HALF = OBJ_SIZE / 2.0
FINGER_OPEN = 0.02
HOLD_FINGER = 0.0125        # finger value where the pads just touch the cube

LIMITS = {
    'j1': (-math.pi, math.pi),
    'j2': (-2.8, 2.8),
    'j3': (-0.20, 0.0),
    'j4': (-math.pi, math.pi),
    'lift': (-0.14, 0.10),
    'finger': (0.0, FINGER_OPEN),
}

# ---- feel -------------------------------------------------------------------
DT = 1.0 / 30.0

# Speed multiplier per mode
SPEED_MODES = {
    'SLOW':   0.40,
    'NORMAL': 1.00,
    'FAST':   2.50,
}

DRIVE_STEP, DRIVE_PEND_MAX, V_MAX = 0.03, 0.06, 0.40
TURN_STEP, TURN_PEND_MAX, W_MAX = 0.08, 0.20, 1.2
ROT_STEP, LIN_STEP = 0.04, 0.01
ROT_RATE, LIN_RATE, FINGER_RATE = 1.5, 0.15, 0.06
LIFT_STEP = LIN_STEP * 0.5   # finer control for crane lift

# ---- scene ------------------------------------------------------------------
# Three cargo crates: (start_x, start_y, colour_name)
CARGOS = [
    (0.42,  0.12,  'YELLOW'),
    (0.42, -0.20,  'RED'),
    (0.55,  0.00,  'BLUE'),
]

OBJ_START = (0.42, 0.12)    # world x, y (kept for respawn compat)

# Preset poses: (j1, j2, j3, j4, lift)
PRESET_STOW    = (0.0,   0.0,   0.0,   0.0,   0.0)
PRESET_PICK    = (0.0,   0.0,  -0.18,  0.0,  -0.14)
PRESET_DELIVER = (math.pi/2, 0.3, -0.10, 0.0, 0.05)

HELP = __doc__.split('Keys')[1].split('Geometry')[0]


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def approach(cur, tgt, step):
    d = tgt - cur
    return tgt if abs(d) <= step else cur + math.copysign(step, d)


class ScaraTeleop(Node):

    def __init__(self):
        super().__init__('scara_teleop')
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.tele_pub  = self.create_publisher(String, '/crane_telemetry', 10)

        # base pose + wheels
        self.x = self.y = self.th = 0.0
        self.pend_dist = self.pend_turn = 0.0
        self.wheel_l = self.wheel_r = 0.0

        # arm
        self.cur = {'j1': 0.0, 'j2': 0.0, 'j3': 0.0, 'j4': 0.0,
                    'lift': 0.0, 'finger': FINGER_OPEN}
        self.tgt = dict(self.cur)
        self.closing = False

        # speed
        self.speed_mode = 'NORMAL'

        # multi-cargo: list of [x, y, z, yaw, vz, colour, attached]
        self.cargos = []
        for (ox, oy, col) in CARGOS:
            self.cargos.append([ox, oy, OBJ_HALF, 0.0, 0.0, col, False])

        # which cargo is currently attached (-1 = none)
        self.held_idx = -1

        # auto pick & place state
        self._auto_state = None
        self._auto_step  = 0

        # subscribe to GUI control pad + presets
        self.create_subscription(String, '/crane_pad_keys', self._pad_cb, 10)
        self.create_subscription(String, '/crane_preset',   self._preset_cb, 10)

        self.fd = sys.stdin.fileno()
        self.old_term = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)

        print(HELP, flush=True)
        self.create_timer(DT, self._tick)

    def restore_terminal(self):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_term)

    # ---- GUI pad / preset callbacks -----------------------------------------
    def _pad_cb(self, msg):
        for ch in msg.data:
            self._on_key(ch)

    def _preset_cb(self, msg):
        name = msg.data.lower()
        if name == 'stow':
            self._apply_preset(PRESET_STOW)
        elif name == 'pick':
            self._apply_preset(PRESET_PICK)
        elif name == 'deliver':
            self._apply_preset(PRESET_DELIVER)
        elif name == 'auto':
            self._start_auto()

    def _apply_preset(self, pose):
        j1, j2, j3, j4, lift = pose
        self.tgt['j1']   = j1
        self.tgt['j2']   = j2
        self.tgt['j3']   = j3
        self.tgt['j4']   = j4
        self.tgt['lift'] = clamp(lift, *LIMITS['lift'])

    # ---- keyboard -----------------------------------------------------------
    def _read_keys(self):
        while select.select([self.fd], [], [], 0)[0]:
            data = os.read(self.fd, 64).decode(errors='ignore')
            for ch in data:
                self._on_key(ch)
            print(f'\rx={self.x:+.2f} y={self.y:+.2f} heading={math.degrees(self.th):+4.0f}deg '
                  f'lift={self.tgt["lift"]:+.2f} grip={"closed" if self.closing else "open  "} '
                  f'spd={self.speed_mode}   ',
                  end='', flush=True)

    def _nudge(self, name, delta):
        lo, hi = LIMITS[name]
        spd = SPEED_MODES.get(self.speed_mode, 1.0)
        self.tgt[name] = clamp(self.tgt[name] + delta * spd, lo, hi)

    def _on_key(self, ch):
        c = ch.lower()
        spd = SPEED_MODES.get(self.speed_mode, 1.0)
        if c == 'w':
            self.pend_dist = clamp(self.pend_dist + DRIVE_STEP * spd,
                                   -DRIVE_PEND_MAX, DRIVE_PEND_MAX)
        elif c == 's':
            self.pend_dist = clamp(self.pend_dist - DRIVE_STEP * spd,
                                   -DRIVE_PEND_MAX, DRIVE_PEND_MAX)
        elif c == 'a':
            self.pend_turn = clamp(self.pend_turn + TURN_STEP,
                                   -TURN_PEND_MAX, TURN_PEND_MAX)
        elif c == 'd':
            self.pend_turn = clamp(self.pend_turn - TURN_STEP,
                                   -TURN_PEND_MAX, TURN_PEND_MAX)
        elif c == 'j':
            self._nudge('j1', ROT_STEP)
        elif c == 'l':
            self._nudge('j1', -ROT_STEP)
        elif c == 'u':
            self._nudge('j2', ROT_STEP)
        elif c == 'o':
            self._nudge('j2', -ROT_STEP)
        elif c == 't':
            self._nudge('j3', LIN_STEP)
        elif c == 'g':
            self._nudge('j3', -LIN_STEP)
        elif c == 'r':
            self._nudge('j4', 2 * ROT_STEP)
        elif c == 'f':
            self._nudge('j4', -2 * ROT_STEP)
        elif c == 'i':
            self._nudge('lift', LIFT_STEP)
        elif c == 'k':
            self._nudge('lift', -LIFT_STEP)
        elif c == ' ':
            self._toggle_gripper()
        elif c == 'n':
            self._respawn_cargo()
        elif c == 'h':
            for k in ('j1', 'j2', 'j3', 'j4'):
                self.tgt[k] = 0.0
        elif c == '1':
            self.speed_mode = 'SLOW'
            self.get_logger().info('Speed: SLOW')
        elif c == '2':
            self.speed_mode = 'NORMAL'
            self.get_logger().info('Speed: NORMAL')
        elif c == '3':
            self.speed_mode = 'FAST'
            self.get_logger().info('Speed: FAST')
        elif c == 'q':
            self.get_logger().info('Quitting.')
            rclpy.shutdown()

    # ---- gripper / cargo ----------------------------------------------------
    def _grip_pose(self):
        """World x, y, z of gripper_base, and gripper yaw."""
        j1, j2 = self.cur['j1'], self.cur['j2']
        xc = L1 * math.cos(j1) + L2 * math.cos(j1 + j2)
        yc = L1 * math.sin(j1) + L2 * math.sin(j1 + j2)
        c, s = math.cos(self.th), math.sin(self.th)
        gx = self.x + xc * c - yc * s
        gy = self.y + xc * s + yc * c
        gz = GRIPPER_Z_BASE + self.cur['lift'] + self.cur['j3']
        yaw = self.th + j1 + j2 + self.cur['j4']
        return gx, gy, gz, yaw

    def _cargo_in_grasp(self, idx):
        gx, gy, gz, _ = self._grip_pose()
        c = self.cargos[idx]
        dxy = math.hypot(c[0] - gx, c[1] - gy)
        return dxy < 0.035 and abs(c[2] - (gz + HELD_Z)) < 0.04

    def _nearest_cargo_dist(self):
        """Return (distance_m, name, idx) of closest non-attached cargo."""
        gx, gy, _, _ = self._grip_pose()
        best_d, best_i = 999.0, -1
        for i, c in enumerate(self.cargos):
            if c[6]:
                continue
            d = math.hypot(c[0] - gx, c[1] - gy)
            if d < best_d:
                best_d, best_i = d, i
        if best_i >= 0:
            return best_d, self.cargos[best_i][5], best_i
        return 999.0, '---', -1

    def _toggle_gripper(self):
        if not self.closing:
            self.closing = True
            return
        self.closing = False
        if self.held_idx >= 0:
            gx, gy, gz, yaw = self._grip_pose()
            c = self.cargos[self.held_idx]
            c[0], c[1], c[2], c[3] = gx, gy, gz + HELD_Z, yaw
            c[4] = 0.0
            c[6] = False
            self.get_logger().info(f'Released {c[5]} cargo.')
            self.held_idx = -1

    def _respawn_cargo(self):
        """Respawn the first non-held cargo in front of the robot."""
        for i, c in enumerate(self.cargos):
            if not c[6]:
                cos_th, sin_th = math.cos(self.th), math.sin(self.th)
                ox, oy = CARGOS[i][0], CARGOS[i][1]
                c[0] = self.x + ox * cos_th - oy * sin_th
                c[1] = self.y + ox * sin_th + oy * cos_th
                c[2] = OBJ_HALF
                c[3] = self.th
                c[4] = 0.0
                break

    # ---- auto pick & place --------------------------------------------------
    def _start_auto(self):
        if self.held_idx >= 0:
            self._auto_state = 'DELIVER'
        else:
            self._auto_state = 'MOVE_TO_CARGO'
        self._auto_step = 0
        self.get_logger().info(f'Auto sequence: {self._auto_state}')

    def _tick_auto(self):
        if self._auto_state is None:
            return
        dist, name, idx = self._nearest_cargo_dist()

        if self._auto_state == 'MOVE_TO_CARGO':
            if idx < 0:
                self._auto_state = None
                return
            cx, cy = self.cargos[idx][0], self.cargos[idx][1]
            self.closing = False
            dx, dy = cx - self.x, cy - self.y
            target_j1 = math.atan2(dy * math.cos(self.th) - dx * math.sin(self.th),
                                   dx * math.cos(self.th) + dy * math.sin(self.th))
            self.tgt['j1']   = clamp(target_j1, *LIMITS['j1'])
            self.tgt['j3']   = -0.15
            self.tgt['lift'] = -0.14
            if dist < 0.08:
                self._auto_state = 'CLOSE_GRIPPER'
                self._auto_step  = 0

        elif self._auto_state == 'CLOSE_GRIPPER':
            self.closing = True
            self._auto_step += 1
            if self._auto_step > 20 and self.held_idx >= 0:
                self._auto_state = 'LIFT_UP'
                self._auto_step  = 0

        elif self._auto_state == 'LIFT_UP':
            self.tgt['lift'] = 0.08
            self.tgt['j3']   = 0.0
            self._auto_step += 1
            if self._auto_step > 30:
                self._auto_state = 'DELIVER'
                self._auto_step  = 0

        elif self._auto_state == 'DELIVER':
            pad_x, pad_y = 0.30, -0.32
            dx = pad_x - self.x
            dy = pad_y - self.y
            target_j1 = math.atan2(dy * math.cos(self.th) - dx * math.sin(self.th),
                                   dx * math.cos(self.th) + dy * math.sin(self.th))
            self.tgt['j1'] = clamp(target_j1, *LIMITS['j1'])
            self.tgt['j3'] = -0.12
            self._auto_step += 1
            if self._auto_step > 40:
                self._auto_state = 'RELEASE'
                self._auto_step  = 0

        elif self._auto_state == 'RELEASE':
            self._toggle_gripper()
            self._auto_step += 1
            if self._auto_step > 10:
                self._auto_state = 'HOME'
                self._auto_step  = 0

        elif self._auto_state == 'HOME':
            for k in ('j1', 'j2', 'j3', 'j4'):
                self.tgt[k] = 0.0
            self.tgt['lift'] = 0.0
            self._auto_step += 1
            if self._auto_step > 40:
                self._auto_state = None
                self.get_logger().info('Auto sequence complete!')

    # ---- main loop ----------------------------------------------------------
    def _tick(self):
        self._read_keys()
        if not rclpy.ok():
            return

        # auto sequence
        self._tick_auto()

        # base
        ds  = clamp(self.pend_dist, -V_MAX * DT, V_MAX * DT)
        dth = clamp(self.pend_turn, -W_MAX * DT, W_MAX * DT)
        self.pend_dist -= ds
        self.pend_turn -= dth
        self.x  += ds * math.cos(self.th + dth / 2.0)
        self.y  += ds * math.sin(self.th + dth / 2.0)
        self.th += dth
        self.wheel_l -= (ds - dth * WHEEL_TRACK / 2.0) / WHEEL_R
        self.wheel_r -= (ds + dth * WHEEL_TRACK / 2.0) / WHEEL_R

        # arm slew
        for k in ('j1', 'j2', 'j4'):
            self.cur[k] = approach(self.cur[k], self.tgt[k], ROT_RATE * DT)
        for k in ('j3', 'lift'):
            self.cur[k] = approach(self.cur[k], self.tgt[k], LIN_RATE * DT)

        # gripper
        if self.closing:
            self.tgt['finger'] = HOLD_FINGER if self.held_idx >= 0 else 0.0
        else:
            self.tgt['finger'] = FINGER_OPEN
        self.cur['finger'] = approach(self.cur['finger'], self.tgt['finger'],
                                      FINGER_RATE * DT)

        # try to grab a cargo
        if self.closing and self.held_idx < 0 and self.cur['finger'] <= HOLD_FINGER + 1e-4:
            for i, c in enumerate(self.cargos):
                if not c[6] and self._cargo_in_grasp(i):
                    c[6] = True
                    self.held_idx = i
                    self.get_logger().info(f'Grabbed {c[5]} cargo.')
                    break

        # update each cargo's physics
        for i, c in enumerate(self.cargos):
            if c[6]:
                gx, gy, gz, yaw = self._grip_pose()
                c[0], c[1], c[2], c[3] = gx, gy, gz + HELD_Z, yaw
            else:
                if c[2] > OBJ_HALF:
                    c[4] -= 9.8 * DT
                    c[2] = max(OBJ_HALF, c[2] + c[4] * DT)
                    if c[2] == OBJ_HALF:
                        c[4] = 0.0

        self._publish()
        self._publish_telemetry()

    # ---- publishing ---------------------------------------------------------
    def _publish(self):
        # Primary cargo (index 0) uses original URDF joint names
        c0 = self.cargos[0]
        c1 = self.cargos[1] if len(self.cargos) > 1 else [0.42, -0.20, OBJ_HALF, 0.0, 0.0, 'RED', False]
        c2 = self.cargos[2] if len(self.cargos) > 2 else [0.55, 0.00,  OBJ_HALF, 0.0, 0.0, 'BLUE', False]
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ['chassis_x_joint', 'chassis_y_joint', 'chassis_yaw_joint',
                   'front_left_wheel_joint', 'front_right_wheel_joint', 'crane_lift_joint',
                   'joint1', 'joint2', 'joint3', 'joint4', 'left_finger_joint',
                   'obj_x_joint', 'obj_y_joint', 'obj_z_joint', 'obj_yaw_joint',
                   'cargo2_x_joint', 'cargo2_y_joint', 'cargo2_z_joint', 'cargo2_yaw_joint',
                   'cargo3_x_joint', 'cargo3_y_joint', 'cargo3_z_joint', 'cargo3_yaw_joint']
        js.position = [self.x, self.y, self.th,
                       self.wheel_l, self.wheel_r, self.cur['lift'],
                       self.cur['j1'], self.cur['j2'], self.cur['j3'], self.cur['j4'],
                       self.cur['finger'],
                       c0[0], c0[1], c0[2], c0[3],
                       c1[0], c1[1], c1[2], c1[3],
                       c2[0], c2[1], c2[2], c2[3]]
        self.joint_pub.publish(js)


    def _publish_telemetry(self):
        dist, cargo_name, _ = self._nearest_cargo_dist()
        near = dist < 0.05
        tele = {
            'x':             round(self.x, 3),
            'y':             round(self.y, 3),
            'heading_deg':   round(math.degrees(self.th), 1),
            'lift':          round(self.cur['lift'], 4),
            'j1_deg':        round(math.degrees(self.cur['j1']), 1),
            'j2_deg':        round(math.degrees(self.cur['j2']), 1),
            'j3':            round(self.cur['j3'], 4),
            'j4_deg':        round(math.degrees(self.cur['j4']), 1),
            'dist_to_cargo': round(dist, 3),
            'cargo_name':    cargo_name,
            'gripper':       'closing' if self.closing else 'open',
            'holding':       self.held_idx >= 0,
            'near_grasp':    near,
            'speed_mode':    self.speed_mode,
        }
        msg = String()
        msg.data = json.dumps(tele)
        self.tele_pub.publish(msg)


def main():
    if not sys.stdin.isatty():
        print('scara_teleop needs an interactive terminal (run it with `ros2 run`, not from a launch file).')
        return
    rclpy.init()
    node = ScaraTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.restore_terminal()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
