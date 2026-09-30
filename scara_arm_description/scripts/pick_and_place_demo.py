#!/usr/bin/env python3
"""
pick_and_place_demo.py

A fully scripted pick-and-place sequence for the SCARA arm -- no manual
sliders involved. This node:

  1. Publishes /joint_states itself (so do NOT run joint_state_publisher
     or joint_state_publisher_gui alongside this -- use
     pick_and_place_demo.launch.py, not display.launch.py).
  2. Broadcasts a one-time static world -> chassis_link transform (the
     chassis doesn't move in this demo, so it doesn't need fake_diff_drive).
  3. Solves basic 2-link planar inverse kinematics to compute joint1/joint2
     for a target (x, y), since we're commanding Cartesian pick/place
     points rather than raw joint angles.
  4. Runs a timed state machine: swing to the pickup point, descend,
     close the gripper, ascend, swing to the place point, descend, open
     the gripper, ascend, return home -- then holds forever.
  5. Broadcasts the payload object's pose: relative to "world" while it's
     just sitting somewhere, relative to "gripper_base" while it's being
     carried, so it visually rides along with the gripper.

All geometry constants below must match urdf/scara_arm.urdf.xacro.
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker

# ---- geometry (keep in sync with the URDF) ---------------------------------
LINK1_LEN = 0.30
LINK2_LEN = 0.25
SLIDER_TRAVEL = 0.20
FINGER_TRAVEL = 0.02        # 0 = closed, FINGER_TRAVEL = fully open
GRIPPER_BASE_Z = 0.02
FINGER_Z = 0.05
CHASSIS_HEIGHT_Z = 0.10      # wheel_radius + chassis_z/2

# Fingertip height above ground when joint3 is fully extended, with the
# crane lift and mobile base left at their defaults (see the reply for the
# stacked-height derivation). Used only to place the payload sensibly.
GROUND_PICK_Z = 0.150
TCP_OFFSET_Z = -(GRIPPER_BASE_Z / 2.0 + FINGER_Z)  # gripper_base -> held payload
PAYLOAD_SIZE = 0.04

# ---- Cartesian pick / place targets, both within the arm's reach envelope --
PICK_XY = (0.35, 0.10)
PLACE_XY = (0.30, -0.25)
HOME_JOINTS = (0.0, 0.0)  # (joint1, joint2)

# ---- timing (seconds) -------------------------------------------------------
T_SWING = 3.0
T_VERTICAL = 1.5
T_GRIP = 1.0

JOINT_NAMES = [
    'front_left_wheel_joint', 'front_right_wheel_joint',
    'crane_lift_joint',
    'joint1', 'joint2', 'joint3', 'joint4',
    'left_finger_joint',
]


def solve_2r_ik(x, y, l1, l2):
    """Elbow-up planar 2-link IK. Returns (theta1, theta2) in radians."""
    r2 = x * x + y * y
    cos_t2 = (r2 - l1 * l1 - l2 * l2) / (2.0 * l1 * l2)
    cos_t2 = max(-1.0, min(1.0, cos_t2))
    t2 = math.acos(cos_t2)
    t1 = math.atan2(y, x) - math.atan2(l2 * math.sin(t2), l1 + l2 * math.cos(t2))
    return t1, t2


def lerp(a, b, frac):
    return a + (b - a) * frac


class PickAndPlaceDemo(Node):

    def __init__(self):
        super().__init__('pick_and_place_demo')

        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.marker_pub = self.create_publisher(Marker, '/payload_marker', 10)
        self._send_static_chassis_tf()

        self.pick_j1, self.pick_j2 = solve_2r_ik(*PICK_XY, LINK1_LEN, LINK2_LEN)
        self.place_j1, self.place_j2 = solve_2r_ik(*PLACE_XY, LINK1_LEN, LINK2_LEN)
        self.get_logger().info(
            f'Pick joints: j1={math.degrees(self.pick_j1):.1f} deg, '
            f'j2={math.degrees(self.pick_j2):.1f} deg'
        )
        self.get_logger().info(
            f'Place joints: j1={math.degrees(self.place_j1):.1f} deg, '
            f'j2={math.degrees(self.place_j2):.1f} deg'
        )

        # Sequence of (duration, function(frac) -> None) steps. Each function
        # sets self.j1/j2/j3/finger for the given fraction [0..1] of that step.
        self.steps = [
            (T_SWING, self._step_swing(HOME_JOINTS, (self.pick_j1, self.pick_j2))),
            (T_VERTICAL, self._step_vertical(0.0, -SLIDER_TRAVEL)),
            (T_GRIP, self._step_grip(FINGER_TRAVEL, 0.0, on_done=self._attach_payload)),
            (T_VERTICAL, self._step_vertical(-SLIDER_TRAVEL, 0.0)),
            (T_SWING, self._step_swing((self.pick_j1, self.pick_j2), (self.place_j1, self.place_j2))),
            (T_VERTICAL, self._step_vertical(0.0, -SLIDER_TRAVEL)),
            (T_GRIP, self._step_grip(0.0, FINGER_TRAVEL, on_done=self._release_payload)),
            (T_VERTICAL, self._step_vertical(-SLIDER_TRAVEL, 0.0)),
            (T_SWING, self._step_swing((self.place_j1, self.place_j2), HOME_JOINTS)),
        ]
        self.step_index = 0
        self.step_elapsed = 0.0

        # Live joint values.
        self.j1, self.j2, self.j3, self.j4, self.finger = 0.0, 0.0, 0.0, 0.0, FINGER_TRAVEL

        # Payload state.
        self.attached = False
        self.payload_world_xyz = (PICK_XY[0], PICK_XY[1], GROUND_PICK_Z)

        self.dt = 0.05
        self.timer = self.create_timer(self.dt, self._tick)
        self.get_logger().info('pick_and_place_demo started: pick -> place -> home, then holds.')

    # -- static chassis anchor (no fake_diff_drive running in this demo) ----
    def _send_static_chassis_tf(self):
        static_bc = StaticTransformBroadcaster(self)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'chassis_link'
        t.transform.translation.z = CHASSIS_HEIGHT_Z
        t.transform.rotation.w = 1.0
        static_bc.sendTransform(t)

    # -- step factories: each returns a function(frac) that updates self.j*  -
    def _step_swing(self, start_j12, end_j12):
        def _run(frac):
            self.j1 = lerp(start_j12[0], end_j12[0], frac)
            self.j2 = lerp(start_j12[1], end_j12[1], frac)
        return _run

    def _step_vertical(self, start_j3, end_j3):
        def _run(frac):
            self.j3 = lerp(start_j3, end_j3, frac)
        return _run

    def _step_grip(self, start_finger, end_finger, on_done=None):
        def _run(frac):
            self.finger = lerp(start_finger, end_finger, frac)
            if frac >= 1.0 and on_done is not None:
                on_done()
        return _run

    def _attach_payload(self):
        if not self.attached:
            self.attached = True
            self.get_logger().info('Gripper closed -- payload attached.')

    def _release_payload(self):
        if self.attached:
            self.attached = False
            self.payload_world_xyz = (PLACE_XY[0], PLACE_XY[1], GROUND_PICK_Z)
            self.get_logger().info('Gripper opened -- payload placed.')

    # -- main loop ------------------------------------------------------------
    def _tick(self):
        if self.step_index < len(self.steps):
            duration, step_fn = self.steps[self.step_index]
            self.step_elapsed += self.dt
            frac = min(1.0, self.step_elapsed / duration)
            step_fn(frac)
            if frac >= 1.0:
                self.step_index += 1
                self.step_elapsed = 0.0
                if self.step_index >= len(self.steps):
                    self.get_logger().info('Sequence complete -- holding at home.')

        self._publish_joint_states()
        self._publish_payload_marker()

    def _publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = [0.0, 0.0, 0.0, self.j1, self.j2, self.j3, self.j4, self.finger]
        self.joint_pub.publish(msg)

    def _publish_payload_marker(self):
        m = Marker()
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'payload'
        m.id = 0
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.scale.x = m.scale.y = m.scale.z = PAYLOAD_SIZE
        m.color.r, m.color.g, m.color.b, m.color.a = 0.95, 0.85, 0.10, 1.0
        m.pose.orientation.w = 1.0
        if self.attached:
            # Marker pose is expressed relative to whatever frame_id we give it --
            # RViz resolves it through TF itself, so this just rides on the gripper.
            m.header.frame_id = 'gripper_base'
            m.pose.position.z = TCP_OFFSET_Z
        else:
            m.header.frame_id = 'world'
            x, y, z = self.payload_world_xyz
            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.position.z = z
        self.marker_pub.publish(m)


def main():
    rclpy.init()
    node = PickAndPlaceDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
