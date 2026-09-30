#!/usr/bin/env python3
"""
fake_diff_drive.py

RViz has no physics engine, so joint_state_publisher_gui's wheel sliders
only spin the wheel MESHES in place -- nothing makes the chassis actually
translate. This node bridges that gap with simple dead-reckoning: it
watches how far the front_left / front_right wheel joints have turned
since the last tick, applies standard differential-drive kinematics, and
publishes a moving world -> chassis_link transform.

Drag front_left_wheel_joint and front_right_wheel_joint by the SAME
amount, in the same direction -> robot drives straight.
Drag them by different amounts -> robot turns (tank-drive style).
Rear wheels mimic the front ones automatically (see the URDF), so you
only ever need to touch these two sliders to "drive".

These constants must match urdf/scara_arm.urdf.xacro.
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

WHEEL_RADIUS = 0.06
WHEEL_TRACK = 0.34          # chassis_y (0.30) + wheel_width (0.04)
CHASSIS_HEIGHT_Z = 0.10     # wheel_radius (0.06) + chassis_z/2 (0.04)

LEFT_JOINT = 'front_left_wheel_joint'
RIGHT_JOINT = 'front_right_wheel_joint'


class FakeDiffDrive(Node):

    def __init__(self):
        super().__init__('fake_diff_drive')
        self.create_subscription(JointState, '/joint_states', self._on_joint_states, 10)
        self.broadcaster = TransformBroadcaster(self)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_left = None
        self.last_right = None

        self.get_logger().info(
            'fake_diff_drive ready: drag front_left_wheel_joint and '
            'front_right_wheel_joint in RViz to move the base.'
        )

    def _on_joint_states(self, msg: JointState):
        try:
            left = msg.position[msg.name.index(LEFT_JOINT)]
            right = msg.position[msg.name.index(RIGHT_JOINT)]
        except ValueError:
            return  # message doesn't (yet) include the wheel joints

        if self.last_left is not None:
            d_left = left - self.last_left
            d_right = right - self.last_right

            # Arc length each wheel rolled through this tick.
            s_left = WHEEL_RADIUS * d_left
            s_right = WHEEL_RADIUS * d_right

            d_center = (s_left + s_right) / 2.0
            d_theta = (s_right - s_left) / WHEEL_TRACK

            # Integrate pose using the heading at the START of this tick.
            self.x += d_center * math.cos(self.theta)
            self.y += d_center * math.sin(self.theta)
            self.theta += d_theta

            self._publish_tf()

        self.last_left = left
        self.last_right = right

    def _publish_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'chassis_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = CHASSIS_HEIGHT_Z
        t.transform.rotation.z = math.sin(self.theta / 2.0)
        t.transform.rotation.w = math.cos(self.theta / 2.0)
        self.broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = FakeDiffDrive()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
