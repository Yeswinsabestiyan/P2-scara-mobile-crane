import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('scara_arm_description')

    default_xacro_path = os.path.join(pkg_share, 'urdf', 'scara_arm.urdf.xacro')
    default_rviz_config = os.path.join(pkg_share, 'rviz', 'teleop.rviz')

    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=default_xacro_path,
        description='Absolute path to the SCARA robot xacro file',
    )

    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model'), ' sim_world:=true']),
        value_type=str,
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', default_rviz_config],
    )

    crosshair_node = Node(
        package='scara_arm_description',
        executable='gripper_crosshair.py',
        name='gripper_crosshair',
        output='screen',
    )

    return LaunchDescription([
        model_arg,
        robot_state_publisher_node,
        rviz_node,
        crosshair_node,
    ])
