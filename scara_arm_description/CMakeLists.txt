cmake_minimum_required(VERSION 3.8)
project(scara_arm_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf launch rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/fake_diff_drive.py
    scripts/pick_and_place_demo.py
    scripts/scara_teleop.py
    scripts/crane_control_pad.py
    scripts/gripper_crosshair.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
