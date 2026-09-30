# P2 - SCARA Mobile Crane (Complete Enhanced Edition)

A comprehensive ROS 2 package featuring a mobile 4-DOF SCARA robotic arm mounted on a differential-drive crane chassis. Includes telescoping vertical mast lift, ground-reaching prismatic slider, interactive multi-color cargo crates, real-time keyboard teleoperation, Tkinter GUI Control Pad with Live Telemetry HUD, and custom RViz2 visualization overlays.

---

## Highlights & Features

- **Telescoping Crane Mast + Full Ground Reach**:
  - Prismatic crane column lift for vertical workspace expansion.
  - Prismatic z-slider joint capable of reaching directly to floor level for cargo pickup.
- **Interactive Multi-Color Cargo System**:
  - 3 cargo crates (**Yellow**, **Red**, and **Blue**) simulated directly in RViz.
  - Dynamic attachment and cargo respawning functionality.
- **GUI Control Pad with Telemetry HUD**:
  - Built-in Tkinter interface (`crane_control_pad.py`).
  - Hold-to-jog buttons for smooth continuous joint & mobile base actuation.
  - Live coordinates (X/Y position, heading angle, lift height, joint angles).
  - Distance-to-cargo sensor readout with visual progress bars and dynamic **READY-TO-GRASP** indicator.
  - 1-Click **AUTO PICK & PLACE** routine.
  - Instant preset poses: `STOW`, `PICK (ground)`, and `DELIVER (drop pad)`.
- **Keyboard Teleoperation**:
  - Full-featured keyboard teleop (`scara_teleop.py`) with 3 selectable speed profiles (`SLOW`, `NORMAL`, `FAST`).
- **RViz Visual Overlays**:
  - Downward laser crosshair for precision overhead targeting.
  - Real-time 3D end-effector trajectory trail.
  - Workspace reachability boundary ring display.

---

## Project Structure

```text
P2-scara-mobile-crane/
├── scara_arm_description/
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── launch/
│   │   ├── display.launch.py             # Basic model inspection with Joint State Publisher GUI
│   │   ├── pick_and_place_demo.launch.py # Autonomous pick-and-place demonstration launch
│   │   └── teleop.launch.py              # Main interactive simulation with RViz & visual overlays
│   ├── rviz/
│   │   ├── scara.rviz                    # Default RViz configuration
│   │   └── teleop.rviz                   # Teleoperation RViz layout with camera tracking & markers
│   ├── scripts/
│   │   ├── crane_control_pad.py          # Tkinter GUI Control Pad with Telemetry HUD
│   │   ├── fake_diff_drive.py            # Differential drive odometry and transform broadcaster
│   │   ├── gripper_crosshair.py          # Visual overlays (crosshairs, trajectory trail, workspace ring)
│   │   ├── pick_and_place_demo.py        # Automated pick & place trajectory executor
│   │   └── scara_teleop.py               # Interactive keyboard teleoperation node
│   └── urdf/
│       └── scara_arm.urdf.xacro          # URDF/Xacro kinematic description with mobile base & crane
├── controls.txt                          # Quick reference guide for keyboard & GUI controls
├── P2 -.txt                              # Project specification and startup workflow
├── .gitignore
└── README.md
```

---

## Robot Kinematics & Joint Mapping

| Joint Name | Type | Axis | Description |
| :--- | :--- | :--- | :--- |
| `crane_column_joint` | Prismatic | Z | Vertical telescoping mast lift |
| `arm_joint_1` (Shoulder) | Revolute | Z | Primary SCARA horizontal link |
| `arm_joint_2` (Elbow) | Revolute | Z | Secondary SCARA horizontal link |
| `arm_joint_3` (Slider) | Prismatic | Z | Vertical ground-reach plunger |
| `arm_joint_4` (Wrist) | Revolute | Z | End-effector orientation |
| `gripper_left_joint` / `gripper_right_joint` | Prismatic | Y | Parallel jaw gripper fingers |
| `left_wheel_joint` / `right_wheel_joint` | Continuous | Y | Mobile base differential drive |

---

## Prerequisites & Dependencies

Make sure you have a working ROS 2 installation (Humble, Iron, or newer) on Ubuntu / WSL2:

```bash
sudo apt update
sudo apt install -y \
  ros-${ROS_DISTRO}-robot-state-publisher \
  ros-${ROS_DISTRO}-joint-state-publisher \
  ros-${ROS_DISTRO}-joint-state-publisher-gui \
  ros-${ROS_DISTRO}-rviz2 \
  ros-${ROS_DISTRO}-xacro \
  python3-tk
```

---

## Build & Installation

1. Clone or copy `scara_arm_description` into your ROS 2 workspace `src/` folder:
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/Yeswinsabestiyan/P2-scara-mobile-crane.git
   ```

2. Make Python scripts executable:
   ```bash
   chmod +x P2-scara-mobile-crane/scara_arm_description/scripts/*.py
   ```

3. Build the package with `colcon`:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select scara_arm_description
   source install/setup.bash
   ```

---

## Usage Guide

### Terminal 1: Launch Robot Simulation & RViz

Launches the complete mobile crane model, differential drive publisher, RViz2 with pre-configured camera view, and downward visual targeting overlays:

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch scara_arm_description teleop.launch.py
```

### Terminal 2: Keyboard Teleoperation

Control the mobile base, crane mast, SCARA joints, and gripper from your keyboard:

```bash
source ~/ros2_ws/install/setup.bash
ros2 run scara_arm_description scara_teleop.py
```

#### Keyboard Keybindings:
- **`W` / `S`** : Drive mobile base forward / reverse
- **`A` / `D`** : Turn mobile base left / right
- **`I` / `K`** : Raise / Lower telescoping crane mast
- **`J` / `L`** : Rotate SCARA shoulder (`arm_joint_1`)
- **`U` / `O`** : Rotate SCARA elbow (`arm_joint_2`)
- **`T` / `G`** : Lower / Raise vertical slider (`arm_joint_3`, reaches floor)
- **`R` / `F`** : Rotate wrist clockwise / counter-clockwise (`arm_joint_4`)
- **`SPACE`** : Toggle gripper (automatically locks onto cargo when aligned)
- **`N`** : Respawn cargo crate in front of crane
- **`H`** : Return arm to Home pose
- **`1` / `2` / `3`** : Speed mode (**1** = SLOW, **2** = NORMAL, **3** = FAST)
- **`Q`** : Quit teleop node

---

### Terminal 3 (Optional): GUI Control Pad & Telemetry HUD

Provides a graphical jog interface with real-time feedback:

```bash
source ~/ros2_ws/install/setup.bash
ros2 run scara_arm_description crane_control_pad.py
```

- **Hold-to-Jog Buttons**: Intuitive continuous axis movement.
- **Telemetry HUD**: Displays current coordinates $(X, Y, \theta)$, mast height, and joint angles.
- **Proximity Sensor**: Distance to cargo with color progress bar and `READY-TO-GRASP` indicator.
- **Presets**: `STOW`, `PICK (ground)`, and `DELIVER (pad)`.
- **Auto Routine**: 1-Click automated pick-and-place execution.

---

### Autonomous Pick & Place Demo

To launch an automated standalone demonstration:

```bash
ros2 launch scara_arm_description pick_and_place_demo.launch.py
```

---

## License

This project is licensed under the MIT License.
