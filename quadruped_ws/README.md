# my_bot ROS 2 Workspace

Quadruped simulation and IK testing workspace for **ROS 2 Humble** +
**Gazebo Sim (Ignition Fortress / Garden)**.

The URDF was exported from SolidWorks via sw_urdf_exporter and uses the
`gz_ros2_control` plugin — this means **Gazebo Sim (Ignition)**, not
Gazebo Classic. Make sure you have the correct simulator installed.

---

## Package layout

```
quadruped_ws/src/
├── my_bot_description/          # URDF + meshes + RViz config
│   ├── urdf/
│   │   └── my_robot.urdf        # unchanged SolidWorks export
│   ├── meshes/                  # ← drop your STL files here
│   ├── config/
│   │   └── ros2_control.yaml    # controller definitions
│   ├── launch/
│   │   └── display.launch.py    # RViz-only (no Gazebo)
│   └── rviz/
│       └── urdf.rviz
│
├── my_bot_gazebo/               # Gazebo Sim world + spawn
│   ├── worlds/
│   │   └── flat_ground.sdf
│   └── launch/
│       └── spawn.launch.py      # Gazebo + RSP + controllers
│
├── my_bot_control/              # IK math + ROS nodes (Python)
│   ├── my_bot_control/
│   │   ├── kinematics.py        # direct port of Kinematics.cpp + RobotConfig.cpp
│   │   └── ik_solver_node.py    # ROS 2 node: /foot_target → /joint_trajectory
│   ├── config/
│   │   └── robot_params.yaml    # link lengths + shoulder origins
│   └── test/
│       └── test_kinematics.py   # pure-Python unit tests (no ROS needed)
│
└── my_bot_bringup/              # Top-level launch orchestration
    └── launch/
        ├── sim.launch.py        # Full sim: Gazebo + IK node
        └── ik_test.launch.py    # Headless IK test: RViz only, no Gazebo
```

---

## Dependencies

### APT packages
```bash
sudo apt install -y \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-gz-ros2-control \
  ros-humble-ros-gz-sim \
  ros-humble-ros-gz-bridge \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-joint-trajectory-controller \
  ros-humble-velocity-controllers \
  ros-humble-joint-state-broadcaster \
  ros-humble-rviz2
```

---

## First-time setup

```bash
# 1. Source ROS 2 Humble
source /opt/ros/humble/setup.bash

# 2. Build (description first, it generates files other packages depend on)
cd ~/quadruped_ws
colcon build --packages-select my_bot_description
colcon build --packages-select my_bot_gazebo my_bot_control
colcon build --packages-select my_bot_bringup

# 3. Source the workspace overlay
source install/setup.bash
```

---

## Usage

### Step 1 — Inspect the URDF in RViz (no Gazebo)
```bash
ros2 launch my_bot_description display.launch.py
```
Move the sliders in the joint_state_publisher_gui to confirm all 16 joints
(12 leg + 4 wheel) articulate correctly.

---

### Step 2 — Test IK math without Gazebo
```bash
ros2 launch my_bot_bringup ik_test.launch.py
```
In a second terminal, publish a foot target for leg 1:
```bash
ros2 topic pub --once /foot_target geometry_msgs/PointStamped \
  '{header: {frame_id: "leg_1"}, point: {x: 0.0, y: 80.0, z: -200.0}}'
```
Watch the IK solver log for solved angles and inspect the robot in RViz.

---

### Step 3 — Full Gazebo simulation
```bash
# Drop your STL meshes into my_bot_description/meshes/ first, then rebuild.
ros2 launch my_bot_bringup sim.launch.py
```
Send foot targets exactly as in Step 2.

---

### Step 4 — Run IK unit tests (no ROS required)
```bash
cd src/my_bot_control
python3 -m pytest test/test_kinematics.py -v
```

---

## Topic reference

| Topic | Type | Direction |
|---|---|---|
| `/foot_target` | `geometry_msgs/PointStamped` | your code → IK node |
| `/leg_position_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | IK node → controller |
| `/joint_states` | `sensor_msgs/JointState` | controller → everyone |
| `/tf` `/tf_static` | `tf2_msgs/TFMessage` | RSP → everyone |

**`/foot_target` usage:**
- `header.frame_id` — which leg: `"leg_1"`, `"leg_2"`, `"leg_3"`, `"leg_4"`
- `point.x/y/z` — desired foot position in the **local leg frame** [mm]

---

## IK convention (from Kinematics.cpp)

```
theta1 = atan2(x, y)                       shoulder yaw
theta3 = atan2(sqrt(1-D²), D)             knee angle   (D = law of cosines)
theta2 = atan2(-z, r-a) - atan2(c·sin θ3, b + c·cos θ3)   femur angle

Link lengths (RobotConfig.cpp):
    a = 60 mm   (shoulder offset)
    b = 95 mm   (femur / upper leg)
    c = 180 mm  (tibia / lower leg)
```

All IK output is in **degrees** (matching the ESP servo values).
The IK node converts to **radians** before publishing to the ROS controller.

---

## Adding sensors later

1. Add the Gazebo plugin XML to `my_bot_description/urdf/my_robot.urdf`
   (the URDF already has `<gazebo>` sections as examples).
2. Add the sensor topic to the ros_gz_bridge in `spawn.launch.py`.
3. Create a new node in `my_bot_control/` or a new package
   `my_bot_perception/` — no existing nodes need to change.
