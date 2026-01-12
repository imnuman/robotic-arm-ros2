# Robotic Arm Control with ROS2

ROS2-based control system for 6-DOF robotic arms with AI vision and Bengali voice command support.

![Robotic Arm Demo](assets/demo.gif)

## Features

- **ROS2 Humble**: Latest ROS2 LTS release support
- **AI Vision**: Object detection and pose estimation
- **Voice Control**: Bengali language voice commands
- **MoveIt2**: Advanced motion planning
- **Gripper Control**: Multiple gripper types supported
- **Simulation**: Gazebo integration for testing

## Hardware Support

| Arm | DOF | Controller | Status |
|-----|-----|------------|--------|
| xArm | 6 | xArm SDK | Supported |
| MyCobot | 6 | Elephant Robotics | Supported |
| Interbotix | 5/6 | Dynamixel | Supported |
| Custom Servo | 6 | PCA9685/Arduino | Supported |

## Installation

### Prerequisites

- Ubuntu 22.04
- ROS2 Humble
- Python 3.10+
- NVIDIA GPU (optional, for AI features)

### Install ROS2 Humble

```bash
# Add ROS2 repository
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Humble
sudo apt update
sudo apt install ros-humble-desktop

# Source ROS2
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Install Package

```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone repository
git clone https://github.com/imnuman/robotic-arm-ros2.git

# Install dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --symlink-install

# Source workspace
source install/setup.bash
```

## Usage

### Launch Arm Controller

```bash
# Launch with real hardware
ros2 launch arm_control arm_bringup.launch.py hardware:=true

# Launch in simulation
ros2 launch arm_control arm_bringup.launch.py hardware:=false

# With MoveIt2 planning
ros2 launch arm_control arm_moveit.launch.py
```

### Basic Control

```python
import rclpy
from arm_control import ArmController

rclpy.init()
arm = ArmController()

# Move to home position
arm.home()

# Move to joint positions (radians)
arm.move_joints([0, -0.5, 0.5, 0, 0.5, 0])

# Move to Cartesian position
arm.move_cartesian(x=0.3, y=0.1, z=0.2)

# Gripper control
arm.gripper_open()
arm.gripper_close()

# Pick and place
arm.pick(x=0.25, y=0.0, z=0.1)
arm.place(x=0.25, y=0.2, z=0.1)
```

### Voice Control

```bash
# Start Bengali voice control node
ros2 run arm_control voice_control_node

# Supported commands:
# "ধরো" - Close gripper
# "ছাড়ো" - Open gripper
# "বামে" - Move left
# "ডানে" - Move right
# "উপরে" - Move up
# "নিচে" - Move down
# "হোম" - Go to home position
```

### Vision-Guided Pick

```bash
# Start vision node
ros2 run arm_control vision_node

# Start pick-place node
ros2 run arm_control pick_place_node
```

```python
from arm_control import VisionPickPlace

picker = VisionPickPlace()

# Pick detected object
picker.pick_object(class_name='bottle')

# Place at location
picker.place_at(x=0.3, y=0.2, z=0.1)
```

## Architecture

```
robotic-arm-ros2/
├── arm_control/               # Main package
│   ├── arm_control/
│   │   ├── __init__.py
│   │   ├── arm_controller.py  # Core controller
│   │   ├── motion_planner.py  # Motion planning
│   │   ├── gripper.py         # Gripper control
│   │   ├── vision_node.py     # Object detection
│   │   └── voice_node.py      # Voice commands
│   ├── launch/
│   │   ├── arm_bringup.launch.py
│   │   └── arm_moveit.launch.py
│   ├── config/
│   │   ├── arm_config.yaml
│   │   └── controllers.yaml
│   └── package.xml
├── arm_description/           # URDF/meshes
│   ├── urdf/
│   │   └── arm.urdf.xacro
│   └── meshes/
├── arm_moveit_config/         # MoveIt2 config
│   └── config/
└── arm_msgs/                  # Custom messages
    └── msg/
        ├── JointCommand.msg
        └── GripperState.msg
```

## Topics & Services

### Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/joint_states` | sensor_msgs/JointState | Current joint positions |
| `/arm/target_joints` | arm_msgs/JointCommand | Target joint command |
| `/arm/gripper/state` | arm_msgs/GripperState | Gripper state |
| `/voice_commands` | std_msgs/String | Voice commands |
| `/detections` | vision_msgs/Detection2DArray | Object detections |

### Services

| Service | Type | Description |
|---------|------|-------------|
| `/arm/home` | std_srvs/Trigger | Move to home |
| `/arm/move_joints` | arm_msgs/MoveJoints | Move joints |
| `/arm/move_cartesian` | arm_msgs/MoveCartesian | Move Cartesian |
| `/gripper/open` | std_srvs/Trigger | Open gripper |
| `/gripper/close` | std_srvs/Trigger | Close gripper |

## Configuration

### Arm Configuration

```yaml
# config/arm_config.yaml
arm:
  type: "6dof_servo"
  controller: "pca9685"

  joints:
    - name: "base"
      channel: 0
      min_angle: -90
      max_angle: 90
      home: 0

    - name: "shoulder"
      channel: 1
      min_angle: -90
      max_angle: 90
      home: -45

    # ... more joints

gripper:
  type: "servo"
  channel: 6
  open_position: 0
  closed_position: 90
```

### Hardware Interface

```yaml
# config/controllers.yaml
controller_manager:
  ros__parameters:
    update_rate: 100

    joint_trajectory_controller:
      type: joint_trajectory_controller/JointTrajectoryController

    gripper_controller:
      type: position_controllers/GripperActionController
```

## API Reference

### ArmController

```python
class ArmController:
    def __init__(
        self,
        config_file: str = 'arm_config.yaml',
        use_simulation: bool = False
    ):
        ...

    def home(self) -> bool:
        """Move to home position."""

    def move_joints(
        self,
        positions: List[float],
        speed: float = 1.0
    ) -> bool:
        """Move to joint positions (radians)."""

    def move_cartesian(
        self,
        x: float, y: float, z: float,
        roll: float = 0, pitch: float = 0, yaw: float = 0
    ) -> bool:
        """Move to Cartesian position."""

    def gripper_open(self) -> bool:
        """Open gripper."""

    def gripper_close(self, force: float = 0.5) -> bool:
        """Close gripper with specified force."""

    def pick(self, x: float, y: float, z: float) -> bool:
        """Pick object at position."""

    def place(self, x: float, y: float, z: float) -> bool:
        """Place object at position."""

    def get_pose(self) -> Tuple[List[float], List[float]]:
        """Get current pose (position, orientation)."""
```

## Simulation

### Gazebo

```bash
# Launch Gazebo simulation
ros2 launch arm_control arm_gazebo.launch.py

# In another terminal, control the arm
ros2 run arm_control demo_pick_place
```

### RViz2 Visualization

```bash
# Launch RViz2 with MoveIt2
ros2 launch arm_moveit_config moveit_rviz.launch.py
```

## Troubleshooting

### Common Issues

**Servo not moving**
```bash
# Check I2C connection
i2cdetect -y 1

# Test servo channel
ros2 run arm_control test_servo --channel 0
```

**MoveIt2 planning fails**
```bash
# Check URDF
ros2 run xacro xacro arm.urdf.xacro > arm.urdf
check_urdf arm.urdf
```

**Camera not detected**
```bash
# Check video devices
v4l2-ctl --list-devices

# Test camera
ros2 run image_tools cam2image
```

## Contributing

Contributions welcome! Areas needed:
- Additional arm drivers
- Gripper types
- Vision algorithms
- Voice command expansion

## License

MIT License - see [LICENSE](LICENSE)

## Acknowledgments

- [ROS2](https://docs.ros.org/)
- [MoveIt2](https://moveit.ros.org/)
- [Elephant Robotics](https://www.elephantrobotics.com/)

## Contact

- **Author**: Al Numan
- **Email**: admin@numanab.com
- **Project**: [ProjectX - BahuBol](https://projectxbd.com)

---

*Part of the BahuBol (বাহুবল) intelligent robotic arm project.*
