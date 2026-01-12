"""
Robotic Arm Controller
======================
ROS2-based controller for 6-DOF robotic arms.

Author: Al Numan
Project: BahuBol - ProjectX
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger
from geometry_msgs.msg import Pose, Point, Quaternion
import numpy as np
from typing import List, Tuple, Optional
import logging
import time

logger = logging.getLogger(__name__)


class ArmController(Node):
    """
    ROS2 controller for 6-DOF robotic arms.

    Supports multiple hardware interfaces and provides
    high-level motion commands.

    Example:
        arm = ArmController()
        arm.home()
        arm.move_joints([0, -0.5, 0.5, 0, 0.5, 0])
        arm.pick(0.3, 0.0, 0.1)
    """

    # Default joint configuration
    DEFAULT_JOINTS = ['base', 'shoulder', 'elbow', 'wrist1', 'wrist2', 'wrist3']

    # Default home position (radians)
    DEFAULT_HOME = [0.0, -0.785, 0.785, 0.0, 0.785, 0.0]

    def __init__(
        self,
        node_name: str = 'arm_controller',
        config_file: str = None,
        use_simulation: bool = False
    ):
        """
        Initialize arm controller.

        Args:
            node_name: ROS2 node name
            config_file: Path to configuration YAML
            use_simulation: Use simulated hardware
        """
        super().__init__(node_name)

        self.use_simulation = use_simulation
        self.joint_names = self.DEFAULT_JOINTS
        self.home_position = self.DEFAULT_HOME
        self.num_joints = len(self.joint_names)

        # Current state
        self._current_joints = [0.0] * self.num_joints
        self._current_pose = None
        self._gripper_state = 'open'

        # Load configuration
        if config_file:
            self._load_config(config_file)

        # Setup publishers
        self.joint_pub = self.create_publisher(
            Float64MultiArray,
            '/arm/joint_commands',
            10
        )

        self.gripper_pub = self.create_publisher(
            Float64MultiArray,
            '/arm/gripper_command',
            10
        )

        # Setup subscribers
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_callback,
            10
        )

        # Setup services
        self.home_srv = self.create_service(
            Trigger,
            '/arm/home',
            self._home_service_callback
        )

        self.get_logger().info(f"Arm controller initialized ({'simulation' if use_simulation else 'hardware'})")

    def _load_config(self, config_file: str):
        """Load configuration from YAML file."""
        import yaml

        try:
            with open(config_file, 'r') as f:
                config = yaml.safe_load(f)

            arm_config = config.get('arm', {})
            self.joint_names = [j['name'] for j in arm_config.get('joints', [])]
            self.home_position = [j.get('home', 0.0) for j in arm_config.get('joints', [])]
            self.num_joints = len(self.joint_names)

            self.get_logger().info(f"Loaded config: {config_file}")

        except Exception as e:
            self.get_logger().error(f"Failed to load config: {e}")

    def _joint_state_callback(self, msg: JointState):
        """Handle joint state updates."""
        for i, name in enumerate(msg.name):
            if name in self.joint_names:
                idx = self.joint_names.index(name)
                if idx < len(msg.position):
                    self._current_joints[idx] = msg.position[idx]

    def _home_service_callback(self, request, response):
        """Service callback for home command."""
        success = self.home()
        response.success = success
        response.message = "Moved to home" if success else "Failed to move home"
        return response

    def home(self) -> bool:
        """
        Move arm to home position.

        Returns:
            True if successful
        """
        self.get_logger().info("Moving to home position")
        return self.move_joints(self.home_position, speed=0.5)

    def move_joints(
        self,
        positions: List[float],
        speed: float = 1.0,
        wait: bool = True
    ) -> bool:
        """
        Move to specified joint positions.

        Args:
            positions: Target joint positions (radians)
            speed: Movement speed factor (0.0-1.0)
            wait: Wait for completion

        Returns:
            True if successful
        """
        if len(positions) != self.num_joints:
            self.get_logger().error(
                f"Expected {self.num_joints} joints, got {len(positions)}"
            )
            return False

        # Validate positions
        positions = [float(p) for p in positions]

        # Publish command
        msg = Float64MultiArray()
        msg.data = positions

        self.joint_pub.publish(msg)
        self.get_logger().info(f"Moving joints: {positions}")

        if wait:
            return self._wait_for_joints(positions)

        return True

    def _wait_for_joints(
        self,
        target: List[float],
        tolerance: float = 0.05,
        timeout: float = 10.0
    ) -> bool:
        """Wait for joints to reach target."""
        start_time = time.time()

        while time.time() - start_time < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)

            # Check if reached
            errors = [
                abs(t - c) for t, c in zip(target, self._current_joints)
            ]

            if all(e < tolerance for e in errors):
                self.get_logger().info("Target reached")
                return True

        self.get_logger().warning("Timeout waiting for joints")
        return False

    def move_cartesian(
        self,
        x: float,
        y: float,
        z: float,
        roll: float = 0.0,
        pitch: float = 0.0,
        yaw: float = 0.0,
        speed: float = 1.0
    ) -> bool:
        """
        Move to Cartesian position using inverse kinematics.

        Args:
            x, y, z: Position (meters)
            roll, pitch, yaw: Orientation (radians)
            speed: Movement speed factor

        Returns:
            True if successful
        """
        # Compute inverse kinematics
        joints = self._inverse_kinematics(x, y, z, roll, pitch, yaw)

        if joints is None:
            self.get_logger().error("IK solution not found")
            return False

        return self.move_joints(joints, speed=speed)

    def _inverse_kinematics(
        self,
        x: float,
        y: float,
        z: float,
        roll: float,
        pitch: float,
        yaw: float
    ) -> Optional[List[float]]:
        """
        Compute inverse kinematics.

        Simple geometric IK for 6-DOF arm.
        For production, use MoveIt2 IK solver.
        """
        # Arm dimensions (example for generic 6-DOF)
        L1 = 0.10  # Base height
        L2 = 0.20  # Upper arm
        L3 = 0.20  # Forearm
        L4 = 0.10  # Wrist

        try:
            # Base rotation
            q1 = np.arctan2(y, x)

            # Distance in XY plane
            r = np.sqrt(x*x + y*y)

            # Height above base
            h = z - L1

            # Distance to wrist
            d = np.sqrt(r*r + h*h)

            # Elbow angle (law of cosines)
            cos_q3 = (L2*L2 + L3*L3 - d*d) / (2*L2*L3)
            cos_q3 = np.clip(cos_q3, -1, 1)
            q3 = np.pi - np.arccos(cos_q3)

            # Shoulder angle
            alpha = np.arctan2(h, r)
            beta = np.arccos((L2*L2 + d*d - L3*L3) / (2*L2*d))
            q2 = -(alpha + beta)

            # Wrist angles (simplified)
            q4 = -(q2 + q3) + pitch
            q5 = roll
            q6 = yaw - q1

            return [q1, q2, q3, q4, q5, q6]

        except Exception as e:
            self.get_logger().error(f"IK failed: {e}")
            return None

    def gripper_open(self) -> bool:
        """Open the gripper."""
        return self._gripper_command(0.0)

    def gripper_close(self, force: float = 0.5) -> bool:
        """
        Close the gripper.

        Args:
            force: Grip force (0.0-1.0)
        """
        return self._gripper_command(1.0, force)

    def _gripper_command(self, position: float, force: float = 0.5) -> bool:
        """Send gripper command."""
        msg = Float64MultiArray()
        msg.data = [position, force]

        self.gripper_pub.publish(msg)
        self._gripper_state = 'closed' if position > 0.5 else 'open'

        self.get_logger().info(f"Gripper: {self._gripper_state}")
        return True

    def pick(
        self,
        x: float,
        y: float,
        z: float,
        approach_height: float = 0.1
    ) -> bool:
        """
        Pick object at specified position.

        Args:
            x, y, z: Object position
            approach_height: Height above object for approach

        Returns:
            True if successful
        """
        self.get_logger().info(f"Picking at ({x}, {y}, {z})")

        # Open gripper
        self.gripper_open()
        time.sleep(0.5)

        # Approach from above
        if not self.move_cartesian(x, y, z + approach_height):
            return False

        # Move down
        if not self.move_cartesian(x, y, z, speed=0.3):
            return False

        # Close gripper
        self.gripper_close()
        time.sleep(0.5)

        # Lift
        if not self.move_cartesian(x, y, z + approach_height):
            return False

        return True

    def place(
        self,
        x: float,
        y: float,
        z: float,
        approach_height: float = 0.1
    ) -> bool:
        """
        Place object at specified position.

        Args:
            x, y, z: Target position
            approach_height: Height above target for approach

        Returns:
            True if successful
        """
        self.get_logger().info(f"Placing at ({x}, {y}, {z})")

        # Approach from above
        if not self.move_cartesian(x, y, z + approach_height):
            return False

        # Move down
        if not self.move_cartesian(x, y, z, speed=0.3):
            return False

        # Open gripper
        self.gripper_open()
        time.sleep(0.5)

        # Retract
        if not self.move_cartesian(x, y, z + approach_height):
            return False

        return True

    def get_joint_positions(self) -> List[float]:
        """Get current joint positions."""
        return self._current_joints.copy()

    def get_pose(self) -> Tuple[List[float], List[float]]:
        """
        Get current end-effector pose.

        Returns:
            Tuple of (position [x,y,z], orientation [roll,pitch,yaw])
        """
        # Forward kinematics (simplified)
        pos, orient = self._forward_kinematics(self._current_joints)
        return pos, orient

    def _forward_kinematics(
        self,
        joints: List[float]
    ) -> Tuple[List[float], List[float]]:
        """Compute forward kinematics."""
        # Simplified FK for demo
        L1, L2, L3, L4 = 0.10, 0.20, 0.20, 0.10

        q1, q2, q3, q4, q5, q6 = joints

        x = (L2*np.cos(q2) + L3*np.cos(q2+q3)) * np.cos(q1)
        y = (L2*np.cos(q2) + L3*np.cos(q2+q3)) * np.sin(q1)
        z = L1 + L2*np.sin(q2) + L3*np.sin(q2+q3)

        roll = q5
        pitch = q2 + q3 + q4
        yaw = q1 + q6

        return [x, y, z], [roll, pitch, yaw]


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)

    arm = ArmController()

    try:
        rclpy.spin(arm)
    except KeyboardInterrupt:
        pass
    finally:
        arm.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
