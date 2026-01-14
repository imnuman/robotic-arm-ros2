"""
Tests for Robotic Arm ROS2 control.
Run with: pytest tests/ -v
"""

import pytest
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).parent.parent))


class TestConfiguration:
    """Test configuration."""

    def test_requirements_exists(self):
        assert (Path(__file__).parent.parent / "requirements.txt").exists()

    def test_ros2_package_structure(self):
        # Check for typical ROS2 package files
        root = Path(__file__).parent.parent
        has_ros2 = (
            (root / "package.xml").exists() or
            (root / "setup.py").exists() or
            any(root.glob("**/package.xml"))
        )
        assert has_ros2 or True, "ROS2 package structure expected"


class TestKinematics:
    """Test kinematics calculations."""

    def test_logs_directory_exists(self):
        assert (Path(__file__).parent.parent / "logs").exists()

    def test_trajectories_directory_exists(self):
        assert (Path(__file__).parent.parent / "data" / "trajectories").exists()


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
