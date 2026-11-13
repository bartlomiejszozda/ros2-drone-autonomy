"""Shared pytest setup for the collider tests."""

import gc
import os
from pathlib import Path

import pytest

# Must be set before the first ROS context is created; see the XML file for why.
os.environ.setdefault('FASTRTPS_DEFAULT_PROFILES_FILE',
                      str(Path(__file__).with_name('fastdds_no_intraprocess.xml')))


@pytest.fixture(autouse=True, scope='module')
def collect_ros_garbage():
    """Free the ROS objects a test module leaves behind before the next module runs."""
    yield
    # test_flake8 forks worker processes. rclpy objects still waiting for the garbage
    # collector would be finalized inside those children, where the DDS threads they
    # belong to do not exist, and the workers segfault.
    gc.collect()
