from glob import glob

from setuptools import find_packages, setup

package_name = 'collider'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Bartlomiej Szozda',
    maintainer_email='bartekszozda19@gmail.com',
    description=('Vision-guided drone control on ROS 2 with ArduPilot SITL and Gazebo: '
                 'target tracking, visual-servoing steering, safety supervision, '
                 'mission orchestration and YOLO vehicle detection.'),
    license='AGPL-3.0-only',
    entry_points={
        'console_scripts': [
            "starter_node = collider.src.steering.StarterNode:main",
            "tracker_node = collider.src.tracker.TrackerNode:main",
            "target_motion_node = collider.src.tracker.TargetMotionNode:main",
            "steering_node = collider.src.steering.SteeringNode:main",
            "safety_node = collider.src.safety.SafetyNode:main",
            "mission_node = collider.src.mission.MissionNode:main",
            "assistant_node = collider.src.assistant.AssistantNode:main",
            "car_detector_node = collider.src.detector.CarDetectorNode:main",
            "benchmark_yolo = collider.src.detector.BenchmarkYolo:main",
        ],
    },
)
