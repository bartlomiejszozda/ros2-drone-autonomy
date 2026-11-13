"""
Start the Collider nodes with one command, optionally together with the simulator.

    ros2 launch collider collider.launch.py
    ros2 launch collider collider.launch.py backend:=rc
    ros2 launch collider collider.launch.py with_sim:=true
    ros2 launch collider collider.launch.py --show-args
    ros2 launch collider collider.launch.py use_sim_time:=False   # only if no simulator runs

Each node is still its own process. Steering starts disabled, the Starter waits for a goal,
and the safety supervisor only acts on a violation, so launching sends nothing to the vehicle.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    backend = LaunchConfiguration('backend')
    use_sim_time = LaunchConfiguration('use_sim_time')

    def collider_node(executable, name, condition=None, extra_parameters=()):
        return Node(
            package='collider',
            executable=executable,
            name=name,
            parameters=[params_file, {'use_sim_time': use_sim_time}, *extra_parameters],
            output='screen',
            emulate_tty=True,  # keep coloured, unbuffered log lines
            condition=condition,
        )

    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('ardupilot_gz_bringup'), 'launch', 'iris_runway.launch.py'])),
        condition=IfCondition(LaunchConfiguration('with_sim')),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('collider'), 'config', 'collider.yaml']),
            description='YAML file with parameters for all Collider nodes'),
        DeclareLaunchArgument(
            'backend', default_value='cmd_vel', choices=['cmd_vel', 'rc'],
            description='Steering backend: cmd_vel (GUIDED) or rc (LOITER)'),
        DeclareLaunchArgument(
            # Capitalised: the included simulator launch only accepts 'True' or 'False'.
            'use_sim_time', default_value='True', choices=['True', 'False'],
            description='Use Gazebo simulation time (/clock); False only without a simulator'),
        DeclareLaunchArgument(
            'with_sim', default_value='false',
            description='Also start Gazebo and ArduPilot SITL (iris_runway)'),
        DeclareLaunchArgument(
            'starter', default_value='true', description='Start the takeoff action server'),
        DeclareLaunchArgument(
            'steering', default_value='true', description='Start the steering node'),
        DeclareLaunchArgument(
            'safety', default_value='true',
            description='Start the safety supervisor (leave on when flying)'),
        DeclareLaunchArgument(
            'mission', default_value='true', description='Start the mission orchestrator'),
        DeclareLaunchArgument(
            'assistant', default_value='true',
            description='Start the AI assistant (needs OPENROUTER_API_KEY)'),
        DeclareLaunchArgument(
            'car_detector', default_value='true',
            description='Start the YOLO car detector (needs ultralytics)'),

        simulator,
        collider_node('safety_node', 'safety',
                      condition=IfCondition(LaunchConfiguration('safety'))),
        collider_node('tracker_node', 'tracker'),
        collider_node('target_motion_node', 'target_motion'),
        collider_node('starter_node', 'starter',
                      condition=IfCondition(LaunchConfiguration('starter'))),
        # A dictionary after the YAML file wins, so the launch argument overrides the file.
        collider_node('steering_node', 'steering',
                      condition=IfCondition(LaunchConfiguration('steering')),
                      extra_parameters=[{'backend': backend}]),
        # The mission switches to the mode the chosen backend needs: LOITER (5) for rc,
        # GUIDED (4) for cmd_vel.
        collider_node('mission_node', 'mission',
                      condition=IfCondition(LaunchConfiguration('mission')),
                      extra_parameters=[{'steering_mode': PythonExpression(
                          ["5 if '", backend, "' == 'rc' else 4"])}]),
        collider_node('assistant_node', 'assistant',
                      condition=IfCondition(LaunchConfiguration('assistant'))),
        collider_node('car_detector_node', 'car_detector',
                      condition=IfCondition(LaunchConfiguration('car_detector'))),
    ])
