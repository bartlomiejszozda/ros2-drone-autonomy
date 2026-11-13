"""
Keep the target centred by steering yaw and climb, through RC or cmd_vel.

Steering starts disabled. Enable it with the /steering/enable service once the drone
is flying in the backend's mode (GUIDED for cmd_vel, LOITER for RC).
"""

import math
import signal

import rclpy
from ardupilot_msgs.msg import Status
from collider_interfaces.msg import TargetTrack
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_srvs.srv import SetBool

from collider.src.node_time import age, now_sec
from collider.src.steering.Centering import CenteringController, CenteringGains, heading_deg
from collider.src.steering.SteeringBackends import CmdVelBackend, RcBackend, StickScale
from collider.src.tracker.TargetMotionEstimator import TrackEstimate
from collider.src.tracker.TargetMotionNode import stamp_to_sec

MODE_NAMES = {4: 'GUIDED', 5: 'LOITER'}


def track_from_message(msg: TargetTrack) -> TrackEstimate | None:
    if not msg.valid:
        return None
    return TrackEstimate(
        stamp_sec=stamp_to_sec(msg.header.stamp), measured=msg.measured,
        measurements=msg.measurements, age_sec=msg.age_sec,
        bearing_deg=msg.bearing_deg, elevation_deg=msg.elevation_deg,
        bearing_rate_dps=msg.bearing_rate_dps, elevation_rate_dps=msg.elevation_rate_dps)


def connect_mavlink(url: str, timeout_sec: float):
    from pymavlink import mavutil
    connection = mavutil.mavlink_connection(url)
    if connection.wait_heartbeat(timeout=timeout_sec) is None:
        raise RuntimeError(f'No MAVLink heartbeat on {url} within {timeout_sec} s')
    return connection


class SteeringNode(Node):
    def __init__(self, backend=None, **kwargs):
        super().__init__('steering', **kwargs)
        kind = self.declare_parameter('backend', 'cmd_vel').value
        track_topic = self.declare_parameter('track_topic', '/collider/target_track').value
        pose_topic = self.declare_parameter('pose_topic', '/ap/v1/pose/filtered').value
        status_topic = self.declare_parameter('status_topic', '/ap/v1/status').value
        cmd_vel_topic = self.declare_parameter('cmd_vel_topic', '/ap/v1/cmd_vel').value
        mavlink_url = self.declare_parameter('mavlink_url', 'udp:localhost:14550').value
        rate_hz = float(self.declare_parameter('rate_hz', 10.0).value)
        self._telemetry_timeout = float(
            self.declare_parameter('telemetry_timeout_sec', 1.0).value)
        self._track_timeout = float(self.declare_parameter('track_timeout_sec', 0.5).value)
        defaults = CenteringGains()
        gains = CenteringGains(**{
            name: float(self.declare_parameter(name, getattr(defaults, name)).value)
            for name in vars(defaults)})
        self._controller = CenteringController(gains)

        if backend is not None:
            self._backend = backend
        elif kind == 'cmd_vel':
            self._backend = CmdVelBackend(self, cmd_vel_topic)
        elif kind == 'rc':
            self._backend = RcBackend(connect_mavlink(mavlink_url, 10.0), StickScale())
        else:
            raise ValueError(f"backend must be 'cmd_vel' or 'rc', not {kind!r}")

        self._enabled = False
        self._commanding = False
        self._status = None
        self._status_time = -math.inf
        self._attitude = None
        self._pose_stamp = None
        self._pose_time = -math.inf
        self._track = None
        self._track_time = -math.inf
        self.create_subscription(Status, status_topic, self._on_status, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, pose_topic, self._on_pose, qos_profile_sensor_data)
        self.create_subscription(TargetTrack, track_topic, self._on_track, 10)
        self.create_service(SetBool, '/steering/enable', self._on_enable)
        self.create_timer(1.0 / rate_hz, self._tick)
        mode = MODE_NAMES.get(self._backend.required_mode(), self._backend.required_mode())
        self.get_logger().info(
            f'{self._backend.name()} ready; needs {mode}. Enable with /steering/enable')

    def _on_status(self, msg: Status):
        self._status = msg
        self._status_time = now_sec(self)

    def _on_pose(self, msg: PoseStamped):
        o = msg.pose.orientation
        self._attitude = (o.x, o.y, o.z, o.w)
        self._pose_stamp = stamp_to_sec(msg.header.stamp)
        self._pose_time = now_sec(self)

    def _on_track(self, msg: TargetTrack):
        self._track = track_from_message(msg)
        self._track_time = now_sec(self)

    def _on_enable(self, request, response):
        self._enabled = request.data
        if not self._enabled:
            self._release('disabled')
        response.success = True
        response.message = 'steering enabled' if self._enabled else 'steering disabled'
        self.get_logger().info(response.message)
        return response

    def _tick(self):
        if not self._enabled:
            return
        problem = self._vehicle_problem()
        if problem is not None:
            self._release(problem)
            return
        track = self._usable_track()
        # No usable track: still command, but zero motion, so the drone hovers in place.
        motion = self._controller.compute(track, self._attitude, self._pose_stamp)
        self._backend.send(motion, heading_deg(self._attitude))
        if not self._commanding:
            self.get_logger().info('Steering active')
        self._commanding = True

    def _vehicle_problem(self) -> str | None:
        now = now_sec(self)
        if age(now, self._status_time) > self._telemetry_timeout or self._status is None:
            return 'no fresh status'
        if age(now, self._pose_time) > self._telemetry_timeout or self._attitude is None:
            return 'no fresh pose'
        if not (self._status.armed and self._status.flying):
            return 'vehicle is not armed and flying'
        required = self._backend.required_mode()
        if self._status.mode != required:
            return f'vehicle mode is {self._status.mode}, steering needs ' \
                   f'{MODE_NAMES.get(required, required)}'
        return None

    def _usable_track(self) -> TrackEstimate | None:
        if age(now_sec(self), self._track_time) > self._track_timeout:
            return None
        return self._track

    def _release(self, reason: str):
        """Send one stop command after commanding; do nothing more until conditions recover."""
        if self._commanding:
            self._backend.stop()
            self._commanding = False
            self.get_logger().warning(f'Steering paused: {reason}')
        elif self._enabled:
            self.get_logger().warning(f'Steering waiting: {reason}', throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = SingleThreadedExecutor()
    try:
        node = SteeringNode()
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Under ros2 launch, Ctrl+C reaches the node twice (terminal and launch);
        # ignore the second one so it cannot interrupt the clean-up below.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if node is not None:
            try:
                node._release('shutting down')
            except Exception as error:  # the ROS context may already be gone
                print(f'Could not send the final stop command: {error}')
        executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
