# ros2-drone-autonomy

[![CI](https://github.com/bartlomiejszozda/ros2-drone-autonomy/actions/workflows/ci.yml/badge.svg)](https://github.com/bartlomiejszozda/ros2-drone-autonomy/actions/workflows/ci.yml)
[![License: AGPL-3.0](https://img.shields.io/badge/license-AGPL--3.0-blue.svg)](LICENSE)

Vision-based autonomy for a quadcopter, built on ROS 2 Humble, ArduPilot Copter SITL and
Gazebo Harmonic. The drone takes off, finds a target in its onboard camera and steers
towards it with visual servoing, while a safety supervisor watches the flight. A YOLO
detector reports vehicles on the ground, and an LLM assistant answers questions about
what the camera sees.

<!-- demo: add docs/media/demo.gif (takeoff, target lock, pursuit) -->

## Features

- **Mission control** as ROS 2 actions: one `/mission/run` goal handles preflight checks,
  takeoff, steering and landing / RTL / hover, with feedback and cancellation.
- **Target tracking** from the onboard camera, published as azimuth/elevation observations.
  Trackers sit behind a common interface (blob tracker by default, OpenCV trackers available).
- **Target motion estimation** that removes the drone's own rotation from the camera
  angles before steering.
- **Visual-servoing steering** with two interchangeable backends: velocity commands in
  GUIDED mode, or RC override in LOITER.
- **Safety supervisor** that switches ArduPilot to BRAKE on limit violations or on demand.
- **Vehicle detection** with YOLO11n fine-tuned on top-down UAV imagery (VisDrone).
- **AI assistant**: ask questions about the current camera frame from the terminal
  (`ask where is the red car`), via OpenRouter.
- Runs on **simulation time**, so pausing Gazebo does not trigger timeouts.
- **Reproducible environment**: Docker image with the full toolchain, workspace pinned
  with `vcs`, optional NVIDIA GPU.

## Architecture

![Collider components in a seven-layer robotics stack](docs/architecture-layers.svg)

Collider covers perception, control, behavior and mission logic (layers 3, 4, 6 and 7).
ArduPilot SITL provides state estimation, the inner control loops and flight modes, and
Gazebo simulates the airframe, sensors and camera. Layer 5 is still empty: steering reacts
to the current target bearing and does not plan a path.

![Collider ROS 2 node graph](docs/node-graph.svg)

Nodes talk through topics for continuous data (images, observations) and actions for
anything with a start, progress and an end (takeoff, mission, assistant questions).
Only `steering`, `starter`, `safety` and `mission` send commands to ArduPilot.

**[Interactive architecture page](https://bartlomiejszozda.github.io/ros2-drone-autonomy/architecture.html)**:
both diagrams with link highlighting, launch arguments and ready-to-run commands.
See [docs/architecture.md](docs/architecture.md) for design decisions
and [docs/nodes.md](docs/nodes.md) for parameters and interfaces.

## Quick start

Requires Linux with Docker Engine; an NVIDIA GPU is optional. Full setup, including
remote use over SSH, is in [docker/README.md](docker/README.md).

```bash
mkdir -p ~/collider_ws/src && cd ~/collider_ws
git clone https://github.com/bartlomiejszozda/ros2-drone-autonomy.git src/collider
vcs import --recursive src < src/collider/docker/collider.repos

cd src/collider/docker
docker compose up -d --build
docker compose exec -u dev dev bash
```

Inside the container:

```bash
# collider_interfaces is nested in the repo, so its path is listed explicitly
colcon build --symlink-install --base-paths src src/collider/interfaces/collider_interfaces
source install/setup.bash
ros2 launch collider collider.launch.py with_sim:=true

# second shell: fly a 30 s mission, then launch the target
ros2 action send_goal /mission/run collider_interfaces/action/Mission \
  '{altitude_m: 5.0, duration_sec: 30.0, end_action: land}' --feedback
src/collider/scripts/launch_target
```

Camera stream in the browser: run `ros2 run web_video_server web_video_server --ros-args -p port:=8765`
and open `http://localhost:8765/stream?topic=/collider/cars/image`.

## Dependencies

The workspace is assembled with [vcstool](https://github.com/dirk-thomas/vcstool) from
[`docker/collider.repos`](docker/collider.repos); upstream repositories are pinned to
tested commits.

| Repository | Role |
|---|---|
| [ArduPilot](https://github.com/ArduPilot/ardupilot) | Copter SITL and its DDS interface (`ardupilot_msgs`) |
| [ardupilot_gazebo](https://github.com/bartlomiejszozda/ardupilot_gazebo/tree/collider) (fork) | Gazebo plugin; Iris model with a forward camera |
| [ardupilot_gz](https://github.com/bartlomiejszozda/ardupilot_gz/tree/collider) (fork) | Bringup and worlds; runway with the sphere target and cars, camera bridge, sim-time launch options |
| [SITL_Models](https://github.com/ArduPilot/SITL_Models) | Additional vehicle models |
| [micro-ROS-Agent](https://github.com/micro-ROS/micro-ROS-Agent) | DDS bridge between SITL and ROS 2 |
| [ros_gz](https://github.com/gazebosim/ros_gz) | Gazebo and ROS 2 bridge |
| [sdformat_urdf](https://github.com/ros/sdformat_urdf) | SDFormat to URDF for robot descriptions |

The forks track upstream with the changes on their `collider` branch.

## Tests

Unit and node-level tests run without the simulator, PyTorch or network access; ArduPilot
services, the YOLO model and the LLM client are replaced with fakes.

```bash
python3 -m pytest src/collider/test -q
```

Real-model tests and the YOLO benchmark are described in
[docs/development.md](docs/development.md).

## Limitations and roadmap

- The default target is a dark sphere found by a blob tracker; general-purpose trackers
  lose it when the camera rotates quickly.
- Steering uses fixed gains with short-horizon prediction and does not model the
  airframe's inertia; the stock Iris is slow to climb and lean.
- Running Gazebo faster than real time is not supported yet.

Planned: multi-object tracking for detections (ByteTrack), following ground vehicles,
more detection classes, and inertia-aware steering.

## License

AGPL-3.0, see [LICENSE](LICENSE). The YOLO weights are derived from Ultralytics (AGPL-3.0)
and fine-tuned on VisDrone, which is licensed for non-commercial research only. The test
image `test/drone_vehicles.jpg` is a VisDrone frame under CC BY-NC-SA 3.0, see
[its source note](test/drone_vehicles.SOURCE.md).
