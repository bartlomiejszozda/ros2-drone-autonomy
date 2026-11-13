# Development

Commands assume the dev container described in [docker/README.md](../docker/README.md):
workspace root `/ws`, this repository at `/ws/src/collider`, and ROS 2 Humble plus the
workspace sourced by every shell. On another setup, adjust the paths and source
`/opt/ros/humble/setup.bash` and `install/setup.bash` yourself.

## Building

First build of the whole workspace (ArduPilot, the Gazebo bridge and this repository):

```bash
cd /ws
colcon build --symlink-install --base-paths src src/collider/interfaces/collider_interfaces
source install/setup.bash
```

`collider_interfaces` is nested inside this repository. colcon stops descending once it
finds a package, so a plain `colcon build` never discovers it; its path has to be listed in
`--base-paths`.

Rebuilding only this repository:

```bash
colcon build --symlink-install \
  --base-paths src/collider src/collider/interfaces/collider_interfaces \
  --packages-select collider_interfaces collider
source install/setup.bash
```

With `--symlink-install`, edits to Python files, the launch file and `config/collider.yaml`
apply on the next start. Rebuild after changing `.msg` or `.action` files or `setup.py`,
and after adding a new launch or config file.

## Running

Start the simulator in its own terminal, so Collider can be restarted without restarting
Gazebo:

```bash
ros2 launch ardupilot_gz_bringup iris_runway.launch.py
ros2 launch collider collider.launch.py                 # second terminal
```

Or both at once, headless:

```bash
ros2 launch collider collider.launch.py with_sim:=true use_gz_sim_gui:=False rviz:=False
```

Launch arguments are listed in [nodes.md](nodes.md#launch-file). Common variants:
`backend:=rc`, `steering:=false`, `assistant:=false`, `car_detector:=false`. Without a
simulator there is no `/clock`; use `use_sim_time:=False` or the node timers never fire.
Do not start a node with `ros2 run` while the launch file already runs it: duplicate node
names break parameter and service calls.

Camera streams in the browser (forward port 8765 when working remotely):

```bash
ros2 run web_video_server web_video_server --ros-args -p port:=8765
```

- `http://localhost:8765/stream?topic=/iris/static_camera_sensor/image`
- `http://localhost:8765/stream?topic=/collider/tracker/image`
- `http://localhost:8765/stream?topic=/collider/cars/image`

### Flying

A whole flight with one goal:

```bash
ros2 action send_goal /mission/run collider_interfaces/action/Mission \
  '{altitude_m: 5.0, duration_sec: 30.0, end_action: land}' --feedback

# duration_sec 0 steers until cancelled; Ctrl+C on send_goal is not a reliable cancel:
ros2 service call /mission/run/_action/cancel_goal action_msgs/srv/CancelGoal '{}'
```

Step by step:

```bash
ros2 action send_goal /starter/takeoff collider_interfaces/action/Takeoff \
  '{altitude_m: 5.0, timeout_sec: 60.0}' --feedback
ros2 service call /steering/enable std_srvs/srv/SetBool '{data: true}'
ros2 service call /steering/enable std_srvs/srv/SetBool '{data: false}'
```

`examples/prepare_drone_client.py` sends a takeoff goal and can cancel it, which exercises
the Starter's cleanup:

```bash
python3 src/collider/examples/prepare_drone_client.py --altitude 5 --timeout 60 --cancel-after 4
```

With `backend:=rc`, switch to LOITER (`mode: 5`) before enabling steering and back to GUIDED
(`mode: 4`) before disabling it. The mission does this itself.

Safety supervisor:

```bash
ros2 topic echo /safety/state
ros2 service call /safety/brake std_srvs/srv/Trigger '{}'   # also land, rtl
ros2 service call /safety/reset std_srvs/srv/Trigger '{}'
```

### Assistant

```bash
export OPENROUTER_API_KEY=...                  # in the shell that starts the nodes
export PATH="$PATH:/ws/src/collider/scripts"
ask how many cars are there, and which colours
ask_new where is the red car, give its centre in pixels
tail -n 3 ~/.ros/collider/ai_history.jsonl | jq .
```

`ask` and `ask_new` wrap `ros2 action send_goal /collider/ask`. The shell still parses the
question: write `'` as `\'`, and `*` may expand to file names. To try another model, send
the goal directly:

```bash
ros2 action send_goal /collider/ask collider_interfaces/action/Ask \
  "{question: 'What do you see?', model: <openrouter-model-id>}" --feedback
```

Models with Zero Data Retention providers are listed at
<https://openrouter.ai/api/v1/endpoints/zdr>.

## Inspecting

```bash
ros2 node list
ros2 param get /tracker focal_px
ros2 topic echo /collider/target_track --field bearing_deg
ros2 topic echo /ap/v1/pose/filtered --field pose.position.z   # height above home
ros2 topic hz /collider/cars                                   # detector rate
ros2 topic info -v /collider/tracker/image                     # QoS and subscribers
ros2 interface show collider_interfaces/msg/TargetTrack
```

When piping `ros2 topic echo` through `timeout` into `grep`, set `PYTHONUNBUFFERED=1`;
otherwise buffered output is lost when `timeout` stops the process.

Recording a flight and replaying the inputs of `target_motion_node` allows tuning `alpha`
and `beta` without flying again. Stop the simulator before replaying.

```bash
ros2 bag record -o flight1 /collider/target /collider/target_track \
  /ap/v1/pose/filtered /ap/v1/status /ap/v1/cmd_vel
ros2 bag play flight1
```

### ArduPilot

Copter mode numbers: GUIDED 4, LOITER 5, RTL 6, LAND 9, BRAKE 17.

```bash
ros2 service call /ap/v1/mode_switch ardupilot_msgs/srv/ModeSwitch '{mode: 9}'
ros2 service call /ap/v1/arm_motors ardupilot_msgs/srv/ArmMotors '{arm: false}'
ros2 service call /ap/v1/get_parameters rcl_interfaces/srv/GetParameters \
  "{names: ['GUID_TIMEOUT']}"
ros2 service call /ap/v1/set_parameters rcl_interfaces/srv/SetParameters \
  "{parameters: [{name: 'GUID_TIMEOUT', value: {type: 3, double_value: 3.0}}]}"
```

A returned parameter with `type: 0` does not exist.

Manual velocity commands work only when armed in GUIDED. ArduPilot keeps executing the last
command for `GUID_TIMEOUT` (3 s) after publishing stops. In `base_link`, x is forward, y left,
z up and positive `angular.z` turns left; in `map`, x is east and y north.

```bash
ros2 topic pub -r 10 /ap/v1/cmd_vel geometry_msgs/msg/TwistStamped \
  '{header: {frame_id: base_link}, twist: {angular: {z: 0.15}}}'
ros2 topic pub --once /ap/v1/cmd_vel geometry_msgs/msg/TwistStamped \
  '{header: {frame_id: base_link}}'                           # stop now
```

### Gazebo

The world is `runway` and the target model is `sphere`. `scripts/launch_target [world]`
holds the sphere against gravity with a persistent wrench, then pushes it up for 3 s.

```bash
src/collider/scripts/launch_target
gz model -m sphere -p        # sphere pose

# Pause and resume
gz service -s /world/runway/control --reqtype gz.msgs.WorldControl \
  --reptype gz.msgs.Boolean --timeout 1000 --req 'pause: true'
gz service -s /world/runway/control --reqtype gz.msgs.WorldControl \
  --reptype gz.msgs.Boolean --timeout 1000 --req 'pause: false'
```

If Gazebo shows no image, a stale `gz sim` process is usually still running; find it with
`ps aux | grep gz` and kill it.

## Tests

```bash
cd /ws
ROS_DOMAIN_ID=87 ROS_LOCALHOST_ONLY=1 python3 -m pytest src/collider/test -q
ROS_DOMAIN_ID=87 ROS_LOCALHOST_ONLY=1 python3 -m pytest src/collider/test/test_mission.py -q
```

The tests need neither the simulator nor network access. Node tests start real ROS nodes,
action clients and services in-process and replace everything external with fakes: fake
ArduPilot mode, arm, takeoff and status endpoints, a fake Starter and steering service for
the mission, a fake `/clock`, synthetic camera images, `FakeModel` and a fake HTTP `post`
for the assistant, and `FakeYolo`/`FakeDetector` for the car detector, so torch is not
required. The separate domain id keeps test traffic away from a running simulation.

`test/conftest.py` points Fast DDS at `test/fastdds_no_intraprocess.xml`, which disables
intra-process delivery. Fast DDS 2.6 can crash when a subscriber in the same process is
destroyed during delivery, which happens when many nodes are created and destroyed in one
test process. Normal runs, with one node per process, are not affected.

`test_black_spot_finder.py` writes an annotated frame to `test/tmp/` (ignored by git). The
`test/` directory also contains the ament linters (`test_flake8.py`, `test_pep257.py`,
`test_copyright.py`).

### Real YOLO model

The integration test is skipped unless enabled. It loads the real weights, runs them on
`test/drone_vehicles.jpg` (a VisDrone frame, see `test/drone_vehicles.SOURCE.md`), checks that
a vehicle is found, and prints timings with `-s`:

```bash
RUN_YOLO_INTEGRATION=1 YOLO_DEVICE=cpu \
  python3 -m pytest src/collider/test/test_car_detector_real.py -q -s
```

`YOLO_MODEL` overrides the weights path (default `/opt/models/yolo11n-visdrone.pt`) and
`YOLO_IMGSZ` the inference size (default 1280).

`benchmark_yolo` measures model load time, the first (cold) inference, and the median and
p95 of warmed end-to-end `YoloCarDetector.detect` calls, including result conversion:

```bash
ros2 run collider benchmark_yolo --device cpu --imgsz 640 --iterations 10
ros2 run collider benchmark_yolo --device cuda:0 --imgsz 1280 --iterations 50
```

Without `--image` it downloads a public VisDrone frame; pass a local file, for example
`--image src/collider/test/drone_vehicles.jpg`, to avoid that. Other options: `--model`,
`--warmup` (default 2).

## Known limitations

- Faster than real time does not fly. Above a real-time factor of 1 the copter keeps
  climbing at about 1.5 m/s with idle motors, even in LAND. At 1x it behaves normally.
  Collider's nodes, `SIM_SPEEDUP`, changing speed in flight, enabling SITL time sync,
  software rendering and leftover Gazebo wrenches have been ruled out as causes. Gazebo sets
  the pace and SITL follows in lockstep, so `SIM_SPEEDUP` has no effect here. ArduPilot saves it; keep it at 1. Any speed-up also needs GPU
  rendering, since software rendering tops out near 1.2x.
- Tracking is limited to a near-black target. `BlackSpotTracker` thresholds for near-black
  pixels and takes the blob closest to its previous position, so any other dark object in the
  search region can capture it. The OpenCV trackers lose the target when the camera rotates
  and need interactive ROI selection, which is why they are not used.
- Steering ignores vehicle dynamics. The controller is proportional with rate feed-forward
  and no model of inertia. The stock iris is slow to lean, accelerate and climb, so it cannot
  keep up with fast or abruptly moving targets.
- A lost target does not stop the vehicle. Steering keeps the fixed forward cruise with zero
  yaw and climb until the track returns, steering is disabled, or safety intervenes.
- The RC backend's pitch and roll mapping is approximate, because LOITER turns stick input
  into a lean angle and acceleration rather than a velocity. Ending an RC session requires
  switching to GUIDED or BRAKE before disabling steering.
- While simulation time is stalled, safety cannot detect telemetry loss and only logs a
  warning. ArduPilot's own failsafes still apply.
- The Starter supports Copter SITL taking off from the ground near home only. After a
  cleanup timeout it rejects all goals until restarted, and the vehicle must be recovered by
  hand.
- `car_detector` does not track vehicles across frames (`track_id` is -1), and no node uses
  `/collider/cars` yet. YOLO and Gazebo share GPU memory; check `nvidia-smi` on small GPUs.
- Vision language models return only approximate pixel coordinates.
