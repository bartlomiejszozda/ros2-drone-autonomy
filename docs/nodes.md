# Node reference

All nodes are in the `collider` package. Defaults below are the values declared in code.
`config/collider.yaml` sets most of them explicitly; where the file uses a different value,
the table says so. For the overall data flow see [architecture.md](architecture.md).

Topic, service and action names that are parameters can be remapped through the parameter
file. The names of the servers each node provides (`/starter/takeoff`, `/steering/enable`,
`/safety/*`, `/mission/run`, `/collider/ask`) are fixed in code.

## Launch file

```bash
ros2 launch collider collider.launch.py [argument:=value ...]
```

Starts every node below as a separate process, each with the parameter file and
`use_sim_time`. `tracker` and `target_motion` always start; the others can be switched off.

| Argument | Default | Effect |
| --- | --- | --- |
| `params_file` | installed `config/collider.yaml` | Parameter file for all nodes |
| `backend` | `cmd_vel` | Steering backend, `cmd_vel` or `rc`; overrides the YAML value |
| `use_sim_time` | `True` | Follow Gazebo's `/clock`; use `False` only without a simulator |
| `with_sim` | `false` | Also include `ardupilot_gz_bringup/iris_runway.launch.py` |
| `starter` | `true` | Start `starter` |
| `steering` | `true` | Start `steering` |
| `safety` | `true` | Start `safety` |
| `mission` | `true` | Start `mission`; its `steering_mode` is set to 5 for `rc`, 4 otherwise |
| `assistant` | `true` | Start `assistant` (needs `OPENROUTER_API_KEY` to answer) |
| `car_detector` | `true` | Start `car_detector` (needs `ultralytics` and the model file) |

`use_sim_time` must be written `True` or `False`: the included simulator launch accepts only
those spellings. `--show-args` also lists the simulator's own arguments (`rviz`,
`use_gz_sim_gui`, ...), which can be passed through when `with_sim:=true`.

Write floating-point values in the YAML with a decimal point (`10.0`). A parameter declared
with a float default rejects an integer at startup.

## tracker

Executable `tracker_node`. Finds the target in each camera frame and publishes one
observation per frame. Sends nothing to the vehicle.

| Kind | Name | Type |
| --- | --- | --- |
| Subscribes | `image_topic` | `sensor_msgs/Image` (best effort, keep last 1) |
| Publishes | `observation_topic` | `collider_interfaces/TargetObservation` |
| Publishes | `debug_image_topic` | `sensor_msgs/Image` (frame with the box drawn) |

| Parameter | Default | Description |
| --- | --- | --- |
| `image_topic` | `/iris/static_camera_sensor/image` | Camera input |
| `observation_topic` | `/collider/target` | Output |
| `debug_image_topic` | `/collider/tracker/image` | Annotated image |
| `focal_px` | `205.47` | Pinhole focal length in pixels; must be positive |

Behaviour:

- Uses `BlackSpotTracker`, which tracks the darkest blob near its previous position.
- When the target is not found, it publishes `tracking: false` (other fields zero) and resets
  the tracker, so the next frame is searched in full.
- Logs only changes (`Target acquired ...`, `Target lost ...`), not every frame.
- Accepts `rgb8` and `bgr8`; other encodings are skipped with a throttled warning.
- The debug image is drawn and published only while it has subscribers.

## target_motion

Executable `target_motion_node`. Converts observations into a world bearing and elevation
using the attitude at the image timestamp, and estimates their rates.

| Kind | Name | Type |
| --- | --- | --- |
| Subscribes | `observation_topic` | `collider_interfaces/TargetObservation` |
| Subscribes | `pose_topic` | `geometry_msgs/PoseStamped` (sensor data QoS) |
| Publishes | `track_topic` | `collider_interfaces/TargetTrack` |

| Parameter | Default | Description |
| --- | --- | --- |
| `observation_topic` | `/collider/target` | Tracker output |
| `pose_topic` | `/ap/v1/pose/filtered` | Attitude source |
| `track_topic` | `/collider/target_track` | Output |
| `alpha` | `0.5` | Filter gain for the value, in (0, 1] |
| `beta` | `0.1` | Filter gain for the rate, in [0, 1] |
| `max_coast_sec` | `0.5` | How long a lost target is predicted before the track is dropped |
| `max_pose_gap_sec` | `0.1` | Maximum distance from the image time to a usable pose |

Behaviour:

- Keeps the last 200 poses and interpolates the attitude at the image stamp. Without a pose
  within `max_pose_gap_sec` the image is treated as a missed detection.
- A missed detection coasts the track along its last rates (`measured: false`, `age_sec`
  growing). After `max_coast_sec` it publishes `valid: false`.
- The next detection starts a new track with `measurements: 1` and zero rates. Rates need
  several measurements to settle.
- If time goes backwards (simulator restart), the pose history and the track are discarded.

Higher `alpha`/`beta` react faster and pass more noise; lower values are smoother and lag.

## starter

Executable `starter_node`. Serves the `Takeoff` action and owns arming, takeoff and the
cleanup after a cancelled or failed takeoff. Scoped to Copter SITL taking off from the
ground near home.

| Kind | Name | Type |
| --- | --- | --- |
| Action server | `/starter/takeoff` | `collider_interfaces/action/Takeoff` |
| Client | `mode_service` | `ardupilot_msgs/srv/ModeSwitch` |
| Client | `prearm_service` | `std_srvs/srv/Trigger` |
| Client | `arm_service` | `ardupilot_msgs/srv/ArmMotors` |
| Client | `takeoff_service` | `ardupilot_msgs/srv/Takeoff` |
| Subscribes | `status_topic`, `pose_topic` | `ardupilot_msgs/Status`, `geometry_msgs/PoseStamped` |
| Subscribes | `safety_state_topic` | `std_msgs/String` (transient local) |

| Parameter | Default | Description |
| --- | --- | --- |
| `mode_service` | `/ap/v1/mode_switch` | |
| `prearm_service` | `/ap/v1/prearm_check` | |
| `arm_service` | `/ap/v1/arm_motors` | |
| `takeoff_service` | `/ap/v1/experimental/takeoff` | |
| `status_topic` | `/ap/v1/status` | |
| `pose_topic` | `/ap/v1/pose/filtered` | `pose.position.z` is height above home |
| `safety_state_topic` | `/safety/state` | Takeoff is refused while this is not `OK` |
| `timeout_sec` | `10.0` | Service discovery and response limit; also the limit for armed confirmation |
| `retry_interval_sec` | `1.0` | Delay between mode/prearm retries and cleanup retries |
| `telemetry_timeout_sec` | `2.0` | Maximum age of status and pose |
| `cleanup_timeout_sec` | `60.0` | Time allowed for cleanup |
| `ground_height_m` | `0.3` | Height around home that counts as on the ground |
| `altitude_tolerance_m` | `0.2` | Allowed shortfall below the requested altitude |
| `altitude_hold_sec` | `0.5` | Time the altitude must hold before success |
| `max_altitude_m` | `50.0` | Largest accepted goal altitude |

All numeric parameters must be finite and positive. Service names differ between ArduPilot
versions; check `ros2 service list -t` and override them if the simulator uses unversioned
names (`/ap/mode_switch`, ...).

Goal: `altitude_m` above home and `timeout_sec` for the whole sequence (0 means no limit).
Goals are rejected if the altitude is not above `ground_height_m + altitude_tolerance_m` or
exceeds `max_altitude_m`, if the timeout is negative, while another goal runs, or after an
unresolved cleanup failure. While safety is tripped a goal is accepted and aborted at once
with the reason; nothing is sent to the vehicle.

Sequence, reported in feedback as `stage`:

```text
WAIT_TELEMETRY  MODE_CHANGE  PREARM  ARMING  TAKEOFF  CLIMBING
```

- `WAIT_TELEMETRY` requires fresh telemetry from a Copter that is disarmed and on the ground.
- `MODE_CHANGE` and `PREARM` retry until GUIDED is confirmed and the prearm check passes.
- `ARMING` must see armed in a status message received after the arm reply.
- `CLIMBING` succeeds once the vehicle is armed, flying and in GUIDED, with altitude at least
  `altitude_m - altitude_tolerance_m` for `altitude_hold_sec`. The drone then stays airborne
  in GUIDED.

Stale telemetry, an ArduPilot failsafe, a rejected arm or takeoff command, or the vehicle
leaving armed GUIDED during the climb all start cleanup. So do cancellation and the goal
timeout.

Cleanup depends on what has been sent:

| Situation | Action |
| --- | --- |
| No arm command sent yet | Finish immediately; no mode rollback |
| Arm or takeoff request still pending | Wait for its reply before deciding |
| Armed on the ground, no takeoff sent | Disarm, confirm from telemetry |
| Takeoff sent, airborne, or telemetry lost after arming | Request LAND, wait for ground, disarm if still armed |

Results: `SUCCEEDED` with `success: true` on reaching altitude, `CANCELED` after a cancel
request, `ABORTED` otherwise. `cleanup_completed` reports whether ground and disarmed state
were confirmed. If cleanup exceeds `cleanup_timeout_sec`, the node reports an unresolved
vehicle state and rejects all further goals; restarting the node does not land or disarm
the drone.

Cleanup needs the node to keep running; killing it mid-goal leaves the vehicle as it is.
Cancel through the action's cancel request. Closing a CLI client is not a reliable cancel.

## steering

Executable `steering_node`. Keeps the target centred by commanding yaw rate and climb rate,
while flying forward at a fixed cruise speed. Starts disabled.

| Kind | Name | Type |
| --- | --- | --- |
| Service | `/steering/enable` | `std_srvs/srv/SetBool` |
| Subscribes | `track_topic` | `collider_interfaces/TargetTrack` |
| Subscribes | `pose_topic`, `status_topic` | sensor data QoS |
| Publishes | `cmd_vel_topic` | `geometry_msgs/TwistStamped`, frame `map` (`cmd_vel` backend) |
| MAVLink | `mavlink_url` | `RC_CHANNELS_OVERRIDE` (`rc` backend) |

| Parameter | Default | Description |
| --- | --- | --- |
| `backend` | `cmd_vel` | `cmd_vel` (needs GUIDED) or `rc` (needs LOITER) |
| `track_topic` | `/collider/target_track` | |
| `pose_topic` | `/ap/v1/pose/filtered` | |
| `status_topic` | `/ap/v1/status` | |
| `cmd_vel_topic` | `/ap/v1/cmd_vel` | |
| `mavlink_url` | `udp:localhost:14550` | `rc` only |
| `rate_hz` | `10.0` | Command rate |
| `telemetry_timeout_sec` | `1.0` | Maximum age of status and pose; `collider.yaml` uses `3.0` because status arrives only every ~0.75 s |
| `track_timeout_sec` | `0.5` | Older tracks count as missing |
| `yaw_gain` | `1.0` | deg/s of yaw rate per degree of bearing error |
| `yaw_rate_feedforward` | `1.0` | Share of the target's bearing rate added to the yaw rate |
| `max_yaw_rate_dps` | `45.0` | |
| `climb_gain` | `0.1` | m/s of climb per degree of target elevation |
| `max_climb_mps` | `1.0` | |
| `deadband_deg` | `1.0` | Smaller bearing and elevation errors are ignored |
| `max_prediction_sec` | `0.3` | Limit on extrapolating the track to the current time |

Control law, evaluated every tick with the track predicted to the latest pose time:

```text
bearing_error = wrap_180(predicted_bearing - heading)
yaw_rate      = yaw_gain * bearing_error + yaw_rate_feedforward * bearing_rate
up            = climb_gain * predicted_elevation
```

Forward speed is not a parameter. Every command carries the default
`DesiredMotion.forward_mps` (18 m/s, set in `steering/Centering.py`); the controller only
changes yaw rate and climb. The `rc` backend saturates the pitch stick at that speed.

On every tick while enabled:

- Status and pose must be fresh, the vehicle armed and flying, and in the backend's mode.
  Otherwise the node sends one stop command and waits, logging the reason.
- If the track is missing, invalid or older than `track_timeout_sec`, the node keeps
  commanding with zero yaw and climb rate. The forward cruise continues; the vehicle does not
  stop when the target is lost.
- Disabling, and shutting down, sends one stop command: zero velocity for `cmd_vel`, centred
  sticks for `rc`.

With `backend:=rc` the node waits up to 10 s for a MAVLink heartbeat at startup and fails if
none arrives. Do not run another RC override source at the same time. To end an RC session,
switch to GUIDED or BRAKE first, then disable steering; releasing the sticks in LOITER can
make the vehicle descend in SITL.

## safety

Executable `safety_node`. Supervises the vehicle and takes control away from Collider when a
rule is broken or a manual button is pressed.

| Kind | Name | Type |
| --- | --- | --- |
| Publishes | `/safety/state` | `std_msgs/String`, transient local |
| Service | `/safety/brake`, `/safety/land`, `/safety/rtl` | `std_srvs/srv/Trigger` |
| Service | `/safety/reset` | `std_srvs/srv/Trigger` |
| Subscribes | `status_topic`, `pose_topic` | sensor data QoS |
| Client | `mode_service` | `ardupilot_msgs/srv/ModeSwitch` |
| Client | `steering_enable_service` | `std_srvs/srv/SetBool` |
| Client | `<takeoff_action>/_action/cancel_goal` | `action_msgs/srv/CancelGoal` |

| Parameter | Default | Description |
| --- | --- | --- |
| `telemetry_timeout_sec` | `1.0` | Age at which status or pose counts as lost; `collider.yaml` uses `3.0` |
| `violation_action` | `brake` | Final mode after BRAKE: `brake`, `land` or `rtl` |
| `brake_hold_sec` | `2.0` | Time in BRAKE before switching to the final mode |
| `retry_interval_sec` | `1.0` | Between mode-switch attempts |
| `clock_stall_warning_sec` | `3.0` | Warn when simulation time stops this long while armed |
| `status_topic` | `/ap/v1/status` | |
| `pose_topic` | `/ap/v1/pose/filtered` | |
| `mode_service` | `/ap/v1/mode_switch` | |
| `steering_enable_service` | `/steering/enable` | |
| `takeoff_action` | `/starter/takeoff` | |

Rules, checked at 10 Hz in this order:

| Violation | Applies when | Reaction |
| --- | --- | --- |
| Status telemetry lost | Last fresh status said armed | BRAKE, then `violation_action` |
| ArduPilot failsafe active (`Status.failsafe` not empty) | Armed | Mode left to ArduPilot |
| Pose telemetry lost | Armed and flying | BRAKE, then `violation_action` |

A disarmed vehicle is never commanded. On every trip the node also disables steering and
cancels all takeoff goals, skipping either if its server is not running.

State strings:

```text
OK
TRIPPED (BRAKE): pose telemetry lost
TRIPPED (LAND): manual LAND
TRIPPED (ArduPilot failsafe): ArduPilot failsafe active [22]
```

Behaviour:

- Manual buttons trip the supervisor with that mode as the final mode (BRAKE first for
  `brake_hold_sec`). Pressed while already tripped, they only change the final mode, for
  example BRAKE now and RTL after inspecting the situation.
- If ArduPilot rejects BRAKE, the node falls back to LAND.
- The trip is latched. The node re-requests its mode if the vehicle leaves it.
- `/safety/reset` is refused while a rule is still broken. After a reset steering stays
  disabled until enabled again.

Choosing the final mode: BRAKE holds position and lets the operator decide; LAND descends in
place and works without a position estimate; RTL climbs to `RTL_ALT_M` if lower (15 m by
default), flies home and lands, and needs a home position. Check `RTL_ALT_M` before using RTL
in a small area.

## mission

Executable `mission_node`. Runs a whole flight from one goal by coordinating the Starter,
the steering node and ArduPilot modes.

| Kind | Name | Type |
| --- | --- | --- |
| Action server | `/mission/run` | `collider_interfaces/action/Mission` |
| Action client | `takeoff_action` | `collider_interfaces/action/Takeoff` |
| Client | `steering_enable_service` | `std_srvs/srv/SetBool` |
| Client | `mode_service` | `ardupilot_msgs/srv/ModeSwitch` |
| Subscribes | `status_topic` | sensor data QoS |
| Subscribes | `safety_state_topic` | transient local |

| Parameter | Default | Description |
| --- | --- | --- |
| `steering_mode` | `4` | 4 (GUIDED) for `cmd_vel`, 5 (LOITER) for `rc`; set by the launch file from `backend` |
| `require_safety` | `true` | Refuse to start unless a safety state is received |
| `service_timeout_sec` | `5.0` | Waiting for servers and replies |
| `takeoff_timeout_sec` | `60.0` | Passed as the Takeoff goal's `timeout_sec` |
| `landing_timeout_sec` | `120.0` | Waiting for landed and disarmed |
| `telemetry_timeout_sec` | `2.0` | Status older than this counts as lost |
| `takeoff_action` | `/starter/takeoff` | |
| `steering_enable_service` | `/steering/enable` | |
| `mode_service` | `/ap/v1/mode_switch` | |
| `status_topic` | `/ap/v1/status` | |
| `safety_state_topic` | `/safety/state` | |

Goal fields: `altitude_m` (> 0), `duration_sec` (0 steers until the goal is cancelled) and
`end_action` (`land`, `rtl` or `hover`; empty means `land`). One mission runs at a time.

| Stage | What happens |
| --- | --- |
| `CHECK` | Safety state is `OK`; takeoff, steering and mode servers respond |
| `TAKEOFF` | Takeoff goal to the Starter; its feedback is forwarded |
| `STEERING` | Switch to LOITER first if `steering_mode` is 5, enable steering, wait `duration_sec` |
| `FINISH` | Switch back to GUIDED first if using RC, then disable steering |
| `LAND` / `RTL` | Request the mode, wait for fresh status showing disarmed |
| `DONE` | Finished; with `hover` the vehicle stays in GUIDED |

Behaviour:

- While steering, the mission aborts if safety trips, status telemetry is lost, or the
  vehicle is no longer armed and flying.
- After a safety trip the mission never requests a flight mode again; the supervisor owns it.
  A failed takeoff is already cleaned up by the Starter. In both cases the mission only
  disables steering, if it enabled it.
- Cancelling during takeoff forwards the cancel to the Starter and waits for its cleanup.
  Cancelling during steering stops steering and performs `end_action`; the result is
  `CANCELED`.
- Mode switches are tried three times, one second apart.
- The result carries `success`, `message` (prefixed with the stage) and `last_stage`.

## assistant

Executable `assistant_node`. Answers questions about the newest camera frame with a vision
language model on OpenRouter. Sends nothing to the vehicle.

| Kind | Name | Type |
| --- | --- | --- |
| Action server | `/collider/ask` | `collider_interfaces/action/Ask` |
| Subscribes | `image_topic` | `sensor_msgs/Image` (best effort, keep last 1) |

| Parameter | Default | Description |
| --- | --- | --- |
| `model` | `qwen/qwen3.8-27b` | OpenRouter model id; a goal's `model` field overrides it |
| `system_prompt` | `''` | Sent first with every question; empty means none |
| `max_history_turns` | `1000` | Earlier question/answer pairs sent as memory |
| `history_file` | `~/.ros/collider/ai_history.jsonl` | Append-only log; empty disables it |
| `image_topic` | `/iris/static_camera_sensor/image` | |
| `max_image_age_sec` | `2.0` | Refuse to answer about an older frame |
| `request_timeout_sec` | `60.0` | HTTP timeout |

The API key comes only from the `OPENROUTER_API_KEY` environment variable of the node's
process. Without it the node starts, warns, and fails every question.

Behaviour:

- Each question is sent with the system prompt, earlier turns as text, and the newest frame
  as a JPEG data URL. `new_conversation: true` clears the memory first. Memory is lost when
  the node restarts.
- Failed and cancelled questions are not added to memory. Every question, including failed
  ones, is appended to `history_file` as one JSON line with a wall-clock timestamp.
- Feedback (`waiting for <model>`, `elapsed_sec`) is published every 0.5 s. A cancelled HTTP
  request cannot be interrupted; its answer is discarded.
- Empty questions, and questions while another is running, are rejected.
- Typical failures in `message`: `no camera image received yet`, `newest camera image is
  older than ...`, `OPENROUTER_API_KEY is not set ...`, `HTTP 404: ... ZDR violation` when
  the account allows only Zero Data Retention providers and the model has none.

Pixel coordinates in answers are approximate and models differ in convention (image pixels
or a 0 to 1000 scale). State the convention and the image size (640x480) in the question.

## car_detector

Executable `car_detector_node`. Detects all vehicles in each frame with YOLO. Sends nothing
to the vehicle.

| Kind | Name | Type |
| --- | --- | --- |
| Subscribes | `image_topic` | `sensor_msgs/Image` (best effort, keep last 1) |
| Publishes | `detections_topic` | `collider_interfaces/CarDetections` |
| Publishes | `debug_image_topic` | `sensor_msgs/Image` (boxes drawn) |

| Parameter | Default | Description |
| --- | --- | --- |
| `image_topic` | `/iris/static_camera_sensor/image` | |
| `detections_topic` | `/collider/cars` | |
| `debug_image_topic` | `/collider/cars/image` | |
| `model` | `/opt/models/yolo11n-visdrone.pt` | `.pt`, `.onnx` or `.engine` |
| `classes` | `[car, van, bus, truck]` | Class names in the model's metadata |
| `confidence` | `0.25` | Minimum score |
| `device` | `''` | Empty picks `cuda:0` when available, else `cpu` |
| `imgsz` | `1280` | Inference size; boxes stay in input pixels |
| `focal_px` | `205.47` | For the azimuth/elevation fields |

Behaviour:

- One `CarDetections` per frame, with an empty `cars` array when nothing is found.
- Each `CarDetection` has `class_name`, `confidence`, `track_id`, camera angles and a box.
  `track_id` is always -1; tracking across frames is not enabled.
- Loading the model takes several seconds and the first inference is slower than the rest.
  Frames arriving meanwhile are dropped, not queued.
- A class name the model does not know stops the node at startup with the list of unknown
  names.
- `rgb8` frames are converted to BGR before inference.
