# ballrobot_localization_pkg: Discovery

Survey of the existing sensor nodes in `ballrobot_pkg`, made before any fusion work.
Branch `imu_standard_msg` at commit `c09bb0e`. Nothing in `ballrobot_pkg`, `xiao/` or
`teensy/` was edited.

## Sources examined

| Source | What it told us |
| --- | --- |
| `ballrobot_pkg/src/nodes/uwb_node.py` | UWB publisher (topic, type, stamp, frame, parameters) |
| `ballrobot_pkg/src/nodes/uwb_position_reader.py` | Serial parsing and where the UWB timestamp is taken |
| `ballrobot_pkg/src/nodes/uwb_trilateration.py` | Closed form solve and the `clamped` flag |
| `ballrobot_pkg/src/nodes/imu_node.py` | BNO085 UART-RVC publisher |
| `ballrobot_pkg/src/nodes/imu_quaternion_node.py` | Optional debug node, Euler to `sensor_msgs/Imu` |
| `ballrobot_pkg/src/nodes/controllers/teensy_interface_node.py` | Encoder bridge, publishes `/wheel_states` |
| `xiao/encoder_bridge/TigerBall_nRF52840_Sense_v4_6.ino` | Encoder counts per revolution and wheel speed units |
| `ballrobot_pkg/launch/bringup.launch.py` | Which nodes run and which topics are recorded |
| `~/Downloads/bags/*` (19 bags) | Real rates, frame_ids, stamps and values |
| `ros2_ws/src/outputs/run2026*/uwb_log_*.csv` | The only recorded UWB data (2 runs, 24 Sep 2026) |

ROS 2 is not installed on the development Mac (`$ROS_DISTRO` is empty, no `ros2` or
`colcon`). Bags were read directly from their SQLite files with a small CDR decoder in
place of `ros2 bag info` and `ros2 topic echo --once`. The distro on the Pi still needs
to be confirmed there.

## Topic table

| topic | type | frame_id | rate | covariance populated | notes |
| --- | --- | --- | --- | --- | --- |
| `/uwb/position` | `geometry_msgs/PointStamped` | `map` (parameter `frame_id`) | 5 to 7 Hz (CSV logs); 0 messages in every bag | No (message type has no covariance) | `clamped` is not published. It is only written to the node's CSV and a throttled warning. Stamp is taken after the serial block finishes arriving, minus `latency_compensation_s` (default 0). |
| `/imu/rpy` | `geometry_msgs/Vector3Stamped` | `imu_link` | 100 Hz | No | Euler angles in degrees: x roll, y pitch, z yaw. Absolute yaw (UART-RVC mode). At rest roll reads about 179 deg, so the board is mounted upside down. |
| `/imu/accel` | `geometry_msgs/Vector3Stamped` | `imu_link` | 100 Hz | No | Raw int16 counts. At rest the mean is about (-12, -2, -1010), so the units are milli g and gravity is included. Same header as `/imu/rpy` (exact stamp match). |
| `/imu/data_quat` | `sensor_msgs/Imu` | `imu_link` (copied from `/imu/rpy`) | 100 Hz when enabled | Orientation all zero; angular velocity and accel covariance[0] = -1 | Off by default (`enable_imu_quat:=false`), not present in any bag. |
| `/wheel_states` | `sensor_msgs/JointState` | empty string | 70 to 100 Hz | No (type has none) | `velocity` = [left, right] in rad/s of the gearbox output shaft. `position` = total wheel angle in rad (empty in older bags). Stamp is `now()` when the line is parsed on the Pi. |

Stamp checks on real data: zero stamps found in no bag. Median lag from `header.stamp`
to bag receive time is 4 to 8 ms, worst case about 120 ms.

## Answers to the specific questions

1. **How UWB exposes `clamped`:** not at all on ROS. It exists inside
   `uwb_position_reader.read_block()` and is logged to `uwb_log_run<id>.csv`, but
   `uwb_node._publish_position()` publishes clamped solutions on `/uwb/position` exactly
   like good ones. See the inference note below.
2. **Encoder units:** rad/s at the wheel (gearbox output shaft), already converted on the
   XIAO. Counts per output revolution: **4480** (`COUNTS_PER_OUTPUT_REV`, Pololu #2825,
   64 CPR x 70:1, quadrature x4) in `TigerBall_nRF52840_Sense_v4_6.ino`. The Pi node does
   not hold a CPR value.
3. **ROS 2 distro:** not determinable on this machine. To be read from `$ROS_DISTRO` on
   the Pi.

## Contradictions with the task document

| # | Task document says | Workspace shows | Impact |
| --- | --- | --- | --- |
| C1 | UWB node outputs a `clamped` flag | `clamped` is not published (no field, no topic) | Blocking for 4.1 as written. A workaround without edits exists (see below). |
| C2 | IMU uses linear acceleration (gravity removed) and rotation vector reports | Node runs BNO085 in **UART-RVC** mode: Euler degrees plus accelerometer in mg **with gravity** | `imu0_remove_gravitational_acceleration: false` would be wrong. Adapter must scale mg to m/s^2 and robot_localization must remove gravity, or accel must not be fused. |
| C3 | ekf_odom fuses IMU `vyaw` | RVC frames carry **no angular velocity**. No gyro data is published anywhere | `vyaw` cannot come from the IMU. Yaw rate would come only from wheel odometry (and from differencing yaw, which is not offered by any node). |
| C4 | Pololu 37D, 100:1 | Firmware constant is Pololu #2825, 70:1, 4480 counts per output rev | Only matters if the adapter integrated counts. It does not need to: the input is already rad/s. |
| C5 | `uwb_serial_reader.py` with an `elapsed_ms` bug that stamps before a blocking read | No such file or symbol. `uwb_position_reader.read_block()` takes `time.monotonic()` **after** the block has been read | The described bug does not exist in this code. The real caveat is the opposite: the stamp is receipt time, so it lags the true ranging time by the unmeasured ranging, CLI and USB latency (`latency_compensation_s` is 0). |
| C6 | 3D anchor positions, enclosure about 40 m x 25 m, ANCHOR_POSITIONS defines map | `uwb_node` overrides `ANCHOR_POSITIONS` at runtime with its parameter default `(0,0,1.3) (1.8,0,1.3) (0,1.8,1.3)`. `uwb_trilateration.py` holds a different, unused set `(0,0,0.60) (10,0,0.63) (0,15,0.61)`. All three anchors in the live set sit at exactly z = 1.3 m | The map frame is defined by the `uwb_node` launch parameters, not by `uwb_trilateration.py`. The anchors are exactly coplanar, so z is not just poorly observed, the sign of z is resolved only by `tag_below_anchor_plane`. |
| C7 | A recorded bag supplies the raw sensor topics | 19 bags exist, **none contains a single `/uwb/position` message**. The newest 21 minute bag (`run_20261012_111340`) has no `metadata.yaml` and a corrupt SQLite file; a recovered copy still has no UWB | Steps 7 and 8 cannot produce real global results until a bag with UWB is recorded. |
| C8 | `sudo apt install ros-$ROS_DISTRO-robot-localization`; `colcon build`; offline replay | Development machine is macOS with no ROS 2 | Install, build, replay, `view_frames` and the four comparison runs must be executed on the Pi or a Linux machine. |
| C9 | `/wheel_states` provides a frame | `frame_id` is empty | Not a problem: the adapter sets `odom` and `base_link` itself. |

## `clamped` can be inferred without editing

Clamping forces `z_sq = 0`, so a clamped solution lands exactly on the anchor plane. With
the live anchors all at z = 1.3 m, checked against both CSV logs:

| Log | samples | clamped | clamped with z == 1.3 | unclamped with z == 1.3 |
| --- | --- | --- | --- | --- |
| `run20260924_191935` | 408 | 34 (8.3%) | 34 | 0 |
| `run20260924_192117` | 1596 | 209 (13.1%) | 209 | 0 |

So an adapter can flag a sample as clamped when `abs(z - anchor_plane_z) < 1e-6`. This is
exact for a horizontal anchor plane (the live configuration) and needs a general
point to plane test if the anchors are ever mounted at different heights.

## IMU data observed (UART-RVC)

* Rest accel about (-12, -2, -1010) counts. Magnitude about 1 g in milli g units, so
  scale is 9.80665e-3 m/s^2 per count and gravity is present.
* Roll about +179 deg at rest: sensor z axis points down. The `base_link` to `imu_link`
  static TF needs a 180 deg roll, not zero, once measured.
* RVC yaw is the BNO085 fused heading with the magnetometer included (confirmed by
  the team, 13 Oct 2026). It is absolute but needs `yaw_offset_deg` to align
  magnetic heading with map +x, and the EMI concern applies.
* The only real drive in the bags (8 Oct, 17:00) shows a weak negative correlation
  (r about -0.17) between IMU yaw rate and wheel yaw rate `r (w_R - w_L)`. Too weak
  to change defaults, but `yaw_sign` and the wheel signs must be checked on the
  first UWB bag (README section 8).

## Resolution (13 Oct 2026)

| Item | Decision |
| --- | --- |
| C1 clamped | Inferred in `uwb_pose_adapter` from distance to the anchor plane. No edit needed. |
| C2, C3 IMU | Use the fused roll, pitch, yaw. Derive yaw rate from orientation. Scale accel to m/s^2 and let `robot_localization` remove gravity, but leave accel fusion off by default. |
| C5 timestamp | Documented in README limitations. No change to the existing reader. |
| C6 anchors | `adapters.yaml` carries the `uwb_node` anchor set. |
| C7, C8 | Package built to run on the Pi. Comparison table pending a bag with UWB. |
