# Navigation work: inventory, deviations and open decisions

This is the inventory the navigation brief asks for at the top of the PR description,
plus every place the implementation differs from the brief and why. Branch
`sensor_fusion`, 14 Oct 2026. Commits `4e2e72d` (phase 1) to `c45052d`.

## 1. Inventory (read before writing)

### Packages

| Package | Build type | Role |
| --- | --- | --- |
| `ballrobot_pkg` | ament_cmake (Python nodes installed as programs, custom msgs) | Sensor nodes, drive interface, teleop, bringup |
| `ballrobot_localization_pkg` | ament_python | UWB, IMU and wheel adapters, the EKF, evaluation tools |
| `robot_navigation_pkg` (new) | ament_python | Alignment, go to goal, routes, drive calibration |

**ROS 2 distro:** not determinable here. The development Mac has no ROS 2. Check
`$ROS_DISTRO` on the Pi. The launch files use the named-argument form of
`static_transform_publisher`, which needs Humble or newer.

### Nodes (as found, before this work)

| Node | Publishes | Subscribes | Types | frame_id |
| --- | --- | --- | --- | --- |
| `uwb_node` | `/uwb/position` (5 to 7 Hz) | | `geometry_msgs/PointStamped` | `map` |
| `imu_node` | `/imu/rpy`, `/imu/accel` (100 Hz) | | `geometry_msgs/Vector3Stamped` (deg; raw mg counts) | `imu_link` |
| `imu_quaternion_node` (opt-in) | `/imu/data_quat` | `/imu/rpy` | `sensor_msgs/Imu` | `imu_link` |
| `teensy_interface_node` | `/wheel_states` (70 to 100 Hz) | `wheel_speed_cmd` | `sensor_msgs/JointState`; `ballrobot_pkg/LeftRightFloat32` | empty |
| `cmd_vel_bridge` | `wheel_speed_cmd` | `/cmd_vel` | `LeftRightFloat32`; `geometry_msgs/Twist` | |
| `wasd_teleop` | `/cmd_vel` (one Twist per key press) | | `geometry_msgs/Twist` | |
| `heading_ble_node` | BLE to the website (heading display) | `/imu/rpy` | | |
| `solenoid_release_node`, `sound_player_node` | `solenoid_released`; | `play_sound`, `sound_volume`, ... | `std_msgs` | |

### The serial interface node (`teensy_interface_node.py`)

* Owns the drive XIAO's USB serial port (115200, found by USB name in bringup).
* Parser: reads non-blocking at 50 Hz, splits on `\n`, keeps partial lines, and parses
  `MEAS,<l_rad_s>,<r_rad_s>[,<l_rad>,<r_rad>]` into `/wheel_states`. Other lines were
  only logged.
* **REF lines:** it wrote `REF,<l_rad_s>,<r_rad_s>` for every `wheel_speed_cmd` from
  `cmd_vel_bridge`. That was the Teensy-era wheel speed reference. Firmware v4.6
  discards everything the Pi sends (`while (Serial.available()) Serial.read();`), so
  REF never had an effect on the XIAO. It is no longer sent.

### Launch files

`ballrobot_pkg/launch/bringup.launch.py` (all sensor nodes, the interface node,
`cmd_vel_bridge`, BLE and sound nodes, optional bag recording, shared setup file
`config:=<name>`); `ballrobot_localization_pkg` `localization.launch.py` and
`offline.launch.py`.

### Website

In the repo: `GUI/controller_v4_6.html` with `GUI/controller_core_v4.mjs`. Findings that
matter for v4.7:

* It enables Arm only after an exact `VERSION,4.6` reply. `VERSION,4.7` would show
  "Flash the v4.6 XIAO sketch" and keep Arm disabled.
* It sends `K` immediately on every connect.
* Unknown lines (`OWNER,...`, `STATE,BUSY,...`, `STATE,NOT_OWNER`) are only logged:
  safe.
* Stop is enabled whenever connected and sends `K`, even if the website did not arm.

## 2. Deviations from the brief, and why

| Brief | Implemented | Why |
| --- | --- | --- |
| 1.4: `V` replies `VERSION,4.7` | `VERSION,4.6` over BLE, `VERSION,4.7` over serial | Team decision: firmware only, the website stays unchanged and would otherwise refuse to arm. |
| 1.4: BLE connect during a Pi run does not disarm | Firmware does not disarm, but the **website sends K on connect**, so connecting the website stops a Pi run | Team decision: firmware only. Effectively v4.6 behaviour for decision 8.1 (below). |
| 1.4: `R` reply to the requester | `STATE,FAULT_RESET` to both sources | `FAULT` goes to both, so its reset does too; the Pi learns that a keeper cleared the fault. |
| 1.4: serial `A` during a fault | v4.6 ignores silently; serial now gets `FAULT,ENCODER_STALL` back | So `~/arm` can report "fault" instead of a timeout. BLE unchanged. |
| 2: `config/drive.yaml` | Drive parameters in the shared setup file, `teensy_interface_node` block (`ballrobot_pkg/config/<name>.yaml`) | bringup passes one parameter file to the interface node; the same block also feeds the wheel odometry, so `r_eff`/`b_eff` exist once. |
| 2: `/drive/*` topics | Plus `/drive/command` (each M/P/S line sent) | `calibrate.py` needs commanded percent paired with `MEAS`. |
| 3: new wheel odometry node, `/wheel/odom` | Existing `wheel_odom_adapter` extended (50 Hz averaging, brief covariances), topic `/localization/wheel_odom` | The brief says skip if one exists. |
| 4: "IMU gyro z only" | Yaw rate derived from the fused RVC yaw, as d(yaw)/dt through the static mount | UART-RVC has no gyro output. d(yaw)/dt is the true heading rate while the chassis pitches inside the shell (up to 39 deg measured; the body z rate would be up to 29 % off). |
| 4: separate `ekf.yaml` in the navigation package | The single EKF lives in `ballrobot_localization_pkg/config/ekf.yaml` | Team decision: one EKF for every use case. That package already had the adapters; its earlier dual EKF was replaced. |
| 4: UWB variance 0.3^2 | 0.05 m^2 (about 0.22 m) | Measured sample noise in the September logs is about 0.1 m; raised for outdoor ranges. TODO calibrate. |
| 5: `/goal_pose` | Also refused until a geofence is configured (`require_geofence`) | The alignment drive heads in an unknown direction, and no fence coordinates exist yet. |
| 5: geofence look ahead | Skipped during ALIGN; instead ALIGN needs `d_align + 0.5 m` clearance from every edge | The EKF heading is unknown until alignment, so a heading-based look ahead points the wrong way. |
| 6: package `tigerball_navigation` | `robot_navigation_pkg` | Team naming. |
| 6: `navigation.launch.py` | As specified, plus `sensors:=false` to skip bringup | Lets navigation run with sensors started separately. |

Not changed: the UWB and IMU nodes and `uwb_trilateration.py`.
`TigerBall_nRF52840_Sense_v4_6.ino` is untouched; v4.7 is in its own sketch folder
(the Arduino IDE compiles every `.ino` in a folder together).

## 3. Decisions to raise with the team (brief section 8)

1. **BLE connect during a Pi-owned run.** The firmware is written not to disarm, but
   the unchanged website sends `K` on connect, so in practice connecting the website
   stops the robot. To let the website watch an autonomous run, the website must skip
   its initial `K` when it receives `OWNER,SERIAL`. A website disconnect during a Pi
   run does not stop it.
2. **Keeper stop.** Confirmed from the source: the website's Stop button is enabled
   whenever it is connected and sends `K`, which stops a Pi run. The keeper must be
   connected to use it (see 1).
3. **Alignment direction.** Unknown by design. Start the robot facing open ground;
   the controller refuses to align closer than 2.0 m to the fence.
4. **Dwell while armed.** During a route the robot stays armed and stationary between
   waypoints (up to `dwell_s`, default 10 to 40 s). The alternative is to disarm and
   re-arm at each point.

## 4. Risks

* Nothing has run on hardware or under ROS 2 yet. Firmware compiles for the XIAO
  nRF52840 Sense; the logic is verified in host simulations only (section 8 of the
  README).
* Sign conventions are unverified: `M,+x,+x` must drive forward, and IMU yaw must
  increase counter clockwise (`yaw_sign`). The only real drive in the old bags shows a
  weak negative correlation between IMU and wheel yaw rate. `calibrate.py` flags a
  negative `b_eff` if the signs disagree.
* `wasd_teleop` publishes one Twist per key press; the interface node sends `S` after
  0.3 s without `cmd_vel`, so with firmware v4.7 it only moves while keys repeat. Use
  `teleop_twist_keyboard` (repeats at 10 Hz) for phase 2 testing.
* `Serial` going false when the Pi closes the port (host loss disarm) must be checked
  on the bench (F9).
