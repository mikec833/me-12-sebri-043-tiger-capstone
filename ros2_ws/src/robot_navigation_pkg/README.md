# robot_navigation_pkg

Point-to-point navigation for the Tiger Enrichment Ball Robot. The robot drives to a
goal, or round a route of surveyed safe points, using the single EKF in
`ballrobot_localization_pkg` (UWB, IMU yaw rate, wheel odometry). It drives through
the Pi's USB serial link to the XIAO (firmware v4.7). Python `rclpy` only, no Nav2,
sized for the Pi 3.

**Status (14 Oct 2026):** all code is written and tested offline (section 8). Nothing
has run on the robot or under ROS 2 yet. `INVENTORY.md` records what the repo
contained, every deviation from the navigation brief, and the decisions still open
for the team.

---

## 1. Quick start

1. **Flash firmware v4.7**: `xiao/TigerBall_nRF52840_Sense_v4_7/` (board: Seeed XIAO
   nRF52840 Sense, non-mbed core). The website keeps working unchanged. v4.6 stays in
   `xiao/encoder_bridge/` for rollback.
2. **Build** on the Pi:
   ```bash
   sudo apt install ros-$ROS_DISTRO-robot-localization ros-$ROS_DISTRO-teleop-twist-keyboard
   cd ~/ros2_ws && colcon build && source install/setup.bash
   ```
3. **Bench check** (wheels raised): run the firmware bench matrix (section 7), then
   drive with `teleop_twist_keyboard` after arming:
   ```bash
   ./scripts/run_sensors.sh config:=outdoor record:=false        # terminal 1
   ros2 service call /teensy_interface_node/arm std_srvs/srv/Trigger   # terminal 2
   ros2 run teleop_twist_keyboard teleop_twist_keyboard          # terminal 3
   ```
4. **Calibrate on the ground** inside UWB coverage (section 6) and paste the values
   into the setup file.
5. **Survey the geofence and waypoints** into `config/nav.yaml` and
   `config/waypoints.yaml`. Goals are refused until a fence exists.
6. **Navigate**:
   ```bash
   ros2 launch robot_navigation_pkg navigation.launch.py config:=outdoor
   ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
       "{header: {frame_id: map}, pose: {position: {x: 12.0, y: 8.0}}}"
   ros2 service call /go_to_goal/start_route std_srvs/srv/Trigger
   ros2 service call /go_to_goal/stop std_srvs/srv/Trigger
   ros2 topic echo /nav/status
   ```

The keeper can stop the robot at any time from the website (Stop sends `K`), or take
over by pressing Arm. Navigation then aborts and never re-arms by itself.

---

## 2. Topic graph

```
 /goal_pose, ~/start_route, ~/stop
          |
          v
   go_to_goal --/cmd_vel 20 Hz--> teensy_interface_node --USB serial--> XIAO v4.7 --> Sabertooth
      ^   |  \                      |   ^  (C, A, K, S, M, P lines)      |    ^
      |   |   +--~/arm, ~/disarm--->+   |                                |    +-- BLE <-- keeper website
      |   +--/set_pose (alignment)      +-- STATE / FAULT / OWNER / MEAS <+
      |                                 |
      |   /drive/armed, /drive/owner, /drive/status, /drive/command, /wheel_states
      |
   /odometry/filtered <-- ekf <-- /localization/{uwb_pose, imu, wheel_odom} <-- adapters <-- sensors
```

| Topic / service | Type | Publisher | Notes |
| --- | --- | --- | --- |
| `/goal_pose` | `geometry_msgs/PoseStamped` | you | frame `map` (or `odom`; identical) |
| `/cmd_vel` | `geometry_msgs/Twist` | `go_to_goal` (only while active), teleop | linear.x m/s, angular.z rad/s |
| `/nav/status` | `std_msgs/String` | `go_to_goal` | `<STATE>,<detail>`, 2 Hz and on change |
| `/set_pose` | `geometry_msgs/PoseWithCovarianceStamped` | `go_to_goal`, `align_heading` | heading from alignment, frame `odom` |
| `/drive/armed` | `std_msgs/Bool` (latched) | `teensy_interface_node` | armed **with the Pi as owner** |
| `/drive/owner` | `std_msgs/String` (latched) | `teensy_interface_node` | `NONE`, `BLE`, `SERIAL`, `UNKNOWN` |
| `/drive/status` | `std_msgs/String` | `teensy_interface_node` | every non-MEAS firmware line |
| `/drive/command` | `std_msgs/String` | `teensy_interface_node` | every M/P/S line sent |
| `/calibration/segment` | `std_msgs/String` | `calibration_drive` | `BEGIN,<kind>,<value>` / `END,<kind>` |
| `/teensy_interface_node/arm`, `/disarm`, `/reset_fault` | `std_srvs/Trigger` | `teensy_interface_node` | arm sends C then A and waits for `STATE,ARMED` |
| `/go_to_goal/start_route`, `/go_to_goal/stop` | `std_srvs/Trigger` | `go_to_goal` | |

## 3. Frames

`map -(static identity)-> odom -(ekf)-> base_link -> {imu_link, uwb_tag_link}`. The
controller works in `odom`, which equals the UWB anchor frame of the setup in use.
Waypoints and the geofence are in that frame too, so they belong to one setup file
(anchor survey).

## 4. How a run works

`go_to_goal` is a 20 Hz state machine:

```
IDLE -> ARMING -> ALIGN (first goal only) -> ROTATE <-> DRIVE -> ARRIVED -> DWELL -> next goal
                                               (via PAUSE)                         or IDLE
any state -> ABORT -> IDLE (disarmed)
```

* **ALIGN.** UWB gives position only and IMU yaw is relative, so the robot drives
  straight for `d_align` (1.5 m). A line is fitted through the UWB fixes, and the
  heading goes to the EKF on `/set_pose`. The fit is accepted if the path is longer
  than 1.0 m and the residual is below 0.15 m. It retries once, then aborts. It
  refuses to start closer than `d_align + 0.5 m` to any fence edge. It runs again
  after `align_max_age_s`.
* **ROTATE.** Pivot towards the goal (`omega = k_omega e_psi`, between `omega_min`
  and `omega_max`) until `|e_psi| < 20 deg`.
* **DRIVE.** `v = clip(k_rho rho, v_min, v_max) cos(e_psi)`, `omega = k_psi e_psi`.
  Back to ROTATE above 50 deg, ARRIVED within 0.4 m.
* **PAUSE.** Every ROTATE/DRIVE switch holds exactly zero for 0.3 s, so the firmware
  reversal gate sees a clean stop. Commands are slew limited otherwise.
* **ABORT.** The robot aborts on any of these:
  * filtered pose older than 0.5 s, or UWB older than 2 s;
  * position std above 0.5 m;
  * outside the geofence, or predicted outside it within 0.5 s;
  * no 0.2 m progress in 15 s, or the goal timeout;
  * `OWNER,BLE` (keeper takeover), an unexpected disarm, or any `FAULT`.

  The robot is disarmed and the route cancelled. Only a new goal or `start_route`
  arms it again.

The interface node turns `/cmd_vel` into firmware lines (`drive_command.py`):

* `v_L,R = v -/+ omega b_eff / 2` and `p = sign(w)(p0 + kp |w|)`;
* a pivot is `P,-p,p` with equal magnitudes;
* in an arc, the inner wheel stops instead of reversing;
* output is clipped to `max_percent`.

It sends `S` when `/cmd_vel` is older than 0.3 s, and nothing at all unless the Pi
owns an armed robot.

## 5. Parameters

### `config/nav.yaml` (go_to_goal, align_heading)

| Parameter | Default | Units | Meaning |
| --- | --- | --- | --- |
| `v_max`, `v_min` | 0.3, 0.05 | m/s | drive speed limits |
| `omega_max`, `omega_min`, `omega_drive` | 0.6, 0.2, 0.4 | rad/s | rotate limits, steering limit while driving |
| `k_rho`, `k_psi`, `k_omega` | 0.5, 1.0, 1.0 | 1/s | gains |
| `rotate_to_drive_deg`, `drive_to_rotate_deg` | 20, 50 | deg | switching thresholds |
| `arrive_radius` | 0.4 | m | |
| `pause_s` | 0.3 | s | zero command between ROTATE and DRIVE |
| `accel_v`, `accel_omega` | 0.3, 1.5 | m/s^2, rad/s^2 | slew limits |
| `max_pose_age_s`, `max_fix_age_s` | 0.5, 2.0 | s | staleness aborts |
| `max_position_std` | 0.5 | m | from the EKF covariance |
| `fence_lookahead_s` | 0.5 | s | |
| `progress_distance`, `progress_window_s` | 0.2, 15 | m, s | no progress abort |
| `goal_timeout_s`, `arm_timeout_s` | 120, 3 | s | |
| `align_max_age_s` | 900 | s | realign after this long |
| `align_fence_margin` | 0.5 | m | ALIGN needs `d_align` plus this clearance |
| `geofence` | `[0.0]` (none) | m | **TODO survey.** Flat `x1, y1, x2, y2, ...` in map coordinates |
| `require_geofence` | true | | goals and routes are refused without a fence |
| `v_align`, `d_align` | 0.15, 1.5 | m/s, m | alignment drive (shared `/**` block) |
| `align_min_length`, `align_max_rms` | 1.0, 0.15 | m | fit acceptance |
| `align_yaw_variance` | 0.05 | rad^2 | sent with the heading |

### `config/waypoints.yaml` (routes)

`waypoints: {name: [x, y]}` (**TODO survey**: average `/localization/uwb_pose` for 30 s
on each point). Under `route:` it also sets:

* `mode`: `sequence`, or `shuffled` (a new order each session that never starts where
  the last one ended);
* `points` (empty means all of them);
* `dwell_s: [min, max]`, a random dwell drawn per point;
* `loops`.

Every point must be inside the geofence or the route is refused.

### Drive parameters (shared setup file)

They live in the `teensy_interface_node` block of `ballrobot_pkg/config/<setup>.yaml`.
The wheel odometry reads `r_eff` and `b_eff` from the same block. All are **TODO
calibrate**. The defaults are deliberately slow, with `p0` and `kp` taken from the
wheels-raised motor ID runs.

| Parameter | Default | Units |
| --- | --- | --- |
| `r_eff`, `b_eff` | 0.072, 0.22 | m |
| `p0`, `kp` | 3.0, 1.8 | %, % per rad/s |
| `max_percent` | 20 | % (firmware C limit, integer 10 to 100) |
| `accel_ms`, `brake_ms` | 800, 700 | ms (firmware C ramps) |
| `eps_v` | 0.02 | m/s |

## 6. Calibration (on the ground, inside UWB coverage)

Do not calibrate wheels-raised; shell load changes the speed curve.

```bash
ros2 launch robot_navigation_pkg navigation.launch.py config:=outdoor     # or bringup + localization
ros2 bag record -o calib_$(date +%Y%m%d_%H%M) /wheel_states /uwb/position \
    /localization/uwb_pose /imu/rpy /drive/command /drive/status /calibration/segment
ros2 run robot_navigation_pkg calibration_drive --ros-args -p mode:=all
python3 src/robot_navigation_pkg/scripts/calibrate.py calib_<date>
```

`calibration_drive` runs three manoeuvres, stopping for a few seconds before and
after each one:
* straight until UWB shows 3.5 m;
* a pivot until the IMU has turned 3.2 turns;
* five speed levels held 3 s each.

`calibrate.py` computes:
* `r_eff = d_UWB / mean dtheta`;
* `b_eff = r_eff (dtheta_R - dtheta_L) / dpsi`;
* `p0` and `kp`, from a fit of percent against steady |wheel rate|.

It then prints a block to paste into the setup file. A **negative `b_eff`** means the
IMU yaw and the wheels disagree on turn direction: fix `imu_adapter.yaw_sign` or the
motor polarity first. Commit the bag name and values with the setup file.

## 7. Acceptance tests (brief section 7)

Record a bag for every phase 5 and 6 trial:

```bash
ros2 bag record -o nav_trial_<n> /uwb/position /localization/uwb_pose /imu/rpy \
    /wheel_states /odometry/filtered /cmd_vel /goal_pose /nav/status \
    /drive/status /drive/owner /drive/armed /drive/command /set_pose /tf /tf_static
```

| Phase | Test | Result |
| --- | --- | --- |
| 1 | Firmware bench matrix F1 to F13 (below), wheels raised | **not run** (host simulation passes, section 8) |
| 1 | Sign conventions: `M,+x,+x` drives forward, forward roll gives positive MEAS | **not run** |
| 2 | `teleop_twist_keyboard` drives on blocks; killing the node stops within 1.2 s | **not run** |
| 3 | Calibration values committed with bags and script output | **not run** |
| 4 | Replayed bags: filtered path within UWB noise, yaw tracks IMU over 2 min | **not run** (needs a bag with UWB) |
| 5 | 5 m goal on the ground, 5 trials, arrival within 0.4 m | **not run** |
| 6 | Shuffled 4-waypoint route completed twice without abort | **not run** |

### Firmware bench matrix (wheels raised)

| # | Procedure | Pass |
| --- | --- | --- |
| F1 | Website only: arm, drive, pivot, stop, close tab | identical to v4.6 |
| F2 | Serial only: `C,800,700,20`, `A`, `M,15,15` at 10 Hz | drives; stop sending: soft stop at 0.5 s, `TIMEOUT_DISARMED` at 1.2 s |
| F3 | Serial driving, website connects | firmware does not disarm, but the website's own `K` on connect stops it (decision 1) |
| F4 | Serial driving, website sends `M` | ignored |
| F5 | Serial driving, website sends `A` | robot stops, `OWNER,BLE` on both, serial `M` ignored |
| F6 | F5, serial sends `A` | `STATE,BUSY,BLE`, no change |
| F7 | Serial driving, website `K` | stops, `OWNER,NONE` |
| F8 | Website driving, website disconnects | disarm (v4.6) |
| F9 | Serial driving, kill the Pi node or close the port | disarm within 1.2 s (check `Serial` goes false) |
| F10 | Stall fault during a serial run, `R` from the website | fault clears |
| F11 | 100 character line on serial while serial owns | disarm, robot neutral |
| F12 | Legacy `REF` lines | ignored, no effect |
| F13 | Both sources flood commands for 60 s | no hang, `MEAS` still at 100 Hz |

Serial commands can be typed with `screen /dev/ttyACM0 115200` (or the Arduino serial
monitor, newline endings) with the ROS interface node stopped.

## 8. Offline verification (done)

| What | How | Result |
| --- | --- | --- |
| Firmware v4.7 compiles | `arduino-cli`, `Seeeduino:nrf52:xiaonRF52840Sense`, `-Wall` | no warnings; 134,852 B flash, 14,652 B RAM |
| Firmware arbitration | `xiao/firmware_sim/run.sh`: real sketch against mocks, F2 to F13 plus extras | 47 / 47 checks |
| cmd_vel to firmware lines | `ballrobot_pkg/test/test_drive_command.py` | 8 tests pass |
| Calibration maths | `test/test_calibrate.py`, synthetic bag with known values | recovers r_eff, b_eff, p0, kp within 1 % |
| Navigation maths | `test/test_nav_math.py` | 10 tests pass |
| Controller closed loop | `test/test_go_to_goal_sim.py` with an rclpy stub: 5 m goal from unknown heading, second goal without realign, keeper takeover, UWB loss, no progress, geofence, shuffled route twice, sequence route | 14 tests pass |

Run everything without ROS:

```bash
xiao/firmware_sim/run.sh
python3 -m pytest ros2_ws/src/robot_navigation_pkg/test ros2_ws/src/ballrobot_pkg/test/test_drive_command.py
```

## 9. Known limitations

* **Heading** depends on the alignment drive and then on integrating the IMU yaw
  rate. There are no bias states, so heading slowly drifts between alignments; UWB
  plus the v_y = 0 constraint correct it only weakly.
* **Website connect stops a Pi run.** The website sends `K` on every connect (it was
  left unchanged). See decision 1 in `INVENTORY.md`.
* **Three UWB anchors**, no lever arm correction, and coplanar anchors (see
  `ballrobot_localization_pkg` README).
* **One EKF.** A dual (map and odom) filter is a documented future option if a never
  jumping `odom` frame is needed.
* `wasd_teleop` sends one Twist per key press, so with the 0.3 s cmd_vel timeout it
  only moves while keys repeat. Use `teleop_twist_keyboard`.
