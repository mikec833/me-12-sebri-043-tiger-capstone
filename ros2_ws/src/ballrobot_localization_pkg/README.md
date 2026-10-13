# ballrobot_localization_pkg

Position and heading estimate for the Tiger Enrichment Ball Robot. It fuses UWB,
the BNO085 IMU and the wheel encoders with **one** `robot_localization` EKF, the
same filter for localization only runs and for navigation (`robot_navigation_pkg`).

This package only adds new nodes. It reads the topics that `ballrobot_pkg`
already publishes and never edits or restarts those nodes. `DISCOVERY.md` records
what each existing topic actually contains and why the adapters below exist.

**Status (14 Oct 2026):** the code is written, and the math and the evaluation
script are tested offline. It has **not yet been built or run under ROS 2**. No
recorded bag contains UWB data yet, so the comparison table in
`results/summary.md` is still empty. Section 8 lists what to do first.

**Changed 14 Oct 2026:** the first version ran two EKFs (`ekf_odom` and `ekf_map`)
and fused magnetometer yaw. The team confirmed the BNO085 yaw is **not**
magnetometer referenced (it is relative to power on), and asked for one EKF for
every use. The package now runs a single EKF on `/odometry/filtered`, fuses only
the IMU yaw rate, and gets absolute heading from an alignment drive (section 5).

---

## 1. Quick start

On the Pi, or any Ubuntu machine with ROS 2 (Humble or newer):

```bash
sudo apt install ros-$ROS_DISTRO-robot-localization
cd ~/ros2_ws
colcon build --packages-select ballrobot_localization_pkg
source install/setup.bash
```

**Live, on the robot.** Start the sensors as usual, then start localization with
the **same setup name** (see section 1a):

```bash
./scripts/run_sensors.sh config:=outdoor                                        # terminal 1
ros2 launch ballrobot_localization_pkg localization.launch.py config:=outdoor   # terminal 2
```

**Offline, from a bag.** The bag supplies the raw sensor topics and only the new
nodes run:

```bash
ros2 launch ballrobot_localization_pkg offline.launch.py \
    config:=src/outputs/run_<id>/config_outdoor.yaml    # terminal 1
ros2 bag play bags/run_<id> --clock                    # terminal 2
```

Pass the setup the bag was recorded with. `bringup.launch.py` saves a copy of it
next to each run's CSVs, as shown above.

**All four comparison runs**, then evaluation and `results/summary.md`:

```bash
CONFIG=src/outputs/run_<id>/config_outdoor.yaml \
    src/ballrobot_localization_pkg/scripts/run_comparison.sh bags/run_<id> 30
```

The `30` is when, in seconds after the first UWB sample, run D starts its 10 s UWB
blackout. Pick a stretch where the robot is moving. `CONFIG` defaults to `default`.

### 1a. Shared setup files

Each physical setup (an anchor survey, plus any calibration that goes with it) is
one YAML file in `ballrobot_pkg/config/`. Both packages read the same file:

| Package | How to select it | What it reads |
| --- | --- | --- |
| `ballrobot_pkg` | `run_sensors.sh config:=<name>` | Each sensor node reads its own block, e.g. `uwb_node.anchor_positions`. |
| `ballrobot_localization_pkg` | `localization.launch.py config:=<name>`, `evaluate.py run --config <name>` | `uwb_node.anchor_positions` for the clamp test and the anchor plot, `teensy_interface_node.r_eff` and `b_eff` for the wheel odometry, plus optional `uwb_pose_adapter`, `imu_adapter` and `wheel_odom_adapter` blocks. |

`<name>` is a file in `ballrobot_pkg/config/` without `.yaml`, or a path. Both
launch files use the same rule. The anchor layout is therefore written down once.
If the two packages were given different anchors, the clamp test would silently
stop working (the outdoor plane is 4.5 cm higher than the default one).

Adapter parameters are resolved in this order, last wins:
`config/adapters.yaml` < anchors from the setup's `uwb_node` block < the setup's
own adapter blocks < launch arguments such as `uwb_dropout`. To store per-setup
calibration, add a block to the setup file:

```yaml
imu_adapter:
  ros__parameters:
    yaw_sign: -1.0            # if calibrate.py reports a negative b_eff
```

After you edit or add a setup file, run `colcon build`, or pass the file by path.
The code is in `ballrobot_localization_pkg/setup_config.py`.

---

## 2. Node graph and topics

```
 ballrobot_pkg (sensors)            ballrobot_localization_pkg                     robot_localization
 uwb_node      /uwb/position  ---> uwb_pose_adapter   /localization/uwb_pose   ---> ekf  (x, y)
 imu_node      /imu/rpy       -+-> imu_adapter        /localization/imu        ---> ekf  (vyaw)
               /imu/accel     -+
 teensy_iface  /wheel_states  ---> wheel_odom_adapter /localization/wheel_odom ---> ekf  (vx, vy, vyaw)

 ekf -> /odometry/filtered + TF odom -> base_link       (subscribes /set_pose)
 static_transform_publisher x3 -> TF map -> odom (identity),
                                  base_link -> imu_link, base_link -> uwb_tag_link
```

| Topic | Type | Frame | Rate | Publisher |
| --- | --- | --- | --- | --- |
| `/uwb/position` | `geometry_msgs/PointStamped` | `map` | 5 to 7 Hz | `ballrobot_pkg/uwb_node` |
| `/imu/rpy` | `geometry_msgs/Vector3Stamped` (deg) | `imu_link` | 100 Hz | `ballrobot_pkg/imu_node` |
| `/imu/accel` | `geometry_msgs/Vector3Stamped` (milli g) | `imu_link` | 100 Hz | `ballrobot_pkg/imu_node` |
| `/wheel_states` | `sensor_msgs/JointState` (rad/s) | empty | 70 to 100 Hz | `ballrobot_pkg/teensy_interface_node` |
| `/localization/uwb_pose` | `geometry_msgs/PoseWithCovarianceStamped` | `map` | UWB rate minus drops | `uwb_pose_adapter` |
| `/localization/imu` | `sensor_msgs/Imu` | `imu_link` | 100 Hz | `imu_adapter` |
| `/localization/wheel_odom` | `nav_msgs/Odometry` (twist only) | `odom` / `base_link` | 50 Hz | `wheel_odom_adapter` |
| `/odometry/filtered` | `nav_msgs/Odometry` | `odom` (= `map`) / `base_link` | 30 Hz | `ekf` |
| `/set_pose` | `geometry_msgs/PoseWithCovarianceStamped` | `odom` | on alignment | `robot_navigation_pkg` |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | | 1 Hz per node | all of the above |

Every adapter copies `header.stamp` from its input unchanged. A message with a zero
stamp is dropped and logged, never restamped. Each adapter publishes its counters
(received, published, and each drop reason) on `/diagnostics`.

---

## 3. Frames (REP 105)

```
map ──(static identity)──> odom ──(ekf)──> base_link ──(static)──> imu_link
                                                   └─(static)──> uwb_tag_link
```

| Frame | Meaning |
| --- | --- |
| `map` | UWB anchor survey frame. Origin and axes are the `anchor_positions` parameter of `uwb_node` (see section 7). |
| `odom` | The EKF world frame. Identical to `map` (static identity), so positions are UWB map coordinates. |
| `base_link` | Centre of the inner drivetrain chassis. |
| `imu_link` | BNO085 mount. The default roll is **pi**, because every bag shows the board reading about 179 deg roll at rest (mounted upside down). |
| `uwb_tag_link` | UWB tag mount. The offset is published for completeness but not used by the EKF (see the limitations). |

Static TF offsets are launch arguments: `imu_x imu_y imu_z imu_roll imu_pitch imu_yaw`
and `uwb_x uwb_y uwb_z`, in metres and radians. Apart from `imu_roll` they default
to zero and **must be measured** on the robot.

---

## 4. Parameters

All adapter parameters live in `config/adapters.yaml`, and every one has its units
and source written next to it there. Values marked TODO there are first guesses.

### uwb_pose_adapter

| Parameter | Default | Units | How it was determined |
| --- | --- | --- | --- |
| `uwb_var_x`, `uwb_var_y` | 0.05 | m^2 | Sample-to-sample noise in the September UWB logs is about 0.1 m (0.01 m^2). Raised for longer outdoor ranges and multipath. |
| `uwb_var_z` | 1.0 | m^2 | The anchors are coplanar, so z is barely observed. Kept large on purpose. |
| `orientation_variance` | 1e6 | rad^2 | The pose carries no orientation. |
| `infer_clamped` | true | | `uwb_node` does not publish `clamped`, so it is inferred (see below). |
| `anchor_positions` | 1.8 m triangle | m | Taken from the shared setup file (section 1a). This value is only a fallback. |
| `clamp_tolerance` | 1e-3 | m | Distance from the anchor plane below which a sample counts as clamped. |
| `dropout_windows` | [0, 0] | s | Flat `[start, end, ...]` list. A zero-length pair does nothing. |
| `dropout_time_reference` | relative | | `relative`: seconds after the first UWB sample. `absolute`: raw stamp seconds. |

How clamping is detected: when the three ranges don't agree, `uwb_trilateration.py`
sets z^2 to 0, which puts the solution exactly on the plane through the three
anchors. The adapter drops any sample within `clamp_tolerance` of that plane. On the
two September logs this caught 243 of 243 clamped samples, with no false positives.

### imu_adapter

| Parameter | Default | Units | How it was determined |
| --- | --- | --- | --- |
| `sync_slop` | 0.005 | s | `/imu/rpy` and `/imu/accel` share a stamp, so this only covers jitter. |
| `accel_scale` | 9.80665e-3 | m/s^2 per count | The UART-RVC accelerometer reports milli g. At rest it reads about 1010 counts. |
| `yaw_sign` | 1.0 | | **TODO, verify.** Sets the sign of the fused yaw rate: a counter clockwise turn must give a positive rate. `calibrate.py` reports a negative `b_eff` when it is wrong. |
| `yaw_offset_deg` | 0.0 | deg | Shifts the published orientation only. Not fused. |
| `roll_variance`, `pitch_variance`, `yaw_variance` | 0.0012, 0.0012, 0.03 | rad^2 | Published for completeness. Orientation is not fused. |
| `derive_angular_velocity` | true | | The RVC frame has no gyro output, so the rate is derived from orientation. |
| `angular_velocity_window` | 5 | samples | 50 ms at 100 Hz. Lag is about half of that. |
| `angular_velocity_mode` | world_yaw | | `world_yaw`: d(yaw)/dt, the true heading rate even while the chassis tilts in the shell. `body`: full body rate. |
| `mount_roll`, `mount_pitch`, `mount_yaw` | pi, 0, 0 | rad | Set by the launch file from `imu_roll` etc., so the rate is expressed through the same mount as the TF. |
| `angular_velocity_variance` | 0.0025 | (rad/s)^2 | About 3 deg/s. |
| `linear_acceleration_variance` | 0.25 | (m/s^2)^2 | About 0.5 m/s^2. Acceleration is not fused. |

### wheel_odom_adapter

| Parameter | Default | Units | How it was determined |
| --- | --- | --- | --- |
| `input_units` | rad_s | | The XIAO firmware already converts counts to rad/s at the gearbox output. |
| `counts_per_rev` | 4480 | counts | Pololu #2825, 64 CPR x 70:1 (firmware constant). Only used for `counts_per_s` input. |
| `left_sign`, `right_sign` | 1.0 | | **TODO, verify.** Both wheels must read positive when driving forward. |
| `k_v` | 1.0 | | Extra slip factor. Leave at 1 once `r_eff` is calibrated against UWB. |
| `wheel_radius` | 0.072 | m | Fallback. Normally `teensy_interface_node.r_eff` from the shared setup file (`calibrate.py`). |
| `b_eff` | 0.22 | m | Fallback. Normally `teensy_interface_node.b_eff` from the shared setup file. |
| `var_vx` | 0.05 | (m/s)^2 | Navigation brief starting value. |
| `var_vy` | 0.001 | (m/s)^2 | Tight: the v_y = 0 constraint keeps heading observable between UWB fixes. |
| `var_vz` | 0.01 | (m/s)^2 | Unused in two_d_mode. |
| `var_wz` | 0.1 | (rad/s)^2 | Loose: shell slip makes the wheel yaw rate unreliable. |
| `output_rate_hz` | 50 | Hz | Wheel rates are averaged and published at this rate to save CPU. |

Kinematics: `v_x = k_v r (w_L + w_R) / 2`, `v_y = 0`, `v_z = 0`,
`w_z = k_v r (w_R - w_L) / b_eff`.

### EKF (`config/ekf.yaml`)

| Setting | Value |
| --- | --- |
| node name | `ekf` (subscribes `/set_pose`) |
| `world_frame` | odom (static identity to map) |
| `frequency`, `sensor_timeout` | 30 Hz, 0.2 s |
| `two_d_mode` | true |
| `imu0_config` | vyaw only |
| `odom0_config` | vx, vy, vyaw |
| `pose0_config` | x, y |
| `pose0_rejection_threshold` | 5.0 (2 DOF; well past the 99 % value of 3.03, so a briefly over confident filter still accepts good fixes) |
| `initial_estimate_covariance` | position 1.0 m^2, yaw pi^2 (unknown until alignment), rates 1e-3 |

The process noise is the 15 x 15 diagonal from the task brief: position 0.01,
roll and pitch 0.01, yaw 0.02, vx and vy 0.05, vz 0.01, vroll and vpitch 0.01,
vyaw 0.02, ax and ay 0.1, az 0.05. These are starting points for offline tuning,
inflated because the filter has no IMU bias states.

### Launch arguments

| Argument | Default | Effect |
| --- | --- | --- |
| `use_sim_time` | false | Set to true by `offline.launch.py`. |
| `two_d_mode` | true | `robot_localization` two_d_mode. |
| `use_imu` | true | false clears `imu0_config` (run B). |
| `use_wheel_odom` | true | false clears `odom0_config` (run C). |
| `uwb_dropout` | empty | `start,end[,...]` seconds; overrides `dropout_windows` (run D). |
| `config` | default | Shared setup file: a name in `ballrobot_pkg/config/` or a path (section 1a). |
| `adapters_config`, `ekf_config` | package files | Alternative parameter files. |

---

## 5. How heading is found, and why the IMU is used this way

`imu_node` runs the BNO085 in **UART-RVC** mode. Each frame contains roll, pitch and
yaw from the chip's own fusion plus the raw accelerometer. There is no gyro output
and no gravity-free linear acceleration, and the yaw is relative to power on (the
team confirmed it does not use the magnetometer; in testing it is reliable). So:

* **Yaw rate only.** The adapter derives d(yaw)/dt from the fused yaw and the EKF
  fuses that as vyaw. Unlike the body z rate, this is the true heading rate while
  the chassis pitches inside the shell (-23 to +39 deg in the 8 Oct drive, which
  would put the body rate up to 29 % off).
* **No orientation.** IMU yaw has an arbitrary zero, and roll and pitch do not
  matter in two_d_mode.
* **No acceleration.** It still contains gravity, the shell vibrates, and the
  filter has no bias states. Wheel odometry already gives velocity.
* **Absolute heading from an alignment drive.** UWB gives position only. The robot
  drives straight for about 1.5 m, a line is fitted through the UWB fixes, and the
  heading is sent to the EKF on `/set_pose` (`robot_navigation_pkg` `align_heading`,
  or the ALIGN state of `go_to_goal`). After that the EKF integrates the yaw rate,
  and heading stays weakly observable through v_y = 0 plus UWB. Before alignment
  the yaw variance is pi^2: positions are right, heading is not.

---

## 6. Replay and evaluation

`scripts/run_comparison.sh` records these topics during each replay:
`/uwb/position /imu/rpy /imu/accel /wheel_states /localization/* /odometry/*
/diagnostics /tf /tf_static`. To evaluate a single replay by hand:

```bash
python3 src/ballrobot_localization_pkg/scripts/evaluate.py run \
    --bag results/run_A/bag --out results/run_A --label A \
    [--dropout 30,40] [--ground-truth survey.csv]
python3 src/ballrobot_localization_pkg/scripts/evaluate.py summarize
```

It writes `xy.png`, `position_time.png`, `metrics.json` and `summary.md` for each
run, and `results/summary.md` for the comparison. It reports:

* Output rates.
* UWB samples dropped as clamped (from the adapter's `/diagnostics` counters).
* UWB rejections. `robot_localization` does not report rejected measurements on
  `/diagnostics`, even with `print_diagnostics: true`. The script lists any
  EKF diagnostics that mention rejection, and also **estimates** which samples
  failed the 5.0 gate. It does this by comparing each UWB sample with the last
  `/odometry/filtered` estimate before it, over x and y. To see the filter's own
  decisions, set `debug: true` in `ekf.yaml` and read its debug file.
* The filtered XY error against UWB at the start and end of each dropout window.
* RMS error of raw UWB and of `/odometry/filtered` against a ground truth CSV with
  columns `t,x,y`. `t` is either absolute stamp seconds or seconds after the first
  UWB sample.
* The offset between raw IMU yaw and the UWB track heading while driving forward.
  It is a diagnostic: a steady value means alignment and the IMU agree.
* A yaw rate sign check between the IMU and the wheels. A negative slope means
  `yaw_sign` or a wheel sign is wrong.

On a laptop without ROS, `pip install rosbags numpy matplotlib pyyaml` and the
same script will run.

---

## 7. Known limitations

* **No IMU bias states.** `robot_localization` has no bias states, so the process
  noise is inflated instead. Slow yaw rate bias shows up as heading drift between
  alignments.
* **Heading needs an alignment drive** after every start (and after a long UWB
  outage). The drive direction is unknown, so start the robot facing open ground.
* **UWB tag lever arm ignored.** `robot_localization` does not apply a lever arm to a
  `PoseWithCovarianceStamped` given in the world frame, so the offset from
  `base_link` to the tag is assumed negligible. The ball rolls, so if the tag is
  not near the chassis centre this error rotates with the chassis.
* **z is poorly observed.** With the live configuration all three anchors are at
  exactly 1.3 m, so the solution can't tell above from below the plane
  (`tag_below_anchor_plane` decides). Clamped samples are dropped and z gets a large
  variance.
* **Three anchors is the minimum.** Three anchors give exactly one solution, with no
  redundancy to reject a bad range, and geometry that degrades fast near the edges.
  A 40 m x 25 m enclosure with canopy and steel mesh needs at least 4, and better 6
  to 8, spread around the perimeter with some at different heights.
* **Map frame lives in `uwb_node` parameters.** `uwb_node` overwrites
  `uwb_trilateration.ANCHOR_POSITIONS` at runtime with its `anchor_positions`
  parameter, which comes from the shared setup file (`default`: a 1.8 m triangle
  at 1.3 m; `outdoor`: a 15 m triangle at 1.345 m). The values written in
  `uwb_trilateration.py` are not used. Always replay with the same setup the bag
  was recorded with.
* **UWB timestamps.** The task brief described an `elapsed_ms` bug in a
  `uwb_serial_reader.py` that stamps before a blocking serial read. **No such file
  or code exists** in this workspace. The reader actually used,
  `uwb_position_reader.read_block()`, takes `time.monotonic()` after the whole block
  has arrived. So the real issue is the opposite one: the stamp lags the true
  ranging time by the ranging, CLI and USB latency, and `latency_compensation_s` is
  0 (not measured). Measure that latency and set it on `uwb_node`.
* **Wheel timestamps.** `teensy_interface_node` stamps each line when its 50 Hz
  timer parses it, while the XIAO sends at 100 Hz. Stamp spacing therefore jitters
  between about 2 and 31 ms.
* **Encoder hardware.** The task brief names Pololu 37D 100:1 motors. The firmware
  says #2825, 70:1, 4480 counts/rev. It does not matter here because rad/s arrives
  already converted.

---

## 8. First things to do with real data

1. Record a bag with `/uwb/position` present (no bag recorded so far contains any):
   `./scripts/run_sensors.sh`. Include straight runs, in-place spins and a
   stationary minute.
2. Check `$ROS_DISTRO` on the Pi. The static TF publishers use the named-argument
   form (`--x`, `--frame-id`), which needs Humble or newer.
3. Build, replay, and check the TF tree with `ros2 run tf2_tools view_frames`. You
   should see `map -> odom -> base_link -> {imu_link, uwb_tag_link}`.
4. Set the signs. Drive forward: both wheel rates must be positive. Turn counter
   clockwise: `/localization/wheel_odom` `w_z` must be positive, and the yaw rate
   check in `evaluate.py` must report a positive slope. Earlier data hints at a
   possible mismatch: the only real drive (8 Oct, 17:00) shows a weak negative
   correlation between IMU and wheel yaw rate.
5. Run the drive calibration (`robot_navigation_pkg`: `calibration_drive`, then
   `scripts/calibrate.py`) and paste `r_eff`, `b_eff`, `p0`, `kp` into the setup file.
   Both the drive and the wheel odometry read them from there.
6. Run an alignment drive, then `run_comparison.sh`, and tune the variances.

---

## 9. Future improvements

* **Lever arm correction.** Transform the tag position to `base_link` in the
  adapter, using the current orientation, before fusing.
* **Fuse raw UWB ranges** instead of the trilaterated position, with a custom
  filter (for example an EKF or factor graph with one range measurement per
  anchor). This handles 2 out of 3 anchors, rejects single bad ranges, and avoids
  the clamping problem altogether.
* **Bias states** for gyro-derived yaw rate and accelerometer, in a custom filter.
* **A second EKF** (dual `map` and `odom` instances) if a continuous, never-jumping
  `odom` frame is needed later; it costs another filter on the Pi 3.
* **An elevated anchor**, or anchors at clearly different heights, to make z
  observable.
* **SHTP mode on the BNO085** instead of UART-RVC, to get the real gyro,
  gravity-free linear acceleration, the game rotation vector (no magnetometer),
  and the chip's own accuracy estimate. This needs a change to `imu_node`.
* **Publish `clamped`** from `uwb_node` (for example as a covariance in a
  `PoseWithCovarianceStamped`), so the adapter no longer has to infer it.
