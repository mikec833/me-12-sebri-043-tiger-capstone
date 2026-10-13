# ballrobot_localization_pkg comparison runs

Not run yet. No recorded bag contains `/uwb/position` (see DISCOVERY.md, C7), and
ROS 2 is needed to replay. Once a bag with UWB exists, run on the Pi:

```bash
src/ballrobot_localization_pkg/scripts/run_comparison.sh bags/run_<id> <dropout_start_s>
```

Run an alignment drive first (robot_navigation_pkg align_heading) or the
heading in each replay starts unknown. That replaces this file with the table
below, filled in, plus figures per run.

| Run | IMU yaw rate | Wheel odom | Purpose | filtered Hz | clamped dropped | over gate (est.) | dropout drift (m) | GT RMS filtered (m) | final cov trace (m^2) |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| A | on | on | Baseline | not run | | | | | |
| B | off | on | Value of the IMU yaw rate | not run | | | | | |
| C | on | off | Value of the encoders | not run | | | | | |
| D | on | on, 10 s UWB dropout | Drift under canopy | not run | | | | | |
