# ballrobot_localization_pkg comparison runs

Not run yet. No recorded bag contains `/uwb/position` (see DISCOVERY.md, C7), and
ROS 2 is needed to replay. Once a bag with UWB exists, run on the Pi:

```bash
src/ballrobot_localization_pkg/scripts/run_comparison.sh bags/run_<id> <dropout_start_s>
```

That replaces this file with the table below, filled in, plus figures per run.

| Run | fuse_imu_yaw | Wheel odom | Purpose | global Hz | local Hz | clamped dropped | over gate (est.) | dropout drift global (m) | GT RMS global (m) | final cov trace (m^2) |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| A | true | on | Baseline | not run | | | | | | |
| B | false | on | Effect of magnetometer EMI | not run | | | | | | |
| C | true | off | Value of the encoders | not run | | | | | | |
| D | true | on, 10 s UWB dropout | Drift under canopy | not run | | | | | | |
