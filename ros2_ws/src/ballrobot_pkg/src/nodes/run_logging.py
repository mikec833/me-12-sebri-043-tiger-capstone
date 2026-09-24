"""
run_logging.py

Shared CSV run-numbering helper for imu_node.py, imu_quaternion_node.py,
uwb_node.py, teensy_interface_node.py and cmd_vel_bridge.py.

Each of those nodes declares a `run_id` parameter (string, default
""). bringup.launch.py generates one timestamp when it builds the
launch description and passes it to every node it starts as that
parameter, so all of them stamp their CSVs with the same identifier
-- e.g. imu_log_run20260924_161530.csv and
uwb_log_run20260924_161530.csv are then guaranteed to be from the same
run, even if one sensor has been started stand-alone more times than
the other in the past (which is what previously let e.g. imu run 2
line up with uwb run 1 instead of uwb run 2).

If a node is started outside that launch file with no run_id override,
run_id stays "" and resolve_run_id() falls back to the old behaviour:
scan the node's own output directory and auto-increment past the
highest existing "<filename_prefix><N>.csv". Runs started that way
are NOT guaranteed to line up across sensors -- pass a shared run_id
explicitly (e.g. `--ros-args -p run_id:=$(date +%Y%m%d_%H%M%S)` to
every node you start by hand) if you need that.
"""

import glob
import os


def stamp_to_seconds(stamp) -> float:
    """Convert a builtin_interfaces/Time (e.g. msg.header.stamp) to float epoch seconds.

    Nodes log this alongside their own elapsed-time-since-startup column
    so CSVs from different nodes/processes -- which each start their
    elapsed-time clock at a slightly different moment -- can still be
    joined on a common time axis (e.g. pandas merge_asof) when analyzing
    a run split across imu_log/uwb_log/wheel_speed_log/etc.
    """
    return stamp.sec + stamp.nanosec * 1e-9


def resolve_run_id(output_dir: str, filename_prefix: str, run_id_param: str) -> str:
    run_id_param = (run_id_param or "").strip()
    if run_id_param:
        return run_id_param

    existing = glob.glob(os.path.join(output_dir, f'{filename_prefix}*.csv'))
    run_numbers = [0]
    for path in existing:
        digits = os.path.basename(path)[len(filename_prefix):-len('.csv')]
        if digits.isdigit():
            run_numbers.append(int(digits))
    return str(max(run_numbers) + 1)
