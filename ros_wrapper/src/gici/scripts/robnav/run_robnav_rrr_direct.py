#!/usr/bin/env python3
"""Run GLINS RobNav RRR through the direct-rosbag benchmark executable."""
import argparse
import csv
import datetime as dt
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time

import rosbag

from run_robnav_rrr import (
    BASE_CONFIG,
    DATA_ROOT,
    SUPPORTED_DATASETS,
    bag_info,
    check_ros_master,
    discover_paths,
    generate_estimator_config,
    git_commit,
    print_command,
    read_process_cpu_time,
    read_timing_events,
    select_dataset,
    select_dcb,
    signal_process_group,
    summarize_nav,
    summarize_timing,
    write_timing_summary,
    write_yaml,
)

WORKSPACE_ROOT = Path("/home/slam/glins_ws")
WORKSPACE = WORKSPACE_ROOT / "ros_wrapper"
DIRECT_EXECUTABLE = WORKSPACE / "devel/lib/gici_ros/gici_robnav_bag_main"
DIRECT_RESULT_ROOT = WORKSPACE_ROOT / "results/robnav_rrr_direct"
IMU_TOPIC = "/adi/adis16465/imu"
LIDAR_TOPIC = "/hesai/at128/points"
ROVER_OBSERVATIONS_TOPIC = "/gnss_rover/observations"
REFERENCE_OBSERVATIONS_TOPIC = "/gnss_reference/observations"
EPHEMERIDES_TOPIC = "/gnss_reference/ephemerides"
ANTENNA_POSITION_TOPIC = "/gnss_reference/antenna_position"


def read_base_ecef(reference_bag):
    """Read the fixed reference ECEF position that ROS playback previously preloaded."""
    with rosbag.Bag(str(reference_bag), "r") as bag:
        for _, message, _ in bag.read_messages(topics=[ANTENNA_POSITION_TOPIC]):
            position = list(message.pos)
            if len(position) != 3 or not all(math.isfinite(value) for value in position):
                raise RuntimeError("Reference antenna position is not a finite ECEF triplet")
            return position
    raise RuntimeError(f"No antenna position found in {reference_bag}")


def read_total_running_time(path):
    if not path.is_file():
        raise RuntimeError("total_running_time.txt was not generated")
    for line in path.read_text().splitlines():
        key, separator, value = line.partition("=")
        if key == "total_running_time_s" and separator:
            total = float(value)
            if math.isfinite(total) and total > 0.0:
                return total
    raise RuntimeError("total_running_time.txt has no valid total_running_time_s")


def parse_direct_output(output):
    values = {}
    for key in ("formal_input_loop_time_s", "drain_time_s"):
        match = re.search(rf"^{key}=([-+0-9.eE]+)$", output, re.MULTILINE)
        if match:
            values[key] = float(match.group(1))
    counts = re.search(
        r"^ephemerides=(\d+), imu=(\d+), lidar=(\d+), rover_gnss=(\d+), "
        r"reference_gnss=(\d+)$", output, re.MULTILINE)
    if counts:
        values.update(dict(zip(
            ("ephemeris_count", "imu_count", "lidar_count", "rover_obs_count", "reference_obs_count"),
            map(int, counts.groups()))))
    ephemeris_phases = re.search(
        r"^ephemerides_preload=(\d+), ephemerides_formal=(\d+)$",
        output, re.MULTILINE)
    if ephemeris_phases:
        values.update(dict(zip(
            ("ephemeris_preload_count", "ephemeris_formal_count"),
            map(int, ephemeris_phases.groups()))))
    peaks = re.search(
        r"^queue_peak_addin=(\d+), queue_peak_lidar_frontend=(\d+), "
        r"queue_peak_backend=(\d+)$", output, re.MULTILINE)
    if peaks:
        values.update(dict(zip(
            ("peak_measurement_addin", "peak_lidar_frontend", "peak_backend"),
            map(int, peaks.groups()))))
    return values


def stop_process(proc, timeout_s=10.0):
    if proc is None or proc.poll() is not None:
        return None if proc is None else proc.returncode
    signal_process_group(proc, signal.SIGINT)
    try:
        return proc.wait(timeout=timeout_s)
    except subprocess.TimeoutExpired:
        signal_process_group(proc, signal.SIGTERM)
        return proc.wait(timeout=5.0)


def write_run_summary(path, summary):
    fields = [
        "dataset", "run_id", "sequence_duration_s", "total_running_time_s",
        "process_cpu_start_s", "process_cpu_end_s", "process_cpu_time_s",
        "formal_input_loop_time_s", "drain_time_s", "ephemeris_count",
        "ephemeris_preload_count", "ephemeris_formal_count", "imu_count",
        "lidar_count", "rover_obs_count", "reference_obs_count", "nav_count",
        "nav_first_sow", "nav_last_sow", "lidar_frontend_count",
        "lidar_backend_add_count", "lidar_frontend_minus_backend_add",
        "peak_measurement_addin", "peak_lidar_frontend", "peak_backend",
        "direct_exit_code", "pipeline_drained", "run_valid",
    ]
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerow({field: summary.get(field) for field in fields})


def print_summary(summary, run_dir):
    print("\n" + "=" * 80)
    print("Direct-bag Run Summary")
    print("=" * 80)
    print("sequence duration     : {}".format(summary["sequence_duration_s"]))
    print("total running time    : {}".format(summary["total_running_time_s"]))
    print("process CPU time      : {}".format(summary["process_cpu_time_s"]))
    print("formal input loop     : {}".format(summary["formal_input_loop_time_s"]))
    print("drain time            : {}".format(summary["drain_time_s"]))
    print("NAV records           : {}".format(summary["nav_count"]))
    print("LiDAR frontend        : {}".format(summary["lidar_frontend_count"]))
    print("LiDAR backend add     : {}".format(summary["lidar_backend_add_count"]))
    print("run valid             : {}".format("YES" if summary["run_valid"] else "NO"))
    print("result dir            : {}".format(run_dir))


def main():
    parser = argparse.ArgumentParser(
        description="Run GLINS RobNav RRR with direct rosbag input and Total wall-time output.")
    parser.add_argument("dataset", nargs="?", choices=SUPPORTED_DATASETS,
                        help="building02 / street00 / street01 / street02")
    parser.add_argument("--duration", type=float, default=None,
                        help="Optional direct-input duration in seconds, for example --duration 100")
    args = parser.parse_args()

    dataset = select_dataset(args.dataset)
    run_id = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = DIRECT_RESULT_ROOT / dataset / run_id
    run_dir.mkdir(parents=True, exist_ok=False)

    errors = []
    interrupted = False
    direct_proc = None
    direct_exit_code = None
    cpu_start = None
    cpu_end = None
    direct_output = ""
    nav_summary = {"valid": False, "errors": [], "count": 0,
                   "first_sow": None, "last_sow": None}
    timing_summary = {"rows": [], "valid": False, "detail_module_sum_s": None,
                      "lidar_frontend_count": 0, "lidar_backend_add_count": 0}
    direct_values = {}
    total_running_time_s = None
    metadata = {}

    try:
        check_ros_master()
        if not DIRECT_EXECUTABLE.is_file():
            raise RuntimeError(f"Direct executable not found: {DIRECT_EXECUTABLE}")

        sensor_bag, rover_bag, reference_bag, ephemeris_bag = discover_paths(dataset)
        infos = {
            "sensor": bag_info(sensor_bag), "rover": bag_info(rover_bag),
            "reference": bag_info(reference_bag), "ephemeris": bag_info(ephemeris_bag),
        }
        sensor_start, sensor_end = infos["sensor"]["start"], infos["sensor"]["end"]
        full_duration = sensor_end - sensor_start
        duration = full_duration if args.duration is None else min(args.duration, full_duration)
        if duration <= 0.0:
            raise RuntimeError("Requested duration must be positive")
        requested_end = sensor_start + duration
        for name in ("rover", "reference"):
            if infos[name]["start"] > sensor_start or infos[name]["end"] < requested_end:
                raise RuntimeError(f"{name} bag does not fully cover the requested sensor interval")
        if infos["ephemeris"]["start"] > sensor_start:
            raise RuntimeError("Ephemeris bag starts after the sensor bag")

        dcb_path, utc = select_dcb(sensor_start)
        base_ecef = read_base_ecef(reference_bag)
        estimator_config = generate_estimator_config(dcb_path, run_dir)
        metadata = {
            "run": {"dataset": dataset, "run_id": run_id, "result_dir": str(run_dir)},
            "glins": {
                "workspace": str(WORKSPACE_ROOT), "repository": "Garfield-cn/GLINS",
                "git_commit": git_commit(), "git_dirty": "not_checked",
                "build_type": "RelWithDebInfo", "config_file": str(estimator_config),
                "base_config_file": str(BASE_CONFIG),
                "direct_executable": str(DIRECT_EXECUTABLE),
            },
            "data": {
                "sensor_bag": str(sensor_bag), "rover_bag": str(rover_bag),
                "reference_bag": str(reference_bag), "ephemeris_bag": str(ephemeris_bag),
                "dcb_file": str(dcb_path), "base_ecef": base_ecef,
            },
            "timing": {
                "bag_sensor_start": sensor_start, "bag_sensor_end": sensor_end,
                "direct_sensor_start": sensor_start, "direct_sensor_end": requested_end,
                "sequence_duration_s": duration,
                "definition": "GRLINS-aligned total wall-clock running time",
            },
        }

        command = [
            DIRECT_EXECUTABLE, estimator_config, sensor_bag, rover_bag, reference_bag,
            ephemeris_bag, IMU_TOPIC, LIDAR_TOPIC, ROVER_OBSERVATIONS_TOPIC,
            REFERENCE_OBSERVATIONS_TOPIC, EPHEMERIDES_TOPIC,
            *(f"{value:.12f}" for value in base_ecef),
            f"{sensor_start:.9f}", f"{duration:.6f}",
        ]
        print("\n" + "=" * 80)
        print("RobNav GLINS RRR Direct-bag Run")
        print("=" * 80)
        print(f"dataset              : {dataset}")
        print(f"result dir           : {run_dir}")
        print(f"sensor UTC date      : {utc.isoformat()}")
        print(f"sequence duration    : {duration:.3f} s")
        print(f"base ECEF            : {base_ecef}")
        print_command(command)

        environment = os.environ.copy()
        environment["GLINS_RESULT_DIR"] = str(run_dir)
        # Keep verbose ROS and glog output out of a pipe so it cannot block a long run.
        direct_log_path = run_dir / "direct_runner.log"
        with direct_log_path.open("w") as direct_log:
            direct_proc = subprocess.Popen(
                [str(part) for part in command], cwd=WORKSPACE, env=environment,
                stdout=direct_log, stderr=subprocess.STDOUT, text=True, start_new_session=True)
            cpu_start = read_process_cpu_time(direct_proc.pid)
            while direct_proc.poll() is None:
                try:
                    cpu_end = read_process_cpu_time(direct_proc.pid)
                except (FileNotFoundError, ProcessLookupError):
                    break
                time.sleep(0.05)
            direct_exit_code = direct_proc.wait()
        direct_output = direct_log_path.read_text(errors="replace")
        print(direct_output, end="")
        if direct_exit_code != 0:
            raise RuntimeError(f"Direct executable exited with code {direct_exit_code}")
        if cpu_end is None:
            cpu_end = cpu_start

        total_running_time_s = read_total_running_time(run_dir / "total_running_time.txt")
        direct_values = parse_direct_output(direct_output)

    except KeyboardInterrupt:
        interrupted = True
        errors.append("interrupted_by_user")
        print("\nInterrupted by user")
    except Exception as error:
        errors.append(str(error))
        print(f"\nRun error: {error}", file=sys.stderr)
    finally:
        if direct_proc is not None and direct_proc.poll() is None:
            direct_exit_code = stop_process(direct_proc)
        elif direct_proc is not None and direct_exit_code is None:
            direct_exit_code = direct_proc.returncode

        nav_path = run_dir / "glins.nav"
        timing_path = run_dir / "timing_events.csv"
        nav_exists = nav_path.is_file() and nav_path.stat().st_size > 0
        timing_exists = timing_path.is_file() and timing_path.stat().st_size > 0
        if nav_exists:
            try:
                nav_summary = summarize_nav(nav_path)
                errors.extend(nav_summary["errors"])
            except (OSError, ValueError) as error:
                errors.append(f"Unable to parse glins.nav: {error}")
        else:
            errors.append("glins.nav not generated or empty")
        if timing_exists:
            try:
                events = read_timing_events(timing_path)
                timing_summary = summarize_timing(events)
                if not timing_summary["valid"]:
                    errors.append("timing_events.csv contains an invalid duration")
            except (OSError, ValueError, RuntimeError) as error:
                errors.append(f"Unable to parse timing_events.csv: {error}")
        else:
            errors.append("timing_events.csv not generated or empty")

        write_timing_summary(run_dir / "timing_summary.csv", timing_summary["rows"])
        process_cpu_time_s = None if cpu_start is None or cpu_end is None else cpu_end - cpu_start
        lidar_frontend_minus_backend_add = (
            timing_summary["lidar_frontend_count"] - timing_summary["lidar_backend_add_count"])
        run_valid = all([
            not interrupted, direct_exit_code == 0,
            total_running_time_s is not None and total_running_time_s > 0.0,
            nav_exists and nav_summary["valid"] and nav_summary["count"] > 0,
            timing_exists and timing_summary["valid"] and bool(timing_summary["rows"]),
            not errors,
        ])
        summary = {
            "dataset": dataset, "run_id": run_id,
            "sequence_duration_s": metadata.get("timing", {}).get("sequence_duration_s"),
            "total_running_time_s": total_running_time_s,
            "process_cpu_start_s": cpu_start, "process_cpu_end_s": cpu_end,
            "process_cpu_time_s": process_cpu_time_s,
            "formal_input_loop_time_s": direct_values.get("formal_input_loop_time_s"),
            "drain_time_s": direct_values.get("drain_time_s"),
            "ephemeris_count": direct_values.get("ephemeris_count"),
            "ephemeris_preload_count": direct_values.get("ephemeris_preload_count"),
            "ephemeris_formal_count": direct_values.get("ephemeris_formal_count"),
            "imu_count": direct_values.get("imu_count"),
            "lidar_count": direct_values.get("lidar_count"),
            "rover_obs_count": direct_values.get("rover_obs_count"),
            "reference_obs_count": direct_values.get("reference_obs_count"),
            "nav_count": nav_summary["count"], "nav_first_sow": nav_summary["first_sow"],
            "nav_last_sow": nav_summary["last_sow"],
            "lidar_frontend_count": timing_summary["lidar_frontend_count"],
            "lidar_backend_add_count": timing_summary["lidar_backend_add_count"],
            "lidar_frontend_minus_backend_add": lidar_frontend_minus_backend_add,
            "peak_measurement_addin": direct_values.get("peak_measurement_addin"),
            "peak_lidar_frontend": direct_values.get("peak_lidar_frontend"),
            "peak_backend": direct_values.get("peak_backend"),
            "direct_exit_code": direct_exit_code,
            "pipeline_drained": direct_exit_code == 0,
            "run_valid": run_valid,
        }
        metadata.setdefault("runtime", {}).update({
            "direct_exit_code": direct_exit_code, "process_cpu_start_s": cpu_start,
            "process_cpu_end_s": cpu_end, "process_cpu_time_s": process_cpu_time_s,
            "total_running_time_s": total_running_time_s,
            "direct_output": direct_output, "errors": errors,
            "run_valid": run_valid,
        })
        write_run_summary(run_dir / "run_summary.csv", summary)
        write_yaml(run_dir / "runtime_config.yaml", metadata)
        print_summary(summary, run_dir)
        if errors:
            print("run errors            : " + "; ".join(errors), file=sys.stderr)

    return 0 if summary["run_valid"] else 1


if __name__ == "__main__":
    sys.exit(main())
