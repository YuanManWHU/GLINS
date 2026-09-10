#!/usr/bin/env python3
"""Run and summarize a reproducible GLINS RobNav RRR experiment."""
import argparse
import csv
from dataclasses import dataclass
import datetime as dt
import glob
import math
import os
from pathlib import Path
import signal
import statistics
import subprocess
import sys
import time

import rosbag
import yaml

WORKSPACE_ROOT = Path("/home/slam/glins_ws")
WORKSPACE = WORKSPACE_ROOT / "ros_wrapper"
DATA_ROOT = Path("/media/ym/BLUE/Datasets/RobNav_Dataset/Robot2/i2Nav-Fusion")
BASE_CONFIG = WORKSPACE / "src/gici/option/robnav/ros_rostopic_rtk_imu_lidar_rrr_robnav.yaml"
RESULT_ROOT = WORKSPACE_ROOT / "results/robnav_rrr"
SUPPORTED_DATASETS = ["building02", "street00", "street01", "street02"]
TOPICS = [
    "/adi/adis16465/imu", "/hesai/at128/points", "/gnss_rover/observations",
    "/gnss_reference/observations", "/gnss_reference/ephemerides",
]
DETAIL_MODULES = {
    "initialization", "lidar_initialization", "lidar_preprocess", "gnss_aux_rtk",
    "gnss_factor", "lidar_factor", "optimization", "gnss_post", "lidar_post",
    "marginalization",
}
POST_PLAY_WAIT_S = 1.0
CLK_TCK = os.sysconf(os.sysconf_names["SC_CLK_TCK"])


@dataclass
class TimingEvent:
    event_id: int
    scope: str
    module: str
    trigger: str
    data_timestamp: float
    gps_sow: float
    wall_start_ms: float
    duration_ms: float
    thread_id: str


def print_command(command):
    print("\n$", " ".join(str(part) for part in command), flush=True)


def run(command):
    print_command(command)
    return subprocess.run([str(part) for part in command], check=True)


def bag_info(path):
    """Use the earliest actual record timestamp, not rosbag metadata."""
    with rosbag.Bag(str(path), "r") as bag:
        first_record = next(bag.read_messages(), None)
        if first_record is None:
            raise RuntimeError(f"Empty bag: {path}")
        return {"start": float(first_record.timestamp.to_sec()), "end": float(bag.get_end_time())}


def select_dataset(argument):
    if argument:
        return argument
    print("Select dataset:")
    for index, name in enumerate(SUPPORTED_DATASETS, 1):
        print(f"  {index}. {name}")
    while True:
        value = input("dataset> ").strip()
        if value in SUPPORTED_DATASETS:
            return value
        if value.isdigit() and 1 <= int(value) <= len(SUPPORTED_DATASETS):
            return SUPPORTED_DATASETS[int(value) - 1]
        print("Invalid selection.")


def discover_paths(dataset):
    sensor_bag = DATA_ROOT / dataset / f"{dataset}.bag"
    if not sensor_bag.is_file():
        raise RuntimeError(f"Sensor bag not found: {sensor_bag}")
    patterns = [
        str(DATA_ROOT / "GNSS" / "*" / dataset / "gici_bags"),
        str(DATA_ROOT / "GNSS" / dataset / "gici_bags"),
    ]
    gnss_dirs = sorted({
        Path(path).resolve() for pattern in patterns for path in glob.glob(pattern)
        if Path(path).is_dir()
    })
    if len(gnss_dirs) != 1:
        raise RuntimeError(
            f"Expected exactly one GNSS gici_bags directory for {dataset}, "
            f"found {len(gnss_dirs)}:\n  " + "\n  ".join(map(str, gnss_dirs))
        )
    rover_bag = gnss_dirs[0] / "gnss_rover.bag"
    reference_bag = gnss_dirs[0] / "gnss_reference.bag"
    ephemeris_bag = gnss_dirs[0] / "gnss_ephemeris.bag"
    for path in (rover_bag, reference_bag, ephemeris_bag):
        if not path.is_file():
            raise RuntimeError(f"GNSS bag not found: {path}")
    return sensor_bag, rover_bag, reference_bag, ephemeris_bag


def select_dcb(sensor_start):
    utc = dt.datetime.fromtimestamp(sensor_start, tz=dt.timezone.utc)
    dcb_dir = DATA_ROOT / "GNSS" / "DCB" / str(utc.year)
    exact = dcb_dir / f"CAS0MGXRAP_{utc.year}{utc.timetuple().tm_yday:03d}0000_01D_01D_DCB.BSX"
    if exact.is_file():
        return exact, utc
    candidates = sorted(dcb_dir.glob(f"*{utc.year}{utc.timetuple().tm_yday:03d}*DCB.BSX"))
    if len(candidates) == 1:
        return candidates[0], utc
    raise RuntimeError(f"Unable to select unique DCB for UTC date {utc.date()}: {candidates}")


def generate_estimator_config(dcb_path, run_dir):
    """Copy the base config and inject only the selected DCB path."""
    if not BASE_CONFIG.is_file():
        raise RuntimeError(f"Common RRR config not found: {BASE_CONFIG}")
    with BASE_CONFIG.open() as stream:
        config = yaml.safe_load(stream)
    for item in config["stream"]["streamers"]:
        streamer = item.get("streamer", {})
        if streamer.get("tag") == "str_dcb_file":
            streamer["path"] = str(dcb_path)
            break
    else:
        raise RuntimeError("str_dcb_file streamer not found in base config")
    path = run_dir / "estimator_runtime.yaml"
    with path.open("w") as stream:
        yaml.safe_dump(config, stream, sort_keys=False)
    return path


def check_ros_master():
    result = subprocess.run(["rosnode", "list"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    if result.returncode != 0:
        raise RuntimeError("ROS master is not available. Start roscore first.")


def read_process_cmdline(pid):
    raw = (Path("/proc") / str(pid) / "cmdline").read_bytes()
    return raw.replace(b"\0", b" ").decode(errors="replace").strip()


def find_gici_ros_main_pids():
    processes = {}
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        try:
            # rosrun cmdline includes the target name, so verify the executable itself.
            if (entry / "exe").resolve().name != "gici_ros_main":
                continue
            command = read_process_cmdline(int(entry.name))
        except (FileNotFoundError, PermissionError, ProcessLookupError, OSError):
            continue
        processes[int(entry.name)] = command
    return processes


def wait_for_new_gici_ros_main(existing_pids, launch_proc, timeout_s=10.0):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if launch_proc.poll() is not None:
            raise RuntimeError("GLINS exited while waiting for gici_ros_main")
        current = find_gici_ros_main_pids()
        new_pids = set(current) - existing_pids
        if len(new_pids) == 1:
            pid = next(iter(new_pids))
            return pid, current[pid]
        if len(new_pids) > 1:
            raise RuntimeError(f"Multiple new gici_ros_main processes found: {sorted(new_pids)}")
        time.sleep(0.1)
    raise RuntimeError("Unable to locate gici_ros_main PID")


def read_process_cpu_time(pid):
    """Read utime + stime from /proc/<pid>/stat in seconds."""
    stat_text = (Path("/proc") / str(pid) / "stat").read_text()
    closing_parenthesis = stat_text.rfind(")")
    if closing_parenthesis < 0:
        raise RuntimeError(f"Malformed /proc/{pid}/stat")
    fields = stat_text[closing_parenthesis + 2:].split()
    if len(fields) < 13:
        raise RuntimeError(f"Incomplete /proc/{pid}/stat")
    return (int(fields[11]) + int(fields[12])) / CLK_TCK


def signal_process_group(proc, sig):
    """Signal the managed child process group, including rosrun descendants."""
    try:
        os.killpg(proc.pid, sig)
    except ProcessLookupError:
        pass


def stop_process_gracefully(proc, name, timeout_s=10.0):
    if proc is None:
        return "not_started", None
    if proc.poll() is not None:
        return "already_exited", proc.returncode
    print(f"\nStopping {name} with SIGINT ...")
    signal_process_group(proc, signal.SIGINT)
    try:
        return "SIGINT", proc.wait(timeout=timeout_s)
    except subprocess.TimeoutExpired:
        print(f"{name} did not exit after SIGINT; sending SIGTERM ...")
        signal_process_group(proc, signal.SIGTERM)
    try:
        return "SIGTERM", proc.wait(timeout=5.0)
    except subprocess.TimeoutExpired:
        print(f"{name} did not exit after SIGTERM; sending SIGKILL ...")
        signal_process_group(proc, signal.SIGKILL)
        return "SIGKILL", proc.wait()


def percentile(values, probability):
    if not values:
        return math.nan
    values = sorted(values)
    if len(values) == 1:
        return values[0]
    position = (len(values) - 1) * probability
    lower, upper = math.floor(position), math.ceil(position)
    if lower == upper:
        return values[lower]
    weight = position - lower
    return values[lower] * (1.0 - weight) + values[upper] * weight


def read_timing_events(path):
    required = {
        "event_id", "scope", "module", "trigger", "data_timestamp", "gps_sow",
        "wall_start_ms", "duration_ms", "thread_id",
    }
    with path.open(newline="") as stream:
        reader = csv.DictReader(stream)
        if not reader.fieldnames or not required.issubset(reader.fieldnames):
            raise RuntimeError("timing_events.csv has an invalid header")
        return [TimingEvent(
            event_id=int(row["event_id"]), scope=row["scope"], module=row["module"],
            trigger=row["trigger"], data_timestamp=float(row["data_timestamp"]),
            gps_sow=float(row["gps_sow"]), wall_start_ms=float(row["wall_start_ms"]),
            duration_ms=float(row["duration_ms"]), thread_id=row["thread_id"],
        ) for row in reader]


def summarize_timing(events):
    valid = all(math.isfinite(event.duration_ms) and event.duration_ms >= 0.0 for event in events)
    groups = {}
    for event in events:
        groups.setdefault((event.scope, event.module, event.trigger), []).append(event.duration_ms)
    rows = []
    for key in sorted(groups):
        values = groups[key]
        rows.append({
            "scope": key[0], "module": key[1], "trigger": key[2], "count": len(values),
            "total_ms": sum(values), "mean_ms": statistics.mean(values),
            "std_ms": statistics.stdev(values) if len(values) > 1 else 0.0,
            "median_ms": statistics.median(values), "p95_ms": percentile(values, 0.95),
            "max_ms": max(values),
        })
    detail_sum_s = sum(
        event.duration_ms for event in events
        if event.scope == "detail" and event.module in DETAIL_MODULES
    ) / 1000.0
    frontend_count = sum(
        event.scope == "coarse" and event.module == "lidar_frontend_total"
        and event.trigger == "lidar" for event in events
    )
    backend_count = sum(
        event.scope == "coarse" and event.module == "backend_add"
        and event.trigger == "lidar" for event in events
    )
    return {
        "rows": rows, "valid": valid, "detail_module_sum_s": detail_sum_s,
        "lidar_frontend_count": frontend_count, "lidar_backend_add_count": backend_count,
    }


def write_timing_summary(path, rows):
    fields = [
        "scope", "module", "trigger", "count", "total_ms", "mean_ms", "std_ms",
        "median_ms", "p95_ms", "max_ms",
    ]
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def summarize_nav(path):
    records, errors, previous_sow = [], [], None
    with path.open() as stream:
        for line_number, line in enumerate(stream, 1):
            fields = line.split()
            if len(fields) != 11 or fields[0] != "0":
                errors.append(f"glins.nav line {line_number} does not have the required 11 columns")
                continue
            try:
                values = [float(value) for value in fields[1:]]
            except ValueError:
                errors.append(f"glins.nav line {line_number} contains a non-numeric value")
                continue
            if not all(math.isfinite(value) for value in values):
                errors.append(f"glins.nav line {line_number} contains a non-finite value")
                continue
            if previous_sow is not None and values[0] <= previous_sow:
                errors.append("glins.nav SOW is not strictly increasing")
            previous_sow = values[0]
            records.append(values)
    return {
        "valid": not errors, "errors": errors, "count": len(records),
        "first_sow": records[0][0] if records else None,
        "last_sow": records[-1][0] if records else None,
        "latitude_min": min((row[1] for row in records), default=None),
        "latitude_max": max((row[1] for row in records), default=None),
        "longitude_min": min((row[2] for row in records), default=None),
        "longitude_max": max((row[2] for row in records), default=None),
        "height_min": min((row[3] for row in records), default=None),
        "height_max": max((row[3] for row in records), default=None),
    }


def git_commit():
    try:
        return subprocess.check_output(
            ["git", "show", "-s", "--format=" + chr(37) + "H", "HEAD"],
            cwd=WORKSPACE_ROOT, text=True, stderr=subprocess.DEVNULL,
        ).strip()
    except (OSError, subprocess.CalledProcessError):
        return "unavailable"


def write_yaml(path, contents):
    with path.open("w") as stream:
        yaml.safe_dump(contents, stream, sort_keys=False)


def write_run_summary(path, summary):
    fields = [
        "dataset", "run_id", "bag_sensor_start", "bag_sensor_end", "play_sensor_start",
        "play_sensor_end", "sequence_duration_s", "gici_pid",
        "process_cpu_start_s", "process_cpu_end_s", "process_cpu_time_s", "detail_module_sum_s",
        "post_play_wait_s", "nav_count", "nav_first_sow",
        "nav_last_sow", "lidar_frontend_count", "lidar_backend_add_count",
        "lidar_frontend_minus_backend_add", "rosbag_exit_code", "glins_exit_code", "shutdown_mode",
        "run_valid",
    ]
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerow({field: summary.get(field) for field in fields})


def print_summary(summary, run_dir):
    print("\n" + "=" * 80)
    print("Run Summary")
    print("=" * 80)
    print(f"sequence duration     : {summary['sequence_duration_s']}")
    print(f"process CPU time      : {summary['process_cpu_time_s']}")
    print(f"detail module sum     : {summary['detail_module_sum_s']}")
    print(f"post-play wait        : {summary['post_play_wait_s']}")
    print(f"NAV records           : {summary['nav_count']}")
    print(f"LiDAR frontend        : {summary['lidar_frontend_count']}")
    print(f"LiDAR backend add     : {summary['lidar_backend_add_count']}")
    print(f"LiDAR frontend - backend add : {summary['lidar_frontend_minus_backend_add']}")
    print(f"run valid             : {'YES' if summary['run_valid'] else 'NO'}")
    print(f"result dir            : {run_dir}")


def main():
    parser = argparse.ArgumentParser(description="Run GLINS RTK/IMU/LiDAR RRR on RobNav datasets.")
    parser.add_argument("dataset", nargs="?", choices=SUPPORTED_DATASETS,
                        help="building02 / street00 / street01 / street02")
    parser.add_argument("--duration", type=float, default=None,
                        help="Optional short playback duration, e.g. --duration 60")
    args = parser.parse_args()

    dataset = select_dataset(args.dataset)
    run_id = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = RESULT_ROOT / dataset / run_id
    run_dir.mkdir(parents=True, exist_ok=False)
    errors, interrupted = [], False
    glins_proc = play_proc = None
    gici_pid, gici_command = None, ""
    cpu_start = cpu_end = rosbag_exit_code = glins_exit_code = None
    shutdown_mode = "not_started"
    post_play_wait_s = None
    nav_summary = {"valid": False, "errors": [], "count": 0, "first_sow": None, "last_sow": None}
    timing_summary = {"rows": [], "valid": False, "detail_module_sum_s": None,
                      "lidar_frontend_count": 0, "lidar_backend_add_count": 0}
    metadata = {}

    try:
        check_ros_master()
        sensor_bag, rover_bag, reference_bag, ephemeris_bag = discover_paths(dataset)
        infos = {
            "sensor": bag_info(sensor_bag), "rover": bag_info(rover_bag),
            "reference": bag_info(reference_bag), "ephemeris": bag_info(ephemeris_bag),
        }
        sensor_start, sensor_end = infos["sensor"]["start"], infos["sensor"]["end"]
        earliest_start = min(info["start"] for info in infos.values())
        play_start = sensor_start - earliest_start
        full_duration = sensor_end - sensor_start
        play_duration = full_duration if args.duration is None else min(args.duration, full_duration)
        requested_end = sensor_start + play_duration
        eph_preload_duration = sensor_start - infos["ephemeris"]["start"]
        if abs(earliest_start + play_start - sensor_start) > 1e-3:
            raise RuntimeError("Internal rosbag start-time calculation mismatch")
        for name in ("rover", "reference"):
            if infos[name]["start"] > sensor_start or infos[name]["end"] < requested_end:
                raise RuntimeError(f"{name} bag does not fully cover the requested sensor interval")
        if infos["ephemeris"]["start"] > sensor_start:
            raise RuntimeError("Ephemeris bag starts after the sensor bag")

        dcb_path, utc = select_dcb(sensor_start)
        estimator_config = generate_estimator_config(dcb_path, run_dir)
        metadata = {
            "run": {"dataset": dataset, "run_id": run_id, "result_dir": str(run_dir)},
            "glins": {
                "workspace": str(WORKSPACE_ROOT), "repository": "Garfield-cn/GLINS",
                "git_commit": git_commit(), "git_dirty": "not_checked",
                "build_type": "RelWithDebInfo", "config_file": str(estimator_config),
                "base_config_file": str(BASE_CONFIG),
            },
            "data": {
                "sensor_bag": str(sensor_bag), "rover_bag": str(rover_bag),
                "reference_bag": str(reference_bag), "ephemeris_bag": str(ephemeris_bag),
                "dcb_file": str(dcb_path),
            },
            "timing": {
                "bag_sensor_start": sensor_start, "bag_sensor_end": sensor_end,
                "play_sensor_start": sensor_start, "play_sensor_end": requested_end,
                "sequence_duration_s": play_duration, "play_global_start": earliest_start,
                "play_start_offset_s": play_start,
            },
            "runtime": {"gici_pid": None, "rviz_started": False},
        }

        print("\n" + "=" * 80)
        print("RobNav GLINS RRR Run")
        print("=" * 80)
        print(f"dataset              : {dataset}")
        print(f"run id               : {run_id}")
        print(f"result dir           : {run_dir}")
        print(f"sensor bag           : {sensor_bag}")
        print(f"rover bag            : {rover_bag}")
        print(f"reference bag        : {reference_bag}")
        print(f"ephemeris bag        : {ephemeris_bag}")
        print(f"DCB                  : {dcb_path}")
        print(f"sensor UTC date      : {utc.isoformat()}")
        print(f"bag sensor start     : {sensor_start:.9f}")
        print(f"bag sensor end       : {sensor_end:.9f}")
        print(f"play sensor start    : {sensor_start:.9f}")
        print(f"play sensor end      : {requested_end:.9f}")
        print(f"global play start    : {earliest_start:.9f}")
        print(f"sequence duration    : {play_duration:.3f} s")

        existing_pids = set(find_gici_ros_main_pids())
        glins_env = os.environ.copy()
        glins_env["GLINS_RESULT_DIR"] = str(run_dir)
        launch_command = ["rosrun", "gici_ros", "gici_ros_main", str(estimator_config)]
        print("\n[1/4] Starting GLINS RRR ...")
        print_command(launch_command)
        # Isolate GLINS so its rosrun wrapper and gici_ros_main stop together.
        glins_proc = subprocess.Popen(launch_command, cwd=WORKSPACE, env=glins_env,
                                      start_new_session=True)
        gici_pid, gici_command = wait_for_new_gici_ros_main(existing_pids, glins_proc)
        if str(estimator_config) not in gici_command:
            raise RuntimeError("Located gici_ros_main is not using the current estimator config")
        metadata["runtime"]["gici_pid"] = gici_pid
        print("\nGLINS Process")
        print(f"rosrun pid            : {glins_proc.pid}")
        print(f"gici_ros_main pid     : {gici_pid}")
        print(f"cmdline               : {gici_command}")

        print("\n[2/4] Preloading ephemerides ...")
        run(["rosbag", "play", ephemeris_bag, "--topics", "/gnss_reference/ephemerides",
             "--rate=1000", f"--duration={eph_preload_duration:.6f}"])
        print("\n[3/4] Preloading reference antenna position ...")
        run(["rosbag", "play", reference_bag, "--topics", "/gnss_reference/antenna_position",
             "--duration=1"])
        if glins_proc.poll() is not None:
            raise RuntimeError("GLINS exited during preload")

        print("\n[4/4] Starting synchronized RRR playback ...")
        cpu_start = read_process_cpu_time(gici_pid)
        print(f"process CPU start     : {cpu_start:.6f} s")
        formal_command = [
            "rosbag", "play", sensor_bag, rover_bag, reference_bag, ephemeris_bag,
            f"--start={play_start:.6f}", f"--duration={play_duration:.6f}", "--topics", *TOPICS,
        ]
        print_command(formal_command)
        # Keep rosbag playback separately controllable during failures or Ctrl+C.
        play_proc = subprocess.Popen([str(part) for part in formal_command], start_new_session=True)
        rosbag_exit_code = play_proc.wait()
        if rosbag_exit_code != 0:
            raise RuntimeError(f"rosbag playback exited with code {rosbag_exit_code}")
        if glins_proc.poll() is not None:
            raise RuntimeError("GLINS exited before formal playback completed")
        print("rosbag playback       : finished")
        # Keep a bounded callback window; persistent polling is not backend backlog.
        post_play_wait_s = POST_PLAY_WAIT_S
        print(f"post-play callback wait: {post_play_wait_s:.1f} s")
        time.sleep(post_play_wait_s)
        cpu_end = read_process_cpu_time(gici_pid)
        print(f"process CPU end       : {cpu_end:.6f} s")

    except KeyboardInterrupt:
        interrupted = True
        errors.append("interrupted_by_user")
        print("\nInterrupted by user")
    except Exception as error:
        errors.append(str(error))
        print(f"\nRun error: {error}", file=sys.stderr)
    finally:
        if play_proc is not None and play_proc.poll() is None:
            stop_process_gracefully(play_proc, "rosbag playback", timeout_s=5.0)
        shutdown_mode, glins_exit_code = stop_process_gracefully(glins_proc, "GLINS")

        nav_path, timing_path = run_dir / "glins.nav", run_dir / "timing_events.csv"
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
        process_cpu_time = None if cpu_start is None or cpu_end is None else cpu_end - cpu_start
        lidar_frontend_minus_backend_add = (
            timing_summary["lidar_frontend_count"] - timing_summary["lidar_backend_add_count"]
        )
        run_valid = all([
            not interrupted, rosbag_exit_code == 0,
            process_cpu_time is not None and process_cpu_time > 0.0,
            nav_exists and nav_summary["valid"] and nav_summary["count"] > 0,
            timing_exists and timing_summary["valid"] and bool(timing_summary["rows"]),
            shutdown_mode == "SIGINT", not errors,
        ])
        summary = {
            "dataset": dataset, "run_id": run_id,
            "bag_sensor_start": metadata.get("timing", {}).get("bag_sensor_start"),
            "bag_sensor_end": metadata.get("timing", {}).get("bag_sensor_end"),
            "play_sensor_start": metadata.get("timing", {}).get("play_sensor_start"),
            "play_sensor_end": metadata.get("timing", {}).get("play_sensor_end"),
            "sequence_duration_s": metadata.get("timing", {}).get("sequence_duration_s"),
            "gici_pid": gici_pid, "process_cpu_start_s": cpu_start,
            "process_cpu_end_s": cpu_end, "process_cpu_time_s": process_cpu_time,
            "detail_module_sum_s": timing_summary["detail_module_sum_s"],
            "post_play_wait_s": post_play_wait_s, "nav_count": nav_summary["count"],
            "nav_first_sow": nav_summary["first_sow"], "nav_last_sow": nav_summary["last_sow"],
            "lidar_frontend_count": timing_summary["lidar_frontend_count"],
            "lidar_backend_add_count": timing_summary["lidar_backend_add_count"],
            "lidar_frontend_minus_backend_add": lidar_frontend_minus_backend_add, "rosbag_exit_code": rosbag_exit_code,
            "glins_exit_code": glins_exit_code, "shutdown_mode": shutdown_mode,
            "run_valid": run_valid,
        }
        metadata.setdefault("runtime", {}).update({
            "gici_pid": gici_pid, "rosbag_exit_code": rosbag_exit_code,
            "glins_exit_code": glins_exit_code, "shutdown_mode": shutdown_mode,
            "post_play_wait_s": post_play_wait_s, "process_cpu_start_s": cpu_start,
            "process_cpu_end_s": cpu_end, "process_cpu_time_s": process_cpu_time,
            "run_valid": run_valid, "errors": errors,
            "nav_summary": nav_summary,
            "timing_event_count": sum(row["count"] for row in timing_summary["rows"]),
        })
        write_run_summary(run_dir / "run_summary.csv", summary)
        write_yaml(run_dir / "runtime_config.yaml", metadata)
        print_summary(summary, run_dir)
        if errors:
            print("run errors            : " + "; ".join(errors), file=sys.stderr)

    return 0 if summary["run_valid"] else 1


if __name__ == "__main__":
    sys.exit(main())
