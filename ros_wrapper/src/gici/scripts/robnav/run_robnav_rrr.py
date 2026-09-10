#!/usr/bin/env python3
import argparse
import datetime as dt
import glob
import os
import signal
import subprocess
import sys
import time

import rosbag
import yaml


WORKSPACE = os.path.expanduser("~/glins_ws/ros_wrapper")
DATA_ROOT = "/media/ym/BLUE/Datasets/RobNav_Dataset/Robot2/i2Nav-Fusion"
BASE_CONFIG = os.path.join(
    WORKSPACE,
    "src/gici/option/robnav/ros_rostopic_rtk_imu_lidar_rrr_robnav.yaml",
)

SUPPORTED_DATASETS = ["building02", "street00", "street01", "street02"]

TOPICS = [
    "/adi/adis16465/imu",
    "/hesai/at128/points",
    "/gnss_rover/observations",
    "/gnss_reference/observations",
    "/gnss_reference/ephemerides",
]


def run(cmd, check=True):
    print("\n$", " ".join(cmd), flush=True)
    return subprocess.run(cmd, check=check)


def bag_info(path):
    """
    Get bag timing using the actual record timestamps.

    Do NOT rely on Bag.get_start_time() for the start time: some converted
    GNSS bags (e.g. street00 ephemeris) have inconsistent metadata/index start
    times. rosbag play may see an earlier actual record timestamp than
    get_start_time() reports.

    read_messages() is timestamp-ordered, so the first yielded message gives
    the actual earliest record time without scanning the whole bag.
    """
    with rosbag.Bag(path, "r") as bag:
        first_record = next(bag.read_messages(), None)
        if first_record is None:
            raise RuntimeError(f"Empty bag: {path}")

        actual_start = float(first_record.timestamp.to_sec())

        # End time has not shown the same issue in these RobNav bags and is
        # only used for coverage checks / requested duration.
        actual_end = float(bag.get_end_time())

        return {
            "start": actual_start,
            "end": actual_end,
        }


def select_dataset(arg):
    if arg:
        return arg

    print("Select dataset:")
    for i, name in enumerate(SUPPORTED_DATASETS, 1):
        print(f"  {i}. {name}")

    while True:
        value = input("dataset> ").strip()
        if value in SUPPORTED_DATASETS:
            return value
        if value.isdigit() and 1 <= int(value) <= len(SUPPORTED_DATASETS):
            return SUPPORTED_DATASETS[int(value) - 1]
        print("Invalid selection.")


def discover_paths(dataset):
    sensor_bag = os.path.join(DATA_ROOT, dataset, f"{dataset}.bag")
    if not os.path.isfile(sensor_bag):
        raise RuntimeError(f"Sensor bag not found: {sensor_bag}")

    patterns = [
        os.path.join(DATA_ROOT, "GNSS", "*", dataset, "gici_bags"),
        os.path.join(DATA_ROOT, "GNSS", dataset, "gici_bags"),
    ]

    gnss_dirs = sorted(set(
        os.path.realpath(p)
        for pattern in patterns
        for p in glob.glob(pattern)
        if os.path.isdir(p)
    ))

    if len(gnss_dirs) != 1:
        raise RuntimeError(
            f"Expected exactly one GNSS gici_bags directory for {dataset}, "
            f"found {len(gnss_dirs)}:\n  " + "\n  ".join(gnss_dirs)
        )

    gnss_dir = gnss_dirs[0]
    rover_bag = os.path.join(gnss_dir, "gnss_rover.bag")
    reference_bag = os.path.join(gnss_dir, "gnss_reference.bag")
    ephemeris_bag = os.path.join(gnss_dir, "gnss_ephemeris.bag")

    for path in (rover_bag, reference_bag, ephemeris_bag):
        if not os.path.isfile(path):
            raise RuntimeError(f"GNSS bag not found: {path}")

    return sensor_bag, rover_bag, reference_bag, ephemeris_bag


def select_dcb(sensor_start):
    utc = dt.datetime.fromtimestamp(sensor_start, tz=dt.timezone.utc)
    year = utc.year
    doy = utc.timetuple().tm_yday

    dcb_dir = os.path.join(DATA_ROOT, "GNSS", "DCB", str(year))
    exact = os.path.join(
        dcb_dir,
        f"CAS0MGXRAP_{year}{doy:03d}0000_01D_01D_DCB.BSX",
    )

    if os.path.isfile(exact):
        return exact, utc

    candidates = sorted(
        glob.glob(os.path.join(dcb_dir, f"*{year}{doy:03d}*DCB.BSX"))
    )
    if len(candidates) == 1:
        return candidates[0], utc

    raise RuntimeError(
        f"Unable to select unique DCB for UTC date {utc.date()} (DOY {doy}).\n"
        f"Expected: {exact}\n"
        f"Candidates: {candidates}"
    )


def generate_runtime_config(dataset, dcb_path):
    if not os.path.isfile(BASE_CONFIG):
        raise RuntimeError(
            f"Common RRR config not found:\n  {BASE_CONFIG}\n"
            "Copy the validated building02 YAML to this path first."
        )

    with open(BASE_CONFIG, "r") as f:
        cfg = yaml.safe_load(f)

    for item in cfg["stream"]["streamers"]:
        streamer = item.get("streamer", {})
        if streamer.get("tag") == "str_dcb_file":
            streamer["path"] = dcb_path
            break
    else:
        raise RuntimeError("str_dcb_file streamer not found in base config")

    runtime_config = f"/tmp/glins_rrr_{dataset}.yaml"
    with open(runtime_config, "w") as f:
        yaml.safe_dump(cfg, f, sort_keys=False)

    return runtime_config


def check_ros_master():
    result = subprocess.run(
        ["rosnode", "list"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    if result.returncode != 0:
        raise RuntimeError("ROS master is not available. Start roscore first.")


def terminate_process(proc, name):
    if proc is None or proc.poll() is not None:
        return

    print(f"\nStopping {name} ...")
    proc.send_signal(signal.SIGINT)
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        proc.terminate()


def main():
    parser = argparse.ArgumentParser(
        description="Run GLINS RTK/IMU/LiDAR RRR on RobNav datasets."
    )
    parser.add_argument(
        "dataset",
        nargs="?",
        choices=SUPPORTED_DATASETS,
        help="building02 / street00 / street01 / street02; omit for interactive selection",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=None,
        help="Optional short playback duration, e.g. --duration 60",
    )
    args = parser.parse_args()

    dataset = select_dataset(args.dataset)
    check_ros_master()

    sensor_bag, rover_bag, reference_bag, ephemeris_bag = discover_paths(dataset)

    print(f"\nDataset: {dataset}")
    print("Reading actual rosbag record-time starts ...")

    infos = {
        "sensor": bag_info(sensor_bag),
        "rover": bag_info(rover_bag),
        "reference": bag_info(reference_bag),
        "ephemeris": bag_info(ephemeris_bag),
    }

    for name, info in infos.items():
        info["duration"] = info["end"] - info["start"]
        print(
            f"  {name:10s}: "
            f"start={info['start']:.9f}  "
            f"end={info['end']:.9f}  "
            f"duration={info['duration']:.3f}"
        )

    sensor_start = infos["sensor"]["start"]
    sensor_end = infos["sensor"]["end"]

    # IMPORTANT:
    # rosbag Player computes --start from full_view.getBeginTime(), i.e. the
    # earliest record time across ALL input bags before topic filtering.
    earliest_start = min(v["start"] for v in infos.values())

    play_start = sensor_start - earliest_start
    full_duration = sensor_end - sensor_start
    play_duration = (
        full_duration
        if args.duration is None
        else min(args.duration, full_duration)
    )

    # Single-bag ephemeris preload also starts from that bag's true record start.
    eph_preload_duration = sensor_start - infos["ephemeris"]["start"]

    # Numerical sanity check.
    reconstructed_start = earliest_start + play_start
    if abs(reconstructed_start - sensor_start) > 1e-3:
        raise RuntimeError(
            "Internal start-time calculation mismatch:\n"
            f"  reconstructed={reconstructed_start:.9f}\n"
            f"  sensor_start={sensor_start:.9f}"
        )

    requested_end = sensor_start + play_duration

    if infos["rover"]["start"] > sensor_start or infos["rover"]["end"] < requested_end:
        raise RuntimeError(
            "Rover bag does not fully cover requested sensor interval:\n"
            f"  requested: {sensor_start:.6f} -> {requested_end:.6f}\n"
            f"  rover:     {infos['rover']['start']:.6f} -> {infos['rover']['end']:.6f}"
        )

    if infos["reference"]["start"] > sensor_start or infos["reference"]["end"] < requested_end:
        raise RuntimeError(
            "Reference bag does not fully cover requested sensor interval:\n"
            f"  requested: {sensor_start:.6f} -> {requested_end:.6f}\n"
            f"  reference: {infos['reference']['start']:.6f} -> {infos['reference']['end']:.6f}"
        )

    if infos["ephemeris"]["start"] > sensor_start:
        raise RuntimeError("Ephemeris bag starts after the sensor bag.")

    dcb_path, utc = select_dcb(sensor_start)
    runtime_config = generate_runtime_config(dataset, dcb_path)

    print("\nResolved run:")
    print(f"  sensor UTC date     : {utc.isoformat()}")
    print(f"  DCB                 : {dcb_path}")
    print(f"  runtime YAML        : {runtime_config}")
    print(f"  global earliest     : {earliest_start:.9f}")
    print(f"  sensor start        : {sensor_start:.9f}")
    print(f"  play --start        : {play_start:.6f}")
    print(f"  play --duration     : {play_duration:.6f}")
    print(f"  ephemeris preload   : {eph_preload_duration:.6f}")

    gici_proc = None

    def cleanup(*_):
        terminate_process(gici_proc, "GLINS")
        sys.exit(130)

    signal.signal(signal.SIGINT, cleanup)
    signal.signal(signal.SIGTERM, cleanup)

    try:
        print("\n[1/4] Starting GLINS RRR ...")
        gici_proc = subprocess.Popen(
            ["rosrun", "gici_ros", "gici_ros_main", runtime_config],
            cwd=WORKSPACE,
        )

        time.sleep(2.0)
        if gici_proc.poll() is not None:
            raise RuntimeError("GLINS exited during startup.")

        print("\n[2/4] Preloading ephemerides ...")
        run([
            "rosbag", "play",
            ephemeris_bag,
            "--topics", "/gnss_reference/ephemerides",
            "--rate=1000",
            f"--duration={eph_preload_duration:.6f}",
        ])

        print("\n[3/4] Preloading reference antenna position ...")
        run([
            "rosbag", "play",
            reference_bag,
            "--topics", "/gnss_reference/antenna_position",
            "--duration=1",
        ])

        print("\n[4/4] Starting synchronized RRR playback ...")
        run([
            "rosbag", "play",
            sensor_bag,
            rover_bag,
            reference_bag,
            ephemeris_bag,
            f"--start={play_start:.6f}",
            f"--duration={play_duration:.6f}",
            "--topics",
            *TOPICS,
        ])

        print("\nPlayback finished.")
        print("GLINS remains running for output inspection.")
        print("Press Ctrl+C here to stop GLINS.")

        while gici_proc.poll() is None:
            time.sleep(0.5)

    finally:
        terminate_process(gici_proc, "GLINS")


if __name__ == "__main__":
    main()
