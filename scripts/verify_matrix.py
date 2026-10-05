#!/usr/bin/env python3
"""Run actual Gazebo missions and archive independent, machine-readable evidence."""

import argparse
import json
import os
import signal
import random
import uuid
import subprocess
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def stop(process):
    if process.poll() is None:
        os.kill(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=12)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()


def verify(robot, world, index, timeout, mission_file=None):
    destination = ROOT / "evidence/matrix"
    destination.mkdir(parents=True, exist_ok=True)
    suffix = "" if mission_file is None else "_" + Path(mission_file).stem
    report = destination / f"{robot}_{world}{suffix}.json"
    report.unlink(missing_ok=True)
    locks = ROOT / ".runtime/domains"
    locks.mkdir(parents=True, exist_ok=True)
    for domain in random.sample(list(range(20, 70)) + list(range(100, 150)), 100):
        domain_lock = locks / str(domain)
        try:
            fd = os.open(domain_lock, os.O_CREAT | os.O_EXCL | os.O_WRONLY)
            os.close(fd)
            break
        except FileExistsError:
            continue
    else:
        raise RuntimeError("No free test ROS domain")
    env = {
        **os.environ,
        "ROBOT_DEMO_SKIP_BUILD": "1",
        "ROBOT_DEMO_SOFTWARE_RENDERING": "1",
        "ROS_DOMAIN_ID": str(domain),
        "IGN_PARTITION": f"matrix_{index}_{uuid.uuid4().hex[:10]}",
    }
    started = time.monotonic()
    first_success = None
    data = {}
    last_read = None
    logpath = report.with_suffix(".log")
    with logpath.open("w") as log:
        process = subprocess.Popen(
            [
                "./run.sh",
                f"robot:={robot}",
                f"world:={world}",
                "gui:=false",
                "rviz:=false",
                f"report_file:={report}",
            ]
            + (
                [f"mission_file:={Path(mission_file).resolve()}"]
                if mission_file
                else []
            ),
            cwd=ROOT,
            env=env,
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        try:
            while time.monotonic() - started < timeout and process.poll() is None:
                if report.exists():
                    try:
                        data = json.loads(report.read_text())
                    except json.JSONDecodeError:
                        continue
                    last_read = time.monotonic() - (
                        time.time() - report.stat().st_mtime
                    )
                    if data.get("state") == "SUCCEEDED":
                        first_success = first_success or time.monotonic()
                        if time.monotonic() - first_success > 2:
                            break
                time.sleep(0.5)
            preset = (
                Path(mission_file)
                if mission_file
                else ROOT / "src/robot_demo/config" / f"{world}.json"
            )
            goal_config = json.loads(preset.read_text())
            expected_count = len(
                goal_config["waypoints_gps"]
                if goal_config["waypoint_mode"] == "gps"
                else goal_config["waypoints_world"]
            )
            required = (
                data.get("state") == "SUCCEEDED",
                data.get("waypoints_reached") == expected_count,
                len(data.get("ground_truth_waypoint_min_errors_m", []))
                == expected_count
                and all(
                    v < 0.25 for v in data.get("ground_truth_waypoint_min_errors_m", [])
                ),
                data.get("ground_truth_goal_error_m", 99) < 0.25,
                data.get("obstacle_contact_frames", -1) == 0,
                data.get("contact_sensor_messages", 0) > 100,
                data.get("ground_truth_samples", 0) > 100,
                data.get("map_observed_cells", 0) > 500,
                data.get("last_cmd_vel") == [0.0, 0.0],
                all(
                    data.get("sensor_messages", {}).get(k, 0) > 10
                    for k in ("pose", "gps", "lidar", "depth")
                ),
                last_read is not None and time.monotonic() - last_read < 3,
            )
            if mission_file:
                cfg = json.loads(Path(mission_file).read_text())
                count = len(
                    cfg["waypoints_gps"]
                    if cfg["waypoint_mode"] == "gps"
                    else cfg["waypoints_world"]
                )
                assert data.get("waypoints_reached") == count
                assert all(
                    v < 0.25
                    for v in data.get("ground_truth_waypoint_min_errors_m", [99.0])
                ), "A waypoint was not actually visited"
            passed = all(required)
            if passed:
                # Export the real sensor-built map before shutting the simulation down.
                command = 'source /opt/ros/humble/setup.bash; source install/setup.bash; ros2 service call /map/save std_srvs/srv/Trigger "{}"'
                subprocess.run(
                    ["bash", "-c", command],
                    cwd=ROOT,
                    env={
                        **env,
                        "HOME": str(ROOT / ".runtime"),
                        "FASTRTPS_DEFAULT_PROFILES_FILE": str(
                            ROOT / "src/robot_demo/config/fastdds_udp.xml"
                        ),
                    },
                    stdout=log,
                    stderr=log,
                    timeout=15,
                )
            data.update(
                validation_passed=passed,
                ros_domain_id=domain,
                validation_wall_s=round(time.monotonic() - started, 2),
                validation_checks=dict(
                    zip(
                        (
                            "mission_succeeded",
                            "all_waypoints_reached",
                            "all_waypoints_verified",
                            "truth_goal_within_25cm",
                            "zero_contacts",
                            "contact_stream_observed",
                            "truth_stream_observed",
                            "sensor_map_built",
                            "final_command_zero",
                            "all_sensors_observed",
                            "report_fresh",
                        ),
                        required,
                    )
                ),
            )
        finally:
            stop(process)
            domain_lock.unlink(missing_ok=True)
    report.write_text(json.dumps(data, indent=2) + "\n")
    print(
        f"{robot:11} {world:16} {'PASS' if passed else 'FAIL'} {data.get('state')} error={data.get('ground_truth_goal_error_m')} clearance={data.get('ground_truth_minimum_clearance_m')}",
        flush=True,
    )
    return data


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot", choices=["custom", "turtlebot3", "jackal"])
    parser.add_argument("--world", choices=["obstacle_course", "warehouse", "outdoor"])
    parser.add_argument("--timeout", type=float, default=240)
    parser.add_argument("--mission-file", type=Path)
    args = parser.parse_args()
    robots = [args.robot] if args.robot else ["custom", "turtlebot3", "jackal"]
    worlds = [args.world] if args.world else ["obstacle_course", "warehouse", "outdoor"]
    results = []
    for robot in robots:
        for world in worlds:
            results.append(
                verify(robot, world, len(results), args.timeout, args.mission_file)
            )
    (
        ROOT
        / (
            "evidence/mission_variants.json"
            if args.mission_file
            else "evidence/matrix_summary.json"
        )
    ).write_text(json.dumps(results, indent=2) + "\n")
    return 0 if all(r["validation_passed"] for r in results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
