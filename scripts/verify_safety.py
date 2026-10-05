#!/usr/bin/env python3
"""Inject real process / clock faults and assert the independent guard stops."""

import json
import os
import re
import signal
import subprocess
import time
import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_srvs.srv import SetBool
from verify_matrix import ROOT, stop


def experiment(fault, index):
    directory = ROOT / "evidence/safety"
    directory.mkdir(parents=True, exist_ok=True)
    env = {
        **os.environ,
        "ROS_DOMAIN_ID": str(75 + index),
        "IGN_PARTITION": f"safety_{index}_{os.getpid()}",
        "ROBOT_DEMO_SKIP_BUILD": "1",
        "ROBOT_DEMO_SOFTWARE_RENDERING": "1",
    }
    os.environ["ROS_DOMAIN_ID"] = env["ROS_DOMAIN_ID"]
    os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"] = str(
        ROOT / "src/robot_demo/config/fastdds_udp.xml"
    )
    logpath = directory / f"{fault}.log"
    report = directory / f"{fault}_mission.json"
    os.environ["ROS_LOG_DIR"] = str(ROOT / ".runtime/log")
    observations = []
    state = {}
    with logpath.open("w") as log:
        process = subprocess.Popen(
            ["./run.sh", "gui:=false", "rviz:=false", f"report_file:={report}"],
            cwd=ROOT,
            env=env,
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        rclpy.init()
        node = rclpy.create_node("fault_verifier")
        node.create_subscription(
            Twist,
            "/cmd_vel",
            lambda m: observations.append((time.monotonic(), m.linear.x, m.angular.z)),
            100,
        )
        node.create_subscription(
            String, "/safety/status", lambda m: state.update(json.loads(m.data)), 10
        )

        def spin(seconds):
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.05)

        try:
            deadline = time.monotonic() + 30
            while time.monotonic() < deadline:
                spin(0.2)
                if observations and observations[-1][1] > 0.15:
                    break
            assert observations and observations[-1][1] > 0.15, (
                "Robot never began moving"
            )
            if fault == "pause_service":
                client = node.create_client(SetBool, "/mission/enable")
                assert client.wait_for_service(timeout_sec=5)
                request = SetBool.Request()
                request.data = False
                future = client.call_async(request)
                rclpy.spin_until_future_complete(node, future, timeout_sec=5)
                assert future.done() and future.result().success
            elif fault == "clock_stall":
                subprocess.run(
                    [
                        "ign",
                        "service",
                        "-s",
                        "/world/navigation/control",
                        "--reqtype",
                        "ignition.msgs.WorldControl",
                        "--reptype",
                        "ignition.msgs.Boolean",
                        "--timeout",
                        "2000",
                        "--req",
                        "pause: true",
                    ],
                    env=env,
                    check=True,
                    stdout=log,
                    stderr=log,
                )
            else:
                content = logpath.read_text()
                pattern = (
                    r"\[navigator-\d+\]: process started with pid \[(\d+)\]"
                    if fault == "controller_exit"
                    else r"\[parameter_bridge-6\]: process started with pid \[(\d+)\]"
                )
                match = re.search(pattern, content)
                assert match, "Target process not found"
                os.kill(int(match.group(1)), signal.SIGKILL)
            injected = time.monotonic()
            spin(3.5)
            trace = [
                (round(t - injected, 3), v, w)
                for t, v, w in observations
                if t >= injected
            ]
            assert trace, "Guard stopped publishing entirely"
            zeros = [t for t, v, w in trace if abs(v) < 1e-6 and abs(w) < 1e-6]
            assert zeros, "No zero command observed"
            deadline = (
                0.75
                if fault in ("clock_stall", "controller_exit", "pause_service")
                else 2.5
            )
            assert min(zeros) < deadline, f"Stop exceeded {deadline}s"
            assert all(
                abs(v) < 1e-6 and abs(w) < 1e-6 for t, v, w in trace if t > 2.7
            ), "Robot resumed during fault"
            result = dict(
                fault=fault,
                passed=True,
                first_zero_after_fault_s=min(zeros),
                last_guard_state=state,
                commands_after_fault=trace,
            )
        finally:
            node.destroy_node()
            rclpy.shutdown()
            stop(process)
    (directory / f"{fault}.json").write_text(json.dumps(result, indent=2) + "\n")
    print(f"{fault}: PASS, zero in {min(zeros):.3f}s", flush=True)
    return result


def main():
    results = [
        experiment(fault, i)
        for i, fault in enumerate(
            ("pause_service", "clock_stall", "controller_exit", "gps_loss")
        )
    ]
    (ROOT / "evidence/safety_summary.json").write_text(
        json.dumps(results, indent=2) + "\n"
    )


if __name__ == "__main__":
    main()
