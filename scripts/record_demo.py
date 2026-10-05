#!/usr/bin/env python3
"""Record a private X11 display without MIT-SHM, with live report captions.
Requires python3-xlib, python3-pil and ffmpeg. Run the demo with autostart:=false.
"""

import os
import argparse
import json
import subprocess
import time
from pathlib import Path
from Xlib import X, display
from PIL import Image, ImageDraw, ImageFont

parser = argparse.ArgumentParser()
parser.add_argument("--output", default="evidence/demo.mp4")
parser.add_argument("--report", default="evidence/mission_report.json")
parser.add_argument("--start-mission", action="store_true")
parser.add_argument("--max-seconds", type=float, default=180.0)
args = parser.parse_args()
os.environ.setdefault(
    "FASTRTPS_DEFAULT_PROFILES_FILE",
    str(Path(__file__).resolve().parents[1] / "src/robot_demo/config/fastdds_udp.xml"),
)
display_connection = display.Display()
root = display_connection.screen().root
width, height = 2400, 1080
geometry = root.get_geometry()
if geometry.width < width or geometry.height < height:
    raise RuntimeError("Recording requires an X11 display of at least 2400x1080 pixels")
for window in root.query_tree().children:
    name = window.get_wm_name() or ""
    if name == "Gazebo":
        window.configure(x=0, y=0, width=1200, height=1000)
    if "RViz" in name:
        window.configure(x=1200, y=0, width=1200, height=1000)
        window.raise_window()
display_connection.sync()
font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 24)
small = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 21)
Path(args.output).parent.mkdir(parents=True, exist_ok=True)
fps = 12
encoder = subprocess.Popen(
    [
        "ffmpeg",
        "-y",
        "-loglevel",
        "error",
        "-f",
        "rawvideo",
        "-pixel_format",
        "rgb24",
        "-video_size",
        f"{width}x{height}",
        "-framerate",
        str(fps),
        "-i",
        "-",
        "-an",
        "-c:v",
        "libx264",
        "-preset",
        "veryfast",
        "-crf",
        "23",
        "-pix_fmt",
        "yuv420p",
        "-movflags",
        "+faststart",
        args.output,
    ],
    stdin=subprocess.PIPE,
)
started = time.monotonic()
success_time = None
enabled = False
frames = 0
try:
    while time.monotonic() - started < args.max_seconds:
        frame_started = time.monotonic()
        elapsed = frame_started - started
        if args.start_mission and elapsed > 5 and not enabled:
            # No ros2 daemon dependency; this client communicates directly with the node.
            import rclpy
            from std_srvs.srv import SetBool

            rclpy.init()
            node = rclpy.create_node("recording_mission_start")
            client = node.create_client(SetBool, "/mission/enable")
            if not client.wait_for_service(timeout_sec=5):
                raise RuntimeError("Mission service unavailable")
            request = SetBool.Request()
            request.data = True
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=5)
            if not future.done() or not future.result().success:
                raise RuntimeError("Mission start failed")
            node.destroy_node()
            rclpy.shutdown()
            enabled = True
        data = {}
        try:
            data = json.loads(Path(args.report).read_text())
        except (FileNotFoundError, json.JSONDecodeError):
            pass
        raw = root.get_image(0, 0, width, 1000, X.ZPixmap, 0xFFFFFFFF)
        frame = Image.new("RGB", (width, height), (21, 29, 43))
        frame.paste(
            Image.frombytes("RGB", (width, 1000), raw.data, "raw", "BGRX"), (0, 0)
        )
        draw = ImageDraw.Draw(frame)
        draw.text(
            (24, 1007),
            f"{data.get('robot', 'robot').upper()}  |  {data.get('world', 'world').replace('_', ' ').upper()}  |  HUMBLE + FORTRESS  |  GPS + sensor-built map",
            font=font,
            fill="white",
        )
        state = data.get("state", "INITIALIZING")
        distance = data.get("goal_error_m", 0)
        contact = data.get("obstacle_contact_frames", 0)
        clearance = data.get("ground_truth_minimum_clearance_m")
        line = f"State: {state}     Goal error: {distance:.2f} m     Obstacle contacts: {contact}"
        line += f"     Observed map cells: {data.get('map_observed_cells', 0)}"
        if clearance is not None:
            line += f"     Minimum clearance: {clearance:.2f} m"
        draw.text(
            (24, 1044),
            line,
            font=small,
            fill=(99, 225, 162) if state == "SUCCEEDED" else (205, 216, 232),
        )
        encoder.stdin.write(frame.tobytes())
        frames += 1
        if state == "SUCCEEDED" and success_time is None:
            success_time = time.monotonic()
        if (
            success_time is not None
            and time.monotonic() - success_time > 8
            and frames / fps >= 60
        ):
            break
        time.sleep(max(0, 1 / fps - (time.monotonic() - frame_started)))
finally:
    encoder.stdin.close()
    encoder.wait()
    display_connection.close()
    Path(args.output).with_suffix(".json").write_text(
        json.dumps(
            {
                "capture": "actual Gazebo and RViz X11 windows",
                "frames": frames,
                "fps": fps,
                "video_seconds": frames / fps,
                "wall_seconds": time.monotonic() - started,
                "speed_note": "Captured frames are encoded at 12 fps; capture throughput varies, so wall/video duration differ. This is actual GUI footage with no fabricated sensor data.",
            },
            indent=2,
        )
        + "\n"
    )
print(args.output, frames / fps, "seconds")
