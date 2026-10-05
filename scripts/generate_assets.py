#!/usr/bin/env python3
"""Export the same generated URDF / SDF / configurations used by launch."""

import argparse
import sys
from pathlib import Path

root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(root / "src/robot_demo"))
from robot_demo.assets import generate  # noqa: E402 - local source bootstrap

parser = argparse.ArgumentParser()
parser.add_argument(
    "--robot", choices=["custom", "turtlebot3", "jackal"], default="custom"
)
parser.add_argument(
    "--world",
    choices=["obstacle_course", "warehouse", "outdoor"],
    default="obstacle_course",
)
parser.add_argument("--output", type=Path, default=root / ".runtime/exported")
parser.add_argument("--mission", type=Path)
args = parser.parse_args()
generate(
    root / "src/robot_demo", args.output.resolve(), args.robot, args.world, args.mission
)
print(args.output.resolve())
