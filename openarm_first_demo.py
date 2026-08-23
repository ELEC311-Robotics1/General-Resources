#!/usr/bin/python3
# ============================================================
# ELEC 311 -- Lecture 1 Demo: "It Doesn't Know Where Anything Is"
#
#
# CHOREOGRAPHY
#   1. Put an object on the table. Play the recorded sequence.
#      The arm picks it up and moves it. Let them be impressed.
#   2. Put the object back. Move it TWO INCHES to the side.
#      Say nothing about having moved it.
#   3. Play the same sequence again. The arm closes on empty air
#      with total confidence and completes the whole routine.
#   4. "What does this robot know about that object?"
#      Wait. Let someone say it: nothing.
#      "It knows four joint angles I gave it. That is all it has.
#       Weeks 3 to 6 are how a robot gets told where things ARE."
#   5. Do NOT fix it. The failure is the lecture. W7 is where they
#      build the working version themselves.
#
#   python OpenManipulatorX_Teach.py      -> save as demo_pick.json
#   Mark the object's "correct" spot with tape so you can reset fast.
#
# Usage:
#   python openarm_first_demo.py                  # plays demo_pick.json
#   python openarm_first_demo.py my_seq.json
# ============================================================

from __future__ import annotations

import json
import sys
import time

from openarm import OpenArmX

DEFAULT_FILE = "demo_pick.json"

# Slow: see each move land.
VEL = 30
ACC = 15


def load(path: str) -> list[dict]:
    try:
        with open(path) as fh:
            return json.load(fh)
    except FileNotFoundError:
        print(f"\n  No file '{path}'.")
        print("  Record one first:  python OpenManipulatorX_Teach.py\n")
        sys.exit(1)


def run_sequence(arm: OpenArmX, waypoints: list[dict]) -> None:
    """Drive the arm through every recorded pose, in order."""
    for i, wp in enumerate(waypoints, 1):
        arm.set_position_deg(11, wp["q1"])
        arm.set_position_deg(12, wp["q2"])
        arm.set_position_deg(13, wp["q3"])
        arm.set_position_deg(14, wp["q4"])
        arm.wait_until_done()

        # Gripper acts on arrival, not on the way. Wait for it to
        # actually finish: a fixed sleep either lags or cuts the jaws
        # off mid-travel. The short sleep first lets the motor start
        # moving, otherwise wait_until_done() sees a still gripper and
        # returns immediately.
        arm.gripper_open() if wp["gripper"] == "open" else arm.gripper_close()
        time.sleep(0.15)
        arm.wait_until_done()

        print(f"    {i}/{len(waypoints)}")


def main() -> None:
    path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_FILE
    waypoints = load(path)

    arm = OpenArmX()
    arm.home()
    arm.set_profile(vel=VEL, acc=ACC)
    time.sleep(0.5)

    print(f"\n  Loaded {len(waypoints)} waypoints from {path}.")
    print("  Enter = play    q = quit\n")

    try:
        while True:
            if input("> ").strip().lower() == "q":
                break

            run_sequence(arm, waypoints)

            arm.home()
            arm.set_profile(vel=VEL, acc=ACC)

    except KeyboardInterrupt:
        print("\n  Interrupted.")

    finally:
        arm.close()
        print("=^..^=")


if __name__ == "__main__":
    main()