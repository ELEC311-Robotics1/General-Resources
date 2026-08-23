#!/usr/bin/python3
# ============================================================
# OpenManipulatorX_Teach.py -- Record Waypoints by Hand-Guiding
#
# Records a pose sequence for ONE arm:
#   1. The arm parks, then all torque is released
#   2. Student lifts the arm by hand into each pose
#   3. Enter records the current joint angles
#   4. 'g' toggles the gripper AND records that pose -- a gripper
#      action is its own waypoint (same pose, new jaw state), so you
#      do not record the identical pose twice by hand
#   5. 'q' saves the sequence as JSON
#
# JSON, not .npy, so waypoints stay human-readable: a student can open
# the file and nudge a number by hand.
#
# ---------------------------- SAFETY ----------------------------
# The gripper is an actuator, and in this mode your hands are inside
# its workspace. It is left UNPOWERED except during a 'g' command,
# and 'g' asks you to confirm before it moves. Say no if a hand,
# a sleeve, or a cable is anywhere near the jaws.
#
# The arm parks BEFORE torque is released. Do not reorder this: cutting
# torque from a raised pose drops the whole arm.
# ----------------------------------------------------------------
#
# Typical pick sequence, as keystrokes:
#   Enter   above object    (open)
#   Enter   down to object  (open)
#   g       CLOSE           <- grab, pose auto-recorded
#   Enter   lift up         (closed)
#   Enter   above target    (closed)
#   g       OPEN            <- release, pose auto-recorded
#
# Usage: python OpenManipulatorX_Teach.py
# ============================================================

from __future__ import annotations

import json

from openarm import OpenArmX


def main() -> None:
    arm = OpenArmX()

    print("\n" + "=" * 52)
    print("  TEACH MODE -- Record Waypoints")
    print("=" * 52)
    print("\n  Enter   record current pose")
    print("  g       toggle gripper + record this pose (asks first)")
    print("  u       undo last waypoint")
    print("  q       save and quit\n")

    # Park, THEN release. teach_mode() enforces the order.
    arm.teach_mode()

    waypoints: list[dict] = []
    gripper_is_open = True

    try:
        while True:
            cmd = input(f"  [{len(waypoints)} recorded] > ").strip().lower()

            if cmd == "q":
                break

            elif cmd == "g":
                want = "closed" if gripper_is_open else "open"
                if input(f"    Hands clear? gripper -> {want} [y/N] ") != "y":
                    print("    Cancelled.")
                    continue
                gripper_is_open = not gripper_is_open
                arm.gripper_pulse("open" if gripper_is_open else "close")

                # A gripper action is its own waypoint: same pose, new
                # jaw state. Recording it here saves re-recording the
                # identical pose by hand on either side of the toggle.
                pos = arm.read_positions_deg()
                waypoints.append(
                    {
                        "q1": pos[11],
                        "q2": pos[12],
                        "q3": pos[13],
                        "q4": pos[14],
                        "gripper": "open" if gripper_is_open else "closed",
                    }
                )
                print(
                    f"    #{len(waypoints)}: gripper "
                    f"{'OPEN' if gripper_is_open else 'CLOSED'} (pose recorded)"
                )

            elif cmd == "u":
                if waypoints:
                    waypoints.pop()
                    print(f"    Removed. {len(waypoints)} left.")
                else:
                    print("    Nothing to undo.")

            elif cmd == "":
                pos = arm.read_positions_deg()
                wp = {
                    "q1": pos[11],
                    "q2": pos[12],
                    "q3": pos[13],
                    "q4": pos[14],
                    "gripper": "open" if gripper_is_open else "closed",
                }
                waypoints.append(wp)
                print(
                    f"    #{len(waypoints)}: "
                    f"q1={wp['q1']:.1f}  q2={wp['q2']:.1f}  "
                    f"q3={wp['q3']:.1f}  q4={wp['q4']:.1f}  "
                    f"gripper={wp['gripper']}"
                )

            else:
                print("    Unknown command.")

    except KeyboardInterrupt:
        pass

    if waypoints:
        name = input("\n  Filename (no extension): ").strip() or "waypoints"
        with open(f"{name}.json", "w") as fh:
            json.dump(waypoints, fh, indent=2)
        print(f"  Saved {len(waypoints)} waypoints to {name}.json\n")
        for i, wp in enumerate(waypoints, 1):
            print(
                f"    {i}. q=({wp['q1']:.0f}, {wp['q2']:.0f}, "
                f"{wp['q3']:.0f}, {wp['q4']:.0f})  gripper={wp['gripper']}"
            )
    else:
        print("\n  No waypoints recorded.")

    # Joints are already loose; close() parks and powers down cleanly.
    arm.close()
    print("=^..^=")


if __name__ == "__main__":
    main()