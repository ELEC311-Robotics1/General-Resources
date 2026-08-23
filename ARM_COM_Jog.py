#!/usr/bin/python3
# ============================================================
# ELEC 311 -- ARM_COM_Jog.py
#
# Interactive control of the OpenMANIPULATOR-X.
# Type a command, watch the arm. That is the whole program.
#
# SAFETY
#   Nobody works alone. One person types, one watches the arm.
#   Hands clear of the gripper.
#   To stop: type  q  and WAIT. Never Ctrl+C during a motion.
#
# Usage:  python ARM_COM_Jog.py
# ============================================================

from openarm import OpenArmX

HELP = """
  s              status: every position and current
  h              home   (the pistol pose)
  p              park   (lays the arm down flat)

  j 12 60        joint 12 to +60 degrees
  a 0 -15 8 0    all four joints at once (11 12 13 14)
  v 5           profile velocity: lower is slower

  go / gc        gripper open / close
  ?              show this list again
  q              park, power down, and quit
"""


def main() -> None:
    arm = OpenArmX()
    arm.home()

    print("\n" + "=" * 52)
    print("  ELEC 311 -- Jog the Arm")
    print("=" * 52)
    print(HELP)
    print("  Predict which way it will move BEFORE you press Enter.\n")

    while True:
        try:
            cmd = input("> ").strip().split()
        except EOFError:
            break
        if not cmd:
            continue
        c = cmd[0].lower()

        try:
            if c == "q":
                break
            elif c == "?":
                print(HELP)
            elif c == "s":
                arm.status()
            elif c == "h":
                arm.home()
            elif c == "p":
                arm.park()
            elif c == "j" and len(cmd) == 3:
                arm.set_position_deg(int(cmd[1]), float(cmd[2]))
                print(f"  joint {cmd[1]} -> {cmd[2]} deg")
            elif c == "a" and len(cmd) == 5:
                angles = [float(v) for v in cmd[1:5]]
                arm.set_all_deg(angles)
                print(f"  joints -> {angles}")
            elif c == "v" and len(cmd) == 2:
                arm.set_profile(vel=int(cmd[1]))
                print(f"  profile velocity: {cmd[1]}")
            elif c == "go":
                arm.gripper_open()
            elif c == "gc":
                arm.gripper_close()
            else:
                print("  Unknown command. Type  ?  for the list.")

        except ValueError:
            print("  That is not a number. Try:  j 12 60")

    # close() parks the arm and powers it down safely.
    arm.close()
    print("=^..^=")


if __name__ == "__main__":
    main()
