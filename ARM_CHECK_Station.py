#!/usr/bin/python3
# ============================================================
# ELEC 311 -- ARM_CHECK_Station.py
#
# DIAGNOSTIC. Run it, read it, do not modify it.
# Run this at the START of every hardware session, all semester.
#
# It answers four questions:
#   1. Which port is the arm on?
#   2. Is the USB latency timer set to 1 ms?
#   3. Does every motor answer?
#   4. Is any motor sitting on a hardware error?
#
# Nothing moves. No torque is enabled.
#
# Usage:  python ARM_CHECK_Station.py
# ============================================================

import glob
import sys

from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler

from openarm import (
    ADDR_HARDWARE_ERROR,
    ADDR_PRESENT_POSITION,
    ALL_IDS,
    CENTER,
    DEFAULT_BAUD,
    GRIPPER_ID,
    PROTOCOL,
    TICK2DEG,
)
from openarm_port import get_port


def check_latency_timer() -> None:
    """The FTDI default is 16 ms, which is too slow for Week 6."""
    if sys.platform == "win32":
        print("  Latency timer : cannot be read on Windows.")
        print("                  Device Manager > Ports > your COM port >")
        print("                  Properties > Port Settings > Advanced > set to 1")
        return

    paths = glob.glob("/sys/bus/usb-serial/devices/*/latency_timer")
    if not paths:
        print("  Latency timer : not found (no USB serial device?)")
        return
    for p in paths:
        try:
            value = int(open(p).read().strip())
        except (OSError, ValueError):
            continue
        verdict = "OK" if value == 1 else "TOO SLOW -- tell the instructor"
        print(f"  Latency timer : {value} ms   {verdict}")


def main() -> None:
    print("\n" + "=" * 52)
    print("  ELEC 311 -- STATION CHECK")
    print("=" * 52 + "\n")

    port_name = get_port()
    port = PortHandler(port_name)
    packet = PacketHandler(PROTOCOL)

    if not port.openPort():
        print(f"  FAIL: could not open {port_name}. Is the U2D2 plugged in?")
        sys.exit(1)
    if not port.setBaudRate(DEFAULT_BAUD):
        print(f"  FAIL: could not set baud rate {DEFAULT_BAUD}.")
        sys.exit(1)

    print(f"  Port          : {port_name}")
    print(f"  Baud rate     : {DEFAULT_BAUD}")
    check_latency_timer()

    print("\n  ID   Name       Position     Error   Result")
    print("  " + "-" * 46)

    failures = 0
    for mid in ALL_IDS:
        name = "Gripper" if mid == GRIPPER_ID else f"Joint {mid - 10}"
        _, comm, _ = packet.ping(port, mid)

        if comm != COMM_SUCCESS:
            print(f"  {mid}   {name:9s}  {'--':>9s}     --    NO REPLY")
            failures += 1
            continue

        ticks, _, _ = packet.read4ByteTxRx(port, mid, ADDR_PRESENT_POSITION)
        err, _, _ = packet.read1ByteTxRx(port, mid, ADDR_HARDWARE_ERROR)
        deg = (ticks - CENTER) * TICK2DEG
        verdict = "OK" if err == 0 else "ERROR FLAG"
        if err != 0:
            failures += 1
        print(f"  {mid}   {name:9s}  {deg:8.2f} deg   {err:3d}    {verdict}")

    port.closePort()

    print("\n" + "=" * 52)
    if failures == 0:
        print("  ALL CHECKS PASSED. You may run ARM_COM_Jog.py\n")
        print("  Add this to team_log.txt:")
        print(f"    station: port={port_name}  all 5 motors OK")
    else:
        print(f"  {failures} PROBLEM(S) FOUND. Do NOT continue.")
        print("  Raise your hand. A motor that is silent now will fail")
        print("  silently in the middle of a motion later.")
    print("=" * 52 + "\n")
    print("=^..^=")


if __name__ == "__main__":
    main()
