#!/usr/bin/python3
# ============================================================
# ELEC 311 -- OpenMANIPULATOR-X Library (Position Mode)
# Dynamixel XM430-W350 x5 via U2D2
#
# Motor IDs: 11-14 (joints), 15 (gripper)
# Protocol 2.0, POSITION MODE ONLY.
#   Velocity / PWM / current modes are deferred to ELEC 426.
#   They are deliberately absent from this file.
#
# References:
#   XM430-W350 control table:
#     https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/#control-table
#   Dynamixel SDK (Python):
#     https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/
# ============================================================

from __future__ import annotations

import glob
import sys
import time

import numpy as np
from dynamixel_sdk import (
    COMM_SUCCESS,
    DXL_HIBYTE,
    DXL_HIWORD,
    DXL_LOBYTE,
    DXL_LOWORD,
    GroupSyncRead,
    GroupSyncWrite,
    PacketHandler,
    PortHandler,
)

from openarm_port import get_port

# ============================================================
# Configuration
# ============================================================
DEFAULT_BAUD = 1_000_000
PROTOCOL = 2.0

JOINT_IDS = [11, 12, 13, 14]
GRIPPER_ID = 15
ALL_IDS = JOINT_IDS + [GRIPPER_ID]

# === XM430-W350 control table (position-mode subset) ===
ADDR_OPERATING_MODE = 11
ADDR_TORQUE_ENABLE = 64
ADDR_LED = 65
ADDR_PROFILE_ACCEL = 108
ADDR_PROFILE_VELOCITY = 112
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_CURRENT = 126
ADDR_PRESENT_VELOCITY = 128
ADDR_PRESENT_POSITION = 132
ADDR_HARDWARE_ERROR = 70

MODE_POSITION = 3

# === Unit conversions ===
CENTER = 2048  # tick value at 0 deg
TICK2DEG = 360.0 / 4096.0
DEG2TICK = 4096.0 / 360.0
TICK2RAD = 2.0 * np.pi / 4096.0
RAD2TICK = 4096.0 / (2.0 * np.pi)


# ============================================================
# Geometry and calibration -- SINGLE SOURCE OF TRUTH
#
# Every kinematics script imports these. Do not redefine them
# locally; that is how the three old library copies drifted.
# ============================================================

# Link dimensions (mm), from the DH table:
#   i | a_i | d_i | alpha_i | theta_i
#   1 |  0  | D1  |  pi/2   |   q1
#   2 | L1  |  0  |    0    |   q2
#   3 | L2  |  0  |    0    |   q3
#   4 | L3  |  0  |    0    |   q4
D1 = 77.0
L1 = 130.0
L2 = 124.0
L3 = 126.0

# Encoder-zero to DH-zero offsets (rad).
#
# These are NOT DH parameters. The DH table above has theta_i = q_i,
# with no offsets in it. These constants exist because the joint-2 and
# joint-3 motor shafts are not perfectly aligned with their links, so
# the encoder's zero is not the model's zero.
#
# MEASURED BY HAND with a ruler (2026-06). Verified to apply to all
# three arms. Re-measure if an arm is disassembled or a horn is reseated.
ALPHA = np.radians(105.0)  # joint 2 offset
BETA = np.radians(8.0)  # additional joint 3 offset

# Pistol pose -- the ELEC 311 home configuration, in SERVO degrees.
# This is what students meet on day one. Not the DH zero: the DH zero
# (arm fully extended horizontal) is mechanically unreachable.
HOME_DEG = [0.0, -15.0, 8.0, 0.0]

# Park pose reference (servo deg, joints 12/13/14 only). safe_off()
# compares against this to decide whether parking is still needed.
QPARK = np.array([100.0, -105.0, -10.0])
PARK_TOLERANCE_DEG = 20.0

GRIPPER_OPEN_TICKS = 2500
GRIPPER_CLOSE_TICKS = 1400
# Softer close for teach mode, where fingers are near the jaws. A goal
# position has no force limit: if the gripper meets an obstruction it
# keeps pushing until the overload shutdown trips. Closing less far
# reduces both the pinch and the number of gripper_reset() calls.
# (Proper fix is current-based position mode -- ELEC 426.)
GRIPPER_CLOSE_SOFT_TICKS = 1700

# Soft joint limits in SERVO degrees, applied to every write.
#
# WARNING: these are conservative starting values, NOT yet verified
# against the arm's mechanical stops. Widen or tighten after the W3
# calibration session. They are deliberately wide enough to contain the
# full park sequence (q2 -> 100, q3 -> -110, q4 -> -40), so clamping
# never fights the rest-position routine.
JOINT_LIMITS_DEG: dict[int, tuple[float, float]] = {
    11: (-150.0, 150.0),
    12: (-115.0, 110.0),
    13: (-120.0, 90.0),
    14: (-110.0, 115.0),
    15: (-80.0, 60.0),
}


# ============================================================
# Helpers
# ============================================================
def _to_signed32(val: int) -> int:
    """Dynamixel returns unsigned; reinterpret as signed 32-bit."""
    return val - 4294967296 if val > 2147483647 else val


def _check_latency_timer() -> None:
    """
    Warn if the USB latency timer is not 1 ms.

    The FTDI default is 16 ms, which caps the control loop far below the
    50 Hz that W6 trajectory tracking needs. On Linux this resets every
    time the U2D2 is replugged -- a udev rule is the permanent fix.
    Windows: Device Manager -> Port Settings -> Advanced -> Latency Timer.
    """
    if sys.platform == "win32":
        return
    for path in glob.glob("/sys/bus/usb-serial/devices/*/latency_timer"):
        try:
            with open(path) as fh:
                value = int(fh.read().strip())
            if value != 1:
                print(
                    f"  WARNING: latency_timer = {value} ms (want 1 ms).\n"
                    f"           Fix: echo 1 | sudo tee {path}"
                )
        except (OSError, ValueError):
            pass  # not fatal; just skip the check


class OpenArmX:
    """
    Position-mode driver for the OpenMANIPULATOR-X.

    All writes are clamped to JOINT_LIMITS_DEG. All reads are checked
    for communication failures and fall back to the last good value --
    an unchecked dropped packet reads as tick 0, which converts to
    -180 deg and will destabilize any closed-loop script.
    """

    def __init__(self, port: str | None = None, baud: int = DEFAULT_BAUD) -> None:
        if port is None:
            port = get_port()
        self.port_handler = PortHandler(port)
        self.packet_handler = PacketHandler(PROTOCOL)

        if not self.port_handler.openPort():
            raise RuntimeError(f"Failed to open {port}")
        if not self.port_handler.setBaudRate(baud):
            raise RuntimeError(f"Failed to set baud rate {baud}")

        _check_latency_timer()

        self.sync_write_pos = GroupSyncWrite(
            self.port_handler, self.packet_handler, ADDR_GOAL_POSITION, 4
        )
        self.sync_read_pos = GroupSyncRead(
            self.port_handler, self.packet_handler, ADDR_PRESENT_POSITION, 4
        )
        for mid in ALL_IDS:
            self.sync_read_pos.addParam(mid)

        # Last known-good encoder reading, used when a packet drops.
        self._last_positions: dict[int, int] = {mid: CENTER for mid in ALL_IDS}
        self.dropped_reads = 0  # cumulative; check after a run

        self.enforce_limits = True

        self.reboot_all()
        print(f"Connected: {port} at {baud}")

    def close(self) -> None:
        self.safe_off()
        self.port_handler.closePort()
        print("Port closed.")

    # ---------------------------------------------------------
    # Low-level
    # ---------------------------------------------------------
    def write1(self, mid: int, addr: int, val: int) -> None:
        self.packet_handler.write1ByteTxRx(self.port_handler, mid, addr, val)

    def write4(self, mid: int, addr: int, val: int) -> None:
        self.packet_handler.write4ByteTxRx(
            self.port_handler, mid, addr, int(val) & 0xFFFFFFFF
        )

    def read1(self, mid: int, addr: int) -> int:
        val, _, _ = self.packet_handler.read1ByteTxRx(self.port_handler, mid, addr)
        return val

    def read2(self, mid: int, addr: int) -> int:
        val, _, _ = self.packet_handler.read2ByteTxRx(self.port_handler, mid, addr)
        return val - 65536 if val > 32767 else val

    def read4(self, mid: int, addr: int) -> int:
        val, _, _ = self.packet_handler.read4ByteTxRx(self.port_handler, mid, addr)
        return _to_signed32(val)

    # ---------------------------------------------------------
    # Reboot -- clears hardware errors, resets to position mode
    # ---------------------------------------------------------
    def reboot_all(self) -> None:
        for mid in ALL_IDS:
            self.packet_handler.reboot(self.port_handler, mid)
        time.sleep(1.0)
        for mid in ALL_IDS:
            self.write1(mid, ADDR_TORQUE_ENABLE, 0)
            self.write1(mid, ADDR_OPERATING_MODE, MODE_POSITION)

    # ---------------------------------------------------------
    # Torque
    # ---------------------------------------------------------
    def torque_on(self, ids: list[int] | None = None) -> None:
        for mid in ALL_IDS if ids is None else ids:
            self.write1(mid, ADDR_TORQUE_ENABLE, 1)

    def torque_off(self, ids: list[int] | None = None) -> None:
        for mid in ALL_IDS if ids is None else ids:
            self.write1(mid, ADDR_TORQUE_ENABLE, 0)

    def led(self, on: bool, ids: list[int] | None = None) -> None:
        for mid in ALL_IDS if ids is None else ids:
            self.write1(mid, ADDR_LED, 1 if on else 0)

    # ---------------------------------------------------------
    # Profile
    # ---------------------------------------------------------
    def set_profile(
        self, vel: int = 100, acc: int = 50, ids: list[int] | None = None
    ) -> None:
        for mid in ALL_IDS if ids is None else ids:
            self.write4(mid, ADDR_PROFILE_VELOCITY, vel)
            self.write4(mid, ADDR_PROFILE_ACCEL, acc)

    # ---------------------------------------------------------
    # Wait until motion completes
    # ---------------------------------------------------------
    def wait_until_done(self, tolerance: int = 5, timeout: float = 10.0) -> bool:
        start = time.time()
        while time.time() - start < timeout:
            vels = self.read_all_velocities()
            if all(abs(v) < tolerance for v in vels.values()):
                time.sleep(0.3)
                return True
            time.sleep(0.05)
        print("  (motion timeout)")
        return False

    # ---------------------------------------------------------
    # Read
    # ---------------------------------------------------------
    def read_position(self, mid: int) -> int:
        return self.read4(mid, ADDR_PRESENT_POSITION)

    def read_all_positions(self) -> dict[int, int]:
        """
        SyncRead every encoder. On any communication failure, return the
        last good reading rather than a corrupt one.
        """
        if self.sync_read_pos.txRxPacket() != COMM_SUCCESS:
            self.dropped_reads += 1
            return self._last_positions

        positions: dict[int, int] = {}
        for mid in ALL_IDS:
            if not self.sync_read_pos.isAvailable(mid, ADDR_PRESENT_POSITION, 4):
                self.dropped_reads += 1
                return self._last_positions
            positions[mid] = _to_signed32(
                self.sync_read_pos.getData(mid, ADDR_PRESENT_POSITION, 4)
            )

        self._last_positions = positions
        return positions

    def read_positions_deg(self) -> dict[int, float]:
        raw = self.read_all_positions()
        return {mid: (val - CENTER) * TICK2DEG for mid, val in raw.items()}

    def read_positions_rad(self) -> dict[int, float]:
        raw = self.read_all_positions()
        return {mid: (val - CENTER) * TICK2RAD for mid, val in raw.items()}

    def read_velocity(self, mid: int) -> int:
        return self.read4(mid, ADDR_PRESENT_VELOCITY)

    def read_all_velocities(self) -> dict[int, int]:
        return {mid: self.read_velocity(mid) for mid in ALL_IDS}

    def read_current(self, mid: int) -> int:
        return self.read2(mid, ADDR_PRESENT_CURRENT)

    def read_all_currents(self) -> dict[int, int]:
        return {mid: self.read_current(mid) for mid in ALL_IDS}

    # ---------------------------------------------------------
    # Soft limits
    # ---------------------------------------------------------
    def _clamp_ticks(self, mid: int, ticks: int) -> int:
        """Clamp a goal position to the soft limits, warning if it bites."""
        if not self.enforce_limits or mid not in JOINT_LIMITS_DEG:
            return ticks
        lo_deg, hi_deg = JOINT_LIMITS_DEG[mid]
        lo = round(lo_deg * DEG2TICK) + CENTER
        hi = round(hi_deg * DEG2TICK) + CENTER
        clamped = int(np.clip(ticks, lo, hi))
        if clamped != ticks:
            print(
                f"  LIMIT: ID {mid} requested {(ticks - CENTER) * TICK2DEG:.1f} deg, "
                f"clamped to {(clamped - CENTER) * TICK2DEG:.1f} deg"
            )
        return clamped

    # ---------------------------------------------------------
    # Position commands
    # ---------------------------------------------------------
    def set_position(self, mid: int, ticks: int) -> None:
        """Send a goal position in ticks. Every write funnels through here."""
        self.write4(mid, ADDR_GOAL_POSITION, self._clamp_ticks(mid, int(ticks)))

    def set_position_deg(self, mid: int, deg: float) -> None:
        # round(), not int(): int() truncates toward zero, which makes
        # rounding asymmetric about the 2048 center.
        self.set_position(mid, round(deg * DEG2TICK) + CENTER)

    def set_all_positions(self, ticks_list: list[int]) -> None:
        """SyncWrite goal positions to joints 11-14 in one packet."""
        self.sync_write_pos.clearParam()
        for mid, val in zip(JOINT_IDS, ticks_list):
            v = self._clamp_ticks(mid, int(val))
            self.sync_write_pos.addParam(
                mid,
                [
                    DXL_LOBYTE(DXL_LOWORD(v)),
                    DXL_HIBYTE(DXL_LOWORD(v)),
                    DXL_LOBYTE(DXL_HIWORD(v)),
                    DXL_HIBYTE(DXL_HIWORD(v)),
                ],
            )
        self.sync_write_pos.txPacket()

    def set_all_deg(self, angles: list[float]) -> None:
        self.set_all_positions([round(a * DEG2TICK) + CENTER for a in angles])

    # ---------------------------------------------------------
    # Gripper
    # ---------------------------------------------------------
    def gripper_open(self, pos: int = GRIPPER_OPEN_TICKS) -> None:
        self.set_position(GRIPPER_ID, pos)

    def gripper_close(self, pos: int = GRIPPER_CLOSE_TICKS) -> None:
        self.set_position(GRIPPER_ID, pos)

    def gripper_set(self, pos: int) -> None:
        """Send an exact gripper position (ticks)."""
        self.set_position(GRIPPER_ID, int(pos))

    def gripper_reset(self) -> None:
        """Recover the gripper from an overload shutdown."""
        err = self.read1(GRIPPER_ID, ADDR_HARDWARE_ERROR)
        print(f"Gripper error register: {err}")

        self.packet_handler.reboot(self.port_handler, GRIPPER_ID)
        time.sleep(1.5)  # must wait -- 0.5 s is not enough

        self.write1(GRIPPER_ID, ADDR_TORQUE_ENABLE, 0)
        self.write1(GRIPPER_ID, ADDR_OPERATING_MODE, MODE_POSITION)
        self.write4(GRIPPER_ID, ADDR_GOAL_POSITION, CENTER)
        time.sleep(0.1)
        self.write1(GRIPPER_ID, ADDR_TORQUE_ENABLE, 1)

        # Reboot drops the motor from the SyncRead group; re-add it.
        self.sync_read_pos.addParam(GRIPPER_ID)
        print("Gripper recovered.")

    # ---------------------------------------------------------
    # Hand-guiding support
    # ---------------------------------------------------------
    def gripper_pulse(self, action: str, dwell: float = 0.6) -> None:
        """
        Move the gripper, then immediately release its torque.

        In teach mode a student's hands are inside the workspace. A
        permanently-powered gripper is a live actuator next to fingers,
        so it stays dead except for the moment it is actually moving.
        """
        self.torque_on([GRIPPER_ID])
        if action == "open":
            self.gripper_open()
        else:
            self.gripper_close(GRIPPER_CLOSE_SOFT_TICKS)
        time.sleep(dwell)
        self.torque_off([GRIPPER_ID])

    def is_parked(self) -> bool:
        """True if joints 12-14 are already at the rest pose."""
        aux = self.read_positions_deg()
        q = np.array([aux[12], aux[13], aux[14]])
        return bool(np.linalg.norm(q - QPARK) < PARK_TOLERANCE_DEG)

    def teach_mode(self) -> None:
        """
        Release the joints for hand-guiding, safely.

        ORDER MATTERS. Cutting torque from an extended pose drops the
        arm: joints 2 and 3 are holding the whole structure against
        gravity, and nothing catches it. So park first -- the arm is
        already laid down and low -- and only then release. The student
        lifts it into the work area from a resting pose.

        If the arm is already at rest, parking again is pointless
        motion, so that case is skipped.

        The gripper is released too; use gripper_pulse() to move it.
        """
        if not self.is_parked():
            self.park()
        self.torque_off()
        print("Teach mode: all torque off. Lift the arm by hand.")

    # ---------------------------------------------------------
    # Park -- lay the arm down flat and low, like a snake
    #
    # The waypoints are hand-tuned so the arm never swings through
    # itself or the table on the way down. Do not "simplify" this into
    # a single set_all_deg().
    # ---------------------------------------------------------
    def park(self) -> None:
        self.torque_on()
        self.set_profile(vel=10, acc=5)
        time.sleep(0.05)

        for q12, q13, q14 in (
            (None, -90, -40),  # elbow and wrist clear first
            (35, -110, -20),
            (80, -110, -10),
            (100, -105, -10),  # final rest pose (QPARK)
        ):
            if q12 is None:
                self.set_position_deg(11, 0)
            else:
                self.set_position_deg(12, q12)
            self.set_position_deg(13, q13)
            self.set_position_deg(14, q14)
            self.wait_until_done()

        self.gripper_close()
        self.wait_until_done()
        print("Parked.")

    # ---------------------------------------------------------
    # Home -- the pistol pose
    # ---------------------------------------------------------
    def home(self, vel: int = 40) -> None:
        self.torque_on()
        self.set_profile(vel=vel, acc=20)
        time.sleep(0.05)
        self.set_all_deg(HOME_DEG)
        self.gripper_open()
        self.wait_until_done()
        print("Home.")

    # ---------------------------------------------------------
    # Safe torque off
    #
    # Parking first matters: torque-off from an extended pose drops the
    # arm. But if it is ALREADY at rest, parking again is pointless
    # motion, so that case is skipped -- hand-calibrated tolerance.
    # ---------------------------------------------------------
    def safe_off(self) -> None:
        if self.is_parked():
            print("Already near park position. Torque OFF.")
            self.torque_off()
            return
        self.park()
        self.torque_off()
        print("Torque OFF.")

    # ---------------------------------------------------------
    # Status
    # ---------------------------------------------------------
    def status(self) -> None:
        pos = self.read_positions_deg()
        cur = self.read_all_currents()

        print("\n  ID   Name       Pos(deg)   Current")
        print("  " + "-" * 40)
        for mid in ALL_IDS:
            name = f"Joint {mid - 10}" if mid != GRIPPER_ID else "Gripper"
            print(f"  {mid}   {name:8s}  {pos[mid]:8.2f}  {cur[mid]:5d}")
        if self.dropped_reads:
            print(f"\n  Dropped reads so far: {self.dropped_reads}")
        print()