"""
FarmBot Arm Controller
Scans /dev/cu.usbserial-AI0283MB for 7 Dynamixel AX-18A servos using pypot,
prints each one's ID and current position, then moves all to zero
if the full chain of 7 is found.

Baudrates tried: 1000000 (factory default for AX-18A), 57600
"""

import sys
import time

try:
    import pypot.dynamixel
except ImportError:
    sys.exit(
        "[ERROR] pypot not installed.  Run:  pip install pypot"
    )

PORT = "/dev/cu.usbserial-AI0283MB"
BAUDRATES = [1000000, 57600]
EXPECTED_IDS = list(range(1, 8))   # 7 servos, IDs 1-7
MOVE_SPEED = 100                   # goal speed (deg/s) for zero move


def run(baudrate: int) -> None:
    """
    Open one connection, scan, print positions, then move to zero if all 7
    servos are found. Keeping scan and move in the same DxlIO session ensures
    _known_models is already populated before any write command is sent
    (pypot silently drops writes when it cannot resolve motor models).
    """
    print(f"\n  Trying {baudrate} bps ...")
    try:
        dxl_io = pypot.dynamixel.DxlIO(PORT, baudrate=baudrate)
    except Exception as exc:
        print(f"  [ERROR] Could not open {PORT} at {baudrate}: {exc}")
        return False

    with dxl_io:
        # --- Scan ---
        found = dxl_io.scan(EXPECTED_IDS)
        if not found:
            print("  (no response)")
            return False

        present = dxl_io.get_present_position(found)
        positions = dict(zip(found, present))

        print(f"\n  Found {len(found)} servo(s) at {baudrate} bps:")
        for sid in found:
            print(f"    Servo ID {sid:2d}  present position = {positions[sid]:+.1f} °")

        if len(found) < len(EXPECTED_IDS):
            missing = sorted(set(EXPECTED_IDS) - set(found))
            print(f"\n[WARN] Missing servo ID(s): {missing}")
            print("[WARN] Not moving — need all 7 servos before commanding motion.")
            return True

        # --- Move to zero (same open connection, _known_models already warm) ---
        print("\n[INFO] Moving all 7 servos to zero position (0 °) ...")

        dxl_io.enable_torque(found)
        time.sleep(0.1)

        # torque_limit=0 silently prevents all motion even when torque is enabled
        dxl_io.set_torque_limit({sid: 100.0 for sid in found})
        time.sleep(0.1)

        dxl_io.set_moving_speed({sid: MOVE_SPEED for sid in found})
        time.sleep(0.1)

        # Move tip-to-base (highest IDs first) one at a time to avoid
        # current spikes that stall the heavy base/shoulder joints.
        current = dict(zip(found, dxl_io.get_present_position(found)))
        for sid in reversed(found):
            travel = abs(current[sid])          # degrees from zero
            if travel < 2.0:
                print(f"    Servo ID {sid:2d}  already at zero, skipping")
                continue
            wait = max(1.0, travel / MOVE_SPEED + 0.5)   # travel time + buffer
            print(f"    Servo ID {sid:2d}  moving {current[sid]:+.1f}° → 0°  (wait {wait:.1f} s)")
            dxl_io.set_goal_position({sid: 0.0})
            time.sleep(wait)

        final = dxl_io.get_present_position(found)
        print("\n  Final positions after zero move:")
        for sid, pos in zip(found, final):
            print(f"    Servo ID {sid:2d} → {pos:+.1f} °")

    print("\n[DONE] All servos at zero.")
    return True


def main() -> None:
    print(f"FarmBot Arm Controller  |  port={PORT}")
    print(f"Looking for {len(EXPECTED_IDS)} AX-18A servos (IDs {EXPECTED_IDS[0]}-{EXPECTED_IDS[-1]})")
    print(f"Baudrates to try: {BAUDRATES}")

    for baud in BAUDRATES:
        if run(baud):
            break

    print("\n--- Done ---")


if __name__ == "__main__":
    main()
