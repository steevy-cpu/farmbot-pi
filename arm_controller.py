"""
FarmBot Arm Controller
Scans /dev/ttyUSB0 for 7 Dynamixel AX-18A servos using pypot,
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

PORT = "/dev/ttyUSB0"
BAUDRATES = [1000000, 57600]
EXPECTED_IDS = list(range(1, 8))   # 7 servos, IDs 1-7
MOVE_SPEED = 100                   # goal speed (deg/s) for zero move


def scan_at_baudrate(baudrate: int) -> tuple[list[int], dict[int, float]]:
    """
    Open the port at *baudrate*, scan for IDs 1-7, and read present
    positions for every servo that responds.

    Returns (found_ids, positions) where positions is {id: degrees}.
    """
    found: list[int] = []
    positions: dict[int, float] = {}

    print(f"\n  Trying {baudrate} bps ...")
    try:
        dxl_io = pypot.dynamixel.DxlIO(PORT, baudrate=baudrate)
    except Exception as exc:
        print(f"  [ERROR] Could not open {PORT} at {baudrate}: {exc}")
        return found, positions

    with dxl_io:
        found = dxl_io.scan(EXPECTED_IDS)
        if found:
            present = dxl_io.get_present_position(found)
            # pypot returns a list aligned to the id list
            positions = dict(zip(found, present))

    return found, positions


def move_to_zero(baudrate: int, ids: list[int]) -> None:
    """
    Re-open the port and command every servo in *ids* to 0 degrees,
    then wait until all have settled (or 3 s timeout per servo).
    """
    print("\n[INFO] Moving all 7 servos to zero position (0 °) ...")
    try:
        dxl_io = pypot.dynamixel.DxlIO(PORT, baudrate=baudrate)
    except Exception as exc:
        print(f"[ERROR] Could not re-open {PORT}: {exc}")
        return

    with dxl_io:
        # Enable torque so the servos will actually move
        dxl_io.enable_torque(ids)

        # Set a modest speed so the move is controlled
        speed_dict = {sid: MOVE_SPEED for sid in ids}
        dxl_io.set_moving_speed(speed_dict)

        # Command all to 0 degrees simultaneously
        goal_dict = {sid: 0.0 for sid in ids}
        dxl_io.set_goal_position(goal_dict)

        # Poll until all servos stop moving (or 5 s timeout)
        deadline = time.time() + 5.0
        while time.time() < deadline:
            time.sleep(0.1)
            moving = dxl_io.get_moving_speed(ids)
            # get_moving_speed returns current speed; 0 means stopped
            if all(abs(v) < 1.0 for v in moving):
                break

        # Read final positions for confirmation
        final = dxl_io.get_present_position(ids)
        print("\n  Final positions after zero move:")
        for sid, pos in zip(ids, final):
            print(f"    Servo ID {sid:2d} → {pos:+.1f} °")

    print("\n[DONE] All servos at zero.")


def main() -> None:
    print(f"FarmBot Arm Controller  |  port={PORT}")
    print(f"Looking for {len(EXPECTED_IDS)} AX-18A servos (IDs {EXPECTED_IDS[0]}-{EXPECTED_IDS[-1]})")
    print(f"Baudrates to try: {BAUDRATES}")

    found_ids: list[int] = []
    working_baudrate: int | None = None

    for baud in BAUDRATES:
        ids, positions = scan_at_baudrate(baud)

        if ids:
            print(f"\n  Found {len(ids)} servo(s) at {baud} bps:")
            for sid in ids:
                pos = positions[sid]
                print(f"    Servo ID {sid:2d}  present position = {pos:+.1f} °")

            # Keep the baudrate that found the most servos
            if len(ids) > len(found_ids):
                found_ids = ids
                working_baudrate = baud
        else:
            print("  (no response)")

    print("\n--- Scan complete ---")

    if not found_ids:
        print("[WARN] No AX-18A servos found.")
        print("       Check wiring, power supply, and that /dev/ttyUSB0 is correct.")
        sys.exit(1)

    print(f"[INFO] {len(found_ids)}/{len(EXPECTED_IDS)} servo(s) found at {working_baudrate} bps.")

    if len(found_ids) < len(EXPECTED_IDS):
        missing = sorted(set(EXPECTED_IDS) - set(found_ids))
        print(f"[WARN] Missing servo ID(s): {missing}")
        print("[WARN] Not moving — need all 7 servos before commanding motion.")
        sys.exit(1)

    # All 7 found: move each to zero
    move_to_zero(working_baudrate, found_ids)


if __name__ == "__main__":
    main()
