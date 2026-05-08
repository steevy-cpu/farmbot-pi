"""
FarmBot Arm Controller
Scans /dev/cu.usbserial-AI0283MB for 7 Dynamixel AX-18A servos using pypot,
prints each one's ID and current position, then moves all to zero
if the full chain of 7 is found.

Baudrates tried: 1000000 (factory default for AX-18A), 57600

AX-18A notes (Protocol 1.0):
- Position range: 0-1023 raw = 0-300°; pypot center (0°) = raw 512 = 150°
- Default Shutdown register = 0x24 (overheating + overload) — overload resets
  torque_limit to 0, silently stalling the motor. Fix: re-arm torque_limit
  before every movement step and move in small increments.
- Moving Speed = 0 means MAX speed in joint mode; use non-zero for control.
- Compliance slope default 32; lower = stiffer/more torque near goal.
"""

import sys
import time

try:
    import pypot.dynamixel
except ImportError:
    sys.exit("[ERROR] pypot not installed.  Run:  pip install pypot")

PORT        = "/dev/cu.usbserial-AI0283MB"
BAUDRATES   = [1000000, 57600]
EXPECTED_IDS = list(range(1, 8))   # 7 servos, IDs 1-7
MOVE_SPEED  = 50                   # deg/s — slow enough to avoid overload trips
STEP_DEG    = 25                   # move this many degrees per increment


def _flush(dxl_io):
    """Clear the RX buffer after sync writes to prevent parse errors."""
    dxl_io._serial.reset_input_buffer()
    time.sleep(0.05)


def _arm_torque(dxl_io, ids):
    """Enable torque and restore torque_limit (overload alarm resets it to 0)."""
    dxl_io.enable_torque(ids)
    time.sleep(0.05)
    dxl_io.set_torque_limit({sid: 100.0 for sid in ids})
    time.sleep(0.05)
    _flush(dxl_io)


def move_servo_to_zero(dxl_io, sid, start_deg):
    """
    Move one servo to 0° in STEP_DEG increments.
    Re-arms torque before every step so an overload shutdown between steps
    doesn't silently stall the motor.
    """
    pos = start_deg
    step = 0

    while abs(pos) > 2.0:
        step += 1
        direction = -1 if pos > 0 else 1
        delta = min(STEP_DEG, abs(pos))
        target = pos + direction * delta
        wait = delta / MOVE_SPEED + 0.4

        print(f"      step {step}: {pos:+.1f}° → {target:+.1f}°  (wait {wait:.1f} s)")

        _arm_torque(dxl_io, [sid])
        dxl_io.set_goal_position({sid: target})
        time.sleep(wait)

        # Read actual position to track progress
        _flush(dxl_io)
        actual = dxl_io.get_present_position([sid])[0]
        print(f"               actual: {actual:+.1f}°")

        if abs(actual - pos) < 1.0:
            print(f"      Servo {sid} is stuck — stopping.")
            break

        pos = actual

    _flush(dxl_io)
    return dxl_io.get_present_position([sid])[0]


def run(baudrate: int) -> bool:
    """
    Open one DxlIO session, scan, print positions, then move all to zero.
    Scan and move share the same connection so _known_models stays warm
    (pypot silently drops writes when it can't resolve motor models).
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

        # --- Set speed (same for all, sync write) ---
        _arm_torque(dxl_io, found)
        dxl_io.set_moving_speed({sid: MOVE_SPEED for sid in found})
        _flush(dxl_io)

        # --- Move tip-to-base, one servo at a time, in small increments ---
        print("\n[INFO] Moving all 7 servos to zero (stepped, tip-to-base) ...")
        finals = {}
        for sid in reversed(found):
            start = positions[sid]
            if abs(start) < 2.0:
                print(f"\n  Servo ID {sid:2d}  already at zero, skipping")
                finals[sid] = start
                continue
            print(f"\n  Servo ID {sid:2d}  {start:+.1f}° → 0° ...")
            finals[sid] = move_servo_to_zero(dxl_io, sid, start)

        print("\n  Final positions:")
        for sid in found:
            print(f"    Servo ID {sid:2d} → {finals[sid]:+.1f} °")

    print("\n[DONE]")
    return True


def main() -> None:
    print(f"FarmBot Arm Controller  |  port={PORT}")
    print(f"Looking for {len(EXPECTED_IDS)} AX-18A servos (IDs {EXPECTED_IDS[0]}-{EXPECTED_IDS[-1]})")
    print(f"Baudrates to try: {BAUDRATES}")
    print(f"Move: {STEP_DEG}° steps at {MOVE_SPEED} °/s\n")

    for baud in BAUDRATES:
        if run(baud):
            break

    print("\n--- Done ---")


if __name__ == "__main__":
    main()
