# farmbot-pi

Raspberry Pi controller for the FarmBot project — a precision farming robot that combines XYZ stepper motor motion control with a Dynamixel servo arm.

## Overview

This repository contains the Python software running on the Raspberry Pi that acts as the central controller between:

- **Arduino Mega 2560** — drives 5 stepper motors (XL, XR, Y, ZL, ZR) across X/Y/Z axes via a custom PlatformIO firmware
- **Dynamixel AX-18A** — a servo arm connected over RS-485/TTL using Protocol 1.0

## Files

| File | Description |
|------|-------------|
| `farmbot_controller.py` | Interactive CLI to send motion commands to the Arduino over serial (115200 baud). Supports homing, relative/absolute moves, emergency stop, and status reporting. |
| `scan_dynamixel.py` | Scans `/dev/ttyUSB0` across all common baudrates (1000000, 57600, 115200, 9600, 4800, 19200, 38400) and IDs 1–10 to locate a connected AX-18A servo using Dynamixel Protocol 1.0. |
| `test_connection.py` | Automated test that sends STATUS → HALL → X500 to verify the Arduino serial link is working. |

## Requirements

```bash
pip install pyserial
```

## Usage

### Control the Arduino (XYZ motion)
```bash
python3 farmbot_controller.py
```
Auto-detects the Arduino on `/dev/ttyACM*`, `/dev/ttyUSB*`, or `usbmodem*` (macOS).

### Find the Dynamixel servo
```bash
python3 scan_dynamixel.py
```
Reports the baudrate and ID of any detected AX-18A servo.

### Test Arduino connection
```bash
python3 test_connection.py
```

## Serial Commands (Arduino)

| Command | Action |
|---------|--------|
| `X####` / `Y####` / `Z####` | Move axis by relative steps |
| `PX25/50/75` | Move X to absolute percentage position |
| `H` / `HALL` | Home all axes |
| `HX` / `HY` / `HZ` | Home individual axis |
| `R` / `STATUS` | Report position and system status |
| `S` / `STOP` | Emergency stop |
| `S0` / `CLEAR` | Resume after emergency stop |

## Hardware

- Raspberry Pi (any model with USB)
- Arduino Mega 2560 (connected via USB)
- Dynamixel AX-18A servo (connected via USB-to-TTL adapter on `/dev/ttyUSB0`)
