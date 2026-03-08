# Technical Detail: Feetech SCS (Protocol 0)

This document provides a low-level look at the communication standard used by the SO-ARM100 servos and the `feetech_driver.py` module.

---

## 1. Physical Layer
- **Signal:** Half-duplex TTL (single wire for TX/RX).
- **Baudrate:** 1,000,000 (1 Mbps).
- **Data Bits:** 8.
- **Stop Bits:** 1.
- **Parity:** None.

## 2. Packet Framing

All commands sent to the servos must follow this byte sequence:

| Byte | Field | Description |
| :--- | :--- | :--- |
| 0 | 0xFF | Header 1 |
| 1 | 0xFF | Header 2 |
| 2 | ID | Servo ID (1-253), 254 (0xFE) is Broadcast |
| 3 | Length | Parameters Count + 2 |
| 4 | Instruction | Read (0x02), Write (0x03), etc. |
| 5..N | Parameters | Address and Data (Little-Endian) |
| N+1 | Checksum | `~(sum(bytes[2:N]) & 0xFF)` |

## 3. Comprehensive Control Table (Feetech STS/SMS)

Below is the full register map for Protocol 0 servos. Addresses marked with **(Teleop)** are actively used or configured in this project's scripts.

### EPROM (Persistent Settings)
*Changes here stay even after power-off.*

| Address | Size | Name | Purpose |
| :--- | :--- | :--- | :--- |
| 3 | 2 | Model Number | Read-only hardware model ID. |
| 5 | 1 | **ID (Teleop)** | Servo ID (1-253). |
| 6 | 1 | Baud Rate | Communication speed setting. |
| 7 | 1 | Return Delay | Delay before responding to a read request. |
| 8 | 1 | Status Level | Controls when status packets are sent. |
| 9 | 2 | **Min Pos Limit (Teleop)** | Clockwise physical limit. |
| 11 | 2 | **Max Pos Limit (Teleop)** | Counter-clockwise physical limit. |
| 13 | 1 | Max Temp Limit | Thermal protection threshold. |
| 14 | 1 | Max Voltage | Upper voltage alarm limit. |
| 15 | 1 | Min Voltage | Lower voltage alarm limit. |
| 16 | 2 | **Max Torque (Teleop)** | Maximum output force (0-1000). |
| 18 | 1 | Phase | Phase of the motor. |
| 19 | 1 | Unload Condition | Conditions that cause torque disable. |
| 20 | 1 | LED Alarm | Conditions that cause LED to flash. |
| 21 | 1 | **P Coefficient (Teleop)**| Proportional gain for the PID. |
| 22 | 1 | **D Coefficient (Teleop)**| Derivative gain for the PID. |
| 23 | 1 | **I Coefficient (Teleop)**| Integral gain for the PID. |
| 24 | 2 | Min Startup Force | Minimum PWM to start moving. |
| 26 | 1 | CW Dead Zone | Clockwise dead zone. |
| 27 | 1 | CCW Dead Zone | Counter-clockwise dead zone. |
| 28 | 2 | **Prot. Current (Teleop)**| Current limit to prevent burnout. |
| 30 | 1 | Angular Resolution | Resolution setting. |
| 31 | 2 | **Home Offset (Teleop)** | Calibration offset for the encoder. |
| 33 | 1 | Operating Mode | 0=Pos, 1=Velocity, 2=PWM, 3=Step. |
| 34 | 1 | Protective Torque | Torque limit when protection active. |
| 35 | 1 | Protection Time | Time before protection triggers. |
| 36 | 1 | **Overload Torque (Teleop)**| Torque limit when stalled. |

### SRAM (Volatile Settings)
*Reset to EPROM defaults when powered off.*

| Address | Size | Name | Purpose |
| :--- | :--- | :--- | :--- |
| 40 | 1 | **Torque Enable (Teleop)**| 1 = Enabled, 0 = Disabled. |
| 41 | 1 | Acceleration | Motion smoothing/ramp value. |
| 42 | 2 | **Goal Position (Teleop)**| Target encoder value (0-4095). |
| 44 | 2 | Goal Time | Time to reach target. |
| 46 | 2 | Goal Speed | Speed limit for the movement. |
| 48 | 2 | Torque Limit | Temporary torque limit (runtime). |
| 55 | 1 | Lock | 1 = Lock EPROM, 0 = Unlock. |
| 56 | 2 | **Present Pos (Teleop)** | Current real-time encoder value. |
| 58 | 2 | Present Speed | Current rotational speed. |
| 60 | 2 | Present Load | Current load/effort on the motor. |
| 62 | 1 | Present Voltage | Current input voltage. |
| 63 | 1 | Present Temp | Current internal temperature. |
| 65 | 1 | Status | Internal error status bits. |
| 66 | 1 | **Moving (Teleop)** | 1 = Currently in motion, 0 = Idle. |
| 69 | 2 | Present Current | Real-time current consumption. |
| 71 | 2 | Goal Position 2 | Internal target after smoothing. |

## 4. Checksum Implementation (Python)

The checksum is calculated by summing all bytes starting from the ID to the last parameter, taking only the lowest 8 bits, and then performing a bitwise NOT operation.

```python
def calculate_checksum(id, length, instruction, params):
    checksum = id + length + instruction + sum(params)
    return (~checksum) & 0xFF
```

## 5. Timing Considerations
- **Wait Time:** When reading a position (`INST_READ`), the computer must wait for the servo to finish sending its status packet before sending the next command.
- **Sequential Latency:** Because the TTL bus is half-duplex, you can only talk to one motor at a time. The `teleop_standalone.py` script minimizes this by using a tightly optimized sequential loop.
