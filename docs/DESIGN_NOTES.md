# Design Notes: SO-ARM100 Standalone Teleoperation Suite

This document outlines the architectural decisions, mapping algorithms, and safety protocols implemented in the standalone Python suite for the SO-ARM100.

---

## 1. System Architecture

The suite is designed to operate without a complex robotics middleware (like ROS) or heavy dependencies (like LeRobot). It relies on a three-tier architecture:

1.  **Hardware Layer:** Feetech STS3215 servos daisy-chained on a TTL serial bus.
2.  **Driver Layer (`feetech_driver.py`):** Encapsulates the binary Feetech Protocol 0 (SCS) logic.
3.  **Application Layer:** Specific scripts for monitoring, calibration, and teleoperation.

## 2. Communication Protocol (Feetech SCS)

All scripts communicate via a 1,000,000 baud (1Mbaud) serial connection. The protocol follows a request-response or broadcast model.

### Packet Structure:
`[0xFF, 0xFF, ID, Length, Instruction, Parameters..., Checksum]`

-   **Header (0xFF, 0xFF):** Marks the start of a message.
-   **ID:** Target servo (1–6). 0xFE is used for broadcast to all servos.
-   **Length:** Remaining bytes in the packet.
-   **Instruction:** 0x02 (Read), 0x03 (Write), etc.
-   **Checksum:** `~(sum(ID+Len+Inst+Params) & 0xFF) & 0xFF`.

## 3. Mapping Logic (Relative Position Control)

Since no two 3D-printed arms are identical, direct raw encoder mapping is avoided. Instead, we use **Relative Normalization**.

### The Normalization Formula:
1.  **Normalize Leader (0 to 1):**
    `normalized_pos = (Leader_Present_Pos - Leader_Min) / (Leader_Max - Leader_Min)`
2.  **Unnormalize for Follower (Raw Target):**
    `Follower_Goal = (normalized_pos * (Follower_Max - Follower_Min)) + Follower_Min`

### Advantages:
-   **Mechanical Symmetry:** The follower arm perfectly mirrors the leader's relative movements, even if its physical range is slightly offset or shorter.
-   **Safe Limits:** The follower never attempts to move outside its own calibrated mechanical limits.

## 4. Calibration Strategy

### Persistent Homing Offsets:
The `calibrate_standalone.py` script calculates a **Homing Offset** such that the arm's "rest" position corresponds to the value `2048` (center of the 12-bit encoder). This offset is written directly to the servo's non-volatile memory (**EPROM Address 31**), ensuring it persists even after power cycles.

### Dynamic Range Recording:
The calibration script tracks the absolute minimum and maximum values seen while the user manually manipulates the joints. These limits are saved in `config/*.json` and are used by the teleoperation script for normalization.

## 5. Safety Protocols

### Real-Time Torque Management:
-   **Leader:** Torque is explicitly disabled to allow free manual manipulation.
-   **Follower:** Torque is enabled for movement but automatically disabled upon script termination (via Python `finally` blocks).

### Gripper Burnout Protection:
The gripper joint (ID 6) is configured with reduced current and torque limits:
-   **Max Torque:** 50% (500/1000).
-   **Protection Current:** 250mA.
-   This allows the gripper to maintain a stall (holding an object) for long periods without overheating the motor.

### PID Smoothing:
-   The P-coefficient is reduced from the factory default (32) to **16**.
-   This reduces oscillation and "jitter" when the follower arm reaches its target position, resulting in a more professional, "damped" feel.

---

## 6. Control Table Reference

For a complete map of all EPROM and SRAM registers for the Feetech STS/SMS series, including those used for teleoperation, see the [Servo Protocol Documentation](SERVO_PROTOCOL.md#3-comprehensive-control-table-feetech-stssms).
