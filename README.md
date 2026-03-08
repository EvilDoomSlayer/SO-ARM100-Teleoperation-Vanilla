# SO-ARM100 Standalone Teleoperation Suite

A lightweight, zero-dependency Python suite for bilateral teleoperation of the SO-ARM100 robot. This project provides scripts for monitoring, calibrating, and teleoperating two robot arms (Leader and Follower) using the Feetech SCS (Protocol 0) standard.

## 🚀 Quick Start

1. **Install Dependencies:**
   ```bash
   pip install pyserial
   ```

2. **Connect your Robots:**
   - Plug in both Leader and Follower arms via USB-to-TTL adapters.
   - Note the COM ports (e.g., `COM3` and `COM4`).

3. **Verify Connection:**
   ```bash
   python src/monitor_servos.py --port COM3
   ```

4. **Calibrate (If needed):**
   ```bash
   python src/calibrate_standalone.py --port COM3 --output leader_calib.json
   python src/calibrate_standalone.py --port COM4 --output follower_calib.json
   ```

5. **Start Teleoperation:**
   ```bash
   python src/teleop_standalone.py --leader COM3 --follower COM4
   ```

## 📁 Project Structure

- **`src/`**: Core Python scripts and the unified Feetech driver.
- **`config/`**: JSON calibration files containing homing offsets and joint ranges.
- **`docs/`**: Detailed design notes and technical documentation.

## 🛠️ Included Tools

| Script | Purpose |
| :--- | :--- |
| `teleop_standalone.py` | Main script for mirroring movement between two arms. |
| `calibrate_standalone.py` | Utility to set homing positions and record mechanical limits. |
| `monitor_servos.py` | Live diagnostic tool to view raw encoder values with torque disabled. |
| `feetech_driver.py` | Unified driver for Feetech SCS Protocol 0 communication. |

## 📖 Documentation

For detailed information on the system architecture, mapping logic, and safety mechanisms, see:
- [Design Notes](docs/DESIGN_NOTES.md)
- [Servo Protocol Details](docs/SERVO_PROTOCOL.md)
