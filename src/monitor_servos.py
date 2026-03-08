"""
Standalone SO-ARM100 Encoder Monitoring Script.
This script provides real-time diagnostics for the robot.
It disables torque on all servos and streams their raw positions.
"""

import time
import argparse
from feetech_driver import FeetechDriver

def main():
    """
    Main entry point for the monitoring script.
    
    Workflow:
    1. Connect to the robot serial port.
    2. Disable torque on all specified servos for free movement.
    3. Enter a loop that prints live-updating positions for all joint.
    """
    parser = argparse.ArgumentParser(description="Monitor SO-ARM100 Encoder Values")
    parser.add_argument("--port", required=True, help="Serial port of the robot (e.g. COM3)")
    parser.add_argument("--ids", default="1,2,3,4,5,6", help="Comma-separated servo IDs")
    args = parser.parse_args()

    servo_ids = [int(x) for x in args.ids.split(",")]
    id_to_name = {1: "shoulder_pan", 2: "shoulder_lift", 3: "elbow_flex", 
                  4: "wrist_flex", 5: "wrist_roll", 6: "gripper"}

    print(f"Connecting to robot on {args.port}...")
    robot = FeetechDriver(args.port)

    # Torque disable allows manual manipulation of the arm
    print("Disabling torque on all servos to allow free movement...")
    for sid in servo_ids:
        robot.set_torque(sid, False)

    print("\nMonitoring positions. Press Ctrl+C to stop.\n")
    print(f"{'ID':<4} | {'Name':<15} | {'Position':<8}")
    print("-" * 35)

    try:
        while True:
            # Construct a single-line summary to keep terminal clean
            output = ""
            for sid in servo_ids:
                pos = robot.read_word(sid, robot.ADDR_PRESENT_POSITION)
                name = id_to_name.get(sid, "unknown")
                val = str(pos) if pos is not None else "LOST"
                output += f"ID {sid} ({name}): {val:<5}  |  "
            
            # Use carriage return (\r) to stay on the same line
            print(f"\r{output}", end="", flush=True)
            time.sleep(0.05)
            
    except KeyboardInterrupt:
        print("\n\nMonitoring stopped.")
    finally:
        robot.close()
        print("Done.")

if __name__ == "__main__":
    main()
