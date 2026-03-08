"""
Standalone SO-ARM100 Calibration Script.
This script guides the user through calibrating a single SO-ARM100 robot.
It handles setting homing offsets and recording mechanical joint ranges.
"""

import time
import argparse
import json
import os
from feetech_driver import FeetechDriver

def main():
    """
    Main entry point for the calibration script.
    
    Workflow:
    1. Reset existing homing offsets in servo memory.
    2. Interactively set a new homing/rest position.
    3. Manually record the min/max limits for each joint.
    4. Save calibration to a JSON file and update hardware EPROM.
    """
    parser = argparse.ArgumentParser(description="Standalone SO-ARM100 Calibration")
    parser.add_argument("--port", required=True, help="Serial port of the robot to calibrate")
    parser.add_argument("--output", required=True, help="Output JSON filename (e.g. leader_calib.json)")
    parser.add_argument("--ids", default="1,2,3,4,5,6", help="Comma-separated servo IDs")
    args = parser.parse_args()

    servo_ids = [int(x) for x in args.ids.split(",")]
    id_to_name = {1: "shoulder_pan", 2: "shoulder_lift", 3: "elbow_flex", 
                  4: "wrist_flex", 5: "wrist_roll", 6: "gripper"}

    print(f"Connecting to robot on {args.port}...")
    robot = FeetechDriver(args.port)

    # 1. Reset Homing Offsets
    print("\n[Step 1] Resetting existing homing offsets...")
    for sid in servo_ids:
        robot.set_torque(sid, False)
        robot.write_word(sid, robot.ADDR_HOMING_OFFSET, 0)

    # 2. Set Homing Position
    print("\n[Step 2] Homing")
    input("Manually move the arm to its REST/CENTER position and press Enter...")
    calibration_data = {}
    for sid in servo_ids:
        pos = robot.read_word(sid, robot.ADDR_PRESENT_POSITION)
        if pos is None:
            print(f"Warning: Could not read ID {sid}. Check connection.")
            continue
        
        # Calculate offset to make current pos 2048 (middle of 4096)
        # Result: hardware will report '2048' at this exact physical location.
        offset = pos - 2048
        
        name = id_to_name.get(sid, f"servo_{sid}")
        calibration_data[name] = {
            "id": sid,
            "drive_mode": 0,
            "homing_offset": offset,
            "range_min": 0,
            "range_max": 4095
        }
        print(f"ID {sid} ({name}): Homing Offset set to {offset}")

    # 3. Record Range of Motion
    print("\n[Step 3] Range of Motion")
    print("Move ALL joints through their full range of motion.")
    print("Keep moving until you have reached the mechanical limits of every joint.")
    input("Press Enter when you are ready to start recording...")
    
    mins = {sid: 4095 for sid in servo_ids}
    maxes = {sid: 0 for sid in servo_ids}
    
    print("Recording... Press Ctrl+C when finished moving all joints.")
    try:
        while True:
            for sid in servo_ids:
                pos = robot.read_word(sid, robot.ADDR_PRESENT_POSITION)
                if pos is not None:
                    mins[sid] = min(mins[sid], pos)
                    maxes[sid] = max(maxes[sid], pos)
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("\nRecording stopped.")

    # 4. Save and Apply
    # Ensure config directory exists
    config_dir = "../config"
    os.makedirs(config_dir, exist_ok=True)
    output_path = os.path.join(config_dir, args.output)
    
    print(f"\n[Step 4] Saving to {output_path}...")
    for sid in servo_ids:
        name = id_to_name.get(sid, f"servo_{sid}")
        if name in calibration_data:
            calibration_data[name]["range_min"] = mins[sid]
            calibration_data[name]["range_max"] = maxes[sid]
            
            # Persist homing offset to hardware EPROM
            robot.write_word(sid, robot.ADDR_HOMING_OFFSET, calibration_data[name]["homing_offset"])

    with open(output_path, 'w') as f:
        json.dump(calibration_data, f, indent=4)
    
    robot.close()
    print("Calibration Complete!")

if __name__ == "__main__":
    main()
