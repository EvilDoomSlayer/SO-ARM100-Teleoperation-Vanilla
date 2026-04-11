"""
Standalone SO-ARM100 Teleoperation Script.
This script performs bilateral teleoperation between two SO-ARM100 robots (Leader and Follower).
It uses relative position mapping based on calibration files to ensure accurate movement mirroring.
"""

import argparse
import json
import os
from feetech_driver import FeetechDriver

# --- Configuration Constants ---
DEFAULT_LEADER_PORT = "COM3"
DEFAULT_FOLLOWER_PORT = "COM4"
DEFAULT_LEADER_CALIB = "../config/leader_calib.json"
DEFAULT_FOLLOWER_CALIB = "../config/follower_calib.json"
DEFAULT_IDS = "1,2,3,4,5,6"
# -------------------------------

def load_calibration(path):
    """
    Loads calibration data from a JSON file.
    
    Args:
        path (str): Path to the calibration JSON file.
        
    Returns:
        dict or None: Calibration dictionary if successful, None otherwise.
    """
    if not path or not os.path.exists(path):
        return None
    try:
        with open(path, 'r') as f:
            return json.load(f)
    except Exception as e:
        print(f"Warning: Could not load calibration from {path}: {e}")
        return None

def normalize(val, min_val, max_val):
    """
    Normalizes a raw encoder value to a 0.0-1.0 range based on calibrated limits.
    
    Args:
        val (int): Current raw position.
        min_val (int): Calibrated minimum limit.
        max_val (int): Calibrated maximum limit.
        
    Returns:
        float: Normalized position (0.0 to 1.0).
    """
    if max_val == min_val: return 0.5
    val = max(min(val, max_val), min_val)
    return (val - min_val) / (max_val - min_val)

def unnormalize(norm_val, min_val, max_val):
    """
    Converts a normalized 0.0-1.0 value back to a raw encoder target for a specific arm.
    
    Args:
        norm_val (float): Normalized position (0.0 to 1.0).
        min_val (int): Calibrated minimum limit of the target arm.
        max_val (int): Calibrated maximum limit of the target arm.
        
    Returns:
        int: Raw encoder target value.
    """
    return int(norm_val * (max_val - min_val) + min_val)

def main():
    """
    Main entry point for the teleoperation script.
    
    Initializes drivers, configures follower PID/safety, and enters the
    high-frequency mirroring loop.
    """
    parser = argparse.ArgumentParser(description="Standalone SO-ARM100 Teleoperation")
    parser.add_argument("--leader", default=DEFAULT_LEADER_PORT, help="Port for Leader arm")
    parser.add_argument("--follower", default=DEFAULT_FOLLOWER_PORT, help="Port for Follower arm")
    parser.add_argument("--leader_calib", default=DEFAULT_LEADER_CALIB, help="Path to Leader calibration")
    parser.add_argument("--follower_calib", default=DEFAULT_FOLLOWER_CALIB, help="Path to Follower calibration")
    parser.add_argument("--ids", default=DEFAULT_IDS, help="Comma-separated servo IDs")
    args = parser.parse_args()

    servo_ids = [int(x) for x in args.ids.split(",")]
    leader_calib = load_calibration(args.leader_calib)
    follower_calib = load_calibration(args.follower_calib)

    # Standard SO-ARM100 ID-to-Name mapping
    id_to_name = {1: "shoulder_pan", 2: "shoulder_lift", 3: "elbow_flex", 
                  4: "wrist_flex", 5: "wrist_roll", 6: "gripper"}

    print(f"Connecting to Leader on {args.leader}...")
    leader = FeetechDriver(args.leader)
    print(f"Connecting to Follower on {args.follower}...")
    follower = FeetechDriver(args.follower)

    # Apply configuration to Follower for smooth and safe operation
    print("Configuring Follower PID and Safety Settings...")
    for sid in servo_ids:
        follower.set_torque(sid, False)
        # Smoother movement than factory defaults
        follower.configure_pid(sid, p=16, i=0, d=32)
        if sid == 6: # Gripper safety
            follower.set_gripper_safety(sid)
        follower.set_torque(sid, True)

    # Leader torque MUST be disabled to allow manual movement
    print("Disabling torque on Leader servos...")
    for sid in servo_ids:
        leader.set_torque(sid, False)

    print("-" * 40)
    print(f"Teleoperation started for IDs: {servo_ids}")
    print(f"Calibration: {'Enabled' if (leader_calib and follower_calib) else 'Disabled'}")
    print("Press Ctrl+C to stop.")
    print("-" * 40)
    
    try:
        while True:
            for sid in servo_ids:
                raw_pos = leader.read_word(sid, leader.ADDR_PRESENT_POSITION)
                if raw_pos is not None:
                    name = id_to_name.get(sid)
                    
                    # If calibration exists for both, perform relative mapping
                    if leader_calib and follower_calib and name in leader_calib and name in follower_calib:
                        l_min, l_max = leader_calib[name]['range_min'], leader_calib[name]['range_max']
                        f_min, f_max = follower_calib[name]['range_min'], follower_calib[name]['range_max']
                        
                        norm = normalize(raw_pos, l_min, l_max)
                        target_pos = unnormalize(norm, f_min, f_max)
                        follower.write_word(sid, follower.ADDR_GOAL_POSITION, target_pos)
                    else:
                        # Fallback to raw direct mapping
                        follower.write_word(sid, follower.ADDR_GOAL_POSITION, raw_pos)
            
    except KeyboardInterrupt:
        print("\nStopping teleoperation...")
    finally:
        # For safety, disable follower torque on exit
        for sid in servo_ids:
            follower.set_torque(sid, False)
        leader.close()
        follower.close()
        print("Done.")

if __name__ == "__main__":
    main()
