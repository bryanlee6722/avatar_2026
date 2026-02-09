from calibration import Calibration
import numpy as np
import math

def test_calibration():
    calib = Calibration()
    
    # test case (s_pitch, s_roll, e_yaw , e_pitch)
    leader_angles = [-math.pi/4, -math.pi/6, math.pi/6, math.pi/4]
    
    print("=== [Step 1] Leader Forward Kinematics (FK) ===")
    p_leader = calib.forward_kinematics(leader_angles, calib.leader_L1, calib.leader_L2, calib.leader_offset)
    print(f"Leader Angles: {leader_angles}")
    print(f"Leader Hand Position: X={p_leader[0]:.4f}, Y={p_leader[1]:.4f}, Z={p_leader[2]:.4f}")
    
    print("\n=== [Step 2] Scaling to Follower Workspace ===")
    p_follower_target = calib.scale_position(p_leader)
    print(f"Follower Target Position: X={p_follower_target[0]:.4f}, Y={p_follower_target[1]:.4f}, Z={p_follower_target[2]:.4f}")
    
    # examine scaling ratio
    leader_arm_vec = p_leader - np.array([calib.leader_offset, 0, 0])
    follower_arm_vec = p_follower_target - np.array([calib.follower_offset, 0, 0])
    print(f"Leader Arm Length: {np.linalg.norm(leader_arm_vec):.4f}")
    print(f"Follower Arm Length: {np.linalg.norm(follower_arm_vec):.4f}")
    print(f"Ratio check: {np.linalg.norm(follower_arm_vec)/np.linalg.norm(leader_arm_vec):.4f} (Target: {calib.ratio:.4f})")

    print("\n=== [Step 3] Follower Inverse Kinematics (IK) ===")
    # IK based on leader angles
    follower_angles = calib.inverse_kinematics(p_follower_target, leader_angles)
    print(f"Calculated Follower Angles (rad): {follower_angles}")
    print(f"Calculated Follower Angles (deg): {[math.degrees(a) for a in follower_angles]}")

    print("\n=== [Step 4] Final Verification (FK vs IK) ===")
    # FK based on angles from IK and examine
    p_final_check = calib.forward_kinematics(follower_angles, calib.follower_L1, calib.follower_L2, calib.follower_offset)
    error = np.linalg.norm(p_follower_target - p_final_check)
    
    print(f"Final Position from IK angles: X={p_final_check[0]:.4f}, Y={p_final_check[1]:.4f}, Z={p_final_check[2]:.4f}")
    print(f"Distance Error: {error:.6f} meters")

    if error < 0.001:
        print("\nTEST SUCCESS: IK successfully converged to target!")
    else:
        print("\nTEST FAILED: Error is too large.")

if __name__ == "__main__":
    test_calibration()