# leader FK > scaling> follower IK(leader arm angle based)

import numpy as np
import math

class Calibration:
    def __init__(self):
        # leader arm
        self.leader_L1 = 0.25
        self.leader_L2 = 0.30
        self.leader_offset = 0.20
        
        # follower arm
        self.follower_L1 = 0.15
        self.follower_L2 = 0.20
        self.follower_offset = 0.15

        # scaling
        self.total_leader_L = self.leader_L1 + self.leader_L2
        self.total_follower_L = self.follower_L1 + self.follower_L2
        self.ratio = self.total_follower_L / self.total_leader_L

    def forward_kinematics(self, angles, L1, L2, offset):

        # joints: Shoulder(Pitch, Roll) -> Elbow(Yaw, Pitch)
        # angles: [sh_pitch, sh_roll, el_yaw, el_pitch]

        th1, th2, th3, th4 = angles

        # 1. shoulder pitch (y축 회전)
        R1 = np.array([
            [math.cos(th1),  0, math.sin(th1)],
            [0,              1, 0            ],
            [-math.sin(th1), 0, math.cos(th1)]
        ])

        # 2. shoulder roll (x축 회전)
        R2 = np.array([
            [1, 0,              0             ],
            [0, math.cos(th2), -math.sin(th2)],
            [0, math.sin(th2),  math.cos(th2)]
        ])

        # 3. elbow yaw (z축 회전)
        R3 = np.array([
            [math.cos(th3), -math.sin(th3), 0],
            [math.sin(th3),  math.cos(th3), 0],
            [0,              0,             1]
        ])

        # 4. elbow pitch (y축 회전)
        R4 = np.array([
            [math.cos(th4),  0, math.sin(th4)],
            [0,              1, 0            ],
            [-math.sin(th4), 0, math.cos(th4)]
        ])

        # L1 endpoint position
        # default position: (L1, 0, 0)
        p1 = R1 @ R2 @ np.array([L1, 0, 0])

        # L2 endpoint position
        p2 = R1 @ R2 @ R3 @ R4 @ np.array([L2, 0, 0])

        # final position including offset
        final_pos = np.array([offset, 0, 0]) + p1 + p2
        return final_pos

    def scale_position(self, leader_pos):
        vector_from_shoulder = leader_pos - np.array([self.leader_offset, 0, 0])
        follower_pos = (vector_from_shoulder * self.ratio) + np.array([self.follower_offset, 0, 0])
        return follower_pos

# follower IK and get follower angle based on leader angle
    def inverse_kinematics(self, target_pos, initial_angles):
        curr_angles = np.array(initial_angles, dtype=float)
        learning_rate = 0.05
        iterations = 100
        tolerance = 0.0001

        for i in range(iterations):
            curr_pos = self.forward_kinematics(curr_angles, self.follower_L1, self.follower_L2, self.follower_offset)
            error = target_pos - curr_pos
                
            if np.linalg.norm(error) < tolerance:
                break

            # Numerical Jacobian
            jacobian = np.zeros((3, 4))
            eps = 1e-6
            for j in range(4):
                temp_angles = curr_angles.copy()
                temp_angles[j] += eps
                next_pos = self.forward_kinematics(temp_angles, self.follower_L1, self.follower_L2, self.follower_offset)
                jacobian[:, j] = (next_pos - curr_pos) / eps

            j_inv = np.linalg.pinv(jacobian)
            curr_angles += learning_rate * np.dot(j_inv, error)

        return curr_angles

    def calculate_follower_angles(self, leader_angles):
        p_leader = self.forward_kinematics(leader_angles, self.leader_L1, self.leader_L2, self.leader_offset)
        p_follower = self.scale_position(p_leader)
        follower_angles = self.inverse_kinematics(p_follower, leader_angles)
        return follower_angles