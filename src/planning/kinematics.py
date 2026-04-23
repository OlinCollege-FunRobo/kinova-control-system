"""
Kinematics definitions for the 7 DoF Kinova arm.
Handles forward and inverse kinematics calculations, workspace bounds, and joint limits.
"""

import numpy as np
from math import pi
import random

def calc_dh(joint_angles: list):
    """
    Takes current joint angles and returns DH table

    Args:
        joint_angles (list): List containing current joint angles of arm

    Returns:
        DH_TABLE (np.ndarray): NDarray representing DH table
    """
    q1,q2,q3,q4,q5,q6,q7 = joint_angles

    DH_TABLE = np.array([[pi, 0.0, 0.0, 0.0],
                         [pi/2, 0.0, -(0.1564 + 0.1284), q1],
                         [pi/2, 0.0 -(0.0054 + 0.0064),q2 + pi],
                         [pi/2, 0.0, -(0.2104 + 0.2104), q3 + pi],
                         [pi/2, 0.0, -(0.0064 + 0.0064), q4 + pi],
                         [pi/2, 0.0, -(0.2084 + 0.1059), q5 + pi],
                         [pi/2, 0.0, 0.0, q6 + pi],
                         [pi, 0.0, -(0.1059 + 0.0615), q7 + pi]])

    return DH_TABLE


def calc_inverse_kinematics(self, ee: ut.EndEffector, joint_values: ut.List[float], tol: float = 0.01, ilimit: int = 1000):

        # Arm is underactuated, only do position IK
        p_ee = np.array([ee.x, ee.y, ee.z])
        # Get initial guess of joint values; ensure they are sampled from valid joint values.
        lim = [
                [-2*np.pi / 3, 2*np.pi / 3],
                [-2*np.pi / 3, 2*np.pi / 3],
                [-2*np.pi / 3, 2*np.pi / 3],
                [-2*np.pi / 3, 2*np.pi / 3],
                [-2*np.pi / 3, 2*np.pi / 3]
              ]
        if joint_values is not None:
            print(f"Initial guess provided {joint_values}")
            guess = joint_values
        else:
            guess = [
                    random.uniform(*lim[0]), 
                    random.uniform(*lim[1]),
                    random.uniform(*lim[2]),
                    random.uniform(*lim[3]),
                    random.uniform(*lim[4]),
                    ]

        for r in range(10):
            
            icount = 0
            while icount < ilimit:
                fk_result, _ = self.calc_forward_kinematics(guess, True)
                diff = p_ee - np.array([fk_result.x, fk_result.y, fk_result.z])
                if np.linalg.norm(diff) < tol and ut.check_joint_limits(guess, lim):
                    return guess
                guess += self.calc_inv_jacobian(guess)@diff
                icount += 1
