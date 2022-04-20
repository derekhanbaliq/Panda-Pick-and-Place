import sys
import numpy as np
from copy import deepcopy

import rospy
import roslib

# Common interfaces for interacting with both the simulation and real environments!
from core.interfaces import ArmController
from core.interfaces import ObjectDetector

# for timing that is consistent with simulation or real time as appropriate
from core.utils import time_in_seconds

# The library you implemented over the course of this semester!
from lib.calculateFK import FK
from lib.calcJacobian import FK
from lib.solveIK import IK
from lib.rrt import rrt
from lib.loadmap import loadmap

from math import pi, sin, cos
from time import perf_counter


def getTagPos(H_t0_c, H_ti_c):
    """
        input: 4x4 matrix - tag i frame with respect to camera frame
        output: 4x4 matrix - tag i box's center frame with respect to world frame
    """

    H_t0_w = np.array([
        [1, 0, 0, -0.500],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1],
    ])

    H_tic_ti = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, -0.025],  # half of the box side length
        [0, 0, 0, 1],
    ])

    H_tic_w = H_t0_w @ np.linalg.inv(H_t0_c) @ H_ti_c @ H_tic_ti
    # print(H_tic_w)

    return H_tic_w


if __name__ == "__main__":

    try:
        team = rospy.get_param("team")  # 'red' or 'blue'
    except KeyError:
        print('Team must be red or blue - make sure you are running final.launch!')
        exit()

    rospy.init_node("team_script")
    arm = ArmController()
    detector = ObjectDetector()

    arm.safe_move_to_position(arm.neutral_position())  # on your mark!

    print("\n****************")
    if team == 'blue':
        print("** BLUE TEAM  **")
    else:
        print("**  RED TEAM  **")
    print("****************")
    input("\nWaiting for start... Press ENTER to begin!\n")  # get set!
    print("Go!\n")  # go!

    # STUDENT CODE HERE

    # 1 - get centers of boxes

    H_tic_w = []
    H_name = []

    for (name, pose) in detector.get_detections():
        if name == "tag0":
            H_t0_c = pose
            print("H_t0_c = {}".format(H_t0_c))
            break

    for (name, pose) in detector.get_detections():
        print("H_name c = {}".format(name))
        print("pose c = {}".format(pose))
        H_tic_w.append(getTagPos(H_t0_c, pose))  # H_tic_w[0] is not usable!
        H_name.append(name)

    print("*" * 100)
    print("H_tic_w = {}".format(H_tic_w))
    print("H_name = {}".format(H_name))

    # 2 - ik_solver() test

    ik = IK()

    H_rot_z = np.array([
        [1, 0, 0, 0],
        [0, -1, 0, 0],
        [0, 0, -1, 0],
        [0, 0, 0, 1],
    ])

    target = H_tic_w[2] @ H_rot_z
    seed = arm.neutral_position()  # use neutral configuration as seed

    start = perf_counter()
    q, success, rollout = ik.inverse(target, seed)
    stop = perf_counter()
    dt = stop - start

    if success:
        print("Solution found in {time:2.2f} seconds ({it} iterations).".format(time=dt, it=len(rollout)))
        # arm.safe_move_to_position(q)
    else:
        print('IK Failed for this target using this seed.')

    # 3 execute grip test

    arm.open_gripper()  # open gripper
    arm.safe_move_to_position(q)  # ik_solver()
    arm.exec_gripper_cmd(0.05, 10)

    arm.safe_move_to_position(seed)  # ik_solver()

    # Move around...
    # arm.safe_move_to_position(arm.neutral_position() + .1)

    # END STUDENT CODE
