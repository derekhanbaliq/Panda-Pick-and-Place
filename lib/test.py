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

from lib.getTagPos import getTagPos
from lib.lineTraj import Movement

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

    # STUDENT CODE HERE !!!!!!!!!!!!!!!!!!!!!!!

    # 1 - get centers of boxes

    H_tic_w = []
    H_name = []

    for (name, pose) in detector.get_detections():
        H_tic_w.append(getTagPos(detector.get_detections()[0][1], pose))  # H_tic_w[0] is not usable!
        H_name.append(name)

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

    H_upc_tic = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, -0.2],
        [0, 0, 0, 1],
    ])

    target = H_tic_w[1] @ H_rot_z @ H_upc_tic
    seed = arm.neutral_position()  # use neutral configuration as seed

    start = perf_counter()
    q, success, rollout = ik.inverse(target, seed)
    stop = perf_counter()
    dt = stop - start
    print("q = {}".format(q))

    if success:
        print("Solution found in {time:2.2f} seconds ({it} iterations).".format(time=dt, it=len(rollout)))
        # arm.safe_move_to_position(q)
    else:
        print('IK Failed for this target using this seed.')

    # 3 execute grip test

    arm.open_gripper()  # open gripper
    arm.safe_move_to_position(q)  # ik_solver()
    # arm.exec_gripper_cmd(0.05, 10)
    #
    # # arm.safe_move_to_position(seed)  # ik_solver()

    # 4 line down

    move = Movement()
    fk = FK()

    line_start_vec = fk.forward(q)[0][-1]
    print("line_start_vec = {}".format(line_start_vec))

    box_center = H_tic_w[2] @ H_rot_z

    desired_altitude = box_center[2][-1]
    print("desired_altitude = {}".format(desired_altitude))

    move.follow_trajectory(desired_altitude, line_start_vec)
    # arm = ArmController(on_state_callback=callback)

    # move.active = True
    move.start_time = time_in_seconds()

    # 5 catch and back

    arm.exec_gripper_cmd(0.05, 10)

    arm.safe_move_to_position(seed)  # ik_solver()

    # Move around...
    # arm.safe_move_to_position(arm.neutral_position() + .1)

    # END STUDENT CODE !!!!!!!!!!!!!!!!!!!!!!!
