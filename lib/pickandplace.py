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
# from lib.lineTraj import Movement


detector = ObjectDetector()


def static_pick_place(block_number):
    # 1 - get centers of boxes

    arm = ArmController()
    detector = ObjectDetector()
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
    
    # 2 move up

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
        [0, 0, 1, -0.1],
        [0, 0, 0, 1],
    ])  # H - move to up box center 10cm
    print("H_tic_w:\n",H_tic_w)
    target = H_tic_w[block_number] @ H_rot_z @ H_upc_tic
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

    arm.open_gripper()  # open gripper
    arm.safe_move_to_position(q)  # ik_solver()

    # 3 line down
    """
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
    """

    target = H_tic_w[1] @ H_rot_z
    seed_1 = q

    start = perf_counter()
    q, success, rollout = ik.inverse(target, seed_1)
    stop = perf_counter()
    dt = stop - start
    print("q = {}".format(q))

    if success:
        print("Solution found in {time:2.2f} seconds ({it} iterations).".format(time=dt, it=len(rollout)))
        # arm.safe_move_to_position(q)
    else:
        print('IK Failed for this target using this seed.')

    arm.safe_move_to_position(q)  # ik_solver()

    # 4 catch and back

    arm.exec_gripper_cmd(0.05, 10)

    arm.safe_move_to_position(seed_1)  # ik_solver()



    ### define tower placement height 
    
    target = np.array([
        [1, 0, 0, .934],
        [0, 1, 0, .687],
        [0, 0, 1, 0.2+block_number*0.05+.05+0.01],
        [0, 0, 0, 1],
    ])

    q_vertically_above_tower, success, rollout = IK().inverse(target, seed_1)
    arm.safe_move_to_position(q_vertically_above_tower)


    ###and the final goal to place

    target = np.array([
        [1, 0, 0, .934],
        [0, 1, 0, .687],
        [0, 0, 1, .2+block_number*0.05+0.01],
        [0, 0, 0, 1],
    ])
    q_goal_position, success, rollout = IK().inverse(target, q_vertically_above_tower)
    arm.safe_move_to_position(q_goal_position)

    arm.open_gripper()

    arm.move_to_position(seed)







