import sys
import numpy as np
from copy import deepcopy

import math
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

# my lib
from lib.getTagPos import Tag


def my_ik_move(target, seed):
    start = perf_counter()
    q, success, rollout = ik.inverse(target, seed)
    stop = perf_counter()
    dt = stop - start
    print("q = {}".format(q))  # 1 x 7 vector!

    if success:
        print("Solution found in {time:2.2f} seconds ({it} iterations).".format(time=dt, it=len(rollout)))
        # arm.safe_move_to_position(q)
    else:
        print('IK Failed for this target using this seed.')

    arm.safe_move_to_position(q)  # ik_solver()

    return q  # return the q as a backup for further use


def pick_and_place_tag6(H_tic_w, i):
    """
        compulsorily pick and place as tag6 with white face up
    """
    # 2 - open gripper & move to static-up
    arm.open_gripper()
    H_rot = tag.get_H_staticRot("tag6")  # deal with the white faucet side case by case
    target = H_tic_w @ H_rot @ tag.H_upc_tic
    seed = arm.neutral_position()  # use neutral configuration as seed
    q_static_up = my_ik_move(target, seed)

    # 3 - line down
    target = H_tic_w @ H_rot
    seed = q_static_up
    my_ik_move(target, seed)

    # 4 - catch and back to static-up
    arm.exec_gripper_cmd(0.049, 15)
    arm.safe_move_to_position(q_static_up)  # ik_solver() back to static-up

    # 5 - go to tower-up
    H_twr_w = tag.get_H_twr_w("tag6", i)  # real height
    target = H_twr_w @ tag.get_H_twrUp("tag6")  # to tower & white face up -> to tower up
    q_tower_up = my_ik_move(target, q_static_up)

    # 6 - line down & place
    target = H_twr_w
    my_ik_move(target, q_tower_up)

    # 7 - place and back to tower-up
    arm.open_gripper()  # open gripper
    arm.safe_move_to_position(q_tower_up)

def pick_and_rotate_tag5(H_tic_w,i):
    arm.open_gripper()
    H_rot = tag.get_H_staticRot("tag5")
    target = H_tic_w @ H_rot
    seed = arm.neutral_position()  # use neutral configuration as seed
    q_static_up = my_ik_move(target, seed)


    H_down = np.array([
        [1, 0, 0, -0.1 * math.sqrt(1/2)],
        [0, 1, 0, 0],
        [0, 0, 1, 0.1 * math.sqrt(1/2)],
        [0, 0, 0, 1],
    ])

    target_down = H_tic_w @ H_rot @ H_down

    my_ik_move(target_down, q_static_up)
    arm.exec_gripper_cmd(0.049, 15)

    arm.safe_move_to_position(q_static_up)  # ik_solver() back to static-up

    #rotate 909090

    H_90 = np.array([
        [cos(-math.pi/2), 0, sin(-math.pi/2), 0],
        [0, 1, 0, 0],
        [-sin(-math.pi/2), 0, cos(-math.pi/2), 0],
        [0, 0, 0, 1],
    ])
    target_9090 = H_tic_w @ H_rot @ H_90
    q_after_9090 = my_ik_move(target_9090, q_static_up)

    H_down = np.array([
        [1, 0, 0, 0.1 * math.sqrt(1/2)],
        [0, 1, 0, 0],
        [0, 0, 1, 0.1 * math.sqrt(1/2)],
        [0, 0, 0, 1],
    ])
    q_ready_to_pick = my_ik_move(target_9090 @ H_down, q_after_9090)
    arm.open_gripper()
    """
    # 5 - go to tower-up
    H_twr_w = tag.get_H_twr_w("tag5", i)  # real height
    target = H_twr_w @ tag.get_H_twrUp("tag5")  # to tower & white face up -> to tower up
    q_tower_up = my_ik_move(target, q_ready_to_pick)

    # 6 - line down & place
    target = H_twr_w
    my_ik_move(target, q_tower_up)

    # 7 - place and back to tower-up
    arm.open_gripper()  # open gripper
    arm.safe_move_to_position(q_tower_up)
    """

def pick_and_place_tagx(tag_name, H_tic_w, i):
    # 2 - open gripper & move to static-up
    arm.open_gripper()
    H_rot = tag.get_H_staticRot(tag_name)  # deal with the white faucet side case by case
    target = H_tic_w @ H_rot @ tag.H_upc_tic
    seed = arm.neutral_position()  # use neutral configuration as seed
    q_static_up = my_ik_move(target, seed)

    # 3 - line down
    target = H_tic_w @ H_rot
    seed = q_static_up
    my_ik_move(target, seed)

    # 4 - catch and back to static-up
    arm.exec_gripper_cmd(0.049, 15)
    arm.safe_move_to_position(q_static_up)  # ik_solver() back to static-up

    # 5 - go to tower-up
    H_twr_w = tag.get_H_twr_w(tag_name, i)  # real height
    target = H_twr_w @ tag.get_H_twrUp(tag_name)  # to tower & white face up -> to tower up
    q_tower_up = my_ik_move(target, q_static_up)

    # 6 - line down & place
    target = H_twr_w
    my_ik_move(target, q_tower_up)

    # 7 - place and back to tower-up
    arm.open_gripper()  # open gripper
    arm.safe_move_to_position(q_tower_up)


if __name__ == "__main__":

    try:
        team = rospy.get_param("team")  # 'red' or 'blue'
    except KeyError:
        print('Team must be red or blue - make sure you are running final.launch!')
        exit()

    rospy.init_node("team_script")
    arm = ArmController()
    # detector = ObjectDetector()

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

    # 0 - init

    tag = Tag()
    ik = IK()

    # 1 - get centers of boxes

    H_t0_c = tag.get_H_t0_c()  # get H_t0_c
    tag_name, H_tic_w = tag.get_tag_data(H_t0_c)  # get all data of tags
    # print("*" * 100)
    # print("H_name = {}".format(tag_name))
    # print("H_tic_w = {}".format(H_tic_w))

    for i in range(len(tag_name) - 1):  # cuz you pop 1 data when i == 0!

        if i == 0:
            flag = False
            if flag is False:
                for j in range(len(tag_name)):
                    if tag_name[j] == "tag5":
                        pick_and_rotate_tag5(H_tic_w[j],0)
                        tag_name.pop(j)
                        H_tic_w.pop(j)
                        flag = True
                        break
            if flag is False:
                for j in range(len(tag_name)):
                    if tag_name[j] == "tag6":
                        pick_and_place_tag6(H_tic_w[j], 0)
                        tag_name.pop(j)
                        H_tic_w.pop(j)
                        flag = True
                        break

            if flag is False:
                pick_and_place_tag6(H_tic_w[0], 0)
                tag_name.pop(0)
                H_tic_w.pop(0)
                flag = True
            continue
        """
        if tag_name[i] == "tag1" or tag_name[i] == "tag2" or tag_name[i] == "tag3" or tag_name[i] == "tag4" or \
                tag_name[i] == "tag5" or tag_name[i] == "tag6":
            print("static pick & place!")
            pick_and_place_tagx(tag_name[i], H_tic_w[i], i+1)

        elif tag_name[i] == "tag7" or tag_name[i] == "tag8" or tag_name[i] == "tag9" or tag_name[i] == "tag10" or \
                tag_name[i] == "tag11" or tag_name[i] == "tag12":
            print("dynamic pick & place TBD!!!")
        """
    # END STUDENT CODE !!!!!!!!!!!!!!!!!!!!!!!
