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


def pick_and_place_tagx(team, tag_name, H_tic_w, i):
    """
        pick & place box with side-case method
    """

    q_static_up_ref = np.array([-0.03450483, 0.22539452, -0.14064529, -1.77050944, 0.03436498, 1.99356833, 0.78106695])
    q_tower_up_ref_side = np.array([-0.08188998, 0.04771562, -0.00321775, -1.92776809, 1.53728915, 1.49257634, -1.19063492])

    # 1 - open gripper & move to static-up
    arm.open_gripper()
    H_rot = tag.get_H_staticRot(tag_name)  # deal with the white faucet side case by case
    target = H_tic_w @ H_rot @ tag.H_upc_tic
    # seed = arm.neutral_position()  # use neutral configuration as seed
    seed = q_static_up_ref
    q_static_up = my_ik_move(target, seed)

    # 2 - line down
    target = H_tic_w @ H_rot
    seed = q_static_up
    my_ik_move(target, seed)

    # 3 - catch and back to static-up
    arm.exec_gripper_cmd(0.049, 15)
    arm.safe_move_to_position(q_static_up)  # ik_solver() back to static-up

    # 4 - go to tower-up
    H_twr_w = tag.get_H_twr_w(team, tag_name, i)  # real height
    target = H_twr_w @ tag.get_H_twrUp(tag_name)  # to tower & white face up -> to tower up
    # q_tower_up = my_ik_move(target, q_static_up)
    q_tower_up = my_ik_move(target, q_tower_up_ref_side)
    # q_tower_up_ref_side = arm.get_positions()
    # print("q_tower_up_ref_side? = {}".format(q_tower_up_ref_side))

    # 5 - line down & place
    target = H_twr_w
    my_ik_move(target, q_tower_up)

    # 6 - place and back to tower-up
    arm.open_gripper()  # open gripper
    arm.safe_move_to_position(q_tower_up)


def pick_and_place_tag6(team, H_tic_w_old, i, is_first_box=False):
    """
        compulsorily pick and place as tag6 with white face up
    """

    q_static_up_ref = np.array([-0.03450483, 0.22539452, -0.14064529, -1.77050944, 0.03436498, 1.99356833, 0.78106695])
    q_tower_up_ref_up = np.array([0.20041057, 0.11261646, 0.09532655, -1.90959175, -0.01184781, 2.02094364, 1.08574385])

    # 0 - create new H_tic_w with suitable direction
    H_tic_w = np.array([
        [-1, 0, 0, H_tic_w_old[0, 3]],
        [0, -1, 0, H_tic_w_old[1, 3]],
        [0, 0, 1, H_tic_w_old[2, 3]],
        [0, 0, 0, 1],
    ])

    # 1 - open gripper & move to static-up
    arm.open_gripper()
    H_rot = tag.get_H_staticRot("tag6")  # deal with the white faucet side case by case
    target = H_tic_w @ H_rot @ tag.H_upc_tic
    # seed = arm.neutral_position()  # use neutral configuration as seed
    seed = q_static_up_ref
    q_static_up = my_ik_move(target, seed)
    # q_static_up_ref = arm.get_positions()
    # print("q_static_up_ref = {}".format(q_static_up_ref))

    # 2 - line down
    target = H_tic_w @ H_rot
    seed = q_static_up
    my_ik_move(target, seed)

    # 3 - catch and back to static-up
    arm.exec_gripper_cmd(0.049, 15)
    arm.safe_move_to_position(q_static_up)  # ik_solver() back to static-up

    # 4 - go to tower-up
    H_twr_w = tag.get_H_twr_w(team, "tag6", i)  # real height
    target = H_twr_w @ tag.get_H_twrUp("tag6")  # to tower & white face up -> to tower up
    # q_tower_up = my_ik_move(target, q_static_up)
    if is_first_box is False:
        q_tower_up = my_ik_move(target, q_tower_up_ref_up)
    else:
        q_tower_up = q_tower_up_ref_up
        arm.safe_move_to_position(q_tower_up)

    # q_tower_up_ref_up = arm.get_positions()
    # print("q_tower_up_ref_up = {}".format(q_tower_up_ref_up))

    # 5 - line down & place
    target = H_twr_w
    my_ik_move(target, q_tower_up)

    # 6 - place and back to tower-up
    arm.open_gripper()  # open gripper
    arm.safe_move_to_position(q_tower_up)


def pick_and_place_first_box(team, tag_name, H_tic_w):
    """
        pick & place the first box specially in order to avoid the tower-up collision
    """

    flag = False

    for j in range(len(tag_name)):
        if tag_name[j] == "tag5":
            pick_and_place_tag6(team, H_tic_w[j], 0, True)
            tag_name.pop(j)
            H_tic_w.pop(j)
            flag = True
            break
    if flag is False:
        for j in range(len(tag_name)):
            if tag_name[j] == "tag6":
                pick_and_place_tag6(team, H_tic_w[j], 0, True)
                tag_name.pop(j)
                H_tic_w.pop(j)
                flag = True
                break
    if flag is False:
        pick_and_place_tag6(team, H_tic_w[0], 0, True)
        tag_name.pop(0)
        H_tic_w.pop(0)


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

    # 1 - get data of box center
    H_t0_c = tag.get_H_t0_c()  # get H_t0_c
    tag_name, H_tic_w = tag.get_tag_data(H_t0_c)  # get all data of tags
    print("H_name = {}".format(tag_name))
    # print("H_tic_w = {}".format(H_tic_w))

    # 2 - dealing with the first box
    pick_and_place_first_box(team, tag_name, H_tic_w)

    # 3 - dealing with the rest boxes
    for i in range(len(tag_name)):  # cuz you pop 1 data when i == 0!

        if tag_name[i] == "tag1" or tag_name[i] == "tag2" or tag_name[i] == "tag3" or tag_name[i] == "tag4":
            print("static side pick & place!")
            pick_and_place_tagx(team, tag_name[i], H_tic_w[i], i + 1)  # you have already towered the first box!

        elif tag_name[i] == "tag5" or tag_name[i] == "tag6":
            print("static side pick & place!")
            pick_and_place_tag6(team, H_tic_w[i], i + 1)  # you have already towered the first box!

        elif tag_name[i] == "tag7" or tag_name[i] == "tag8" or tag_name[i] == "tag9" or tag_name[i] == "tag10" or \
                tag_name[i] == "tag11" or tag_name[i] == "tag12":
            print("dynamic pick & place TBD!!!")

    # END STUDENT CODE !!!!!!!!!!!!!!!!!!!!!!!
