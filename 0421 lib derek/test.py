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
    q_static_up_ref = []
    q_tower_up_ref_up = []

    if team == "red":
        q_static_up_ref = np.array(
            [-0.03450483, 0.22539452, -0.14064529, -1.77050944, 0.03436498, 1.99356833, 0.78106695])
        q_tower_up_ref_side = np.array(
            [-0.08188998, 0.04771562, -0.00321775, -1.92776809, 1.53728915, 1.49257634, -1.19063492])
    else:
        q_static_up_ref = np.array(
            [-0.03074419, -0.10781547, 0.23909529, -2.18551048, 0.02920031, 2.08056687, 2.88118093])
        q_tower_up_ref_side = np.array(
            [-0.86048895, 0.4852505, 0.26522349, -1.35016221, 1.28036102, 0.99785097, -1.08143])

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


def pick_and_place_tag6(team, H_tic_w, i, is_first_box=False):
    """
        pick and place as tag6 with white face up
    """
    q_static_up_ref = []
    q_tower_up_ref_up = []

    if team == "red":
        q_static_up_ref = np.array(
            [-0.03450483, 0.22539452, -0.14064529, -1.77050944, 0.03436498, 1.99356833, 0.78106695])
        q_tower_up_ref_up = np.array(
            [0.20041057, 0.11261646, 0.09532655, -1.90959175, -0.01184781, 2.02094364, 1.08574385])
    else:
        q_static_up_ref = np.array(
            [0.19376103, 0.03519338, 0.23693431, -2.02041482, -0.00933215, 2.05461371, -1.56212044])
        q_tower_up_ref_up = np.array(
            [-0.1333603, 0.11358311, -0.16514736, -1.90954666, 0.02063303, 2.02083333, 0.47894547])

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


def dynamic_pick_and_place(team, i, dynamic_start):
    """
        dynamic pick and place
    """

    isSucceed = False

    if team == "red":
        H_sweep = np.array([
            [0, 0, -1, 0],
            [0, -1, 0, 0.73],
            [-1, 0, 0, 0.225],
            [0, 0, 0, 1]
        ])
        seed = np.array([0.95148562, 1.17466788, 0.58635499, -1.05438613, 0.93178158, 1.5359424, 1.85779167])
        seed_twr = np.array([-0.08188998, 0.04771562, -0.00321775, -1.92776809, 1.53728915, 1.49257634, -1.19063492])  # q_tower_up_ref_side
    else:
        H_sweep = np.array([
            [0, 0, 1, 0],
            [0, -1, 0, -0.73],
            [1, 0, 0, 0.225],
            [0, 0, 0, 1]
        ])
        seed = np.array([-0.03222569, -1.16041228, -1.83640442, -2.1817086, 0.42630637, 1.99819784, -1.27256441])
        seed_twr = np.array([-0.86048895, 0.4852505, 0.26522349, -1.3501621, 1.28036102, 0.99785097, -1.08143])
        # seed = arm.neutral_position()

    # 1 - go to the plate
    arm.open_gripper()
    H_ready = tag.get_H_dynamicRot(team)
    target = H_ready  # sweep start position
    q_ready = my_ik_move(target, seed)
    # print("q_ready:\n", q_ready)

    # if perf_counter() - dynamic_start < 60:
    #     arm.neutral_position()
    #     return isSucceed

    # 2 - sweep H
    target_sweep = H_sweep
    q_dynamic_2 = my_ik_move(target_sweep, seed)
    # print("q_catch:\n", q_dynamic_2)

    # if perf_counter() - dynamic_start < 60:
    #     arm.neutral_position()
    #     return isSucceed

    # 3 - catch and check gripper
    arm.exec_gripper_cmd(0.045, 10)

    gripper_state = arm.get_gripper_state()
    # print("gripper_state = {}".format(gripper_state))
    gripper_position = gripper_state['position']
    # print("gripper_position = {}".format(gripper_position))
    gripper_dist = abs(gripper_position[1] - gripper_position[0])
    print("gripper_dist = {}".format(gripper_dist))

    is_gripped = False  # suppose it didn't catch
    while is_gripped is False:
        if gripper_dist <= 0.045 and perf_counter() - dynamic_start < 60:  # didn't catch & not exceed -> keep trying
            arm.open_gripper()
            start_time = perf_counter()
            while perf_counter() - start_time < 3 and perf_counter() - dynamic_start < 60:  # whatever
                print("wait to grip again...")
            arm.exec_gripper_cmd(0.045, 10)
        elif gripper_dist > 0.045 and perf_counter() - dynamic_start < 60:
            is_gripped = True
            isSucceed = True
        elif gripper_dist > 0.045 and perf_counter() - dynamic_start >= 60:
            is_gripped = True
            isSucceed = True
        else:  # didn't catch & exceed -> stop and back to neutral pos
            is_gripped = True  # jump out of the loop cuz it has already exceeded the time!
            isSucceed = False
    if isSucceed is False:
        arm.open_gripper()
        arm.neutral_position()
        return isSucceed

    # 4 - go to tower-up
    H_twr_w = tag.get_H_twr_w(team, "tag12", i)  # real height
    target_twrUp = H_twr_w @ tag.get_H_twrUp("tag12")  # to tower & white face up -> to tower up
    q_tower_up = my_ik_move(target_twrUp, seed_twr)

    # 5 - line down & place
    target_twr = H_twr_w
    my_ik_move(target_twr, q_tower_up)

    # 6 - place and back to tower-up
    arm.open_gripper()  # open gripper
    arm.safe_move_to_position(q_tower_up)

    return isSucceed


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
    # pick_and_place_first_box(team, tag_name, H_tic_w)
    floor_cnt = 1

    # 3 - dealing with the rest boxes
    dynamic_start = perf_counter()
    try_cnt = 0
    while try_cnt < 3 and perf_counter() - dynamic_start < 60:
        isSucceed = dynamic_pick_and_place(team, floor_cnt, dynamic_start)
        if isSucceed is True:
            floor_cnt = floor_cnt + 1
        try_cnt = try_cnt + 1

    # 4 - dealing with the rest boxes
    for i in range(len(tag_name)):  # cuz you pop 1 data when i == 0!

        if tag_name[i] == "tag1" or tag_name[i] == "tag2" or tag_name[i] == "tag3" or tag_name[i] == "tag4":
            print("static side pick & place!")
            pick_and_place_tagx(team, tag_name[i], H_tic_w[i], floor_cnt + i)  # you have already towered the first box!

        elif tag_name[i] == "tag5" or tag_name[i] == "tag6":
            print("static side pick & place!")
            pick_and_place_tag6(team, H_tic_w[i], floor_cnt + i)  # you have already towered the first box!

        elif tag_name[i] == "tag7" or tag_name[i] == "tag8" or tag_name[i] == "tag9" or tag_name[i] == "tag10" or \
                tag_name[i] == "tag11" or tag_name[i] == "tag12":
            print("dynamic pick & place TBD!!!")
            dynamic_start = perf_counter()
            dynamic_pick_and_place(team, floor_cnt + i, dynamic_start)

    # END STUDENT CODE !!!!!!!!!!!!!!!!!!!!!!!
