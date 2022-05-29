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
import math
from time import perf_counter

# instantiated object!
detector = ObjectDetector()


class Tag:

    def __init__(self):
        """
            H_t0_w & H_tic_ti: only used in get_H_t0_c()
        """

        self.H_t0_w = np.array([
            [1, 0, 0, -0.500],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1],
        ])

        self.H_tic_ti = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, -0.025],  # half of the box side length
            [0, 0, 0, 1],
        ])

        self.H_upc_tic = np.array([  # H - move to up box center 10cm
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, -0.1],
            [0, 0, 0, 1],
        ])

    def get_H_t0_c(self):
        """
            get H matrix of tag 0 with respect to camera frame
        """

        H_t0_c = []

        for (name, pose) in detector.get_detections():
            if name == "tag0":
                H_t0_c = pose
                # print("H_t0_c = {}".format(H_t0_c))
                break

        return H_t0_c

    def getTagCenterPos(self, H_t0_c, H_ti_c):
        """
            input: 4x4 matrix - tag i frame with respect to camera frame
            output: 4x4 matrix - tag i box's center frame with respect to world frame
        """

        H_tic_w = self.H_t0_w @ np.linalg.inv(H_t0_c) @ H_ti_c @ self.H_tic_ti
        # print(H_tic_w)

        return H_tic_w

    def get_tag_data(self, H_t0_c):
        """
            get tag_name & H_ti_c & H_tic_w without Tag0!!!!
        """

        tag_name = []
        H_ti_c = []
        H_tic_w = []

        for (name, pose) in detector.get_detections():
            print("H_name = {}".format(name))
            # print("pose c = {}".format(pose))
            if tag_name != "tag0":
                tag_name.append(name)
                # H_ti_c.append(pose)
                H_tic_w.append(self.getTagCenterPos(H_t0_c, pose))

        return tag_name, H_tic_w

    def get_H_staticRot(self, tag_name):
        """
            deal with the white face side case by case
        """

        H_rot = np.identity(4)

        if tag_name == "tag0":
            pass

        elif tag_name == "tag1" or tag_name == "tag2" or tag_name == "tag3" or tag_name == "tag4":
            print("side case!")
            H_rot = np.array([
                [0, 1, 0, 0],
                [1, 0, 0, 0],
                [0, 0, -1, 0],
                [0, 0, 0, 1],
            ])

        elif tag_name == "tag5":
            print("down case! - TBD!!!!")

            # H_1 = np.array([
            #     [-1, 0, 0, 0],
            #     [0, 1, 0, 0],
            #     [0, 0, -1, 0],
            #     [0, 0, 0, 1],
            # ])

            H_1 = np.array([
                [1, 0, 0, 0],
                [0, -1, 0, 0],
                [0, 0, -1, 0],
                [0, 0, 0, 1],
            ])

            H_rotate = np.array([
                [math.sqrt(1/2), 0, math.sqrt(1/2), 0],
                [0, 1, 0, 0],
                [-math.sqrt(1/2), 0, math.sqrt(1/2), 0],
                [0, 0, 0, 1],
            ])

            H_translation = np.array([
                [1, 0, 0, 0.1 * math.sqrt(1/2)],
                [0, 1, 0, 0],
                [0, 0, 1, -0.1 * math.sqrt(1/2)],
                [0, 0, 0, 1],
            ])

            H_rot = H_1 @ H_rotate @ H_translation


        elif tag_name == "tag6":
            print("up case!")
            H_rot = np.array([
                [1, 0, 0, 0],
                [0, -1, 0, 0],
                [0, 0, -1, 0],
                [0, 0, 0, 1],
            ])

        return H_rot

    def get_H_twr_w(self, tag_name, i):
        """
            tower & rotate to standard parallel direction?
        """

        H_twr_w = np.identity(4)

        if tag_name == "tag0":
            pass

        elif tag_name == "tag1" or tag_name == "tag2" or tag_name == "tag3" or tag_name == "tag4":
            print("side case!")
            H_twr_w = np.array([  # define tower placement height
                [0, 1, 0, .562],
                [0, 0, 1, .169],
                [1, 0, 0, .2 + 0.005 + (i + 1) * 0.05],
                [0, 0, 0, 1],
            ])

        elif tag_name == "tag5":
            print("down case! - TBD too!!!!")
            H_twr_w = np.array([  # define tower placement height
                [1, 0, 0, .562],
                [0, -1, 0, .169],
                [0, 0, -1, .2 + 0.01 + (i + 1) * 0.05],
                [0, 0, 0, 1],
            ])

        elif tag_name == "tag6":
            print("up case!")
            H_twr_w = np.array([  # define tower placement height
                [1, 0, 0, .562],
                [0, -1, 0, .169],
                [0, 0, -1, .2 + 0.01 + (i + 1) * 0.05],
                [0, 0, 0, 1],
            ])

        return H_twr_w

    def get_H_twrUp(self, tag_name):
        """
            tower & rotate to standard parallel direction?
        """

        H_twrUp = np.identity(4)

        if tag_name == "tag0":
            pass

        elif tag_name == "tag1" or tag_name == "tag2" or tag_name == "tag3" or tag_name == "tag4":
            print("side case!")
            H_twrUp = np.array([
                [1, 0, 0, 0.2],
                [0, 1, 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1],
            ])

        elif tag_name == "tag5":
            print("down case! - TBD too!!!!")
            H_twrUp = np.array([
                [1, 0, 0, 0],
                [0, 1, 0, 0],
                [0, 0, 1, -0.2],
                [0, 0, 0, 1],
            ])

        elif tag_name == "tag6":
            print("up case!")
            H_twrUp = np.array([
                [1, 0, 0, 0],
                [0, 1, 0, 0],
                [0, 0, 1, -0.2],
                [0, 0, 0, 1],
            ])

        return H_twrUp
