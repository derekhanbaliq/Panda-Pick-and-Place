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

    def get_H_t0_c(self):
        """
            get H matrix of tag 0 with respect to camera frame
        """

        H_t0_c = []

        for (name, pose) in detector.get_detections():
            if name == "tag0":
                H_t0_c = pose
                print("H_t0_c = {}".format(H_t0_c))
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
            get tag_name & H_ti_c & H_tic_w
        """

        tag_name = []
        H_ti_c = []
        H_tic_w = []

        for (name, pose) in detector.get_detections():
            print("H_name c = {}".format(name))
            print("pose c = {}".format(pose))
            tag_name.append(name)
            H_ti_c.append(pose)
            H_tic_w.append(self.getTagCenterPos(H_t0_c, pose))

        return tag_name, H_ti_c, H_tic_w
