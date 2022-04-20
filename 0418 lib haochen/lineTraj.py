import sys
import numpy as np
# from copy import deepcopy

import rospy
import roslib

# Common interfaces for interacting with both the simulation and real environments!
from core.interfaces import ArmController
from core.interfaces import ObjectDetector
from core.utils import time_in_seconds

# for timing that is consistent with simulation or real time as appropriate
# from core.utils import time_in_seconds
from lib.IK_velocity import IK_velocity
# The library you implemented over the course of this semester!
from lib.calculateFK import FK
from lib.calcJacobian import calcJacobian
from lib.solveIK import IK
# from lib.rrt import rrt
# from lib.loadmap import loadmap
from math import cos, sin, pi
import matplotlib.pyplot as plt
import geometry_msgs


class Movement:

    # def __init__(self, q1, q2, q3):
    def __init__(self):
        self.last_iteration_time = None
        self.dt = None
        self.last_iteration_timelast_iteration_time = None
        # self.station1 = q1
        # self.station2 = q2
        # self.station3 = q3

        """
        Demo class for testing Jacobian and Inverse Velocity Kinematics.
        Contains trajectories and controller callback function
        """
        # active = False  # When to stop commanding arm
        self.start_time = 0  # start time
        dt = 0.03  # constant for how to turn velocities into positions
        self.fk = FK()
        point_pub = rospy.Publisher('/vis/trace', geometry_msgs.msg.PointStamped, queue_size=10)
        counter = 0
        x0 = np.array([0.307, 0, 0.487])  # corresponds to neutral position
        last_iteration_time = None

        ##################
        ## TRAJECTORIES ##
        ##################

    @staticmethod
    def line(t, x0, f=1, L=.5):
        """
        Calculate the position and velocity of the line trajector

        Inputs:
        t - time in sec since start
        f - frequency in Hz of the line trajectory
        L - length of the line in meters

        Outputs:
        xdes = 0x3 np array of target end effector position in the world frame
        vdes = 0x3 np array of target end effector linear velocity in the world frame
        """
        ## STUDENT CODE GOES HERE

        # TODO: replace these!

        # X direction
        # xdes = x0 + np.array([L*sin(f*t), 0, 0])
        # vdes = np.array([-f*L*cos(f*t),0,0])

        # Y direction
        # xdes = x0 + np.array([0, L*sin(f*t), 0])
        # vdes = np.array([0, -f*L*cos(f*t),0])

        # Z direction
        if sin(f * t)>0:
            a = -sin(f * t)
            b = -cos(f * t)
        if sin(f * t)<0:
            a = sin(f * t)
            b = cos(f * t)
        if sin(f * t)==0:
            a = 0
            b = 1

        xdes = x0 + np.array([0, 0, L * a])
        vdes = np.array([0, 0, -f * L * b])
        ades = np.array([0, 0, f*f*L*a])

        # YZ direction
        # xdes = x0 + np.array([0, L*sin(f*t), L*sin(f*t)])
        # vdes = np.array([0, -f*L*cos(f*t), -f*L*cos(f*t)])

        # XZ direction
        # xdes = x0 + np.array([L*sin(f*t), 0, L*sin(f*t)])
        # vdes = np.array([-f*L*cos(f*t), 0, -f*L*cos(f*t)])

        # XY direction
        # xdes = x0 + np.array([L*sin(f*t), L*sin(f*t), 0])
        # vdes = np.array([-f*L*cos(f*t), -f*L*cos(f*t), 0])

        # XYZ direction
        # xdes = x0 + np.array([L*sin(f*t), L*sin(f*t), L*sin(f*t)])
        # vdes = np.array([-f*L*cos(f*t), -f*L*cos(f*t), -f*L*cos(f*t)])
        ## END STUDENT CODE

        return xdes, vdes

        ###################
        ## VISUALIZATION ##
        ###################

    def show_ee_position(self):
        msg = geometry_msgs.msg.PointStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'endeffector'
        msg.point.x = 0
        msg.point.y = 0
        msg.point.z = 0
        self.point_pub.publish(msg)

    ################
    ## CONTROLLER ##
    ################

    def follow_trajectory(self, desired_altitude, x0):

        arm = ArmController()

        while 1:
            t = time_in_seconds() - self.start_time

            # get desired trajectory position and velocity
            xdes, vdes = Movement.line(t, x0)

            # get current end effector position
            q = arm.get_positions()
            joints, T0e = self.fk.forward(q)
            x = (T0e[0:3, 3])

            # First Order Integrator, Proportional Control with Feed Forward
            kp = 3
            v = vdes + kp * (xdes - x)

            # Velocity Inverse Kinematics
            dq = IK_velocity(q, v, np.array([np.nan, np.nan, np.nan])) * 0.5  # !!!!!

            # Get the correct timing to update with the robot
            if self.last_iteration_time is None:
                self.last_iteration_time = time_in_seconds()

            self.dt = time_in_seconds() - self.last_iteration_time
            self.last_iteration_time = time_in_seconds()

            new_q = q + self.dt * dq

            arm.safe_set_joint_positions_velocities(new_q, dq)

            # Downsample visualization to reduce rendering overhead
            # self.counter = self.counter + 1
            # if self.counter == 10:
            #     self.show_ee_position()
            #     self.counter = 0

            altitude = self.fk.forward(new_q)[0][-1][-1]  # joint positions' end effector's z
            if desired_altitude - 0.05 < altitude < desired_altitude + 0.05:
                break

    @staticmethod
    def move_to_station(goal_station):
        arm.safe_move_to_position(goal_station)

    @staticmethod
    def move_to_home_station(self):
        home_config = []  # CHANGE NEEDED
        arm.safe_move_to_position(home_config)

    @staticmethod
    def rotate_claw(camera_tips, Q):
        if z_axis is upward:
            pass
        if z_axis is not upward:
            end_eff, T0e = fk.calculateFK(Q)
            Rotationmatrix = [[cos(theta), 0, -sin(theta)], [0, 1, 0], [sin(theta), 0, cos(theta)]]
            # add other rotation matrices
            R = T0e[0:3, 0:3] * RotationMatrix
            # now convert R to qq
            arm.same_move_to_position(qq)


if __name__ == "__main__":

    if len(sys.argv) < 2:
        print(
            "usage:\n\tpython jacobianDemo.py line\n\tpython jacobianDemo.py ellipse\n\tpython jacobianDemo.py eight\n\tpython jacobianDemo.py spiral")
        exit()

    rospy.init_node("follower")

    JD = Movement()

    if sys.argv[1] == 'line':
        callback = lambda state: JD.follow_trajectory(state, JacobianDemo.line)
    elif sys.argv[1] == 'ellipse':
        callback = lambda state: JD.follow_trajectory(state, JacobianDemo.ellipse)
    elif sys.argv[1] == 'spiral':
        callback = lambda state: JD.follow_trajectory(state, JacobianDemo.spiral)
    elif sys.argv[1] == 'eight':
        callback = lambda state: JD.follow_trajectory(state, JacobianDemo.eight)
    else:
        print("invalid option")
        exit()

    arm = ArmController(on_state_callback=callback)

    # reset arm
    print("resetting arm...")
    arm.safe_move_to_position(arm.neutral_position())

    # start tracking trajectory
    JD.active = True
    JD.start_time = time_in_seconds()

    input("Press Enter to stop")
