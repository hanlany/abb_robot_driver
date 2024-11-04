#!/usr/bin/env python

import os
from time import sleep
import numpy as np

import rospy
import actionlib
from pydrake.all import BsplineTrajectory, KinematicTrajectoryOptimization, Solve
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, JointTrajectoryControllerState

# planner params
nq = 6
qmin = np.array([-3.14159, -1.0995, -4.1015, -3.4906, -2.0071, -6.9813])
qmax  = np.array([3.14159, 1.9198, 0.9599, 3.4906, 2.0071, 6.9813])
dqmin = np.array([-2.618, -2.7925, -2.967, -5.585, -6.9813, -7.854])
dqmax = np.array([2.618, 2.7925, 2.967, 5.585, 6.9813, 7.854])
ddqmin = -10*np.ones(nq)
ddqmax = 10*np.ones(nq)
dddqmin = -100*np.ones(nq)
dddqmax = 100*np.ones(nq)
dt = 4e-3

# ROS subscriber
curr_state = JointTrajectoryControllerState()
# curr_state.actual.positions = np.zeros(nq) # initialize with zeros
# def robotStateCallback(data):
#     curr_state = data
#     print(curr_state)



if __name__ == "__main__":

    rospy.init_node('state_fb_test', anonymous=True)
    # rospy.Subscriber("/egm/joint_velocity_trajectory_controller/state", JointTrajectoryControllerState, robotStateCallback)
    client = actionlib.SimpleActionClient('egm/joint_velocity_trajectory_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    # client = actionlib.SimpleActionClient('egm/joint_position_trajectory_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    if not client.wait_for_server(rospy.Duration.from_sec(1.0)):
        print("joint_trajectory_action server not available")
    print("Connected to follow_joint_trajectory server")

    curr_state = rospy.wait_for_message("/egm/joint_velocity_trajectory_controller/state", JointTrajectoryControllerState)
    print(curr_state.actual.positions)

    # rospy.spin()
