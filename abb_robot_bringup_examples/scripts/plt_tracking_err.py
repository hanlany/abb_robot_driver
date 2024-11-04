#! /usr/bin/env python

import pickle
import rospy
import actionlib
import pickle
import numpy as np

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, FollowJointTrajectoryActionFeedback
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import matplotlib.pyplot as plt
import numpy as np

nq = 6


desired_p = []
desired_v = []
actual_p = []
actual_v = []
error_p = []
error_v = []

def jointFeedback(data):
    # print(data)
    print('got a point')
    desired_p.append(data.feedback.desired.positions)
    desired_v.append(data.feedback.desired.velocities)
    actual_p.append(data.feedback.actual.positions)
    actual_v.append(data.feedback.actual.velocities)
    error_p.append(data.feedback.error.positions) 
    error_v.append(data.feedback.error.velocities) 


def plotTracking(error, pid, symbol, ylabel):
    # Create a 2x3 grid of subplots
    fig, axes = plt.subplots(3, 2)

    # Flatten the axes array to iterate over subplots
    axes = axes.flatten()

    # Loop through the subplots and plot each column of the Nx6 matrix
    for i in range(6):
        ax = axes[i]
        ax.plot(error[:, i], label=symbol+str(i))
        ax.set_title(symbol)
        ax.set_xlabel('time')
        # ax.set_ylabel(ylabel)
        ax.set_ylabel(ylabel+str(i))
        # ax.legend()

    plt.tight_layout()

if __name__ == '__main__':
    rospy.init_node('traj_tracking_error', anonymous=True)
    rospy.Subscriber("/egm/joint_velocity_trajectory_controller/follow_joint_trajectory/feedback", FollowJointTrajectoryActionFeedback, jointFeedback)
    
    rospy.spin()

    desired_p = np.array(desired_p)
    desired_v = np.array(desired_v)
    actual_p = np.array(actual_p)
    actual_v = np.array(actual_v)
    error_p = np.array(error_p)
    error_v = np.array(error_v)


    # plotTracking(desired_p, actual_p, 'q', 'rad')
    # plotTracking(desired_v, actual_v, 'dq', 'rad/s')
    plotTracking(error_p, 0, 'q', 'rad')
    plotTracking(error_v, 1, 'dq', 'rad/s')

    # Adjust layout and show the plot
    plt.show()
