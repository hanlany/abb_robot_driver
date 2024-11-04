#! /usr/bin/env python

import roslib
import rospy
import actionlib
import pickle
import numpy as np

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

nq = 6
qmin = np.array([-3.14159, -1.0995, -4.1015, -3.4906, -2.0071, -6.9813])
qmax  = np.array([3.14159, 1.9198, 0.9599, 3.4906, 2.0071, 6.9813])
dqmin = np.array([-2.618, -2.7925, -2.967, -5.585, -6.9813, -7.854])
dqmax = np.array([2.618, 2.7925, 2.967, 5.585, 6.9813, 7.854])
ddqmin = -50*np.ones(nq) 
ddqmax = 50*np.ones(nq) 
dddqmin = -1e3*np.ones(nq) 
dddqmax = 1e3*np.ones(nq) 
dt = 4e-3

# test_goal = FollowJointTrajectoryGoal()
# goal_traj = JointTrajectory()
# goal_traj.header.frame_id = 'odom_combined'
# goal_traj.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
# p1 = JointTrajectoryPoint()
# p1.positions = [0, 0, 0, 0, 0, 0]
# # p1.time_from_start = 0.0
# p1.time_from_start.secs = 0
# p1.time_from_start.nsecs = 0
# p2 = JointTrajectoryPoint()
# p2.positions = [-0.454687, -0.0195174, 0.280673, 0.286559, -0.837908, -0.121895]
# # p2.time_from_start = 0.173677235
# p2.time_from_start.secs = 0
# p2.time_from_start.nsecs = 173677235
# goal_traj.points.append(p1)
# goal_traj.points.append(p2)   
# test_goal.trajectory = goal_traj

# with open('/home/shield/code/shield_ws/src/abb_robot_driver/abb_robot_bringup_examples/scripts/test_goal.pkl', 'rb') as file:
#     test_goal = pickle.load(file)

# with open('/home/shield/code/shield_ws/src/abb_robot_driver/abb_robot_bringup_examples/scripts/calib_start.pkl', 'rb') as file:
# with open('/home/shield/code/shield_ws/src/abb_robot_driver/abb_robot_bringup_examples/scripts/calib_end.pkl', 'rb') as file:
with open('/home/shield/code/shield_ws/src/abb_robot_driver/abb_robot_bringup_examples/scripts/tpva_to_home.pkl', 'rb') as file:
# with open('/home/shield/code/shield_ws/src/abb_robot_driver/abb_robot_bringup_examples/scripts/tpva_bspline.pkl', 'rb') as file:
# with open('/home/shield/code/shield_ws/src/abb_robot_driver/abb_robot_bringup_examples/scripts/tpva_ruckig.pkl', 'rb') as file:
    tpva = pickle.load(file)

test_goal = FollowJointTrajectoryGoal()
goal_traj = JointTrajectory()
goal_traj.header.frame_id = 'odom_combined'
goal_traj.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

j = 0
for r in tpva:
    p = JointTrajectoryPoint()
    p.time_from_start.secs = int(r[0])
    p.time_from_start.nsecs = int(rospy.Time.from_sec(r[0]).to_nsec() % 1e9)
    p.positions = [0,0,0,0,0,0]
    p.velocities = [0,0,0,0,0,0]
    p.accelerations = [0,0,0,0,0,0]

    for i in range(nq):
        p.positions[i] = r[1+i]
        p.velocities[i] = r[1+nq+i]
        p.accelerations[i] = r[1+2*nq+i]
    goal_traj.points.append(p)

test_goal.trajectory = goal_traj


if __name__ == '__main__':
    rospy.init_node('abb_traj_pb_test')
    client = actionlib.SimpleActionClient('egm/joint_velocity_trajectory_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    # client = actionlib.SimpleActionClient('egm/joint_position_trajectory_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    if not client.wait_for_server(rospy.Duration.from_sec(1.0)):
        print("joint_trajectory_action server not available")
    print("Connected to follow_joint_trajectory server")

    client.send_goal_and_wait(test_goal)
    client.wait_for_result(rospy.Duration.from_sec(5.0))
