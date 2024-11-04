#! /usr/bin/env python

import time
import rospy
import pickle

import numpy as np
from pydrake.all import BsplineTrajectory, KinematicTrajectoryOptimization, Solve
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal

q0 = np.array([0, 0, 0, 0, 0, 0])
# q0 = np.array([0, 1.57, -1.57, 0, 0, 0])
# qF = np.array([0, 0.0, 0, 0, 0, 0])
# qF = np.array([-0.8, 0.0, 0, 0, 0, 0])
# qF = np.array([0, 1.57, -1.57, 0, 0, 0])
# qF = np.array([0, 0, 0, 0, 0, 0])
# qF = np.array([0, 0, -1.57, 0, 0, 0])
qF = np.array([0, 0, -np.pi, 0, 0, 0])


wp = np.array([])
# wp = np.array([[0.75, 0.3, -0.5, 0, 0, 0],
#                [1.5, 0, 0, 0, 0, 0],
#                [0.75, -0.3, 0.5, 0, 0, 0],
#                [-0.75, 0.3, -0.5, 0, 0, 0],
#                [-1.5, 0, 0, 0, 0, 0],
#                [-0.75, -0.3, 0.5, 0, 0, 0]])
# wp = np.array([[0.75, 0.3, -0.5, 0, 0, 0],
#                [1.5, 0, 0, 0, 0, 0],
#                [0.75, -0.3, 0.5, 0, 0, 0],
#                [-0.75, 0.3, -0.5, 0, 0, 0],
#                [-1.5, 0, 0, 0, 0, 0],
#                [-0.75, -0.3, 0.5, 0, 0, 0],
#                [0.75, 0.3, -0.5, 0, 0, 0],
#                [1.5, 0, 0, 0, 0, 0],
#                [0.75, -0.3, 0.5, 0, 0, 0],
#                [-0.75, 0.3, -0.5, 0, 0, 0],
#                [-1.5, 0, 0, 0, 0, 0],
#                [-0.75, -0.3, 0.5, 0, 0, 0],
#                [0.75, 0.3, -0.5, 0, 0, 0],
#                [1.5, 0, 0, 0, 0, 0],
#                [0.75, -0.3, 0.5, 0, 0, 0],
#                [-0.75, 0.3, -0.5, 0, 0, 0],
#                [-1.5, 0, 0, 0, 0, 0],
#                [-0.75, -0.3, 0.5, 0, 0, 0],
#                [0.75, 0.3, -0.5, 0, 0, 0],
#                [1.5, 0, 0, 0, 0, 0],
#                [0.75, -0.3, 0.5, 0, 0, 0],
#                [-0.75, 0.3, -0.5, 0, 0, 0],
#                [-1.5, 0, 0, 0, 0, 0],
#                [-0.75, -0.3, 0.5, 0, 0, 0]])

# wp = np.array([[0, 0, -3.1415, 0, 0, 0],
#                [1.5, 0, 0, 0, 0, 0],
#                [-1.5, 0, 0, 0, 0, 0],
#                [0, 0, -3.1415, 0, 0, 0],
#                [1.5, 0, 0, 0, 0, 0],
#                [-1.5, 0, 0, 0, 0, 0],
#                [0, 0, -3.1415, 0, 0, 0],
#                [1.5, 0, 0, 0, 0, 0],
#                [-1.5, 0, 0, 0, 0, 0]])

# wp = np.array([[0, 0, -1.57, 0, 0, 0]])
# wp = np.array([[-1.54/2, 0, 0, 0, 0, 0]])


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

# trajopt = KinematicTrajectoryOptimization(nq, np.shape(wp)[0]*7, 6)
trajopt = KinematicTrajectoryOptimization(nq, 10, 4)
prog = trajopt.get_mutable_prog()
trajopt.AddDurationCost(10.0)
trajopt.AddPathLengthCost(1.0)
trajopt.AddPositionBounds(qmin, qmax)
trajopt.AddVelocityBounds(dqmin, dqmax)
trajopt.AddAccelerationBounds(ddqmin, ddqmax)
trajopt.AddJerkBounds(ddqmin, ddqmax)
trajopt.AddDurationConstraint(0.5, 25)
trajopt.AddPathPositionConstraint(q0, q0, 0)
trajopt.AddPathPositionConstraint(qF, qF, 1)
prog.AddQuadraticErrorCost(np.eye(nq), q0, trajopt.control_points()[:, -1])
trajopt.AddPathVelocityConstraint(np.zeros((nq, 1)), np.zeros((nq, 1)), 0) # start and end with zero velocity
trajopt.AddPathVelocityConstraint(np.zeros((nq, 1)), np.zeros((nq, 1)), 1) # start and end with zero velocity

ds = 1.0/(np.shape(wp)[0]+1)
for i, r in zip(range(np.shape(wp)[0]), wp):
    trajopt.AddPathPositionConstraint(r, r, (i+1)*ds)

# Solve once without the collisions and set that as the initial guess for
# the version with collisions.
result = Solve(prog)
if not result.is_success():
    print("Trajectory optimization failed, even without collisions!")
print("trajopt succeeded!")
op_traj = trajopt.ReconstructTrajectory(result)
print('traj duration: ', op_traj.end_time())

tpva = np.empty((int(np.ceil(op_traj.end_time()/dt))+1, 1+3*nq))
test_goal = FollowJointTrajectoryGoal()
goal_traj = JointTrajectory()
goal_traj.header.frame_id = 'odom_combined'
goal_traj.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

j = 0
for t in np.arange(op_traj.start_time(), op_traj.end_time(), dt):
    p = JointTrajectoryPoint()
    p.time_from_start.secs = int(t)
    p.time_from_start.nsecs = int(rospy.Time.from_sec(t).to_nsec() % 1e9)
    p.positions = [0,0,0,0,0,0]
    p.velocities = [0,0,0,0,0,0]
    p.accelerations = [0,0,0,0,0,0]

    tpva[j, 0] = t
    # print(t)

    for i in range(nq):
        p.positions[i] = op_traj.value(t)[i][0]
        p.velocities[i] = op_traj.EvalDerivative(t, 1)[i][0]
        p.accelerations[i] = op_traj.EvalDerivative(t, 2)[i][0]

        tpva[j, 1+i] = op_traj.value(t)[i][0]
        tpva[j, 1+nq+i] = op_traj.EvalDerivative(t, 1)[i][0]
        tpva[j, 1+2*nq+i] = op_traj.EvalDerivative(t, 2)[i][0]

    j += 1
    # p.positions = op_traj.value(t).tolist()
    # p.velocities = op_traj.EvalDerivative(t, 1).tolist()
    # p.accelerations = op_traj.EvalDerivative(t, 2).tolist()

    goal_traj.points.append(p)

p = JointTrajectoryPoint()
p.time_from_start.secs = int(op_traj.end_time())
p.time_from_start.nsecs = int(rospy.Time.from_sec(op_traj.end_time()).to_nsec() % 1e9)
p.positions = [0,0,0,0,0,0]
p.velocities = [0,0,0,0,0,0]
p.accelerations = [0,0,0,0,0,0]
tpva[j, 0] = op_traj.end_time()
for i in range(nq):
    p.positions[i] = op_traj.value(op_traj.end_time())[i][0]
    p.velocities[i] = op_traj.EvalDerivative(op_traj.end_time(), 1)[i][0]
    p.accelerations[i] = op_traj.EvalDerivative(op_traj.end_time(), 2)[i][0]

    tpva[j, 1+i] = op_traj.value(op_traj.end_time())[i][0]
    tpva[j, 1+nq+i] = op_traj.EvalDerivative(op_traj.end_time(), 1)[i][0]
    tpva[j, 1+2*nq+i] = op_traj.EvalDerivative(op_traj.end_time(), 2)[i][0]

goal_traj.points.append(p)
test_goal.trajectory = goal_traj
# print(test_goal)
    
# with open('/home/shield/code/shield_ws/src/abb_robot_driver/abb_robot_bringup_examples/scripts/test_goal.pkl', 'wb') as file:      
#     pickle.dump(test_goal, file)
with open('/home/shield/code/shield_ws/src/abb_robot_driver/abb_robot_bringup_examples/scripts/tpva_bspline.pkl', 'wb') as file:
    pickle.dump(tpva, file)
# with open('/home/shield/code/shield_ws/src/abb_robot_driver/abb_robot_bringup_examples/scripts/calib_end.pkl', 'wb') as file:
#     pickle.dump(tpva, file)
