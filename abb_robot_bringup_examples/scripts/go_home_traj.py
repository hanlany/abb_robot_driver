#! /usr/bin/env python

import time
import rospy
import pickle

import numpy as np
from pydrake.all import BsplineTrajectory, KinematicTrajectoryOptimization, Solve
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, JointTrajectoryControllerState
from sensor_msgs.msg import JointState


rospy.init_node('go_home_traj', anonymous=True)
# rospy.spin()
curr_state = rospy.wait_for_message("/egm/joint_states", JointState, timeout=1.0)
q0 = curr_state.position
print(q0)

# q0 = np.array([0, 0, -3.1415, 0, 0, 0])
# q0 = np.array([0.00058822512060402, 1.569683791249516, -1.5705770154791285, 0.0007466675359856412, 4.436155628132667e-05, -0.000591564214865302])
# q0 = np.array([0.27330770581770886, 1.0466748392536425, -0.6409724074794156, -0.1989392819171392, 0.9698131472957838, -1.8029857574881636])
qF = np.array([0, 0, 0, 0, 0, 0])  ############# HOME


nq = 6
qmin = np.array([-3.14159, -1.0995, -4.1015, -3.4906, -2.0071, -6.9813])
qmax  = np.array([3.14159, 1.9198, 0.9599, 3.4906, 2.0071, 6.9813])
dqmin = np.array([-2.618, -2.7925, -2.967, -5.585, -6.9813, -7.854])
dqmax = np.array([2.618, 2.7925, 2.967, 5.585, 6.9813, 7.854])
ddqmin = -2*np.ones(nq) 
ddqmax = 2*np.ones(nq) 
dddqmin = -2*np.ones(nq) 
dddqmax = 2*np.ones(nq) 
dt = 4e-3

trajopt = KinematicTrajectoryOptimization(nq, 7, 5)
prog = trajopt.get_mutable_prog()
trajopt.AddDurationCost(1.0)
trajopt.AddPathLengthCost(1.0)
trajopt.AddPositionBounds(qmin, qmax)
trajopt.AddVelocityBounds(dqmin, dqmax)
trajopt.AddAccelerationBounds(ddqmin, ddqmax)
trajopt.AddDurationConstraint(0.1, 5)
trajopt.AddPathPositionConstraint(q0, q0, 0)
trajopt.AddPathPositionConstraint(qF, qF, 1)
prog.AddQuadraticErrorCost(np.eye(nq), q0, trajopt.control_points()[:, -1])
trajopt.AddPathVelocityConstraint(np.zeros((nq, 1)), np.zeros((nq, 1)), 0) # start and end with zero velocity
trajopt.AddPathVelocityConstraint(np.zeros((nq, 1)), np.zeros((nq, 1)), 1) # start and end with zero velocity

# Solve once without the collisions and set that as the initial guess for
# the version with collisions.
result = Solve(prog)
if not result.is_success():
    print("Trajectory optimization failed, even without collisions!")
print("trajopt succeeded!")
op_traj = trajopt.ReconstructTrajectory(result)
print('traj duration: ', op_traj.end_time())

tpva = np.empty((int(np.ceil(op_traj.end_time()/dt))+1, 1+3*nq))

j = 0
for t in np.arange(op_traj.start_time(), op_traj.end_time(), dt):
    tpva[j, 0] = t

    for i in range(nq):
        tpva[j, 1+i] = op_traj.value(t)[i][0]
        tpva[j, 1+nq+i] = op_traj.EvalDerivative(t, 1)[i][0]
        tpva[j, 1+2*nq+i] = op_traj.EvalDerivative(t, 2)[i][0]

    j += 1

tpva[j, 0] = op_traj.end_time()
for i in range(nq):
    tpva[j, 1+i] = op_traj.value(op_traj.end_time())[i][0]
    tpva[j, 1+nq+i] = op_traj.EvalDerivative(op_traj.end_time(), 1)[i][0]
    tpva[j, 1+2*nq+i] = op_traj.EvalDerivative(op_traj.end_time(), 2)[i][0]

    
with open('/home/shield/code/shield_ws/src/abb_robot_driver/abb_robot_bringup_examples/scripts/tpva_to_home.pkl', 'wb') as file:      
    pickle.dump(tpva, file)
