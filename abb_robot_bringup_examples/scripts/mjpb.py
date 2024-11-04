
import os
import mujoco as mp
from mujoco import MjData, MjModel
import mujoco_viewer
from time import sleep
import numpy as np
from numpy import genfromtxt
import pickle

static_planner = False

if static_planner:
  # dt = 1e-2
  dt = 0.05
  # dt = 6e-3
else:
  # dt = 6e-3
  dt = 4e-3
  # dt = 0.5
  # dt = 1

model_dir = '/home/shield/code/shield_obs_ws/src/parallel_search/third_party/mujoco-2.3.2/model/abb/irb_1600'
mjcf = 'irb1600_6_12_realshield_obs.xml'
mjcf_arm = 'irb1600_6_12_realshield_obs.xml'

# with open('/home/shield/code/shield_ws/src/abb_robot_driver/abb_robot_bringup_examples/scripts/tpva_to_home.pkl', 'rb') as file:
with open('/home/shield/code/shield_ws/src/abb_robot_driver/abb_robot_bringup_examples/scripts/tpva_bspline.pkl', 'rb') as file:
    tpva = pickle.load(file)

# just using arm model for calculating ee traj
arm_model = MjModel.from_xml_path(os.path.join(model_dir, mjcf_arm))
arm_data = MjData(arm_model)
viewer = mujoco_viewer.MujocoViewer(arm_model, arm_data)

nq = arm_model.nq

def upsampleTraj(traj, dx=0.1):
  col = np.shape(traj)[1]
  uptraj = np.empty((0, col))
  for i in range(np.shape(traj)[0]-1):
    if np.array_equal(traj[i,:], -1*np.ones((arm_model.nq,))):
      uptraj = np.append(uptraj, traj[i,:][np.newaxis,:], axis=0)
      continue
    if np.array_equal(traj[i+1,:], -1*np.ones((arm_model.nq,))):
      continue

    x1 = traj[i,:]
    x2 = traj[i+1,:]
    dist = np.linalg.norm(x2-x1)
    N = int(dist/dx)
    samp = np.linspace(x1, x2, N)
    uptraj = np.append(uptraj, samp, axis=0)

  return uptraj

traj = tpva[:, 1:1+nq]
if static_planner:
  traj = upsampleTraj(traj, 1e-2)

skp = 0
i=0
while i <= np.shape(traj)[0]:
  if i == np.shape(traj)[0]:
    i=0
    continue
  skp += 1
  # if skp % 2 == 0:
  #   continue

  if np.array_equal(traj[i,:], traj[-1,:]):
    sleep(2)
    i+=1
    continue
  if viewer.is_alive:
    arm_data.qpos[:] = traj[i,:]
    # print(traj[i,:])
    mp.mj_step(arm_model, arm_data)
    viewer.render()
    sleep(dt)
  else:
      break
  i+=1

# close
viewer.close()

