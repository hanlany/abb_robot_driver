import pickle

import matplotlib.pyplot as plt
import seaborn as sns
import numpy as np

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

tpva=[]
# with open('/home/shield/code/shield_ws/src/abb_robot_driver/abb_robot_bringup_examples/scripts/tpva_bspline.pkl', 'rb') as file:
with open('/home/shield/code/shield_ws/src/abb_robot_driver/abb_robot_bringup_examples/scripts/tpva_ruckig.pkl', 'rb') as file:
    tpva = pickle.load(file)

# plotting
fig, axes = plt.subplots(2, 3, figsize=(12, 8))
for i, j, k in zip(range(nq), [0,0,0,1,1,1], [0,1,2,0,1,2]):
    sns.lineplot(x=tpva[:, 0], y=tpva[:, 1+i], ax=axes[j, k])
    sns.lineplot(x=tpva[:, 0], y=qmin[i], ax=axes[j, k])
    sns.lineplot(x=tpva[:, 0], y=qmax[i], ax=axes[j, k])
    axes[j, k].set_title('jpos ' + str(i+1))

fig, axes = plt.subplots(2, 3, figsize=(12, 8))
for i, j, k in zip(range(nq), [0,0,0,1,1,1], [0,1,2,0,1,2]):
    sns.lineplot(x=tpva[:, 0], y=tpva[:, 1+nq+i], ax=axes[j, k])
    sns.lineplot(x=tpva[:, 0], y=dqmin[i], ax=axes[j, k])
    sns.lineplot(x=tpva[:, 0], y=dqmax[i], ax=axes[j, k])
    axes[j, k].set_title('jvel ' + str(i+1))

fig, axes = plt.subplots(2, 3, figsize=(12, 8))
for i, j, k in zip(range(nq), [0,0,0,1,1,1], [0,1,2,0,1,2]):
    sns.lineplot(x=tpva[:, 0], y=tpva[:, 1+2*nq+i], ax=axes[j, k])
    sns.lineplot(x=tpva[:, 0], y=ddqmin[i], ax=axes[j, k])
    sns.lineplot(x=tpva[:, 0], y=ddqmax[i], ax=axes[j, k])
    axes[j, k].set_title('jacc ' + str(i+1))

# Adjust the spacing between subplots
plt.tight_layout()

# Display the plots
plt.show()
