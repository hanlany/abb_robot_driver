#! /usr/bin/env python
# This example shows the usage of intermediate waypoints. It will only work with Ruckig Pro or enabled Online API (e.g. default when installed by pip / PyPI).
 
from copy import copy
from pathlib import Path
from sys import path
import numpy as np
import pickle

# Path to the build directory including a file similar to 'ruckig.cpython-37m-x86_64-linux-gnu'.
build_path = Path(__file__).parent.absolute().parent / 'build'
path.insert(0, str(build_path))
 
from ruckig import InputParameter, OutputParameter, Result, Ruckig
 
nq = 6
qmin = np.array([-3.14159, -1.0995, -4.1015, -3.4906, -2.0071, -6.9813])
qmax  = np.array([3.14159, 1.9198, 0.9599, 3.4906, 2.0071, 6.9813])
dqmin = np.array([-2.618, -2.7925, -2.967, -5.585, -6.9813, -7.854])
dqmax = np.array([2.618, 2.7925, 2.967, 5.585, 6.9813, 7.854])
ddqmin = -20*np.ones(nq) 
ddqmax = 20*np.ones(nq) 
dddqmin = -100*np.ones(nq) 
dddqmax = 100*np.ones(nq) 
dt = 4e-3

if __name__ == '__main__':
    # Create instances: the Ruckig OTG as well as input and output parameters
    otg = Ruckig(6, 4e-3, 50)  # DoFs, control cycle rate, maximum number of intermediate waypoints for memory allocation
    inp = InputParameter(6)  # DoFs
    out = OutputParameter(6, 50)  # DoFs, maximum number of intermediate waypoints for memory allocation
 
    inp.current_position = [0, 0, 0, 0, 0, 0]
    inp.current_velocity = [0, 0, 0, 0, 0, 0]
    inp.current_acceleration = [0, 0, 0, 0, 0, 0]

    # inp.intermediate_positions = [
    #     [0.75, 0.3, -0.5, 0, 0, 0],
    #     [1.5, 0, 0, 0, 0, 0],
    #     [0.75, -0.3, 0.5, 0, 0, 0],
    #     [-0.75, 0.3, -0.5, 0, 0, 0],
    #     [-1.5, 0, 0, 0, 0, 0],
    #     [-0.75, -0.3, 0.5, 0, 0, 0],
    #     [0.75, 0.3, -0.5, 0, 0, 0],
    #     [1.5, 0, 0, 0, 0, 0],
    #     [0.75, -0.3, 0.5, 0, 0, 0],
    #     [-0.75, 0.3, -0.5, 0, 0, 0],
    #     [-1.5, 0, 0, 0, 0, 0],
    #     [-0.75, -0.3, 0.5, 0, 0, 0],
    #     [0.75, 0.3, -0.5, 0, 0, 0],
    #     [1.5, 0, 0, 0, 0, 0],
    #     [0.75, -0.3, 0.5, 0, 0, 0],
    #     [-0.75, 0.3, -0.5, 0, 0, 0],
    #     [-1.5, 0, 0, 0, 0, 0],
    #     [-0.75, -0.3, 0.5, 0, 0, 0],
    #     [0.75, 0.3, -0.5, 0, 0, 0],
    #     [1.5, 0, 0, 0, 0, 0],
    #     [0.75, -0.3, 0.5, 0, 0, 0],
    #     [-0.75, 0.3, -0.5, 0, 0, 0],
    #     [-1.5, 0, 0, 0, 0, 0],
    #     [-0.75, -0.3, 0.5, 0, 0, 0]
    # ]

    # inp.intermediate_positions = [
    #     [0, 0, -3.1415, 0, 0, 0],
    #     [1.5, 0, 0, 0, 0, 0],
    #     [-1.5, 0, 0, 0, 0, 0],
    #     [0, 0, -3.1415, 0, 0, 0],
    #     [1.5, 0, 0, 0, 0, 0],
    #     [-1.5, 0, 0, 0, 0, 0],
    #     [0, 0, -3.1415, 0, 0, 0],
    #     [1.5, 0, 0, 0, 0, 0],
    #     [-1.5, 0, 0, 0, 0, 0]
    # ]
 
    inp.target_position = [0, 0, np.pi, 0, 0, 0]
    inp.target_velocity = [0, 0, 0, 0, 0, 0]
    inp.target_acceleration = [0, 0, 0, 0, 0, 0]
 
    inp.max_velocity = [2.618, 2.7925, 2.967, 5.585, 6.9813, 7.854]
    inp.max_acceleration = [50, 50, 50, 50, 50, 50]
    # inp.max_jerk = [1e3, 1e3, 1e3, 1e3, 1e3, 1e3]
    inp.max_jerk = [5e2, 5e2, 5e2, 5e2, 5e2, 5e2]
    # inp.max_jerk = [1e2, 1e2, 1e2, 1e2, 1e2, 1e2]
    # inp.max_jerk = [50, 50, 50, 50, 50, 50]
 
 
    print('\t'.join(['t'] + [str(i) for i in range(otg.degrees_of_freedom)]))
 
    # Generate the trajectory within the control loop
    first_output, out_list = None, []
    res = Result.Working
    while res == Result.Working:
        res = otg.update(inp, out)
 
        # print('\t'.join([f'{out.time:0.3f}'] + [f'{p:0.3f}' for p in out.new_position]))
        print('\t'.join([f'{out.time:0.3f}'] + [f'{p:0.3f}' for p in out.new_velocity]))
        out_list.append(copy(out))
 
        out.pass_to_input(inp)
 
        if not first_output:
            first_output = copy(out)
 
    tpva = np.empty((len(out_list), 1+3*nq))
    j=0
    for j, o in zip(range(len(out_list)), out_list):
        tpva[j, 0] = o.time
        for k, p, v, a in zip(range(nq), o.new_position, o.new_velocity, o.new_acceleration):
            tpva[j, 1+k] = p
            tpva[j, 1+nq+k] = v
            tpva[j, 1+2*nq+k] = a        


    print(f'Calculation duration: {first_output.calculation_duration:0.1f} [µs]')
    print(f'Trajectory duration: {first_output.trajectory.duration:0.4f} [s]')
 
    with open('/home/shield/code/shield_ws/src/abb_robot_driver/abb_robot_bringup_examples/scripts/tpva_ruckig.pkl', 'wb') as file:      
        pickle.dump(tpva, file)

    # Plot the trajectory
    # path.insert(0, str(Path(__file__).parent.absolute().parent / 'test'))
    # from plotter import Plotter
 
    # print(Path(__file__).parent.absolute())
    # Plotter.plot_trajectory(Path(__file__).parent.absolute() / 'tpva.pdf', otg, inp, out_list, plot_jerk=False)
