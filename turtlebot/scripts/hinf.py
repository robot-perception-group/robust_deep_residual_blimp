#! /usr/bin/env python

from TurtleInf import *
from control import *
from controller_simulation import ControllerSimulation
import os
import scipy.io


if __name__ == '__main__':
    # Define object
    obj = TurtleInf()
    dt = 1 / obj.rate
    os.chdir("/home/yang/catkin_ws/src/turtlebot/scripts/")
    content = scipy.io.loadmat("matlab.mat")
    # SISO-hinf for v
    K1_A = content['K1_A']
    K1_B = content['K1_B']
    K1_C = content['K1_C']
    K1_D = content['K1_D']
    K1 = ss(K1_A, K1_B, K1_C, K1_D)
    Kd1 = c2d(sysc=K1, Ts=dt)
    # SISO-hinf for omega
    K2_A = content['K2_A']
    K2_B = content['K2_B']
    K2_C = content['K2_C']
    K2_D = content['K2_D']
    K2 = ss(K2_A, K2_B, K2_C, K2_D)
    Kd2 = c2d(sysc=K2, Ts=dt)
    # Define controller object
    v_cs = ControllerSimulation(a=Kd1.A, b=Kd1.B, c=Kd1.C, d=Kd1.D)
    omega_cs = ControllerSimulation(a=Kd2.A, b=Kd2.B, c=Kd2.C, d=Kd2.D)
    # Run controller simulation
    i = 0
    while i <= obj.total_timesteps:
        e = obj.get_e()
        if obj.R > 0.2:
            action = np.array([v_cs.sim(u=np.array([e[0]]), y_max=10, y_min=2), omega_cs.sim(u=np.array([e[1]]))], dtype=object)
            obj.get_noise(v_max=1, w_max=2)
            if i % 50 == 0:
                print("Du/u in %: ", 100*obj.noise/action)
            action = action + obj.noise
            obj.move_one_step(action=action)
        else:
            obj.stop()
            obj.reset()
        rospy.Rate(obj.rate).sleep()
        i = i + 1
