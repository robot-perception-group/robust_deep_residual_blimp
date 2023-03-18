#! /usr/bin/env python

import turtlesim.srv
from TurtleEnv import *


class TurtleInf(TurtleEnv):
    def __init__(self, total_timesteps=int(5e7), x_max=11.1, y_max=11.1, rate=100):
        random.seed(0)
        rospy.init_node('hinf_turtle_node')
        # initialize the turtlesim to the start configuration and set the background color to the value of the background
        reset_client = rospy.ServiceProxy('/reset', Empty)
        reset_object = EmptyRequest()
        reset_client(reset_object)
        # Observation bound
        self.x_max = x_max
        self.y_max = y_max
        self.pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=1)
        self.pos = Pose()
        rospy.Subscriber('/turtle1/pose', Pose, self.callback)
        self.rate = rate
        # Random goal
        self.goal_x1 = random.uniform(0, self.x_max)
        self.goal_x2 = random.uniform(0, self.x_max)
        self.goal_y1 = random.uniform(0, self.y_max)
        self.goal_y2 = random.uniform(0, self.y_max)
        self.number = 1
        # Mark target
        self.get_target()
        # print("Goal position: ", [self.goal_x, self.goal_y])
        self.R = self.distance1()
        self.total_timesteps = total_timesteps
        self.noise = np.zeros((2,))

    def get_x(self):
        self.R = self.distance1()

    def get_e(self):
        self.get_x()
        # range ]-inf, 0]
        delta_R = 0 - self.R
        # range: ]-2pi, 2pi[
        delta_theta = self.sigma() - self.pos.theta
        # range ]-pi, pi]
        if delta_theta <= -np.pi:
            delta_theta = delta_theta + 2*np.pi
        elif delta_theta > np.pi:
            delta_theta = delta_theta - 2*np.pi
        return [delta_R, delta_theta]


    def get_noise(self, v_max=1, w_max=1):
        self.noise = self.noise + [random.normalvariate(mu=0, sigma=1), random.normalvariate(mu=0, sigma=1)]
        if abs(self.noise[0]) >= v_max:
            self.noise[0] = np.sign(self.noise[0]) * v_max
        if abs(self.noise[1]) >= w_max:
            self.noise[1] = np.sign(self.noise[1]) * w_max

    def stop(self):
        cmd = Twist()
        cmd.linear.x = 0
        cmd.angular.z = 0
        self.pub.publish(cmd)

    def total_response(self, i_here, e_here, u_here):
        r = 0
        for k in range(i_here + 1):
            r = e_here[k] * u_here[i_here + 1 - k] + r
        return r

    def move_one_step(self, action):
        cmd = Twist()
        cmd.linear.x = action[0]
        cmd.angular.z = action[1]
        self.pub.publish(cmd)

    def reset(self):
        # Reset background
        reset_client = rospy.ServiceProxy('/clear', Empty)
        reset_object = EmptyRequest()
        reset_client(reset_object)
        self.now = rospy.Time.now()
        self.step_count = 0
        # Clear old target
        kill_client = rospy.ServiceProxy('/kill', turtlesim.srv.Kill)
        kill_obj = turtlesim.srv.KillRequest()
        kill_obj.name = "target1"
        kill_client(kill_obj)
        # Random goal
        self.goal_x1 = self.goal_x2
        self.goal_y1 = self.goal_y2
        self.goal_x2 = random.uniform(0, self.x_max)
        self.goal_y2 = random.uniform(0, self.y_max)
        # Mark new target
        self.get_target()
        # print("Goal position: ", [self.goal_x, self.goal_y])
        observation = self._get_obs()
        return observation
