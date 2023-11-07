import gym
import rospy
import random
import torch
import os
import scipy.io
from math import pow, sqrt
from gym import spaces
from control import *
from turtlesim.srv import Spawn, SpawnRequest, Kill, KillRequest, TeleportAbsolute, TeleportAbsoluteRequest
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty, EmptyRequest
from controller_simulation import ControllerSimulation
from model import scale_a


class PIDController:
    def __init__(
            self,
            pid_param=np.array([1.0, 0.2, 0.05]),
            gain=1.0,
            offset=0.0,
            delta_t=0.01,
            i_from_sensor=False,
            d_from_sensor=False,
    ):
        self.pid_param = pid_param
        self.gain = gain
        self.offset = offset
        self.delta_t = delta_t
        self.i_from_sensor = i_from_sensor
        self.d_from_sensor = d_from_sensor

        self.err_sum, self.prev_err = 0.0, 0.0
        self.windup = 0.0

    def action(self, err, err_i=0, err_d=0):
        if not self.i_from_sensor:
            self.err_sum += err * self.delta_t
            self.err_sum = np.clip(self.err_sum, -1, 1)
            err_i = self.err_sum * (1 - self.windup)

        if not self.d_from_sensor:
            err_d = (err - self.prev_err) / (self.delta_t)
            self.prev_err = err

        ctrl = self.gain * np.dot(self.pid_param, np.array([err, err_i, err_d]))
        return ctrl + self.offset

    def reset(self):
        self.err_sum, self.prev_err = 0, 0
        self.windup = 0.0

class TurtleEnv(gym.Env):
    """Custom Environment that follows gym interface"""
    metadata = {'render.modes': ['human']}

    def __init__(self, total_timesteps=int(5e7), x_max=11.1, y_max=11.1,
                 test_mode=False, index=1, seed=0, rate=1000, teacher=False, teacher_weight=None,
                 weight_interval=None, noise_lim=None, pid=False):
        self.index = index
        if not test_mode:
            rospy.init_node('VectorEnv' + str(self.index))
        self.seed(seed)
        # Initialize the turtlesim to the start configuration and set the background color to the value of the background
        reset_client = rospy.ServiceProxy('/turtlesim' + str(self.index) + '/reset', Empty)
        reset_object = EmptyRequest()
        reset_client(reset_object)
        # Test mode
        self.tm = test_mode
        # Observation bound
        self.x_max = x_max
        self.y_max = y_max
        # Range of reaching the destination
        if not self.tm:
            self.r = 0.1
        else:
            # to compare with the hinf controller
            self.r = 0.2
        # step count in one episode
        self.step_count = 0
        self.gl_step = 0
        self.total_timesteps = total_timesteps
        # episode count
        self.episode_count = 0
        self.done_count = 0
        self.action = None
        # s = (x, y, delta_x1, delta_y1, delta_theta1, distance1, delta_x2, delta_y2, delta_theta2, distance2)
        self.observation_space = spaces.Box(low=np.array([-1, -1]), high=np.array([1, 1]),
                                             dtype=np.float32)
        # a = (linear.x, angular.z, variance)
        # Low bound scenario
        # self.action_space = spaces.Box(low=np.array([0, -1.5]), high=np.array([2, 1.5]), dtype=np.float32)
        # High bound scenario
        self.action_space = spaces.Box(low=np.array([0, -20]), high=np.array([10, 20]), dtype=np.float32)
        self.pub = rospy.Publisher('/turtlesim' + str(self.index) + '/turtle1/cmd_vel', Twist, queue_size=1)
        self.pos = Pose()
        rospy.Subscriber('/turtlesim' + str(self.index) + '/turtle1/pose', Pose, self.callback)
        if not test_mode:
            self.now = rospy.Time.now()
        # Random goal
        self.goal_x1 = random.uniform(0, self.x_max)
        self.goal_x2 = random.uniform(0, self.x_max)
        self.goal_y1 = random.uniform(0, self.y_max)
        self.goal_y2 = random.uniform(0, self.y_max)
        # Mark target
        self.get_target()
        # Noise
        self.noise = np.zeros((2,))
        # Teacher
        self.teacher = teacher
        self.pid = pid
        self.help = False
        self.rate = rate
        # Noise limit: [v_noise, w_noise]
        if noise_lim is None:
            self.noise_lim = [1, 1]
        else:
            self.noise_lim = noise_lim
        if self.teacher:
            dt = 1 / rate
            os.chdir(os.path.expanduser("~/catkin_ws/src/turtlebot/scripts"))
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
            # MIMO-hinf
            K3_A = content['K3_A']
            K3_B = content['K3_B']
            K3_C = content['K3_C']
            K3_D = content['K3_D']
            K3 = ss(K3_A, K3_B, K3_C, K3_D)
            Kd3 = c2d(sysc=K3, Ts=dt)
            # Define controller object
            self.v_cs = ControllerSimulation(a=Kd1.A, b=Kd1.B, c=Kd1.C, d=Kd1.D)
            self.omega_cs = ControllerSimulation(a=Kd2.A, b=Kd2.B, c=Kd2.C, d=Kd2.D)
            self.command = ControllerSimulation(a=Kd3.A, b=Kd3.B, c=Kd3.C, d=Kd3.D)
            self.get_x()
            self.teacher_action = torch.zeros((2, 1))
            self.p = teacher_weight
            self.p_decay = 2 * teacher_weight / self.total_timesteps
            # self.p_decay = 0
            self.weight_interval = weight_interval

            self.v_pid = PIDController(
                delta_t=dt,
                pid_param=np.array([8, 0, 0]))
            self.omega_pid = PIDController(
                delta_t=dt,
                pid_param=np.array([10, 0.1, 0.5]))

    def callback(self, current_pos):
        self.pos.x = current_pos.x
        self.pos.y = current_pos.y
        self.pos.theta = current_pos.theta

    def distance1(self):
        # Euclidean distance between current position and the goal.
        return sqrt(pow((self.goal_x1 - self.pos.x), 2) + pow((self.goal_y1 - self.pos.y), 2))

    def distance2(self):
        # Euclidean distance between current position and the goal.
        return sqrt(pow((self.goal_x2 - self.pos.x), 2) + pow((self.goal_y2 - self.pos.y), 2))

    def _get_obs(self):
        delta_x1 = self.goal_x1 - self.pos.x
        delta_y1 = self.goal_y1 - self.pos.y
        delta_x2 = self.goal_x2 - self.pos.x
        delta_y2 = self.goal_y2 - self.pos.y
        # range: ]-pi, pi]
        theta_target1 = np.arctan2(delta_y1, delta_x1)
        theta_target2 = np.arctan2(delta_y2, delta_x2)
        # range: ]-2pi, 2pi[
        delta_theta1 = theta_target1 - self.pos.theta
        delta_theta2 = theta_target2 - self.pos.theta
        # range ]-pi, pi]
        if delta_theta1 <= -np.pi:
            delta_theta1 = delta_theta1 + 2*np.pi
        elif delta_theta1 > np.pi:
            delta_theta1 = delta_theta1 - 2*np.pi
        if delta_theta2 <= -np.pi:
            delta_theta2 = delta_theta2 + 2*np.pi
        elif delta_theta2 > np.pi:
            delta_theta2 = delta_theta2 - 2*np.pi
        distance1 = self.distance1()
        distance2 = self.distance2()
        # Normalization -> [-1,1]
        theta = self.pos.theta / np.pi
        x = (2 * self.pos.x - self.x_max) / self.x_max
        y = (2 * self.pos.y - self.y_max) / self.y_max
        goal_x1 = (2 * self.goal_x1 - self.x_max) / self.x_max
        goal_y1 = (2 * self.goal_x2 - self.x_max) / self.x_max
        delta_x1 = delta_x1 / self.x_max
        delta_y1 = delta_y1 / self.y_max
        delta_x2 = delta_x2 / self.x_max
        delta_y2 = delta_y2 / self.y_max
        sin_delta_theta1 = np.sin(delta_theta1)
        cos_delta_theta2 = np.cos(delta_theta1)
        theta_target1 = theta_target1 / np.pi
        theta_target2 = theta_target2 / np.pi
        delta_theta1 = delta_theta1 / np.pi
        delta_theta2 = delta_theta2 / np.pi
        distance1 = (2 * distance1 - np.sqrt(2) * self.x_max) / (np.sqrt(2) * self.x_max)
        distance2 = (2 * distance2 - np.sqrt(2) * self.x_max) / (np.sqrt(2) * self.x_max)
        if self.teacher:
            if not self.pid:
                v_cs = self.get_y1(y_max=self.action_space.high[0], y_min=1)
                omega_cs = self.get_y2(y_max=self.action_space.high[1], y_min=self.action_space.low[1])
            else:
                v_cs = self.v_pid.action(err=-self.get_e()[0])
                omega_cs = self.v_pid.action(err=self.get_e()[1])
                v_cs = np.clip(v_cs, 1, self.action_space.high[0])
                omega_cs = np.clip(omega_cs, self.action_space.low[1], self.action_space.high[1])
            # command = self.get_y(y_max=self.action_space.high, y_min=[1, self.action_space.low[1]])
            self.teacher_action = torch.as_tensor([v_cs, omega_cs], dtype=torch.float32)
            # self.teacher_action = torch.as_tensor(command, dtype=torch.float32)
            scaled_v = scale_a(v_cs, low=torch.as_tensor(self.action_space.low[0]),
                               high=torch.as_tensor(self.action_space.high[0]))
            scaled_omega = scale_a(omega_cs, low=torch.as_tensor(self.action_space.low[1]),
                                   high=torch.as_tensor(self.action_space.high[1]))
            return np.array(np.array([delta_theta1, distance1, scaled_v, scaled_omega, self.p]), dtype=np.float32)
        else:
            return np.array(np.array([delta_theta1, distance1]), dtype=np.float32)

    def _update_teacher_weight(self):
        if self.weight_interval is not None:
            assert self.weight_interval[0] >= -1
            assert self.weight_interval[1] <= 1
            self.p = random.uniform(self.weight_interval[0], self.weight_interval[1])
        elif self.weight_interval is None:
            if self.p > 0:
                self.p -= self.p_decay
                if self.p < 0:
                    self.p = 0

    def _get_teacher_action(self):
        return self.teacher_action

    def _get_reward(self, case=1, k1=0.1, b=0.5):
        # r = k1*(-d) + b
        if case == 1:
            d = self.distance1()
        elif case == 2:
            # calculate the min. reward
            d = np.sqrt(2)*self.x_max
        elif case == 3:
            # calculate the max. reward
            d = 0
        elif case == 4:
            # calculate the mid. reward
            d = np.sqrt(2)*self.x_max/2
        return k1 * (-d) + b

    def _get_info(self):
        return {"action": self.action, "distance": self.distance1()}

    def duration(self):
        d = rospy.Time.now() - self.now
        return d.to_sec()

    def move_one_step(self, action):
        action = action.squeeze()
        cmd = Twist()
        if action[0] >= self.action_space.low[0] and action[0] <= self.action_space.high[0]:
            cmd.linear.x = action[0]
        elif action[0] < self.action_space.low[0]:
            cmd.linear.x = self.action_space.low[0]
        elif action[0] > self.action_space.high[0]:
            cmd.linear.x = self.action_space.high[0]
        if action[1] >= self.action_space.low[1] and action[1] <= self.action_space.high[1]:
            cmd.angular.z = action[1]
        elif action[1] < self.action_space.low[1]:
            cmd.angular.z = self.action_space.low[1]
        elif action[1] > self.action_space.high[1]:
            cmd.angular.z = self.action_space.high[1]
        self.pub.publish(cmd)

    def sigma(self):
        delta_x = self.goal_x1 - self.pos.x
        delta_y = self.goal_y1 - self.pos.y
        # range: ]-pi, pi]
        return np.float32(np.arctan2(delta_y, delta_x))

    def get_target(self):
        theta = self.sigma() - np.pi
        service_client = rospy.ServiceProxy('/turtlesim' + str(self.index) + '/spawn', Spawn)
        request = SpawnRequest()
        request.x = self.goal_x1
        request.y = self.goal_y1
        request.theta = theta
        name = 'target1'
        request.name = name
        service_client(request)

    def step(self, action):
        if self.teacher:
            self._update_teacher_weight()
            teacher_action = self._get_teacher_action()
            action = self.p * teacher_action + (1 - self.p) * action
        # Noise
        self.get_noise(v_max=self.noise_lim[0], w_max=self.noise_lim[1])
        self.action = action
        action = action + self.noise
        self.move_one_step(action)
        self.step_count = self.step_count + 1
        self.gl_step = self.gl_step + 1
        range_criterion = self.distance1() <= self.r
        no_rotation = range_criterion and action[1] == 0
        done = range_criterion
        reward = self._get_reward(b=0)
        # Extra bonus for reaching the goal
        if no_rotation:
            reward = reward + 100
        elif range_criterion:
            reward = reward + 50
        observation = self._get_obs()
        info = self._get_info()
        if done:
            self.done_count += 1
        # if (self.gl_step % self.rate == 0) and not self.tm:
        #     print(f"Environment: {self.index}\n"
        #           f"Progress: {self.gl_step} / {self.total_timesteps} ( {100*self.gl_step/self.total_timesteps} % )\n"
        #           f"Episode: {self.episode_count}\n"
        #           f"Done: {self.done_count}\n"
        #           f"Observation: {observation}\n"
        #           f"Goal position: {[self.goal_x1, self.goal_y1]}\n"
        #           f"Teacher stand-by: {self.teacher}\n"
        #           f"Teacher intervention: {self.help}\n"
        #           f"Reward: {reward}\n"
        #           f"Distance: {info['distance']}")
        #     # print("Theta_target", self.theta_target)
        #     # print("Theta: ", self.pos.theta)
        #     # print("Delta theta", self.delta_theta)
        #     print("\n")
        return observation, reward, done, info

    def clear_target(self):
        # Clear old target
        kill_client = rospy.ServiceProxy('/turtlesim' + str(self.index) + '/kill', Kill)
        kill_obj = KillRequest()
        name = 'target1'
        kill_obj.name = name
        kill_client(kill_obj)

    def clear_background(self):
        # Reset background
        reset_client = rospy.ServiceProxy('/turtlesim' + str(self.index) + '/clear', Empty)
        reset_object = EmptyRequest()
        reset_client(reset_object)

    def reset_turtle(self):
        # Reset turtle
        teleport_client = rospy.ServiceProxy('/turtlesim' + str(self.index) + '/turtle1/teleport_absolute',
                                             TeleportAbsolute)
        teleport_obj = TeleportAbsoluteRequest()
        teleport_obj.x = 5.5
        teleport_obj.y = 5.5
        teleport_obj.theta = 0
        teleport_client(teleport_obj)

    def random_goal(self):
        # Random goal
        self.goal_x1 = self.goal_x2
        self.goal_y1 = self.goal_y2
        self.goal_x2 = random.uniform(0, self.x_max)
        self.goal_y2 = random.uniform(0, self.y_max)
        self.get_target()

    def reset(self):
        self.episode_count = self.episode_count + 1
        self.step_count = 0
        self.clear_target()
        self.clear_background()
        self.random_goal()
        observation = self._get_obs()
        return observation

    def deep_reset(self):
        self.episode_count = self.episode_count + 1
        self.step_count = 0
        self.clear_target()
        self.reset_turtle()
        self.clear_background()
        self.random_goal()
        observation = self._get_obs()
        return observation

    def teacher_intervene(self, intervene=False):
        self.help = intervene

    # Extension for teacher-student architechture
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
        return np.asarray([delta_R, delta_theta])

    def get_y1(self, y_max=np.Inf, y_min=-np.Inf):
        u = self.get_e()[0]
        return self.v_cs.sim(u, y_max, y_min)

    def get_y2(self, y_max=np.Inf, y_min=-np.Inf):
        u = self.get_e()[1]
        return self.omega_cs.sim(u, y_max, y_min)

    def get_y(self, y_max=np.Inf, y_min=-np.Inf):
        u = self.get_e()
        return self.command.sim(u, y_max, y_min)

    def reset_v_controller(self):
        self.v_cs.controller_reset()
        self.v_pid.reset()

    def reset_omega_controller(self):
        self.omega_cs.controller_reset()
        self.omega_pid.reset()

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

    def seed(self, seed=0):
        # Seed python RNG
        random.seed(seed)
        # Seed numpy RNG
        np.random.seed(seed)
        # seed the RNG for all devices (both CPU and CUDA)
        torch.manual_seed(seed)

