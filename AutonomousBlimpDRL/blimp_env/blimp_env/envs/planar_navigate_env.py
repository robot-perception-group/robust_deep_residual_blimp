""" navigate env with position target """
# !/usr/bin/env python

from __future__ import absolute_import, division, print_function

from typing import Any, Callable, Dict, List, Optional, Tuple, Union

import numpy as np
import rospy
import random
import os
import scipy.io
from blimp_env.envs.common.abstract import ROSAbstractEnv
from blimp_env.envs.common.action import Action
from geometry_msgs.msg import Point, Quaternion
from std_msgs.msg import Float32MultiArray
from rotors_comm.msg import WindSpeed
from blimp_env.envs.script import close_simulation, change_buoynacy
from controller_simulation import ControllerSimulation
from control import *
import line_profiler
import copy

profile = line_profiler.LineProfiler()

Observation = Union[np.ndarray, float]


class PlanarNavigateEnv(ROSAbstractEnv):
    """Navigate blimp by path following decomposed to altitude and planar control"""

    @classmethod
    def default_config(cls) -> dict:
        config = super().default_config()
        config["simulation"].update(
            {
                "enable_wind": False,
                "enable_wind_sampling": False,
                "wind_speed": 2.0,
                "enable_buoyancy_sampling": False,
                "buoyancy_range": [0.9, 1.1],
            }
        )
        config["observation"].update(
            {
                "type": "PlanarKinematics",
                "noise_stdv": 0.02,
                "scale_obs": True,
                "enable_airspeed_sensor": True,
            }
        )
        config["action"].update(
            {
                "type": "SimpleContinuousDifferentialAction",
                "act_noise_stdv": 0.05,
                "disable_servo": True,
                "max_servo": -0.5,
                "max_thrust": 0.5,
            }
        )
        config["target"].update(
            {
                "type": "RandomGoal",
                "target_name_space": "goal_",
                "new_target_every_ts": 1200,
            }
        )
        config.update(
            {
                "duration": 1200,
                "simulation_frequency": 30,  # [hz]
                "policy_frequency": 10,  # [hz] has to be greater than 5 to overwrite backup controller
                "reward_weights": np.array(
                    [100, 0.8, 0.2]
                ),  # success, tracking, action
                "tracking_reward_weights": np.array(
                    [0.25, 0.25, 0.25, 0.25]
                ),  # z_diff, planar_dist, yaw_diff, vel_diff
                "success_threshhold": 5,  # [meters]
            }
        )
        return config

    def _create_pub_and_sub(self):
        super()._create_pub_and_sub()
        self.rew_rviz_pub = rospy.Publisher(
            self.config["name_space"] + "/rviz_reward", Float32MultiArray, queue_size=1
        )
        self.state_rviz_pub = rospy.Publisher(
            self.config["name_space"] + "/rviz_state", Quaternion, queue_size=1
        )
        self.vel_rviz_pub = rospy.Publisher(
            self.config["name_space"] + "/rviz_vel", Point, queue_size=1
        )
        self.vel_diff_rviz_pub = rospy.Publisher(
            self.config["name_space"] + "/rviz_vel_diff", Point, queue_size=1
        )
        self.ang_rviz_pub = rospy.Publisher(
            self.config["name_space"] + "/rviz_ang", Point, queue_size=1
        )
        self.ang_diff_rviz_pub = rospy.Publisher(
            self.config["name_space"] + "/rviz_ang_diff", Point, queue_size=1
        )
        self.act_rviz_pub = rospy.Publisher(
            self.config["name_space"] + "/rviz_act", Quaternion, queue_size=1
        )
        self.pos_cmd_pub = rospy.Publisher(
            self.config["name_space"] + "/rviz_pos_cmd",
            Point,
            queue_size=1,
        )
        if self.config["simulation"]["enable_wind"]:
            self.wind_state_pub = rospy.Publisher(
                self.config["name_space"] + "/wind_state", WindSpeed, queue_size=1
            )
        if self.config["observation"]["enable_airspeed_sensor"]:
            self.airspeed_rviz_pub = rospy.Publisher(
                self.config["name_space"] + "/rviz_airspeed", Point, queue_size=1
            )

    @profile
    def one_step(self, action: Action) -> Tuple[Observation, float, bool, dict]:
        """[perform a step action and observe result]

        Args:
            action (Action): action from the agent [-1,1] with size (4,)

        Returns:
            Tuple[Observation, float, bool, dict]:
                obs: np.array [-1,1] with size (9,),
                reward: scalar,
                terminal: bool,
                info: dictionary of all the step info,
        """
        self._simulate(action)
        obs, obs_info = self.observation_type.observe()
        reward, reward_info = self._reward(obs.copy(), action, copy.deepcopy(obs_info))
        terminal = self._is_terminal(copy.deepcopy(obs_info))
        info = {
            "step": self.steps,
            "obs": obs,
            "obs_info": obs_info,
            "act": action,
            "reward": reward,
            "reward_info": reward_info,
            "terminal": terminal,
        }

        self._update_goal_and_env()
        self._step_info(info)

        return obs, reward, terminal, info

    def _step_info(self, info: dict):
        """publish all the step information to rviz

        Args:
            info ([dict]): [dict contain all step information]
        """
        obs_info = info["obs_info"]
        proc_info = obs_info["proc_dict"]

        self.rew_rviz_pub.publish(
            Float32MultiArray(data=np.array(info["reward_info"]["rew_info"]))
        )
        self.state_rviz_pub.publish(
            Quaternion(
                proc_info["planar_dist"],
                proc_info["yaw_diff"],
                proc_info["z_diff"],
                proc_info["vel_diff"],
            )
        )
        self.vel_rviz_pub.publish(Point(*obs_info["velocity"]))
        self.vel_diff_rviz_pub.publish(
            Point(
                obs_info["velocity_norm"],
                self.goal["velocity"],
                proc_info["vel_diff"],
            )
        )
        if self.config["observation"]["enable_airspeed_sensor"]:
            self.airspeed_rviz_pub.publish(Point(obs_info["airspeed"], 0, 0))
        self.ang_rviz_pub.publish(Point(*obs_info["angle"]))
        self.ang_diff_rviz_pub.publish(Point(0, 0, proc_info["yaw_diff"]))
        self.act_rviz_pub.publish(Quaternion(*info["act"]))

        self.pos_cmd_pub.publish(Point(*self.goal["position"]))

        if self.dbg:
            print(
                f"================= [ PlanarNavigateEnv ] step {self.steps} ================="
            )
            print("STEP INFO:", info)
            print("\r")

    def reset(self) -> Observation:
        self.steps = 0
        self.done = False
        self._reset()

        if self.config["target"]["type"] == "MultiGoal" and self.config["target"].get(
                "enable_random_goal", True
        ):
            n_waypoints = np.random.randint(4, 8)
            self.target_type.sample_new_wplist(n_waypoints=n_waypoints)
        if self.config["simulation"]["enable_wind_sampling"]:
            self._sample_wind_state()
        if self.config["simulation"]["enable_buoyancy_sampling"]:
            self._sample_buoyancy(
                buoyancy_range=self.config["simulation"]["buoyancy_range"]
            )
        obs, _ = self.observation_type.observe()
        return obs

    def _update_goal_and_env(self):
        """update goal and env state"""
        self.goal = self.target_type.sample()

        if (
                self.config["simulation"]["enable_wind"]
                and self.config["simulation"]["enable_wind_sampling"]
        ):
            self.wind_state_pub.publish(self.wind_state)

    def _sample_wind_state(self):
        self.wind_state = WindSpeed()
        wind_speed = self.config["simulation"]["wind_speed"]
        self.wind_state.velocity.x = np.random.uniform(-wind_speed, wind_speed)
        self.wind_state.velocity.y = np.random.uniform(-wind_speed, wind_speed)
        self.wind_state.velocity.z = np.random.uniform(
            -wind_speed / 10, wind_speed / 10
        )

    def _sample_buoyancy(
            self,
            deflation_range=[0.0, 1.5],
            freeflop_angle_range=[0.0, 1.5],
            collapse_range=[0.0, 0.02],
            buoyancy_range=[0.9, 1.1],
    ):
        change_buoynacy(
            robot_id=self.config["robot_id"],
            ros_port=self.config["ros_port"],
            gaz_port=self.config["gaz_port"],
            deflation=np.random.uniform(*deflation_range),
            freeflop_angle=np.random.uniform(*freeflop_angle_range),
            collapse=np.random.uniform(*collapse_range),
            buoyancy=np.random.uniform(*buoyancy_range),
        )

    def _reward(
            self, obs: np.array, act: np.array, obs_info: dict
    ) -> Tuple[float, dict]:
        """calculate reward
        total_reward = success_reward + tracking_reward + action_reward
        success_reward: +1 if agent stay in the vicinity of goal
        tracking_reward: - L2 distance to goal - yaw angle difference - z diff - vel diff
        action_reward: penalty for motor use

        Args:
            obs (np.array): ("z_diff", "planar_dist", "yaw_diff", "vel_diff", "vel", "yaw_vel", "action")
            act (np.array): agent action [-1,1] with size (4,)
            obs_info (dict): contain all information of a step

        Returns:
            Tuple[float, dict]: [reward scalar and a detailed reward info]
        """
        track_weights = self.config["tracking_reward_weights"].copy()
        reward_weights = self.config["reward_weights"].copy()

        success_reward = self.compute_success_rew(
            obs_info["position"], obs_info["goal_dict"]["position"]
        )
        obs[1] = (obs[1] + 1) / 2  # dist -1 should have max reward
        tracking_reward = np.dot(track_weights, -np.abs(obs[0:4]))
        action_reward = self.action_type.action_rew()

        reward = np.dot(
            reward_weights,
            (
                success_reward,
                tracking_reward,
                action_reward,
            ),
        )
        reward = np.clip(reward, -1, 1)
        rew_info = (reward, success_reward, tracking_reward, action_reward)
        reward_info = {"rew_info": rew_info}

        return float(reward), reward_info

    def compute_success_rew(self, pos: np.array, goal_pos: np.array) -> float:
        """task success if distance to goal is less than sucess_threshhold

        Args:
            pos ([np.array]): [position of machine]
            goal_pos ([np.array]): [position of planar goal]
            k (float): scaler for success

        Returns:
            [float]: [1 if success, otherwise 0]
        """
        return (
            1.0
            if np.linalg.norm(pos[0:3] - goal_pos[0:3])
               <= self.target_type.trigger_dist
            else 0.0
        )

    def _is_terminal(self, obs_info: dict) -> bool:
        """if episode terminate
        - time: episode duration finished

        Returns:
            bool: [episode terminal or not]
        """
        time = False
        if self.config["duration"] is not None:
            time = self.steps >= int(self.config["duration"]) - 1

        success = False
        if self.config["target"]["type"] == "MultiGoal":
            success = self.target_type.wp_index == self.target_type.wp_max_index
        else:
            success_reward = self.compute_success_rew(
                obs_info["position"], obs_info["goal_dict"]["position"]
            )
            success = success_reward >= 0.9

        return time or success

    def close(self) -> None:
        return super().close()

    def stop(self) -> None:
        self.gaz.pause_sim()

    def go(self) -> None:
        self.gaz.unpause_sim()


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


class HinfController_MIMO:
    def __init__(
            self,
            dt=0.01,
            max_servo: float = -0.5,
            min_servo: float = -1,
            max_thrust: float = 0.7,
            min_thrust: float = 0.3,
            enable_servo_remapping: bool = False,
    ):
        self.max_servo = max_servo
        self.min_servo = min_servo
        self.max_thrust = max_thrust
        self.min_thrust = min_thrust
        self.enable_servo_remapping = enable_servo_remapping
        self.dt = dt
        os.chdir("/home/yang/catkin_ws/src/AutonomousBlimpDRL/RL/rl/rlyang_script")
        content = scipy.io.loadmat("matlab2.mat")
        # SISO-hinf for v
        K_A = content['A']
        K_B = content['B']
        K_C = content['C']
        K_D = content['D']
        K = ss(K_A, K_B, K_C, K_D)
        Kd = c2d(sysc=K, Ts=self.dt)
        # Define controller object
        self.mimo = ControllerSimulation(a=Kd.A, b=Kd.B, c=Kd.C, d=Kd.D)

    def action(self, u, y_max=1, y_min=-1):
        trim = np.array([0, 0, -1, 0.5])
        output = self.mimo.sim(u, y_max, y_min).squeeze()
        output = output + trim
        if self.enable_servo_remapping:
            # Servo remapping trick
            output[2] = np.clip(output[2], -2, 1)
            if output[2] > 0:
                output[2] = 0
            elif output[2] < -1:
                output[2] = output[2] + 2
            # End remapping
        output = np.clip(output, -1, 1)
        output[2] = np.clip(output[2], self.min_servo, self.max_servo)
        output[3] = np.clip(output[3], self.min_thrust, self.max_thrust)
        return output

    def reset(self):
        self.mimo.controller_reset()


class HinfController:
    def __init__(
            self,
            dt=0.01,
            as_ds_threshold=10,
            max_servo: float = -0.5,
            min_servo: float = -1,
            max_thrust: float = 0.6,
            min_thrust: float = 0.4,
            enable_servo_remapping: bool = False,
    ):
        self.max_servo = max_servo
        self.min_servo = min_servo
        self.max_thrust = max_thrust
        self.min_thrust = min_thrust
        self.enable_servo_remapping = enable_servo_remapping
        self.as_ds_threshold = as_ds_threshold
        self.dt = dt
        os.chdir("/home/yang/catkin_ws/src/AutonomousBlimpDRL/RL/rl/rlyang_script")
        content = scipy.io.loadmat("matlab.mat")
        # MIMO-hinf for ascend motion
        Kas_A = content['Kas_A']
        Kas_B = content['Kas_B']
        Kas_C = content['Kas_C']
        Kas_D = content['Kas_D']
        Kas = ss(Kas_A, Kas_B, Kas_C, Kas_D)
        Kdas = c2d(sysc=Kas, Ts=self.dt)
        # Define controller object
        self.mimo_as = ControllerSimulation(a=Kdas.A, b=Kdas.B, c=Kdas.C, d=Kdas.D)
        # MIMO-hinf for descend motion
        Kds_A = content['Kds_A']
        Kds_B = content['Kds_B']
        Kds_C = content['Kds_C']
        Kds_D = content['Kds_D']
        Kds = ss(Kds_A, Kds_B, Kds_C, Kds_D)
        Kdds = c2d(sysc=Kds, Ts=self.dt)
        # Define controller object
        self.mimo_ds = ControllerSimulation(a=Kdds.A, b=Kdds.B, c=Kdds.C, d=Kdds.D)
        # SISO-hinf for yaw motion (yaw)
        Kya_A = content['Kya_A']
        Kya_B = content['Kya_B']
        Kya_C = content['Kya_C']
        Kya_D = content['Kya_D']
        Kya = ss(Kya_A, Kya_B, Kya_C, Kya_D)
        Kdya = c2d(sysc=Kya, Ts=self.dt)
        # Define controller object
        self.mimo_ya = ControllerSimulation(a=Kdya.A, b=Kdya.B, c=Kdya.C, d=Kdya.D)

    def action(self, u, y_max=1, y_min=-1):
        if self.as_ds_threshold is not None:
            # Ascent controller takes over
            if u[1] <= self.as_ds_threshold:
                trim = np.array([0, 0, -1, 0.5])
                output = np.insert(self.mimo_as.sim(u[:-1], y_max, y_min), 0, self.mimo_ya.sim(u[-1], y_max, y_min))
                # print("1 ", output)
                # output = np.insert(output, -1, 0)
                # print("2 ", output)
                output = output + trim
                if self.enable_servo_remapping:
                    # Servo remapping trick
                    output[2] = np.clip(output[2], -2, 1)
                    if output[2] > 0:
                        output[2] = 0
                    elif output[2] < -1:
                        output[2] = output[2] + 2
                    # End remapping
                output = np.clip(output, -1, 1)
                output[2] = np.clip(output[2], self.min_servo, self.max_servo)
                output[3] = np.clip(output[3], self.min_thrust, self.max_thrust)
            # Descend controller takes over
            else:
                trim = np.array([0, 0, 1, -0.5])
                output = np.insert(self.mimo_ds.sim(u[:-1], y_max, y_min), 0, self.mimo_ya.sim(u[-1], y_max, y_min))
                # print("1 ", output)
                # output = np.insert(output, -1, 0)
                # print("2 ", output)
                output = output + trim
                if self.enable_servo_remapping:
                    # Servo remapping trick
                    output[2] = np.clip(output[2], -2, 1)
                    if output[2] > 0:
                        output[2] = 0
                    elif output[2] < -1:
                        output[2] = output[2] + 2
                    # End remapping
                output = np.clip(output, -1, 1)
                output[2] = np.clip(output[2], -self.max_servo, -self.min_servo)
                output[3] = np.clip(output[3], -self.max_thrust, -self.min_thrust)
        else:
            # Default: ascend controller
            trim = np.array([0, 0, -1, 0.5])
            output = np.insert(self.mimo_as.sim(u[:-1], y_max, y_min), 0, self.mimo_ya.sim(u[-1], y_max, y_min))
            # print("1 ", output)
            # output = np.insert(output, -1, 0)
            # print("2 ", output)
            output = output + trim
            if self.enable_servo_remapping:
                # Servo remapping trick
                output[2] = np.clip(output[2], -2, 1)
                if output[2] > 0:
                    output[2] = 0
                elif output[2] < -1:
                    output[2] = output[2] + 2
                # End remapping
            output = np.clip(output, -1, 1)
            output[2] = np.clip(output[2], self.min_servo, self.max_servo)
            output[3] = np.clip(output[3], self.min_thrust, self.max_thrust)
        return output

    def reset(self):
        self.mimo_as.controller_reset()
        self.mimo_ds.controller_reset()
        self.mimo_ya.controller_reset()


class ResidualPlanarNavigateEnv(PlanarNavigateEnv):
    @classmethod
    def default_config(cls) -> dict:
        config = super().default_config()
        config["simulation"].update(
            {
                "enable_wind": True,
                "enable_wind_sampling": True,
                "wind_speed": 1.0,
                "enable_buoyancy_sampling": True,
                "buoyancy_range": [0.9, 1.1],
            }
        )
        config["observation"].update(
            {
                "type": "PlanarKinematics",
                "noise_stdv": 0.02,
                "scale_obs": True,
                "enable_rsdact_feedback": True,
                "enable_airspeed_sensor": True,
                "enable_next_goal": True,  # only support target type: MultiGoal
                "weight_interval": None,  # for sampling method
            }
        )
        config["action"].update(
            {
                "type": "SimpleContinuousDifferentialAction",
                "act_noise_stdv": 0.05,
                "disable_servo": False,
                "max_servo": -0.5,
                "max_thrust": 0.5,
            }
        )
        trigger_dist = 5
        config["target"].update(
            {
                "type": "MultiGoal",
                "target_name_space": "goal_",
                "trigger_dist": trigger_dist,
                "enable_dependent_wp": True,
                "enable_random_goal": True,
                "dist_range": [10, 40],
            }
        )
        config.update(
            {
                "duration": 2400,
                "simulation_frequency": 30,  # [hz]
                "policy_frequency": 10,  # [hz] has to be greater than 5 to overwrite backup controller
                "reward_weights": np.array(
                    [500, 1, 10, 0]
                ),  # success, tracking, action, bonus
                "tracking_reward_weights": np.array(
                    [2, 5, 2, 0]
                ),  # z_diff, planar_dist, yaw_diff, vel_diff
                "reward_scale": 1,
                "clip_reward": False,
                "enable_residual_ctrl": True,
                "mixer_type": "absolute",  # absolute, relative, hybrid
                "mixer_param": (0.5, 0.5),  # alpha, beta
                "weight_interval": None,
                "uniform": True,
                "MIMO_controller": False,
                "pid": False,
                "test_mode": False,
                "similarity": 0.4,    # Expect the PPO agent to be not far away from the controller in yaw and pitch controls
                "as_ds_threshold": 0,
                "base_ctrl_config": {
                    "yaw": {
                        "pid_param": np.array([10, 0.001, 0.01]),
                        "gain": 0.5,
                        "d_from_sensor": True,
                    },
                    "alt": {
                        "pid_param": np.array([6.0, 0.02, 0]),
                        "gain": 2.0,
                        "offset": 0.005,
                    },
                    "serv": {
                        "pid_param": np.array([3.0, 0.01, 0]),
                        "gain": 2.0,
                        "offset": -1,
                    },
                    "vel": {
                        "pid_param": np.array([1.6, 0, 0]),
                        "gain": -1,
                        "offset": 0.2,
                    },
                },
            }
        )
        return config

    def __init__(self, config: Optional[Dict[Any, Any]] = None) -> None:
        super().__init__(config=config)

        self.base_act = np.zeros(self.action_type.act_dim)
        delta_t = 1 / self.config["policy_frequency"]
        self.weight_interval = self.config["weight_interval"]
        self.uniform = self.config["uniform"]
        self.hit = False
        self.test_mode = self.config["test_mode"]
        self.similarity = self.config["similarity"]
        self.as_ds_threshold = self.config["as_ds_threshold"]

        self.pid = self.config["pid"]
        self.max_servo = -0.5
        self.min_servo = -1
        self.max_thrust = 0.6
        self.min_thrust = 0.4

        if self.config["MIMO_controller"]:
            self.base_controller = HinfController_MIMO(dt=delta_t)
        else:
            self.base_controller = HinfController(dt=delta_t, as_ds_threshold=self.as_ds_threshold)

        self.yaw_basectrl = PIDController(
            delta_t=delta_t,
            **self.config["base_ctrl_config"]["yaw"],
        )
        self.alt_basectrl = PIDController(
            delta_t=delta_t,
            **self.config["base_ctrl_config"]["alt"],
        )
        self.serv_basectrl = PIDController(
            delta_t=delta_t,
            **self.config["base_ctrl_config"]["serv"],
        )
        self.vel_basectrl = PIDController(
            delta_t=delta_t,
            **self.config["base_ctrl_config"]["vel"],
        )
        # self.yaw_basectrl = PIDController(
        #     delta_t=delta_t,
        #     **self.config["base_ctrl_config"]["yaw"],
        # )
        # self.alt_basectrl = PIDController(
        #     delta_t=delta_t,
        #     **self.config["base_ctrl_config"]["alt"],
        # )
        # self.vel_basectrl = PIDController(
        #     delta_t=delta_t,
        #     **self.config["base_ctrl_config"]["vel"],
        # )

    def _create_pub_and_sub(self):
        self.ang_vel_rviz_pub = rospy.Publisher(
            self.config["name_space"] + "/rviz_ang_vel", Point, queue_size=1
        )
        self.base_act_rviz_pub = rospy.Publisher(
            self.config["name_space"] + "/rviz_base_act", Quaternion, queue_size=1
        )
        return super()._create_pub_and_sub()

    def _update_teacher_weight(self):
        if self.weight_interval is not None:
            if self.uniform:
                self.p = random.uniform(self.weight_interval[0], self.weight_interval[1])  # uniform distribution
                self.p = np.clip(self.p, 0, 1)
            else:
                self.p = np.random.beta(self.weight_interval[0], self.weight_interval[1])  # beta distribution
                self.p = np.clip(self.p, 0, 1)

    @profile
    def one_step(self, action: Action) -> Tuple[Observation, float, bool, dict]:
        """[perform a step action and observe result]

        Args:
            action (Action): action from the agent [-1,1] with size (4,)

        Returns:
            Tuple[Observation, float, bool, dict]:
                obs: np.array [-1,1] with size (9,),
                reward: scalar,
                terminal: bool,
                info: dictionary of all the step info,
        """
        self.base_act = (
            self.base_ctrl() if self.config["enable_residual_ctrl"] else np.zeros(4)
        )
        # self.base_act = np.insert(self.base_act, 2, -1)
        # print("Base act :", self.base_act)
        joint_act = self.mixer(
            action,
            self.base_act,
            self.config["mixer_param"][0],
            self.config["mixer_param"][1],
        )
        # joint_act = np.zeros_like(joint_act)
        # joint_act[2] = -1
        # # if joint_act[3] < 0.3:
        # #     joint_act[3] = 0.3
        # # if joint_act[3] > 0.7:
        # #     joint_act[3] = 0.7
        # joint_act[3] = 0.5
        # joint_act[1] = 1
        # print("joint_act: ", joint_act)
        self._simulate(joint_act)
        # self.base_act = (
        #     self.base_ctrl() if self.config["enable_residual_ctrl"] else np.zeros(4)
        # )
        if self.weight_interval is not None:
            obs, obs_info = self.observation_type.observe(joint_act=joint_act, rsdact=self.base_act.copy(), p=self.p)
        else:
            obs, obs_info = self.observation_type.observe(joint_act=joint_act, rsdact=self.base_act.copy())
        reward, reward_info = self._reward(
            obs.copy(), action, self.base_act, copy.deepcopy(obs_info)
        )
        terminal = self._is_terminal(copy.deepcopy(obs_info))
        if terminal:
            self.base_controller.reset()
            if self.pid:
                self.yaw_basectrl.reset()
                self.alt_basectrl.reset()
                self.serv_basectrl.reset()
                self.vel_basectrl.reset()
        info = {
            "step": self.steps,
            "obs": obs,
            "obs_info": obs_info,
            "act": action,
            "base_act": self.base_act,
            "joint_act": joint_act,
            "reward": reward,
            "reward_info": reward_info,
            "terminal": terminal,
        }

        self._update_goal_and_env()
        self._step_info(info)

        return obs, reward, terminal, info

    def _step_info(self, info: dict):
        obs_info = info["obs_info"]
        proc_info = obs_info["proc_dict"]

        self.ang_vel_rviz_pub.publish(Point(0, 0, proc_info["yaw_vel"]))
        self.base_act_rviz_pub.publish(Quaternion(*info["base_act"]))
        return super()._step_info(info)

    def mixer(self, action, base_act, alpha=0.5, beta=0.5):
        if self.config["enable_residual_ctrl"] == False:
            return action
        if self.weight_interval is None:
            if self.config["mixer_type"] == "absolute":
                joint_act = beta * action + (1 - beta) * base_act
            elif self.config["mixer_type"] == "relative":
                joint_act = base_act * (1 + beta * action)
            elif self.config["mixer_type"] == "hybrid":
                absolute = beta * action + (1 - beta) * base_act
                relative = base_act * (1 + beta * action)
                joint_act = alpha * absolute + (1 - alpha) * relative
            else:
                raise NotImplementedError
        else:
            self._update_teacher_weight()
            joint_act = (1 - self.p) * action + self.p * base_act
        # joint_act[2] = action[2]
        return np.clip(joint_act, -1, 1)

    # def base_ctrl(self):
    #     """
    #     generate base control signal
    #     """
    #     obs, obs_dict = self.observation_type.observe()
    #
    #     yaw_ctrl = self.yaw_basectrl.action(
    #         err=-obs[2], err_d=obs_dict["angular_velocity"][2]
    #     )
    #     alt_ctrl = self.alt_basectrl.action(obs[0])
    #     vel_ctrl = self.vel_basectrl.action(-obs[3])
    #
    #     return np.clip(np.array([yaw_ctrl, alt_ctrl, 0, vel_ctrl]), -1, 1)

    def base_ctrl(self):
        """
        generate base control signal
        """
        obs, obs_dict = self.observation_type.observe()
        if not self.pid:
            # distance between machine and goal in xy-plane, always positive
            delta_d = np.linalg.norm(self.goal["position"][0:2] -
                                     obs_dict["position"][0:2])
            delta_h = self.goal["position"][2] - obs_dict["position"][2]
            delta_psi = self.observation_type.compute_yaw_diff(self.goal["position"], obs_dict["position"],
                                                               obs_dict["angle"][2])
            e = np.array([-delta_d, delta_h, -delta_psi]).reshape((3, 1))
            # print("e: ", e)
            return self.base_controller.action(e).squeeze()
        else:
            yaw_ctrl = self.yaw_basectrl.action(
                err=-obs[2], err_d=obs_dict["angular_velocity"][2]
            )
            alt_ctrl = self.alt_basectrl.action(obs[0])
            serv_ctrl = self.serv_basectrl.action(obs[0])
            vel_ctrl = self.vel_basectrl.action(-(obs[1]+1))

            if serv_ctrl > self.max_servo:
                serv_ctrl = self.max_servo
            elif serv_ctrl < self.min_servo:
                serv_ctrl = self.min_servo
            if vel_ctrl > self.max_thrust:
                vel_ctrl = self.max_thrust
            elif vel_ctrl < self.min_thrust:
                vel_ctrl = self.min_thrust

            return np.clip(np.array([yaw_ctrl, alt_ctrl, serv_ctrl, vel_ctrl]), -1, 1)

    def clear_basectrl_param(self):
        self.yaw_basectrl.reset()
        self.alt_basectrl.reset()
        self.serv_basectrl.reset()
        self.vel_basectrl.reset()
        self.base_controller.reset()

    def reset(self) -> Observation:
        self.clear_basectrl_param()
        self.base_controller.reset()
        return super().reset()

    def _reward(
            self, obs: np.array, act: np.array, base_act: np.array, obs_info: dict
    ) -> Tuple[float, dict]:
        """calculate reward
        total_reward = success_reward + tracking_reward + action_reward
        success_reward: +1 if agent stay in the vicinity of goal
        tracking_reward: - L2 distance to goal - yaw angle difference - z diff - vel diff
        action_reward: penalty for motor use

        Args:
            obs (np.array): ("z_diff", "planar_dist", "yaw_diff", "vel_diff", "vel", "yaw_vel" "action")
            act (np.array): agent action [-1,1] with size (4,)
            obs_info (dict): contain all information of a step

        Returns:
            Tuple[float, dict]: [reward scalar and a detailed reward info]
        """
        track_weights = self.config["tracking_reward_weights"].copy()
        reward_weights = self.config["reward_weights"].copy()

        success_reward = self.compute_success_rew(
            obs_info["position"], obs_info["goal_dict"]["position"]
        )

        # obs[1] = (obs[1] + 1) / 2  # dist -1 should have max reward
        # tracking_reward = np.dot(track_weights, -np.abs(obs[0:4]))
        # distance = np.linalg.norm(obs_info["position"][0:2] - obs_info["goal_dict"]["position"][0:2])
        h = np.abs(obs[0])  # [0, 1]
        r = obs[1] + 1  # [0, 2]
        be = np.abs(obs[2])  # [0, 1]
        tracking_reward = (track_weights[0] * (-h) + 2) + (track_weights[1] * (-r) + 10) + \
                          (track_weights[2] * (-be) + 2)
        action_reward = 0
        # # Yaw bonus if PPO turns like the controller while sharp turning
        # if abs(base_act[0]) > 0.8 and abs(act[0] > 0.8):
        #     action_reward = action_reward + 0.5
        # Yaw penalty if PPO acts against controller
        if act[0] * base_act[0] < 0 and abs(act[0] - base_act[0]) > self.similarity:
            action_reward = action_reward - 0.5
        # Pitch penalty if PPO acts against controller
        if act[1] * base_act[1] < 0 and abs(act[1] - base_act[1]) > self.similarity:
            action_reward = action_reward - 0.5
        # # Discourage if actor acts slower than controller
        # if abs(act[3]) < (base_act[3]):
        #     action_reward = action_reward - 0.5
        if self.as_ds_threshold is not None:
            # Reverse penalty (Throttle and servo have opposite sign if moving forward)
            if act[2] * act[3] > 0:
                action_reward = action_reward - 0.5

            # Servo action reward if actor does not act against the controller:
            if act[2] * base_act[2] > 0:
                action_reward = action_reward + 1
            if not self.pid:
                # Descend penalty for H-inf servo
                if base_act[1] == -1 and base_act[2] == abs(self.base_controller.max_servo)\
                        and act[2] > abs(self.base_controller.max_servo) + self.similarity/2:
                    action_reward = action_reward - 0.5
            else:
                # Descend penalty PID servo
                if base_act[1] == -1 and base_act[2] == -1 \
                        and act[2] > -1 + self.similarity/2:
                    action_reward = action_reward - 0.5

            # if obs_info["original_dict"]["z_diff"] > 0 and act[2] > 0:
            #     action_reward = action_reward - 0.5
            # # Servo action while steep descending
            # if obs_info["original_dict"]["z_diff"] < - self.as_ds_threshold and act[2] < 0:
            #     action_reward = action_reward - 1
        else:
            if not self.pid:
                # Descend penalty if PPO acts against controller
                if base_act[1] == -1 and base_act[2] == -1:
                    if abs(act[1] - base_act[1]) > 0.1:
                        action_reward = action_reward - 0.5
                    if abs(act[2] - base_act[2]) > 0.1:
                        action_reward = action_reward - 0.5
        bonus_reward = 0
        if self.config["observation"].get("enable_next_goal", False):
            dist = np.linalg.norm(
                obs_info["position"][0:2] - obs_info["goal_dict"]["position"][0:2]
            )
            bonus_reward += -np.abs(obs_info["proc_dict"]["next_yaw_diff"]) / (1 + dist)

        reward = self.config["reward_scale"] * np.dot(
            reward_weights,
            (success_reward, tracking_reward, action_reward, bonus_reward),
        )
        if self.config["clip_reward"]:
            reward = np.clip(reward, -1, 1)

        rew_info = (
            reward,
            success_reward,
            tracking_reward,
            action_reward,
            bonus_reward,
        )

        return float(reward), {"rew_info": rew_info}

    def _is_terminal(self, obs_info: dict) -> bool:
        """if episode terminate
        - time: episode duration finished

        Returns:
            bool: [episode terminal or not]
        """
        time = False
        if self.config["duration"] is not None:
            time = self.steps >= int(self.config["duration"]) - 1

        if self.config["target"]["type"] == "MultiGoal":
            if self.test_mode:
                # success = self.target_type.wp_index == self.target_type.wp_max_index
                time = False
                success = False
            else:
                success = self.target_type.close_enough(self.target_type.wp_list[self.target_type.wp_index].position,
                                                        self.target_type.env.observation_type.pos_data,
                                                        self.target_type.trigger_dist)
        else:
            success_reward = self.compute_success_rew(
                obs_info["position"], obs_info["goal_dict"]["position"]
            )
            success = success_reward >= 0.9

        return time or success


class YawControlEnv(ResidualPlanarNavigateEnv):
    @classmethod
    def default_config(cls) -> dict:
        config = super().default_config()
        config["simulation"].update(
            {
                "enable_wind": False,
                "enable_wind_sampling": False,
                "wind_speed": 0.0,
                "enable_buoyancy_sampling": False,
                "enable_next_goal": False,
            }
        )
        config["observation"].update(
            {
                "type": "DummyYaw",
                "noise_stdv": 0.02,
                "scale_obs": True,
                "enable_rsdact_feedback": True,
            }
        )
        config["action"].update(
            {
                "type": "DummyYawAction",
                "act_noise_stdv": 0.05,
                "disable_servo": True,
            }
        )
        config["target"].update(
            {
                "type": "RandomGoal",
                "target_name_space": "goal_",
                "new_target_every_ts": 1200,
            }
        )
        config.update(
            {
                "duration": 1200,
                "simulation_frequency": 30,  # [hz]
                "policy_frequency": 10,  # [hz] has to be greater than 5 to overwrite backup controller
                "reward_weights": np.array([1.0, 1.0, 0]),  # success, tracking, action
                "tracking_reward_weights": np.array([1.0]),  # yaw_diff
                "success_seconds": 5,  # seconds within threshold as success
                "reward_scale": 0.1,
                "clip_reward": False,
                "enable_residual_ctrl": True,
                "mixer_type": "absolute",  # absolute, relative, hybrid
                "mixer_param": (0.5, 0.5),  # alpha, beta
                "pid_param": np.array([1.0, 0.0, 0.05]),
            }
        )
        return config

    def __init__(self, config: Optional[Dict[Any, Any]] = None) -> None:
        super().__init__(config=config)
        delta_t = 1 / self.config["policy_frequency"]
        self.yaw_basectrl = PIDController(
            pid_param=self.config["pid_param"], delta_t=delta_t, d_from_sensor=True
        )

        self.success_cnt = 0

    @profile
    def one_step(self, action: Action) -> Tuple[Observation, float, bool, dict]:
        """[perform a step action and observe result]

        Args:
            action (Action): action from the agent [-1,1] with size (4,)

        Returns:
            Tuple[Observation, float, bool, dict]:
                obs: np.array [-1,1] with size (9,),
                reward: scalar,
                terminal: bool,
                info: dictionary of all the step info,
        """
        joint_act = self.mixer(
            action,
            self.base_act,
            self.config["mixer_param"][0],
            self.config["mixer_param"][1],
        )
        self._simulate(joint_act)
        self.base_act = (
            self.base_ctrl()
            if self.config["enable_residual_ctrl"]
            else np.zeros(self.action_type.act_dim)
        )
        obs, obs_info = self.observation_type.observe(self.base_act.copy())
        reward, reward_info = self._reward(
            obs.copy(), joint_act, copy.deepcopy(obs_info)
        )
        terminal = self._is_terminal(copy.deepcopy(obs_info))
        info = {
            "step": self.steps,
            "obs": obs,
            "obs_info": obs_info,
            "act": action,
            "base_act": self.base_act,
            "joint_act": joint_act,
            "reward": reward,
            "reward_info": reward_info,
            "terminal": terminal,
        }

        self._update_goal_and_env()
        self._step_info(info)

        return obs, reward, terminal, info

    def _step_info(self, info: dict):
        """publish all the step information to rviz

        Args:
            info ([dict]): [dict contain all step information]
        """
        obs_info = info["obs_info"]
        proc_info = obs_info["proc_dict"]

        self.rew_rviz_pub.publish(
            Float32MultiArray(data=np.array(info["reward_info"]["rew_info"]))
        )
        self.state_rviz_pub.publish(Quaternion(0, proc_info["yaw_diff"], 0, 0))
        self.ang_diff_rviz_pub.publish(Point(0, 0, proc_info["yaw_diff"]))
        self.ang_vel_rviz_pub.publish(Point(0, 0, proc_info["yaw_vel"]))

        self.act_rviz_pub.publish(Quaternion(*info["act"], 0, 0, 0))
        self.base_act_rviz_pub.publish(
            Quaternion(*info["base_act"], *info["joint_act"], 0, 0)
        )

        self.pos_cmd_pub.publish(Point(*self.goal["position"]))

        if self.dbg:
            print(
                f"================= [ PlanarNavigateEnv ] step {self.steps} ================="
            )
            print("STEP INFO:", info)
            print("\r")

    def mixer(self, action, base_act, alpha=0.5, beta=0.5):
        if self.config["enable_residual_ctrl"] == False:
            return action

        if self.config["mixer_type"] == "absolute":
            joint_act = beta * action + (1 - beta) * base_act
        elif self.config["mixer_type"] == "relative":
            joint_act = base_act * (1 + beta * action)
        elif self.config["mixer_type"] == "hybrid":
            absolute = beta * action + (1 - beta) * base_act
            relative = base_act * (1 + beta * action)
            joint_act = alpha * absolute + (1 - alpha) * relative
        else:
            raise NotImplementedError

        return np.clip(joint_act, -1, 1)

    def base_ctrl(self):
        """
        generate base control signal
        """
        obs, obs_dict = self.observation_type.observe()

        yaw_ctrl = self.yaw_basectrl.action(
            err=-obs[0], err_d=obs_dict["angular_velocity"][2]
        )
        return np.clip(np.array([yaw_ctrl]), -1, 1)

    def _reward(
            self, obs: np.array, act: np.array, obs_info: dict
    ) -> Tuple[float, dict]:
        """calculate reward
        total_reward = success_reward + tracking_reward + action_reward
        success_reward: 0
        tracking_reward: - yaw angle difference
        action_reward: penalty for motor use

        Args:
            obs (np.array): ("yaw_diff", "yaw_vel", "action")
            act (np.array): agent action [-1,1] with size (1,)
            obs_info (dict): contain all information of a step

        Returns:
            Tuple[float, dict]: [reward scalar and a detailed reward info]
        """
        track_weights = self.config["tracking_reward_weights"].copy()
        reward_weights = self.config["reward_weights"].copy()

        yaw_diff = obs_info["proc_dict"]["yaw_diff"]
        success_reward = self.compute_success_rew(yaw_diff)

        tracking_reward = np.dot(track_weights, -np.abs(obs[0]))

        action_reward = self.action_type.action_rew()

        reward = self.config["reward_scale"] * np.dot(
            reward_weights,
            (success_reward, tracking_reward, action_reward),
        )

        if self.config["clip_reward"]:
            reward = np.clip(reward, -1, 1)

        rew_info = (reward, success_reward, tracking_reward, action_reward)

        return float(reward), {"rew_info": rew_info}

    def compute_success_rew(
            self,
            yaw_diff: np.array,
            epsilon: float = 0.1,
    ) -> float:
        """yaw_diff less than 0.1 for 5 seconds

        Args:
            yaw_diff (np.array): [scaled yaw diff]
            epsilon (float, optional): [tolerence]. Defaults to 0.1.

        Returns:
            float: [0.0 or 1.0]
        """
        if np.abs(yaw_diff) < epsilon:
            self.success_cnt += 1
        else:
            self.success_cnt = 0

        return float(
            self.success_cnt
            > self.config["success_seconds"] * self.config["simulation_frequency"]
        )

    def _is_terminal(self, obs_info: dict) -> bool:
        """if episode terminate
        - time: episode duration finished

        Returns:
            bool: [episode terminal or not]
        """
        time = False
        if self.config["duration"] is not None:
            time = self.steps >= int(self.config["duration"]) - 1

        return time

    def close(self) -> None:
        close_simulation()


if __name__ == "__main__":
    import copy
    from blimp_env.envs.common.gazebo_connection import GazeboConnection
    from blimp_env.envs.script import close_simulation

    # ============== profile ==============#
    # 1. pip install line-profiler
    # 2. in terminal:
    # kernprof -l -v blimp_env/envs/planar_navigate_env.py

    auto_start_simulation = False
    if auto_start_simulation:
        close_simulation()

    ENV = ResidualPlanarNavigateEnv  # PlanarNavigateEnv, ResidualPlanarNavigateEnv, YawControlEnv
    env_kwargs = {
        "DBG": True,
        "simulation": {
            "gui": True,
            "enable_meshes": True,
            "auto_start_simulation": auto_start_simulation,
            "enable_wind": False,
            "enable_wind_sampling": False,
            "enable_buoyancy_sampling": False,
            "wind_speed": 0,
            "wind_direction": (1, 0),
            "position": (0, 0, 30),  # initial spawned position
        },
        "observation": {
            "DBG_ROS": False,
            "DBG_OBS": False,
            "noise_stdv": 0,
        },
        "action": {
            "DBG_ACT": False,
            "act_noise_stdv": 0,
            "disable_servo": True,
        },
        "target": {
            "DBG_ROS": False,
            "enable_random_goal": False,
            "trigger_dist": 5,
            "wp_list": [
                (20, 20, -100, 3),
                (20, -20, -100, 3),
                (-20, -20, -100, 3),
                (-20, 20, -100, 3),
            ],
        },
        "mixer_type": "absolute",
        "mixer_param": (0.5, 1),
    }


    @profile
    def env_step():
        env = ENV(copy.deepcopy(env_kwargs))
        env.reset()
        for _ in range(100000):
            action = env.action_space.sample()
            action = np.zeros_like(action)  # [yaw, pitch, servo, thrust]
            obs, reward, terminal, info = env.step(action)

        GazeboConnection().unpause_sim()


    env_step()
