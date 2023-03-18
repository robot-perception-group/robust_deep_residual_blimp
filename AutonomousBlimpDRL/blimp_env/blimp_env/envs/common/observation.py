""" observation type """
#!/usr/bin/env python

from math import e
import subprocess
from typing import TYPE_CHECKING, Any, Dict, Tuple
import time
import numpy as np
import pandas as pd
import rospy
from blimp_env.envs.common import utils
from blimp_env.envs.script.blimp_script import respawn_model, resume_simulation
from gym import spaces
from sensor_msgs.msg import Imu
from transforms3d.euler import quat2euler
from uav_msgs.msg import uav_pose

if TYPE_CHECKING:
    from blimp_env.envs.common.abstract import AbstractEnv

GRAVITY = 9.81


class ObservationType:
    """abstract observation type"""

    def __init__(
        self, env: "AbstractEnv", **kwargs  # pylint: disable=unused-argument
    ) -> None:
        self.env = env

    def space(self) -> spaces.Space:
        """Get the observation space."""
        raise NotImplementedError()

    def observe(self):
        """Get an observation of the environment state."""
        raise NotImplementedError()

    def compute_yaw_diff(self):
        """Compute delta_psi."""
        raise NotImplementedError()


class ROSObservation(ObservationType):
    """kinematirc obervation from sensors"""

    def __init__(
        self,
        env: "AbstractEnv",
        name_space="machine_0",
        DBG_ROS=False,
        DBG_OBS=False,
        real_experiment=False,
        **kwargs  # pylint: disable=unused-argument
    ):
        super().__init__(env)
        self.name_space = name_space
        self.dbg_ros = DBG_ROS
        self.dbg_obs = DBG_OBS
        self.real_exp = real_experiment
        self.imu_namespace = (
            self.name_space + "/Imu" if self.real_exp else self.name_space + "/tail/imu"
        )

        self.obs_dim = 15

        self.pos_data = np.array([0, 0, 0])
        self.vel_data = np.array([0, 0, 0])
        self.acc_data = np.array([0, 0, 0])
        self.ori_data = np.array([0, 0, 0, 0])
        self.ang_data = np.array([0, 0, 0])
        self.ang_vel_data = np.array([0, 0, 0])
        self.airspeed_data = np.array([0])

        self.ros_cnt = 0

        self._create_pub_and_sub()

    def space(self) -> spaces.Space:
        return spaces.Box(
            low=np.full((self.obs_dim), -1),
            high=np.full((self.obs_dim), 1),
            dtype=np.float32,
        )

    def _create_pub_and_sub(self):
        rospy.Subscriber(
            self.imu_namespace,
            Imu,
            self._imu_callback,
        )

        rospy.Subscriber(self.name_space + "/pose", uav_pose, self._pose_callback)
        time.sleep(1)

    def _imu_callback(self, msg):
        """imu msg callback

        Args:
            msg ([Imu]): imu sensor raw data
        """
        acc = utils.obj2array(msg.linear_acceleration)
        if self.real_exp:
            acc[2] += GRAVITY
        else:
            acc[2] -= GRAVITY

        self.acc_data = acc

        if self.dbg_ros:
            self.ros_cnt += 1
            if self.ros_cnt % 100 == 0:
                print(
                    "[ KinematicObservation ] imu_callback: linear_acceleration",
                    self.acc_data,
                )

    def _pose_callback(self, msg):
        """pose msg callback

        Args:
            msg ([uav_pose]): gcs processed sensor data
        """
        self.pos_data = utils.obj2array(msg.position)
        self.vel_data = utils.obj2array(msg.velocity)
        self.ori_data = utils.obj2array(msg.orientation)
        self.ang_data = quat2euler(self.ori_data)
        self.ang_vel_data = utils.obj2array(msg.angVelocity)
        self.airspeed_data = np.array(msg.POI.x)
        if self.airspeed_data < 0.25:
            self.airspeed_data = np.zeros(1)

        if self.dbg_ros:
            print(
                "[ KinematicObservation ] pose_callback: position",
                self.pos_data,
            )
            print(
                "[ KinematicObservation ] pose_callback: velocity",
                self.vel_data,
            )
            print(
                "[ KinematicObservation ] pose_callback: orientation",
                self.ori_data,
            )
            print(
                "[ KinematicObservation ] pose_callback: angle",
                self.ang_data,
            )
            print(
                "[ KinematicObservation ] pose_callback: ang_vel",
                self.ang_vel_data,
            )
            print(
                "[ KinematicObservation ] pose_callback: airspeed",
                self.airspeed_data,
            )

    def check_connection(self):
        """check ros connection"""
        while (self.pos_data == np.zeros(3)).all():
            rospy.loginfo("[ observation ] waiting for pose subscriber...")
            try:
                pose_data = rospy.wait_for_message(
                    self.name_space + "/pose",
                    uav_pose,
                    timeout=50,
                )
                imu_data = rospy.wait_for_message(
                    self.imu_namespace,
                    Imu,
                    timeout=50,
                )
            except:
                rospy.loginfo("[ observation ] cannot find pose subscriber")
                self.obs_err_handle()
                pose_data = rospy.wait_for_message(
                    self.name_space + "/pose",
                    uav_pose,
                    timeout=50,
                )

            self.pos_data = utils.obj2array(pose_data.position)

        rospy.loginfo("[ observation ] pose ready")

    def observe(self) -> np.ndarray:
        raise NotImplementedError

    def obs_err_handle(self):
        try:
            rospy.loginfo("[ observation ] respawn model...")
            reply = respawn_model(**self.env.config["simulation"])
            rospy.loginfo("[ observation ] respawn model status ", reply)
        except:
            rospy.loginfo("[ observation ] resume simulation...")
            reply = resume_simulation(**self.env.config["simulation"])
            rospy.loginfo("[ observation ] resume simulation status ", reply)
        return reply


class PlanarKinematicsObservation(ROSObservation):
    """Planar kinematics observation with actuator feedback"""

    OBS = ["z_diff", "planar_dist", "yaw_diff", "vel_diff", "vel", "yaw_vel"]
    OBS_RED = ["z_diff", "planar_dist", "yaw_diff"]
    OBS_range = {
        "z_diff": [-100, 100],  # Dependent from z_range in "target"->_generate_waypoint
        "planar_dist": [0, 100 * np.sqrt(3)],  # Dependent from x_range, y_range, z_range in "target"->_generate_waypoint
        "yaw_diff": [-np.pi, np.pi],
        "vel_diff": [-11.5, 11.5],
        "vel": [0, 11.5],
        "yaw_vel": [-27, 27],
    }

    def __init__(
        self,
        env: "AbstractEnv",
        noise_stdv=0.02,
        scale_obs=True,
        enable_rsdact_feedback=True,  # teacher action
        enable_airspeed_sensor=True,  # add airspeed sensor
        enable_next_goal=True,  # add next goal, only used with multigoal
        enable_actuator_status=True,  # actuator status provided as observation
        weight_interval=None,  # not use the sampling method by default
        sys_ident=False,     # System identification mode
        trigger_dist=5,
        **kwargs: dict
    ) -> None:
        super().__init__(env, **kwargs)
        self.noise_stdv = noise_stdv
        self.scale_obs = scale_obs
        self.enable_rsdact_feedback = enable_rsdact_feedback
        self.enable_airspeed_sensor = enable_airspeed_sensor
        self.enable_next_goal = enable_next_goal
        self.enable_actuator_status = enable_actuator_status
        self.weight_interval = weight_interval
        self.sys_ident = sys_ident

        self.obs_name = self.OBS.copy()
        self.obs_red_name = self.OBS_RED.copy()
        self.obs_dim = len(self.OBS_RED)
        self.range_dict = self.OBS_range
        self.buffer = SystemIdent()

        if self.enable_rsdact_feedback:
            self.obs_dim += 4
            self.obs_name.append("residual_act")
            self.obs_red_name.append("residual_act")

        if self.enable_airspeed_sensor:
            self.obs_dim += 1
            self.obs_name.append("airspeed")
            self.range_dict.update({"airspeed": [0, 7]})

        if self.enable_next_goal:
            self.obs_dim += 1
            self.obs_name.append("next_yaw_diff")
            self.obs_red_name.append("next_yaw_diff")
            self.range_dict.update(
                {
                    "next_yaw_diff": [-np.pi, np.pi],
                }
            )

        if self.enable_actuator_status:
            self.actuator_list = [0, 1, 5, 6]
            self.obs_dim += len(self.actuator_list)
            self.obs_name.append("actuator")
            self.obs_red_name.append("actuator")

        if self.weight_interval is not None:
            self.obs_dim += 1
            self.obs_name.append("random_teacher_weight")
            self.obs_red_name.append("random_teacher_weight")

    def observe(self, joint_act=np.zeros(4), rsdact=np.zeros(4), p=0) -> np.ndarray:
        obs, obs_dict = self._observe(joint_act, rsdact, p)
        while np.isnan(obs).any():
            rospy.loginfo("[ observation ] obs corrupted by NA")
            self.obs_err_handle()
            obs, obs_dict = self._observe(joint_act, rsdact, p)
        return obs, obs_dict

    def _observe(self, joint_act=np.zeros(4), rsdact=np.zeros(4), p=0) -> np.ndarray:
        distance = np.linalg.norm(self.pos_data[0:3] - self.env.goal["position"][0:3])
        obs_dict = {
            "position": self.pos_data,
            "velocity": self.vel_data,
            "velocity_norm": np.linalg.norm(self.vel_data),
            "linear_acceleration": self.acc_data,
            "acceleration_norm": np.linalg.norm(self.acc_data),
            "orientation": self.ori_data,
            "angle": self.ang_data,
            "angular_velocity": self.ang_vel_data,
            "airspeed": self.airspeed_data,
            "distance": distance,
            "joint_action": joint_act,
            "goal_position": self.env.goal["position"],
        }

        goal_dict = self.env.goal
        processed_dict, original_dict = self.process_obs(obs_dict, goal_dict, self.scale_obs)
        if self.enable_rsdact_feedback:
            processed_dict.update({"residual_act": rsdact})

        if self.weight_interval is not None:
            processed_dict.update({"random_teacher_weight": p})

        if self.enable_actuator_status:
            actuator = self.env.action_type.get_cur_act()[self.actuator_list]
            processed_dict.update({"actuator": actuator})

        proc_df = pd.DataFrame.from_records([processed_dict])
        processed = np.hstack(proc_df[self.obs_name].values[0])

        processed_dict_red = processed_dict.copy()
        processed_dict_red.pop("vel_diff")
        processed_dict_red.pop("vel")
        processed_dict_red.pop("yaw_vel")
        proc_df_red = pd.DataFrame.from_records([processed_dict_red])
        processed_red = np.hstack(proc_df_red[self.obs_red_name].values[0])

        obs_dict.update({"proc_dict": processed_dict})
        obs_dict.update({"goal_dict": goal_dict})
        obs_dict.update({"original_dict": original_dict})

        if self.dbg_obs:
            print("[ observation ] state", processed)
            print("[ observation ] obs dict", obs_dict)

        if self.sys_ident:
            a_0 = obs_dict["velocity"][1]
            a_1 = obs_dict["velocity"][0]
            a_2 = obs_dict["velocity"][2]
            a_6 = obs_dict["angular_velocity"][0]
            a_7 = obs_dict["angular_velocity"][1]
            a_8 = obs_dict["angular_velocity"][2]
            a_9 = processed_dict["actuator"]
            a_10 = obs_dict["position"]
            print(f"u: {a_0}\n"
                  f"v: {a_1}\n"
                  f"w: {a_2}\n"
                  f"p: {a_6}\n"
                  f"q: {a_7}\n"
                  f"r: {a_8}\n"
                  f"actuator: {a_9}\n"
                  f"position: {a_10}\n")
            pointer = self.buffer.store(a_0, a_1, a_2, a_6, a_7, a_8)
            if pointer == len(self.buffer.u_buff):
                self.buffer.evaluate()
        return processed_red, obs_dict

    def process_obs(
        self, obs_dict: dict, goal_dict: dict, scale_obs: bool = True
    ) -> dict:
        obs_pos, goal_pos, next_goal_pos, goal_vel = (
            obs_dict["position"],
            goal_dict["position"],
            goal_dict["next_position"],
            goal_dict["velocity"],
        )
        vel = np.linalg.norm(obs_dict["velocity"])
        # vel = obs_dict["airspeed"]
        planar_dist = np.linalg.norm(obs_pos[0:3] - goal_pos[0:3])
        yaw_diff = self.compute_yaw_diff(goal_pos, obs_pos, obs_dict["angle"][2])
        state_dict = {
            "z_diff": obs_pos[2] - goal_pos[2],
            "planar_dist": planar_dist,
            "yaw_diff": yaw_diff,
            "vel_diff": vel - goal_vel,
            "vel": vel,
            "yaw_vel": obs_dict["angular_velocity"][2],
        }
        if self.enable_airspeed_sensor:
            state_dict.update({"airspeed": obs_dict["airspeed"]})

        if self.enable_next_goal:
            next_yaw_diff = self.compute_yaw_diff(
                next_goal_pos, obs_pos, obs_dict["angle"][2]
            )
            state_dict.update({"next_yaw_diff": next_yaw_diff})

        if scale_obs:
            state_dict_proc = self.scale_obs_dict(state_dict, self.noise_stdv)
        else:
            state_dict_proc = state_dict

        return state_dict_proc, state_dict

    def scale_obs_dict(self, state_dict: dict, noise_level: float = 0.0) -> dict:
        for key, val in state_dict.items():
            proc = utils.lmap(val, self.range_dict[key], [-1, 1])
            proc += np.random.normal(0, noise_level, proc.shape)
            proc = np.clip(proc, -1, 1)
            state_dict[key] = proc
        return state_dict

    @classmethod
    def compute_yaw_diff(
        cls, goal_pos: np.array, obs_pos: np.array, obs_yaw: float
    ) -> float:
        """compute yaw angle of the vector machine position to goal position
        then compute the difference of this angle to machine yaw angle
        last, make sure this angle lies within (-pi, pi)

        Args:
            goal_pos (np.array): [machine position]
            obs_pos (np.array): [goal postiion]
            obs_yaw (float): [machine yaw angle]

        Returns:
            float: [yaw angle differences]
        """
        pos_diff = obs_pos - goal_pos
        goal_yaw = np.arctan2(pos_diff[1], pos_diff[0]) - np.pi
        ang_diff = goal_yaw - obs_yaw

        if ang_diff > np.pi:
            ang_diff -= 2 * np.pi
        elif ang_diff < -np.pi:
            ang_diff += 2 * np.pi

        return ang_diff


class SystemIdent:
    def __init__(self, time=20, frequency=10):
        self.u_buff = np.zeros((time * frequency, 1), dtype=np.float32)
        self.v_buff = np.zeros((time * frequency, 1), dtype=np.float32)
        self.w_buff = np.zeros((time * frequency, 1), dtype=np.float32)
        self.p_buff = np.zeros((time * frequency, 1), dtype=np.float32)
        self.q_buff = np.zeros((time * frequency, 1), dtype=np.float32)
        self.r_buff = np.zeros((time * frequency, 1), dtype=np.float32)
        self.ud_buff = np.zeros((time * frequency - 1, 1), dtype=np.float32)
        self.vd_buff = np.zeros((time * frequency - 1, 1), dtype=np.float32)
        self.wd_buff = np.zeros((time * frequency - 1, 1), dtype=np.float32)
        self.pd_buff = np.zeros((time * frequency - 1, 1), dtype=np.float32)
        self.qd_buff = np.zeros((time * frequency - 1, 1), dtype=np.float32)
        self.rd_buff = np.zeros((time * frequency - 1, 1), dtype=np.float32)
        self.frequency = frequency
        self.pointer = 0

    def store(self, u, v, w, p, q, r):
        if self.pointer < len(self.u_buff):
            self.u_buff[self.pointer] = u
            self.v_buff[self.pointer] = v
            self.w_buff[self.pointer] = w
            self.p_buff[self.pointer] = p
            self.q_buff[self.pointer] = q
            self.r_buff[self.pointer] = r
        self.pointer += 1
        return self.pointer

    def evaluate(self):
        for i in range(len(self.ud_buff)):
            self.ud_buff[i] = (self.u_buff[i+1] - self.u_buff[i]) / (1 / self.frequency)
            self.vd_buff[i] = (self.v_buff[i + 1] - self.v_buff[i]) / (1 / self.frequency)
            self.wd_buff[i] = (self.w_buff[i + 1] - self.w_buff[i]) / (1 / self.frequency)
            self.pd_buff[i] = (self.p_buff[i + 1] - self.p_buff[i]) / (1 / self.frequency)
            self.qd_buff[i] = (self.q_buff[i + 1] - self.q_buff[i]) / (1 / self.frequency)
            self.rd_buff[i] = (self.r_buff[i + 1] - self.r_buff[i]) / (1 / self.frequency)
        print(f"u:\n {self.u_buff[2:]}\n\n"
              f"v:\n {self.v_buff[2:]}\n\n"
              f"w:\n {self.w_buff[2:]}\n\n"
              f"u_dot:\n {self.ud_buff[2:]}\n\n"
              f"v_dot:\n {self.vd_buff[2:]}\n\n"
              f"w_dot:\n {self.wd_buff[2:]}\n\n"
              f"p:\n {self.p_buff[2:]}\n\n"
              f"q:\n {self.q_buff[2:]}\n\n"
              f"r:\n {self.r_buff[2:]}\n\n"
              f"p_dot:\n {self.pd_buff[2:]}\n\n"
              f"q_dot:\n {self.qd_buff[2:]}\n\n"
              f"r_dot:\n {self.rd_buff[2:]}\n\n"
              f"u_ave:\n {self.u_buff[2:].mean()}\n\n"
              f"v_ave:\n {self.v_buff[2:].mean()}\n\n"
              f"w_ave:\n {self.w_buff[2:].mean()}\n\n"
              f"u_dot_ave:\n {self.ud_buff[2:].mean()}\n\n"
              f"v_dot_ave:\n {self.vd_buff[2:].mean()}\n\n"
              f"w_dot_ave:\n {self.wd_buff[2:].mean()}\n\n"
              f"p_ave:\n {self.p_buff[2:].mean()}\n\n"
              f"q_ave:\n {self.q_buff[2:].mean()}\n\n"
              f"r_ave:\n {self.r_buff.mean()}\n\n"
              f"p_dot_ave:\n {self.pd_buff[2:].mean()}\n\n"
              f"q_dot_ave:\n {self.qd_buff[2:].mean()}\n\n"
              f"r_dot_ave:\n {self.rd_buff[2:].mean()}\n\n"
              )


class DummyYawObservation(PlanarKinematicsObservation):
    """Planar kinematics observation with actuator feedback"""

    OBS = ["yaw_diff", "yaw_vel"]
    OBS_range = {
        "yaw_diff": [-np.pi, np.pi],
        "yaw_vel": [-15, 15],
    }

    def __init__(
        self,
        env: "AbstractEnv",
        noise_stdv=0.02,
        scale_obs=True,
        enable_rsdact_feedback=True,
        **kwargs: dict
    ) -> None:
        super().__init__(env, noise_stdv=noise_stdv, scale_obs=scale_obs, **kwargs)
        self.enable_rsdact_feedback = enable_rsdact_feedback

        self.obs_name = self.OBS.copy()
        self.obs_dim = len(self.OBS)
        self.range_dict = self.OBS_range

        if self.enable_rsdact_feedback:
            self.obs_dim += 1
            self.obs_name.append("residual_act")

        self.obs_dim += 1
        self.obs_name.append("actuator")

    def observe(self, rsdact=np.array([0.0])) -> np.ndarray:
        obs, obs_dict = self._observe(rsdact)
        while np.isnan(obs).any():
            rospy.loginfo("[ observation ] obs corrupted by NA")
            self.obs_err_handle()
            obs, obs_dict = self._observe(rsdact)
        return obs, obs_dict

    def _observe(self, rsdact=np.array([0.0])) -> np.ndarray:
        obs_dict = {
            "position": self.pos_data,
            "velocity": self.vel_data,
            "velocity_norm": np.linalg.norm(self.vel_data),
            "linear_acceleration": self.acc_data,
            "orientation": self.ori_data,
            "angle": self.ang_data,
            "angular_velocity": self.ang_vel_data,
        }

        goal_dict = self.env.goal
        processed_dict = self.process_obs(obs_dict, goal_dict, self.scale_obs)

        if self.enable_rsdact_feedback:
            processed_dict.update({"residual_act": rsdact})

        actuator = self.env.action_type.get_cur_act()[[0]]
        processed_dict.update({"actuator": actuator})

        proc_df = pd.DataFrame.from_records([processed_dict])
        processed = np.hstack(proc_df[self.obs_name].values[0])

        obs_dict.update({"proc_dict": processed_dict})
        obs_dict.update({"goal_dict": goal_dict})

        if self.dbg_obs:
            print("[ observation ] state", processed)
            print("[ observation ] obs dict", obs_dict)

        return processed, obs_dict

    def process_obs(
        self, obs_dict: dict, goal_dict: dict, scale_obs: bool = True
    ) -> dict:
        obs_pos, goal_pos = obs_dict["position"], goal_dict["position"]
        yaw_diff = self.compute_yaw_diff(goal_pos, obs_pos, obs_dict["angle"][2])

        state_dict = {"yaw_diff": yaw_diff, "yaw_vel": obs_dict["angular_velocity"][2]}

        if scale_obs:
            state_dict = self.scale_obs_dict(state_dict, self.noise_stdv)

        return state_dict


class AerobaticObservation(ROSObservation):
    """aerobatic observation with actuator feedback"""

    OBS = [
        "roll_diff",
        "pitch_diff",
        "yaw_diff",
        "vel_diff",
        "roll_vel_diff",
        "pitch_vel_diff",
        "yaw_vel_diff",
        "z_diff",
        "planar_dist",
        "planar_yaw_diff",
    ]
    OBS_range = {
        "roll_diff": [-np.pi, np.pi],
        "pitch_diff": [-np.pi, np.pi],
        "yaw_diff": [-np.pi, np.pi],
        "vel_diff": [0, 20],
        "roll_vel_diff": [-90, 90],
        "pitch_vel_diff": [-90, 90],
        "yaw_vel_diff": [-90, 90],
        "z_diff": [-100, 100],
        "planar_dist": [0, 200 * np.sqrt(2)],
        "planar_yaw_diff": [-np.pi, np.pi],
    }

    def __init__(
        self,
        env: "AbstractEnv",
        noise_stdv=0.02,
        scale_obs=True,
        enable_airspeed_sensor=False,  # add airspeed sensor
        enable_trig_obs=True,
        **kwargs: dict
    ) -> None:
        super().__init__(env, **kwargs)
        self.noise_stdv = noise_stdv
        self.scale_obs = scale_obs
        self.enable_airspeed_sensor = enable_airspeed_sensor
        self.enable_trig_obs = enable_trig_obs

        self.obs_name = self.OBS.copy()
        self.obs_dim = len(self.OBS)
        self.range_dict = self.OBS_range

        if self.enable_airspeed_sensor:
            self.obs_dim += 1
            self.obs_name.append("airspeed")
            self.range_dict.update({"airspeed": [0, 7]})

        self.actuator_list = [0, 1, 2, 3, 4, 5, 6, 7]
        self.obs_dim += len(self.actuator_list)
        self.obs_name.append("actuator")

        if self.enable_trig_obs:
            self.obs_dim += 2  # sin servo, cos servo

            angle_tri_name = [
                "roll_sin",
                "pitch_sin",
                "yaw_sin",
                "roll_cos",
                "pitch_cos",
                "yaw_cos",
            ]
            self.obs_dim += len(angle_tri_name)
            self.obs_name.extend(angle_tri_name)
            for k in angle_tri_name:
                self.range_dict.update({k: [-1, 1]})

    def observe(self) -> np.ndarray:
        obs, obs_dict = self._observe()
        while np.isnan(obs).any():
            rospy.loginfo("[ observation ] obs corrupted by NA")
            self.obs_err_handle()
            obs, obs_dict = self._observe()
        return obs, obs_dict

    def _observe(self) -> np.ndarray:
        goal_dict = self.env.goal
        obs_dict = {
            "task": goal_dict["task"],
            "position": self.pos_data,
            "velocity": self.vel_data,
            "velocity_norm": np.linalg.norm(self.vel_data),
            "linear_acceleration": self.acc_data,
            "acceleration_norm": np.linalg.norm(self.acc_data),
            "orientation": self.ori_data,
            "angle": self.ang_data,
            "angle_sin": np.sin(self.ang_data),
            "angle_cos": np.cos(self.ang_data),
            "angular_velocity": self.ang_vel_data,
            "airspeed": self.airspeed_data,
        }

        processed_dict = self.process_obs(obs_dict, goal_dict, self.scale_obs)

        actuator = self.env.action_type.get_cur_act()[self.actuator_list]
        if self.enable_trig_obs:
            servo = actuator[5]
            np.append(actuator, np.sin(servo))
            np.append(actuator, np.cos(servo))
        processed_dict.update({"actuator": actuator})

        proc_df = pd.DataFrame.from_records([processed_dict])
        processed = np.hstack(proc_df[self.obs_name].values[0])

        obs_dict.update({"proc_dict": processed_dict})
        obs_dict.update({"goal_dict": goal_dict})

        if self.dbg_obs:
            print("[ observation ] state", processed)
            print("[ observation ] obs dict", obs_dict)

        return processed, obs_dict

    def process_obs(
        self, obs_dict: dict, goal_dict: dict, scale_obs: bool = True
    ) -> dict:
        (
            obs_ang,
            obs_angvel,
            obs_pos,
            obs_vel,
            goal_ang,
            goal_angvel,
            goal_pos,
            goal_vel,
        ) = (
            obs_dict["angle"],
            obs_dict["angular_velocity"],
            obs_dict["position"],
            obs_dict["velocity"],
            goal_dict["angle"],
            goal_dict["angular_velocity"],
            goal_dict["position"],
            goal_dict["velocity"],
        )

        state_dict = {
            "roll_diff": obs_ang[0] - goal_ang[0],
            "pitch_diff": obs_ang[1] - goal_ang[1],
            "yaw_diff": obs_ang[2] - goal_ang[2],
            "vel_diff": np.linalg.norm(obs_vel) - goal_vel,
            "roll_vel_diff": obs_angvel[0] - goal_angvel[0],
            "pitch_vel_diff": obs_angvel[1] - goal_angvel[1],
            "yaw_vel_diff": obs_angvel[2] - goal_angvel[2],
            "z_diff": obs_pos[2] - goal_pos[2],
            "planar_dist": np.linalg.norm(obs_pos[0:2] - goal_pos[0:2]),
            "planar_yaw_diff": self.compute_yaw_diff(
                goal_pos, obs_pos, obs_dict["angle"][2]
            ),
        }

        if self.enable_airspeed_sensor:
            state_dict.update({"airspeed": obs_dict["airspeed"]})

        if self.enable_trig_obs:
            angle_sin, angle_cos = obs_dict["angle_sin"], obs_dict["angle_cos"]
            state_dict.update(
                {
                    "roll_sin": angle_sin[0],
                    "pitch_sin": angle_sin[1],
                    "yaw_sin": angle_sin[2],
                    "roll_cos": angle_cos[0],
                    "pitch_cos": angle_cos[1],
                    "yaw_cos": angle_cos[2],
                }
            )

        if scale_obs:
            state_dict = self.scale_obs_dict(state_dict, self.noise_stdv)

        return state_dict

    def scale_obs_dict(self, state_dict: dict, noise_level: float = 0.0) -> dict:
        for key, val in state_dict.items():
            proc = utils.lmap(val, self.range_dict[key], [-1, 1])
            proc += np.random.normal(0, noise_level, proc.shape)
            proc = np.clip(proc, -1, 1)
            state_dict[key] = proc
        return state_dict

    @classmethod
    def compute_yaw_diff(
        cls, goal_pos: np.array, obs_pos: np.array, obs_yaw: float
    ) -> float:
        """compute yaw angle of the vector machine position to goal position
        then compute the difference of this angle to machine yaw angle
        last, make sure this angle lies within (-pi, pi)

        Args:
            goal_pos (np.array): [machine position]
            obs_pos (np.array): [goal postiion]
            obs_yaw (float): [machine yaw angle]

        Returns:
            float: [yaw angle differences]
        """
        pos_diff = obs_pos - goal_pos
        goal_yaw = np.arctan2(pos_diff[1], pos_diff[0])
        ang_diff = goal_yaw - obs_yaw

        if ang_diff > np.pi:
            ang_diff -= 2 * np.pi
        elif ang_diff < -np.pi:
            ang_diff += 2 * np.pi

        return ang_diff


def observation_factory(env: "AbstractEnv", config: dict) -> ObservationType:
    """observation factory for different observation type"""
    if config["type"] == "PlanarKinematics":
        return PlanarKinematicsObservation(env, **config)
    elif config["type"] == "DummyYaw":
        return DummyYawObservation(env, **config)
    elif config["type"] == "AerobaticObservation":
        return AerobaticObservation(env, **config)
    else:
        raise ValueError("Unknown observation type")
