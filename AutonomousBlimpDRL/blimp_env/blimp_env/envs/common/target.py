from math import pi
from typing import TYPE_CHECKING, Any, Dict, List, Optional, Tuple, Union

import numpy as np
import rospy
from blimp_env.envs.common import utils
from blimp_env.envs.script.blimp_script import respawn_target, spawn_target
from gym import spaces
from librepilot.msg import AutopilotInfo
from transforms3d.euler import euler2quat, quat2euler
from visualization_msgs.msg import InteractiveMarkerInit, Marker, MarkerArray
from geometry_msgs.msg import Point

import time

if TYPE_CHECKING:
    from blimp_env.envs.common.abstract import AbstractEnv


class WayPoint:
    def __init__(self, position=np.zeros(3), velocity=np.zeros(1), angle=np.zeros(3)):
        self.position = np.array(position)
        self.velocity = np.array(velocity)
        self.angle = np.array(angle)

    def to_ENU(self):
        return np.array([self.position[1], self.position[0], -self.position[2]])


class TargetType:
    """abstract target type"""

    def __init__(
        self,
        env: "AbstractEnv",
        **kwargs,  # pylint: disable=unused-argument
    ) -> None:
        self.env = env

    def space(self) -> spaces.Space:
        """get target space"""
        raise NotImplementedError

    def sample(self):
        """sample a goal"""
        raise NotImplementedError()

    def check_connection(self):
        pass


class RandomGoal(TargetType):
    """a random generated goal during training"""

    def __init__(
        self,
        env: "AbstractEnv",
        target_name_space="target_0",
        new_target_every_ts: int = 1200,
        DBG_ROS=False,
        range_dict={"xy": [-105, 105], "z": [-5, -210], "v": [2, 7]},
        **kwargs,  # pylint: disable=unused-argument
    ) -> None:
        super().__init__(env)

        self.target_name_space = target_name_space
        self.dbg_ros = DBG_ROS

        self.target_dim = 7
        self.pos_cmd_data = np.zeros(3)
        self.vel_cmd_data = np.zeros(1)
        self.ang_cmd_data = np.zeros(3)

        self.new_target_every_ts = new_target_every_ts
        self.x_range, self.y_range, self.z_range, self.v_range = (
            range_dict["xy"],
            range_dict["xy"],
            range_dict["z"],
            range_dict["v"],
        )

        self._pub_and_sub = False
        self._create_pub_and_sub()

    def space(self) -> spaces.Space:
        """gym space, only for testing purpose"""
        return spaces.Box(
            low=np.full((self.target_dim), -1),
            high=np.full((self.target_dim), 1),
            dtype=np.float32,
        )

    def _create_pub_and_sub(self) -> None:
        """create publicator and subscriber"""
        self.wp_viz_publisher = rospy.Publisher(
            self.target_name_space + "/rviz_pos_cmd", Marker, queue_size=1
        )
        self._pub_and_sub = True

    def publish_waypoint_toRviz(self, waypoint):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()
        marker.action = marker.ADD
        marker.type = marker.SPHERE
        marker.id = 0
        marker.scale.x, marker.scale.y, marker.scale.z = 2, 2, 2
        marker.color.a, marker.color.r, marker.color.g, marker.color.b = 1, 1, 1, 0
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = (
            waypoint[1],
            waypoint[0],
            -waypoint[2],
        )  ## NED --> rviz(ENU)
        marker.pose.orientation.w = 1
        self.wp_viz_publisher.publish(marker)

    def generate_goal(self):
        x = np.random.uniform(*self.x_range)
        y = np.random.uniform(*self.y_range)
        z = np.random.uniform(*self.z_range)
        pos_cmd = np.array([x, y, z])

        v_cmd = np.random.uniform(*self.v_range)

        phi, the = 0, 0
        psi = np.random.uniform(-pi, pi)
        ang_cmd = np.array([phi, the, psi])
        q_cmd = euler2quat(0, 0, psi)
        return pos_cmd, v_cmd, ang_cmd, q_cmd

    def check_planar_distance(self, waypoint0, waypoint1, min_dist=30):
        """check if planar distance between 2 waypoints are greater than min_dist"""
        dist = np.linalg.norm(waypoint0[0:2] - waypoint1[0:2])
        return dist > min_dist

    def sample_new_goal(self, origin=np.array([0, 0, -100])):
        far_enough = False
        while far_enough == False:
            pos_cmd, v_cmd, ang_cmd, _ = self.generate_goal()
            far_enough = self.check_planar_distance(pos_cmd, origin)

        self.pos_cmd_data = pos_cmd
        self.vel_cmd_data = v_cmd
        self.ang_cmd_data = ang_cmd

    def sample(self) -> Dict[str, np.ndarray]:
        """sample target state

        Returns:
            dict: target info dictionary with key specified by self.target_name
        """
        if self.env.steps % self.new_target_every_ts == 0:
            self.sample_new_goal()
        self.publish_waypoint_toRviz(self.pos_cmd_data)
        return {
            "position": self.pos_cmd_data,
            "velocity": self.vel_cmd_data,
            "angle": self.ang_cmd_data,
        }


class MultiGoal(TargetType):
    """a specified goal sequences."""

    def __init__(
        self,
        env: "AbstractEnv",
        target_name_space="goal_0",
        trigger_dist=5,  # [m] dist to trigger next waypoint
        wp_list=[  # TODO
            (40, 40, -15, 3),
            (40, -40, -15, 3),
            (-40, -40, -15, 3),
            (-40, 40, -15, 3),
        ],  # [m] (x, y, z, v) in NED
        enable_dependent_wp=False,  # waypoint generated depend on previous waypoint
        dist_range=[10, 40],  # [m] new wp range of prev wp
        DBG_ROS=False,
        **kwargs,  # pylint: disable=unused-argument
    ) -> None:
        super().__init__(env)

        self.enable_dependent_wp = enable_dependent_wp

        self.target_name_space = target_name_space
        self.dbg_ros = DBG_ROS
        self.target_dim = 9
        self.min_dist = dist_range[0]
        self.max_dist = dist_range[1]

        self.wp_list = []
        for wp in wp_list:
            self.wp_list.append(WayPoint(wp[0:3], wp[3]))

        self.wp_max_index = len(self.wp_list)
        self.wp_index = 0
        self.next_wp_index = self.wp_index + 1
        self.wp = self.wp_list[self.wp_index]
        self.next_wp = self.wp_list[self.next_wp_index]
        self.trigger_dist = trigger_dist
        print("Tigger distance :", trigger_dist)
        self._pub_and_sub = False
        self._create_pub_and_sub()

    def space(self) -> spaces.Space:
        """gym space, only for testing purpose"""
        return spaces.Box(
            low=np.full((self.target_dim), -1),
            high=np.full((self.target_dim), 1),
            dtype=np.float32,
        )

    def _create_pub_and_sub(self) -> None:
        """create publicator and subscriber"""
        self.wp_viz_publisher = rospy.Publisher(
            self.target_name_space + "/rviz_pos_cmd", Marker, queue_size=1
        )
        self.wplist_viz_publisher = rospy.Publisher(
            self.target_name_space + "/rviz_waypoint_list", MarkerArray, queue_size=10
        )
        self.path_viz_publisher = rospy.Publisher(
            self.target_name_space + "/rviz_path", Marker, queue_size=5
        )
        self._pub_and_sub = True

    def create_rviz_marker(
        self, waypoint: np.array, scale=(2, 2, 2), color=(1, 1, 1, 0)
    ):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()
        marker.action = marker.ADD
        marker.type = marker.SPHERE
        marker.id = 0
        marker.scale.x, marker.scale.y, marker.scale.z = scale
        marker.color.a, marker.color.r, marker.color.g, marker.color.b = color
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = (
            waypoint[1],
            waypoint[0],
            -waypoint[2],
        )  ## NED --> rviz(ENU)
        marker.pose.orientation.w = 1
        return marker

    def publish_waypoint_toRviz(
        self, waypoint: WayPoint, scale: Tuple = (4, 4, 4), color: Tuple = (1, 0, 0, 1)
    ):
        marker = self.create_rviz_marker(waypoint.position, scale=scale, color=color)
        self.wp_viz_publisher.publish(marker)

    def publish_wplist_toRviz(self, wp_list: Tuple[WayPoint]):
        markerArray = MarkerArray()

        for wp in wp_list:
            marker = self.create_rviz_marker(wp.position)
            markerArray.markers.append(marker)

            id = 0
            for m in markerArray.markers:
                m.id = id
                id += 1

            self.wplist_viz_publisher.publish(markerArray)

    def check_planar_distance(self, waypoint0, waypoint1, min_dist=10):
        """check if planar distance between 2 waypoints are greater than min_dist"""
        return np.linalg.norm(waypoint0[0:3] - waypoint1[0:3]) > min_dist

    def close_enough(self, waypoint0, waypoint1, trigger_dist) -> bool:
        """check if planar distance between 2 waypoints are less than trigger_dist"""
        return np.linalg.norm(waypoint0[0:3] - waypoint1[0:3]) < trigger_dist

    def _wp_index_plus_one(self):
        self.wp_index += 1
        if self.wp_index >= self.wp_max_index:
            self.wp_index = 0

        self.next_wp_index = self.wp_index + 1
        if self.next_wp_index >= self.wp_max_index:
            self.next_wp_index = 0

    def sample(self) -> Dict[str, np.ndarray]:
        """sample target state

        Returns:
            dict: target info dictionary with key specified by self.target_name
        """

        if self.env._pub_and_sub:
            if self.close_enough(
                self.wp_list[self.wp_index].position,
                self.env.observation_type.pos_data,
                self.trigger_dist,
            ):
                self._wp_index_plus_one()
                self.wp, self.next_wp = (
                    self.wp_list[self.wp_index],
                    self.wp_list[self.next_wp_index],
                )
                # Reset the internal states of the Hinf controller after hitting each way point
                self.env.base_controller.reset()

            self.publish_waypoint_toRviz(self.wp)
            self.publish_wplist_toRviz(self.wp_list)

        return {
            "position": self.wp.position,
            "velocity": self.wp.velocity,
            "angle": self.wp.angle,
            "next_position": self.next_wp.position,
        }

    def _generate_waypoint(
        self,
        x_range=np.array([-100, 100]),
        y_range=np.array([-100, 100]),
        z_range=np.array([-130, -70]),
        v_range=np.array([0, 0]),
    ):
        x = np.random.uniform(*x_range)
        y = np.random.uniform(*y_range)
        z = np.random.uniform(*z_range)
        v = np.random.uniform(*v_range)
        return np.array([x, y, z]), np.array([v])

    def _generate_valid_waypoint(self, prev_pos_cmd=np.array([0, 0, -100])):
        far_enough = False
        if self.enable_dependent_wp:
            x_range = prev_pos_cmd[0] + np.array([-self.max_dist, self.max_dist])
            y_range = prev_pos_cmd[1] + np.array([-self.max_dist, self.max_dist])

        while far_enough == False:
            pos_cmd, v_cmd = self._generate_waypoint(x_range=x_range, y_range=y_range)
            far_enough = self.check_planar_distance(
                pos_cmd, prev_pos_cmd, min_dist=self.min_dist
            )
        return WayPoint(pos_cmd, v_cmd)

    def _generate_random_wplist(self, n_waypoints, origin=np.array([0, 0, -100])):
        wp_list = []
        wp = WayPoint(origin, 0)
        for _ in range(n_waypoints):
            wp = self._generate_valid_waypoint(prev_pos_cmd=wp.position)
            wp_list.append(wp)
        return wp_list

    def sample_new_wplist(self, n_waypoints=4):
        self.wp_list = self._generate_random_wplist(n_waypoints)
        self.wp_max_index = len(self.wp_list)
        self.wp_index = 0
        self.next_wp_index = self.wp_index + 1
        self.wp = self.wp_list[self.wp_index]
        self.next_wp = self.wp_list[self.next_wp_index]


class AerobaticGoal(TargetType):
    """a fixed aerobatic goal"""

    TASK_TABLE = {
        "stand": 0,  # pitch 90 degree
        "backward": 1,  # control blimp to move backward
        "upside_down": 2,  # pitch 180 degree, or roll 180 degree
        "loop": 3,  # max pitch velocity
        "roll": 4,  # max roll velocity
    }

    def __init__(
        self,
        env: "AbstractEnv",
        target_name_space: str = "goal_0",
        task_name: Union[str, int] = "stand",
        range_dict={"xy": [-105, 105], "z": [-5, -210], "v": [0, 20]},
        robot_init_position=(0, 0, -100),
        DBG_ROS: bool = False,
        **kwargs,  # pylint: disable=unused-argument
    ) -> None:
        super().__init__(env)

        self.task_table = self.TASK_TABLE
        self.total_tasks = len(self.TASK_TABLE)
        if isinstance(task_name, str):
            if task_name != "random":
                self.task = self.task_table[task_name]
            else:
                self.task = self.sample_task()
        elif isinstance(task_name, int):
            self.task = task_name
        else:
            raise ValueError("unrecognzied task")

        self.target_name_space = target_name_space
        self.dbg_ros = DBG_ROS
        self.robot_init_position = robot_init_position

        self.target_dim = 10
        self.x_range, self.y_range, self.z_range, self.v_range = (
            range_dict["xy"],
            range_dict["xy"],
            range_dict["z"],
            range_dict["v"],
        )
        self._init_goal()

        self._pub_and_sub = False
        self._create_pub_and_sub()

    def space(self) -> spaces.Space:
        """gym space, only for testing purpose"""
        return spaces.Box(
            low=np.full((self.target_dim), -1),
            high=np.full((self.target_dim), 1),
            dtype=np.float32,
        )

    def _create_pub_and_sub(self) -> None:
        """create publicator and subscriber"""
        self.wp_viz_publisher = rospy.Publisher(
            self.target_name_space + "/rviz_pos_cmd", Marker, queue_size=1
        )
        self._pub_and_sub = True

    def publish_waypoint_toRviz(self, waypoint):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()
        marker.action = marker.ADD
        marker.type = marker.SPHERE
        marker.id = 0
        marker.scale.x, marker.scale.y, marker.scale.z = 2, 2, 2
        marker.color.a, marker.color.r, marker.color.g, marker.color.b = 1, 1, 1, 0
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = (
            waypoint[1],
            waypoint[0],
            -waypoint[2],
        )  ## NED --> rviz(ENU)
        marker.pose.orientation.w = 1
        self.wp_viz_publisher.publish(marker)

    def _init_goal(self) -> List[tuple]:
        self.pos_cmd_data = np.array(self.robot_init_position)
        self.vel_cmd_data = np.zeros(1)
        self.ang_cmd_data = np.zeros(3)
        self.angvel_cmd_data = np.zeros(3)

        if self.task == int(0):
            self.ang_cmd_data[1] = np.pi / 2

        elif self.task == int(1):
            self.sample_planar_goal()

        elif self.task == int(2):
            self.ang_cmd_data[0] = np.pi
            self.ang_cmd_data[1] = np.pi

        elif self.task == int(3):
            self.angvel_cmd_data[1] = 50

        elif self.task == int(4):
            self.angvel_cmd_data[0] = 50

        else:
            raise ValueError("unrecognized task")

    def sample_task(self):
        return np.random.randint(self.total_tasks)

    def generate_planar_goal(self):
        x = np.random.uniform(*self.x_range)
        y = np.random.uniform(*self.y_range)
        z = np.random.uniform(*self.z_range)
        pos_cmd = np.array([x, y, z])
        v_cmd = np.random.uniform(*self.v_range)

        return pos_cmd, v_cmd

    def check_planar_distance(self, waypoint0, waypoint1, min_dist=30):
        """check if planar distance between 2 waypoints are greater than min_dist"""
        dist = np.linalg.norm(waypoint0[0:2] - waypoint1[0:2])
        return dist > min_dist

    def sample_planar_goal(self, origin=np.array([0, 0, -100])):
        far_enough = False
        while far_enough == False:
            pos_cmd, v_cmd = self.generate_planar_goal()
            far_enough = self.check_planar_distance(pos_cmd, origin)

        self.pos_cmd_data = pos_cmd
        self.vel_cmd_data = v_cmd

    def sample(self) -> Dict[str, np.ndarray]:
        """sample target state

        Returns:
            dict: target info dictionary with key specified by self.target_name
        """
        self.publish_waypoint_toRviz(self.pos_cmd_data)
        return {
            "task": self.task,
            "position": self.pos_cmd_data,
            "velocity": self.vel_cmd_data,
            "angle": self.ang_cmd_data,
            "angular_velocity": self.angvel_cmd_data,
        }


class ROSTarget(TargetType):
    """ROS Abstract Target"""

    def __init__(
        self,
        env: "AbstractEnv",
        target_name_space="target_0",
        DBG_ROS=False,
        **kwargs,  # pylint: disable=unused-argument
    ) -> None:
        super().__init__(env)
        respawn_target(**kwargs)

        self.target_name_space = target_name_space
        self.dbg_ros = DBG_ROS
        self.target_dim = 9

        self.pos_cmd_data = np.array([0, 0, 0])
        self.vel_cmd_data = 0.0
        self.ang_cmd_data = np.array([0, 0, 0])

        self._pub_and_sub = False
        self._create_pub_and_sub()

    def space(self) -> spaces.Space:
        """gym space, only for testing purpose"""
        return spaces.Box(
            low=np.full((self.target_dim), -1),
            high=np.full((self.target_dim), 1),
            dtype=np.float32,
        )

    def _create_pub_and_sub(self) -> None:
        """create publicator and subscriber"""
        rospy.Subscriber(
            self.target_name_space + "/AutopilotInfo",
            AutopilotInfo,
            self._autopilot_info_callback,
        )
        rospy.Subscriber(
            self.target_name_space + "/update_full",
            InteractiveMarkerInit,
            self._goal_cmd_callback,
        )
        self._pub_and_sub = True

    def _autopilot_info_callback(self, msg: AutopilotInfo) -> None:
        """autopilot info msg callback

        Args:
            msg ([AutopilotInfo]): autopilot command from path planner or task manager
        """
        self.vel_cmd_data = msg.VelocityDesired.x

        if self.dbg_ros:
            print(
                "[ Target ] velocity_cmd: ",
                self.vel_cmd_data,
            )

    def _goal_cmd_callback(self, msg: InteractiveMarkerInit) -> None:
        """goal command msg callback including position command and orientation command
        Command is in ENU frame. Need to convert to NED frame

        Args:
            msg ([InteractiveMarkerInit]): positon and orientation command from task manager
        """
        if self._pub_and_sub is True and msg.markers is not None:
            pose = msg.markers[0].pose

            pos = pose.position
            self.pos_cmd_data = np.array([pos.y, pos.x, -pos.z])

            quat = utils.obj2array(pose.orientation)
            self.ang_cmd_data = quat2euler(quat)

        if self.dbg_ros:
            print(
                "[ Target ] position: ",
                self.pos_cmd_data,
            )
            print(
                "[ Target ] angle: ",
                self.ang_cmd_data,
            )

    def check_connection(self):
        raise NotImplementedError

    def sample(self):
        raise NotImplementedError


class InteractiveGoal(ROSTarget):
    """a waypoint in 3D space defined by desired position, velocity and yaw angle"""

    def sample(self) -> Dict[str, np.ndarray]:
        """sample target state

        Returns:
            dict: target info dictionary with key specified by self.target_name
        """
        return {
            "position": self.pos_cmd_data,
            "velocity": self.vel_cmd_data,
            "angle": self.ang_cmd_data,
        }

    def check_connection(self) -> None:
        """check ros connection"""
        while self._pub_and_sub is not True:
            try:
                rospy.logdebug("[ target ] waiting for target startup")
                time.sleep(1)
            except rospy.ROSInterruptException as err:
                rospy.logdebug("unable to establish ros connection:", err)
                break
        rospy.logdebug("target publisher and subscriber started")

        while self.vel_cmd_data == 0.0:
            try:
                rospy.logdebug("[ target ] waiting for velocity subscriber")
                vel_cmd_data = rospy.wait_for_message(
                    self.target_name_space + "/AutopilotInfo",
                    AutopilotInfo,
                    timeout=100,
                )
                self.vel_cmd_data = vel_cmd_data.VelocityDesired.x
            except TimeoutError:
                self.timeout_handle()

        while (self.pos_cmd_data == np.array([0.0, 0.0, 0.0])).all():
            try:
                rospy.logdebug("[ target ] waiting for position subscriber")
                msg = rospy.wait_for_message(
                    self.target_name_space + "/update_full",
                    InteractiveMarkerInit,
                    timeout=100,
                )
                position = msg.markers[0].pose.position
                self.pos_cmd_data = np.array([position.y, position.x, -position.z])
            except TimeoutError:
                self.timeout_handle()
        rospy.logdebug("target ready")

    def timeout_handle(self):
        rospy.logdebug("[ target ] unable to establish ros connection, respawn...")
        reply = respawn_target(**self.env.config["target"])
        rospy.logdebug("target respawn:", reply)
        return reply


def target_factory(env: "AbstractEnv", config: dict) -> TargetType:
    """generate different types of target

    Args:
        config (dict): [config should specify target type]

    Returns:
        TargetType: [a target will generate goal for RL agent]
    """
    if config["type"] == "RandomGoal":
        return RandomGoal(env, **config)
    elif config["type"] == "MultiGoal":
        return MultiGoal(env, **config)
    elif config["type"] == "AerobaticGoal":
        return AerobaticGoal(env, **config)
    elif config["type"] == "InteractiveGoal":
        return InteractiveGoal(env, **config)
    else:
        raise ValueError("Unknown target type")
