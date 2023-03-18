import datetime
import os
import rospy
import numpy as np

from blimp_env.envs import ResidualPlanarNavigateEnv
from config import generate_config, save_config
from my_ppo import MyPPO
from util import *

if __name__ == '__main__':
    os.system("~/catkin_ws/src/AutonomousBlimpDRL/blimp_env/blimp_env/envs/script/cleanup.sh")
    # exp setup
    seed = 1
    ENV = ResidualPlanarNavigateEnv
    AGENT = MyPPO
    n_env = 7
    robot_id = 0

    env_name = ENV.__name__
    agent_name = AGENT.__name__
    exp_name = env_name + "_" + agent_name

    pid = True  # Set to true if using PID, to false if using H_infinity

    if pid:
        exp_time = "S1_pid_uni"
    elif not pid:
        exp_time = "S1_hinf_uni"

    # Following settings do not need to be adjusted
    env_default_config = ENV.default_config()
    duration = env_default_config["duration"]
    simulation_frequency = env_default_config["simulation_frequency"]
    policy_frequency = env_default_config["policy_frequency"]

    days = 1
    one_day_ts = 24 * 3600 * policy_frequency
    TIMESTEP = int(days * one_day_ts)

    target_kl = 0.02
    gamma = 0.99
    lam = 0.9
    log_sigma = -1
    actor_size = (64, 64)
    critic_size = (196, 196)
    steps_per_epoch = 1920
    batch_size = 128

    uniform = True
    weight_interval = [0, 1]
    similarity = 0.4
    full_obs = False
    double_att_control = True
    as_ds_threshold = 0
    display = True  # Display the training progress
    gui = False
    wind = False
    log = False
    test_mode = not log
    cpu = False
    rnn = True
    debug = False
    mesh = True
    close_prev_sim = True if robot_id == 0 else False

    if double_att_control:
        max_servo = 1
        min_servo = -1
        max_thrust = 1
        min_thrust = -1
    else:
        max_servo = 0
        min_servo = -1
        max_thrust = 1
        min_thrust = 0
        as_ds_threshold = None


    def exp_training(exp_config, env_config, agent_config):
        agent = AGENT(
            env=subprocvecenv_handle(env_config["n_workers"], ResidualPlanarNavigateEnv, env_config),
            # env=PlanarNavigateEnv(env_config),
            **agent_config,
        )
        rospy.sleep(1)
        agent.train()
        agent.save(exp_config["final_model_save_path"])


    # exp_config
    os.chdir(os.path.expanduser("~/catkin_ws/src/AutonomousBlimpDRL/RL/rl"))
    exp_path = os.path.join("./agent", exp_name, exp_time)

    exp_config = {
        "final_model_save_path": os.path.join(exp_path, "final_model"),
    }

    # env_config
    robot_id = 0
    close_prev_sim = True if robot_id == 0 else False
    env_config = {
        "simulation": {
            "gui": gui,
            "auto_start_simulation": True,
            "enable_wind": wind,
            "enable_meshes": mesh,
            "enable_wind_sampling": wind,
            "enable_buoyancy_sampling": wind,
        },
        "observation": {
            "weight_interval": weight_interval,
            "enable_actuator_status": full_obs,
            "DBG_OBS": debug,
            "enable_next_goal": full_obs,
            "enable_airspeed_sensor": full_obs,
        },
        "action": {
            "type": "SimpleContinuousAction",
            "act_noise_stdv": 0.05,
            # PPO action and joint action restriction!
            "max_servo": max_servo,
            "min_servo": min_servo,
            "max_thrust": max_thrust,
            "min_thrust": min_thrust,
            "dbg_act": False,
        },
        "n_workers": n_env,
        "seed": seed,
        "weight_interval": weight_interval,
        # "duration": 100000,
        "MIMO_controller": False,
        "pid": pid,
        "uniform": uniform,
        "similarity": similarity,
        "test_mode": test_mode,
        "as_ds_threshold": as_ds_threshold,
    }

    # agent_config
    agent_config = {
        "clip_ratio": 0.5,
        "lam": lam,
        "gamma": gamma,
        "cpu": cpu,
        "rnn": rnn,
        "log": log,
        "seed": seed,
        "total_steps": TIMESTEP,
        "n_env": n_env,
        "target_kl": target_kl,
        "max_episode_length": np.Inf,
        "rate": policy_frequency,
        "display": display,
        "steps_per_epoch": steps_per_epoch,
        "batch_size": batch_size,
        "actor_size": actor_size,
        "critic_size": critic_size,
        "log_sigma": log_sigma,
    }

    # start training
    config = generate_config(
        agent_name=agent_name,
        exp_config=exp_config,
        env_config=env_config,
        agent_config=agent_config,
    )
    # print(config)
    save_config(exp_path, config)
    exp_training(**config)
