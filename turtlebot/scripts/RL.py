import rospy
from stable_baselines3.common.vec_env import SubprocVecEnv
from os.path import exists
from my_ppo import *
from scipy.io import savemat
import time
import os


def make_env(total_timesteps, teacher, seed=0, rate=1000, rank=1, test_mode=False, pid=False):
    """
    Utility function for multiprocessed env.

    :param env_id: (str) the environment ID
    :param num_env: (int) the number of environments you wish to have in subprocesses
    :param seed: (int) the inital seed for RNG
    :param rank: (int) index of the subprocess
    """
    def _init():
        env = TurtleEnv(total_timesteps=total_timesteps, teacher=teacher, seed=seed + rank, rate=rate, index=rank,
                        test_mode=test_mode, teacher_weight=teacher_weight, weight_interval=weight_interval,
                        noise_lim=noise, pid=pid)
        return env
    return _init


if __name__ == '__main__':
    # Preprocessing
    steps_per_epoch = 1000
    batch_size = 100
    n_epochs = 20
    rate = 100
    total_timesteps = int(5e4)
    n_env = 8
    log = False
    teacher = True
    teacher_weight = 0.5
    if teacher and rate > 100:
        rate = 100
    # weight_interval = None
    requires_grad = True
    log_sigma = -1
    rnn = True
    cpu = False
    noise = [1, 1]
    os.chdir(os.path.expanduser("~/catkin_ws/src/turtlebot/"))
    # Hyper parameters
    clip_ratio = 0.25
    lam = 0.95
    gamma = 0.999
    for group_index in range(4):
        for seed_index in range(3):

            seed = seed_index + 1
            group = group_index + 1

            # if group == 1 or group == 2 or group == 3 or (group == 4 and (seed == 1 or seed == 2)):
            #     continue

            if group % 2 == 0:
                # Group 2 and 4: Hinf
                pid = False
                controller_name = "hinf"
            else:
                # Group 1 and 3: PID
                pid = True
                controller_name = "pid"

            if group == 1 or group == 2:
                weight_interval = [0, 0]
            elif group == 3 or group == 4:
                weight_interval = [0, 1]
            else:
                raise("Error in group_index")
            agent_path = "agent/G" + str(group) + "_S" + str(seed) + ".zip"
            group_name = str(group_index+1)

            env = SubprocVecEnv(
                [make_env(total_timesteps=total_timesteps, teacher=teacher, rank=i + 1, rate=rate, pid=pid)
                 for i in range(n_env)])
            time.sleep(0.1)
            # env = TurtleEnv(total_timesteps=total_timesteps, teacher=teacher, seed=seed, index=1)
            # squash: tanh squash for the policy network output
            # clamp: clamp the policy network output to [-1, 1] (only taking place while interacting with ROS)
            # output_log_sigma: add one output more to the policy network give log_sigma for all actions
            agent = MyPPO(env=env, seed=seed, rate=rate, cpu=cpu, rnn=rnn, clip_ratio=clip_ratio,
                          log=log, teacher=teacher, n_env=n_env, lam=lam, gamma=gamma,
                          requires_grad=requires_grad, log_sigma=log_sigma, group=group, steps_per_epoch=steps_per_epoch,
                          batch_size=batch_size, n_epochs=n_epochs)
            agent.train()
            agent.save(agent_path)

            wandb.finish()
            time.sleep(2)
            # os.system("rosnode kill " + 'PPO_actor_'+"G" + str(group) + "_S" + str(seed))
            # if exists(agent_path):
            #     seed = seed + 1
            #     n_env = 1
            #     noise = 10 * noise
            #     steps = 500
            #     env = SubprocVecEnv([make_env(total_timesteps=total_timesteps, teacher=teacher, rank=i+1, rate=rate,
            #                                   test_mode=True)
            #                          for i in range(n_env)])
            #     rospy.sleep(0.1)
            #     agent = MyPPO(env=env, seed=seed, rate=rate, cpu=True, rnn=rnn, log=False, teacher=teacher,
            #                   n_env=n_env).load(agent_path)
            #     obs = env.reset()
            #     done_count = 0
            #     omega = []
            #     v = []
            #     i = 0
            #     while done_count < steps:
            #         action = agent.predict(obs)
            #         obs, rewards, dones, info = env.step(actions=action)
            #         if dones:
            #             done_count += 1
            #         v.append(info['action'][0])
            #         omega.append(info['action'][1])
            #         i = i + 1
            #         if (steps % rate == 0):
            #             print(f"Progress: {i} / {steps} ( {100 * i / steps} % )\n"
            #                   f"Done: {done_count}\n"
            #                   f"Observation: {obs}\n"
            #                   f"Reward: {rewards}\n"
            #                   f"Distance: {info['distance']}")
            #             # print("Theta_target", self.theta_target)
            #             # print("Theta: ", self.pos.theta)
            #             # print("Delta theta", self.delta_theta)
            #             print("\n")
            #         rospy.Rate(rate).sleep()
            #     dict = {'done_count': done_count, 'v': v, "omega": omega}
            #     file_name = test_name + ".mat"
            #     savemat(file_name, dict)
            # else:

