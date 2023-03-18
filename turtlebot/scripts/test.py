import torch
from stable_baselines3.common.vec_env import SubprocVecEnv
from os.path import exists
from my_ppo import *
from scipy.io import savemat




if __name__ == '__main__':
    rate = 100
    total_timesteps = int(1e6)
    seed = 11
    log = False
    teacher = True
    teacher_weight = 0
    # weight_interval = None
    requires_grad = True
    log_sigma = -1
    rnn = True
    cpu = False
    noise = [5, 5]
    os.chdir(os.path.expanduser("~/catkin_ws/src/turtlebot/"))

    # Hyper parameters
    clip_ratio = 0.25
    lam = 0.95
    gamma = 0.999

    # Groups
    ng = 6
    # Seeds
    ns = 3

    # Preprocessing
    if teacher and rate > 100:
        rate = 100
    n_env = 1
    steps = 100
    # for i in range(7):
    #     if i == 0:
    #         agent_path = "/home/yang/catkin_ws/src/turtlebot/agent/1_random0.5.zip"
    #         test_name = "1"
    #         weight_interval = [1, 1]
    #     elif i == 1:
    #         agent_path = "/home/yang/catkin_ws/src/turtlebot/agent/1_random0.5.zip"
    #         test_name = "2"
    #         weight_interval = [0.5, 0.5]
    #     elif i == 2:
    #         agent_path = "/home/yang/catkin_ws/src/turtlebot/agent/2_constant0.5.zip"
    #         test_name = "3"
    #         weight_interval = [0.5, 0.5]
    #     elif i == 3:
    #         agent_path = "/home/yang/catkin_ws/src/turtlebot/agent/3_constant0.zip"
    #         test_name = "4"
    #         weight_interval = [0.5, 0.5]
    #     elif i == 4:
    #         agent_path = "/home/yang/catkin_ws/src/turtlebot/agent/1_random0.5.zip"
    #         test_name = "5"
    #         weight_interval = [0, 0]
    #     elif i == 5:
    #         agent_path = "/home/yang/catkin_ws/src/turtlebot/agent/2_constant0.5.zip"
    #         test_name = "6"
    #         weight_interval = [0, 0]
    #     elif i == 6:
    #         agent_path = "/home/yang/catkin_ws/src/turtlebot/agent/3_constant0.zip"
    #         test_name = "7"
    #         weight_interval = [0, 0]
    test_num = 0
    for i in range(1, ng+1):
        if i % 2 == 0:
            # i = 2,4,6 (Hinf)
            pid = False
        else:
            # i = 1,3,5 (PID)
            pid = True
        for j in range(1, ns+1):
            if i == 1 or i == 2:
                # if j >= 2:
                #     continue
                weight_interval = [1, 1]
                string = "G" + str(i) + "_S" + str(j) + ".zip"
                agent_path = "~/catkin_ws/src/turtlebot/agent/" + string
            elif i == 3 or i == 4:
                # # After G1_S3, agents are trained on cuda
                # cpu = False
                # if i == 4 and (j == 1):
                #     cpu = True
                # # G1_S2 might be faulty
                # if i == 3 and j == 1:
                #     j = 8
                # elif i == 3 and j == 2:
                #     j = 9
                #     cpu = True
                # elif i == 3 and j == 3:
                #     j = 10
                #     cpu = True
                weight_interval = [0, 0]
                string = "G" + str(i-2) + "_S" + str(j) + ".zip"
                agent_path = "~/catkin_ws/src/turtlebot/agent/" + string
            elif i == 5 or i == 6:
                weight_interval = [0.5, 0.5]
                string = "G" + str(i-2) + "_S" + str(j) + ".zip"
                agent_path = "~/catkin_ws/src/turtlebot/agent/" + string
            else:
                raise "Group amount incorrect!"

            env = TurtleEnv(total_timesteps=total_timesteps, teacher=teacher, seed=seed, rate=rate, index=1,
                            test_mode=True, teacher_weight=teacher_weight, weight_interval=weight_interval,
                            noise_lim=noise, pid=pid)
            rospy.sleep(0.1)
            agent = MyPPO(env=env, seed=seed, rate=rate, cpu=cpu, rnn=rnn, log=False, teacher=teacher,
                          n_env=n_env, test_mode=True).load(agent_path)
            seed = seed
            obs = env.deep_reset()
            done_count = 0
            omega = []
            v = []
            k = 0
            while done_count < steps:
                if not cpu:
                    obs = torch.tensor(obs).to(torch.device('cuda'))
                action = agent.predict(obs)
                action = action.squeeze()
                obs, rewards, dones, info = env.step(action)
                if dones:
                    done_count += 1
                    env.reset()
                    env.reset_v_controller()
                    env.reset_omega_controller()
                v.append(info['action'][0])
                omega.append(info['action'][1])
                k = k + 1
                if k % rate == 0:
                    print(f"Progress: {done_count} / {steps} ( {100 * done_count / steps} % )\n"
                          f"Group Number: {i}\n"
                          f"Seed Number: {j}\n"
                          f"Steps: {k}\n"
                          f"Observation: {obs}\n"
                          f"Reward: {rewards}\n"
                          f"Distance: {info['distance']}")
                    # print("Theta_target", self.theta_target)
                    # print("Theta: ", self.pos.theta)
                    # print("Delta theta", self.delta_theta)
                    print("\n")
                rospy.Rate(rate).sleep()
            test_num += 1
            dict = {'done_count': done_count, 'v': v, "omega": omega, "t": k}
            file_name = "Turtle" + str(test_num) + ".mat"
            savemat(file_name, dict)




