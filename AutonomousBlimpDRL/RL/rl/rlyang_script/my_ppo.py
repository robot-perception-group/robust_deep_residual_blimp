import torch
import random
import rospy
import os
import time
import warnings
from model import *
from torch.optim import Adam
from buffer import Buffer, RNNBuffer
from mpi_pytorch import setup_pytorch_for_mpi, sync_params, mpi_avg_grads
from mpi_tools import mpi_avg, num_procs
from control import *
from gym import spaces


class MyPPO:
    def __init__(self, env, seed=0, model=MyModel, optimizer=Adam, steps_per_epoch=1000, rate=100,
                 gamma=0.99, pi_lr=5e-5, v_lr=1e-4, batch_size=100, n_epochs=20, lam=0.95, max_grad_norm=0.5,
                 max_episode_length=int(2e3), clip_ratio=0.2, target_kl=np.Inf, total_steps=int(1e6),
                 squash=False, clamp=True, output_log_sigma=False, log=False, cpu=True, rnn=False,
                 weight_interval=None, activation='tanh', n_env=1, window_size=1, requires_grad=True, log_sigma=-1,
                 display=False, actor_size=(64, 64), critic_size=(128, 128)):
        # # Actor node
        # rospy.init_node('PPO_actor')
        # Choose a seed
        self.seed(seed)
        # Select device
        torch.set_default_dtype(torch.float32)
        self.device = torch.device('cpu')
        if torch.cuda.is_available() and not cpu:
            self.device = torch.device('cuda')
        # Special function to avoid certain slowdowns from PyTorch + MPI combo.
        setup_pytorch_for_mpi()
        self.rate = rate
        self.loop_time = 1 / self.rate
        self.env = env
        if output_log_sigma:
            low = np.append(self.env.action_space.low, -1)
            high = np.append(self.env.action_space.high, 1)
            self.env.action_space = spaces.Box(low=low, high=high, dtype=np.float32)
        self.squash = squash
        self.clamp = clamp
        self.rnn = rnn
        # Sync params across processes
        self.ac = model(self.env.observation_space, self.env.action_space, squash=squash, clamp=clamp,
                        output_log_sigma=output_log_sigma, device=self.device,
                        log=log, rnn=rnn, activation=activation, window_size=window_size,
                        requires_grad=requires_grad, log_sigma=log_sigma,
                        hidden_sizes_a=actor_size, hidden_sizes_c=critic_size).to(self.device)
        sync_params(self.ac)
        self.output_log_sigma = output_log_sigma
        self.optimizer = optimizer
        self.steps_per_epoch = int(steps_per_epoch / num_procs())
        self.total_steps = total_steps
        self.gamma = gamma  # Discount factor
        self.batch_size = int(batch_size)
        self.n_epochs = n_epochs
        self.max_grad_norm = max_grad_norm
        self.clip_ratio = clip_ratio
        self.target_kl = target_kl
        self.max_episode_length = max_episode_length
        self.n_updates = int(self.total_steps / self.steps_per_epoch)
        self.pi_lr = pi_lr
        self.pi_lr_decay = (1e-5 / self.pi_lr) ** (1 / (self.n_updates - 1))
        self.v_lr = v_lr
        self.v_lr_decay = (1e-5 / self.v_lr) ** (1 / (self.n_updates - 1))
        self.global_count = 0
        self.batch_count_a, self.batch_count_c = 0, 0
        self.o_dim = self.env.observation_space.shape[0]
        self.a_dim = self.env.action_space.shape[0]
        self.log = log
        self.n_env = n_env
        self.weight_interval = weight_interval
        self.display = display
        # Set up experience buffer
        if self.rnn:
            self.buf = RNNBuffer(self.o_dim, self.a_dim, self.steps_per_epoch, gamma, lam,
                                 self.ac.pi.n_hidden, self.ac.v.n_hidden, self.ac.pi.n_layers, self.ac.v.n_layers,
                                 n_env)
        else:
            self.buf = Buffer(self.o_dim, self.a_dim, self.steps_per_epoch, gamma, lam, n_env)
        # Set up optimizers for policy and value function
        self.pi_optimizer = self.optimizer(self.ac.pi.parameters(), lr=self.pi_lr)
        self.v_optimizer = self.optimizer(self.ac.v.parameters(), lr=self.v_lr)
        # Initialize wandb
        if self.log:
            try:
                wandb.init(project="Blimp_paper")
                wandb.define_metric('Update count')
                wandb.define_metric('Real step')

                wandb.define_metric('Teacher weighting', step_metric='Real step')
                wandb.define_metric('Actor update', step_metric='Real step')
                wandb.define_metric('Critic update', step_metric='Real step')
                wandb.define_metric('log(sigma)', step_metric='Real step')
                wandb.define_metric('Sigma', step_metric='Real step')
                wandb.define_metric('Update count', step_metric='Real step')
                wandb.define_metric('Episode reward', step_metric='Real step')
                wandb.define_metric('Episode length', step_metric='Real step')
                wandb.define_metric('Actor learning rate', step_metric='Update count')
                wandb.define_metric('Critic learning rate', step_metric='Update count')
                wandb.define_metric('Actor loss (negative L)', step_metric='Actor update')
                wandb.define_metric('KL', step_metric='Actor update')
                wandb.define_metric('Entropy', step_metric='Actor update')
                wandb.define_metric('Critic loss', step_metric='Critic update')
            except:
                pass
        print("Observation space: ", self.env.observation_space)
        print("Action space: ", self.env.action_space)

    def loss_pi(self, buffer, batch_index):
        # pi: action distribution; logpi: log probability of getting 'a' according to 'pi'
        o, a, A, logpi_old = buffer['o'][batch_index], buffer['a'][batch_index], \
                             buffer['A'][batch_index], buffer['logpi'][batch_index]
        o = o.to(self.device)
        a = a.to(self.device)
        A = A.to(self.device)
        if self.rnn:
            h_pi, c_pi, = buffer['h_pi'][batch_index], buffer['c_pi'][batch_index]
            h_pi = h_pi.to(self.device)
            c_pi = c_pi.to(self.device)
        logpi_old = logpi_old.to(self.device)
        if self.rnn:
            pi, logpi = self.ac.pi(o, h_pi, c_pi, a)
        else:
            # For 3-output network
            if self.output_log_sigma:
                pi, logpi = self.ac.pi(o, a[:, :-1])
            # For 2-output network
            else:
                pi, logpi = self.ac.pi(o, a)
        ratio = torch.exp(logpi - logpi_old)
        # clipped_A will not go over A*(1+e) if A is positive, but it can be "pulled" due to the clip while.
        # (e.g.: 0.7 (ratio)<0.8 (1-e) so 0.7*A will be clipped to 0.8*A. But 0.7*A is needed)
        clipped_A = torch.clamp(ratio, 1 - self.clip_ratio, 1 + self.clip_ratio) * A
        # Solve the problem above with another min. Then convert the surrogate advantage to a negative loss.
        # The reason is that we are aiming to minimize the loss thus to maximize the L
        loss_pi = -torch.min(ratio * A, clipped_A).mean()
        # Extra information
        approx_kl = (logpi_old - logpi).mean().item()
        # Entropy
        H = pi.entropy().mean().item()
        pi_info = dict(kl=approx_kl, H=H)
        return loss_pi, pi_info

    def loss_v(self, buffer, batch_index):
        o, R = buffer['o'][batch_index], buffer['R'][batch_index]
        o = o.to(self.device)
        R = R.to(self.device)
        if self.rnn:
            h_v, c_v = buffer['h_v'][batch_index], buffer['c_v'][batch_index]
            h_v = h_v.to(self.device)
            c_v = c_v.to(self.device)
            loss_v = ((self.ac.v(o, h_v, c_v) - R) ** 2).mean()
        else:
            loss_v = ((self.ac.v(o) - R) ** 2).mean()
        return loss_v

    def update_a(self, buffer, batch_index):
        # Actor update
        self.pi_optimizer.zero_grad()
        loss_pi, pi_info = self.loss_pi(buffer, batch_index)
        if mpi_avg(pi_info['kl']) > 1.5 * self.target_kl:
            pass
        else:
            loss_pi.backward()
            # average grads across MPI processes
            mpi_avg_grads(self.ac.pi)
            # Clip grads
            nn.utils.clip_grad_norm_(self.ac.pi.parameters(), self.max_grad_norm)
            self.pi_optimizer.step()
        if self.log:
            try:
                wandb.log({'Actor loss (negative L)': loss_pi,
                           'KL': mpi_avg(pi_info['kl']), 'Entropy': mpi_avg(pi_info['H']),
                           'Actor update': self.batch_count_a})
            except:
                pass
        return loss_pi, pi_info

    def update_c(self, buffer, batch_index):
        # Critic update
        self.v_optimizer.zero_grad()
        loss_v = self.loss_v(buffer, batch_index)
        loss_v.backward()
        # average grads across MPI processes
        mpi_avg_grads(self.ac.v)
        # Clip grads
        nn.utils.clip_grad_norm_(self.ac.v.parameters(), self.max_grad_norm)
        self.v_optimizer.step()
        if self.log:
            try:
                wandb.log({'Critic loss': loss_v, 'Critic update': self.batch_count_c})
            except:
                pass
        return loss_v

    def recalculate_A_buff(self):
        with torch.no_grad():
            o = torch.as_tensor(self.buf.o_buff)
            o = o.to(self.device)
            new_v = np.zeros_like(self.buf.v_buff)
            if self.rnn:
                h_v, c_v = torch.as_tensor(self.buf.h_v_buff), \
                           torch.as_tensor(self.buf.c_v_buff)
                h_v = h_v.to(self.device)
                c_v = c_v.to(self.device)
                for i in range(o.shape[1]):
                    new_v[:, i] = self.ac.v(o[:, i, :], h_v[:, i, :, :], c_v[:, i, :, :]).cpu().squeeze(dim=1)
            else:
                new_v = self.ac.v(o)
                new_v = new_v.cpu().numpy()
            self.buf.recalculate_A(new_v)

    def update(self, count):
        buffer = self.buf.get()
        # Prepare for logging
        log_loss_pi = np.zeros(int(self.n_epochs * self.n_env * self.steps_per_epoch / self.batch_size))
        log_loss_v = np.zeros(int(self.n_epochs * self.n_env * self.steps_per_epoch / self.batch_size))
        log_kl = np.zeros(int(self.n_epochs * self.n_env * self.steps_per_epoch / self.batch_size))
        log_H = np.zeros(int(self.n_epochs * self.n_env * self.steps_per_epoch / self.batch_size))
        # Actor update count
        k = 0
        # Critic update count
        n = 0
        # Update pi and v by n_epochs times for one epoch
        for i in range(self.n_epochs):
            start_idx = 0
            buffer_index = np.random.permutation(self.n_env * self.steps_per_epoch)
            # Update v throughout one epoch by mini-batches
            while start_idx < self.n_env * self.steps_per_epoch:
                batch_index = buffer_index[start_idx:(start_idx + self.batch_size)]
                self.batch_count_c += 1
                loss_v = self.update_c(buffer, batch_index)
                log_loss_v[n] = loss_v
                start_idx += self.batch_size
                n += 1
            # Recalculate advantage before updating actor
            self.recalculate_A_buff()
            buffer = self.buf.get()
            # Update pi throughout one epoch by mini-batches (seperated from v loop because of
            # possible abruption due to kl)
            start_idx = 0
            while start_idx < self.n_env * self.steps_per_epoch:
                batch_index = buffer_index[start_idx:(start_idx + self.batch_size)]
                self.batch_count_a += 1
                loss_pi, pi_info = self.update_a(buffer, batch_index)
                # Prepare log list
                log_loss_pi[k] = loss_pi
                log_kl[k] = pi_info['kl']
                log_H[k] = pi_info['H']
                start_idx += self.batch_size
                k += 1
        if self.log:
            try:
                wandb.log({'Actor learning rate': self.pi_lr, 'Critic learning rate': self.v_lr,
                           'Update count': count})
            except:
                pass
        # Reset learning rate
        # Set up optimizers for policy and value function
        self.pi_lr *= self.pi_lr_decay
        self.v_lr *= self.v_lr_decay
        self.pi_optimizer = self.optimizer(self.ac.pi.parameters(), lr=self.pi_lr)
        self.v_optimizer = self.optimizer(self.ac.v.parameters(), lr=self.v_lr)

    def train(self):
        episode_R, episode_length, a = np.zeros(self.n_env), \
                                       np.zeros(self.n_env, dtype=int), \
                                       torch.zeros(self.n_env, self.env.action_space.shape[0])

        o = self.env.env_method(method_name="reset")
        self.global_count = 0
        o = torch.as_tensor(np.array(o), dtype=torch.float32).to(self.device)

        for i in range(self.n_updates):
            for j in range(self.steps_per_epoch):
                now = time.time()
                with torch.no_grad():
                    o = torch.as_tensor(o, dtype=torch.float32).to(self.device)
                    if self.log and self.weight_interval is not None:
                        try:
                            wandb.log({'Teacher weighting': o[-1]})
                        except:
                            pass
                    if self.rnn:
                        a, v, logpi, h_pi, c_pi, h_v, c_v, pi = self.ac.step(o)
                    else:
                        a, v, logpi, pi = self.ac.step(o)
                    # Unscale needed for a [-1,1]-clamped or a tanh-squashed output
                    if self.clamp or self.squash:
                        clamped_a = torch.clamp(input=torch.as_tensor(a), min=-1, max=1)
                        actual_a = unscale_a(clamped_a,
                                             low=torch.as_tensor(self.env.action_space.low),
                                             high=torch.as_tensor(self.env.action_space.high))
                    # Otherwise perform a normal clip due to the environment restriction
                    else:
                        actual_a = torch.clamp(input=torch.as_tensor(a),
                                               min=torch.as_tensor(self.env.action_space.low),
                                               max=torch.as_tensor(self.env.action_space.high))
                    if self.output_log_sigma:
                        actual_a = actual_a[:-1]
                    if self.rnn:
                        actual_a = actual_a.squeeze()
                    # Step the env. Note that if any sub_env is done, it will be reset automatically.
                    # The next_o will be the reset observation from this sub_env
                    actual_a = actual_a.cpu().numpy()
                    # # Dimension trick for 1 env
                    if self.n_env == 1 and self.rnn:
                        actual_a = np.expand_dims(actual_a, axis=0)
                    next_o, reward, done, _ = self.env.step(actions=actual_a)
                    episode_R += reward
                    episode_length += np.ones_like(episode_length, dtype=int)
                    # Save values in memory buffer
                    o = o.cpu().numpy()
                    if self.rnn:
                        self.buf.store(o, a, reward, v, logpi, h_pi, c_pi, h_v, c_v)
                    else:
                        self.buf.store(o, a, reward, v, logpi)
                    next_o = torch.as_tensor(next_o, dtype=torch.float32).to(self.device)
                    # Update o
                    o = next_o
                    # "stop" when the target is reached or the max episode length is reached
                    end_episode = (episode_length == self.max_episode_length)
                    stop = np.logical_or(end_episode, done)
                    # "end_epoch" when the designed epoch length is reached
                    end_epoch = (j == (self.steps_per_epoch - 1))
                    if any(stop) or end_epoch:
                        if any(stop):
                            try:
                                wandb.log({'Episode reward': episode_R[np.where(stop)].mean() /
                                                             episode_length[np.where(stop)].mean(),
                                           'Episode length': episode_length[np.where(stop)].mean()})
                            except:
                                pass
                            episode_R[np.where(stop)], episode_length[np.where(stop)] = 0, 0
                        if end_epoch or any(end_episode):
                            if self.rnn:
                                _, v, _, _, _, _, _, _ = self.ac.step(o)
                            else:
                                _, v, _, _ = self.ac.step(o)
                            if any(end_episode):
                                # indices: Tuple to array to list
                                o_array = self.env.env_method(method_name="reset",
                                                              indices=np.asarray(np.where(end_episode)).
                                                              squeeze().tolist())
                                o[np.where(end_episode), :] = torch.as_tensor(o_array, dtype=torch.float32).\
                                    to(self.device)
                                # # Reset controller if end_episode
                                # if self.teacher:
                                #     self.env.env_method(method_name='reset_v_controller',
                                #                         indices=np.asarray(np.where(end_episode)).
                                #                         squeeze().tolist())
                                #     self.env.env_method(method_name='reset_omega_controller',
                                #                         indices=np.asarray(np.where(end_episode)).
                                #                         squeeze().tolist())
                        # "else" is activated iif any environment is "done"
                        else:
                            # # Reset controller if done
                            # if self.teacher:
                            #     self.env.env_method(method_name='reset_v_controller',
                            #                         indices=np.asarray(np.where(done)).
                            #                         squeeze().tolist())
                            #     self.env.env_method(method_name='reset_omega_controller',
                            #                         indices=np.asarray(np.where(done)).
                            #                         squeeze().tolist())
                            if self.rnn:
                                _, v, _, _, _, _, _, _ = self.ac.step(o)
                            else:
                                _, v, _, _ = self.ac.step(o)
                            # Dimension trick for 1 env
                            if self.n_env == 1:
                                v = v.ravel()
                            v[np.where(done)] = 0
                        # Store the current "finish-path" information into the info center. Prepare for the
                        # A_buff update while updating critic
                        self.buf.info_center(v, finish_indices=(np.logical_or(stop, end_epoch)),
                                             start_index=self.buf.path_start_index,
                                             pointer=self.buf.pointer)
                        # Finish the current path
                        self.buf.finish_path(v, finish_indices=(np.logical_or(stop, end_epoch)))
                        # Reset lstm states either when time out or done (after resetting or deep resetting the env)
                        if self.rnn and (any(np.logical_or(end_episode, done))):
                            self.ac.pi.reset_hc()
                    # global count for logging
                    if self.log:
                        try:
                            wandb.log({'Real step': self.global_count})
                        except:
                            pass
                    self.global_count += 1
                    # rospy.Rate(self.rate).sleep()
                    if self.display:
                        if self.global_count % self.rate == 0:
                            # if self.n_env == 1:
                            #     print(f"Progress: {self.global_count} / {self.total_steps} ( {100 * self.global_count / self.total_steps} % )\n "
                            #           f"Reward: {reward}\n"
                            #           f"Observation: {o}\n"
                            #           f"PPO Action: {actual_a}\n")
                            # else:
                            print(f"Progress: {self.global_count} / {self.total_steps} ( {100 * self.global_count / self.total_steps} % )\n"
                                  f"Reward: {reward[0]}\n"
                                  f"Observation: {o[0]}\n"
                                  f"PPO Action: {actual_a[0]}\n")
                    elapsed = time.time() - now
                    if self.loop_time - elapsed > 0:
                        time.sleep(self.loop_time - elapsed)  # Make sure the loop runs at policy frequency
                    else:
                        warnings.warn("The loop frequency cannot reach the expected policy frequency!")
                        print(f"Loop frequency is slower than expected by: {1/self.loop_time - 1/elapsed} Hz\n\n")
            self.env.env_method(method_name="stop")
            # self.env.stop()
            # Use one memory buffer to update
            self.update(i)
            self.buf.clear_info_center()
            self.env.env_method(method_name="go")

    def save(self, path):
        torch.save(self.ac, path)

    def load(self, path):
        return torch.load(path)

    def seed(self, seed=0):
        # Seed python RNG
        random.seed(seed)
        # Seed numpy RNG
        np.random.seed(seed)
        # seed the RNG for all devices (both CPU and CUDA)
        torch.manual_seed(seed)
