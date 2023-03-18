import numpy as np
import torch
from mpi_tools import mpi_statistics_scalar


class Buffer:
    def __init__(self, o_dim, a_dim, size, gamma=0.99, lam=0.95, n_env=1):
        self.o_buff = np.zeros((n_env, size, o_dim), dtype=np.float32)
        self.a_buff = np.zeros((n_env, size, a_dim), dtype=np.float32)
        self.A_buff = np.zeros((n_env, size), dtype=np.float32)
        self.reward_buff = np.zeros((n_env, size), dtype=np.float32)
        self.R_buff = np.zeros((n_env, size), dtype=np.float32)
        self.v_buff = np.zeros((n_env, size), dtype=np.float32)
        self.logpi_buff = np.zeros((n_env, size), dtype=np.float32)
        self.gamma, self.lam = gamma, lam
        self.n_env = n_env
        self.o_dim = o_dim
        self.a_dim = a_dim
        self.pointer, self.path_start_index, self.max_size = np.zeros(n_env, dtype=int), np.zeros(n_env, dtype=int), \
                                                             size
        # Info center
        self.clear_info_center()

    def clear_info_center(self):
        self.info_last_v = []
        self.info_finish_indices = []
        self.info_start_index = []
        self.info_pointer = []

    def normalize_A(self):
        A_mu, A_sigma = mpi_statistics_scalar(self.A_buff.reshape((-1)))
        self.A_buff = (self.A_buff - A_mu) / (A_sigma + 1e-8)

    def store_mlp_data(self, o, a, reward, v, logpi):
        self.o_buff[:, self.pointer[0], :] = o
        self.a_buff[:, self.pointer[0], :] = a
        self.reward_buff[:, self.pointer[0]] = reward
        self.v_buff[:, self.pointer[0]] = v
        self.logpi_buff[:, self.pointer[0]] = logpi

    def store(self, o, a, reward, v, logpi):
        assert all(self.pointer) < self.max_size
        assert all(self.pointer[:-1] == self.pointer[1:])
        self.store_mlp_data(o, a, reward, v, logpi)
        self.pointer += np.ones_like(self.pointer, dtype=int)

    def recalculate_A(self, v):
        assert self.v_buff.shape == v.shape
        self.v_buff = v
        for i in range(len(self.info_last_v)):
            self.split_path(self.info_last_v[i], self.info_finish_indices[i], self.info_start_index[i],
                            self.info_pointer[i])

    def info_center(self, last_v=None, finish_indices=None, start_index=None, pointer=None):
        self.info_last_v.append(last_v)
        self.info_finish_indices.append(finish_indices)
        self.info_start_index.append(start_index)
        self.info_pointer.append(pointer)

    def split_path(self, last_v=None, finish_indices=None, path_start_index=None, pointer=None):
        finish_envs = np.where(finish_indices)[0]
        # Dimension trick for 1 env
        if self.n_env == 1:
            last_v = last_v.ravel()
        for j in range(len(finish_envs)):
            # Extract the indices from the beginning to the pointer indicating point (terminal state)
            path_slice = slice(path_start_index[finish_envs[j]], pointer[finish_envs[j]])
            # Append a "0" at the end of array "self.reward_buff[path_slice]".
            # The "path_slice" ensures that "self.reward_buff[path_slice]" is an array.
            rewards = np.append(self.reward_buff[finish_envs[j], path_slice], last_v[finish_envs[j]])
            values = np.append(self.v_buff[finish_envs[j], path_slice], last_v[finish_envs[j]])
            n = len(rewards)
            # GAE
            # Sum from the end to beginning (CS 285, Lecture 6, Part 4, 14:04)
            GAE = np.zeros(n, )
            for i in reversed(range(n)):
                GAE[i] = rewards[i] + self.gamma * ((1 - self.lam) * (values[i + 1] if i + 1 < n else 0) +
                                                    self.lam * (GAE[i + 1] if i + 1 < n else 0))
            self.A_buff[finish_envs[j], path_slice] = GAE[:-1]

    def finish_path(self, last_v=None, finish_indices=None):
        finish_envs = np.where(finish_indices)[0]
        # Dimension trick for 1 env
        if self.n_env == 1:
            last_v = last_v.ravel()
        for j in range(len(finish_envs)):
            # Extract the indices from the beginning to the pointer indicating point (terminal state)
            path_slice = slice(self.path_start_index[finish_envs[j]], self.pointer[finish_envs[j]])
            # Append a "0" at the end of array "self.reward_buff[path_slice]".
            # The "path_slice" ensures that "self.reward_buff[path_slice]" is an array.
            rewards = np.append(self.reward_buff[finish_envs[j], path_slice], last_v[finish_envs[j]])
            values = np.append(self.v_buff[finish_envs[j], path_slice], last_v[finish_envs[j]])
            n = len(rewards)
            # GAE
            # Sum from the end to beginning (CS 285, Lecture 6, Part 4, 14:04)
            GAE = np.zeros(n, )
            for i in reversed(range(n)):
                GAE[i] = rewards[i] + self.gamma * ((1 - self.lam) * (values[i + 1] if i + 1 < n else 0) +
                                                    self.lam * (GAE[i + 1] if i + 1 < n else 0))
            self.A_buff[finish_envs[j], path_slice] = GAE[:-1]
            # Reward to go
            R = np.zeros(n, )
            for i in reversed(range(n)):
                R[i] = rewards[i] + self.gamma * (R[i + 1] if i + 1 < n else 0)
            self.R_buff[finish_envs[j], path_slice] = R[:-1]

        # Reset the path start index
        self.path_start_index[finish_envs] = self.pointer[finish_envs]

    def get(self):
        # assert all(self.pointer == self.max_size * np.ones_like(self.pointer, dtype=int))
        self.pointer, self.path_start_index = np.zeros(self.n_env, dtype=int), np.zeros(self.n_env, dtype=int)
        self.normalize_A()
        batch = dict(o=self.o_buff.reshape((-1, self.o_dim)), a=self.a_buff.reshape((-1, self.a_dim)),
                     R=self.R_buff.reshape((-1)), A=self.A_buff.reshape((-1)), logpi=self.logpi_buff.reshape((-1)),
                     reward=self.reward_buff.reshape((-1)))
        return {k: torch.as_tensor(v) for k, v in batch.items()}


class RNNBuffer(Buffer):
    def __init__(self, o_dim, a_dim, size, gamma, lam, h_size_pi, h_size_v, n_layers_pi, n_layers_v, n_env=1):
        super().__init__(o_dim, a_dim, size, gamma, lam, n_env)
        self.n_layers_pi = n_layers_pi
        self.h_size_pi = h_size_pi
        self.n_layers_v = n_layers_v
        self.h_size_v = h_size_v
        self.h_pi_buff = np.zeros((n_env, size, n_layers_pi, h_size_pi), dtype=np.float32)
        self.c_pi_buff = np.zeros((n_env, size, n_layers_pi, h_size_pi), dtype=np.float32)
        self.h_v_buff = np.zeros((n_env, size, n_layers_v, h_size_v), dtype=np.float32)
        self.c_v_buff = np.zeros((n_env, size, n_layers_v, h_size_v), dtype=np.float32)

    def store(self, o, a, reward, v, logpi, h_pi, c_pi, h_v, c_v):
        assert all(self.pointer) < self.max_size
        assert all(self.pointer[:-1] == self.pointer[1:])
        self.store_mlp_data(o, a, reward, v, logpi)
        self.h_pi_buff[:, self.pointer[0], :, :] = h_pi
        self.c_pi_buff[:, self.pointer[0], :, :] = c_pi
        self.h_v_buff[:, self.pointer[0], :, :] = h_v
        self.c_v_buff[:, self.pointer[0], :, :] = c_v
        self.pointer += np.ones_like(self.pointer, dtype=int)

    def get(self):
        # assert all(self.pointer == self.max_size * np.ones_like(self.pointer, dtype=int))
        self.pointer, self.path_start_index = np.zeros(self.n_env, dtype=int), np.zeros(self.n_env, dtype=int)
        self.normalize_A()
        batch = dict(o=self.o_buff.reshape((-1, self.o_dim)), a=self.a_buff.reshape((-1, self.a_dim)),
                     R=self.R_buff.reshape((-1)), A=self.A_buff.reshape((-1)), logpi=self.logpi_buff.reshape((-1)),
                     reward=self.reward_buff.reshape((-1)),
                     h_pi=self.h_pi_buff.reshape((-1, self.n_layers_pi, self.h_size_pi)),
                     c_pi=self.c_pi_buff.reshape((-1, self.n_layers_pi, self.h_size_pi)),
                     h_v=self.h_v_buff.reshape((-1, self.n_layers_v, self.h_size_v)),
                     c_v=self.c_v_buff.reshape((-1, self.n_layers_v, self.h_size_v)))
        return {k: torch.as_tensor(v) for k, v in batch.items()}
