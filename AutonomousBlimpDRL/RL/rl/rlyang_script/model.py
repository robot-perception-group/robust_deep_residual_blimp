import numpy as np
import torch
import torch.nn as nn
import wandb
from torch.distributions import *
from gym.spaces import *


def mlp(sizes, activation=nn.ReLU, output_activation=nn.Identity):
    # Build a feedforward neural network.
    layers = []
    for j in range(len(sizes) - 1):
        act = activation if j < len(sizes) - 2 else output_activation
        layers += [nn.Linear(sizes[j], sizes[j + 1]), act()]
        # Initialization
        nn.init.normal_(layers[j*2].weight, 0, np.sqrt(1 / sizes[j]))
        nn.init.zeros_(layers[j*2].bias)
    return nn.Sequential(*layers)

def unscale_a(scaled_a, low, high):
    return low + (0.5 * (scaled_a + 1.0) * (high - low))

def scale_a(unscaled_a, low, high):
    return 2.0 * ((unscaled_a - low) / (high - low)) - 1.0


class MLPCategorical(nn.Module):
    def __init__(self, o_dim, a_dim, hidden_sizes, activation, squash=False, gain=None):
        super().__init__()
        if not squash:
            self.network = mlp(sizes=[o_dim] + list(hidden_sizes) + [a_dim], activation=activation)
        else:
            self.network = mlp(sizes=[o_dim] + list(hidden_sizes) + [a_dim], activation=activation,
                               output_activation=nn.Tanh)
        # last pi-layer weight initialization trick
        nn.init.xavier_normal_(self.network[-2].weight, gain=(1/100)*gain)

    def distribution(self, o):
        return Categorical(logits=self.network(o))

    def logpi(self, pi, a):
        # Unlike the Gaussian actor, there is only one distribution here. No need so sum.
        return pi.log_prob(a)

    def forward(self, o, a=None):
        # Produce action distributions for given observations, and
        # optionally compute the log likelihood of given actions under
        # those distributions.
        pi = self.distribution(o)
        logpi = None
        if a is not None:
            logpi = self.logpi(pi, a)
        return pi, logpi


class MLPCritic(nn.Module):
    def __init__(self, o_dim, hidden_sizes, activation):
        super().__init__()
        # Output is a scalar "Value"
        self.network = mlp([o_dim] + list(hidden_sizes) + [1], activation)

    def forward(self, o):
        # Squeeze when the last dimension of v-network output is 1. (Convert a (1,1) array to (1,))
        return torch.squeeze(self.network(o), -1)


class MLPGaussian(nn.Module):
    def __init__(self, o_dim, a_dim, hidden_sizes, activation, squash=False, clamp=True,
                 output_log_sigma=False, device=torch.device('cpu'), log=False, gain=None, requires_grad=True,
                 log_sigma=-1):
        super().__init__()
        if not squash:
            self.network = mlp(sizes=[o_dim] + list(hidden_sizes) + [a_dim], activation=activation)
        else:
            self.network = mlp(sizes=[o_dim] + list(hidden_sizes) + [a_dim], activation=activation,
                               output_activation=nn.Tanh)
        # last pi-layer weight initialization trick
        nn.init.xavier_normal_(self.network[-2].weight, gain=(1/100)*gain)
        self.a_dim = a_dim
        self.clamp = clamp
        self.output_log_sigma = output_log_sigma
        self.device = device
        self.log = log
        if not self.output_log_sigma:
            self.log_sigma = nn.Parameter((torch.ones(self.a_dim) * log_sigma).to(self.device),
                                          requires_grad=requires_grad)

    def distribution(self, mu, log_sigma=None):
        # 3-output network
        if self.output_log_sigma:
            # In case of 3-output, log_sigma needs to be given in correct dimension
            log_sigma = torch.clamp(input=log_sigma, min=-1, max=1)
            log_sigma = unscale_a(scaled_a=log_sigma, low=-5, high=-1)
            sigma = log_sigma.exp()
            if self.log:
                try:
                    wandb.log({'Sigma': sigma[-1], 'log(sigma)': log_sigma[-1]})
                except:
                    pass
        # 2-output network
        else:
            # The log_sigma and sigma tensors have the same dimension as the mu tensor
            sigma = self.log_sigma.exp()
            if self.log:
                try:
                    wandb.log({'Sigma': sigma[-1], 'log(sigma)': self.log_sigma[-1]})
                except:
                    pass
        return Normal(mu, sigma)

    def logpi(self, pi, a):
        # The sum of log probabilities through one row (as one output is one row vector)
        return pi.log_prob(a).sum(axis=-1)

    def forward(self, o, a=None):
        # Produce action distributions for given observations, and
        # optionally compute the log likelihood of given actions under
        # those distributions.
        mu = self.network(o)
        # For 3-output network
        if self.output_log_sigma:
            # One batch of input
            log_sigma = mu[:, -1].unsqueeze(dim=1) * torch.ones_like(mu[:, :-1])
            pi = self.distribution(mu[:, :-1], log_sigma)
        # For 2-output network
        else:
            pi = self.distribution(mu)
        logpi = None
        if a is not None:
            logpi = self.logpi(pi, a)
        return pi, logpi


class RNNGaussian(nn.Module):
    def __init__(self, input_dim, output_dim, n_hidden=256, n_layers=2, hidden_sizes=(256, 256), activation=nn.ReLU,
                 squash=False, clamp=True, output_log_sigma=False, device=torch.device('cpu'), log=False,
                 window_size=1, gain=None, requires_grad=True, log_sigma=-1):
        super().__init__()
        self.output_dim = output_dim
        self.clamp = clamp
        self.output_log_sigma = output_log_sigma
        self.device = device
        self.log = log
        self.squash = squash
        self.n_hidden = n_hidden
        self.n_layers = n_layers
        self.h = torch.zeros(n_layers, n_hidden).to(self.device)
        self.c = torch.zeros(n_layers, n_hidden).to(self.device)
        self.window_size = window_size
        if not self.output_log_sigma:
            self.log_sigma = nn.Parameter((torch.ones(self.output_dim) * log_sigma).to(self.device),
                                          requires_grad=requires_grad)
        # Input: (L, N, o_dim)
        # h: (n_layers, N, n_hidden)
        # Output: (L, N, n_hidden)
        self.lstm = nn.LSTM(input_dim, n_hidden, n_layers)
        # Fully connected layer for action/value
        if not squash:
            self.fc = mlp(sizes=[n_hidden] + list(hidden_sizes) + [output_dim], activation=activation)

        else:
            self.fc = mlp(sizes=[n_hidden] + list(hidden_sizes) + [output_dim], activation=activation,
                          output_activation=nn.Tanh)
        # last pi-layer weight initialization trick
        nn.init.xavier_normal_(self.fc[-2].weight, gain=(1/100)*gain)

    def forward(self, x, h, c, a):
        # Batched data!
        x = x.view(1, -1, x.shape[1])
        h = h.view(h.shape[1], -1, h.shape[2]).contiguous()
        c = c.view(c.shape[1], -1, c.shape[2]).contiguous()
        # "window_size" times forward pass of lstm-layer
        for i in range(self.window_size):
            y, (h, c) = self.lstm(x, (h, c))
        # Not useful if input is unbatched (batch_size=1)
        y = y.view(-1, self.n_hidden)
        y = self.fc(y)
        if self.squash:
            y = nn.functional.tanh(y)
        pi = self.distribution(y)
        logpi = None
        if a is not None:
            logpi = self.logpi(pi, a)
        return pi, logpi

    def distribution(self, mu, log_sigma=None):
        # 3-output network
        if self.output_log_sigma:
            # In case of 3-output, log_sigma needs to be given in correct dimension
            log_sigma = unscale_a(scaled_a=log_sigma, low=-3, high=0)
            sigma = log_sigma.exp()
            if self.log:
                wandb.log({'Sigma': sigma[-1], 'log(sigma)': log_sigma[-1]})
        # 2-output network
        else:
            # The log_sigma and sigma tensors have the same dimension as the mu tensor
            sigma = self.log_sigma.exp()
            if self.log:
                wandb.log({'Sigma': sigma[-1], 'log(sigma)': self.log_sigma[-1]})
        return Normal(mu, sigma)

    def logpi(self, pi, a):
        # The sum of log probabilities through one row (as one output is one row vector)
        return pi.log_prob(a).sum(axis=-1)

    def reset_hc(self):
        self.h = torch.zeros_like(self.h).to(self.device)
        self.c = torch.zeros_like(self.c).to(self.device)


class RNNCritic(nn.Module):
    def __init__(self, input_dim, n_hidden=256, n_layers=2, hidden_sizes=(256, 256), activation=nn.ReLU,
                 device=torch.device('cpu'), log=False, window_size=1):
        super().__init__()
        self.device = device
        self.log = log
        self.h = torch.zeros(n_layers, n_hidden).to(self.device)
        self.c = torch.zeros(n_layers, n_hidden).to(self.device)
        self.n_hidden = n_hidden
        self.n_layers = n_layers
        self.window_size = window_size
        # Input: (L, N, o_dim)
        # h: (n_layers, N, n_hidden)
        # Output: (L, N, n_hidden)
        self.lstm = nn.LSTM(input_dim, n_hidden, n_layers)
        # Fully connected layer for action/value
        self.fc = mlp(sizes=[n_hidden] + list(hidden_sizes) + [1], activation=activation)

    def forward(self, x, h, c):
        # Batched data!
        x = x.view(1, -1, x.shape[1])
        h = h.view(h.shape[1], -1, h.shape[2]).contiguous()
        c = c.view(c.shape[1], -1, c.shape[2]).contiguous()
        # "window_size" times forward pass of lstm-layer
        for i in range(self.window_size):
            y, (h, c) = self.lstm(x, (h, c))
        # Not useful if input is unbatched (batch_size=1)
        y = y.view(-1, self.n_hidden)
        y = self.fc(y)
        return y

    def reset_hc(self):
        self.h = torch.zeros_like(self.h).to(self.device)
        self.c = torch.zeros_like(self.c).to(self.device)


class MyModel(nn.Module):
    def __init__(self, o_space, a_space, hidden_sizes_a=(64, 64), hidden_sizes_c=(128, 128), squash=False, clamp=True,
                 output_log_sigma=False, device=torch.device('cpu'), log=False, rnn=False, activation='tanh',
                 window_size=1, requires_grad=True, log_sigma=-1):
        super().__init__()
        self.o_dim = o_space.shape[0]
        self.a_dim = a_space.shape[0]
        self.o_space = o_space
        self.a_space = a_space
        self.squash = squash
        self.clamp = clamp
        self.output_log_sigma = output_log_sigma
        self.device = device
        self.log = log
        self.rnn = rnn
        self.window_size = window_size
        self.requires_grad = requires_grad
        self.log_sigma = log_sigma
        # Calculate the gain for re-initialization for the output layer of pi-network
        gain = nn.init.calculate_gain(activation)
        if self.squash:
            gain = nn.init.calculate_gain('tanh')
        if activation == 'relu':
            activation = nn.ReLU
        elif activation == 'tanh':
            activation = nn.Tanh
        elif activation == 'leaky_relu':
            activation = nn.LeakyReLU
        else:
            raise 'Please define the correct activation function for the MLP network.'

        # d-actor or c-actor
        if isinstance(a_space, Box):
            if self.rnn:
                self.pi = RNNGaussian(self.o_dim, self.a_dim, n_hidden=hidden_sizes_a[0], n_layers=1, hidden_sizes=hidden_sizes_a,
                                      activation=activation, squash=squash, clamp=clamp,
                                      output_log_sigma=output_log_sigma, device=self.device, log=self.log,
                                      window_size=window_size, gain=gain, requires_grad=requires_grad,
                                      log_sigma=log_sigma)
            else:
                self.pi = MLPGaussian(self.o_dim, self.a_dim, hidden_sizes_a, activation, squash, clamp,
                                      output_log_sigma, self.device, self.log, gain=gain, requires_grad=requires_grad,
                                      log_sigma=log_sigma)
        elif isinstance(a_space, Discrete):
            self.pi = MLPCategorical(self.o_dim, a_space.n, hidden_sizes_a, activation, squash, gain=gain)
        # Critic
        if self.rnn:
            self.v = RNNCritic(self.o_dim, n_hidden=hidden_sizes_c[0], n_layers=1, hidden_sizes=hidden_sizes_c,
                               activation=activation, device=self.device, log=self.log,
                               window_size=window_size)
        else:
            self.v = MLPCritic(self.o_dim, hidden_sizes_c, activation)

    def step(self, o):
        if self.rnn:
            with torch.no_grad():
                last_hpi = self.pi.h
                last_cpi = self.pi.c
                last_hv = self.v.h
                last_cv = self.v.c
                # "window_size" times forward pass of lstm-layer
                for i in range(self.window_size):
                    mu, (self.pi.h, self.pi.c) = self.pi.lstm(o, (self.pi.h, self.pi.c))
                mu = self.pi.fc(mu)
                pi = self.pi.distribution(mu)
                a = pi.sample()
                logpi = self.pi.logpi(pi, a)
                for i in range(self.window_size):
                    v, (self.v.h, self.v.c) = self.v.lstm(o, (self.v.h, self.v.c))
                v = self.v.fc(v)
                v = v.squeeze()
                return a.cpu().numpy(), v.cpu().numpy(), logpi.cpu().numpy(), \
                       last_hpi.cpu().numpy(), last_cpi.cpu().numpy(), last_hv.cpu().numpy(), last_cv.cpu().numpy(), pi
        else:
            with torch.no_grad():
                mu = self.pi.network(o)
                # For 3-output network
                if self.output_log_sigma:
                    # One frame of input
                    log_sigma = mu[:, -1] * torch.ones_like(mu[:, -1])
                    pi = self.pi.distribution(mu[:, -1], log_sigma)
                    a = pi.sample()
                    # Append the log_sigma term back to the action array (for the memory buffer)
                    a = torch.cat((a, torch.as_tensor(mu[:, -1]).to(self.device)), dim=0)
                    # Obtain logpi(a)
                    logpi = self.pi.logpi(pi, a[:, -1])
                    v = self.v(o)
                    return a.cpu().numpy(), v.cpu().numpy(), logpi.cpu().numpy(), pi
                # For 2-output network
                else:
                    pi = self.pi.distribution(mu)
                    a = pi.sample()
                    # Obtain logpi(a)
                    logpi = self.pi.logpi(pi, a)
                    v = self.v(o)
                    return a.cpu().numpy(), v.cpu().numpy(), logpi.cpu().numpy(), pi

    def predict(self, o, device=torch.device('cpu')):
        o = torch.as_tensor(o, dtype=torch.float32)
        o = o.unsqueeze(dim=0).to(device)
        if self.rnn:
            with torch.no_grad():
                # "window_size" times forward pass of lstm-layer
                for i in range(self.window_size):
                    a, (self.pi.h, self.pi.c) = self.pi.lstm(o, (self.pi.h, self.pi.c))
                a = self.pi.fc(a)
                if self.clamp or self.squash:
                    a = torch.clamp(input=torch.as_tensor(a), min=-1, max=1)
                    a = unscale_a(a, low=torch.as_tensor(self.a_space.low).to(self.device),
                                  high=torch.as_tensor(self.a_space.high).to(self.device))
                else:
                    a = torch.clamp(input=torch.as_tensor(a),
                                    min=torch.as_tensor(self.a_space.low).to(self.device),
                                    max=torch.as_tensor(self.a_space.high).to(self.device))
        else:
            with torch.no_grad():
                a = self.pi.network(torch.as_tensor(o).to(self.device))
                if self.clamp or self.squash:
                    a = torch.clamp(input=torch.as_tensor(a), min=-1, max=1)
                    a = unscale_a(a, low=torch.as_tensor(self.a_space.low).to(self.device), high=torch.as_tensor(self.a_space.high).to(self.device))
                else:
                    a = torch.clamp(input=torch.as_tensor(a), min=torch.as_tensor(self.a_space.low).to(self.device),
                                    max=torch.as_tensor(self.a_space.high).to(self.device))
        # print("PPO action: ", a)
        return a.cpu().numpy()


