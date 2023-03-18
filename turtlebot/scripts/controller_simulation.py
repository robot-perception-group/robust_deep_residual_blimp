import numpy as np


class ControllerSimulation:
    def __init__(self, a, b, c, d):
        self.A = a
        self.B = b
        self.C = c
        self.D = d
        self.dim_x = a.shape[-1]
        self.dim_y = c.shape[0]
        self.dim_u = b.shape[-1]
        self.x = np.zeros((self.dim_x, 1))
        self.y = np.zeros((self.dim_y, 1))

    def sim(self, u, y_max=np.Inf, y_min=-np.Inf):
        if not np.isscalar(u):
            self.y = self.C @ self.x + self.D @ u
            self.x = self.A @ self.x + self.B @ u
        else:
            self.y = self.C @ self.x + self.D * u
            self.x = self.A @ self.x + self.B * u
        if len(self.y) == 1:
            self.y = self.y[0][0]
            if self.y > y_max:
                self.y = y_max
            elif self.y < y_min:
                self.y = y_min
        return self.y

    def controller_reset(self):
        self.x = np.zeros((self.dim_x, 1))
