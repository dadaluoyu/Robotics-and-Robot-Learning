import math
import numpy as np
from scipy.integrate import solve_ivp


class SimulatedDynamics(object):
    def __init__(self, delta_t=None):
        self.dynamics_type = None
        self.residue_limit_flag = False
        self.delta_t = delta_t
        self.data_type = None

    def initialize_from_config(self, config_data, section_name):
        self.dynamics_type = config_data.get(section_name, 'type')
        self.delta_t = float(config_data.get(section_name, 'delta_t'))

    def get_data_type(self):
        return self.data_type

    def get_delta_t(self):
        return self.delta_t

    def get_state_dim(self):
        raise NotImplementedError

    def get_action_dim(self):
        raise NotImplementedError

    def advance(self, state, actions, delta_t=None):
        raise NotImplementedError

    def delta_state(self, state1, state2):
        return state1 - state2


class NumpySimulatedDynamics(SimulatedDynamics):
    def __init__(self, delta_t=None):
        SimulatedDynamics.__init__(self, delta_t)
        self.delta_t = delta_t
        self.data_type = np.ndarray

    def initialize_from_config(self, config_data, section_name):
        SimulatedDynamics.initialize_from_config(self, config_data, section_name)

    def get_state_dim(self):
        raise NotImplementedError

    def get_action_dim(self):
        raise NotImplementedError

    def advance(self, state, actions, delta_t=None):
        raise NotImplementedError


class InvertedPendulumCartNumpySimulatedDynamics(NumpySimulatedDynamics):

    def __init__(self, delta_t=None, cart_mass=None, pendulum_mass=None, length=None, linearize=False):
        NumpySimulatedDynamics.__init__(self, delta_t)
        # physical constants
        self.g = 9.81
        self.length = length
        self.cart_mass = cart_mass
        self.pendulum_mass = pendulum_mass

        # use a linearized model or solve full dynamics
        self.linearize = linearize

    def initialize_from_config(self, config_data, section_name):
        NumpySimulatedDynamics.initialize_from_config(self, config_data, section_name)
        self.cart_mass = config_data.getfloat(section_name, 'cart_mass')
        self.pendulum_mass = config_data.getfloat(section_name, 'pendulum_mass')
        self.length = config_data.getfloat(section_name, 'length')
        self.linearize = config_data.getboolean(section_name, 'linearize')

    def set_masses(self, cart_mass, pendulum_mass):
        self.cart_mass = cart_mass
        self.pendulum_mass = pendulum_mass

    def get_masses(self):
        return self.cart_mass, self.pendulum_mass

    def advance(self, state, action, delta_t=None):
        if delta_t is None: delta_t = self.delta_t
        s_list = [state[0, 0], state[1, 0], state[2, 0], state[3, 0]]
        x = state[0]
        q = state[1]
        xdot = state[2]
        qdot = state[3]
        self.force = action.reshape(-1,)
        if self.linearize:
            d2xdt2, d2qdt2 = self.linearized_dynamics(s_list)
            x += xdot * delta_t + 0.5 * d2xdt2 * delta_t ** 2
            q += qdot * delta_t + 0.5 * d2qdt2 * delta_t ** 2
            xdot += d2xdt2 * delta_t
            qdot += d2qdt2 * delta_t
        else:
            sol = solve_ivp(self.dynamics, [0, delta_t], s_list, t_eval=[delta_t])
            x = sol.y[0, 0]
            q = sol.y[1, 0]
            xdot = sol.y[2, 0]
            qdot = sol.y[3, 0]
        self.force = None
        return np.asarray([x, q, xdot, qdot]).reshape((-1,1))

    def get_state_dim(self):
        return 4

    def get_action_dim(self):
        return 1

    def get_delta_t(self):
        return self.delta_t

    def get_x_from_state(self, state):
        return state[0]

    def get_q_from_state(self, state):
        return state[1]

    def dynamics(self, t, y):

        dydt = [0.0] * 4

        # dx
        dydt[0] = y[2]
        # dq
        dydt[1] = y[3]
        # ddx
        dydt[2] = self.pendulum_mass * self.g * math.sin(y[1]) * math.cos(y[1])
        dydt[2] -= self.pendulum_mass * self.length * y[3] ** 2 * math.sin(y[1]) - self.force
        dydt[2] /= self.cart_mass + self.pendulum_mass * math.sin(y[1]) ** 2
        # ddq
        dydt[3] = -self.pendulum_mass * y[3] ** 2 * math.sin(y[1]) * math.cos(y[1])
        dydt[3] += self.force * math.cos(y[1]) / self.length
        dydt[3] += (self.cart_mass + self.pendulum_mass) * self.g * math.sin(y[1]) / self.length
        dydt[3] /= self.cart_mass + self.pendulum_mass * math.sin(y[1]) ** 2

        return dydt

    def linearized_dynamics(self, x):

        d2xdt2 = self.force
        d2xdt2 -= self.length * self.pendulum_mass * x[3] ** 2 * math.sin(x[1])
        d2xdt2 += self.g * self.pendulum_mass * math.sin(x[1]) * math.cos(x[1])
        d2xdt2 /= self.cart_mass + self.pendulum_mass * math.sin(x[1]) ** 2

        d2qdt2 = self.force * math.cos(x[1])
        d2qdt2 += self.g * (self.cart_mass + self.pendulum_mass) * math.sin(x[1])
        d2qdt2 -= self.length * self.pendulum_mass * x[3] ** 2 * math.sin(x[1]) * math.cos(x[1])
        d2qdt2 /= self.length * (self.cart_mass + self.pendulum_mass * math.sin(x[1]) ** 2)

        return d2xdt2, d2qdt2
