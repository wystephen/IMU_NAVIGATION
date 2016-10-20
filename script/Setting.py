# -*- coding:utf-8 -*-
# Create by steve in 16-9-21 at 上午9:46

import numpy as np


class settings:
    def __init__(self):
        self.dim_constraint = 2.0
        self.alpha_level = 0.95
        self.range_constraint = 1.0  # [m]
        self.min_rud_sep = 820

        self.init_heading1 = (-95.0 - 2.5) * np.pi / 180.0
        self.init_heading2 = (94.05 + 2.0) * np.pi / 180.0

        self.init_pos1 = np.zeros([1, 3])
        self.init_pos2 = np.zeros([1, 3])

        self.range_constraint_on = False

        self.altitude = 100
        self.latitude = 50

        self.Ts = 1.0 / 820.0

        self.init_heading = 0.0
        self.init_pos = np.zeros([3, 1])

        self.sigma_a = 0.035

        self.sigma_g = 0.35 * np.pi / 180.0

        self.gamma = 200.0

        self.sigma_acc = 4.0 * 0.7 * np.ones([3, 1])

        self.sigma_gyro = 4.0 * 10.0 * np.ones([3, 1]) * 0.1 * np.pi / 180.0

        self.sigma_vel = 5.0 * np.ones([1, 3]) * 0.01

        self.sigma_initial_pos = 1e-2 * 0.1 * np.ones([3, 1])
        self.sigma_initial_vel = 1e-5 * np.ones([3, 1])
        self.sigma_initial_att = (np.pi / 180.0 * np.array([0.1, 0.1, 0.001]).reshape(3, 1))

        self.sigma_initial_pos2 = 1e-2 * 0.1 * np.ones([3, 1])
        self.sigma_initial_vel2 = 1e-5 * np.ones([3, 1])
        self.sigma_initial_att2 = (np.pi / 180.0 * np.array([0.1, 0.1, 0.001]).reshape(3, 1))

        # self.s_P = np.loadtxt("../Data/P.csv", dtype=float, delimiter=",")
        self.gravity = 9.8

        self.time_Window_size = 10
