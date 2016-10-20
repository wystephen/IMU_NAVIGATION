# -*- coding:utf-8 -*-
# Create by steve in 16-10-20 at 下午12:12

import numpy as np

import scipy as sp


class zupte_test:
    def __init__(self, setting):
        self.setting = setting

    def GLRT_Detector(self, u):
        g = self.setting.gravity

        sigma2_a = self.setting.sigma_a
        sigma2_g = self.setting.sigma_g
        sigma2_a = sigma2_a ** 2.0
        sigma2_g = sigma2_g ** 2.0

        W = self.setting.time_Window_size

        N = u.shape[0]
        T = np.zeros([N - W + 1, 1])

        for k in range(N - W + 1):
            ya_m = np.mean(u[k:k + W - 1, 0:3], 0)
            # print(ya_m.shape)

            for l in range(k, k + W - 1):
                tmp = u[l, 0:3] - g * ya_m / np.linalg.norm(ya_m)
                T[k] = T[k] + (np.linalg.norm(u[l, 3:6]) ** 2.0) / sigma2_g + \
                       (np.linalg.norm(tmp) ** 2.0) / sigma2_a
        T = T / W

        zupt = np.zeros([u.shape[0], 1])
        for k in range(T.shape[0]):
            if T[k] < self.setting.gamma:
                zupt[k:k + W] = np.ones([W, 1])

        return zupt
