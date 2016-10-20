# -*- coding:utf-8 -*-
# Create by steve in 16-10-20 at 上午10:50

from PdrEkf import ZUPTaidedIns

from Setting import settings

from zupt_test import zupte_test

import numpy as np

import matplotlib.pyplot as plt

if __name__ == '__main__':
    all_data = np.loadtxt("../Data/u1.csv", delimiter=",")

    print(np.mean(all_data[:, 0]))

    setting = settings()
    setting.Ts = 0.025

    setting.time_Window_size = 5

    zupt = zupte_test(setting)
    ZUPT1 = zupt.GLRT_Detector(all_data)

    print(all_data.shape, ZUPT1.shape)

    plt.figure(1)
    plt.hold(True)
    plt.plot(ZUPT1 * 16, 'y')
    plt.plot(all_data[:, 0], 'r')
    plt.plot(all_data[:, 1], 'g')
    plt.plot(all_data[:, 2], 'b')

    plt.show()
