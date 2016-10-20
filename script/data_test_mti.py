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
    setting.Ts = 0.0025

    setting.time_Window_size = 5

    zupt = zupte_test(setting)
    ZUPT1 = zupt.GLRT_Detector(all_data)

    print(all_data.shape, ZUPT1.shape)

    ins_filter = ZUPTaidedIns(settings=setting)

    ins_filter.init_Nav_eq(all_data[1:20, :], all_data[1:20, :])

    u1 = all_data
    u2 = all_data

    zupt1 = ZUPT1
    zupt2 = ZUPT1

    all_x = np.zeros([18, u1.shape[0]])

    # ax = plt.subplot(111, projection='3d')

    for index in range(u1.shape[0]):
        if (index % 100 == 0):
            print(float(index) / u1.shape[0])
        all_x[:, index] = ins_filter.GetPosition(u1[index, :],
                                                 u2[index, :],
                                                 zupt1[index],
                                                 zupt2[index]).reshape([18])

    np.savetxt("all_x", all_x)
    # Print RESULT

    # print(all_x[:, u1.shape[0] - 1])
    print(np.linalg.norm(all_x[0:3, u1.shape[0] - 1]))
    print(np.linalg.norm(all_x[9:12, u1.shape[0] - 1]))

    # SHOW RESULT
    plt.figure(1)
    plt.grid()

    plt.plot(all_x[0, :], all_x[1, :], all_x[2, :], 'r')
    plt.plot(all_x[9, :], all_x[10, :], all_x[11, :], 'b')

    plt.figure(12)

    plt.plot(all_x[3, :], 'r')
    plt.plot(all_x[4, :], 'g')
    plt.plot(all_x[5, :], 'b')

    plt.figure(13)
    plt.plot(u1[:, 0], 'r')
    plt.plot(u1[:, 1], 'g')
    plt.plot(u1[:, 2], 'b')

    plt.plot(zupt1[:] * 10.0, 'y')

    plt.figure(14)

    plt.plot(all_x[6, :], 'r')
    plt.plot(all_x[7, :], 'g')
    plt.plot(all_x[8, :], 'b')

    plt.figure(15)

    plt.plot(u1[:, 3], 'r')
    plt.plot(u1[:, 4], 'g')
    plt.plot(u1[:, 5], 'b')

    plt.figure(22)
    plt.plot(all_x[12, :], 'r')
    plt.plot(all_x[13, :], 'g')
    plt.plot(all_x[14, :], 'b')

    plt.figure(23)
    plt.plot(u2[:, 0], 'r')
    plt.plot(u2[:, 1], 'g')
    plt.plot(u2[:, 2], 'b')

    plt.plot(zupt2[:] * 10.0, 'y')

    plt.figure(24)

    plt.plot(all_x[15, :], 'r')
    plt.plot(all_x[16, :], 'g')
    plt.plot(all_x[17, :], 'b')

    plt.figure(25)

    plt.plot(u2[:, 3], 'r')
    plt.plot(u2[:, 4], 'g')
    plt.plot(u2[:, 5], 'b')

    plt.show()
