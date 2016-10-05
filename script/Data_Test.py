# -*- coding:utf-8 -*-
# Create by steve in 16-9-21 at 上午9:21

import numpy as np

import scipy as sp

# import ros

import matplotlib.pylab as plt

from Setting import settings

from PdrEkf import ZUPTaidedIns as INS

import mpl_toolkits.mplot3d


if __name__ == '__main__':

    # Load data
    u1 = np.loadtxt("../Data/u1.csv", dtype=float, delimiter=",")
    u2 = np.loadtxt("../Data/u2.csv", dtype=float, delimiter=",")

    zupt1 = np.loadtxt("../Data/zupt1.csv", dtype=int, delimiter=",")
    zupt2 = np.loadtxt("../Data/zupt2.csv", dtype=int, delimiter=",")

    # u1 = np.loadtxt("..\\Data\\u1.csv", dtype=float, delimiter=",")
    # u2 = np.loadtxt("..\\Data\\u2.csv", dtype=float, delimiter=",")
    #
    # zupt1 = np.loadtxt("..\\Data\\zupt1.csv", dtype=int, delimiter=",")
    # zupt2 = np.loadtxt("..\\Data\\zupt2.csv", dtype=int, delimiter=",")

    # u2 = np.loadtxt("../Data/u1.csv", dtype=float, delimiter=",")
    # u1 = np.loadtxt("../Data/u1.csv", dtype=float, delimiter=",")
    #
    # zupt2 = np.loadtxt("../Data/zupt1.csv", dtype=int, delimiter=",")
    # zupt1 = np.loadtxt("../Data/zupt1.csv", dtype=int, delimiter=",")

    para = settings()
    ins_filter = INS(para)

    # the_len = 35000
    # u1 = u1[1:the_len,:]
    # u2 = u2[1:the_len,:]


    # u1[:,3:6] = u1[:,3:6] * np.pi / 180.0
    # u2[:,3:6] = u2[:,3:6] * np.pi / 180.0

    ins_filter.init_Nav_eq(u1[1:20, :], u2[1:20, :])

    # print(ins_filter.x_h,ins_filter.quat1,ins_filter.quat2)

    all_x = np.zeros([18, u1.shape[0]])

    ax = plt.subplot(111, projection='3d')

    for index in range(u1.shape[0]):
        all_x[:, index] = ins_filter.GetPosition(u1[index, :],
                                                 u2[index, :],
                                                 zupt1[index],
                                                 zupt2[index]).reshape([18])

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
