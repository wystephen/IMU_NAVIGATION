# -*- coding:utf-8 -*-
# Create by steve in 16-9-21 at 上午9:21

import numpy as np

import scipy as sp

import ros

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

    para = settings()
    ins_filter = INS(para)

    ins_filter.init_Nav_eq(u1[1:20, :], u2[1:20, :])

    # print(ins_filter.x_h,ins_filter.quat1,ins_filter.quat2)

    all_x = np.zeros([18, u1.shape[0]])

    ax = plt.subplot(111, projection='3d')

    for index in range(u1.shape[0]):
        all_x[:, index] = ins_filter.GetPosition(u1[index, :], u2[index, :], zupt1[index], zupt2[index]).reshape([18])

    plt.figure(1)

    plt.plot(all_x[0, :], all_x[1, :], all_x[2, :], 'r')
    plt.plot(all_x[9, :], all_x[10, :], all_x[11, :], 'b')

    plt.figure(2)

    plt.plot(all_x[3, :], 'r')
    plt.plot(all_x[4, :], 'g')
    plt.plot(all_x[5, :], 'b')

    plt.figure(3)
    plt.plot(all_x[12, :], 'r')
    plt.plot(all_x[13, :], 'g')
    plt.plot(all_x[14, :], 'b')

    plt.show()
