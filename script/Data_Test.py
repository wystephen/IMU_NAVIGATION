# -*- coding:utf-8 -*-
# Create by steve in 16-9-21 at 上午9:21

import numpy as np

import scipy as sp

import ros

import matplotlib.pylab as plt

from Setting import settings

from PdrEkf import ZUPTaidedIns as INS

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


    for index in range(u1.shape[0]):
        ins_filter.GetPosition(u1[index, :], u2[index, :], zupt1[index], zupt2[index])
