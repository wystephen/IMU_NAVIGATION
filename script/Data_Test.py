# -*- coding:utf-8 -*-
# Create by steve in 16-9-21 at 上午9:21

import numpy as np

import scipy as sp

import ros

import matplotlib.pylab as plt

if __name__ == '__main__':

    # Load data
    u1 = np.loadtxt("../Data/u1.csv", dtype=float, delimiter=",")
    u2 = np.loadtxt("../Data/u2.csv", dtype=float, delimiter=",")

    zupt1 = np.loadtxt("../Data/zupt1.csv", dtype=int, delimiter=",")
    zupt2 = np.loadtxt("../Data/zupt2.csv", dtype=int, delimiter=",")

    for index in range(u1.shape(0)):

    print u1.shape;
