# -*- coding:utf-8 -*-
# Create by steve in 16-9-21 at 上午9:22

from Setting import settings
#
import numpy as np

class ZUPTaidedIns:
    def __init__(self, settings):
        self.para = settings

        self.init_filter()

    def init_filter(self):
        self.P = (np.zeros([18, 18]))
        print(self.para.sigma_initial_pos ** 2)

        self.P[0:3, 0:3] = np.diag(np.transpose(self.para.sigma_initial_pos ** 2))

        print(self.P)
