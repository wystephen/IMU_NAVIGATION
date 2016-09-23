# -*- coding:utf-8 -*-
# Create by steve in 16-9-21 at 上午9:22

from Setting import settings
#
import numpy as np

import math

class ZUPTaidedIns:
    def __init__(self, settings):
        self.para = settings

        self.init_filter()

        self.init_vec(self.P)

        self.x_h = np.zeros([18, 1])

    def init_Nav_eq(self, u1, u2):

        f_u = np.mean(u1[0, :])
        f_v = np.mean(u1[1, :])
        f_w = np.mean(u1[2, :])

        # print(f_u,f_v,f_w)
        roll = math.atan2(-f_v, -f_w)  # ToDo:May be wrroy
        pitch = math.atan2(f_u, math.sqrt(f_v ** 2 + f_w ** 2))

        attitude = [roll, pitch, self.para.init_heading1]
        attitude = np.transpose(attitude)

        Rb2t = self.Rt2b(attitude)
        Rb2t = np.transpose(Rb2t)

        quat1 = self.dcm2q(Rb2t)

        x = np.zeros([18, 1])
        x[0:3, 0] = self.para.init_pos1
        x[6:9, 0] = attitude

        f_u = np.mean(u2[0, :])
        f_v = np.mean(u2[1, :])
        f_w = np.mean(u2[2, :])

        # print(f_u,f_v,f_w)
        roll = math.atan2(-f_v, -f_w)  # ToDo:May be wrroy
        pitch = math.atan2(f_u, math.sqrt(f_v ** 2 + f_w ** 2))


        attitude = [roll, pitch, self.para.init_heading2]
        attitude = np.transpose(attitude)

        Rb2t = self.Rt2b(attitude)
        Rb2t = np.transpose(Rb2t)

        quat2 = self.dcm2q(Rb2t)

        x[9:12, 0] = self.para.init_pos2
        x[15:18, 0] = attitude

        # print(x,quat1,quat2)
        self.x_h = x
        self.quat1 = quat1
        self.quat2 = quat2

        return x, quat1, quat2



    def init_filter(self):
        self.P = (np.zeros([18, 18]))
        print(self.para.sigma_initial_pos ** 2)

        self.P[0:3, 0:3] = np.diagflat(np.transpose(self.para.sigma_initial_pos ** 2.0))
        self.P[3:6, 3:6] = np.diagflat(np.transpose(self.para.sigma_initial_vel ** 2.0))
        self.P[6:9, 6:9] = np.diagflat(np.transpose(self.para.sigma_initial_att ** 2.0))

        self.P[9:12, 9:12] = np.diagflat(np.transpose(self.para.sigma_initial_pos2 ** 2.0))
        self.P[12:15, 12:15] = np.diagflat(np.transpose(self.para.sigma_initial_vel2 ** 2.0))
        self.P[15:18, 15:18] = np.diagflat(np.transpose(self.para.sigma_initial_att2 ** 2.0))

        # print(self.P)

        self.R1 = np.diagflat(np.transpose(self.para.sigma_vel ** 2.0))
        self.R2 = np.diagflat(np.transpose(self.para.sigma_vel ** 2.0))
        self.R12 = np.zeros([6, 6])
        self.R12[0:3, 0:3] = self.R1
        self.R12[3:6, 3:6] = self.R2

        # print(self.R12)

        self.Q = np.zeros([12, 12])

        self.Q[0:3, 0:3] = np.diagflat(np.transpose(self.para.sigma_acc ** 2.0))
        self.Q[3:6, 3:6] = np.diagflat(np.transpose(self.para.sigma_gyro ** 2.0))

        self.Q[6:9, 6:9] = np.diagflat(np.transpose(self.para.sigma_acc ** 2.0))
        self.Q[9:12, 9:12] = np.diagflat(np.transpose(self.para.sigma_gyro ** 2.0))

        # print (self.Q)

        self.H1 = np.zeros([3, 18])
        self.H1[0:3, 3:6] = np.diagflat(np.transpose([1.0, 1.0, 1.0]))

        self.H2 = np.zeros([3, 18])
        self.H2[0:3, 12:15] = np.diagflat(np.transpose([1.0, 1.0, 1.0]))

        self.H12 = np.zeros([6, 18])
        self.H12[0:3, :] = self.H1
        self.H12[3:6, :] = self.H2

        # print(self.H12)

    def init_vec(self, P):
        self.Id = np.diagflat(np.ones(self.P.shape[0]))
        return


    def Rt2b(self, ang):
        cr = math.cos(ang[0])
        sr = math.sin(ang[0])

        cp = math.cos(ang[1])
        sp = math.sin(ang[1])

        cy = math.cos(ang[2])
        sy = math.sin(ang[2])

        R = np.array(
            [[cy * cp, sy * cp, -sp],
             [-sy * cr + cy * sp * sr, cy * cr + sy * sp * sr, cp * sr],
             [sy * sr + cy * sp * cr, -cy * sr + sy * sp * cr, cp * cr]]
        )
        return R

    def dcm2q(self, R):
        T = 1.0 + R[0, 0] + R[1, 1] + R[2, 2]
        # print (T)


        # Really Big Change.
        if math.fabs(T) > 1e-4:
            S = 0.5 / math.sqrt(math.fabs(T))

            qw = 0.25 / S
            qx = (R[2, 1] - R[1, 2]) * S
            qy = (R[0, 2] - R[2, 0]) * S
            qz = (R[1, 0] - R[0, 1]) * S
        else:
            if (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
                S = math.sqrt(1 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0

                qw = (R[2, 1] - R[1, 2]) / S
                qx = 0.25 * S
                qy = (R[0, 1] + R[1, 0]) / S
                qz = (R[0, 2] + R[2, 0]) / S

            elif (R[1, 1] > R[2, 2]):
                S = math.sqrt(1 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0

                qw = (R[0, 2] - R[2, 0]) / S
                qx = (R[0, 1] + R[1, 0]) / S
                qy = 0.25 * S
                qz = (R[1, 2] + R[2, 1]) / S
            else:
                S = math.sqrt(1 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0

                qw = (R[1, 0] - R[0, 1]) / S
                qx = (R[0, 2] + R[2, 0]) / S
                qy = (R[1, 2] + R[2, 1]) / S
                qz = 0.25 * S

        return np.array(np.transpose([qx, qy, qz, qw]))

    def q2dcm(self, q):
        p = np.zeros([6, 1])

        p[0:4, 0] = q ** 2.0

        p[4] = p[1] + p[2]

        if math.fabs(p[0] + p[3] + p[4]) > 1e-18:
            p[5] = 2.0 / (p[0] + p[3] + p[4])
        else:
            p[5] = 0.0

        R = np.zeros([3, 3])

        R[0, 0] = 1 - p[5] * p[4]
        R[1, 1] = 1 - p[5] * (p[0] + p[2])
        R[2, 2] = 1 - p[5] * (p[0] + p[1])

        p[0] = p[5] * q[0]
        p[1] = p[5] * q[1]
        p[2] = p[5] * q[2] * q[3]
        p[5] = p[0] * q[1]

        R[0, 1] = p[5] - p[4]
        R[1, 0] = p[5] + p[4]

        p[4] = p[1] * q[3]
        p[5] = p[0] * q[2]

        R[0, 2] = p[5] + p[4]
        R[2, 0] = p[5] - p[4]

        p[4] = p[0] * q[3]
        p[5] = p[1] * q[2]

        R[1, 2] = p[5] - p[4]
        R[2, 1] = p[5] + p[4]

        return R

    def GetPosition(self, u1, u2, zupt1, zupt2):
        self.x_h, self.quat1, self.quat2 = self.Navigation_euqtions(self.x_h, u1, u2, self.quat1, self.quat2,
                                                                    self.para.Ts)

        self.F, self.G = self.state_matrix(self.quat1, self.quat2, u1, u2, self.para.Ts)

        self.P = 1.01 ** 2 * (self.F.dot(self.P)).dot(np.transpose(self.P)) + \
                 (self.G.dot(self.Q)).dot(np.transpose(self.G))

        # self.P = self.para.s_P
        # self.P = self.P * 50.0

        if zupt1 == 1 or zupt2 == 1:
            #print (11)
            if (zupt1 == 1) and (not (zupt2 == 1)):
                #print(1)
                H = self.H1
                R = self.R1
                z = -self.x_h[3:6]
            elif (not (zupt1 == 1)) and (zupt2 == 1):
                #print(2)
                H = self.H2
                R = self.R2
                z = -self.x_h[12:15]
            else:
                #print(3)
                H = self.H12
                R = self.R12
                z = np.zeros([6, 1])
                z[0:3] = -self.x_h[3:6]
                z[3:6] = -self.x_h[12:15]

            # print (R.shape)

            self.K = self.P.dot(np.transpose(H)). \
                dot(
                np.linalg.inv(
                    (H.dot(self.P).dot(np.transpose(H)) + R))
            )
            dx = self.K.dot(z)

            self.P = (self.Id - self.K.dot(H)).dot(self.P)

            self.x_h, self.quat1, self.quat2 = self.comp_internal_states(self.x_h,
                                                                         dx,
                                                                         self.quat1,
                                                                         self.quat2)

        #self.P = (self.P + np.transpose(self.P)) * 0.5

        # print(self.x_h,self.quat1,self.quat2)
        return self.x_h

    def Navigation_euqtions(self, x_h, u1, u2, quat1, quat2, dt):
        y = np.zeros([18, 1])

        w_tb = u1[3:6]
        v = np.linalg.norm(w_tb) * dt

        if math.fabs(v) > 1e-10:
            P = w_tb[0] * dt * 0.5
            Q = w_tb[1] * dt * 0.5
            R = w_tb[2] * dt * 0.5

            OMEGA = np.array([
                [0.0, R, -Q, P],
                [-R, 0.0, P, Q],
                [Q, -P, 0.0, R],
                [-P, -Q, -R, 0.0]
            ])

            q = (math.cos(v / 2.0) * np.diagflat([1.0, 1.0, 1.0, 1.0]) +
                 2.0 / v * math.sin(v / 2.0) * OMEGA).dot(quat1)
            q = q / np.linalg.norm(q)

        ####################

        w_tb = u2[3:6]
        v = np.linalg.norm(w_tb) * dt

        if math.fabs(v) > 1e-10:
            P = w_tb[0] * dt * 0.5
            Q = w_tb[1] * dt * 0.5
            R = w_tb[2] * dt * 0.5

            OMEGA = np.array([
                [0.0, R, -Q, P],
                [-R, 0.0, P, Q],
                [Q, -P, 0.0, R],
                [-P, -Q, -R, 0.0]
            ])

            q2 = (math.cos(v / 2.0) * np.diagflat([1.0, 1.0, 1.0, 1.0]) +
                  2.0 / v * math.sin(v / 2.0) * OMEGA).dot(quat2)
            q2 = q2 / np.linalg.norm(q2)

        g_t = np.array([0, 0, 9.8173])
        g_t = np.transpose(g_t)

        #use rotation transform form imu to word
        Rb2t = self.q2dcm(q)
        f_t = Rb2t.dot(u1[0:3])

        Rb2t = self.q2dcm(q2)
        f_t2 = Rb2t.dot(u2[0:3])

        acc_t = f_t + g_t
        acc_t2 = f_t2 + g_t

        A = np.diagflat(np.transpose([1.0, 1.0, 1.0, 1.0, 1.0, 1.0]))
        A[0, 3] = dt
        A[1, 4] = dt
        A[2, 5] = dt

        B = np.zeros([6, 3])

        B[0:3, 0:3] = np.zeros([3, 3])
        B[3:6, 0:3] = np.diagflat([dt, dt, dt])

        # print(acc_t.shape)
        # print(B.dot(acc_t).shape)
        # print(A.dot(x_h[0:6]).shape)
        acc_t = acc_t.reshape(3, 1)
        acc_t2 = acc_t2.reshape(3, 1)

        #accumulate acc and pose.
        y[0:6] = A.dot(x_h[0:6]) + B.dot(acc_t)
        y[9:15] = A.dot(x_h[9:15]) + B.dot(acc_t2)

        return y, q, q2

    def state_matrix(self, q, q2, u, u2, dt):

        Rb2t = self.q2dcm(q)
        Rb2t2 = self.q2dcm(q2)

        f_t = Rb2t.dot(u[0:3])
        f_t2 = Rb2t.dot(u2[0:3])

        St = np.array([
            [0, -f_t[0], f_t[1]],
            [f_t[2], 0.0, -f_t[0]],
            [-f_t[1], f_t[0], 0.0]
        ])

        St2 = np.array([
            [0, -f_t2[2], f_t2[1]],
            [f_t2[2], 0.0, -f_t2[0]],
            [-f_t2[1], f_t2[0], 0.0]
        ])

        Fc = np.zeros([18, 18])

        Fc[0:3, 3:6] = np.diagflat([1.0, 1.0, 1.0])
        Fc[3:6, 6:9] = St

        Fc[9:12, 12:15] = np.diagflat([1.0, 1.0, 1.0])
        Fc[12:15, 15:18] = St2

        Gc = np.zeros([18, 12])

        Gc[3:6, 0:3] = Rb2t
        Gc[6:9, 3:6] = -Rb2t

        Gc[12:15, 6:9] = Rb2t2
        Gc[15:18, 9:12] = -Rb2t2

        F = np.diagflat(np.ones([1, 18])) + dt * Fc
        G = dt * Gc

        return F, G

    def comp_internal_states(self, x_in, dx, q_in, q_in2):

        R = self.q2dcm(q_in)
        R2 = self.q2dcm(q_in2)

        x_out = x_in + dx

        epsilon = dx[6:9]
        #print (dx)

        OMEGA = np.array([
            [0, -epsilon[2], epsilon[1]],
            [epsilon[2], 0.0, -epsilon[0]],
            [-epsilon[1], epsilon[0], 0.0]
        ])

        R = (np.diagflat([1.0, 1.0, 1.0]) - OMEGA).dot(R)
        #print(R)

        epsilon = dx[15:18]
        OMEGA = np.array([
            [0, -epsilon[2], epsilon[1]],
            [epsilon[2], 0.0, -epsilon[0]],
            [-epsilon[1], epsilon[0], 0.0]
        ])

        R2 = (np.diagflat([1.0, 1.0, 1.0]) - OMEGA).dot(R2)
        #print(R2)

        q_out = self.dcm2q(R)
        q_out2 = self.dcm2q(R2)

        return x_out, q_out,q_out2
