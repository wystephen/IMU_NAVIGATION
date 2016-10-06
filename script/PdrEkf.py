# -*- coding:utf-8 -*-
# Create by steve in 16-9-21 at 上午9:22

from Setting import settings
#
import numpy as np

import math


class ZUPTaidedIns(object):
    """

    """

    def __init__(self, settings):
        '''

        :param settings:
        '''
        self.para = settings
        self.R12 = np.zeros([6, 6])

        self.P = (np.zeros([18, 18]))
        self.Q = np.zeros([12, 12])
        self.H12 = np.zeros([6, 18])
        self.H2 = np.zeros([3, 18])
        self.H1 = np.zeros([3, 18])

        self.init_filter()

        self.init_vec(self.P)

        self.x_h = np.zeros([18, 1])

        self.last_constraint = 0
        self.last_zv = 0

    def init_Nav_eq(self, u1, u2):
        '''

        :param u1:
        :param u2:
        :return:
        '''

        f_u = np.mean(u1[0, :])
        f_v = np.mean(u1[1, :])
        f_w = np.mean(u1[2, :])

        # print(f_u,f_v,f_w)
        roll = math.atan2(-f_v, -f_w)
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
        roll = math.atan2(-f_v, -f_w)
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
        '''
        initial filiter parameter.

        :return:
        '''

        print(self.para.sigma_initial_pos ** 2)

        self.P[0:3, 0:3] = np.diagflat(np.transpose(self.para.sigma_initial_pos ** 2.0))
        self.P[3:6, 3:6] = np.diagflat(np.transpose(self.para.sigma_initial_vel ** 2.0))
        self.P[6:9, 6:9] = np.diagflat(np.transpose(self.para.sigma_initial_att ** 2.0))

        self.P[9:12, 9:12] = np.diagflat(np.transpose(self.para.sigma_initial_pos2 ** 2.0))
        self.P[12:15, 12:15] = np.diagflat(np.transpose(self.para.sigma_initial_vel2 ** 2.0))
        self.P[15:18, 15:18] = np.diagflat(np.transpose(self.para.sigma_initial_att2 ** 2.0))

        # print(self.P)

        self.R2 = np.diagflat(np.transpose(self.para.sigma_vel ** 2.0))
        self.R1 = np.diagflat(np.transpose(self.para.sigma_vel ** 2.0))

        self.R12[0:3, 0:3] = self.R1
        self.R12[3:6, 3:6] = self.R2

        # print(self.R12)


        self.Q[0:3, 0:3] = np.diagflat(np.transpose(self.para.sigma_acc ** 2.0))
        self.Q[3:6, 3:6] = np.diagflat(np.transpose(self.para.sigma_gyro ** 2.0))

        self.Q[6:9, 6:9] = np.diagflat(np.transpose(self.para.sigma_acc ** 2.0))
        self.Q[9:12, 9:12] = np.diagflat(np.transpose(self.para.sigma_gyro ** 2.0))

        # print (self.Q)

        self.H1[0:3, 3:6] = np.diagflat(np.transpose([1.0, 1.0, 1.0]))

        self.H2[0:3, 12:15] = np.diagflat(np.transpose([1.0, 1.0, 1.0]))

        self.H12[0:3, :] = self.H1
        self.H12[3:6, :] = self.H2

        # print(self.H12)

    def init_vec(self, P):
        """

        :param P:
        :return:
        """
        self.Id = np.diagflat(np.ones(self.P.shape[0]))
        return

    def Rt2b(self, ang):
        '''
        :
        :param ang:
        :return:
        '''
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
        """
        Transform from rotation matrix to quanternions.
        :param R:old rotation matrix
        :return:quanternion
        """
        T = 1.0 + R[0, 0] + R[1, 1] + R[2, 2]
        # print (T)


        # Really Big Change.
        # ToDo:Why there are some value is smallter than zero.
        if math.fabs(T) > 1e-3:
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

            elif R[1, 1] > R[2, 2]:
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

        quart = np.array(np.transpose([qx, qy, qz, qw]))

        quart /= np.linalg.norm(quart)

        return quart

    def q2dcm(self, q):
        """

        :param q:
        :return:
        """
        p = np.zeros([6, 1])

        p[0:4] = q.reshape(4, 1) ** 2.0

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
        p[4] = p[5] * q[2] * q[3]
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

        """

        :param u1:
        :param u2:
        :param zupt1:
        :param zupt2:
        :return:
        """
        self.x_h, self.quat1, self.quat2 = self.Navigation_euqtions(self.x_h,
                                                                    u1, u2,
                                                                    self.quat1, self.quat2,
                                                                    self.para.Ts)

        self.F, self.G = self.state_matrix(self.quat1, self.quat2, u1, u2, self.para.Ts)

        # FIX HERE?
        self.P = (self.F.dot(self.P)).dot(np.transpose(self.F)) + \
                 (self.G.dot(self.Q)).dot(np.transpose(self.G))

        # self.P = self.para.s_P
        # self.P = self.P * 50.0

        # zupt1 = 0
        # zupt2 = 0
        # self.last_zv += 1
        # and (self.last_zv > 2)
        if (zupt1 == 1 or zupt2 == 1):
            self.last_zv = 0
            # print (11)
            # self.P = self.P * 30.0
            if (zupt1 == 1) and (not (zupt2 == 1)):
                # print(1)
                H = self.H1
                R = self.R1
                z = -self.x_h[3:6]
            elif (not (zupt1 == 1)) and (zupt2 == 1):
                # print(2)
                H = self.H2
                R = self.R2
                z = -self.x_h[12:15]
            else:
                # print(3)
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

            dx = self.K.dot(z)  # differante to the formula in paper

            self.P = (self.Id - self.K.dot(H)).dot(self.P)

            self.x_h, self.quat1, self.quat2 = self.comp_internal_states(self.x_h,
                                                                         dx,
                                                                         self.quat1,
                                                                         self.quat2)

        # '''
        # RANGE CONSTRAINT.
        #
        # '''
        self.last_constraint += 1

        if self.para.range_constraint_on and \
                        self.last_constraint > self.para.min_rud_sep and \
                        np.linalg.norm(
                                    self.x_h[0:3] - self.x_h[9:12]) > self.para.range_constraint:
            self.last_constraint = 0

            tmp_in1 = np.zeros([10, 1])
            tmp_in2 = np.zeros([10, 1])

            tmp_in1[0:6] = self.x_h[0:6]
            tmp_in1[6:10] = self.quat1.reshape(4, 1)

            tmp_in2[0:6] = self.x_h[9:15]
            tmp_in2[6:10] = self.quat2.reshape(4, 1)

            tmp1, tmp2, self.P = self.range_constraint(tmp_in1, tmp_in2, self.P)

            self.x_h[0:6] = tmp1[0:6]
            self.quat1 = tmp1[6:10]

            self.x_h[9:15] = tmp2[0:6]
            self.quat2 = tmp2[6:10]

        # '''
        # END RANGE CONSTRAINT
        # '''
        self.P = (self.P * 0.5 + np.transpose(self.P) * 0.5)
        # print(self.x_h,self.quat1,self.quat2)
        return self.x_h

    def Navigation_euqtions(self, x_h, u1, u2, quat1, quat2, dt):
        '''

        :type x_h: np.array
        :param x_h:
        :param u1:
        :param u2:
        :param quat1:
        :param quat2:
        :param dt:
        :return:
        '''
        y = np.zeros([18, 1])
        # y = x_h

        w_tb = u1[3:6]
        v = np.linalg.norm(w_tb) * dt

        if math.fabs(v) > 1e-8:
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
        else:
            q = quat1

        ####################

        w_tb = u2[3:6]
        v = np.linalg.norm(w_tb) * dt

        if math.fabs(v) > 1e-8:
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
        else:
            q2 = quat2

        g_t = np.array([0, 0, 9.8173])
        g_t = np.transpose(g_t)

        # use rotation transform form imu to word
        Rb2t = self.q2dcm(q)
        f_t = Rb2t.dot(u1[0:3])

        Rb2t = self.q2dcm(q2)
        f_t2 = Rb2t.dot(u2[0:3])

        acc_t = f_t + g_t
        acc_t2 = f_t2 + g_t

        A = np.diagflat(([1.0, 1.0, 1.0, 1.0, 1.0, 1.0]))
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

        # accumulate acc and pose.
        y[0:6] = A.dot(x_h[0:6]) + B.dot(acc_t)
        y[9:15] = A.dot(x_h[9:15]) + B.dot(acc_t2)

        return y, q, q2

    def state_matrix(self, q, q2, u, u2, dt):
        '''

        :param q:
        :param q2:
        :param u:
        :param u2:
        :param dt:
        :return:
        '''

        Rb2t = self.q2dcm(q)
        Rb2t2 = self.q2dcm(q2)

        f_t = Rb2t.dot(u[0:3])
        f_t2 = Rb2t2.dot(u2[0:3])

        # Fixed here.
        St = np.array([
            [0, -f_t[2], f_t[1]],
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
        '''

        :param x_in:
        :param dx:
        :param q_in:
        :param q_in2:
        :return:
        '''

        R = self.q2dcm(q_in)
        R2 = self.q2dcm(q_in2)

        x_out = x_in + dx

        epsilon = dx[6:9]
        # print (dx)

        OMEGA = np.array([
            [0, -epsilon[2], epsilon[1]],
            [epsilon[2], 0.0, -epsilon[0]],
            [-epsilon[1], epsilon[0], 0.0]
        ])

        R = (np.diagflat([1.0, 1.0, 1.0]) - OMEGA).dot(R)

        # R = R.dot(OMEGA)
        # print(R)

        epsilon = dx[15:18]
        OMEGA = np.array([
            [0, -epsilon[2], epsilon[1]],
            [epsilon[2], 0.0, -epsilon[0]],
            [-epsilon[1], epsilon[0], 0.0]
        ])

        R2 = (np.diagflat([1.0, 1.0, 1.0]) - OMEGA).dot(R2)
        # R2 = R2.dot(OMEGA)
        # print(R2)

        q_out = self.dcm2q(R)
        q_out2 = self.dcm2q(R2)

        return x_out, q_out, q_out2

    def range_constraint(self, x_in1, x_in2, Pin):
        '''

        :param x_in1:
        :param x_in2:
        :param Pin:
        :return:
        '''
        x_h = np.zeros([18, 1])

        x_h[0:6] = x_in1[0:6]
        x_h[9:15] = x_in2[0:6]

        W = np.linalg.inv(Pin)
        W = (W + np.transpose(W)) * 0.5

        lam, z = self.projection(x_h, W)

        x_out1 = np.zeros([18, 1])
        x_out2 = np.zeros([18, 1])

        x_out1[0:6] = z[0:6]
        x_out1[6:10] = self.correct_orientations(x_in1[6:10], z[6:9]).reshape(4, 1)

        x_out2[0:6] = z[9:15]
        x_out2[6:10] = self.correct_orientations(x_in2[6:10], z[15:18]).reshape(4, 1)

        L = np.zeros([3, 18])

        L[0:3, 0:3] = np.diagflat([1.0, 1.0, 1.0])
        L[0:3, 6:9] = np.diagflat([-1.0, -1.0, -1.0])

        z = (np.transpose(L).dot(L)).dot(z)

        A = np.linalg.inv(W + lam * np.transpose(L).dot(L))

        alpha = np.transpose(z).dot(A).dot(z)

        Jp = (((np.diagflat(np.ones([18, 1])) - 1) / alpha).dot(A).dot(z.dot(np.transpose(z)))).dot(A).dot(W)

        Pout = Jp.dot(Pin).dot(np.transpose(Jp))

        return x_out1, x_out2, Pout

    def projection(self, x_h, Pinv):
        '''

        :param x_h:
        :param Pinv:
        :return:
        '''

        L = np.zeros([3, 18])

        L[0:3, 0:3] = np.diagflat([1.0, 1.0, 1.0])
        L[0:3, 9:12] = np.diagflat([-1.0, -1.0, -1.0])

        eta = self.para.range_constraint

        G = np.linalg.cholesky(Pinv)

        U, S, V = np.linalg.svd(L.dot(np.linalg.inv(G)))

        e = np.transpose(V).dot(G).dot(x_h)

        lam = 0.0
        delta = 100000.0
        ctr = 0

        while (math.fabs(delta) > 1e-4 and ctr < 25):
            # g = e[0] ** 2 * S[0, 0] ** 2 / ((1 + lam * S[0, 0] ** 2) ** 2) + \
            #     e[1] ** 2.0 * S[1, 1] ** 2.0 / ((1 + lam * S[1, 1] ** 2) ** 2) + \
            #     e[2] ** 2.0 * S[2, 2] ** 2.0 / ((1 + lam * S[2, 2] ** 2.0) * 2.0) - eta ** 2.0


            g = e[0] * e[0] * S[0] * S[0] / ((1 + lam * S[0] * S[0]) ** 2) + \
                e[1] * e[1] * S[1] * S[1] / ((1 + lam * S[1] * S[1]) ** 2) + \
                e[2] * e[2] * S[2] * S[2] / ((1 + lam * S[2] * S[2]) ** 2) - \
                eta * eta

            # dg = -2.0 * (e[0] ** 2.0 * S[0, 0] ** 4.0 / ((1 + lam * S[0, 0] ** 2.0) ** 3.0) +
            #              e[1] ** 2 * S[1, 1] ** 4.0 / ((1 + lam * S[1, 1] ** 2.0) ** 3.0) +
            #              e[2] ** 2.0 * S[2, 2] ** 4.0 / ((1 + lam * S[2, 2] ** 2.0) ** 3.0)
            #              )


            dg = -2.0 * (e[0] * e[0] * S[0] ** 4.0 / ((1 + lam * S[0] * S[0]) ** 3.0) +
                         e[1] * e[1] * S[1] ** 4.0 / ((1 + lam * S[1] * S[1]) ** 3.0) +
                         e[2] * e[2] * S[2] ** 4.0 / ((1 + lam * S[2] * S[2]) ** 3.0)
                         )

            delta = g / dg

            lam = lam - delta

            ctr = ctr + 1

        if (lam < 0):
            print("ERROR : lam must bigger than zero.")
            z = x_h
        else:
            z = np.linalg.inv(Pinv + (np.transpose(lam * L).dot(L))).dot(Pinv.dot(x_h))

        return lam, z

    def correct_orientations(self, q_in, epsilon):

        R = self.q2dcm(q_in)

        OMEGA = np.array([
            [0, -epsilon[2], epsilon[1]],
            [epsilon[2], 0.0, -epsilon[0]],
            [-epsilon[1], epsilon[0], 0.0]
        ])

        R = (np.diagflat([1.0, 1.0, 1.0]) - OMEGA).dot(R)

        return self.dcm2q(R)
