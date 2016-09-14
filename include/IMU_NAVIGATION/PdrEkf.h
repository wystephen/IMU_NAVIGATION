#pragma once
//Create by steve in 16-9-14 at 下午4:37
//
// Created by steve on 16-9-14.
//
#include "Eigen/Dense"

#include <cmath>

#ifndef IMU_NAVIGATION_PDREKF_H
#define IMU_NAVIGATION_PDREKF_H

class PdrEkf {
public:

protected:

    /*
     * Convert a quaternioin vector to a directional cosine matrix.
     */
    Eigen::Matrix3d Quaternion2Rotation(Eigen::Vector4d q);

    /*
     * Convert a directional cosine matrix in to a quaternion vector.
     */
    Eigen::Vector4d Rotatioin2Quaternion(Eigen::Matrix3d R);

private:

};


Eigen::Vector4d PdrEkf::Rotatioin2Quaternion(Eigen::Matrix3d R) {
    double T(1 + R(0, 0) + R(1, 1) + R(2, 2));
    double S(0.0);

    double qw(0.0), qx(0.0), qy(0.0), qz(0.0);

    if (T > 1e-8) {
        S = 0.5 / std::sqrt(T);

        qw = 0.25 / S;
        qx = (R(2, 1) - R(1, 2)) * S;
        qy = (R(0, 2) - R(2, 0)) * S;
        qz = (R(1, 0) - R(0, 1)) * S;

    } else {
        if ((R(0, 0) > R(1, 1)) && (R(0, 0) > R(2, 2))) {
            S = std::sqrt(1 + R(0, 0) - R(1, 1) - R(2, 2)) * 2.0;

            qw = (R(2, 1) - R(1, 2)) / S;
            qx = 0.25 * S;
            qy = (R(0, 1) + R(1, 0)) / S;
            qz = (R(0, 2) + R(2, 0)) / S;
        } else if (R(1, 1) > R(2, 2)) {
            S = std::sqrt(1 + R(1, 1) - R(0, 0) - R(2, 2)) * 2;

            qw = (R(0, 2) - R(2, 0)) / S;
            qx = (R(0, 1) + R(1, 0)) / S;
            qy = 0.25 * S;
            qz = (R(1, 2) + R(2, 1)) / S;
        } else {
            S = std::sqrt(1 + R(2, 2) - R(0, 0) - R(1, 1)) * 2.0;

            qw = (R(1, 0) - R(0, 1)) / S;
            qx = (R(0, 2) + R(2, 0)) / S;
            qy = (R(1, 2) + R(2, 1)) / S;
            qz = 0.25 * S;
        }

    }

    Eigen::Vector4d q;
    //q << qx << qy << qz << qw;
    q(0) = qx;
    q(1) = qy;
    q(2) = qz;
    q(3) = qw;

    return q;
}

Eigen::Matrix3d PdrEkf::Quaternion2Rotation(Eigen::Vector4d q) {
    Eigen::VectorXd p(6);

    for (int i(0); i < 3; ++i) {
        p(i) = q(i) * q(i);
    }
    p(4) = p(1) + p(2);

    if ((p(0) + p(3) + p(5)) > 0.000001) {
        p(5) = 2 / (p(0) + p(3) + p(4));
    } else {
        p(5) = 0;
    }

    Eigen::Matrix3d R;

    R(0, 0) = 1 - p(5) * p(4);
    R(1, 1) = 1 - p(5) * (p(0) + p(2));
    R(2, 2) = 1 - p(5) * (p(0) + p(1));

    p(0) = p(5) * q(0);
    p(1) = p(5) * q(1);
    p(4) = p(5) * q(2) * q(3);
    p(5) = p(0) * q(1);

    R(0, 1) = p(5) - p(4);
    R(1, 0) = p(5) + p(4);

    p(4) = p(1) * q(3);
    p(5) = p(0) * q(2);

    R(0, 2) = p(5) + p(4);
    R(2, 0) = p(5) - p(4);

    p(4) = p(0) * q(3);
    p(5) = p(1) * q(2);

    R(1, 2) = p(5) - p(4);
    R(2, 1) = p(5) + p(4);

    return R;
}

#endif //IMU_NAVIGATION_PDREKF_H
