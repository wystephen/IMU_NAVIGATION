#pragma once
//Create by steve in 16-9-14 at 下午4:27
//
// Created by steve on 16-9-14.
//

#include <ros/ros.h>

#include <Eigen/Dense>

#include "PdrEkf.h"

#include "SettingPara.h"

#include <deque>

//This value use control whether the MYCHECK(ISDEBUG) should output the file
//name and the line number.
#define ISDEBUG true

#ifndef IMU_NAVIGATION_TWOFOOTCONTAINT_H
#define IMU_NAVIGATION_TWOFOOTCONTAINT_H


#define ZEROLIZEMATRIX(m) {\
        for(int i(0);i<m.rows();++i)\
        {\
            for(int j(0);j<m.cols();++j)\
            {\
                m(i,j) = 0.0;\
            }\
        }}

class TwoFootEkf : public PdrEkf {
public:

    TwoFootEkf(SettingPara &para) : para_ptr_(&para) {
        //Initial the filter.
        Initial();

    }

    /*
     * Compute a estimation of position
     * (return this value as a Matrix)
     * use the giving Matrix u.
     */
    Eigen::MatrixXd GetPosition(Eigen::MatrixXd u,
                                int detector_signal,
                                double time);


protected:

    SettingPara *para_ptr_;//Use the pointer point to a global value .

    Eigen::Matrix<double, 18, 1> x_h_ = Eigen::VectorXd(18);

    Eigen::VectorXd dx_ = Eigen::VectorXd(18);//

    Eigen::Matrix<double, 18, 18> Id_;

    //Value use for filter.
    Eigen::Matrix<double, 18, 18> P_;

    Eigen::Matrix<double, 12, 12> Q_;

    Eigen::Matrix<double, 3, 18> H1_, H2_;

    Eigen::Matrix<double, 6, 18> H12_;

    Eigen::Matrix<double, 3, 3> R1_, R2_;

    Eigen::Matrix<double, 6, 6> R12_;

    //quat
    Eigen::Vector4d quat1_, quat2_;

    std::deque<Eigen::MatrixXd> u_deque_;
    std::deque<double> time_deque_;
    std::deque<int> status_deque_;


    Eigen::Matrix<double, 12, 1> u_;

    //State Matix
    Eigen::Matrix<double, 18, 18> F_;//State transition matrix
    Eigen::Matrix<double, 18, 12> G_;//Process noise gain matrix.


    bool zupt1_ = false;
    bool zupt2_ = false;

    double last_time_ = 0.0;//
    double the_time_ = 0.0;

    int without_range_constraint_times_ = 0;

    /*
     * Initialization.
     */
    bool Initial();

    /*
     *  Convert detector_signal to two bool value.
     * return false when detector_signal is out of
     * range([0,1,2,3]).
     */

    bool Signal2Bool(int signal);

    /*
     * This function is use for calculates the
     * initial stata of the
     * navigation equations.
     */
    bool InitNavEq();


    /*
     * The mechanized navigation euqtions of the
     * inertial navigation system.
     */
    bool NavigationEq();


    /*
     * Compute States Matrix
     */
    bool StateMatrix();

    /*
     * Function corrects the navigation states use
     * the value compute by kalman filter.
     */
    bool ComputeInternalStates();


    /*
     * Function use for test.
     */
    void test();
private:

};

void TwoFootEkf::test() {
    Eigen::Vector4d quart(0.2, 0.2, 0.2, 1);
    std::cout << Rotation2Quaternion(Quaternion2Rotation(quart));
}

bool TwoFootEkf::ComputeInternalStates() {

    Eigen::Matrix3d R1, R2;

    R1 = Quaternion2Rotation(quat1_);
    R2 = Quaternion2Rotation(quat2_);

    x_h_ = x_h_ + dx_;

    Eigen::Vector3d epsilon;
    Eigen::Matrix3d omega;
//////////////////////////////For R1
    epsilon = dx_.block(6, 0, 3, 1);

    omega(0, 0) = 0.0;
    omega(0, 1) = -epsilon(2);
    omega(0, 2) = epsilon(1);

    omega(1, 0) = epsilon(2);
    omega(1, 1) = 0.0;
    omega(1, 2) = -epsilon(0);

    omega(2, 0) = -epsilon(1);
    omega(2, 1) = epsilon(0);
    omega(2, 2) = 0.0;

    R1 = (Eigen::Matrix3d::Identity() - omega) * R1;
//////////////////////////////For R2
    epsilon = dx_.block(15, 0, 3, 1);

    omega(0, 0) = 0.0;
    omega(0, 1) = -epsilon(2);
    omega(0, 2) = epsilon(1);

    omega(1, 0) = epsilon(2);
    omega(1, 1) = 0.0;
    omega(1, 2) = -epsilon(0);

    omega(2, 0) = -epsilon(1);
    omega(2, 1) = epsilon(0);
    omega(2, 2) = 0.0;

    R2 = (Eigen::Matrix3d::Identity() - omega) * R2;



    /*
     * Get the correct oriention of the imu.
     */
    //ToDo:Test this segment.
//    x_h_(6) = std::atan2(R1(2, 1), R1(2, 2));
//    x_h_(7) = -std::atan(R1(2, 0) / std::sqrt(1 - R1(2, 0) * R1(2, 0)));
//    x_h_(8) = std::atan2(R1(1, 0), R1(0, 0));
//
//    x_h_(15) = std::atan2(R2(2, 1), R2(2, 2));
//    x_h_(16) = -std::atan(R2(2, 0) / std::sqrt(1 - R2(2, 0) * R2(2, 0)));
//    x_h_(17) = std::atan2(R2(1, 0), R2(0, 0));

    quat1_ = Rotation2Quaternion(R1);
    quat2_ = Rotation2Quaternion(R2);


    return true;

}

bool TwoFootEkf::StateMatrix() {

    double dt(the_time_ - last_time_);

    Eigen::MatrixXd u = u_;

    Eigen::Vector3d f_t1(Quaternion2Rotation(quat1_) *
                         u.block(0, 0, 3, 1));
    Eigen::Vector3d f_t2(Quaternion2Rotation(quat2_) *
                         u.block(9, 0, 3, 1));

    Eigen::Matrix3d St1, St2;

    St1(0, 0) = 0.0;
    St2(0, 0) = 0.0;
    St1(0, 1) = -f_t1(2);
    St2(0, 1) = -f_t2(2);
    St1(0, 2) = f_t1(1);
    St2(0, 2) = f_t2(1);

    St1(1, 0) = f_t1(2);
    St2(1, 0) = f_t2(2);
    St1(1, 1) = 0.0;
    St2(1, 1) = 0.0;
    St1(1, 2) = -f_t1(0);
    St2(1, 2) = -f_t2(0);

    St1(2, 0) = -f_t1(1);
    St2(2, 0) = -f_t2(1);
    St1(2, 1) = f_t1(0);
    St2(2, 1) = f_t2(0);
    St1(2, 2) = 0.0;
    St2(2, 2) = 0.0;

    Eigen::Matrix3d Zero(Eigen::Matrix3d::Zero());

    Eigen::Matrix3d Id(Eigen::Matrix3d::Identity());

    ZEROLIZEMATRIX(F_);

    F_.block(0, 3, 3, 3) = Id;
    F_.block(3, 6, 3, 3) = St1;

    F_.block(9, 12, 3, 3) = Id;
    F_.block(12, 15, 3, 3) = St2;

    ZEROLIZEMATRIX(G_)

    G_.block(3, 0, 3, 3) = Quaternion2Rotation(quat1_);
    G_.block(6, 3, 3, 3) = -Quaternion2Rotation(quat1_);

    G_.block(12, 6, 3, 3) = Quaternion2Rotation(quat2_);
    G_.block(15, 9, 3, 3) = -Quaternion2Rotation(quat2_);

    F_ = dt * F_;
    for (int i(0); i < F_.cols(); ++i) {
        F_(i, i) += 1;
    }

    G_ = dt * G_;

    return true;

}

bool TwoFootEkf::NavigationEq() {

    Eigen::VectorXd u1(6), u2(6);

    u1 = u_.block(0, 0, 6, 1);
    u2 = u_.block(6, 0, 6, 1);


    Eigen::VectorXd last_x_h(18);
    last_x_h = x_h_;


    Eigen::Vector3d w_tb;
    double v(0.0);
    double P(0.0), Q(0.0), R(0.0), dt(the_time_ - last_time_);


    /*
     * Update quaternion q.
     */

    //For quat1_
    w_tb = u1.block(3, 0, 3, 1);
    v = w_tb.norm() * dt;

    Eigen::Matrix4d OMEGA;

    if (v > 1e-18) {

        P = w_tb(0) * dt * 0.5;
        Q = w_tb(1) * dt * 0.5;
        R = w_tb(2) * dt * 0.5;


        OMEGA(0, 0) = 0;
        OMEGA(0, 1) = R;
        OMEGA(0, 2) = -Q;
        OMEGA(0, 3) = P;

        OMEGA(1, 0) = -R;
        OMEGA(1, 1) = 0;
        OMEGA(1, 2) = P;
        OMEGA(1, 3) = Q;

        OMEGA(2, 0) = Q;
        OMEGA(2, 1) = -P;
        OMEGA(2, 2) = 0;
        OMEGA(2, 3) = R;

        OMEGA(3, 0) = -P;
        OMEGA(3, 1) = -Q;
        OMEGA(3, 2) = -R;
        OMEGA(3, 3) = 0;

        quat1_ = (std::cos(v / 2) * Eigen::Matrix4d::Identity() +
                  2 / v * std::sin(v / 2) * OMEGA) * quat1_;

        quat1_ = quat1_ / quat1_.norm();

    }

    //For quat2_
    w_tb = u2.block(3, 0, 3, 1);
    v = w_tb.norm() * dt;

    if (v > 1e-18) {
        P = w_tb(0) * dt * 0.5;
        Q = w_tb(1) * dt * 0.5;
        R = w_tb(2) * dt * 0.5;

        OMEGA(0, 0) = 0;
        OMEGA(0, 1) = R;
        OMEGA(0, 2) = -Q;
        OMEGA(0, 3) = P;

        OMEGA(1, 0) = -R;
        OMEGA(1, 1) = 0;
        OMEGA(1, 2) = P;
        OMEGA(1, 3) = Q;

        OMEGA(2, 0) = Q;
        OMEGA(2, 1) = -P;
        OMEGA(2, 2) = 0;
        OMEGA(2, 3) = R;

        OMEGA(3, 0) = -P;
        OMEGA(3, 1) = -Q;
        OMEGA(3, 2) = -R;
        OMEGA(3, 3) = 0;

        quat2_ = (std::cos(v / 2) * Eigen::Matrix4d::Identity() +
                  2 / v * std::sin(v / 2) * OMEGA) * quat2_;

        quat2_ = quat2_ / quat2_.norm();
//        std::cout << "quat2_ :" << quat2_ << std::endl;
    }


    /*
     * Update the position and velocity states using the measured specific force,
     * and the newly calculated attitude.
     */

//    Eigen::Vector3d g_t(0,0,para_ptr_->gravity_);//Gravity vector
    Eigen::Vector3d g_t(0, 0, 9.8173);//Gravity vector
    //ToDo:Change this as a constant.

    Eigen::Vector3d f_t(0, 0, 0), f_t2(0, 0, 0);

    f_t = Quaternion2Rotation(quat1_) * u1.block(0, 0, 3, 1);

    f_t2 = Quaternion2Rotation(quat2_) * u2.block(0, 0, 3, 1);

    Eigen::Vector3d acc_t(f_t + g_t);
    Eigen::Vector3d acc_t2(f_t2 + g_t);

    Eigen::MatrixXd A(6, 6), B(6, 3);

    ZEROLIZEMATRIX(A)

    for (int i(0); i < A.cols(); ++i) {
        A(i, i) = 1.0;
    }
    A(0, 3) = dt;
    A(1, 4) = dt;
    A(2, 5) = dt;

    ZEROLIZEMATRIX(B)

    B.block(0, 0, 3, 3) = Eigen::Matrix3d::Zero();
    B.block(3, 0, 3, 3) = dt * Eigen::Matrix3d::Identity();

    x_h_.block(0, 0, 6, 1) = A * last_x_h.block(0, 0, 6, 1) + B * acc_t;
    x_h_.block(9, 0, 6, 1) = A * last_x_h.block(9, 0, 6, 1) + B * acc_t2;

    return true;
}

bool TwoFootEkf::InitNavEq() {
    /*
     * This function used the assumption that the system is
     * stationary during the first
     * some samples(depends on the parameter
     * SettingPara.navigation_initial_min_length_),
     * all data use the average of these data.
     */
    Eigen::MatrixXd u1(6, u_deque_.size());
    Eigen::MatrixXd u2(6, u_deque_.size());

    Eigen::Vector3d attitude(0, 0, 0);

    int index(0);

    //I have use a c++11 feature here.
    for (auto it = u_deque_.begin(); it != u_deque_.end(); ++it) {
        u1.block(0, index, 6, 1) = it->block(0, 0, 6, 1);
        u2.block(0, index, 6, 1) = it->block(6, 0, 6, 1);
        ++index;
    }

    /////////////////////////////////
    /*
     * For u1.
     */

    double f_u(u1.block(0, 0, 1, u_deque_.size()).mean());
    double f_v(u1.block(1, 0, 1, u_deque_.size()).mean());
    double f_w(u1.block(2, 0, 1, u_deque_.size()).mean());

    double roll(std::atan2(-f_v, -f_w));
    double pitch(std::atan2(f_u, std::sqrt(std::pow(f_v, 2) + std::pow(f_w, 2))));

    attitude = Eigen::Vector3d(roll, pitch, para_ptr_->init_heading1_);

    quat1_ = Rotation2Quaternion(Rotation2b(attitude));


    ZEROLIZEMATRIX(x_h_);

    x_h_.block(0, 0, 3, 1) = para_ptr_->init_pos1_;
    x_h_.block(6, 0, 3, 1) = attitude;

    /*
     * For u2
     */

    f_u = u2.block(0, 0, 1, u_deque_.size()).mean();
    f_v = u2.block(1, 0, 1, u_deque_.size()).mean();
    f_w = u2.block(2, 0, 1, u_deque_.size()).mean();

    roll = std::atan2(-f_v, -f_w);
    pitch = std::atan2(f_u, std::sqrt(std::pow(f_v, 2) + std::pow(f_w, 2)));

    attitude = Eigen::Vector3d(roll, pitch, para_ptr_->init_heading2_);

    quat2_ = Rotation2Quaternion(Rotation2b(attitude));

    x_h_.block(9, 0, 3, 1) = para_ptr_->init_pos2_;
    x_h_.block(15, 0, 3, 1) = attitude;

    return true;

}

bool TwoFootEkf::Signal2Bool(int signal) {

    if (signal > 3 || signal < 0) {
        MYERROR("Detector signal is out of range ([0,1,2,3]),please check!")
        return false;
    }


    if (signal == 0) {
        zupt1_ = false;
        zupt2_ = true;
        return true;
    } else if (signal == 1) {
        zupt1_ = true;
        zupt2_ = false;
        return true;

    } else if (signal == 2) {
        zupt1_ = false;
        zupt2_ = true;
        return true;

    } else if (signal == 3) {
        zupt1_ = true;
        zupt2_ = true;
        return true;

    } else {
        MYERROR("signal is :" + signal)
        return false;
    }
    return true;
}

Eigen::MatrixXd TwoFootEkf::GetPosition(Eigen::MatrixXd u,
                                        int detector_signal,
                                        double time) {
    //Update the time stamp.
    last_time_ = the_time_;
    the_time_ = time;

    Signal2Bool(detector_signal);

    u_ = u;

    //Check size of U.
    //Don't use try and catch for unusal value processed
    if (u.rows() != 12 || u.cols() != 1) {
        std::cout << "size of u is :"
                  << u.rows() << " x " << u.cols() << std::endl;
        MYERROR("HERE");
    }


    /*
     * Push data into the vector and not compute other data.
     */
    if (u_deque_.size() < para_ptr_->navigation_initial_min_length_ - 1) {
        u_deque_.push_back(u);
        status_deque_.push_back(detector_signal);
        time_deque_.push_back(time);
        return Eigen::MatrixXd::Zero(18, 1);
    }

    /*
     * Initial data use some data.
     * And the data save in the deque also use to compute
     * the EKF.
     */
    if (u_deque_.size() == para_ptr_->navigation_initial_min_length_ - 1) {
        u_deque_.push_back(u);
        status_deque_.push_back(detector_signal);
        time_deque_.push_back(time);
        //Initial nav equations.
        if (InitNavEq()) {
//            std::cout << "Initial navigation equation," << x_h_ << std::endl;
        }
        last_time_ = 0.0;
        the_time_ = 0.0 - para_ptr_->Ts_;
        for (int i(0); i < u_deque_.size(); ++i) {
            GetPosition(u_deque_.at(i),
                        status_deque_.at(i),
                        time_deque_.at(i));
        }
        return x_h_;
    }

    /*
     * Filter algorithm.
     */
    if (u_deque_.size() >= para_ptr_->navigation_initial_min_length_) {
        //Navigation equations

        /*
         * Time Update
         */
        NavigationEq();

        //Get State matrix.
        StateMatrix();

        //Updata covariance matrix
        P_ = F_ * P_ * F_.transpose() +
             G_ * Q_ * G_.transpose();

        /*
         * Zero-velocity Update.
         */
        if (zupt1_ || zupt2_) {


            Eigen::MatrixXd H;
            Eigen::MatrixXd R;
            Eigen::MatrixXd z;
            Eigen::MatrixXd K;

            if (zupt1_ && zupt2_) {
                z = Eigen::MatrixXd(6, 1);
                H = H12_;
                R = R12_;

                z.block(0, 0, 3, 1) = -x_h_.block(3, 0, 3, 1);
                z.block(3, 0, 3, 1) = -x_h_.block(12, 0, 3, 1);

            } else if (zupt1_ && !zupt2_) {
                H = H1_;
                R = R1_;
                z = -x_h_.block(3, 0, 3, 1);
            } else if (!zupt1_ && zupt2_) {
                H = H2_;
                R = R2_;
                z = -x_h_.block(12, 0, 3, 1);

            }\
            Eigen::MatrixXd tmp(H * P_ * H.transpose() + R);
            K = (P_ * H.transpose()) * tmp.inverse();

            dx_ = K * z;

            if (std::isnan(dx_(2))) {
                std::cout << "nan" << std::endl;
            }


            P_ = (Id_ - K * H) * P_;

            ComputeInternalStates();
        }//End Zero-velocity constaint.


        without_range_constraint_times_++;
        //ToDo:The code use to range constraint.
//        if (para_ptr_->IsRangeConstraint &&
//            without_range_constraint_times_ > para_ptr_->RangConstraintIntervel_
//            && (x_h_.block(0, 0, 3, 1) - x_h_.block(9, 0, 3, 1)).norm() >
//               para_ptr_->rang_constraint_
//                ) {
//            without_range_constraint_times_ = 0;//Rest the counter.
//            std::cout << "IN HERE" <<std::endl;
//
//
//        }


    }

    P_ = (P_ + P_.transpose()) * 0.5;

//    if (without_range_constraint_times_ % 100 == 0) {
//        std::cout << P_.norm() << std::endl;
//        std::cout << "------------------------------" << std::endl;
//
//    }


    return x_h_;

}

bool TwoFootEkf::Initial() {

    the_time_ = 0 - para_ptr_->Ts_;

    ZEROLIZEMATRIX(P_);

    //System number 1.
    for (int i(0); i < 3; ++i) {
        P_(i, i) = std::pow(para_ptr_->sigma_initial_pos1_(i), 2);
    }

    for (int i(3); i < 6; ++i) {
        P_(i, i) = std::pow(para_ptr_->sigma_initial_vel1_(i - 3), 2);
    }

    for (int i(6); i < 9; ++i) {
        P_(i, i) = std::pow(para_ptr_->sigma_initial_att1_(i - 6), 2);
    }

    //System number 2.
    for (int i(9); i < 12; ++i) {
        P_(i, i) = std::pow(para_ptr_->sigma_initial_pos2_(i - 9), 2);
    }

    for (int i(12); i < 15; ++i) {
        P_(i, i) = std::pow(para_ptr_->sigma_initial_vel2_(i - 12), 2);
    }

    for (int i(15); i < 18; ++i) {
        P_(i, i) = std::pow(para_ptr_->sigma_initial_att2_(i - 15), 2);
    }



    /////////////////////////////////////////////////

    //R1,R2,R12
    ZEROLIZEMATRIX(R1_)
    ZEROLIZEMATRIX(R2_)
    ZEROLIZEMATRIX(R12_)


    for (int i(0); i < 3; ++i) {
        R1_(i, i) = std::pow(para_ptr_->sigma_vel_(i), 2);
        R2_(i, i) = std::pow(para_ptr_->sigma_vel_(i), 2);
    }

    R12_.block(0, 0, 3, 3) = R1_;

    R12_.block(3, 3, 3, 3) = R2_;

    //Q
    ZEROLIZEMATRIX(Q_)

    for (int i(0); i < 3; ++i) {
        Q_(i, i) = std::pow(para_ptr_->sigma_acc_(i), 2);
    }

    for (int i(3); i < 6; ++i) {
        Q_(i, i) = std::pow(para_ptr_->sigma_gyro_(i - 3), 2);
    }

    for (int i(6); i < 9; ++i) {
        Q_(i, i) = std::pow(para_ptr_->sigma_acc_(i - 6), 2);
    }

    for (int i(9); i < 12; ++i) {
        Q_(i, i) = std::pow(para_ptr_->sigma_gyro_(i - 9), 2);
    }

    //H1 H2
    ZEROLIZEMATRIX(H1_)
    ZEROLIZEMATRIX(H2_)
    ZEROLIZEMATRIX(H12_)


    H1_.block(0, 3, 3, 3) = Eigen::Matrix3d::Identity();

    H2_.block(0, 12, 3, 3) = Eigen::Matrix3d::Identity();

    H12_.block(0, 3, 3, 3) = Eigen::Matrix3d::Identity();
    H12_.block(3, 12, 3, 3) = Eigen::Matrix3d::Identity();


    /*
     * Initial vec (In matlab code.)
     */


    ZEROLIZEMATRIX(x_h_);

    //Identity matrix
    ZEROLIZEMATRIX(Id_)

    for (int i(0); i < Id_.rows(); ++i) {
        Id_(i, i) = 1.0;
    }

    return true;
}

#endif //IMU_NAVIGATION_TWOFOOTCONTAINT_H
