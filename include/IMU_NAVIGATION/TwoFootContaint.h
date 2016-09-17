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

class TwoFootEkf:public PdrEkf
{
public:

    TwoFootEkf(SettingPara &para):para_ptr_(&para)
    {
        //Initial the filter.
        Initial();

    }

    /*
     * Compute a estimation of position(return this value as a Matrix)
     * use the giving Matrix u.
     */
    Eigen::MatrixXd GetPosition(Eigen::MatrixXd u,int detector_signal,double time);



protected:

    SettingPara* para_ptr_;//Use the pointer point to a global value .

    Eigen::VectorXf x_h_ = Eigen::VectorXf(18);

    Eigen::Matrix<double,18,18> Id_;

    //Value use for filter.
    Eigen::Matrix<double,18,18> P_;

    Eigen::Matrix<double,12,12> Q_;

    Eigen::Matrix<double,3,18> H1_,H2_;

    Eigen::Matrix<double,6,18> H12_;

    Eigen::Matrix<double,3,3> R1_,R2_;

    Eigen::Matrix<double,6,6> R12_;

    //quat
    Eigen::Vector4d quat1_,quat2_;

    std::deque<Eigen::MatrixXd> u_deque_;



    bool zupt1_ = false;
    bool zupt2_ = false;

    double last_time_ = 0.0;//
    double the_time_ = 0.0;

    /*
     * Initialization.
     */
    bool Initial();

    /*
     *  Convert detector_signal to two bool value.
     * return false when detector_signal is out of range([0,1,2,3]).
     */

    bool Signal2Bool(int signal);

    /*
     * This function is use for calculates the initial stata of the
     * navigation equations.
     */
    bool InitNavEq();


    /*
     * The mechanized navigation euqtions of the inertial navigation system.
     */
    bool NavigationEq();

private:

};

bool TwoFootEkf::NavigationEq() {
    Eigen::Vectorxd u1(6),u2(6);

    u1 = u_deque_.end()->block(0,0,6,1);
    u2 = u_deque_.end()->block(6,0,6,1);

    Eigen::VectorXd last_x_h(18);
    last_x_h = x_h_;



    Eigen::Vector3d w_tb;
    double v(0.0);
    double P(0.0),Q(0.0),R(0.0),dt(the_time_-last_time_);


    /*
     * Update quaternion q.
     */

    //For quat1_
    w_tb = u1.block(3,0,3,1);
    v = w_tb.norm() * dt;

    Eigen::Matrix4d OMEGA;

    if(std::fabs(v) > 1e-8)
    {
        P = w_tb(0)*dt*0.5;
        Q = w_tb(1) * dt * 0.5;
        R = w_tb(3) * dt * 0.5;

        OMEGA(0,0) = 0;OMEGA(0,1)   = R;OMEGA(0,2)  =-Q;OMEGA(0,3)  = P;

        OMEGA(1,0) = -R;OMEGA(1,1)  = 0;OMEGA(1,2)  = P;OMEGA(1,3)  = Q;

        OMEGA(2,0) = Q;OMEGA(2,1)   = -P;OMEGA(2,2) = 0;OMEGA(2,3)  = R;

        OMEGA(3,0) = -P;OMEGA(3,1)  = -Q;OMEGA(3,2) = -R;OMEGA(3,3) = 0;

        quat1_ = (std::cos(v/2)*Eigen::Matrix4d::Identity()+
        2/v * std::sin(v/2) *OMEGA) * quat1_;

    }

    //For quat2_
    w_tb = u2.block(3,0,3,1);
    v = w_tb.norm() * dt;

    Eigen::Matrix4d OMEGA;

    if(std::fabs(v) > 1e-8)
    {
        P = w_tb(0)*dt*0.5;
        Q = w_tb(1) * dt * 0.5;
        R = w_tb(3) * dt * 0.5;

        OMEGA(0,0) = 0;OMEGA(0,1)   = R;OMEGA(0,2)  =-Q;OMEGA(0,3)  = P;

        OMEGA(1,0) = -R;OMEGA(1,1)  = 0;OMEGA(1,2)  = P;OMEGA(1,3)  = Q;

        OMEGA(2,0) = Q;OMEGA(2,1)   = -P;OMEGA(2,2) = 0;OMEGA(2,3)  = R;

        OMEGA(3,0) = -P;OMEGA(3,1)  = -Q;OMEGA(3,2) = -R;OMEGA(3,3) = 0;

        quat2_ = (std::cos(v/2)*Eigen::Matrix4d::Identity()+
                  2/v * std::sin(v/2) *OMEGA) * quat2_;
    }


    /*
     * Update the position and velocity states using the measured specific force,
     * and the newly calculated attitude.
     */

    Eigen::Vector3d g_t(0,0,para_ptr_->gravity_);
    

    return  true;
}

bool TwoFootEkf::InitNavEq() {
    /*
     * This function used the assumption that the system is
     * stationary during the first
     * server samples(depends on the parameter
     * SettingPara.navigation_initial_min_length_),
     * all data use the average of these data.
     */
    Eigen::MatrixXd u1(6,u_deque_.size());
    Eigen::MatrixXd u2(6,u_deque_.size());

    Eigen::Vector3d attitude(0,0,0);

    int index(0);

    for(auto it = u_deque_.begin();it!=u_deque_.end();++it)
    {
        u1.block(0,index,6,1) = it->block(0,0,6,1);
        u2.block(0,index,6,1) = it->block(6,0,6,1);
        index ++;
    }

    /////////////////////////////////
    /*
     * For u1.
     */

    double f_u(u1.block(0,0,1,u_deque_.size()).mean());
    double f_v(u1.block(1,0,1,u_deque_.size()).mean());
    double f_w(u1.block(2,0,1,u_deque_.size()).mean());

    double roll(std::atan2(-f_v,-f_w));
    double pitch(std::atan2(f_u,std::sqrt(std::pow(f_v,2)+std::pow(f_w,2))));

    attitude = Eigen::Vector3d(roll,pitch,para_ptr_->init_heading1_);

    quat1_ = Rotation2Quaternion(Rotation2b(attitude));

    x_h_.block(0,0,3,1) = para_ptr_->init_pos1_;
    x_h_.block(6,0,3,1) = attitude;

    /*
     * For u2
     */

    f_u = u2.block(0,0,1,u_deque_.size()).mean();
    f_v = u2.block(1,0,1,u_deque_.size()).mean();
    f_w = u2.block(2,0,1,u_deque_.size()).mean();

    roll = std::atan2(-f_v,-f_w);
    pitch = std::atan2(f_u,std::sqrt(std::pow(f_v,2)+std::pow(f_w,2)));

    attitude = Eigen::Vector3d(roll,pitch,para_ptr_->init_heading2_);

    quat2_ = Rotation2Quaternion(Rotation2b(attitude));

    x_h_.block(9,0,3,1) = para_ptr_->init_pos2_;
    x_h_.block(15,0,3,1) = attitude;

    return true;

}

bool TwoFootEkf::Signal2Bool(int signal) {

    if(signal>3 || signal<0)
    {
        MYERROR("Detector signal is out of range ([0,1,2,3]),please check!")
        return false;
    }

    switch(signal)
    {
        case 0:
            zupt1_ = false;
            zupt2_ = true;
            break;
        case 1:
            zupt1_ = true;
            zupt2_ = false;
            break;
        case 2:
            zupt1_ = false;
            zupt2_ = true;
            break;
        case 3:
            zupt1_ = true;
            zupt2_ = true;
            break;
        default:

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

    //Check size of U.
    //Don't use try and catch for unusal value processed
    if( u.rows() != 12 || u.cols() != 1)
    {
        std::cout << "size of u is :"
                  << u.rows() << " x " << u.cols() << std::endl;
        MYERROR("HERE");
    }

    if(u_deque_.size()<para_ptr_->navigation_initial_min_length_-1)
    {
        u_deque_.push_back(u);
        return true;
    }
    if(u_deque_.size() == para_ptr_->navigation_initial_min_length_-1)
    {
        u_deque_.push_back(u);
        //Initial nav equations.
        return InitNavEq();
    }
    if(u_deque_.size() == para_ptr_->navigation_initial_min_length_)
    {
        //Navigation equations

    }

}

bool TwoFootEkf::Initial() {
    ZEROLIZEMATRIX(P_);

    //System number 1.
    for(int i(0);i<3;++i)
    {
        P_(i,i) = std::sqrt(para_ptr_->sigma_initial_pos1_(i));
    }

    for(int i(3);i<6;++i)
    {
        P_(i,i) = std::sqrt(para_ptr_->sigma_initial_vel1_(i-3));
    }

    for(int i(6);i<9;++i)
    {
        P_(i,i) = std::sqrt(para_ptr_->sigma_initial_att1_(i-6));
    }

    //System number 2.
    for(int i(9);i<12;++i)
    {
        P_(i,i) = std::sqrt(para_ptr_->sigma_initial_pos2_(i-9));
    }

    for(int i(12);i<15;++i)
    {
        P_(i,i) = std::sqrt(para_ptr_->sigma_initial_vel2_(i-12));
    }

    for(int i(15);i<18;++i)
    {
        P_(i,i) = std::sqrt(para_ptr_->sigma_initial_att2_(i-15));
    }

    /////////////////////////////////////////////////

    //R1,R2,R12
    ZEROLIZEMATRIX(R1_)
    ZEROLIZEMATRIX(R2_)
    ZEROLIZEMATRIX(R12_)


    for(int i(0);i<3;++i)
    {
        R1_(i,i) = std::sqrt(para_ptr_->sigma_vel_(i));
        R2_(i,i) = std::sqrt(para_ptr_->sigma_vel_(i));
    }

    R12_.block(0,0,3,3) = R1_;

    R12_.block(3,3,3,3) = R2_;

    //Q
    ZEROLIZEMATRIX(Q_)

    for(int i(0);i<3;++i)
    {
        Q_(i,i) = std::sqrt(para_ptr_->sigma_acc_(i));
    }

    for(int i(3);i<6;++i)
    {
        Q_(i,i) = std::sqrt(para_ptr_->sigma_gyro_(i-3));
    }

    for(int i(6);i<9;++i)
    {
        Q_(i,i) = std::sqrt(para_ptr_->sigma_acc_(i-6));
    }

    for(int i(9);i<12;++i)
    {
        Q_(i,i) = std::sqrt(para_ptr_->sigma_gyro_(i-9));
    }

    //H1 H2
    ZEROLIZEMATRIX(H1_)
    ZEROLIZEMATRIX(H2_)
    ZEROLIZEMATRIX(H12_)



    H1_.block(0,3,3,3) = Eigen::Matrix3d::Identity();

    H2_.block(0,12,3,3) = Eigen::Matrix3d::Identity();

    H12_.block(0,3,3,3) = Eigen::Matrix3d::Identity();
    H12_.block(3,12,3,3) = Eigen::Matrix3d::Identity();


    //Identity matrix
    ZEROLIZEMATRIX(Id_)

    for(int i(0);i<Id_.rows();++i)
    {
        Id_(i,i) = 1.0;
    }

    return true;
}

#endif //IMU_NAVIGATION_TWOFOOTCONTAINT_H
