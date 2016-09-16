#pragma once
//Create by steve in 16-9-14 at 下午4:27
//
// Created by steve on 16-9-14.
//

#include <ros/ros.h>

#include <Eigen/Dense>

#include "PdrEkf.h"

#include "SettingPara.h"

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

        /////////////////////////////////////////////////////////
        // Initial P
//        for(int i(0);i<18;++i)
//        {
//            for(int j(0);j<18;++j)
//            {
//                P_(i,j) = 0;
//            }
//        }
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



    }


protected:

    SettingPara* para_ptr_;//Use the pointer point to a global value .


    //Value use for filter.
    Eigen::Matrix<double,18,18> P_;

    Eigen::Matrix<double,12,12> Q_;

    Eigen::Matrix<double,3,18> H1_,H2_;

    Eigen::Matrix<double,6,18> H12_;

    Eigen::Matrix<double,3,3> R1_,R2_;

    Eigen::Matrix<double,6,6> R12_;





private:


};

#endif //IMU_NAVIGATION_TWOFOOTCONTAINT_H
