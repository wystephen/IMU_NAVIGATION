#pragma once
//Create by steve in 16-9-12 at 上午9:51
//
// Created by steve on 16-9-12.
//

#include "../Cpp_Extent/MyError.h"

#include <stdio.h>
#include <cmath>

#include "../Cpp_Extent/JsonCodert.h"

#include "../Cpp_Extent/FileReader.h"

#include <Eigen/Dense>

enum ZeroDetectorAlogorithm
{
    GLRT,
    MV,
    MAG,
    ARE
};


#ifndef IMU_NAVIGATION_SETTINGPARA_H
#define IMU_NAVIGATION_SETTINGPARA_H

struct SettingPara
{
    bool LoadParaFromJson()
    {
        //ToDo:Load paramenters form a json_like file.

        return true;
    }

    SettingPara(bool isDefult=true)
    {
        latitude_ = 58.0;

        altitude_ = 100.0;

        ComputeGravity();

        // Ts_ = 1/820.0;

//        rang_constraint_ = 1.0;

//        for(int i(0);i<3;++i)
//        {
//            init_pos1_(i) = 0.0;
//            init_pos2_(i) = 0.0;
//            init_pos_(i) = 0.0;
//        }



    }

    int navigation_initial_min_length_ = 20;
    //The minimal length for initial the navigation equational.

    double Ts_ = 1 / 820.0;//Sampling period [s]

    double latitude_ = 58.0;//In paper 58 [degrees]

    double altitude_ = 100.0;//In paper 100 [m]

    double gravity_;//The gravity based on altitude and latitude. [ m/s^2]

    double rang_constraint_ = 1.0; // Max distance between two foots.[m]

    int ZeroDetectorWindowSize_ = 3;// Size of the Zero Detector.

    bool IsRangeConstraint = true;//Use or not use the range constraint to imporve the performence of location.

    int RangConstraintIntervel_ = 820;

    double init_heading1_ = (-97.5) * M_PI /180.0;//heading
    double init_heading2_ = (96.05) * M_PI / 180.0;

    Eigen::Vector3d init_pos1_ = Eigen::Vector3d(0, 0, 0);//pose
    Eigen::Vector3d init_pos2_ = Eigen::Vector3d(0, 0, 0);
    Eigen::Vector3d init_pos_ = Eigen::Vector3d(0, 0, 0);

    double sigma_a_ = 0.035;//Stanard deviation of the accelermter noise [ m/ s^2];
    double sigma_g_ = 0.35 * M_PI /180.0;

    double gamma_ = 200;//Threshold used in the zero-velocity detector.

//    bool Open_Distance_Constaint_ = true;

    //FILTER PARAMETERS
    Eigen::Vector3d sigma_acc_ = 4 * 0.7 * Eigen::Vector3d(1,1,1);//[m/s^2]

    Eigen::Vector3d sigma_gyro_ = 4 * 10 * Eigen::Vector3d(0.1,0.1,0.1) * M_PI / 180.0;//[rad/s]

    Eigen::Vector3d sigma_vel_ = 5 * Eigen::Vector3d(0.01,0.01,0.01) ; // [m/s]

    /////////////////////////////////////////////

    Eigen::Vector3d sigma_initial_pos1_= 1e-2 * Eigen::Vector3d(0.1,0.1,0.1);//Position

    Eigen::Vector3d sigma_initial_vel1_ = 1e-5 * Eigen::Vector3d(1,1,1);

    Eigen::Vector3d sigma_initial_att1_ = (M_PI/180 * Eigen::Vector3d(0.1,0.1,0.001));

    Eigen::Vector3d sigma_initial_pos2_= 1e-2 * Eigen::Vector3d(0.1,0.1,0.1);//Position

    Eigen::Vector3d sigma_initial_vel2_ = 1e-5 * Eigen::Vector3d(1,1,1);

    Eigen::Vector3d sigma_initial_att2_ = (M_PI/180 * Eigen::Vector3d(0.1,0.1,0.001));




    void ComputeGravity()
    {
        //

        if(latitude_>800)
        {
            MYERROR("latitude_ is not set or it is  bigger than 800.0 !")

        }else{
            double lambda(M_PI * latitude_ / 180.0);

            /*
             * gamma=9.780327*(1+0.0053024*sin(lambda)^2-0.0000058*sin(2*lambda)^2);
             */
            double gamma = 9.780327 * (1 + 0.0053024 * std::pow(std::sin(lambda),2.0) -
                                       0.0000058 * std::pow(std::sin(2.0 * lambda), 2.0));

            /*
             * gamma=9.780327*(1+0.0053024*sin(lambda)^2-0.0000058*sin(2*lambda)^2);
             * (h is altitude.)
             */
            gravity_ = gamma - ((3.0877e-6) -
                                (0.004e-6) * std::pow(std::sin(lambda), 2)) * altitude_ +
                       (0.072e-12) * std::pow(altitude_, 2);


        }
    }

};



#endif //IMU_NAVIGATION_SETTINGPARA_H
