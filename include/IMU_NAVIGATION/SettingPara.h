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




#ifndef IMU_NAVIGATION_SETTINGPARA_H
#define IMU_NAVIGATION_SETTINGPARA_H

struct SettingPara
{
    bool LoadParaFromJson()
    {
        //ToDo:Load paramenters form a json_like file.

        return true;
    }

    SettingPara(bool isDefult)
    {
        latitude_ = 58.0;

        altitude_ = 100.0;

        ComputeGravity();

        Ts_ = 1/820.0;

        rang_constraint = 1.0;




    }

    double Ts_ = 10000;//Sampling period [s]

    double latitude_=1000.0;//In paper 58 [degrees]

    double altitude_ = 1000.0;//In paper 100 [m]

    double gravity_;//The gravity based on altitude and latitude. [ m/s^2]

    double rang_constraint = 1.0; // Max distance between two foots.[m]



    void ComputeGravity()
    {
        if(latitude_>800)
        {
            MYERROR("latitude_ is not set or bigger than 800.0 !")

        }else{
            double lambda(M_PI /180.0 * latitude_);

            /*
             * gamma=9.780327*(1+0.0053024*sin(lambda)^2-0.0000058*sin(2*lambda)^2);
             */
            double gamma = 9.780327 * ( 1 + 0.0053024 * std::pow(std::sin(lambda),2.0) -
                                      0.0000058 * std::pow(std::sin(2 * lambda) , 2.0));

            /*
             * gamma=9.780327*(1+0.0053024*sin(lambda)^2-0.0000058*sin(2*lambda)^2);
             * (h is altitude.)
             */
            gravity_ = gamma - ( (3.0877e-6) -
                    (0.004e-6) * std::pow(std::sin(lambda),2)*altitude_+
                    (0.072e-12)*std::pow(altitude_,2));


        }
    }

};



#endif //IMU_NAVIGATION_SETTINGPARA_H
