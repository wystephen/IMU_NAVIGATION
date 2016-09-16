//Create by steve in 16-9-14 at 下午4:27
//
// Created by steve on 16-9-14.
//

#include "../include/IMU_NAVIGATION/TwoFootContaint.h"

#include <ros/ros.h>



#include "../include/IMU_NAVIGATION/DataSynchronization.h"

#include "../include/IMU_NAVIGATION/Zero_Detecter.h"




int main(int argc, char **argv) {
    ros::init(argc, argv, "TwoFoot");

    ros::NodeHandle n_handle;

    SettingPara para(true);

    ros::spin();
    return 0;
}