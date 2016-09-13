//Create by steve in 16-9-12 at 下午2:12
//
// Created by steve on 16-9-12.
//
#include "../include/IMU_NAVIGATION/DataSynchronization.h"

#include <iostream>

#include <ros/ros.h>

#include <sensor_msgs/Imu.h>


int main(int argc,char ** argv)
{
    ros::init(argc,argv,"DataSync");
    ros::NodeHandle n_handle;

    DataSync dataSync(n_handle);




    ros::spin();

    return 0;
}