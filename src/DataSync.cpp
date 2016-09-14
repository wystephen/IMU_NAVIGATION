//Create by steve in 16-9-12 at 下午2:12
//
// Created by steve on 16-9-12.
//
#include "../include/IMU_NAVIGATION/DataSynchronization.h"

#include <iostream>

#include <ros/ros.h>

#include <sensor_msgs/Imu.h>

#include <thread>

DataSync* data_ptr;

void read_data()
{
    ros::Duration dura(0,10);

    while(ros::ok())
    {
         std::cout << (data_ptr->GetImuData(ros::Time::now().toSec())).transpose() << std::endl;
        dura.sleep();

    }

}

int main(int argc,char ** argv)
{
    ros::init(argc,argv,"DataSync");
    ros::NodeHandle n_handle;

    DataSync dataSync(n_handle);

    data_ptr  = &dataSync;

    std::thread j(read_data);
    j.detach();

    ros::spin();

    return 0;
}

