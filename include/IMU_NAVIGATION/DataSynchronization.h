#pragma once
//Create by steve in 16-9-12 at 下午2:04
//
// Created by steve on 16-9-12.
//

#include <ros/ros.h>

#include <iostream>

#include <deque>

#include <sensor_msgs/Imu.h>

//Multi-thread in c++11.
#include <thread>
#include <mutex>

#include "Eigen/Dense"


#ifndef IMU_NAVIGATION_DATASYNCHRONIZATION_H
#define IMU_NAVIGATION_DATASYNCHRONIZATION_H

class DataSync
{
public:

    DataSync(ros::NodeHandle &n_h):
            n_ptr_(&n_h),
            x_h_(18)

    {
        x_h_[1]  =1;


    }

    ~DataSync()
    {
    }

protected:

    ros::NodeHandlePtr n_ptr_;//Pointer of the node.

    ros::Subscriber imu1_sub_,imu2_sub_;//Subscriber for imu data.

    std::deque<sensor_msgs::Imu> deque_imu1_;
    std::deque<sensor_msgs::Imu> deque_imu2_;

    void *Imu1CallBack(sensor_msgs::ImuPtr imu_ptr);

    Eigen::VectorXd x_h_;




private:


};

#endif //IMU_NAVIGATION_DATASYNCHRONIZATION_H
