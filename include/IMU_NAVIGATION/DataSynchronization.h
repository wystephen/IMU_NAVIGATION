#pragma once
//Create by steve in 16-9-12 at 下午2:04
//
// Created by steve on 16-9-12.
//

#include <ros/ros.h>

#include <iostream>


#ifndef IMU_NAVIGATION_DATASYNCHRONIZATION_H
#define IMU_NAVIGATION_DATASYNCHRONIZATION_H

class DataSync
{
public:

    DataSync(ros::NodeHandle & n_h):
            n_ptr_(&n_h)

    {

    }

protected:

    ros::NodeHandlePtr n_ptr_;//Pointer of the node.

    ros::Subscriber imu1_sub,imu2_sub;//Subscriber for imu data.



private:


};

#endif //IMU_NAVIGATION_DATASYNCHRONIZATION_H
