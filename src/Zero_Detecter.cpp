//
// Created by steve on 16-9-11.
//

#include "../include/IMU_NAVIGATION/Zero_Detecter.h"

#include <iostream>

#include <ros/ros.h>



int main(int argc,char** argv) {
    //

    ros::init(argc,argv,"Imu_publish");

    ros::NodeHandle n_handle;

    //c_TransformMatrix = Eigen::Matrix4f::Identity();

    pub = n_.advertise<sensor_msgs::PointCloud2>("pc_t",1);

    ros::Subscriber sub= n_.subscribe("pc",1,transformCallback);
    ros::spin();
    return 0;


}

