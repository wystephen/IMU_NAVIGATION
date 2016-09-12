//
// Created by steve on 16-9-11.
//

#include "../include/IMU_NAVIGATION/imu_data_preprocess.h"

#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>





int main(int argc,char** argv) {
    //

    ros::init(argc,argv,"Imu_publish");

    ros::NodeHandle n_handle;

    //c_TransformMatrix = Eigen::Matrix4f::Identity();

//    pub = n_.advertise<sensor_msgs::PointCloud2>("pc_t",1);
//
//    ros::Subscriber sub= n_.subscribe("pc",1,transformCallback);

    ImuDataPreProcess<double> imupre("Data",n_handle);
    imupre.test();

    imupre.StartPub(n_handle);

    std::cout << " END THE PUBLISH" << std::endl;

    ros::spin();
    return 0;


}