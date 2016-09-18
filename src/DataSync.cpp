//Create by steve in 16-9-12 at 下午2:12
//
// Created by steve on 16-9-12.
//
#include "../include/IMU_NAVIGATION/DataSynchronization.h"

#include <iostream>

#include <ros/ros.h>

#include <sensor_msgs/Imu.h>

#include <thread>

#include "../include/IMU_NAVIGATION/SettingPara.h"

#include "../include/IMU_NAVIGATION/Zero_Detecter.h"

#include "../include/IMU_NAVIGATION/TwoFootContaint.h"

DataSync* data_ptr;

#include <fstream>

void read_data()
{
    ros::Duration dura(0,90);

    SettingPara para;

    ZeroDetector detector(para);

    TwoFootEkf edf2(para);

    std::ofstream of("/home/steve/1.csv");
    std::ofstream of2("/home/steve/2.csv");



    while(ros::ok())
    {
//         std::cout << (data_ptr->GetImuData(ros::Time::now().toSec())).transpose() << std::endl;
        int status(0);
        double the_time(ros::Time::now().toSec());
        Eigen::VectorXd u(data_ptr->GetImuData(the_time));
        status = detector.Detector(u);
//        if(status == 0)
//        {
//            of << 0 << std::endl;
//            of2 << 0 << std::endl;
//        }else if(status == 1)
//        {
//            of << 1 << std::endl;
//            of2 << 0 << std::endl;
//        }else if (status == 3)
//        {
//            of << 0 << std::endl;
//            of2 << 1 << std::endl;
//        }else
//        {
//            of << 1 << std::endl;
//            of2 << 1 << std::endl;
//        }
        edf2.GetPosition(u, status, the_time);
        dura.sleep();
    }
    of.close();
    of2.close();

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

