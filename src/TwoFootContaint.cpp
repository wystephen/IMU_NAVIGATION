//Create by steve in 16-9-14 at 下午4:27
//
// Created by steve on 16-9-14.
//

#include "../include/IMU_NAVIGATION/TwoFootContaint.h"

#include <ros/ros.h>


#include "../include/IMU_NAVIGATION/DataSynchronization.h"

#include "../include/IMU_NAVIGATION/Zero_Detecter.h"

#include "../include/Cpp_Extent/CSVReader.h"


int main(int argc, char **argv) {
    ros::init(argc, argv, "TwoFoot");

    ros::NodeHandle n_handle;

    SettingPara para(true);

    CSVReader r_u1("/home/steve/catkin_ws/src/IMU_NAVIGATION/Data/u1.csv");
    CSVReader r_u2("/home/steve/catkin_ws/src/IMU_NAVIGATION/Data/u2.csv");
    CSVReader r_zupt1("/home/steve/catkin_ws/src/IMU_NAVIGATION/Data/zupt1.csv");
    CSVReader r_zupt2("/home/steve/catkin_ws/src/IMU_NAVIGATION/Data/zupt2.csv");

//    CSVReader r_u1("/home/steve/catkin_ws/src/IMU_NAVIGATION/Data/u2.csv");
//    CSVReader r_u2("/home/steve/catkin_ws/src/IMU_NAVIGATION/Data/u1.csv");
//    CSVReader r_zupt1("/home/steve/catkin_ws/src/IMU_NAVIGATION/Data/zupt2.csv");
//    CSVReader r_zupt2("/home/steve/catkin_ws/src/IMU_NAVIGATION/Data/zupt1.csv");


//
//    CSVReader r_u1("/home/steve/catkin_ws/src/IMU_NAVIGATION/Data/u1.csv");
//    CSVReader r_u2("/home/steve/catkin_ws/src/IMU_NAVIGATION/Data/u1.csv");
//    CSVReader r_zupt1("/home/steve/catkin_ws/src/IMU_NAVIGATION/Data/zupt1.csv");
//    CSVReader r_zupt2("/home/steve/catkin_ws/src/IMU_NAVIGATION/Data/zupt1.csv");
    double time = 0.0;

//
//
//    std::cout << r_u1.rows_ << std::endl;
//    std::cout << r_u2.rows_ << std::endl;
//
//    std::cout << r_zupt1.rows_ << std::endl;
//    std::cout << r_zupt2.rows_ << std::endl;

    for (int j(0); j < 20; ++j) {
        TwoFootEkf edf2(para);
        Eigen::MatrixXd x_h_;
        for (int i(0); i < r_u1.rows_; ++i) {
            int states(0);
            if (*r_zupt1.m_(i, 0) == 1) {
                states += 1;
            }
            if (*r_zupt2.m_(i, 0) == 1) {
                states += 2;
            }
            time += para.Ts_;

            Eigen::MatrixXd u(12, 1);
            for (int j(0); j < 6; ++j) {
                u(j, 0) = *r_u1.m_(i, j);
                u(j + 6, 0) = *r_u2.m_(i, j);
            }

            x_h_ = edf2.GetPosition(u, states, time);

        }
        std::cout << "u1:" << x_h_.block(0, 0, 3, 1).transpose();//<< std::endl;
        std::cout << "  u2:" << x_h_.block(9, 0, 3, 1).transpose() << std::endl;

        std::cout << "u1 len:" << x_h_.block(0, 0, 3, 1).norm()
                  << "  u2 len:" << x_h_.block(9, 0, 3, 1).norm() << std::endl;

    }

//    for (int i(0); i < 10; ++i) {
//        TwoFootEkf edf2(para);
//        std::cout << "-----" << std::endl;
//        edf2.test();
//    }

    std::cout << "END" << std::endl;

    std::cout << M_PI << std::endl;


    return 0;
}