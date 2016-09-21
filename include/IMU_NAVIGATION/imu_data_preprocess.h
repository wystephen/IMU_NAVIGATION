//
// Created by steve on 16-9-11.
//

#include "../Cpp_Extent/CSVReader.h"

#include "../Cpp_Extent/Matrix.h"

#include "../Cpp_Extent/MyError.h"

#include <ros/ros.h>

#include "SettingPara.h"

#include <sensor_msgs/Imu.h>


#ifndef IMU_NAVIGATION_IMU_DATA_PREPROCESS_H
#define IMU_NAVIGATION_IMU_DATA_PREPROCESS_H

enum DATASOURCE{
    MONOIMU,
    DOUBLEIMU,
    MONOFILE,
    DOUBLEFILE,

};

template <class DataType>
class ImuDataPreProcess {
public:
    ImuDataPreProcess(std::string DataDir,ros::NodeHandle &n_h):
            csvReaderu1(DataDir +"/u1.csv"),
                        csvReaderu2(DataDir +"/u2.csv"),
                        csvReaderzupt1(DataDir +"/zupt1.csv"),
                        csvReaderzupt2(DataDir +"/zupt2.csv"),
                                           para_(true),
                                           node_ptr_(&n_h)
    {
        u1=csvReaderu1.m_;
        u2 = csvReaderu2.m_;

        zupt1 = csvReaderzupt1.m_;
        zupt2 = csvReaderzupt2.m_;

        if(u1.GetRows()!=u2.GetRows())
        {
            MYERROR("The rows_ of u1 and u2 is not equal.")
        }

    }

    void test()
    {
        std::cout << "Out" << std::endl;
        std::cout << u1.GetRows() << " x " << u1.GetCols() << std::endl;
        std::cout << u2.GetRows() << " x " << u2.GetCols() << std::endl;
    }


    /*
     * Set a nodehandle for create publisher for subscriber.
     */
    bool SetNodeHandle(ros::NodeHandle &n_h)
    {
        *node_ptr_ = n_h;

        return true;
    }

    /*
     * Use data read from csv file,publish data .Need a right value of the time step.
     */
    bool StartPub(ros::NodeHandle &n_h)
    {
        SetNodeHandle(n_h);

        pub1_ = node_ptr_->advertise<sensor_msgs::Imu>("imu1/data", 100);//Define the publisher.

        pub2_ = node_ptr_->advertise<sensor_msgs::Imu>("imu2/data", 100);

        double start_time(ros::Time::now().toSec());//

        double t_s(para_.Ts_);

        double the_time;

        ros::Duration d;
        d.fromSec(t_s);

        for( int i(0);i<u1.GetRows();++i)
        {
            while(true)
            {
                the_time = ros::Time::now().toSec();
                if(fabs(the_time-start_time - t_s * i)<0.01)
                {
                    break;
                }else
                {
                    d.sleep();
                }
            }

            if(!ros::ok())
            {
                exit(1);
            }


            sensor_msgs::Imu imu1;
            imu1.header.stamp.fromSec(start_time + i *  t_s);
            imu1.header.frame_id = "imu1";

            imu1.linear_acceleration.x = *u1(i,0);
            imu1.linear_acceleration.y = *u1(i,1);
            imu1.linear_acceleration.z = *u1(i,2);

            imu1.angular_velocity.x = *u1(i,3);
            imu1.angular_velocity.y = *u1(i,4);
            imu1.angular_velocity.z = *u1(i,5);

            sensor_msgs::Imu imu2;
            imu2.header.stamp.fromSec(start_time+i*t_s);
            imu2.header.frame_id = "imu2";

            imu2.linear_acceleration.x = *u2(i,0);
            imu2.linear_acceleration.y = *u2(i,1);
            imu2.linear_acceleration.z = *u2(i,2);

            imu2.angular_velocity.x = *u2(i,3);
            imu2.angular_velocity.y = *u2(i,4);
            imu2.angular_velocity.z = *u2(i,5);


            pub1_.publish(imu1);
            pub2_.publish(imu2);



        }

        return true;


    }


protected:
    CSVReader csvReaderu1,csvReaderu2,csvReaderzupt1,csvReaderzupt2;

    Matrix<DataType> u1,u2,zupt1,zupt2;//Data Matrix (Load from File).


    ros::NodeHandlePtr node_ptr_;//The pointer for current nodehandle.

    SettingPara para_;//Save parameters

    ros::Publisher pub1_,pub2_; // publisher for imu data.



private:


};


#endif //IMU_NAVIGATION_IMU_DATA_PREPROCESS_H
