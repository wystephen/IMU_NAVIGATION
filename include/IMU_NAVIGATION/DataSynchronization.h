#pragma once//Create by steve in 16-9-12 at 下午2:04//// Created by steve on 16-9-12.//#include <ros/ros.h>#include <iostream>#include <deque>#include <sensor_msgs/Imu.h>//Multi-thread in c++11.#include <thread>#include <mutex>#include "Eigen/Dense"#ifndef IMU_NAVIGATION_DATASYNCHRONIZATION_H#define IMU_NAVIGATION_DATASYNCHRONIZATION_Hclass DataSync{public:    DataSync(ros::NodeHandle &n_h):            n_ptr_(&n_h),            x_h_(18)    {        //x_h_[1]  =1;        std::cout << x_h_ << std::endl;        imu1_sub_ = n_ptr_->subscribe("imu1/data",10,&DataSync::ImuCallBack,this);        imu2_sub_ = n_ptr_->subscribe("imu2/data",10,&DataSync::ImuCallBack,this);        //std::cout << pow(3,3) << std::endl;    }    ~DataSync()    {    }    //Callback function,use for push imu msg to deque.    void ImuCallBack(const sensor_msgs::ImuConstPtr &imu_ptr);    //Get pair of imu data according to the time stamp.    Eigen::MatrixXd GetImuData(double time);    //Get imu data at the time stamp    Eigen::MatrixXd GetImuDataSingle(double time,std::deque<sensor_msgs::Imu> deque_imu);protected:    ros::NodeHandlePtr n_ptr_;//Pointer of the node.    ros::Subscriber imu1_sub_,imu2_sub_;//Subscriber for imu data.    std::deque<sensor_msgs::Imu> deque_imu1_;//Save data in a deque.    std::deque<sensor_msgs::Imu> deque_imu2_;    std::mutex deque_imu1_mutex_;    std::mutex deque_imu2_mutex_;    Eigen::VectorXd x_h_;    int queue_size_ = 20;private:};void DataSync::ImuCallBack(const sensor_msgs::ImuConstPtr &imu_ptr) {    sensor_msgs::Imu tmp(*imu_ptr);    if(tmp.header.frame_id == "imu1")    {        deque_imu1_mutex_.lock();        deque_imu1_.push_back(tmp);        while(deque_imu1_.size()>queue_size_)        {            deque_imu1_.pop_front();        }        //std::cout << "deque_imu1_size:" << deque_imu1_.size() << std::endl;        deque_imu1_mutex_.unlock();    }else{        deque_imu2_mutex_.lock();        deque_imu2_.push_back(tmp);        //std::cout << "deque_imu2_size:" << deque_imu2_.size() << std::endl;        while(deque_imu2_.size() > queue_size_)        {            deque_imu2_.pop_front();        }        deque_imu2_mutex_.unlock();    }}Eigen::MatrixXd DataSync::GetImuData(double time) {    Eigen::MatrixXd data(12,1);    Eigen::MatrixXd tmp_data(6,1);    /*     *     */    deque_imu1_mutex_.lock();    tmp_data = GetImuDataSingle(time,deque_imu1_);    while(deque_imu1_.size()>queue_size_)    {        deque_imu1_.pop_front();    }    deque_imu1_mutex_.unlock();    for(int i(0);i<6;++i)    {        data(i,0) = tmp_data(i,0);    }    ////////////////////////////////////////    deque_imu2_mutex_.lock();    tmp_data = GetImuDataSingle(time,deque_imu2_);    while(deque_imu2_.size() > queue_size_)    {        deque_imu2_.pop_front();    }    deque_imu2_mutex_.unlock();    for(int i(0);i<6;++i)    {        data(i+6,0)= tmp_data(i,0);    }    /*     *     */    return data;}Eigen::MatrixXd DataSync::GetImuDataSingle(double time,                                           std::deque<sensor_msgs::Imu> deque_imu) {    Eigen::MatrixXd m(6,1);    sensor_msgs::Imu d1,d2;    //Define the MatrixXd//    for(std::deque<sensor_msgs::Imu>::iterator it(deque_imu.end());//            it!=deque_imu.begin();//            --it)//    {//        if(it->header.stamp.toSec() > time)//        {//            continue;//        }else{//            if(it!=deque_imu.end())//            {//                std::cout << "case1" << std::endl;//                d1  = *it;//                ++ it;//                d2 = *it;////                break;////            }else if(it == deque_imu.begin())//            {//                d1 = *it;//                ++ it;//                d2 = *it;//                break;//            }//            else{//                std::cout << "case2" << std::endl;//                d1 = *it;//                std::cout << "case22" << std::endl;//                if(it == deque_imu.begin())//                {//                    ++it;//                }else{//                    --it;//                }//                d2 = *it;////                break;//            }//        }//    }    int now_size = deque_imu.size();    while(deque_imu.size() < 5)    {        return m;    }    d1 = deque_imu.at(now_size-1);    d2 = deque_imu.at(now_size-2);    std::cout << "time:" << time << std::endl;    double alpha(0.0);    alpha = (time - d1.header.stamp.toSec()) / (d2.header.stamp.toSec()-d1.header.stamp.toSec());    //std::cout << "alpha: " << d2.header.stamp.toSec()-d1.header.stamp.toSec() << std::endl;    m(0,0) = (1-alpha) * d1.linear_acceleration.x + alpha * d2.linear_acceleration.x;    m(1,0)= (1-alpha) * d1.linear_acceleration.y + alpha * d2.linear_acceleration.y;    m(2,0) = (1-alpha) * d1.linear_acceleration.z + alpha * d2.linear_acceleration.z;    m(3,0) = (1-alpha) * d1.angular_velocity.x + alpha * d2.angular_velocity.x;    m(4,0) = (1-alpha) * d1.angular_velocity.y + alpha * d2.angular_velocity.y;    m(5,0) = (1-alpha) * d1.angular_velocity.z + alpha * d2.angular_velocity.z;    if(d2.header.stamp.toSec()-d1.header.stamp.toSec() < 0.001)    {        m(0,0) = d2.linear_acceleration.x;        m(1,0) = d2.linear_acceleration.y;        m(2,0) = d2.linear_acceleration.z;        m(3,0) = d2.angular_velocity.x;        m(4,0) = d2.angular_velocity.y;        m(5,0) = d2.angular_velocity.z;    }    return m;}#endif //IMU_NAVIGATION_DATASYNCHRONIZATION_H