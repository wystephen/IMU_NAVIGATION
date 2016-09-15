//
// Created by steve on 16-9-11.
//

#include <deque>
#include <iostream>

#include "SettingPara.h"

#include <memory>

#include <Eigen/Dense>

#ifndef IMU_NAVIGATION_ZERO_DETECTER_H
#define IMU_NAVIGATION_ZERO_DETECTER_H

class ZeroDetector
{
public:
    ZeroDetector(SettingPara &para):para_ptr_(&para)
    {


    }

    /*
     * Zero-volocity detection function.
     * Four value to return,
     * |            |
     * |    1   1   |   3   Both Zero-velocity
     * |    1   0   |   2   Imu1 is Zero-velcity
     * |    0   1   |   1   Imu2 is Zero-velocity
     * |    0   0   |   0   Neither Zero-velocity
     */
    int Detector(Eigen::MatrixXd u);


protected:
    SettingPara* para_ptr_;//Use the pointer point to a global value .

    ZeroDetectorAlogorithm alogorithm_=ZeroDetectorAlogorithm::GLRT;//Choice the algorithm for zero detect.

    std::deque<Eigen::MatrixXd> u_sequl_;

    /*
     * Input a 6 x W matrix to recongnization the status right now.
     */
    bool GLRT_Detector(Eigen::MatrixXd u,double sigma2_a,double sigma2_g,long W);




private:

};

int ZeroDetector::Detector(Eigen::MatrixXd u) {

    int status(0);

    if (u.rows() != 12 || u.cols() != 1)
    {
        MYERROR("Input Matrix in not the right size");
        std::cout << "The size of the Matrix is :"
                  << u.rows() << " x " << u.cols() << std::endl;
    }

    if(u_sequl_.size() < para_ptr_->ZeroDetectorWindowSize_ * 2)
    {
        u_sequl_.push_back(u);
        return status;
    }else{
        u_sequl_.pop_front();
        u_sequl_.push_back(u);
    }

    if(alogorithm_ == ZeroDetectorAlogorithm::GLRT)
    {
        double sigma2_a = std::pow(para_ptr_->sigma_a_,2);
        double sigma2_g = std::pow(para_ptr_->sigma_g_,2);

        double g = para_ptr_->gravity_;

        long W = para_ptr_->ZeroDetectorWindowSize_;

        //Read Data from deque First.

        Eigen::MatrixXd u1(6,W),u2(6,W);
        int index(0);

        for(auto it = u_sequl_.rbegin();it != u_sequl_.rend();++it)
        {

            u1.block(0,index,6,1) = it->block(0,0,6,1);
            u2.block(0,index,6,1) = it->block(6,0,6,1);
            ++index;
            if(index == W)
            {
                break;//Only read W cols for u1 and u2.
            }
        }

        double T(0.0);
        Eigen::Vector3d tmp;

        //Detector for Imu1
        if(GLRT_Detector(u1,sigma2_a,sigma2_g,W))
        {
            status += 1;
        }
        if(GLRT_Detector(u2,sigma2_a,sigma2_g,W))
        {
            status += 2;
        }

        return status;


    }else{
        alogorithm_ = ZeroDetectorAlogorithm ::GLRT;
        return status;
    }


}

bool ZeroDetector::GLRT_Detector(Eigen::MatrixXd u, double sigma2_a, double sigma2_g, long W) {
    Eigen::Vector3d ya_m;
    double g = para_ptr_->gravity_;

    double T(0.0);
    Eigen::MatrixXd Tmatrix(1,1);

    for( int i(0);i< 3;++i)
    {
        ya_m(i) = u.block(i,0,1,u.cols()).mean();
    }

    Eigen::Vector3d tmp;

    for(int i(0);i< u.cols();++i)
    {

        tmp = u.block(0,i,3,1)-g * ya_m/ya_m.norm();
        double tt(0.0);

//        std::cout << " u block size : " << u.block(3,i,3,1).rows()<< std::endl;
//        std::cout << "tmp size :" << tmp.rows()<< std::endl;

        Tmatrix += u.block(3,i,3,1).transpose() * u.block(3,i,3,1) / sigma2_g + tmp.transpose() * tmp /sigma2_a;


    }

    if(Tmatrix.size() != 1)
    {
        MYERROR("Tmatrxi size is not equal to 1")
    }

    T = Tmatrix(0,0);

    T  = T / W;
    if(T < para_ptr_->gamma_)
    {
        return true;
    }else{
        return false;
    }

}
#endif //IMU_NAVIGATION_ZERO_DETECTER_H
