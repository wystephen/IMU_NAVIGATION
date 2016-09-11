//
// Created by steve on 16-9-11.
//

#include "../Cpp_Extent/CSVReader.h"

#include "../Cpp_Extent/Matrix.h"

#include "../Cpp_Extent/MyError.h"


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
    ImuDataPreProcess(std::string DataDir):csvReaderu1(DataDir +"/u1.csv"),
                        csvReaderu2(DataDir +"/u2.csv"),
                        csvReaderzupt1(DataDir +"/zupt1.csv"),
                        csvReaderzupt2(DataDir +"/zupt2.csv")
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
    }


protected:
    CSVReader csvReaderu1,csvReaderu2,csvReaderzupt1,csvReaderzupt2;

    Matrix<DataType> u1,u2,zupt1,zupt2;




private:




};


#endif //IMU_NAVIGATION_IMU_DATA_PREPROCESS_H
