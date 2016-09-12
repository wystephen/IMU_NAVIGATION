//
// Created by steve on 16-8-30.
//

#include <iostream>
#include "FileReader.h"

#include "Matrix.h"

#include "MyError.h"

class CSVReader : public FileReader {
public:

    CSVReader() = default;

    ~CSVReader()
    {

    }

    CSVReader(std::string file_name) : FileReader(file_name) {
        LoadData();
    }


    int rows_ = 0;
    int cols_ = 0;

    void test1();

    Matrix<double> GetMatrix();
    Matrix<double> m_;
private:

protected:
    bool MatSize();



    bool LoadData();

};

bool CSVReader::MatSize() {

    int line_number(0);
    int index(0);
    int line_elements(0);

    while (true) {
        if (line_number == 0) {
            if (file_buf_[index] == ',') {
                ++line_elements;
            }
        }
        if (file_buf_[index] == '\n') {
            ++line_number;
        }

        if (index > file_size_ - 1) {

            break;
        } else {
            ++index;
        }
    }

    rows_ = line_number;
    cols_ = line_elements + 1;


    return false;
}

//Just for test.
void CSVReader::test1() {

    std::cout << *m_(3,2) << std::endl;
    std::cout << *m_.transport()(2,3) << std::endl;
}

inline Matrix<double> CSVReader::GetMatrix()
{
    return m_;
}

bool CSVReader::LoadData() {

    if(rows_*cols_ == 0)
    {
        MatSize();
        m_.set_size(rows_,cols_);
    }
    if(file_size_ == 0)
    {
        MYERROR("FILE IS EMPTY!")
    }
    std::string tmp_str(file_buf_);

    int the_row(0), the_col(0);

    int l_index(0), r_index(0);

    for(int index(0);index < tmp_str.size();++index)
    {
        if(tmp_str[index]==',' || tmp_str[index] == '\n')
        {
            r_index = index;
            *m_(the_row, the_col) = atof(tmp_str.substr(l_index, r_index - l_index).c_str());
            if(tmp_str[index] == '\n')
            {
                ++the_row;
                the_col = 0;
            }else
            {
                ++the_col;
            }
            if(the_row == rows_-1 && the_col == cols_ -1)
            {
                *m_(the_row, the_col ) = atof(tmp_str.substr(r_index + 1, file_size_ - r_index - 1).c_str());
                break;
            }
            l_index = r_index + 1;
        }
    }

    return false;
}

