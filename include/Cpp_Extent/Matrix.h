//
// Created by steve on 16-8-31.
//
#pragma once

#include <iostream>

#include <memory>

#include <cstring>

#include <sstream>

#include "MyError.h"



template <class T>
class Matrix
{
public:


	Matrix<T>(){}

	Matrix<T>(int rows,int cols)
	{
		buf_=new T[rows*cols];
		rows_ = rows;
		cols_ = cols;
	}

	Matrix<T>(const Matrix<T> &m_matrix)
	{
		this->rows_ = m_matrix.rows_;
		this->cols_ = m_matrix.cols_;

		buf_ = new T[rows_*cols_];
		//std::cout << "Before copy" << std::endl;
		memcpy(buf_, m_matrix.buf_, sizeof(T)*(rows_ )*(cols_ ));
		//std::cout << "After copy" << std::endl;
	}

	virtual ~Matrix<T>()
	{
		/*delete[] buf_;*/
	}

	/*Set the size of the matrix,will allocate memory for buf_ base on the size.*/
	void set_size(int rows,int cols)
	{
		if(rows_*cols_!=0)
		{
			delete[] buf_;//delete buf_ is buf_ is exist.
		}
		buf_ = new T[rows*cols];
		rows_=rows;
		cols_ = cols;
	}

	virtual void SetValue(T *value);

	/*Transport of a matrix*/
	Matrix<T> transport();

	void test(){
		std::cout<< "second value is :" << *(buf_ + 2 ) << std::endl;
	}

	/*Safe way to get the address of a element in the matrix,return a pointer.*/
	T* operator()(int a, int b);

	/*Matrix additional operator,two matrix must be same size.*/
	Matrix operator+(Matrix &b_matrix);

	/*Matrix add a single number,every elements in the matrix add this number.*/
	Matrix operator+(double num);

//	/*Copy a Matrix*/
//	Matrix operator=(Matrix &b_matrix);

	/*Typically Matrix multiply Matrix, first matrix's cols must equal to seconde matrix's rows.*/
	Matrix operator*(Matrix &b_matrix);

	/*Every elements in the matrix multiply a double value.*/
	Matrix operator*(double num);

	//TODO:Why not needn't redefine the operator =?
	//Matrix operator=(Matrix tmp_matrix) {
	//	return Matrix<T>(tmp_matrix) ;
	//}


	/*Return the cols_*/
	inline int GetCols()
	{
		return cols_;
	}

	/*Reture the rows_*/
	inline int GetRows()
	{
		return rows_;
	}




protected:

	int rows_=0;
	int cols_= 0;

	T* buf_;//Save the value of the matrix.

private:

};
template <class T>
void Matrix<T>::SetValue(T *value) {
	if(rows_*cols_  == 0)
	{
		std::cout << "ERROR,matrix size is:" << rows_ << " * " << cols_ << std::endl;
	}
	memcpy(buf_,value,sizeof(T)*rows_*cols_);

}

template <class T>
T* Matrix<T>::operator()(int a, int b)
{
	if(a<rows_ && b<cols_)
	{
		return (buf_ + a*cols_ + b);
	}else
	{
		MYERROR("Index is over the barder.")

		std::cout << "	rows,		cols	is " << rows_ << " , " << cols_ << std::endl;
		std::cout << "	index_x,	index_y is " << a << " , " << b << std::endl;

		return (buf_);
	}

}

template <class T>
Matrix<T> Matrix<T>::operator+(Matrix &b_matrix)
{
	if(cols_ == b_matrix.cols_ && rows_ == b_matrix.rows_)
	{
		Matrix<T> sum_matrix(cols_, rows_);
		//TODO:parallelization this function.
		for(int x(0);x<rows_;++x)
		{
			for(int y(0);y<cols_;++y)
			{
				*sum_matrix(x, y) = *b_matrix(x, y) + *(buf_ + x*cols_ +y);
			}
		}
		return sum_matrix;
	}
}

template <class T>
Matrix<T> Matrix<T>::operator+(double num)
{
	for(int x(0);x<rows_;++x)
	{
		for(int y(0);y<cols_;++y)
		{
			*(buf_ + x*cols_ + y) += num;
		}
	}
	return *this;
}

template <class T>
Matrix<T> Matrix<T>::operator*(Matrix& b_matrix)
{
	if(b_matrix.rows_ != this->rows_)
	{
		std::stringstream tmpss;
		tmpss << "Matrix multiply error,first matrix is " << this->rows_ << " x " << this->cols_ << \
			" and the second one is " << b_matrix.rows_ << " x " << b_matrix.cols_;
		MYERROR(tmpss.str().c_str())

		return Matrix<T>();
	}

	//TODO:This segement can be parallelization.

	//TODO:There are another methon to speed up the operator.
	Matrix<T> mul_matrix(this->rows_, b_matrix.cols_);

	for(int x(0);x<mul_matrix.rows_;++x)
	{
		for(int y(0);y<mul_matrix.cols_;++y)
		{
			*mul_matrix(x, y) = 0.0;
			for(int k(0);k<this->cols_;++k)
			{
				*mul_matrix(x, y) += *b_matrix(k, y)* (*(buf_+ x*cols_ +y));
			}
		}
	}
	return mul_matrix;
}

template <class T>
Matrix<T> Matrix<T>::operator*(double num)
{
	for (int x(0); x<rows_; ++x)
	{
		for (int y(0); y<cols_; ++y)
		{
			*(buf_ + x*cols_ + y) = *(buf_ + x*cols_ + y) * num;
		}
	}
	return *this;
}

template <class T>
Matrix<T> Matrix<T>::transport() {


	/*TODO:Use a function for use every element,
     * and then transport just need change a sigle flag value.
     * */
	if(cols_*rows_ == 0)
	{
		return Matrix<T>();
	}
	T *t (new T[rows_*cols_]);
	memcpy(t,buf_,sizeof(T)*rows_*cols_);
	int src_cols(cols_);
	cols_ = rows_;
	rows_=src_cols;
	//delete tmp;

	//transport
	for(int x(0);x<rows_;++x)
	{
		for(int y(0);y<cols_;++y)
		{
			*(buf_+x*cols_+y) = *(t+y*src_cols+x);
		}
	}

	delete [] t;
	return *this;

}

//template <class T>
//Matrix<T> Matrix<T>::operator=(Matrix &b_matrix) {
//	Matrix<T> tmp(b_matrix);
//	return tmp;
//}
//

