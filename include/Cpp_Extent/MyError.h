#pragma once

#include <cstring>
#include <iostream>

#define  MYERROR(err_info) std::cout<<"ERROR:" << __FILE__ << ": In function " << __func__ << "at line " << __LINE__ << std::endl\
<< "COMPLIED ON " << __DATE__ << " at " << __TIME__ <<  std::endl\
<< "Info: " << err_info <<  std::endl << std::endl;


//inline MyError::MyError(std::string err_info)
//{
//	std::cout << "ERROR:" << __FILE__ << ": In function " << __func__ << "at line " << __LINE__ << std::endl;
//	std::cout << "COMPLIED ON" << __DATE__ << " at " << __TIME__ <<  std::endl;
//	std::cout << "Info: " << "a" <<  std::endl;
//	std::cout << std::endl
//		;std::cout << std::endl;
//}
