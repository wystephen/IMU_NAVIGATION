#include <iostream>

#include <thread>
#include <algorithm>


#include <random>

#include "FileReader.h"
#include "JsonCodert.h"
#include "JsonObject.h"
#include "CSVReader.h"

int main()
{
	//JsonObject job;
	//JsonObject t = job["aaa"];
	
	CSVReader fr(std::string("test.csv"));

	fr.test1();



	
	getchar();

	return 0;

}
