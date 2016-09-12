#pragma once


#include <string>
#include <map>

#include <chrono>

#include "JsonCodert.h"

#include <stdio.h>
#include <stdlib.h>

#define ISDEBUG true
#define DEBUGSHOW(a) if(ISDEBUG) std::cout << a << std::endl;


enum ValueType
{
	EMPTY = -1,
	OBJECT = 0,
	INT = 1,
	DOUBLE = 2,
	STRING = 3,
	ARRAY = 4,
	ISNULL = 5,
	BOOL = 6,
};

class JsonObject:public JsonCoder
{
public:

	JsonObject(): int_value_(0), double_value_(0)
	{
	}

	JsonObject(std::string value_str);

	~JsonObject()
	{
	}

	ValueType value_type_ = EMPTY;

	std::map<std::string, JsonObject> map_;
	std::string str_value_;
	int int_value_;
	double double_value_;
	std::vector<JsonObject> array_value_;
	bool bool_value_;

	std::string str_debug_;


	std::string AsString() const;

	int AsInt() const;

	double AsDouble() const;

	JsonObject operator[](std::string key);

	JsonObject operator[](int index);


};

inline JsonObject::JsonObject(std::string value_str)
{
	str_debug_ = value_str;
	//std::cout << value_str << std::endl;
	if (-1 != value_str.find('{') && (value_str.find("{") < value_str.find('[') || value_str.find("[") < 0))
	{

		value_type_ = ValueType::OBJECT;
		//s_buf_ = value_str.substr(1,value_str.size()-1);

		s_buf_ = value_str;//

		ClearString();
		Decoder();

		std::cout << "OBJECT: " << value_str << std::endl;
		
		for (std::map<std::string, std::string>::iterator it = content_map_.begin();
		     it != content_map_.end(); ++it)
		{
			JsonObject tmp_value(it->second);

			map_.insert(std::map<std::string, JsonObject>::value_type(it->first.substr(1, it->first.size() - 2), tmp_value));
		}
	}
	else if (-1 != value_str.find('['))
	{
		value_type_ = ValueType::ARRAY;

		std::cout << "ARRAY: " << value_str << std::endl;

//		//TODO:rewrite this function.
//
//		bool is_array(false), is_object(false);
//		if(value_str.find('[',1)==1)
//		{
//			is_array = true;
//			std::cout << "is_array" << std::endl;
//		}
//		if(value_str.find('{',1) == 1)
//		{
//			is_object = true;
//			std::cout << "is_object" << std::endl;
//		}
//		int index(1);
//
//		while(true)
//		{
//			int last(0);
//			if(is_array)
//			{
//				last = StrInPairs('[', ']', value_str, index);
//			}
//			else if(is_object)
//			{
//				last = StrInPairs('{', '}', value_str, index);
//			}else
//			{
//
//				//TODO:Error in here.
//				last = value_str.find(',', index) - 1;
//				if(last < 0)
//				{
//					last = value_str.size() - 1;
//					std::cout << index << "   " << last << "  " << value_str.size() << "   " << value_str.substr(index, last) << std::endl;
//					array_value_.push_back(JsonObject(value_str.substr(index, last-index)));
//					break;
//				}
//			}
//			std::cout << index << "   " << last << "  " << value_str.size() << "   " <<  value_str.substr(index, last+1) << std::endl;
//			array_value_.push_back(JsonObject(value_str.substr(index, last-index)));
//			index = last + 1;
//
//		}

        //
		if(value_str[1] == '[')
		{
			std::cout << "elements in array is array." << std::endl;

            int l_index(1);
            int r_index(0);
            bool needbreak(false);
            while(true)
            {
                r_index = StrInPairs('[',']',value_str,l_index);
                if(r_index<0)
                {
                    r_index = value_str.size()-1;
                    needbreak = true;
                }

                array_value_.push_back(JsonObject(value_str.substr(l_index,r_index-l_index+1)));
                l_index = r_index+2;
                if(needbreak)
                {
                    break;
                }
                if(r_index == value_str.size()-2)
                {
                    break;
                }
            }


        }else if(value_str[1] == '{')
		{
			std::cout << "element in array is object" << std::endl;

            int l_index(1);
            int r_index(0);
            bool needbreak(false);
            while(true)
            {
                r_index = StrInPairs('{','}',value_str,l_index);
                if(r_index<0)
                {
                    r_index = value_str.size()-1;
                    needbreak = true;
                }

                array_value_.push_back(JsonObject(value_str.substr(l_index,r_index-l_index+1)));
                l_index = r_index+2;
                if(needbreak)
                {
                    break;
                }
                if(r_index == value_str.size()-2)
                {
                    break;
                }
            }


		}else if(value_str[1] == '\"'){
			std::cout << "element in array is string" << std::endl;

			//find "\",\"" in string.
            int l_index(1);
            int r_index(0);
            bool needbreak(false);
            if(value_str.find("\\\"",0)>-1)
            {
                std::cout << "NOTICE:There are some case not acheive in this code!(\"in string)" << std::endl;
                std::cerr << "NOTICE:There are some case not acheive in this code!(\"in string)" << std::endl;
            }
            while(true)
            {
                r_index = value_str.find("\",",l_index);
                //TODO:Add code to string include value "\"".
                if(r_index < 0 )
                {
                    r_index = value_str.size()-2;
                    needbreak = true;
                }
                array_value_.push_back(JsonObject(value_str.substr(l_index,r_index-l_index+1)));
                l_index = r_index +2;
                if(needbreak)
                {
                    break;
                }

            }
		}else{
			std::cout << "element in array is value." << std::endl;

			int l_index(1);
			int r_index(0);
            bool needbreak(false);
			while(true)
			{
				//if(r_index == 0)
				{
					r_index = value_str.find(",",l_index);
				}
				if(r_index <0)
				{

					r_index = value_str.size() -1;

                    needbreak = true;
				}
				array_value_.push_back(JsonObject(value_str.substr(l_index,r_index-l_index)));
                l_index = r_index+1;
                if(needbreak)
                {
                    break;
                }

			}

		}


	}
	else if (-1 != value_str.find("\""))
	{
		std::cout << "STRING: " << value_str << std::endl;
		//value type is string , save value delete "\"".
		value_type_ = ValueType::STRING;
		size_t begin_index = value_str.find("\"");
		size_t end_index(value_str.find_last_of("\""));


		str_value_ = value_str.substr(begin_index + 1, end_index - 1);
	}
	else if (-1 != value_str.find("true"))
	{
		std::cout << "BOOL: " << value_str << std::endl;
		value_type_ = ValueType::BOOL;

		bool_value_ = true;
	}
	else if (-1 != value_str.find("false"))
	{
		std::cout << "BOOL: " << value_str << std::endl;
		value_type_ = ValueType::BOOL;

		bool_value_ = false;
	}
	else if (-1 != value_str.find("null"))
	{
		std::cout << "NULL: " << value_str << std::endl;
		value_type_ = ValueType::ISNULL;
	}
	else if (-1 != value_str.find("."))
	{
		std::cout << "DOUBLE: " << value_str << std::endl;
		value_type_ = ValueType::DOUBLE;
		double_value_ = atof(value_str.c_str());
	}
	else if (-1 != value_str.find("e"))
	{
		std::cout << "DOUBLE: " << value_str << std::endl;
		value_type_ = ValueType::DOUBLE;
		double_value_ = atof(value_str.c_str());
	}
	else
	{
		std::cout << "INT: " << value_str << std::endl;
		value_type_ = ValueType::INT;
		int_value_ = atoi(value_str.c_str());
	}
}

inline std::string JsonObject::AsString() const
{
	if (value_type_ == ValueType::STRING)
	{
		return str_value_;
	}
	else
	{
		std::cout << str_debug_ << std::endl;
	}
}

inline int JsonObject::AsInt() const
{
	if (value_type_ == ValueType::INT)
	{
		return int_value_;
	}else if(value_type_ == ValueType::DOUBLE)
	{
		std::cout << "Convert from double to int will loss information!" << std::endl;
		return int(double_value_);
	}
	else
	{
		return 0;
	}
}

inline double JsonObject::AsDouble() const
{
	if (value_type_ == ValueType::DOUBLE)
	{
		return double_value_;
	}else if(value_type_ == ValueType::INT){
		return double(int_value_);
	} else
	{
		return 0;
	}
}

inline JsonObject JsonObject::operator[](std::string key)
{
	std::map<std::string, JsonObject>::iterator ite;
	ite = map_.find(key);
	if (ite != map_.end())
	{
		return ite->second;
	}
	else
	{
		std::cout << "There are not a key : " << key << " in json document." << std::endl;
		return JsonObject();
	}
}

inline JsonObject JsonObject::operator[](int index)
{
	if (value_type_ == ValueType::ARRAY)
	{
		if (index > array_value_.size())
		{
			std::cout << "Index : " << index << " is out of range 0-" << array_value_.size() << std::endl;
			return JsonObject();
		}
		else
		{
			return array_value_.at(index);
		}
	}
	else
	{
		std::cout << " This value is not a array!" << std::endl;
		return JsonObject();
	}
}



