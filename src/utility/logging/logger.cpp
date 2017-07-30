/*
 * log_entry_helper.cpp
 *
 *  Created on: Oct 6, 2016
 *      Author: rdu
 */

#include "logging/logger.h"

#include <iostream>
#include <sstream>
#include <ctime>

using namespace librav;

namespace
{
	// reference: https://stackoverflow.com/questions/22318389/pass-system-date-and-time-as-a-filename-in-c
	std::string createLogFileName(std::string prefix, std::string path)
	{
		time_t t = time(0);   // get time now
		struct tm * now = localtime( & t );

		char buffer [80];
		strftime (buffer,80,"%Y%m%d%H%M%S",now);
		std::string time_stamp(buffer);

		std::string filename = path + "/" + prefix + "." + time_stamp + ".data";
		return filename;
	}
}

CtrlLogger::CtrlLogger(std::string log_name_prefix, std::string log_save_path):
				head_added_(false),
				log_name_prefix_(log_name_prefix),
				log_save_path_(log_save_path),
				item_counter_(0)
{
	// initialize logger
#ifdef ENABLE_LOGGING
	std::string filename = createLogFileName(log_name_prefix_, log_save_path_);
	spdlog::set_async_mode(256);
	logger_ = spdlog::basic_logger_mt("controller_logger", filename);
	logger_->set_pattern("%v");
#endif
}

CtrlLogger& CtrlLogger::InitLogger(std::string log_name_prefix, std::string log_save_path)
{
	static CtrlLogger instance(log_name_prefix, log_save_path);

	return instance;
}

void CtrlLogger::AddItemNameToEntryHead(std::string name)
{
	auto it = entry_ids_.find(name);

	if(it == entry_ids_.end()) {
		entry_names_[item_counter_] = name;
		entry_ids_[name] = item_counter_++;
	}
}

void CtrlLogger::AddItemDataToEntry(std::string item_name, std::string data_str)
{
	if(!head_added_)
	{
		std::cerr << "No heading for log entries has been added, data ignored!" << std::endl;
		return;
	}

	auto it = entry_ids_.find(item_name);

	if(it != entry_ids_.end())
		item_data_[(*it).second] = data_str;
	else
		std::cerr << "Failed to find data entry!" << std::endl;
}

// adding data using id is faster than using the name, validity of id is not checked
//	in this function.
void CtrlLogger::AddItemDataToEntry(uint64_t item_id, std::string data_str)
{
	if(!head_added_)
		return;

	item_data_[item_id] = data_str;
}

void CtrlLogger::AddItemDataToEntry(std::string item_name, double data)
{
	AddItemDataToEntry(item_name, std::to_string(data));
}

void CtrlLogger::AddItemDataToEntry(uint64_t item_id, double data)
{
	AddItemDataToEntry(item_id, std::to_string(data));
}

void CtrlLogger::PassEntryHeaderToLogger()
{
	if(item_counter_ == 0)
		return;

	std::string head_str;
	for(const auto& item:entry_names_)
		head_str += item.second + " , ";

	std::size_t found = head_str.rfind(" , ");
	if (found != std::string::npos)
		head_str.erase(found);

#ifdef ENABLE_LOGGING
	logger_->info(head_str);
#endif

	item_data_.resize(item_counter_);
	head_added_ = true;
}

void CtrlLogger::PassEntryDataToLogger()
{
	std::string log_entry;

	for(auto it = item_data_.begin(); it != item_data_.end(); it++)
	{
		std::string str;

		if((*it).empty())
			str = "0";
		else
			str = *it;

		if(it != item_data_.end() - 1)
			log_entry += str + " , ";
		else
			log_entry += str;
	}

#ifdef ENABLE_LOGGING
	if(!log_entry.empty())
		logger_->info(log_entry);
#endif
}

/******************************* Data Logger **********************************/

DataLogger::DataLogger(std::string log_name_prefix, std::string log_save_path):
				log_name_prefix_(log_name_prefix),
				log_save_path_(log_save_path)
{
	// initialize logger
#ifdef ENABLE_LOGGING
	std::string filename = createLogFileName(log_name_prefix_, log_save_path_);
	spdlog::set_async_mode(256);
	logger_ = spdlog::basic_logger_mt("data_logger", filename);
	logger_->set_pattern("%v");
#endif
}

DataLogger& DataLogger::InitLogger(std::string log_name_prefix, std::string log_save_path)
{
	static DataLogger instance(log_name_prefix, log_save_path);

	return instance;
}

