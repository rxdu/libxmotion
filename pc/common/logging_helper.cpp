/*
 * log_entry_helper.cpp
 *
 *  Created on: Oct 6, 2016
 *      Author: rdu
 */

#include "common/logging_helper.h"

#include <iostream>
#include <ctime>

using namespace srcl_ctrl;

LoggingHelper::LoggingHelper(std::string log_name_suffix, std::string log_save_path):
		head_added_(false),
		log_name_prefix_(log_name_suffix),
		log_save_path_(log_save_path),
		item_counter_(0)
{
	std::string file_name;

	// ref: https://stackoverflow.com/questions/22318389/pass-system-date-and-time-as-a-filename-in-c
	time_t t = time(0);   // get time now
	struct tm * now = localtime(&t);
	char buffer[80];
	strftime (buffer,80,"%F-%H-%M-%S-",now);
	std::string fdate(buffer);

	if(log_save_path.back() == '/')
		file_name = log_save_path + fdate + log_name_suffix;
	else
		file_name = log_save_path + "/"  + fdate + log_name_suffix;

	// initialize logger
#ifdef ENABLE_LOGGING
	spdlog::set_async_mode(1024*8);
	logger_ = spdlog::rotating_logger_mt("file_logger", file_name, 1024 * 1024 * 5, 3);
	logger_->set_pattern("%R:%S.%e | %l | %v");
#endif
}

LoggingHelper& LoggingHelper::GetInstance(std::string log_name_prefix , std::string log_save_path)
{
	static LoggingHelper instance(log_name_prefix, log_save_path);

	return instance;
}

LoggingHelper::~LoggingHelper()
{
	spdlog::drop_all();
}

void LoggingHelper::AddItemNameToEntryHead(std::string name)
{
	auto it = entry_ids_.find(name);

	if(it == entry_ids_.end()) {
		entry_names_[item_counter_] = name;
		entry_ids_[name] = item_counter_++;
	}
}

void LoggingHelper::AddItemDataToEntry(std::string item_name, std::string data_str)
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
void LoggingHelper::AddItemDataToEntry(uint64_t item_id, std::string data_str)
{
	if(!head_added_)
		return;

	item_data_[item_id] = data_str;
}

void LoggingHelper::AddItemDataToEntry(std::string item_name, double data)
{
	AddItemDataToEntry(item_name, std::to_string(data));
}

void LoggingHelper::AddItemDataToEntry(uint64_t item_id, double data)
{
	AddItemDataToEntry(item_id, std::to_string(data));
}

void LoggingHelper::PassEntryHeaderToLogger()
{
	if(item_counter_ == 0)
		return;

	std::string head_str;
	for(const auto& item:entry_names_)
		head_str += item.second + ", ";

	std::size_t found = head_str.rfind(", ");
	if (found != std::string::npos)
		head_str.erase(found);

#ifdef ENABLE_LOGGING
	logger_->info(head_str);
#endif

	item_data_.resize(item_counter_);
	head_added_ = true;
}

void LoggingHelper::PassEntryDataToLogger()
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

void LoggingHelper::LogStringMsg(std::string msg)
{
#ifdef ENABLE_LOGGING
	logger_->info(msg);
#endif
}
