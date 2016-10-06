/*
 * log_entry_helper.cpp
 *
 *  Created on: Oct 6, 2016
 *      Author: rdu
 */

#include <iostream>
#include <logging/logging_helper.h>

using namespace srcl_ctrl;
using namespace g3;

LoggingHelper::LoggingHelper():
		head_added_(false),
		log_name_prefix_("g3log"),
		log_save_path_("/home/logs"),
		item_counter_(0)
{
	// initialize logger
	log_worker_ = g3::LogWorker::createLogWorker();
	file_sink_hd_ = log_worker_->addDefaultLogger(log_name_prefix_, log_save_path_);
	g3::initializeLogging(log_worker_.get());
}

LoggingHelper::LoggingHelper(std::string log_name_prefix, std::string log_save_path):
		head_added_(false),
		log_name_prefix_(log_name_prefix),
		log_save_path_(log_save_path),
		item_counter_(0)
{
	// initialize logger
	log_worker_ = g3::LogWorker::createLogWorker();
	file_sink_hd_ = log_worker_->addDefaultLogger(log_name_prefix_, log_save_path_);
	g3::initializeLogging(log_worker_.get());
}

LoggingHelper::~LoggingHelper()
{

}

void LoggingHelper::AddItemNameToEntryHead(std::string name)
{
	auto it = entry_indexes_.find(name);

	if(it == entry_indexes_.end()) {
		entry_names_[item_counter_] = name;
		entry_indexes_[name] = item_counter_++;
	}
}

void LoggingHelper::AddItemDataToEntry(std::string item_name, std::string data_str)
{
	if(!head_added_)
	{
		std::cerr << "No heading for log entries has been added, data ignored!" << std::endl;
		return;
	}

	auto it = entry_indexes_.find(item_name);

	if(it != entry_indexes_.end())
		item_data_[(*it).second] = data_str;
}

// adding data using id is faster than using the name, validity of id is not checked
//	in this functions.
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

	LOG(INFO) << head_str;

	item_data_.resize(item_counter_);
	head_added_ = true;
}

void LoggingHelper::PassEntryDataToLogger()
{
	for(const auto& dt:item_data_)
	{
		if(dt.empty())
			log_entry_ += " 0 , ";
		else
			log_entry_ += dt + " , ";
	}
	std::size_t found = log_entry_.rfind(", ");
	if (found != std::string::npos)
		log_entry_.erase(found);

	if(!log_entry_.empty())
		LOG(INFO) << log_entry_;

	log_entry_.clear();
}
