/* 
 * event_logger.cpp
 * 
 * Created on: Oct 6, 2016
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */ 

#include "logging/event_logger.hpp"

#include <iostream>
#include <sstream>
#include <ctime>

using namespace librav;

EventLogger::EventLogger(std::string log_name_prefix, std::string log_save_path)
{
	// initialize logger
#ifdef ENABLE_LOGGING
	std::string filename = CreateLogFileName(log_name_prefix, log_save_path);
	spdlog::set_async_mode(256);
	logger_ = spdlog::basic_logger_mt("event_logger_" + log_name_prefix, filename);
	logger_->set_pattern("%v");
#endif
}
