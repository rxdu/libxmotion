/* 
 * csv_logger.cpp
 * 
 * Created on: Oct 6, 2016
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */ 

#include "logging/csv_logger.hpp"

#include <iostream>
#include <sstream>
#include <ctime>

namespace librav
{
CsvLogger::CsvLogger(std::string log_name_prefix, std::string log_save_path) : log_name_prefix_(log_name_prefix),
                                                                               log_save_path_(log_save_path)
{
    // initialize logger
#ifdef ENABLE_LOGGING
    std::string filename = CreateLogFileName(log_name_prefix_, log_save_path_);
    spdlog::set_async_mode(256);
    logger_ = spdlog::basic_logger_mt("csv_logger_" + log_name_prefix_, filename);
    logger_->set_pattern("%v");
#endif
}

GlobalCsvLogger &GlobalCsvLogger::GetLogger(std::string log_name_prefix, std::string log_save_path)
{
    static GlobalCsvLogger instance(log_name_prefix, log_save_path);

    return instance;
}
} // namespace librav
