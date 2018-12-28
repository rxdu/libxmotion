/* 
 * ctrl_logger.hpp
 * 
 * Created on: Oct 6, 2016
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef CTRL_LOGGER_HPP
#define CTRL_LOGGER_HPP

#include <cstdint>
#include <string>
#include <vector>
#include <map>
#include <memory>
#include <atomic>
#include <sstream>

#include "spdlog/spdlog.h"
#include "logging/logging_utils.hpp"

namespace librav
{
class CtrlLogger
{
  public:
    static CtrlLogger &GetLogger(std::string log_name_prefix = "", std::string log_save_path = "");

    // basic functions
    void AddItemNameToEntryHead(std::string name);
    void AddItemDataToEntry(std::string item_name, std::string data_str);
    void AddItemDataToEntry(uint64_t item_id, std::string data_str);

    void PassEntryHeaderToLogger();
    void PassEntryDataToLogger();

    // extra helper functions
    void AddItemDataToEntry(std::string item_name, double data);
    void AddItemDataToEntry(uint64_t item_id, double data);

  private:
    std::shared_ptr<spdlog::logger> logger_;

    bool head_added_;
    std::string log_name_prefix_;
    std::string log_save_path_;

    std::map<uint64_t, std::string> entry_names_;
    std::map<std::string, uint64_t> entry_ids_;
    std::atomic<uint64_t> item_counter_;
    std::vector<std::string> item_data_;

    CtrlLogger() = delete;
    CtrlLogger(std::string log_name_prefix, std::string log_save_path);

    // prevent copy or assignment
    CtrlLogger(const CtrlLogger &) = delete;
    CtrlLogger &operator=(const CtrlLogger &) = delete;
};
} // namespace librav

#endif /* CTRL_LOGGER_HPP */
