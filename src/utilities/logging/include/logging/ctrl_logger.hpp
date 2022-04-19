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

#include "logging/details/specialized_logger.hpp"

namespace robosw {
class CtrlLogger : public SpecializedLogger {
 public:
  static CtrlLogger &GetLogger(std::string logfile_prefix = "", std::string logfile_path = "");

  // basic functions
  void AddItemNameToEntryHead(std::string name);
  void AddItemDataToEntry(std::string item_name, std::string data_str);
  void AddItemDataToEntry(uint64_t item_id, std::string data_str);

  // extra helper functions
  void AddItemDataToEntry(std::string item_name, double data);
  void AddItemDataToEntry(uint64_t item_id, double data);

  // functions that invoke logger calls
  void PassEntryHeaderToLogger();
  void PassEntryDataToLogger();

 private:
  bool head_added_;
  std::map<uint64_t, std::string> entry_names_;
  std::map<std::string, uint64_t> entry_ids_;
  std::atomic<uint64_t> item_counter_;
  std::vector<std::string> item_data_;

  CtrlLogger(std::string log_name_prefix, std::string log_save_path);

  // non-copyable
  CtrlLogger(const CtrlLogger &) = delete;
  CtrlLogger &operator=(const CtrlLogger &) = delete;
};
} // namespace robosw

#endif /* CTRL_LOGGER_HPP */
