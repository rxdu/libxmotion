/* 
 * csv_logger.hpp
 * 
 * Created on: Oct 6, 2016
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef CSV_LOGGER_HPP
#define CSV_LOGGER_HPP

#include <cstdint>
#include <string>
#include <vector>
#include <map>
#include <memory>
#include <atomic>
#include <sstream>

#include "logging/details/specialized_logger.hpp"

namespace xmotion {
class CsvLogger : public SpecializedLogger {
 public:
  CsvLogger(std::string logfile_prefix, std::string logfile_path) : SpecializedLogger(logfile_prefix, logfile_path) {}

  // non-copyable
  CsvLogger(const CsvLogger &) = delete;
  CsvLogger &operator=(const CsvLogger &) = delete;

  template<class... T>
  void LogData(const T &... value) {
#ifdef ENABLE_LOGGING
    std::ostringstream o;
    build_string(o, value...);

    std::string data = o.str();
    data.pop_back();

    logger_->info(data);
#endif
  }
};

class GlobalCsvLogger : public CsvLogger {
  GlobalCsvLogger(std::string prefix, std::string path) : CsvLogger(prefix, path) {};

 public:
  static GlobalCsvLogger &GetLogger(std::string logfile_prefix = "", std::string logfile_path = "") {
    static GlobalCsvLogger instance(logfile_prefix, logfile_path);
    return instance;
  }

  // prevent copy or assignment
  GlobalCsvLogger(const GlobalCsvLogger &) = delete;
  GlobalCsvLogger &operator=(const GlobalCsvLogger &) = delete;
};
} // namespace xmotion

#endif /* CSV_LOGGER_HPP */
