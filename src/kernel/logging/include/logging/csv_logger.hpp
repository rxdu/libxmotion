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

#include "logging/details/spdlog_headers.hpp"
#include "logging/details/logging_utils.hpp"

namespace librav
{
class CsvLogger
{
  public:
    CsvLogger() = delete;
    CsvLogger(std::string log_name_prefix, std::string log_save_path);

    // prevent copy or assignment
    CsvLogger(const CsvLogger &) = delete;
    CsvLogger &operator=(const CsvLogger &) = delete;

    // basic functions
    void AddItemNameToEntryHead(std::string name);
    void PassEntryHeaderToLogger();

    template <class... T>
    void LogData(const T &... value)
    {
#ifdef ENABLE_LOGGING
        std::ostringstream o;
        build_string(o, value...);

        std::string data = o.str();
        data.pop_back();

        logger_->info(data);
#else
        return;
#endif
    }

  private:
    std::shared_ptr<spdlog::logger> logger_;

    std::string log_name_prefix_;
    std::string log_save_path_;

    std::map<uint64_t, std::string> entry_names_;

    inline void build_string(std::ostream &o) { (void)o; }
    template <class First, class... Rest>
    inline void build_string(std::ostream &o, const First &value, const Rest &... rest)
    {
        o << std::to_string(value) + ",";
        build_string(o, rest...);
    }
};

class GlobalCsvLogger : public CsvLogger
{
  public:
    static GlobalCsvLogger &GetLogger(std::string log_name_prefix = "", std::string log_save_path = "");

  private:
    GlobalCsvLogger() = delete;
    GlobalCsvLogger(std::string prefix, std::string path) : CsvLogger(prefix, path){};

    // prevent copy or assignment
    GlobalCsvLogger(const GlobalCsvLogger &) = delete;
    GlobalCsvLogger &operator=(const GlobalCsvLogger &) = delete;
};
} // namespace librav

#endif /* CSV_LOGGER_HPP */
