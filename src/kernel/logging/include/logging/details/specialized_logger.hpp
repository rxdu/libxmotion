/* 
 * specialized_logger.hpp
 * 
 * Created on: Dec 30, 2018 00:54
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef SPECIALIZED_LOGGER_HPP
#define SPECIALIZED_LOGGER_HPP

#include <memory>
#include <string>
#include <sstream>
#include <ostream>

#include "logging/details/spdlog_headers.hpp"
#include "logging/details/logging_utils.hpp"

namespace librav
{
class SpecializedLogger
{
  public:
    SpecializedLogger() = delete;
    SpecializedLogger(std::string logfile_prefix, std::string logfile_path) : logfile_prefix_(logfile_prefix),
                                                                              logfile_path_(logfile_path)
    {
#ifdef ENABLE_LOGGING
        // initialize logger
        std::string filename = CreateLogFileName(logfile_prefix_, logfile_path_);
        spdlog::installCrashHandlerOnce();
        logger_ = spdlog::basic_logger_mt<spdlog::async_factory>(filename, filename);
        logger_->set_pattern("%v");
#endif
    }

    // non-copyable
    SpecializedLogger(const SpecializedLogger &) = delete;
    SpecializedLogger &operator=(const SpecializedLogger &) = delete;

  protected:
    std::shared_ptr<spdlog::logger> logger_;

    std::string logfile_prefix_;
    std::string logfile_path_;

    // helper functions
    inline void build_string(std::ostream &o) { (void)o; }
    template <class First, class... Rest>
    inline void build_string(std::ostream &o, const First &value, const Rest &... rest)
    {
        o << std::to_string(value) + ",";
        build_string(o, rest...);
    }
};
} // namespace librav

#endif /* SPECIALIZED_LOGGER_HPP */
