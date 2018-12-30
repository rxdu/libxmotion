/* 
 * event_logger.hpp
 * 
 * Created on: Oct 6, 2016
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef EVENT_LOGGER_HPP
#define EVENT_LOGGER_HPP

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
class EventLogger
{
  public:
    EventLogger() = delete;
    EventLogger(std::string log_name_prefix, std::string log_save_path);

    // prevent copy or assignment
    EventLogger(const EventLogger &) = delete;
    EventLogger &operator=(const EventLogger &) = delete;

    // basic functions
    template <class... T>
    void LogEvent(const T &... value)
    {
#ifdef ENABLE_LOGGING
        std::ostringstream o;
        build_string(o, value...);

        std::string data = o.str();
        data.pop_back();

        logger_->info(o.str());
#else
        return;
#endif
    }

    // logger wrapper functions
    void LogInfo(std::string msg)
    {
#ifdef ENABLE_LOGGING
        logger_->info(msg);
#endif
    }

    void LogWarn(std::string msg)
    {
#ifdef ENABLE_LOGGING
        logger_->warn(msg);
#endif
    }

    void LogFatal(std::string msg)
    {
#ifdef ENABLE_LOGGING
        logger_->critical(msg);
#endif
    }

  private:
    std::shared_ptr<spdlog::logger> logger_;

    inline void build_string(std::ostream &o) { (void)o; }
    template <class First, class... Rest>
    inline void build_string(std::ostream &o, const First &value, const Rest &... rest)
    {
        o << value;
        o << ",";
        build_string(o, rest...);
    }
};
} // namespace librav

#endif /* EVENT_LOGGER_HPP */
