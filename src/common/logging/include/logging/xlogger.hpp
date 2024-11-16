/*
 * xlogger.hpp
 *
 * Created on 4/20/24 10:27 AM
 * Description:
 *
 * Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef XMOTION_SRC_UTILITIES_LOGGING_INCLUDE_LOGGING_XLOGGER_HPP_
#define XMOTION_SRC_UTILITIES_LOGGING_INCLUDE_LOGGING_XLOGGER_HPP_

#include <sstream>

#include "logging/details/default_logger.hpp"

/*
 * level: 0 - TRACE, 1 - DEBUG, 2 - INFO, 3 - WARN,
 *        4 - ERROR, 5 - FATAL, 6 - OFF
 */
#ifdef ENABLE_LOGGING
#define XLOG_LEVEL(level)                                  \
  {                                                        \
    xmotion::DefaultLogger::GetInstance()->SetLoggerLevel( \
        static_cast<xmotion::LogLevel>(level));            \
  }
#define XLOG_GET_LEVEL() \
  { xmotion::DefaultLogger::GetInstance()->GetLoggerLevel(); }
#define XLOG_TRACE(...)                                                   \
  {                                                                       \
    xmotion::DefaultLogger::GetInstance()->Log(xmotion::LogLevel::kTrace, \
                                               __VA_ARGS__);              \
  }
#define XLOG_DEBUG(...)                                                   \
  {                                                                       \
    xmotion::DefaultLogger::GetInstance()->Log(xmotion::LogLevel::kDebug, \
                                               __VA_ARGS__);              \
  }
#define XLOG_INFO(...)                                                   \
  {                                                                      \
    xmotion::DefaultLogger::GetInstance()->Log(xmotion::LogLevel::kInfo, \
                                               __VA_ARGS__);             \
  }
#define XLOG_WARN(...)                                                   \
  {                                                                      \
    xmotion::DefaultLogger::GetInstance()->Log(xmotion::LogLevel::kWarn, \
                                               __VA_ARGS__);             \
  }
#define XLOG_ERROR(...)                                                   \
  {                                                                       \
    xmotion::DefaultLogger::GetInstance()->Log(xmotion::LogLevel::kError, \
                                               __VA_ARGS__);              \
  }
#define XLOG_FATAL(...)                                                   \
  {                                                                       \
    xmotion::DefaultLogger::GetInstance()->Log(xmotion::LogLevel::kFatal, \
                                               __VA_ARGS__);              \
  }
#define XLOG_TRACE_STREAM(...)                                            \
  {                                                                       \
    std::stringstream stream_ss;                                          \
    stream_ss << __VA_ARGS__;                                             \
    xmotion::DefaultLogger::GetInstance()->Log(xmotion::LogLevel::kTrace, \
                                               stream_ss.str().c_str());  \
  }
#define XLOG_DEBUG_STREAM(...)                                            \
  {                                                                       \
    std::stringstream stream_ss;                                          \
    stream_ss << __VA_ARGS__;                                             \
    xmotion::DefaultLogger::GetInstance()->Log(xmotion::LogLevel::kDebug, \
                                               stream_ss.str().c_str());  \
  }
#define XLOG_INFO_STREAM(...)                                            \
  {                                                                      \
    std::stringstream stream_ss;                                         \
    stream_ss << __VA_ARGS__;                                            \
    xmotion::DefaultLogger::GetInstance()->Log(xmotion::LogLevel::kInfo, \
                                               stream_ss.str().c_str()); \
  }
#define XLOG_WARN_STREAM(...)                                            \
  {                                                                      \
    std::stringstream stream_ss;                                         \
    stream_ss << __VA_ARGS__;                                            \
    xmotion::DefaultLogger::GetInstance()->Log(xmotion::LogLevel::kWarn, \
                                               stream_ss.str().c_str()); \
  }
#define XLOG_ERROR_STREAM(...)                                            \
  {                                                                       \
    std::stringstream stream_ss;                                          \
    stream_ss << __VA_ARGS__;                                             \
    xmotion::DefaultLogger::GetInstance()->Log(xmotion::LogLevel::kError, \
                                               stream_ss.str().c_str());  \
  }
#define XLOG_FATAL_STREAM(...)                                            \
  {                                                                       \
    std::stringstream stream_ss;                                          \
    stream_ss << __VA_ARGS__;                                             \
    xmotion::DefaultLogger::GetInstance()->Log(xmotion::LogLevel::kFatal, \
                                               stream_ss.str().c_str());  \
  }
#else
#define XLOG_LEVEL(level) \
  {}
#define XLOG_GET_LEVEL() \
  {}
#define XLOG_TRACE(...) \
  {}
#define XLOG_DEBUG(...) \
  {}
#define XLOG_INFO(...) \
  {}
#define XLOG_WARN(...) \
  {}
#define XLOG_ERROR(...) \
  {}
#define XLOG_FATAL(...) \
  {}
#define XLOG_TRACE_STREAM(...) \
  {}
#define XLOG_DEBUG_STREAM(...) \
  {}
#define XLOG_INFO_STREAM(...) \
  {}
#define XLOG_WARN_STREAM(...) \
  {}
#define XLOG_ERROR_STREAM(...) \
  {}
#define XLOG_FATAL_STREAM(...) \
  {}
#endif

#endif  // XMOTION_SRC_UTILITIES_LOGGING_INCLUDE_LOGGING_XLOGGER_HPP_
