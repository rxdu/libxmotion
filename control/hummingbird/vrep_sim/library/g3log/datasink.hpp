/*
 * datasink.hpp
 *
 *  Created on: Jul 24, 2015
 *      Author: rdu
 */

#pragma once

#include <string>
#include <memory>

#include "g3log/logmessage.hpp"

namespace g3 {

   class DataSink {
   public:
	   DataSink(const std::string &log_prefix, const std::string &log_directory);
      virtual ~DataSink();

      void fileWrite(LogMessageMover message);
      std::string changeLogFile(const std::string &directory);
      std::string fileName();


   private:
      std::string _log_file_with_path;
      std::string _log_prefix_backup; // needed in case of future log file changes of directory
      std::unique_ptr<std::ofstream> _outptr;

      void addLogFileHeader();
      std::ofstream &filestream() {
         return *(_outptr.get());
      }


      DataSink &operator=(const DataSink &) = delete;
      DataSink(const DataSink &other) = delete;

   };
} // g3
