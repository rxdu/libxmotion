/*
 * utils_log.h
 *
 *  Created on: Mar 11, 2016
 *      Author: rdu
 */

#ifndef LIBRARY_LIBG3LOG_UTILS_LOG_UTILS_LOG_H_
#define LIBRARY_LIBG3LOG_UTILS_LOG_UTILS_LOG_H_

#include <string>
#include <sstream>

namespace srcl_ctrl {

class UtilsLog {
public:
	UtilsLog();
	~UtilsLog();

private:
	static std::string log_entry;

public:
	static void AppendLogMsgStr(std::string msg_str);
	static void AppendLogMsgTuple3f(float t1, float t2, float t3);
	static void AppendLogMsgTuple4f(float t1, float t2, float t3, float t4);

	static void EmptyLogMsgEntry();
	static std::string GetLogEntry();
};

}

#endif /* LIBRARY_LIBG3LOG_UTILS_LOG_UTILS_LOG_H_ */
