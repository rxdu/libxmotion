/*
 * utils_log.cpp
 *
 *  Created on: Mar 11, 2016
 *      Author: rdu
 */

#include "utils_log.h"

using namespace srcl_ctrl;

std::string UtilsLog::log_entry = "";

UtilsLog::UtilsLog()
{
}

UtilsLog::~UtilsLog()
{
}

void UtilsLog::AppendLogMsgTuple3f(float t1, float t2, float t3)
{
	std::ostringstream log_stream;

	if(!UtilsLog::log_entry.empty())
		log_stream << " , ";

	log_stream << std::to_string(t1);
	log_stream << " , ";
	log_stream << std::to_string(t2);
	log_stream << " , ";
	log_stream << std::to_string(t3);
	log_entry += log_stream.str();
}

void UtilsLog::AppendLogMsgTuple4f(float t1, float t2, float t3, float t4)
{
	std::ostringstream log_stream;

	if(!UtilsLog::log_entry.empty())
		log_stream << " , ";

	log_stream << std::to_string(t1);
	log_stream << " , ";
	log_stream << std::to_string(t2);
	log_stream << " , ";
	log_stream << std::to_string(t3);
	log_stream << " , ";
	log_stream << std::to_string(t4);
	log_entry += log_stream.str();
}

void UtilsLog::AppendLogMsgStr(std::string msg_str)
{
	std::ostringstream log_stream;

	if(!UtilsLog::log_entry.empty())
		log_stream << " , ";
	log_stream << msg_str;
	log_entry += log_stream.str();
}

void UtilsLog::EmptyLogMsgEntry()
{
	UtilsLog::log_entry.clear();
}

std::string UtilsLog::GetLogEntry()
{
	return log_entry;
}
