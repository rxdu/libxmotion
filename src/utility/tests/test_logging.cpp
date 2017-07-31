/*
 * test_logging.cpp
 *
 *  Created on: Oct 6, 2016
 *      Author: rdu
 */

#include <memory>
#include "utility/logging/logger.h"

using namespace librav;

int main(int argc, char* argv[])
{
	//LoggingHelper helper("test_log", "/home/rdu");
//	CtrlLogger& helper = CtrlLogger::InitLogger("log2.txt", "/Users/rdu/Workspace/srcl/librav/build/bin");
//
//	helper.AddItemNameToEntryHead("x1");
//	helper.AddItemNameToEntryHead("x2");
//	helper.AddItemNameToEntryHead("x3");
//	helper.AddItemNameToEntryHead("x4");
//
//	helper.PassEntryHeaderToLogger();
//
//	for(int i = 0; i < 500; i++) {
//		helper.AddItemDataToEntry("x1", i+1);
//		helper.AddItemDataToEntry("x2", i+2);
//		helper.AddItemDataToEntry("x3", i+3);
//		helper.AddItemDataToEntry("x4", i+4);
//
//		helper.PassEntryDataToLogger();
//	}

	CsvLogger logger("logtest", "/Users/rdu/Workspace");

	for(int i = 0; i < 500; i++)
	{
		GlobalCsvLogger::GetLogger("csv", "/Users/rdu/Workspace").LogData(21,34,56);
		logger.LogData(i+1, i+0.5, i*100.5);
	}

	EventLogger logger2("eventlog", "/Users/rdu/Workspace");

	for(int i = 0; i < 500; i++)
			logger2.LogEvent("event1", "event2", i*100.5);
}
