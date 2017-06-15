/*
 * test_logging.cpp
 *
 *  Created on: Oct 6, 2016
 *      Author: rdu
 */

#include <memory>
#include "common/logging_helper.h"

using namespace srcl_ctrl;

int main(int argc, char* argv[])
{
	//LoggingHelper helper("test_log", "/home/rdu");
	LoggingHelper& helper = LoggingHelper::GetInstance("log2.txt", "/Users/rdu/Workspace/srcl/srcl_ctrl/build2/bin");

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
}


