/*
 * typesize_unittest.cpp
 *
 *  Created on: Jul 21, 2016
 *      Author: rdu
 */

#include <stdio.h>
#include <vector>

#include "gtest/gtest.h"

#include "graph/graph.h"
#include "graph/bds_base.h"
#include "graph/bds_example.h"

using namespace srcl_ctrl;

TEST(TypeTest, TypeSize)
{
	std::cout << "Size of BDSExample: " << sizeof(BDSExample) << std::endl;
}

//TEST(TypeTest, TypeValidity)
//{
//	BDSBase<int>* a;
//}
