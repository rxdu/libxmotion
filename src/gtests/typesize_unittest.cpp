/*
 * typesize_unittest.cpp
 *
 *  Created on: Jul 21, 2016
 *      Author: rdu
 */

#include <stdio.h>
#include <vector>

#include "gtest/gtest.h"

#include "planning/graph/graph.h"
#include "planning/graph/bds_base.h"
#include "planning/graph/bds_example.h"

using namespace librav;

TEST(TypeTest, TypeSize)
{
	std::cout << "Size of BDSExample: " << sizeof(BDSExample) << std::endl;
}

//TEST(TypeTest, TypeValidity)
//{
//	BDSBase<int>* a;
//}
