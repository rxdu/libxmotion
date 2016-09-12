/*
 * start_tests.cpp
 *
 *  Created on: Jul 20, 2016
 *      Author: rdu
 */

#include "gtest/gtest.h"


int main(int ac, char* av[])
{
  testing::InitGoogleTest(&ac, av);
  return RUN_ALL_TESTS();
}
