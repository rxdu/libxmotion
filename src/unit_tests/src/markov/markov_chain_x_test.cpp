/* 
 * markov_chain_test.cpp
 * 
 * Created on: Oct 29, 2018 10:09
 * Description: 
 *  [1] Example: https://en.wikipedia.org/wiki/Examples_of_Markov_chains
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include <stdio.h>
#include <vector>
#include <cmath>

#include "gtest/gtest.h"

#include "markov/markov_chain_x.hpp"

using namespace librav;

struct MarkovChainXTest : testing::Test
{
	MarkovChainXTest()
	{
		states.resize(2);
		trans.resize(2,2);
		
		states << 1.0, 0.0;
		trans << 0.9, 0.1, 0.5, 0.5;

		mchain = MarkovChainX<2>(states, trans);
	}

	MarkovChainX<2>::State states;
	MarkovChainX<2>::Transition trans;
	MarkovChainX<2> mchain;

	const double sigma = 0.0000001;
};

TEST_F(MarkovChainXTest, Propagate)
{
	auto res = mchain.CalculateStateAt(2);

	ASSERT_TRUE(res(0) - 0.86 < sigma && res(1) - 0.14 < sigma);

	ASSERT_TRUE(mchain.GetStateNumber() == 1);

	mchain.Propagate(2);

	ASSERT_TRUE(mchain[2](0) - 0.86 < sigma && mchain[2](1) - 0.14 < sigma);

	ASSERT_TRUE(mchain.GetStateNumber() == 3);
}

TEST_F(MarkovChainXTest, Model)
{
	mchain.Propagate(2);

	ASSERT_TRUE(mchain[0](0) - 1 < sigma && mchain[0](1) < sigma);
	ASSERT_TRUE(mchain[1](0) - 0.9 < sigma && mchain[1](1) - 0.1 < sigma);
	ASSERT_TRUE(mchain[2](0) - 0.86 < sigma && mchain[2](1) - 0.14 < sigma);
}