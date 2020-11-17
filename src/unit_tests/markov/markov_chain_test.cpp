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

#include "markov/markov_chain.hpp"

using namespace rnav;

struct MarkovChainTest : testing::Test
{
	MarkovChainTest()
	{
		states << 1.0, 0.0;
		trans << 0.9, 0.1, 0.5, 0.5;

		mchain = MarkovChain<2>(states, trans);
	}

	MarkovChain<2>::State states;
	MarkovChain<2>::Transition trans;
	MarkovChain<2> mchain;

	const double sigma = 0.0000001;
};

TEST_F(MarkovChainTest, Propagate)
{
	auto res = mchain.CalculateStateAt(2);

	ASSERT_TRUE(std::abs(res(0) - 0.86) < sigma && std::abs(res(1) - 0.14) < sigma);

	ASSERT_TRUE(mchain.GetStateNumber() == 1);

	mchain.Propagate(2);

	ASSERT_TRUE(std::abs(mchain[2](0) - 0.86) < sigma && std::abs(mchain[2](1) - 0.14) < sigma);

	ASSERT_TRUE(mchain.GetStateNumber() == 3);
}

TEST_F(MarkovChainTest, Model)
{
	mchain.Propagate(2);

	ASSERT_TRUE(std::abs(mchain[0](0) - 1) < std::abs(sigma && mchain[0](1)) < sigma);
	ASSERT_TRUE(std::abs(mchain[1](0) - 0.9) < sigma && std::abs(mchain[1](1) - 0.1) < sigma);
	ASSERT_TRUE(std::abs(mchain[2](0) - 0.86) < sigma && std::abs(mchain[2](1) - 0.14) < sigma);
}