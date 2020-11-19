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

using namespace rnav;

struct MarkovChainXTest : testing::Test
{
	MarkovChainXTest()
	{
		states.resize(1, 2);
		trans.resize(2, 2);

		states << 1.0, 0.0;
		trans << 0.9, 0.1, 0.5, 0.5;

		mchain = MarkovChainX<2>(states, trans);
	}

	MarkovChainX<2>::State states;
	MarkovChainX<2>::Transition trans;
	MarkovChainX<2> mchain;

	const double sigma = 0.0000001;
};

TEST_F(MarkovChainXTest, StateTransition)
{
	MarkovChainX<2>::State res = mchain.CalculateStateAt(2);
	// std::cout << "result: \n" << res(0) << " , " << res(1) << std::endl;

	ASSERT_TRUE((std::abs(res(0) - 0.86) < sigma) && (std::abs(res(1) - 0.14) < sigma));

	ASSERT_TRUE(mchain.GetStateNumber() == 1);
}

TEST_F(MarkovChainXTest, ModelPropagate)
{
	mchain.Propagate(2);

	ASSERT_TRUE(std::abs(mchain[0](0) - 1) < sigma && std::abs(mchain[0](1)) < sigma);
	ASSERT_TRUE(std::abs(mchain[1](0) - 0.9) < sigma && std::abs(mchain[1](1) - 0.1) < sigma);
	ASSERT_TRUE(std::abs(mchain[2](0) - 0.86) < sigma && std::abs(mchain[2](1) - 0.14) < sigma);
}