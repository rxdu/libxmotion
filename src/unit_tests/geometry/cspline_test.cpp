/* 
 * cspline_test.cpp
 * 
 * Created on: Oct 13, 2018 22:32
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include <stdio.h>
#include <vector>
#include <cmath>

#include "gtest/gtest.h"

#include "geometry/cspline.hpp"

using namespace ivnav;

struct CSplineTest : testing::Test
{
	CSplineTest()
	{
		for (int i = 0; i < 10; i++)
			knots.emplace_back(i + 0.5 * std::sin(i), i + std::cos(i * i));
	}

	std::vector<CSpline::Knot> knots;
};

bool IsCorrect(std::vector<double> &check_val)
{
	const double sigma = 0.00001;

	// value verified
	std::vector<double> true_val{1, 1.32968, 1.54101, 1.51608, 1.30493, 1.37874, 1.99057, 2.7567, 4.30892, 5.94598, 6.399, 6.08017, 5.87446, 6.24475, 6.89748, 7.46532, 7.85726, 8.39968, 9.33011};

	for (int i = 0; i < true_val.size(); ++i)
	{
		if (true_val[i] - check_val[i] > sigma)
			return false;
	}
	return true;
}

TEST_F(CSplineTest, Evaluate)
{
	CSpline spline(knots);

	std::vector<double> check_val;
	for (double x = knots.front().x; x < knots.back().x; x += 0.5)
		check_val.push_back(spline.Evaluate(x));

	ASSERT_TRUE(IsCorrect(check_val)) << "Cubic spline evaluation value not correct";
}

TEST_F(CSplineTest, DelayedConstruct)
{
	CSpline spline;

	spline.Interpolate(knots);
	std::vector<double> check_val;
	for (double x = knots.front().x; x < knots.back().x; x += 0.5)
		check_val.push_back(spline.Evaluate(x));

	ASSERT_TRUE(IsCorrect(check_val)) << "Failed to interpolate after default construction";
}

TEST_F(CSplineTest, Big5)
{
	CSpline spline(knots);

	// copy constructor
	CSpline spline2(spline);
	std::vector<double> check_val2;
	for (double x = knots.front().x; x < knots.back().x; x += 0.5)
		check_val2.push_back(spline2.Evaluate(x));
	ASSERT_TRUE(IsCorrect(check_val2)) << "Copy constructor not correct";

	// assignment operator
	CSpline spline3;
	spline3 = spline;
	std::vector<double> check_val3;
	for (double x = knots.front().x; x < knots.back().x; x += 0.5)
		check_val3.push_back(spline3.Evaluate(x));
	ASSERT_TRUE(IsCorrect(check_val3)) << "Assignment operator not correct";

	// move constructor
	CSpline spline4{CSpline(knots)};
	std::vector<double> check_val4;
	for (double x = knots.front().x; x < knots.back().x; x += 0.5)
		check_val4.push_back(spline4.Evaluate(x));
	ASSERT_TRUE(IsCorrect(check_val4)) << "Move constructor not correct";

	// move assignment operator
	CSpline spline5;
	spline5 = CSpline(knots);
	std::vector<double> check_val5;
	for (double x = knots.front().x; x < knots.back().x; x += 0.5)
		check_val5.push_back(spline5.Evaluate(x));
	ASSERT_TRUE(IsCorrect(check_val5)) << "Move assignment operator not correct";
}
