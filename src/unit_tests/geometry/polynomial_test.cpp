/* 
 * polynomial_test.cpp
 * 
 * Created on: Oct 22, 2018 02:37
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include <stdio.h>
#include <vector>
#include <cmath>

#include "gtest/gtest.h"

#include "geometry/polynomial.hpp"

using namespace autodrive;

struct PolynomialTest : testing::Test
{
    PolynomialTest()
    {
        coeff = Eigen::VectorXd::Zero(5);
        coeff << 1.0, 2.0, 3.0, 4.0, 5.0;

        val = 0.0;
        derivative = 0;
    }

    Eigen::VectorXd coeff;
    double val;
    double derivative;
};

TEST_F(PolynomialTest, Evaluate)
{
    Polynomial<5> pl(coeff);

    val = 1.0;
    ASSERT_TRUE(pl.Evaluate(val) == 1.0 + 2.0 * val + 3.0 * val * val + 4.0 * val * val * val + 5.0 * val * val * val * val)
        << "Incorrect value at t = 1.0";
    val = 1.45;
    ASSERT_TRUE(pl.Evaluate(val) == 1.0 + 2.0 * val + 3.0 * val * val + 4.0 * val * val * val + 5.0 * val * val * val * val)
        << "Incorrect value at t = 1.45";
    val = 2.0;
    ASSERT_TRUE(pl.Evaluate(val) == 1.0 + 2.0 * val + 3.0 * val * val + 4.0 * val * val * val + 5.0 * val * val * val * val)
        << "Incorrect value at t = 2.0";
}

TEST_F(PolynomialTest, EvaluateDerivative)
{
    Polynomial<5> pl(coeff);
    double delta = 0.0000001;

    derivative = 0;
    val = 1.45;
    ASSERT_TRUE(pl.Evaluate(val, derivative) - (1.0 + 2.0 * val + 3.0 * val * val + 4.0 * val * val * val + 5.0 * val * val * val * val) < delta)
        << "Incorrect derivative value at t = 1.45, d = 0";

    derivative = 1;
    val = 1.0;
    ASSERT_TRUE(pl.Evaluate(val, derivative) - (2.0 + 6.0 * val + 12.0 * val * val + 20.0 * val * val * val) < delta)
        << "Incorrect derivative value at t = 1.0, d = 1";
    val = 1.45;
    ASSERT_TRUE(pl.Evaluate(val, derivative) - (2.0 + 6.0 * val + 12.0 * val * val + 20.0 * val * val * val) < delta)
        << "Incorrect derivative value at t = 1.45, d = 1";

    derivative = 2;
    val = 1.0;
    ASSERT_TRUE(pl.Evaluate(val, derivative) - (6.0 + 24.0 * val + 60.0 * val * val) < delta)
        << "Incorrect derivative value at t = 1.0, d = 2";
    val = 1.45;
    ASSERT_TRUE(pl.Evaluate(val, derivative) - (6.0 + 24.0 * val + 60.0 * val * val) < delta)
        << "Incorrect derivative value at t = 1.45, d = 2";

    derivative = 3;
    val = 1.0;
    ASSERT_TRUE(pl.Evaluate(val, derivative) - (24.0 + 120.0 * val) < delta)
        << "Incorrect derivative value at t = 1.0, d = 3";
    val = 1.45;
    ASSERT_TRUE(pl.Evaluate(val, derivative) - (24.0 + 120.0 * val) < delta)
        << "Incorrect derivative value at t = 1.45, d = 3";

    derivative = 4;
    val = 1.0;
    ASSERT_TRUE(pl.Evaluate(val, derivative) == 120.0) << "Incorrect derivative value at t = 1.0, d = 4";
    val = 1.45;
    ASSERT_TRUE(pl.Evaluate(val, derivative) == 120.0) << "Incorrect derivative value at t = 1.45, d = 4";

    derivative = 5;
    val = 1.0;
    ASSERT_TRUE(pl.Evaluate(1.0, 5) == 0.0) << "Incorrect derivative value at t = 1.0, d = 5";
    val = 1.45;
    ASSERT_TRUE(pl.Evaluate(1.0, 5) == 0.0) << "Incorrect derivative value at t = 1.45, d = 5";

    derivative = 6;
    val = 1.0;
    ASSERT_TRUE(pl.Evaluate(1.0, 5) == 0.0) << "Incorrect derivative value at t = 1.0, d = 6";
    val = 1.45;
    ASSERT_TRUE(pl.Evaluate(1.0, 5) == 0.0) << "Incorrect derivative value at t = 1.45, d = 6";
}
