#include <iostream>
#include "polynomial/polynomial.hpp"

using namespace librav;

int main()
{
    Eigen::VectorXd coeff;

    coeff = Eigen::VectorXd::Zero(3);
    coeff << 1.0, 2.0, 3.0;

    Polynomial pl(coeff);

    // std::cout << "base coefficients: \n" << Polynomial::base_coefficients_ << std::endl;

    std::cout << "t=1.0, d = 0: " << pl.evaluate(1.0, 0) << std::endl;
    std::cout << "t=1.0, d = 1: " << pl.evaluate(1.0, 1) << std::endl;

    return 0;
}