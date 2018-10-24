#include <iostream>
#include "geometry/polynomial.hpp"

using namespace librav;

int main()
{
    Eigen::VectorXd coeff;

    coeff = Eigen::VectorXd::Zero(3);
    coeff << 1.0, 2.0, 3.0;

    Polynomial<3> pl(coeff);

    // std::cout << "base coefficients: \n" << Polynomial::base_coefficients_ << std::endl;

    std::cout << "t=1.0, d = 0: " << pl.Evaluate(1.0) << std::endl;
    std::cout << "t=1.0, d = 0: " << pl.Evaluate(1.0, 0) << std::endl;
    std::cout << "t=1.0, d = 1: " << pl.Evaluate(1.0, 1) << std::endl;
    std::cout << "t=1.0, d = 2: " << pl.Evaluate(1.0, 2) << std::endl;
    std::cout << "t=1.0, d = 3: " << pl.Evaluate(1.0, 3) << std::endl;
    std::cout << "t=1.0, d = 4: " << pl.Evaluate(1.0, 4) << std::endl;

    return 0;
}