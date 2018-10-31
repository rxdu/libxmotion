#include <iostream>

#include <eigen3/Eigen/Dense>

int main()
{
    // Eigen::Matrix<double, 60, 60> matrix;
    Eigen::MatrixXd matrix;
    matrix.setZero(600,600);
    matrix << 1;

    return 0;
}