#include <iostream>
#include <memory>
#include <cmath>

#include "fastplot/mathgl_plot.hpp"
#include "fastplot/mathgl_field.hpp"

using namespace librav;

int main()
{
    long n = 50, m = 40;

    // prepare data
    Eigen::MatrixXd dend(50, 40);
    for (int i = 0; i < n; i++)
        for (int j = 0; j < m; j++)
        {
            double x = i / (n - 1.);
            double y = j / (m - 1.);
            int i0 = i + n * j;
            dend(i, j) = 0.6 * std::sin(2.0 * M_PI * x) * std::sin(3.0 * M_PI * y) + 0.4 * std::cos(3.0 * M_PI * x * y);
        }

    Eigen::VectorXd plotd(50);
    for (int i = 0; i < n; i++)
    {
        double xx = i / (n - 1.);
        plotd(i) = 0.7 * std::sin(2 * M_PI * xx) + 0.5 * std::cos(3 * M_PI * xx) + 0.2 * std::sin(M_PI * xx);
    }

    const int32_t x_size = 50;
    const int32_t y_size = 50;

    Eigen::VectorXf x(x_size);
    Eigen::VectorXf y(y_size);
    Eigen::MatrixXf z(x_size, y_size);

    for (int i = 0; i < x_size; i++)
    {
        x(i) = 1 + 0.1 * i;
    }

    for (int j = 0; j < y_size; j++)
    {
        y(j) = 1 + 0.1 * j;
    }

    for (int i = 0; i < x_size; i++)
        for (int j = 0; j < y_size; j++)
        {
            z(i, j) = 0.5 * (sinf(x(i)) + cosf(y(j)));
        }

    // MathGLSurf surf(z);
    // MathGLPlot::Run(&surf);
    // FastPlot::FieldSurf(x, y, z);
    // FastPlot::FieldMesh(x, y, z);
    FastPlot::FieldDens(x, y, z);

    return 0;
}
