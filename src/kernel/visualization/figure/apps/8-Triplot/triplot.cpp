# include <Eigen/Dense>
# include <figure/figure.hpp>

int main() {
Eigen::VectorXd x(10), y(10);
x << 1.0,0.60,0.12,0.81,0.63,0.09,0.27,0.54,0.95,0.96;
y << 0.15,0.97,0.95,0.48,0.80,0.14,0.42,0.91,0.79,0.95;
	
// specify triangles through indices of their vertices
Eigen::MatrixXi T(11,3);
T << 7, 1, 2,   5, 6, 2,    4, 1, 7,    6, 7, 2,
     6, 4, 7,   6, 5, 0,    3, 6, 0,    8, 4, 3, 
     3, 4, 6,   8, 1, 4,    9, 1, 8;

mgl::Figure fig;
//fig.triplot(T, x, y, "bF"); // fully colored blue mesh
fig.triplot(T, x, y, "b?").label("Mesh"); // drawing triangulation with numbers
fig.plot(x, y, " *r"); // mark vertices
fig.legend();
fig.save("mesh");

return 0;
}
