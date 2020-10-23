#include "odeint.hpp"

using namespace boost::numeric::odeint;

typedef std::vector<double> state_type;

const double sigma = 10.0;
const double R = 28.0;
const double b = 8.0 / 3.0;

// the system function can be a classical functions
void lorenz(state_type &x, state_type &dxdt, double t) {
  dxdt[0] = sigma * (x[1] - x[0]);
  dxdt[1] = R * x[0] - x[1] - x[0] * x[2];
  dxdt[2] = x[0] * x[1] - b * x[2];
}

// the system function can also be a functor
class lorenz_class {
 public:
  void operator()(state_type &x, state_type &dxdt, double t) {
    dxdt[0] = sigma * (x[1] - x[0]);
    dxdt[1] = R * x[0] - x[1] - x[0] * x[2];
    dxdt[2] = x[0] * x[1] - b * x[2];
  }
};

struct streaming_observer {
  std::ostream &m_out;
  streaming_observer(std::ostream &out) : m_out(out) {}

  void operator()(const state_type &x, double t) const {
    m_out << t;
    for (size_t i = 0; i < x.size(); ++i) m_out << "\t" << x[i];
    m_out << "\n";
  }
};

int main(int argc, char *argv[]) {
  state_type x(3);
  x[0] = x[1] = x[2] = 10.0;
  const double dt = 0.01;

//   integrate_const(runge_kutta4<state_type>(), lorenz, x, 0.0, 10.0, dt);
  integrate_const(runge_kutta4<state_type>(), lorenz, x, 0.0, 10.0, dt,
                  streaming_observer(std::cout));
  // or use the functor:
  //   integrate_const(runge_kutta4<state_type>(), lorenz_class(), x, 0.0, 10.0,
  //   dt);

  //   runge_kutta4<state_type> rk4;
  //   double t = 0.0;
  //   for (size_t i = 0; i < 1000; ++i, t += dt) {
  //     rk4.do_step(lorenz, x, t, dt);
  //   }

  std::cout << "result: ";
  for (const auto &val : x) std::cout << val << " , ";
  std::cout << std::endl;

  return 0;
}
