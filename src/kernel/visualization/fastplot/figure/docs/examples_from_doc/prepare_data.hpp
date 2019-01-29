# include <Eigen/Dense>

typedef Eigen::VectorXd vec_t;
void prepare_data (vec_t& x, vec_t& y, const unsigned& fun, const unsigned& nevals = 500) {
  x = vec_t::LinSpaced(nevals, 1, 5);
  switch(fun) {
    case 0:
      y = x; break;
    case 1:
      y = x.cwiseProduct(x); break;
    case 3:
      y = (-x).array().exp().matrix(); break;
    case 4:
      y = ((-x).array().exp()*(5*x).array().sin()).matrix(); break;
    default:
      y = (2.5*x).array().cos().matrix(); 
  }
}
