/* 
 * scalar_field.hpp
 * 
 * Created on: Nov 17, 2017 17:58
 * Description: 
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */

#ifndef SCALAR_FIELD_HPP
#define SCALAR_FIELD_HPP

#include "field/field_base.hpp"
#include "lcmtypes/librav.hpp"

namespace librav
{

struct ScalarFieldMatrix
{
  Eigen::VectorXd x;
  Eigen::VectorXd y;
  Eigen::MatrixXd z;
};

class ScalarField : public FieldBase<double>
{
public:
  ScalarField() = delete;
  ScalarField(int64_t size_x, int64_t size_y);
  ~ScalarField() = default;

  void SetValueAtCoordinate(int64_t x, int64_t y, double val);
  double GetValueAtCoordinate(int64_t x, int64_t y);

  // for visualization/debugging purpose
  void PrintField(bool pretty = false) const;
  librav_lcm_msgs::ScalarField_t GenerateScalarFieldMsg();
  ScalarFieldMatrix GenerateFieldMatrix(double x_start, double x_step, double y_start, double y_step, bool normalize_z = false);

protected:
  ScalarFieldMatrix field_matrix_;
};
}

#endif /* SCALAR_FIELD_HPP */
