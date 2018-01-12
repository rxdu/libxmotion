/* 
 * scalar_field.h
 * 
 * Created on: Nov 17, 2017 17:58
 * Description: 
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */

#ifndef SCALAR_FIELD_H
#define SCALAR_FIELD_H

#include "field/field_base.h"
#include "lcmtypes/librav.hpp"

namespace librav
{

class ScalarField : public FieldBase<double>
{
public:
  ScalarField() = delete;
  ScalarField(int64_t size_x, int64_t size_y);
  ~ScalarField() = default;

  void SetValueAtLocation(int64_t x, int64_t y, double val);
  double GetValueAtLocation(int64_t x, int64_t y);

  librav_lcm_msgs::ScalarField_t GenerateScalarFieldMsg();
  FieldMatrix GenerateFieldMatrix(double x_start, double x_step, double y_start, double y_step);

private:
  FieldMatrix field_matrix_;
};
}

#endif /* SCALAR_FIELD_H */
