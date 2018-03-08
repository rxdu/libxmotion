/*
 * information_field.hpp
 *
 * Created on: Nov 17, 2017 11:18
 * Description: This is the field for planning, which may consist of multiple
 *          layers of sub-fields describing different types of collisions
 *
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */

#ifndef INFORMATION_FIELD_HPP
#define INFORMATION_FIELD_HPP

#include <memory>
#include <unordered_map>
#include <vector>
#include <cstdint>

#include "field/threat_basis.hpp"
#include "field/field_object.hpp"

namespace librav
{

class InformationField : public ScalarField
{
public:
  InformationField(int64_t size_x, int64_t size_y);

  void LoadEgoCenteredBasisPattern(int32_t radius_step_size, double angle_step_size, double sigma = 40);
  void LoadUniformBasisPattern(int32_t x_step, int32_t y_step, double sigma = 40);

  void AddThreatBasisField(int32_t id, std::shared_ptr<ThreatBasis> tfield);
  std::shared_ptr<ThreatBasis> GetThreatBasisField(int32_t id);
  void RemoveThreatBasisField(int32_t id);

  void UpdateInformationField();
  void UpdateInformationField(std::vector<FieldObject> objects);

private:
  std::unordered_map<int32_t, std::shared_ptr<ThreatBasis>> threat_basis_fields_;
  std::unordered_map<int32_t, double> basis_coeffs_;
};
}

#endif /* INFORMATION_FIELD_HPP */
