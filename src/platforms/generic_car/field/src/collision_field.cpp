/*
 * collision_field.cpp
 *
 * Created on: Nov 17, 2017 11:18
 * Description:
 *
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */

#include "field/collision_field.hpp"
#include "field/threat_distribution.hpp"

#include <cmath>
#include <iostream>

#include <Eigen/Dense>

using namespace librav;

CollisionField::CollisionField(int64_t size_x, int64_t size_y)
    : ScalarField(size_x, size_y)
{
}

void CollisionField::LoadEgoCenteredBasisPattern(int32_t radius_step_size, double angle_step_size, double sigma)
{
  int32_t basis_id = 0;

  // first add one at center
  auto center_basis = std::make_shared<ThreatBasis>(size_x_, size_y_);
  auto center_raw_coordinate = ConvertToRawCoordinate(0, 0);
  GaussianPositionThreat center_gau(center_raw_coordinate.GetX(), center_raw_coordinate.GetY(), sigma, sigma);
  center_basis->SetCenterPosition(center_raw_coordinate.GetX(), center_raw_coordinate.GetY());
  center_basis->SetThreatBasisDistribution(center_gau);
  threat_basis_fields_.emplace(std::make_pair(basis_id, center_basis));
  basis_coeffs_.emplace(std::make_pair(basis_id, 1.0));
  ++basis_id;

  // then add more around center
  for (double r = radius_step_size; r < SizeX() && r < SizeY(); r += radius_step_size)
    for (double angle = 0; angle < M_PI * 2; angle += angle_step_size)
    {
      int64_t center_x = static_cast<int64_t>(r * cos(angle));
      int64_t center_y = static_cast<int64_t>(r * sin(angle));
      auto raw_coordinate = ConvertToRawCoordinate(center_x, center_y);

      if (raw_coordinate.GetX() >= 0 && raw_coordinate.GetX() < SizeX() &&
          raw_coordinate.GetY() >= 0 && raw_coordinate.GetY() < SizeY())
      {
        auto basis = std::make_shared<ThreatBasis>(size_x_, size_y_);
        GaussianPositionThreat gau(raw_coordinate.GetX(), raw_coordinate.GetY(), sigma, sigma);
        basis->SetCenterPosition(raw_coordinate.GetX(), raw_coordinate.GetY());
        basis->SetThreatBasisDistribution(gau);
        threat_basis_fields_.emplace(std::make_pair(basis_id, basis));
        basis_coeffs_.emplace(std::make_pair(basis_id, 1.0));
        ++basis_id;
      }
    }

  std::cout << "Basis pattern loaded, " << basis_id << " basis field added" << std::endl;
}

void CollisionField::LoadUniformBasisPattern(int32_t x_step, int32_t y_step, double sigma)
{
  int32_t basis_id = 0;
  int32_t bound_x = SizeX() / x_step - 1;
  int32_t bound_y = SizeY() / y_step - 1;
  for (double x = 0 - bound_x * x_step; x < SizeX(); x += x_step)
    for (double y = 0 - bound_y * y_step; y < SizeY(); y += y_step)
    {
      auto raw_coordinate = ConvertToRawCoordinate(x, y);
      if (raw_coordinate.GetX() >= 0 && raw_coordinate.GetX() < SizeX() &&
          raw_coordinate.GetY() >= 0 && raw_coordinate.GetY() < SizeY())
      {
        auto basis = std::make_shared<ThreatBasis>(size_x_, size_y_);
        GaussianPositionThreat gau(raw_coordinate.GetX(), raw_coordinate.GetY(), sigma, sigma);
        basis->SetCenterPosition(raw_coordinate.GetX(), raw_coordinate.GetY());
        basis->SetThreatBasisDistribution(gau);
        threat_basis_fields_.emplace(std::make_pair(basis_id, basis));
        basis_coeffs_.emplace(std::make_pair(basis_id, 1.0));
        ++basis_id;
      }
    }

  std::cout << "Basis pattern loaded, " << basis_id << " basis field added" << std::endl;
}

void CollisionField::AddThreatBasisField(int32_t id, std::shared_ptr<ThreatBasis> tfield)
{
  assert(tfield->SizeX() == this->size_x_ && tfield->SizeY() == this->size_y_);

  threat_basis_fields_.emplace(std::make_pair(id, tfield));
  basis_coeffs_.emplace(std::make_pair(id, 1.0));
}

std::shared_ptr<ThreatBasis> CollisionField::GetThreatBasisField(int32_t id)
{
  assert(threat_basis_fields_.find(id) != threat_basis_fields_.end());

  return threat_basis_fields_[id];
}

void CollisionField::RemoveThreatBasisField(int32_t id)
{
  threat_basis_fields_.erase(id);
}

void CollisionField::UpdateCollisionField()
{
  for (int64_t i = 0; i < size_x_; ++i)
    for (int64_t j = 0; j < size_y_; ++j)
    {
      double threat_val = 0;
      for (const auto &tfd : threat_basis_fields_)
        threat_val += tfd.second->GetValueAtCoordinate(i, j) * basis_coeffs_[tfd.first];
      SetValueAtCoordinate(i, j, threat_val);
    }
}

void CollisionField::UpdateCollisionField(std::vector<FieldObject> objects)
{
  double dist_threshold = 20;

  for (const auto &obj : objects)
    for (const auto &ftd : threat_basis_fields_)
    {
      Eigen::Vector2d basis_pos_vec(ftd.second->center_pos_x_, ftd.second->center_pos_y_);
      Eigen::Vector2d obj_pos_vec(obj.pos_x, obj.pos_y);
      Eigen::Vector2d dist_vec = basis_pos_vec - obj_pos_vec;
      double distance = dist_vec.norm();
      basis_coeffs_[ftd.first] *= 1.0 / distance;
    }

  UpdateCollisionField();
}