/* 
 * speed_profile.hpp
 * 
 * Created on: Nov 14, 2018 08:29
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef SPEED_PROFILE_HPP
#define SPEED_PROFILE_HPP

namespace ivnav
{
class SpeedProfileIF
{
  public:
    virtual ~SpeedProfileIF() = default;

    virtual double GetSpeed(double s) = 0;
    virtual double GetAccel(double s) = 0;

    virtual SpeedProfileIF *GetCopy() = 0;
};

class ConstSpeedProfile : public SpeedProfileIF
{
  public:
    ConstSpeedProfile(double speed) : speed_(speed) {}

    double GetSpeed(double s) override { return speed_; };
    double GetAccel(double s) override { return 0; };

    ConstSpeedProfile *GetCopy() override { return new ConstSpeedProfile(speed_); }

  private:
    double speed_;
};
} // namespace ivnav

#endif /* SPEED_PROFILE_HPP */
