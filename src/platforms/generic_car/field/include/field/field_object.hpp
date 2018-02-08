/* 
 * field_object.hpp
 * 
 * Created on: Jan 24, 2018 12:55
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef FIELD_OBJECT_HPP
#define FIELD_OBJECT_HPP

namespace librav
{
struct FieldObject
{
  FieldObject(double posx = 0, double posy = 0, double velx = 0, double vely = 0, double sc = 1.0) : pos_x(posx),
                                                                                                  pos_y(posy),
                                                                                                  vel_x(velx),
                                                                                                  vel_y(vely),
                                                                                                  scale(sc){};

  double pos_x;
  double pos_y;
  double vel_x;
  double vel_y;
  double scale;
};
}

#endif /* FIELD_OBJECT_HPP */
