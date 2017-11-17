/* 
 * field_utils.h
 * 
 * Created on: Nov 17, 2017 15:48
 * Description: 
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */

#ifndef FIELD_UTILS_H
#define FIELD_UTILS_H

#include <memory>

#include "field/road_field.h"

namespace librav
{

namespace FieldUtils
{
    std::shared_ptr<RoadField> CreateTestRoadField();
    std::shared_ptr<RoadField> CreateDemoRoadField();
}

}

#endif /* FIELD_UTILS_H */
