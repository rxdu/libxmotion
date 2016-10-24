/*
 * custom_loglevels.hpp
 *
 *  Created on: Oct 24, 2016
 *      Author: rdu
 */

#ifndef THIRD_PARTY_G3LOG_SRC_G3LOG_CUSTOM_LOGLEVELS_HPP_
#define THIRD_PARTY_G3LOG_SRC_G3LOG_CUSTOM_LOGLEVELS_HPP_

#include "g3log/loglevels.hpp"

// all values with a + 1 higher than their closest equivalet
// they could really have the same value as well.

const LEVELS DATA {INFO.value + 1, {"DATA"}};

#endif /* THIRD_PARTY_G3LOG_SRC_G3LOG_CUSTOM_LOGLEVELS_HPP_ */
