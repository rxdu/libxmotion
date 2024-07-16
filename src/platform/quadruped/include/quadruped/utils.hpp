/*
 * @file utils.hpp
 * @date 7/13/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef QUADRUPED_MOTION_UTILS_HPP
#define QUADRUPED_MOTION_UTILS_HPP

#include <optional>

#include "quadruped/system_config.hpp"
#include "quadruped/control_context.hpp"

namespace xmotion {
namespace Utils {
/**
 * @brief result = target + delta, if: min <= target + delta <= max
 * @param target
 * @param delta
 * @param min
 * @param max
 * @return
 */
double UpdateRangeLimitedValue(double target, double delta, double min,
                               double max);

/**
 * @brief result = target + delta, if:
 *  min_change <= target + delta - initial <= max_change
 * @param target
 * @param initial
 * @param delta
 * @param min_change
 * @param max_change
 * @return
 */
double UpdateChangeLimitedValue(double target, double initial, double delta,
                                double min_change, double max_change);

double DegreeToRadian(double degree);
double RadianToDegree(double radian);

std::optional<HidSettings::KeyFunction> PollKeyFunction(
    ControlContext &context, HidEventHandler::KeyboardEventType type);
}  // namespace Utils
}  // namespace xmotion

#endif  // QUADRUPED_MOTION_UTILS_HPP
