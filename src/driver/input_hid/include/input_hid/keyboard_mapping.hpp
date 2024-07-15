/*
 * @file keyboard_mapping.hpp
 * @date 7/15/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef XMOTION_KEYBOARD_MAPPING_HPP
#define XMOTION_KEYBOARD_MAPPING_HPP

#include <string>
#include <unordered_map>

#include "interface/driver/keyboard_interface.hpp"

namespace xmotion {
struct KeyboardMapping {
  static std::unordered_map<int, KeyboardCode> keycode_map;
  static std::unordered_map<KeyboardCode, std::string> keyname_map;
  static std::unordered_map<std::string, KeyboardCode> keyname_to_keycode_map;

  static std::string GetKeyName(KeyboardCode code);
  static KeyboardCode GetKeyCode(const std::string& key_name);
};
}  // namespace xmotion

#endif  // XMOTION_KEYBOARD_MAPPING_HPP
