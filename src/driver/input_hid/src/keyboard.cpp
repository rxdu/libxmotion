/*
 * keyboard.cpp
 *
 * Created on 7/5/24 10:17 PM
 * Description:
 *
 * Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "input_hid/keyboard.hpp"

#include <linux/input.h>
#include <unistd.h>
#include <fcntl.h>

#include "logging/xlogger.hpp"

namespace xmotion {
namespace {
static std::unordered_map<int, KeyboardCode> keycode_map = {
    {KEY_A, KeyboardCode::kA},
    {KEY_B, KeyboardCode::kB},
    {KEY_C, KeyboardCode::kC},
    {KEY_D, KeyboardCode::kD},
    {KEY_E, KeyboardCode::kE},
    {KEY_F, KeyboardCode::kF},
    {KEY_G, KeyboardCode::kG},
    {KEY_H, KeyboardCode::kH},
    {KEY_I, KeyboardCode::kI},
    {KEY_J, KeyboardCode::kJ},
    {KEY_K, KeyboardCode::kK},
    {KEY_L, KeyboardCode::kL},
    {KEY_M, KeyboardCode::kM},
    {KEY_N, KeyboardCode::kN},
    {KEY_O, KeyboardCode::kO},
    {KEY_P, KeyboardCode::kP},
    {KEY_Q, KeyboardCode::kQ},
    {KEY_R, KeyboardCode::kR},
    {KEY_S, KeyboardCode::kS},
    {KEY_T, KeyboardCode::kT},
    {KEY_U, KeyboardCode::kU},
    {KEY_V, KeyboardCode::kV},
    {KEY_W, KeyboardCode::kW},
    {KEY_X, KeyboardCode::kX},
    {KEY_Y, KeyboardCode::kY},
    {KEY_Z, KeyboardCode::kZ},
    {KEY_ESC, KeyboardCode::kEsc},
    {KEY_1, KeyboardCode::kNum1},
    {KEY_2, KeyboardCode::kNum2},
    {KEY_3, KeyboardCode::kNum3},
    {KEY_4, KeyboardCode::kNum4},
    {KEY_5, KeyboardCode::kNum5},
    {KEY_6, KeyboardCode::kNum6},
    {KEY_7, KeyboardCode::kNum7},
    {KEY_8, KeyboardCode::kNum8},
    {KEY_9, KeyboardCode::kNum9},
    {KEY_0, KeyboardCode::kNum0},
    {KEY_MINUS, KeyboardCode::kMinus},
    {KEY_EQUAL, KeyboardCode::kEqual},
    {KEY_BACKSPACE, KeyboardCode::kBackspace},
    {KEY_TAB, KeyboardCode::kTab},
    {KEY_ENTER, KeyboardCode::kEnter},
    {KEY_SPACE, KeyboardCode::kSpace},
    {KEY_CAPSLOCK, KeyboardCode::kCapsLock},
    {KEY_F1, KeyboardCode::kF1},
    {KEY_F2, KeyboardCode::kF2},
    {KEY_F3, KeyboardCode::kF3},
    {KEY_F4, KeyboardCode::kF4},
    {KEY_F5, KeyboardCode::kF5},
    {KEY_F6, KeyboardCode::kF6},
    {KEY_F7, KeyboardCode::kF7},
    {KEY_F8, KeyboardCode::kF8},
    {KEY_F9, KeyboardCode::kF9},
    {KEY_F10, KeyboardCode::kF10},
    {KEY_F11, KeyboardCode::kF11},
    {KEY_F12, KeyboardCode::kF12},
    {KEY_LEFTCTRL, KeyboardCode::kLeftCtrl},
    {KEY_RIGHTCTRL, KeyboardCode::kRightCtrl},
    {KEY_LEFTSHIFT, KeyboardCode::kLeftShift},
    {KEY_RIGHTSHIFT, KeyboardCode::kRightShift},
    {KEY_LEFTALT, KeyboardCode::kLeftAlt},
    {KEY_RIGHTALT, KeyboardCode::kRightAlt},
    {KEY_LEFT, KeyboardCode::kLeft},
    {KEY_RIGHT, KeyboardCode::kRight},
    {KEY_UP, KeyboardCode::kUp},
    {KEY_DOWN, KeyboardCode::kDown},
    {KEY_HOME, KeyboardCode::kHome},
    {KEY_END, KeyboardCode::kEnd},
    {KEY_PAGEUP, KeyboardCode::kPageUp},
    {KEY_PAGEDOWN, KeyboardCode::kPageDown}};

static std::unordered_map<KeyboardCode, std::string> keyname_map = {
    {KeyboardCode::kA, "A"},
    {KeyboardCode::kB, "B"},
    {KeyboardCode::kC, "C"},
    {KeyboardCode::kD, "D"},
    {KeyboardCode::kE, "E"},
    {KeyboardCode::kF, "F"},
    {KeyboardCode::kG, "G"},
    {KeyboardCode::kH, "H"},
    {KeyboardCode::kI, "I"},
    {KeyboardCode::kJ, "J"},
    {KeyboardCode::kK, "K"},
    {KeyboardCode::kL, "L"},
    {KeyboardCode::kM, "M"},
    {KeyboardCode::kN, "N"},
    {KeyboardCode::kO, "O"},
    {KeyboardCode::kP, "P"},
    {KeyboardCode::kQ, "Q"},
    {KeyboardCode::kR, "R"},
    {KeyboardCode::kS, "S"},
    {KeyboardCode::kT, "T"},
    {KeyboardCode::kU, "U"},
    {KeyboardCode::kV, "V"},
    {KeyboardCode::kW, "W"},
    {KeyboardCode::kX, "X"},
    {KeyboardCode::kY, "Y"},
    {KeyboardCode::kZ, "Z"},
    {KeyboardCode::kEsc, "Esc"},
    {KeyboardCode::kNum1, "1"},
    {KeyboardCode::kNum2, "2"},
    {KeyboardCode::kNum3, "3"},
    {KeyboardCode::kNum4, "4"},
    {KeyboardCode::kNum5, "5"},
    {KeyboardCode::kNum6, "6"},
    {KeyboardCode::kNum7, "7"},
    {KeyboardCode::kNum8, "8"},
    {KeyboardCode::kNum9, "9"},
    {KeyboardCode::kNum0, "0"},
    {KeyboardCode::kMinus, "-"},
    {KeyboardCode::kEqual, "="},
    {KeyboardCode::kBackspace, "Backspace"},
    {KeyboardCode::kTab, "Tab"},
    {KeyboardCode::kEnter, "Enter"},
    {KeyboardCode::kSpace, "Space"},
    {KeyboardCode::kCapsLock, "CapsLock"},
    {KeyboardCode::kF1, "F1"},
    {KeyboardCode::kF2, "F2"},
    {KeyboardCode::kF3, "F3"},
    {KeyboardCode::kF4, "F4"},
    {KeyboardCode::kF5, "F5"},
    {KeyboardCode::kF6, "F6"},
    {KeyboardCode::kF7, "F7"},
    {KeyboardCode::kF8, "F8"},
    {KeyboardCode::kF9, "F9"},
    {KeyboardCode::kF10, "F10"},
    {KeyboardCode::kF11, "F11"},
    {KeyboardCode::kF12, "F12"},
    {KeyboardCode::kLeftCtrl, "LeftCtrl"},
    {KeyboardCode::kRightCtrl, "RightCtrl"},
    {KeyboardCode::kLeftShift, "LeftShift"},
    {KeyboardCode::kRightShift, "RightShift"},
    {KeyboardCode::kLeftAlt, "LeftAlt"},
    {KeyboardCode::kRightAlt, "RightAlt"},
    {KeyboardCode::kLeft, "Left"},
    {KeyboardCode::kRight, "Right"},
    {KeyboardCode::kUp, "Up"},
    {KeyboardCode::kDown, "Down"},
    {KeyboardCode::kHome, "Home"},
    {KeyboardCode::kEnd, "End"},
    {KeyboardCode::kPageUp, "PageUp"},
    {KeyboardCode::kPageDown, "PageDown"}};
}  // namespace

Keyboard::Keyboard(bool with_daemon) : with_daemon_(with_daemon) {}

Keyboard::~Keyboard() {
  if (with_daemon_) {
    keep_running_ = false;
    if (io_thread_.joinable()) {
      keep_running_ = false;
      io_thread_.join();
    }
  }
  if (fd_ > 0) close(fd_);
}

bool Keyboard::StartMonitoring(const std::string& event_name) {
  fd_ = open(event_name.c_str(), O_RDWR | O_NONBLOCK);
  if (fd_ < 0) {
    XLOG_ERROR_STREAM("Failed to open input device: " << event_name);
    return false;
  }

  if (with_daemon_) {
    keep_running_ = true;
    io_thread_ = std::thread([this]() {
      while (keep_running_) {
        this->PollEvent();
        usleep(10000);
      }
    });
  }

  return true;
}

void Keyboard::PollEvent() {
  struct input_event ev;
  while (ssize_t n = read(fd_, &ev, sizeof(ev)) > 0) {
    if (ev.type == EV_KEY) {
      XLOG_DEBUG_STREAM("Key " << ev.code
                               << (ev.value ? " pressed" : " released"));
      if (key_event_callback_ != nullptr) {
        key_event_callback_(
            keycode_map[ev.code],
            (ev.value ? KeyboardEvent::kPress : KeyboardEvent::kRelease));
      }
    }
  }
}

void Keyboard::RegisterKeyEventCallback(KeyEventCallback callback) {
  key_event_callback_ = callback;
}

std::string Keyboard::GetKeyName(KeyboardCode code) {
  return keyname_map[code];
}
}  // namespace xmotion