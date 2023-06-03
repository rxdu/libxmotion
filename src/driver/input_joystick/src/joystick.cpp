/*
 * joystick.cpp
 *
 * Created on 5/30/23 11:00 PM
 * Description:
 *
 * Copyright (c) 2023 Ruixiang Du (rdu)
 */

#include "input_joystick/joystick.hpp"

#include <sys/ioctl.h>
#include <sys/inotify.h>

#include <fcntl.h>
#include <unistd.h>

#include <linux/input.h>
#include <linux/joystick.h>

#include <iostream>

#define PRINT_DEBUG_MSG 0

namespace robosw {
static const char* js_button_names[32] = {
    "TRIGGER", "THUMB", "THUMB2", "TOP",   "TOP2", "PINKIE", "BASE",  "BASE2",
    "BASE3",   "BASE4", "BASE5",  "BASE6", "",     "",       "",      "DEAD",
    "SOUTH",   "EAST",  "C",      "NORTH", "WEST", "Z",      "TL",    "TR",
    "TL2",     "TR2",   "SELECT", "START", "MODE", "THUMBL", "THUMBR"};

static const char* js_axis_names[32] = {
    "X",        "Y",        "Z",      "RX",     "RY",        "RZ",
    "THROTTLE", "RUDDER",   "WHEEL",  "GAS",    "BRAKE",     "",
    "",         "",         "",       "",       "HAT0X",     "HAT0Y",
    "HAT1X",    "HAT1Y",    "HAT2X",  "HAT2Y",  "HAT3X",     "HAT3Y",
    "PRESSURE", "DISTANCE", "TILT_X", "TILT_Y", "TOOL_WIDTH"};

std::vector<JoystickDescriptor> Joystick::EnumberateJoysticks(int max_index) {
  std::vector<JoystickDescriptor> jss;

  char js_name[128];
  for (int i = 0; i < max_index; ++i) {
    std::string file_name = "/dev/input/event" + std::to_string(i);
    int fd = open(file_name.c_str(), O_RDWR | O_NONBLOCK);
    if (fd != -1) {
      JoystickDescriptor jd;
      ioctl(fd, EVIOCGNAME(sizeof(js_name)), js_name);
      jd.index = i;
      jd.name = {js_name};
      close(fd);
      jss.push_back(jd);
    }
  }
  return jss;
}

////////////////////////////////////////////////////////////////////////////////

Joystick::Joystick(JoystickDescriptor descriptor) : descriptor_(descriptor) {
  // initialize values
  for (int i = 0; i < max_js_buttons; ++i) buttons_[i] = false;
  for (int i = 0; i < max_js_axes; ++i) {
    axes_[i].min = 0;
    axes_[i].max = 0;
    axes_[i].value = 0;
  }
}

Joystick::~Joystick() {
  if (connected_) {
    Close();
  }
}

bool Joystick::Open() {
  if (connected_) return true;

  std::string file_name =
      "/dev/input/event" + std::to_string(descriptor_.index);
  fd_ = open(file_name.c_str(), O_RDWR | O_NONBLOCK);
  if (fd_ != -1) {
    connected_ = true;

    // Setup axes
    for (unsigned int i = 0; i < Joystick::max_js_axes; ++i) {
      input_absinfo axisInfo;
      if (ioctl(fd_, EVIOCGABS(i), &axisInfo) != -1) {
        axes_[i].min = axisInfo.minimum;
        axes_[i].max = axisInfo.maximum;
      }
    }

    // Setup rumble
    ff_effect effect = {0};
    effect.type = FF_RUMBLE;
    effect.id = -1;
    if (ioctl(fd_, EVIOCSFF, &effect) != -1) {
      rumble_effect_id_ = effect.id;
      has_rumble_ = true;
    }

    device_change_notify_ = inotify_init1(IN_NONBLOCK);
    inotify_add_watch(device_change_notify_, file_name.c_str(), IN_ATTRIB);

    keep_running_ = true;
    io_thread_ = std::thread([this]() {
      while (keep_running_) {
        this->Update();
      }
    });
  }
  return connected_;
}

void Joystick::Close() {
  keep_running_ = false;
  if (io_thread_.joinable()) io_thread_.join();

  if (connected_) {
    close(fd_);
    connected_ = false;
  }
}

bool Joystick::IsOpened() const { return connected_; }

std::string Joystick::GetButtonName(const JsButton& btn) const {
  return std::string(js_button_names[static_cast<int>(btn)]);
}

std::string Joystick::GetAxisName(const JsAxis& axis) const {
  return std::string(js_axis_names[static_cast<int>(axis)]);
}

bool Joystick::GetButtonState(const JsButton& btn) const {
  std::lock_guard<std::mutex> lock(buttons_mtx_);
  return buttons_[static_cast<int>(btn)];
}

JsAxisValue Joystick::GetAxisState(const JsAxis& axis) const {
  std::lock_guard<std::mutex> lock(axes_mtx_);
  return axes_[static_cast<int>(axis)];
}

void Joystick::ReadJoystickInput() {
  input_event event;
  while (read(fd_, &event, sizeof(event)) > 0) {
    {
      std::lock_guard<std::mutex> lock(buttons_mtx_);
      if (event.type == EV_KEY && event.code >= BTN_JOYSTICK &&
          event.code <= BTN_THUMBR) {
        buttons_[event.code - 0x120] = event.value;
      }
    }
    {
      std::lock_guard<std::mutex> lock(axes_mtx_);
      if (event.type == EV_ABS && event.code < ABS_TOOL_WIDTH) {
        auto axis = &axes_[event.code];
        float normalized =
            (event.value - axis->min) / (float)(axis->max - axis->min) * 2 - 1;
        axes_[event.code].value = normalized;
      }
    }
  }
}

void Joystick::Update() {
  // Update which joysticks are connected
  inotify_event event;
  //  if (read(device_change_notify_, &event, sizeof(event) + 16) != -1) {
  if (read(device_change_notify_, &event, sizeof(event)) != -1) {
    Close();
    Open();
  }

  // Update and print inputs for each connected joystick
  if (connected_) {
    ReadJoystickInput();

#if PRINT_DEBUG_MSG
    printf("%s - Axes: ", descriptor_.name.c_str());
    for (char axisIndex = 0; axisIndex < max_js_axes; ++axisIndex) {
      if (axes_[axisIndex].max - axes_[axisIndex].min)
        printf("%s:% f ", js_axis_names[axisIndex], axes_[axisIndex].value);
    }
    printf("Buttons: ");
    for (char buttonIndex = 0; buttonIndex < max_js_buttons; ++buttonIndex) {
      if (buttons_[buttonIndex]) printf("%s ", js_button_names[buttonIndex]);
    }
    printf("\n");
#endif
  }
#if PRINT_DEBUG_MSG
  fflush(stdout);
#endif

  usleep(16000);
}

void Joystick::SetJoystickRumble(short weakRumble, short strongRumble) {
  if (has_rumble_) {
    ff_effect effect = {0};
    effect.type = FF_RUMBLE;
    effect.id = rumble_effect_id_;
    effect.replay.length = 5000;
    effect.replay.delay = 0;
    effect.u.rumble.weak_magnitude = weakRumble;
    effect.u.rumble.strong_magnitude = strongRumble;
    ioctl(fd_, EVIOCSFF, &effect);

    input_event play = {0};
    play.type = EV_FF;
    play.code = rumble_effect_id_;
    play.value = 1;
    write(fd_, &play, sizeof(play));
  }
}
}  // namespace robosw