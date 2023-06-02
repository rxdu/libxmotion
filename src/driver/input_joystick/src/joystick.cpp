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

namespace robosw {
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
      //      auto js = std::make_shared<Joystick>();
      //      js->fd_ = fd;
      //      js->connected_ = true;
      //
      //      ioctl(fd, EVIOCGNAME(sizeof(js_name)), js_name);
      //      js->name_ = {js_name};
      //
      //      // Setup axes
      //      for (unsigned int i = 0; i < Joystick::max_js_axes; ++i) {
      //        input_absinfo axisInfo;
      //        if (ioctl(fd, EVIOCGABS(i), &axisInfo) != -1) {
      //          js->axes_[i].min = axisInfo.minimum;
      //          js->axes_[i].max = axisInfo.maximum;
      //        }
      //      }
      //
      //      // Setup rumble
      //      ff_effect effect = {0};
      //      effect.type = FF_RUMBLE;
      //      effect.id = -1;
      //      if (ioctl(fd, EVIOCSFF, &effect) != -1) {
      //        js->rumble_effect_id_ = effect.id;
      //        js->has_rumble_ = true;
      //      }

      jss.push_back(jd);
    }
  }
  return jss;
}

////////////////////////////////////////////////////////////////////////////////

Joystick::Joystick(JoystickDescriptor descriptor) : descriptor_(descriptor) {}

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

void Joystick::SetButtonEventCallback(
    JoystickInterface::ButtonEventCallback cb) {}

void Joystick::SetAxisEventCallback(JoystickInterface::AxisEventCallback cb) {}

void Joystick::ReadJoystickInput() {
  input_event event;
  while (read(fd_, &event, sizeof(event)) > 0) {
    if (event.type == EV_KEY && event.code >= BTN_JOYSTICK &&
        event.code <= BTN_THUMBR) {
      buttons_[event.code - 0x120] = event.value;
    }
    if (event.type == EV_ABS && event.code < ABS_TOOL_WIDTH) {
      auto axis = &axes_[event.code];
      float normalized =
          (event.value - axis->min) / (float)(axis->max - axis->min) * 2 - 1;
      axes_[event.code].value = normalized;
    }
  }
}

void Joystick::Update() {
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

    //    short weakRumble = fabsf(joysticks[i].axes[ABS_X].value) * 0xFFFF;
    //    short strongRumble = fabsf(joysticks[i].axes[ABS_Y].value) * 0xFFFF;
    //    setJoystickRumble(joysticks[i], weakRumble, strongRumble);
  }

  fflush(stdout);
  usleep(16000);
}
}  // namespace robosw