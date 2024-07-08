/*
 * event_handler.hpp
 *
 * Created on 7/8/24 10:22 PM
 * Description:
 *
 * Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef QUADRUPED_EVENT_HANDLER_HPP
#define QUADRUPED_EVENT_HANDLER_HPP

namespace xmotion {
struct EventHandler {
  virtual ~EventHandler() = default;

  // pure virtual functions
  virtual bool Initialize() = 0;
  virtual void Start() = 0;
  virtual void PollEvents() = 0;
};
}  // namespace xmotion

#endif  // QUADRUPED_EVENT_HANDLER_HPP
