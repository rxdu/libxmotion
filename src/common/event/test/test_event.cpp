/*
 * @file test_event.cpp
 * @date 10/7/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include <iostream>

#include "event/event.hpp"
#include "event/event_dispatcher.hpp"
#include "event/event_emitter.hpp"

using namespace xmotion;

int main(int argc, char* argv[]) {
  EventDispatcher::GetInstance().RegisterHandler(
      "test_event", [](std::shared_ptr<BaseEvent> event) {
        auto data =
            std::static_pointer_cast<Event<int, double, std::string>>(event)
                ->GetData();
        auto a = std::get<0>(data);
        auto b = std::get<1>(data);
        auto c = std::get<2>(data);
        std::cout << "Received event: a = " << a << ", b = " << b
                  << ", c = " << c << std::endl;
      });

  EventEmitter emitter;
  emitter.Emit<Event<int, double, std::string>>(
      EventSource::kApplicaton, "test_event", 42, 3.14, "hello");

  return 0;
}
