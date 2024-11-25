/*
 * @file event.hpp
 * @date 10/7/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_EVENT_HPP
#define QUICKVIZ_EVENT_HPP

#include <tuple>
#include <string>
#include <iostream>

namespace xmotion {
enum class EventSource : int {
  kNone = 0,
  kKeyboard,
  kMouse,
  kMouseButton,
  kJoystickAxis,
  kJoystickButton,
  kUiElement,
  kApplicaton,
  kCustomEvent
};

class BaseEvent {
 public:
  virtual ~BaseEvent() = default;
  virtual EventSource GetSource() const = 0;
  virtual std::string GetName() const = 0;
};

template <typename... Args>
class Event : public BaseEvent {
 public:
  // Constructor to create an event with given arguments
  Event(EventSource type, const std::string& name, Args... args)
      : type_(type), name_(name), data_(std::make_tuple(args...)) {}

  // Get the stored data
  EventSource GetSource() const override { return type_; }

  std::string GetName() const override { return name_; }

  const std::tuple<Args...>& GetData() const { return data_; }

  // Helper function to print the data for demonstration purposes
  void Print() const {
    std::cout << "Event name: " << name_ << ", data: [";
    PrintTuple(data_);
    std::cout << "]" << std::endl;
  }

 private:
  EventSource type_;
  std::string name_;
  std::tuple<Args...> data_;

  // Helper function to print the contents of a tuple
  template <typename Tuple, std::size_t... Index>
  void printTupleImpl(const Tuple& tuple, std::index_sequence<Index...>) const {
    ((std::cout << (Index == 0 ? "" : ", ") << std::get<Index>(tuple)), ...);
  }

  // Wrapper to start the print from index 0 to tuple size - 1
  template <typename... T>
  void PrintTuple(const std::tuple<T...>& tuple) const {
    printTupleImpl(tuple, std::make_index_sequence<sizeof...(T)>());
  }
};
}  // namespace xmotion

#endif  // QUICKVIZ_EVENT_HPP