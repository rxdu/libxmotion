/*
 * fsm_template.hpp
 *
 * Created on 7/4/24 11:14 PM
 * Description:
 *
 * Reference:
 * [1] https://honeytreelabs.com/posts/real-time-state-machine-in-cpp/
 *
 * Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef QUADRUPED_MOTION_FSM_TEMPLATE_HPP
#define QUADRUPED_MOTION_FSM_TEMPLATE_HPP

#include <variant>
#include <optional>

namespace xmotion {
template <typename Context>
class FsmState {
 public:
  virtual ~FsmState() = default;

  virtual void Update(Context &context) = 0;
};

// TODO (rdu): check State in States is derived from FsmState
template <typename Context, typename Transition, typename... States>
class FiniteStateMachine {
 public:
  virtual ~FiniteStateMachine() = default;

  using StateVariant = std::variant<States...>;
  using OptionalStateVariant = std::optional<StateVariant>;

  FiniteStateMachine(StateVariant &&initial_state, Context &&context)
      : current_state_{std::move(initial_state)},
        context_{std::move(context)} {}

  void Update() {
    std::visit([this](auto &state) { state.Update(context_); }, current_state_);
    auto new_state = std::visit(
        [this](auto &state) -> OptionalStateVariant {
          return Transition::Transit(state, context_);
        },
        current_state_);
    if (new_state) {
      current_state_ = std::move(new_state.value());
    }
  }

  Context &GetContext() { return context_; }

 private:
  Context context_;
  StateVariant current_state_;
};
}  // namespace xmotion

#endif  // QUADRUPED_MOTION_FSM_TEMPLATE_HPP
