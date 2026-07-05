/**
 * @file state_interface.hpp
 * @date 16-01-2023
 * @brief
 *
 * @copyright Copyright (c) 2023 Ruixiang Du (rdu)
 */

#include <cstdint>

namespace xmotion {
class StateInterface {
 public:
  explicit StateInterface(int64_t id) : id_(id){};
  virtual ~StateInterface() = default;

  // non-copyable
  StateInterface(const StateInterface &other) = delete;
  StateInterface &operator=(const StateInterface &other) = delete;

  // public methods
  /****************** To Be Implemented ******************/
  virtual double operator[](int32_t i) const = 0;
  virtual double &operator[](int32_t i) = 0;
  /*******************************************************/

  int64_t GetId() const { return id_; }

//  protected:
  int64_t id_;
};
}  // namespace xmotion