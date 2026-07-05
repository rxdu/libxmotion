# Integrating xmNavigation into a robot application

xmNavigation is algorithm-centric by design ([ADR 0005](https://github.com/rxdu/xmotion/blob/main/docs/adr/0005-application-level-composition.md)): this library depends on xmBase and math libraries only, and never links hardware or middleware. Composing algorithms with hardware is **application code** — your robot's repository owns it. This page is the reviewed reference pattern for that glue, so an application starts from a known-good shape.

The snippets below were compile-checked against xmNavigation + xmDriver at the time of writing (2026-07-05); they are documentation, not built targets — the family maintains libraries, not applications.

## The in-process composition pattern

One process, direct calls: kinematics (this library) computes; actuator groups (xmDriver) command; the application owns construction, the loop, and shutdown.

```cpp
#include "kinematics/swerve_drive_kinematics.hpp"   // xmNavigation: pure math
#include "xmdriver/hal/motor_factory.hpp"           // xmDriver: construction seam
#include "xmdriver/hal/actuator_group.hpp"          // xmDriver: capability fan-out
#include "xmbase/telemetry/telemetry.hpp"           // xmBase: one instrumentation surface

namespace tel = xmotion::telemetry;
namespace hal = xmotion::hal;

int main() {
  // 1. Devices from configuration — the factory seam, no direct `new`.
  auto fl = hal::MotorFactory::Create(cfg.motor("front_left"));
  auto fr = hal::MotorFactory::Create(cfg.motor("front_right"));
  auto rl = hal::MotorFactory::Create(cfg.motor("rear_left"));
  auto rr = hal::MotorFactory::Create(cfg.motor("rear_right"));
  // (check for nullptr: unknown type is the caller's problem by contract)

  // 2. Hardware composition — xmDriver's capability-typed group.
  hal::SpeedActuatorGroup drive({*fl, *fr, *rl, *rr});   // registration order = fan-out order

  // 3. Algorithm — xmNavigation's kinematics, pure math.
  xmotion::SwerveDriveKinematics kin({.track_width = 0.4,
                                      .wheel_base = 0.6,
                                      .wheel_radius = 0.08,
                                      .max_driving_speed = 2.0,
                                      .max_steering_angle = 1.2});

  // 4. The application loop owns the wiring (and the telemetry span).
  while (running) {
    XM_SPAN("app.drive.cycle");
    auto cmd = kin.ComputeWheelCommands(current_twist);      // algorithm
    if (auto s = drive.SetSpeed(ToRpm(cmd)); !s.ok()) {      // hardware
      XM_WARN("drive fan-out degraded: {}", s.message());    // never silent
    }
  }
  drive.Stop();  // failsafe on the way out — the application owns shutdown order
}
```

What the pattern encodes:

- **Construction is configuration** (`MotorFactory::Create`) — swapping VESC for sim motors is a config change, not a code change; tests use the HAL's sim reference implementation.
- **The group is capability-typed** — it can only do what its members declared (`SpeedControllable` + `SafetyStoppable`); a `Status` failure is aggregated, never swallowed, and the group's own fault counter (`driver.actuator_group.command_fault_count`) counts it without any application code.
- **The algorithm never sees hardware** — `ComputeWheelCommands` is pure math; the application converts to HAL units at the boundary.
- **Telemetry is one spine**: the application's span, the group's fault counter, and any driver-internal events share one clock and one trace identity. Binding the xmTelemetry SDK (production integrations) requires no change to this code.

## The middleware bridge pattern (ROS 2 example)

Middleware stays at the application boundary ([ADR 0004](https://github.com/rxdu/xmotion/blob/main/docs/adr/0004-telemetry-layering.md) stance). The one non-obvious piece is carrying the telemetry trace identity through the message so cross-node causality survives:

```cpp
// Publishing side: stamp the outgoing message with the current context.
std::array<std::uint8_t, tel::kContextWireSize> ctx;
tel::Inject(ctx.data(), ctx.size());
msg.telemetry_ctx = ctx;                        // a fixed-size byte field in your IDL
publisher_->publish(msg);

// Subscribing side: adopt the sender's trace before doing the work.
void OnMsg(const Msg& msg) {
  tel::ContextGuard g(tel::Extract(msg.telemetry_ctx.data(), msg.telemetry_ctx.size()));
  XM_SPAN("app.node.handle_msg");               // same trace as the publisher's span
  // ... estimation / planning call into this library ...
}
```

The estimator/planner code inside the handler is plain xmNavigation — it neither knows nor cares that ROS delivered its input.

## Multi-process composition

A dedicated family communication layer (typed pub/sub with the context bytes carried as a transport property) is specified in [ADR 0006 (proposed)](https://github.com/rxdu/xmotion/blob/main/docs/adr/0006-messaging-layer.md). Until it lands, multi-process applications use the bridge pattern above over their middleware of choice.
