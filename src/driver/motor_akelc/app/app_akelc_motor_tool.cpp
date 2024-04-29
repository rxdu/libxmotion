#include <memory>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32.hpp>

#include "modbus_rtu/modbus_rtu_port.hpp"
#include "motor_akelc/motor_akelc_modbus.hpp"

using namespace xmotion;
using namespace std::chrono_literals;

class AkelcMotorTool : public rclcpp::Node {
 public:
  AkelcMotorTool(std::shared_ptr<ModbusRtuPort> port, int device_id)
      : rclcpp::Node("akelc_motor_tool"), port_(port) {
    // set up motor
    motor_ = std::make_shared<MotorAkelcModbus>(port_, device_id);

    // set up ros
    rpm_pub_ = this->create_publisher<std_msgs::msg::Int32>("motor_rpm", 10);
    rpm_sub_ = this->create_subscription<std_msgs::msg::Int32>(
        "motor_rpm_cmd", 10,
        [this](const std_msgs::msg::Int32::SharedPtr message) {
          motor_->SetTargetRpm(message->data);
          RCLCPP_INFO(this->get_logger(), "Setting target RPM to %d",
                      message->data);
        });

    timer_ = this->create_wall_timer(
        50ms, std::bind(&AkelcMotorTool::FeedbackTimerCallback, this));
  }

 private:
  void FeedbackTimerCallback() {
    if (!motor_->IsReachable()) {
      std::cout << "Controller not reachable." << std::endl;
      return;
    }

    //    RCLCPP_INFO(this->get_logger(), "Driver temperature: %f",
    //                motor_->GetDriverTemperature());
    //    RCLCPP_INFO(this->get_logger(), "Driver input voltage: %f",
    //                motor_->GetDriverInputVoltage());
    //    RCLCPP_INFO(this->get_logger(), "Driver current: %f",
    //                motor_->GetDriverCurrent());

    auto message = std_msgs::msg::Int32();
    message.data = motor_->GetActualRpm();
    RCLCPP_INFO(this->get_logger(), "Driver actual RPM: %d", message.data);
    rpm_pub_->publish(message);
  }

  std::shared_ptr<ModbusRtuPort> port_;
  std::shared_ptr<MotorAkelcModbus> motor_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr rpm_pub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr rpm_sub_;
};

int main(int argc, char* argv[]) {
  std::shared_ptr<ModbusRtuPort> port = std::make_shared<ModbusRtuPort>();

  if (port->Open("/dev/ttyUSB0", 115200, ModbusRtuPort::Parity::kEven,
                 ModbusRtuPort::DataBit::kBit8,
                 ModbusRtuPort::StopBit::kBit1)) {
    std::cout << "Port opened successfully" << std::endl;
  } else {
    std::cout << "Failed to open port" << std::endl;
    return -1;
  }

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AkelcMotorTool>(port, 0x05));
  rclcpp::shutdown();
  return 0;
}
