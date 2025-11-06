#ifndef FASTECH_HARDWARE_INTERFACE
#define FASTECH_HARDWARE_INTERFACE

#pragma once

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_list_macros.hpp"

extern "C" {
#include "FAS_EziMOTIONPlusR.h"
}

namespace fastech_hardware
{
class FastechSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(FastechSystem)
  
  // Life Cycle Nodes
  
  //std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  //std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  // System Interface 
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  int port_id_ = 2;
  int left_motor_id_;
  int right_motor_id_;
  //double position_ = 0.0;
  //double command_ = 0.0;
  std::string port_;
};
}  // namespace fastech_hardware

#endif
