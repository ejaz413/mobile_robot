#include "fastech_hardware/fastech_system.hpp"
//#include "pluginlib/class_list_macros.hpp"

namespace fastech_hardware
{

hardware_interface::CallbackReturn FastechSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)

  {
    return hardware_interface::CallbackReturn::ERROR;
  }
    

  // Convert string to wide string for Fastech SDK

  info_ = info;

  const wchar_t *port_name = L"/dev/ttyUSB0";
  unsigned int baudrate = 115200;
  port_id_ = 0;  // your SDK requires a port ID, usually 0

  int result = FAS_OpenPort(port_name, baudrate, port_id_);
  if (result != 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("FastechSystem"), "Failed to open port!");
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("FastechSystem"), "Fastech driver initialized successfully.");
  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn FastechSystem::on_activate(const rclcpp_lifecycle::State &)
{
  FAS_ServoEnable(port_id_, 1, 1);  // Enable servo
  RCLCPP_INFO(rclcpp::get_logger("FastechSystem"), "Servo enabled.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FastechSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  FAS_ServoEnable(port_id_, 1, 0);  // Disable servo
  RCLCPP_INFO(rclcpp::get_logger("FastechSystem"), "Servo disabled.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type FastechSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  int pos;
  if (FAS_GetActualPos(port_id_, 1, &pos) == 0)
  {
    position_ = static_cast<double>(pos);
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type FastechSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  FAS_MoveSingleAxisAbsPos(port_id_, 1, static_cast<int>(command_), 10000);
  return hardware_interface::return_type::OK;
}

}  // namespace fastech_hardware

PLUGINLIB_EXPORT_CLASS(fastech_hardware::FastechSystem, hardware_interface::SystemInterface)
