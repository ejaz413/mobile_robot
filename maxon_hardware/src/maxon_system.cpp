#include "maxon_hardware/hardware_interface.hpp"

#include <chrono>
#include <cmath>

namespace mobile_base_hardware
{

hardware_interface::CallbackReturn MobileBaseHardwareInterface::on_init(
    const hardware_interface::HardwareInfo & info)
{
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
        return CallbackReturn::ERROR;
    }

    device_name_     = "EPOS4";
    protocol_name_   = "CANopen";
    interface_name_  = "CAN_kvaser_usb 0";
    port_name_       = "CAN0";
    baudrate_        = 1000000;

    node_1_ = 1;
    node_2_ = 2;

    RCLCPP_INFO(rclcpp::get_logger("MaxonSystem"),
                "EPOS4 hardware interface initialized.");

    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MobileBaseHardwareInterface::on_configure(
    const rclcpp_lifecycle::State &)
{
    keyHandle_ = VCS_OpenDevice(
        (char*)device_name_.c_str(),
        (char*)protocol_name_.c_str(),
        (char*)interface_name_.c_str(),
        (char*)port_name_.c_str(),
        &errorCode_);

    if (!keyHandle_) {
        RCLCPP_ERROR(rclcpp::get_logger("MaxonSystem"), "Failed to open EPOS4 device!");
        return CallbackReturn::ERROR;
    }

    if (!VCS_SetProtocolStackSettings(keyHandle_, baudrate_, 5000, &errorCode_))
    {
        RCLCPP_ERROR(rclcpp::get_logger("MaxonSystem"), "Failed to set baudrate!");
        return CallbackReturn::ERROR;
    }

    // Clear faults
    VCS_ClearFault(keyHandle_, node_1_, &errorCode_);
    VCS_ClearFault(keyHandle_, node_2_, &errorCode_);

    return CallbackReturn::SUCCESS;
}

/*----------------------------------------------------------
 *                on_activate()
 *---------------------------------------------------------*/
hardware_interface::CallbackReturn MobileBaseHardwareInterface::on_activate(
    const rclcpp_lifecycle::State &)
{
    // Enable motors
    VCS_SetEnableState(keyHandle_, node_1_, &errorCode_);
    VCS_SetEnableState(keyHandle_, node_2_, &errorCode_);

    // Set velocity mode for differential drive
    VCS_SetOperationMode(keyHandle_, node_1_, OMD_VELOCITY_MODE, &errorCode_);
    VCS_SetOperationMode(keyHandle_, node_2_, OMD_VELOCITY_MODE, &errorCode_);

    RCLCPP_INFO(rclcpp::get_logger("MaxonSystem"), "Motors enabled and set to VELOCITY MODE.");

    // Initialize states to 0
    set_state("base_left_wheel_joint/velocity", 0.0);
    set_state("base_right_wheel_joint/velocity", 0.0);
    set_state("base_left_wheel_joint/position", 0.0);
    set_state("base_right_wheel_joint/position", 0.0);

    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MobileBaseHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State &)
{
    VCS_SetDisableState(keyHandle_, node_1_, &errorCode_);
    VCS_SetDisableState(keyHandle_, node_2_, &errorCode_);

    VCS_CloseDevice(keyHandle_, &errorCode_);

    RCLCPP_INFO(rclcpp::get_logger("MaxonSystem"), "EPOS4 device closed.");

    return CallbackReturn::SUCCESS;
}

/*----------------------------------------------------------
 *                       READ()
 *---------------------------------------------------------*/
hardware_interface::return_type MobileBaseHardwareInterface::read(
    const rclcpp::Time &, const rclcpp::Duration & period)
{
    int vel_raw_1 = 0;
    int vel_raw_2 = 0;

    // "GetVelocityIs" returns raw velocity in counts/sec
    VCS_GetVelocityIs(keyHandle_, node_1_, &vel_raw_1, &errorCode_);
    VCS_GetVelocityIs(keyHandle_, node_2_, &vel_raw_2, &errorCode_);

    double left_vel  = static_cast<double>(vel_raw_1);
    double right_vel = static_cast<double>(vel_raw_2);

    // Update ROS states
    set_state("base_left_wheel_joint/velocity", left_vel);
    set_state("base_right_wheel_joint/velocity", right_vel);

    set_state("base_left_wheel_joint/position",
              get_state("base_left_wheel_joint/position") + left_vel * period.seconds());
    set_state("base_right_wheel_joint/position",
              get_state("base_right_wheel_joint/position") + right_vel * period.seconds());

    return hardware_interface::return_type::OK;
}

/*----------------------------------------------------------
 *                       WRITE()
 *---------------------------------------------------------*/
hardware_interface::return_type MobileBaseHardwareInterface::write(
    const rclcpp::Time &, const rclcpp::Duration &)
{
    double left_cmd  = get_command("base_left_wheel_joint/velocity");
    double right_cmd = get_command("base_right_wheel_joint/velocity");

    // Send velocity in counts/sec
    VCS_SetVelocityMust(keyHandle_, node_1_, static_cast<int>(left_cmd), &errorCode_);
    VCS_SetVelocityMust(keyHandle_, node_2_, static_cast<int>(right_cmd), &errorCode_);

    return hardware_interface::return_type::OK;
}

} // namespace mobile_base_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mobile_base_hardware::MobileBaseHardwareInterface, hardware_interface::SystemInterface)
