#ifndef MOBILE_BASE_HARDWARE_INTERFACE_HPP
#define MOBILE_BASE_HARDWARE_INTERFACE_HPP

#include "rclcpp/macros.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "Definitions.h"
#include <string>

namespace mobile_base_hardware {

    class MobileBaseHardwareInterface : public hardware_interface::SystemInterface
    {
        public:
             RCLCPP_SHARED_PTR_DEFINITIONS(MobileBaseHardwareInterface)

            hardware_interface::CallbackReturn
                on_configure(const rclcpp_lifecycle::State & previous_state) override;
            
            hardware_interface::CallbackReturn
                on_activate(const rclcpp_lifecycle::State & previous_state) override;
            
            hardware_interface::CallbackReturn
                on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

            // SystemInterface override

            hardware_interface::CallbackReturn
                on_init(const hardware_interface::HardwareInfo & info) override;
                

            hardware_interface::return_type
                 read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
            
            hardware_interface::return_type
                write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

        private:

            void* keyHandle_ = nullptr;
            unsigned int errorCode_ = 0;
            unsigned short node_1_;
            unsigned short node_2_;

            double pos_[2];
            double vel_[2];

            double cmd_vel_[2];
            double cmd_pos_[2];

            std::string device_name_;
            std::string protocol_name_;
            std::string interface_name_;
            std::string port_name_;
            int baudrate_;
            
            

    };
}


#endif