// Copyright 2020 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "include/dof6robot_hardware_interface.hpp"

#include <unistd.h>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include <libusb-1.0/libusb.h>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace dof6robot_control
{
hardware_interface::CallbackReturn RobotHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  clock_ = std::make_shared<rclcpp::Clock>(rclcpp::Clock());

  // Get serial port parameters
  usb_port_ = info_.hardware_parameters["usb_port"];
  baud_rate_ = std::stoi(info_.hardware_parameters["baud_rate"]);

  // initialize USB
  libusb_init(NULL);

  hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // Robot has exactly one state and command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %zu command interfaces found. 1 expected.",
        joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have %s command interfaces found. '%s' expected.",
        joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %zu state interface. 1 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobotHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // 检查 baud_rate_ 是否是合法的 itas109::BaudRate
  switch (baud_rate_) {
    case 110:
    case 300:
    case 600:
    case 1200:
    case 2400:
    case 4800:
    case 9600:
    case 14400:
    case 19200:
    case 38400:
    case 56000:
    case 57600:
    case 115200:
    case 921600:
      // 合法
      break;
    default:
      RCLCPP_FATAL(get_logger(), "Invalid baud rate: %d", baud_rate_);
      return hardware_interface::CallbackReturn::ERROR;
  }
  // Initialize protocol
  protocol_ = std::make_shared<fsuservo::FSUS_Protocol>(usb_port_, static_cast<itas109::BaudRate>(baud_rate_));
  
  // Initialize servos
  servos_.clear();
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    // Get servo ID from joint parameters
    uint8_t servo_id = std::stoi(info_.joints[i].parameters.at("servo_id"));
    auto servo = std::make_shared<fsuservo::FSUS_Servo>(servo_id, protocol_.get());
    servo->init();
    
    if (!servo->isOnline)
    {
      RCLCPP_ERROR(get_logger(), "Servo %d is not online!", servo_id);
      return hardware_interface::CallbackReturn::ERROR;
    }
    
    servos_.push_back(servo);
    RCLCPP_INFO(get_logger(), "Servo %d initialized successfully", servo_id);
  }

  // reset values always when configuring hardware
  for (uint i = 0; i < hw_states_.size(); i++)
  {
    hw_states_[i] = 0;
    hw_commands_[i] = 0;
  }

  RCLCPP_INFO(get_logger(), "Successfully configured!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
RobotHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
RobotHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn RobotHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // command and state should be equal when starting
  for (uint i = 0; i < hw_states_.size(); i++)
  {
    hw_commands_[i] = hw_states_[i];
  }

  RCLCPP_INFO(get_logger(), "Successfully activated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobotHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // exit USB port
  libusb_exit(NULL);

  servos_.clear();
  RCLCPP_INFO(get_logger(), "Successfully deactivated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RobotHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  std::stringstream ss;
  ss << "Reading states:";
  for (uint i = 0; i < hw_states_.size(); i++)
  {
    servos_[i]->queryAngle();
    usleep(30000);
    // Read current angle from servo
    hw_states_[i] = servos_[i]->curAngle;
    ss << std::fixed << std::setprecision(2) << std::endl
       << "\t" << hw_states_[i] << " state for joint '" << info_.joints[i].name << "'";
  
  }
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "%s", ss.str().c_str());
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RobotHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // std::stringstream ss;
  for (uint i = 0; i < hw_commands_.size(); i++)
  {
    // Write command to servo
    servos_[i]->setAngle(hw_commands_[i]);
    usleep(20000);
    // protocol_->delay_ms(20); 
    // ss << std::fixed << std::setprecision(2) << std::endl
    //   << "\t" << hw_commands_[i] << " command for joint '" << info_.joints[i].name << "'";
  }
  sleep(1);
  // RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "%s", ss.str().c_str());

  return hardware_interface::return_type::OK;
}

}  // namespace dof6robot_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  dof6robot_control::RobotHardware, hardware_interface::SystemInterface)
