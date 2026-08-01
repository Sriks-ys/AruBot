// Copyright 2021 ros2_control Development Team
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

#include "mecanum_robot_hardware/mecanumbot_system.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

#include <cstring>
#include <cstdint>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mecanum_robot_hardware
{
hardware_interface::CallbackReturn MecanumSystemHardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (
    hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Mecanum System Joint Validation
  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %zu command interfaces found. 1 expected.",
        joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have %s command interfaces found. '%s' expected.",
        joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have '%s' as first state interface. '%s' expected.",
        joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have '%s' as second state interface. '%s' expected.",
        joint.name.c_str(), joint.state_interfaces[1].name.c_str(),
        hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MecanumSystemHardware::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring ...please wait...");

  // reset values always when configuring hardware
  for (const auto & [name, descr] : joint_state_interfaces_)
  {
    set_state(name, 0.0);
  }
  for (const auto & [name, descr] : joint_command_interfaces_)
  {
    set_command(name, 0.0);
  }

  serialDevice = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);
  
  if (serialDevice < 0)
  {
      RCLCPP_ERROR(get_logger(), "Failed to open serial port");
      return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(get_logger(), "Successfully configured!");


  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MecanumSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // command and state should be equal when starting
  for (const auto & [name, descr] : joint_command_interfaces_)
  {
    set_command(name, get_state(name));
  }

  struct termios tty{};
  tcgetattr(serialDevice, &tty);

  cfsetispeed(&tty, B115200);
  cfsetospeed(&tty, B115200);

  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;

  tty.c_lflag = 0;
  tty.c_iflag = 0;
  tty.c_oflag = 0;

  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  tcsetattr(serialDevice, TCSANOW, &tty);

  RCLCPP_INFO(get_logger(), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MecanumSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (serialDevice >= 0)
  {
      ::close(serialDevice);
      serialDevice = -1;
  }

  RCLCPP_INFO(get_logger(), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type MecanumSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  
  uint8_t byte;

  while(::read(serialDevice, &byte, 1) == 1){
    switch (rx_state_) {
      case RxState::WAIT_HEADER1:
          if (byte == HEADER1)
              rx_state_ = RxState::WAIT_HEADER2;
          break;

      case RxState::WAIT_HEADER2:
          if (byte == HEADER2)
          {
              rx_index_ = 0;
              rx_state_ = RxState::READ_PAYLOAD;
          }
          else if (byte != HEADER1)
          {
              rx_state_ = RxState::WAIT_HEADER1;
          }
          break;

      case RxState::READ_PAYLOAD:
          rx_buffer_[rx_index_++] = byte;

          if (rx_index_ == sizeof(feedback))
          {
              std::memcpy(&feedback_, rx_buffer_.data(), sizeof(feedback));
              rx_state_ = RxState::WAIT_HEADER1;

              set_state("rear_left_wheel_joint/velocity", static_cast<double>(feedback_.left_velocity));
              set_state("rear_right_wheel_joint/velocity", static_cast<double>(feedback_.right_velocity));

              set_state("front_left_wheel_joint/velocity", get_command("front_left_wheel_joint/velocity"));
              set_state("front_right_wheel_joint/velocity", get_command("front_right_wheel_joint/velocity"));

              set_state("rear_left_wheel_joint/position", static_cast<double>(feedback_.left_position));
              set_state("rear_right_wheel_joint/position", static_cast<double>(feedback_.right_position));

              set_state("front_left_wheel_joint/position", get_state("front_left_wheel_joint/position") + period.seconds() * get_command("front_left_wheel_joint/velocity"));
              set_state("front_right_wheel_joint/position", get_state("front_right_wheel_joint/position") + period.seconds() * get_command("front_right_wheel_joint/velocity"));
          }
          break;
    }

  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type mecanum_robot_hardware ::MecanumSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  //float fl = static_cast<float>(get_command("front_left_wheel_joint/velocity"));
  //float fr = static_cast<float>(get_command("front_right_wheel_joint/velocity"));
  float rl = static_cast<float>(get_command("rear_left_wheel_joint/velocity"));
  float rr = static_cast<float>(get_command("rear_right_wheel_joint/velocity"));

  send_buffer[0] = HEADER1;
  send_buffer[1] = HEADER2;
  memcpy(send_buffer + 2,  &rl, 4);
  memcpy(send_buffer + 6,  &rr, 4);
  send_buffer[10] = checksum(send_buffer + 2, 8);

  ssize_t n = ::write(serialDevice, send_buffer, sizeof(send_buffer));

  if (n != sizeof(send_buffer))
  {
      RCLCPP_WARN(get_logger(), "Serial write failed");
  }
  
  return hardware_interface::return_type::OK;
}

uint8_t MecanumSystemHardware::checksum(const uint8_t *data, size_t len)
{
    uint8_t check = 0;

    while (len--)
        check ^= *data++;

    return check  ;
}

}  

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  mecanum_robot_hardware::MecanumSystemHardware, hardware_interface::SystemInterface)
