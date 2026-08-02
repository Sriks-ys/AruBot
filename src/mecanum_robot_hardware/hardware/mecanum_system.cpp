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
#include <array>

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

  serialDevice_rear = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);
  serialDevice_front = open("/dev/ttyUSB1", O_RDWR | O_NOCTTY);
  
  if (serialDevice_rear < 0)
  {
      RCLCPP_ERROR(get_logger(), "Failed to open serial port for rear arduino");
      return hardware_interface::CallbackReturn::ERROR;
  }

  if (serialDevice_front < 0)
  {
      RCLCPP_ERROR(get_logger(), "Failed to open serial port for front arduino");
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

  struct termios tty_rear{};
  struct termios tty_front{};
  tcgetattr(serialDevice_rear, &tty_rear);
  tcgetattr(serialDevice_front, &tty_front);

  cfsetispeed(&tty_rear, B115200);
  cfsetospeed(&tty_rear, B115200);

  cfsetispeed(&tty_front, B115200);
  cfsetospeed(&tty_front, B115200);

  tty_rear.c_cflag |= (CLOCAL | CREAD);
  tty_rear.c_cflag &= ~PARENB;
  tty_rear.c_cflag &= ~CSTOPB;
  tty_rear.c_cflag &= ~CSIZE;
  tty_rear.c_cflag |= CS8;

  tty_rear.c_lflag = 0;
  tty_rear.c_iflag = 0;
  tty_rear.c_oflag = 0;

  tty_rear.c_cc[VMIN] = 0;
  tty_rear.c_cc[VTIME] = 0;

  tty_front.c_cflag |= (CLOCAL | CREAD);
  tty_front.c_cflag &= ~PARENB;
  tty_front.c_cflag &= ~CSTOPB;
  tty_front.c_cflag &= ~CSIZE;
  tty_front.c_cflag |= CS8;

  tty_front.c_lflag = 0;
  tty_front.c_iflag = 0;
  tty_front.c_oflag = 0;

  tty_front.c_cc[VMIN] = 0;
  tty_front.c_cc[VTIME] = 0;

  tcsetattr(serialDevice_rear, TCSANOW, &tty_rear);
  tcsetattr(serialDevice_front, TCSANOW, &tty_front);

  RCLCPP_INFO(get_logger(), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MecanumSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (serialDevice_rear >= 0)
  {
      ::close(serialDevice_rear);
      serialDevice_rear = -1;
  }

  if (serialDevice_front >= 0)
  {
      ::close(serialDevice_front);
      serialDevice_front = -1;
  }

  RCLCPP_INFO(get_logger(), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type MecanumSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  
  uint8_t byte;

  while(::read(serialDevice_rear, &byte, 1) == 1){
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
              std::memcpy(&feedback_rear, rx_buffer_.data(), sizeof(feedback));
              rx_state_ = RxState::WAIT_HEADER1;

              set_state("rear_left_wheel_joint/velocity", static_cast<double>(feedback_rear.left_velocity));
              set_state("rear_right_wheel_joint/velocity", static_cast<double>(feedback_rear.right_velocity));

              set_state("rear_left_wheel_joint/position", static_cast<double>(feedback_rear.left_position));
              set_state("rear_right_wheel_joint/position", static_cast<double>(feedback_rear.right_position));
          }
          break;
    }
  }
  rx_index_ = 0;
  rx_state_ = RxState::WAIT_HEADER1;
  rx_buffer_.fill(0);
  while(::read(serialDevice_front, &byte, 1) == 1){
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
              std::memcpy(&feedback_front, rx_buffer_.data(), sizeof(feedback));
              rx_state_ = RxState::WAIT_HEADER1;

              set_state("front_left_wheel_joint/velocity", static_cast<double>(feedback_front.left_velocity));
              set_state("front_right_wheel_joint/velocity", static_cast<double>(feedback_front.right_velocity));

              set_state("front_left_wheel_joint/position", static_cast<double>(feedback_front.left_position));
              set_state("front_right_wheel_joint/position", static_cast<double>(feedback_front.right_position));
          }
          break;
    }

  }
  //RCLCPP_INFO(get_logger(), "Rear Left Wheel: Velocity: %.2f Position: %.2f", feedback_rear.left_velocity, feedback_rear.left_position);
  //RCLCPP_INFO(get_logger(), "Front Left Wheel: Velocity: %.2f Position: %.2f", feedback_front.left_velocity, feedback_front.left_position);

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type mecanum_robot_hardware ::MecanumSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  float fl = static_cast<float>(get_command("front_left_wheel_joint/velocity"));
  float fr = static_cast<float>(get_command("front_right_wheel_joint/velocity"));
  float rl = static_cast<float>(get_command("rear_left_wheel_joint/velocity"));
  float rr = static_cast<float>(get_command("rear_right_wheel_joint/velocity"));


  send_buffer_rear[0] = HEADER1;
  send_buffer_rear[1] = HEADER2;

  send_buffer_front[0] = HEADER1;
  send_buffer_front[1] = HEADER2;


  memcpy(send_buffer_rear + 2,  &rl, 4);
  memcpy(send_buffer_rear + 6,  &rr, 4);

  memcpy(send_buffer_front + 2,  &fl, 4);
  memcpy(send_buffer_front + 6,  &fr, 4);

  send_buffer_rear[10] = checksum(send_buffer_rear + 2, 8);
  send_buffer_front[10] = checksum(send_buffer_front + 2, 8);

  ssize_t nr = ::write(serialDevice_rear, send_buffer_rear, sizeof(send_buffer_rear));
  ssize_t nf = ::write(serialDevice_front, send_buffer_front, sizeof(send_buffer_front));

  if (nr != sizeof(send_buffer_rear))
  {
      RCLCPP_WARN(get_logger(), "Rear Serial write failed");
  }

  if (nf != sizeof(send_buffer_front))
  {
      RCLCPP_WARN(get_logger(), "Front Serial write failed");
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
