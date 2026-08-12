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

#ifndef MECANUM_ROBOT_HARDWARE__MECANUMBOT_SYSTEM_HPP_
#define MECANUM_ROBOT_HARDWARE__MECANUMBOT_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

#include <cstring>
#include <cstdint>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace mecanum_robot_hardware
{
class MecanumSystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(MecanumSystemHardware)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  

private:
  // Parameters for the DiffBot simulation
  double hw_start_sec_;
  double hw_stop_sec_;
  
  int serialDevice_rear;
  int serialDevice_front; 

  const uint8_t HEADER1 = 0xAA;
  const uint8_t HEADER2 = 0x55;

  struct SerialPacket {
    float set_speed_A;
    float set_speed_B;
    uint8_t checksum;
  };

  SerialPacket c;
  uint8_t send_buffer_rear[11];
  uint8_t send_buffer_front[11];

  enum class RxState {
    WAIT_HEADER1,
    WAIT_HEADER2,
    READ_PAYLOAD
  };
  
  

  struct __attribute__((packed)) feedback {
    float left_velocity;
    float right_velocity;
    float left_position;
    float right_position;
    uint8_t stage;
  };

  RxState rx_state_{RxState::WAIT_HEADER1};
  std::size_t rx_index_{0};
  std::array<uint8_t, sizeof(feedback)> rx_buffer_;

  feedback feedback_rear;
  feedback feedback_front;


  uint8_t checksum(const uint8_t *data, size_t len);
};

}  // namespace mecanum_robot_hardware

#endif  // MECANUM_ROBOT_HARDWARE__MECANUMBOT_SYSTEM_HPP_
