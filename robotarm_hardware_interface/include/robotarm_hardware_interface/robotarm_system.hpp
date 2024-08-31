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

#ifndef ROBOTARM_HARDWARE_INTERFACE__ROBOTARM_SYSTEM_HPP_
#define ROBOTARM_HARDWARE_INTERFACE__ROBOTARM_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "robotarm_hardware_interface/visibility_control.h"
#include "robotarm.h"

namespace robotarm_hardware
{
class ROBOTArmSystemHardware
: public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ROBOTArmSystemHardware);

  ROBOTARM_HARDWARE_INTERFACE_PUBLIC
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  ROBOTARM_HARDWARE_INTERFACE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  ROBOTARM_HARDWARE_INTERFACE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  ROBOTARM_HARDWARE_INTERFACE_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  ROBOTARM_HARDWARE_INTERFACE_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  ROBOTARM_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  ROBOTARM_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Parameters for the ROBOTArm simulation
  double hw_start_sec_;
  double hw_stop_sec_;
  double hw_slowdown_;
  double joint_initial_value[7];

  // Store the command for the simulated robot
  std::vector<double> hw_commands_;
  std::vector<double> hw_commands_last_;

  std::vector<double> hw_states_;
  std::vector<std::string> hw_joint_name_;

  robotarm::robotarm robotarm;
};

}  // namespace robotarm_hardware

#endif  // ROBOTARM_HARDWARE_INTERFACE__ROBOTARM_SYSTEM_HPP_
