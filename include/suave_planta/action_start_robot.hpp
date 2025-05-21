// Copyright 2024 Gustavo Rezende Silva
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
#ifndef SUAVE_PLANTA__START_ROBOT_HPP_
#define SUAVE_PLANTA__START_ROBOT_HPP_

#include "rclcpp/rclcpp.hpp"

#include "plansys2_executor/ActionExecutorClient.hpp"

#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "mavros_msgs/msg/state.hpp"

namespace suave_planta
{

class StartRobot : public plansys2::ActionExecutorClient
{
public:
  StartRobot(const std::string & node_name,
    const std::chrono::nanoseconds & rate);

  virtual ~StartRobot();

private:
  bool armed_ = false;
  bool guided_ = false;
  std::string mode_;

  rclcpp::CallbackGroup::SharedPtr callback_group_srv_client_;

  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_motors_cli_;

  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_guided_cli_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr mavros_state_sub_;
  void state_cb(const mavros_msgs::msg::State &msg);

  void arm_thrusters();
  void set_guided_mode();
  void do_work();
};

} // end SUAVE_PLANTA

#endif
