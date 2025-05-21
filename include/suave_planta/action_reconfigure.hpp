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
#ifndef SUAVE_PLANTA__RECONFIGURE_HPP_
#define SUAVE_PLANTA__RECONFIGURE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "plansys2_executor/ActionExecutorClient.hpp"

#include "system_modes_msgs/msg/mode_event.hpp"
#include "system_modes_msgs/srv/change_mode.hpp"
#include "system_modes_msgs/srv/get_mode.hpp"

namespace suave_planta
{

class Reconfigure : public plansys2::ActionExecutorClient
{
public:
  Reconfigure(const std::string & node_name,
    const std::chrono::nanoseconds & rate);

  virtual ~Reconfigure();

private:

  rclcpp::CallbackGroup::SharedPtr callback_group_srv_client_;

  void do_work();

  std::map<std::string, rclcpp::Client<system_modes_msgs::srv::ChangeMode>::SharedPtr> change_mode_cli_map_;

  std::map<std::string, rclcpp::Subscription<system_modes_msgs::msg::ModeEvent>::SharedPtr> mode_sub_map_;
  std::map<std::string, std::string> current_mode_map_;

  bool change_mode(std::string& system_name, std::string& mode_name);

};

} // end SUAVE_PLANTA

#endif
