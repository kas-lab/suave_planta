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
#include "suave_planta/action_reconfigure.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;
namespace suave_planta
{

  Reconfigure::Reconfigure(const std::string & node_name,
    const std::chrono::nanoseconds & rate)
  :   plansys2::ActionExecutorClient(node_name, rate)
  {
    callback_group_srv_client_ = create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);
  }

  Reconfigure::~Reconfigure()
  {
  }

  bool Reconfigure::change_mode(std::string& system_name, std::string& mode_name) {
    if(change_mode_cli_map_.count(system_name) && change_mode_cli_map_[system_name]->service_is_ready()) {
      auto request = std::make_shared<system_modes_msgs::srv::ChangeMode::Request>();
      request->mode_name = mode_name;
      auto change_mode_result_future = change_mode_cli_map_[system_name]->async_send_request(request);
      if (change_mode_result_future.wait_for(1s) == std::future_status::ready) {
        return change_mode_result_future.get()->success;
      }
    }
    return false;
  }

  void Reconfigure::do_work()
  {
    auto action_arguments = this->get_arguments();
    std::string system_name = action_arguments[0];
    std::string mode_name;
    
    if(action_arguments.size() == 3) {  //  If reconfigure action has 3 arguments (f fd_initial fd_goal)
      mode_name = action_arguments[2];
    } else {
      mode_name = action_arguments[1];  // If reconfigure action has 2 arguments (f fd_goal)
    }

    if (!change_mode_cli_map_.count(system_name)) {
      change_mode_cli_map_[system_name] = create_client<system_modes_msgs::srv::ChangeMode>(
        system_name + "/change_mode",
        rmw_qos_profile_services_default,
        callback_group_srv_client_);
    }

    if (!mode_sub_map_.count(system_name)) {
      mode_sub_map_[system_name] = create_subscription<system_modes_msgs::msg::ModeEvent>(
        system_name + "/mode_request_info",
        10,
        [this, system_name](const system_modes_msgs::msg::ModeEvent &msg) -> void {
          this->current_mode_map_[system_name] = msg.goal_mode.label;
        }
      );
    }

    if (current_mode_map_.count(system_name) && mode_name == current_mode_map_[system_name]) {
      return finish(true, 1.0, "Reconfiguration completed!");
    }
    change_mode(system_name, mode_name);
  }

}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<suave_planta::Reconfigure>(
    "reconfigure", 500ms);

  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();

  rclcpp::shutdown();

  return 0;
}
