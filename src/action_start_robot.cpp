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
#include "suave_planning/action_start_robot.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;
namespace suave_plansys
{

  StartRobot::StartRobot(const std::string & node_name,
    const std::chrono::nanoseconds & rate)
  :   plansys2::ActionExecutorClient(node_name, rate)
  {
    callback_group_srv_client_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    arm_motors_cli_ = this->create_client<mavros_msgs::srv::CommandBool>(
      "mavros/cmd/arming",
      rmw_qos_profile_services_default,
      callback_group_srv_client_);

    set_guided_cli_ = this->create_client<mavros_msgs::srv::SetMode>(
      "mavros/set_mode",
      rmw_qos_profile_services_default,
      callback_group_srv_client_);
    mavros_state_sub_  = this->create_subscription<mavros_msgs::msg::State>(
      "mavros/state",
      10,
      std::bind(&StartRobot::state_cb, this, _1));
  }

  StartRobot::~StartRobot()
  {
  }

  void StartRobot::arm_thrusters(){
    if(arm_motors_cli_->service_is_ready()){
      auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
      request->value = true;
      auto arm_motors_result_ = arm_motors_cli_->async_send_request(request);

      // Wait for the result.
      if (arm_motors_result_.wait_for(1s) == std::future_status::ready)
      {
        auto arm_result_ = arm_motors_result_.get();
        if(!arm_result_->success){
          armed_ = false;
          return;
        }

        if(arm_result_->success){
          RCLCPP_INFO(this->get_logger(), "Thrusters armed!");
          armed_ = true;
          return;
        }
      }
    }
  }

  void StartRobot::state_cb(const mavros_msgs::msg::State &msg)
  {
    mode_ = msg.mode;
  }

  void StartRobot::set_guided_mode(){
    if(set_guided_cli_->service_is_ready()){
      auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
      request->custom_mode = "GUIDED";
      auto set_guided_result_ = set_guided_cli_->async_send_request(request);

      // Wait for the result.
      if (set_guided_result_.wait_for(1s) == std::future_status::ready)
      {
        auto result_ = set_guided_result_.get();
        if(!result_->mode_sent){
          return;
        }

        if(result_->mode_sent && mode_ == "GUIDED"){
          guided_ = true;
          RCLCPP_INFO(this->get_logger(), "Mode set to GUIDED!");
          finish(true, 1.0, "Robot in guided mode and armed!");
          return;
        }
      }
    }
  }

  void StartRobot::do_work()
  {
    if(armed_ != true){
      this->arm_thrusters();
      return;
    }
    if(armed_ == true && guided_ == false){
      this->set_guided_mode();
      return;
    }
  }

}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<suave_plansys::StartRobot>(
    "start_robot", 500ms);

  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();

  rclcpp::shutdown();

  return 0;
}
