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
#ifndef SUAVE_PLANTA__SUAVE_PLANTA_CONTROLLER_HPP_
#define SUAVE_PLANTA__SUAVE_PLANTA_CONTROLLER_HPP_

#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"

// #include "suave_planta/visibility_control.h"

namespace suave_planta
{

class SuavePlansysController : public rclcpp::Node
{
public:
  SuavePlansysController(const std::string & node_name);

  virtual ~SuavePlansysController();

  void init();

protected:
  rclcpp::CallbackGroup::SharedPtr step_timer_cb_group_;
  rclcpp::TimerBase::SharedPtr step_timer_;

  std::shared_ptr<plansys2::DomainExpertClient> domain_expert_;
  std::shared_ptr<plansys2::PlannerClient> planner_client_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
  std::shared_ptr<plansys2::ExecutorClient> executor_client_;

  bool first_iteration_ = true;
  bool execute_plan();

  rclcpp::Time start_time_;
  int time_limit_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr save_mission_results_cli;

  bool guided_mode_ = false;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr mavros_state_sub_;
  rclcpp::CallbackGroup::SharedPtr mavros_state_sub_cb_group_;

  bool battery_charged_= true;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_sub_;
  rclcpp::CallbackGroup::SharedPtr diagnostics_sub_cb_group_;

  void diagnostics_cb(const diagnostic_msgs::msg::DiagnosticArray &msg);

  std::unordered_set<std::string> numbers_added_;
  std::vector<plansys2::Predicate> add_symbolic_number(const std::string& number);

  rclcpp::CallbackGroup::SharedPtr time_limit_timer_cb_group_;
  rclcpp::TimerBase::SharedPtr time_limit_timer_;
  
  void step();
  void finish_controlling();
  void time_limit_cb();
  bool request_save_mission_results();
  void mavros_state_cb(const mavros_msgs::msg::State &msg);
};

}  // namespace suave_planta

#endif  // SUAVE_PLANTA__SUAVE_PLANTA_CONTROLLER_HPP_
