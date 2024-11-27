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
#ifndef SUAVE_PLANSY__SUAVE_PLANSYS_CONTROLLER_HPP_
#define SUAVE_PLANSY__SUAVE_PLANSYS_CONTROLLER_HPP_

#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"

// #include "suave_plansys/visibility_control.h"

namespace suave_plansys
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
  void execute_plan();

  rclcpp::Time _start_time;
  bool _search_started = false;
  int _time_limit;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr save_mission_results_cli;

  rclcpp::Subscription<lifecycle_msgs::msg::TransitionEvent>::SharedPtr search_pipeline_transition_sub_;

  rclcpp::CallbackGroup::SharedPtr time_limit_timer_cb_group_;
  rclcpp::TimerBase::SharedPtr time_limit_timer_;

  void step();
  void finish_controlling();
  void time_limit_cb();
  bool request_save_mission_results();
  void search_pipeline_transition_cb_(const lifecycle_msgs::msg::TransitionEvent &msg);
};

}  // namespace suave_plansys

#endif  // SUAVE_PLANSY__SUAVE_PLANSYS_CONTROLLER_HPP_
