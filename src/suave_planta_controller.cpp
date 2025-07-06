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
#include <ctime>

#include "std_msgs/msg/float32.hpp"
#include "std_srvs/srv/empty.hpp"
#include "lifecycle_msgs/msg/transition_event.hpp"

#include "suave_planta/suave_planta_controller.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace suave_planta
{

  SuavePlansysController::SuavePlansysController(const std::string & node_name)
  : Node(node_name), _time_limit(300)
{

}

void SuavePlansysController::init(){
  domain_expert_ = std::make_shared<plansys2::DomainExpertClient>();
  planner_client_ = std::make_shared<plansys2::PlannerClient>();
  problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();
  executor_client_ = std::make_shared<plansys2::ExecutorClient>("suave_planta_controller_executor");

  step_timer_cb_group_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  step_timer_ = this->create_wall_timer(
    500ms, std::bind(&SuavePlansysController::step, this), step_timer_cb_group_);

  save_mission_results_cli =
    this->create_client<std_srvs::srv::Empty>("mission_metrics/save");

  search_pipeline_transition_sub_  = this->create_subscription<lifecycle_msgs::msg::TransitionEvent>(
    "/f_generate_search_path_node/transition_event",
    10,
    std::bind(&SuavePlansysController::search_pipeline_transition_cb_, this, _1));

  time_limit_timer_cb_group_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  time_limit_timer_ = this->create_wall_timer(
    100ms, std::bind(&SuavePlansysController::time_limit_cb, this), time_limit_timer_cb_group_);

  this->declare_parameter("time_limit", 300);

  diagnostics_sub_cb_group_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions diagnostics_sub_options;
  diagnostics_sub_options.callback_group = diagnostics_sub_cb_group_;
  diagnostics_sub_  = this->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    "/diagnostics",
    10,
    std::bind(&SuavePlansysController::diagnostics_cb, this, _1), diagnostics_sub_options);
}

void SuavePlansysController::diagnostics_cb(const diagnostic_msgs::msg::DiagnosticArray &msg){
  for(const auto& status: msg.status){
    if(status.message == "Component status") {
      for(const auto& value: status.values){
        if (value.value == "OK") {
          auto rm_predicate_str = "(c_status " + value.key + " error_string)";
          auto rm_predicate = parser::pddl::fromStringPredicate(rm_predicate_str);
          if(problem_expert_->existPredicate(rm_predicate)) {
            problem_expert_->removePredicate(rm_predicate);
          }
        } else if (value.value == "ERROR") {
          auto add_predicate_str = "(c_status " + value.key + " error_string)";
          problem_expert_->addPredicate(parser::pddl::fromStringPredicate(add_predicate_str));
        }
      }
    }

    if(status.message == "QA status") {
      for(const auto& value : status.values){
        auto predicates = problem_expert_->getPredicates();
        for (const auto& predicate : predicates) {
          if (predicate.name == "qa_has_value" && predicate.parameters[0].name =="obs_" + value.key){
            problem_expert_->removePredicate(predicate);
          }
        }

        std::ostringstream oss;
        oss << std::fixed << std::setprecision(2) << std::stod(value.value);
        std::string value_two_decimals = oss.str();
        add_symbolic_number(value_two_decimals);
        auto add_predicate_str = "(qa_has_value obs_" + value.key + " " + value_two_decimals + "_decimal)";
        problem_expert_->addPredicate(parser::pddl::fromStringPredicate(add_predicate_str));
      }
    }
  }
}

void SuavePlansysController::add_symbolic_number(std::string number_str)
{
  float number_float = std::stof(number_str);
  std::string number_decimal = number_str + "_decimal";

  problem_expert_->addInstance(parser::pddl::fromStringParam(
    number_decimal, "numerical-object"));

  if (number_float < 0.25) {
    problem_expert_->addPredicate(parser::pddl::fromStringPredicate(
      "(lessthan " + number_decimal + " 0.25_decimal)"));
  } else if (number_float < 0.5) {
    problem_expert_->addPredicate(parser::pddl::fromStringPredicate(
      "(lessthan " + number_decimal + " 0.5_decimal)"));
  } else if (number_float < 0.75) {
    problem_expert_->addPredicate(parser::pddl::fromStringPredicate(
      "(lessthan " + number_decimal + " 0.75_decimal)"));
  } else if (number_float < 1.0) {
    problem_expert_->addPredicate(parser::pddl::fromStringPredicate(
      "(lessthan " + number_decimal + " 1.0_decimal)"));
  } else if (number_float < 1.25) {
    problem_expert_->addPredicate(parser::pddl::fromStringPredicate(
      "(lessthan " + number_decimal + " 1.25_decimal)"));
  } else if (number_float < 2.25) {
    problem_expert_->addPredicate(parser::pddl::fromStringPredicate(
      "(lessthan " + number_decimal + " 2.25_decimal)"));
  } else if (number_float < 3.25) {
    problem_expert_->addPredicate(parser::pddl::fromStringPredicate(
      "(lessthan " + number_decimal + " 3.25_decimal)"));
  }

  if (number_float > 0.25) {
    problem_expert_->addPredicate(parser::pddl::fromStringPredicate(
      "(lessthan 0.25_decimal " + number_decimal + ")"));
  } else if (number_float > 0.5) {
    problem_expert_->addPredicate(parser::pddl::fromStringPredicate(
      "(lessthan 0.5_decimal " + number_decimal + ")"));
  } else if (number_float > 0.75) {
    problem_expert_->addPredicate(parser::pddl::fromStringPredicate(
      "(lessthan 0.75_decimal " + number_decimal + ")"));
  } else if (number_float > 1.0) {
    problem_expert_->addPredicate(parser::pddl::fromStringPredicate(
      "(lessthan 1.0_decimal " + number_decimal + ")"));
  } else if (number_float > 1.25) {
    problem_expert_->addPredicate(parser::pddl::fromStringPredicate(
      "(lessthan 1.25_decimal " + number_decimal + ")"));
  } else if (number_float > 2.25) {
    problem_expert_->addPredicate(parser::pddl::fromStringPredicate(
      "(lessthan 2.25_decimal " + number_decimal + ")"));
  } else if (number_float > 3.25) {
    problem_expert_->addPredicate(parser::pddl::fromStringPredicate(
      "(lessthan 3.25_decimal " + number_decimal + ")"));
  }
}

SuavePlansysController::~SuavePlansysController()
{
}

bool SuavePlansysController::execute_plan(){
  // Compute the plan
  auto domain = domain_expert_->getDomain();
  auto problem = problem_expert_->getProblem();
  auto plan = planner_client_->getPlan(domain, problem);

  if (!plan.has_value()) {
    // for (auto instance: problem_expert_->getInstances()){
    //   std::string instance_info = "Instance " + instance.name + " type " + instance.type;
    //   RCLCPP_INFO(get_logger(), "%s", instance_info.c_str());
    // }
    // for (auto predicate: problem_expert_->getPredicates()) {
    //   std::string predicate_str = parser::pddl::toString(predicate);
    //   RCLCPP_INFO(get_logger(), "Predicate: %s", predicate_str.c_str());
    // }

    std::string goal_str = "Could not find plan to reach goal " + parser::pddl::toString(problem_expert_->getGoal());
    RCLCPP_INFO(get_logger(), "%s", goal_str.c_str());
    return false;
  }

  RCLCPP_INFO(get_logger(), "Selected plan: ");
  for (auto item : plan->items){
    RCLCPP_INFO(get_logger(), "  Action: '%s'", item.action.c_str());
  }
  // Execute the plan
  return executor_client_->start_plan_execution(plan.value());
}

void SuavePlansysController::finish_controlling(){
  step_timer_->cancel();
  executor_client_->cancel_plan_execution();
  request_save_mission_results();
  time_limit_timer_->cancel();
}

void SuavePlansysController::step(){
  if (first_iteration_){
    first_iteration_ = !execute_plan();
    return;
  }

  if (!executor_client_->execute_and_check_plan() && executor_client_->getResult()) {
    if (executor_client_->getResult().value().success) {
      RCLCPP_INFO(get_logger(), "Plan execution finished with success!");
      finish_controlling();
    } else {
      RCLCPP_INFO(get_logger(), "Replanning!");
      execute_plan();
      return;
    }
  }
}

void SuavePlansysController::time_limit_cb(){
    _time_limit = get_parameter("time_limit").as_int();
    if(_search_started && (get_clock()->now() - _start_time) >= rclcpp::Duration(_time_limit, 0)){
      RCLCPP_INFO(get_logger(), "Time limit reached!");
      finish_controlling();
    }
}

bool SuavePlansysController::request_save_mission_results(){
  while (!save_mission_results_cli->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service. Exiting.");
      return false;
    }
    RCLCPP_INFO(get_logger(), "mission_metrics/save service not available, waiting again...");
  }

  auto request = std::make_shared<std_srvs::srv::Empty::Request>();
  auto response = save_mission_results_cli->async_send_request(request);
  if (response.wait_for(1s) != std::future_status::ready)
  {
    RCLCPP_ERROR(get_logger(), "Failed to call service mission_metrics/save");
    return false;
  }
  return true;
}

void SuavePlansysController::search_pipeline_transition_cb_(const lifecycle_msgs::msg::TransitionEvent &msg){
  if(msg.goal_state.id == 3){
    _start_time = get_clock()->now();
    _search_started = true;
  }
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<suave_planta::SuavePlansysController>(
    "mission_node");

  node->init();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  executor.remove_node(node);
  rclcpp::shutdown();
  return 0;
}
