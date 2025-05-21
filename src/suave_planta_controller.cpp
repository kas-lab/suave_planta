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
    1s, std::bind(&SuavePlansysController::step, this), step_timer_cb_group_);

  save_mission_results_cli =
    this->create_client<std_srvs::srv::Empty>("mission_metrics/save");

  search_pipeline_transition_sub_  = this->create_subscription<lifecycle_msgs::msg::TransitionEvent>(
    "/search_pipeline/transition_event",
    10,
    std::bind(&SuavePlansysController::search_pipeline_transition_cb_, this, _1));

  time_limit_timer_cb_group_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  time_limit_timer_ = this->create_wall_timer(
    100ms, std::bind(&SuavePlansysController::time_limit_cb, this), time_limit_timer_cb_group_);

  this->declare_parameter("time_limit", 300);

  diagnostics_sub_  = this->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    "/diagnostics",
    10,
    std::bind(&SuavePlansysController::diagnostics_cb, this, _1));
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

        add_symbolic_number(value.value);
        auto add_predicate_str = "(qa_has_value obs_" + value.key + " " + value.value + "_decimal)";
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

  problem_expert_->addPredicate(parser::pddl::fromStringPredicate(
    "(equalTo " + number_decimal + " " + number_decimal + ")"));

  auto instances = problem_expert_->getInstances();

  for (const auto& instance: instances) {
    if (instance.type != "numerical-object") {
      continue;
    }

    float number_float_2 = std::stof(instance.name.substr(0, instance.name.find("_")));

    if (number_float < number_float_2) {
      problem_expert_->addPredicate(parser::pddl::fromStringPredicate(
        "(lessThan " + number_decimal + " " + instance.name + ")"));
    } else if (number_float_2 < number_float) {
      problem_expert_->addPredicate(parser::pddl::fromStringPredicate(
        "(lessThan " + instance.name + " " + number_decimal + ")"));
    }
  }
}

SuavePlansysController::~SuavePlansysController()
{
}

void SuavePlansysController::execute_plan(){
  // Compute the plan
  auto domain = domain_expert_->getDomain();
  auto problem = problem_expert_->getProblem();
  auto plan = planner_client_->getPlan(domain, problem);

  if (!plan.has_value()) {
    for (auto instance: problem_expert_->getInstances()){
      std::cout<<"Instance "<< instance.name.c_str() << " type " <<
        instance.type.c_str() << std::endl;
    }
    for (auto predicate: problem_expert_->getPredicates()) {
      std::cout << "Predicates: " << std::endl;
      std::cout << parser::pddl::toString(predicate)<<std::endl;
    }

    std::cout << "Could not find plan to reach goal " <<
     parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
    return;
  }

  std::cout << "Selected plan: " << std::endl;
  for (auto item : plan->items){
    RCLCPP_INFO(get_logger(), "  Action: '%s'", item.action.c_str());
  }
  // Execute the plan
  executor_client_->start_plan_execution(plan.value());
}

void SuavePlansysController::finish_controlling(){
  step_timer_->cancel();
  executor_client_->cancel_plan_execution();
  request_save_mission_results();
  time_limit_timer_->cancel();
}

void SuavePlansysController::step(){
  if (first_iteration_){
    execute_plan();
    first_iteration_ = false;
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

  auto feedback = executor_client_->getFeedBack();
  for (const auto & action_feedback : feedback.action_execution_status) {
    if (action_feedback.status == plansys2_msgs::msg::ActionExecutionInfo::FAILED) {
      std::string error_str_ = "[" + action_feedback.action + "] finished with error: " + action_feedback.message_status;
      RCLCPP_ERROR(get_logger(), error_str_.c_str());
      break;
    }

    // std::string arguments_str_ = " ";
    // for (const auto & arguments: action_feedback.arguments){
    //   arguments_str_ += arguments + " ";
    // }
    // std::string feedback_str_ = "[" + action_feedback.action + arguments_str_ +
    //   std::to_string(action_feedback.completion * 100.0) + "%]";
    // RCLCPP_INFO(get_logger(), feedback_str_.c_str());
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
    "suave_planta_controller");

  node->init();

  rclcpp::Rate rate(5);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  // executor.add_node(std::make_shared<suave_planta::SuavePlansysController>(
  //   "suave_planta_controller"));
  executor.spin();
  executor.remove_node(node);
  rclcpp::shutdown();
  return 0;
}
