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

  SuavePlansysController::SuavePlansysController(const std::string &node_name)
      : Node(node_name), time_limit_(300)
  {
  }

  void SuavePlansysController::init()
  {
    domain_expert_ = std::make_shared<plansys2::DomainExpertClient>();
    planner_client_ = std::make_shared<plansys2::PlannerClient>();
    problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();
    executor_client_ = std::make_shared<plansys2::ExecutorClient>("suave_planta_controller_executor");

    step_timer_cb_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    step_timer_ = this->create_wall_timer(
        100ms, std::bind(&SuavePlansysController::step, this), step_timer_cb_group_);

    save_mission_results_cli =
        this->create_client<std_srvs::srv::Empty>("mission_metrics/save");
    
    mavros_state_sub_cb_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions mavros_state_sub_options;
    mavros_state_sub_options.callback_group = mavros_state_sub_cb_group_;
    mavros_state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
        "mavros/state",
        10,
        std::bind(&SuavePlansysController::mavros_state_cb, this, _1), mavros_state_sub_options);

    time_limit_timer_cb_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    time_limit_timer_ = this->create_wall_timer(
        100ms, std::bind(&SuavePlansysController::time_limit_cb, this), time_limit_timer_cb_group_);

    this->declare_parameter("time_limit", 300);

    diagnostics_sub_cb_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions diagnostics_sub_options;
    diagnostics_sub_options.callback_group = diagnostics_sub_cb_group_;
    diagnostics_sub_ = this->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
        "/diagnostics",
        10,
        std::bind(&SuavePlansysController::diagnostics_cb, this, _1), diagnostics_sub_options);
  }

  void SuavePlansysController::diagnostics_cb(const diagnostic_msgs::msg::DiagnosticArray &msg)
  {
    std::vector<plansys2::Predicate> new_predicates;
    std::vector<plansys2::Predicate> remove_predicates;
    std::map<std::string, plansys2::Predicate> qa_predicates;
    for (const auto &status : msg.status)
    {
      if (status.message == "Component status")
      {
        for (const auto &value : status.values)
        {
          std::string pred_str = "(c_status " + value.key + " error_string)";
          auto predicate = parser::pddl::fromStringPredicate(pred_str);
          if (value.value == "OK")
          {
            if (problem_expert_->existPredicate(predicate))
            {
              remove_predicates.push_back(predicate);
            }
          }
          else if (value.value == "ERROR")
          {
            new_predicates.push_back(predicate);
          }
        }
      }
      if (status.message == "QA status")
      {
        for (const auto &value : status.values)
        {
          std::ostringstream oss;
          oss << std::fixed << std::setprecision(2) << std::stod(value.value);
          std::string value_two_decimals = oss.str();
          if (value.key == "battery_level" && (std::stod(value.value) >= 0.25) == battery_charged_) {
            continue;
          } else if (value.key == "battery_level") {
            battery_charged_ = (std::stod(value.value) >= 0.25);
          }
          auto new_preds = add_symbolic_number(value_two_decimals);
          new_predicates.insert(new_predicates.end(), new_preds.begin(), new_preds.end());
          auto pred_str = "(qa_has_value obs_" + value.key + " " + value_two_decimals + "_decimal)";
          qa_predicates[value.key] = parser::pddl::fromStringPredicate(pred_str);
        }
      }
    }

    // Build a lookup for relevant QA keys
    const std::vector<std::pair<std::string, std::string>> qa_keys = {
        {"water_visibility", "obs_water_visibility"},
        {"battery_level", "obs_battery_level"}};
    bool had_obs_wv = false;
    bool had_obs_bl = false;
    auto predicates = problem_expert_->getPredicates();
    for (const auto &predicate : predicates)
    {
      if (predicate.name == "qa_has_value")
      {
        for (const auto &[qa_key, obs_name] : qa_keys)
        {
          if (qa_predicates.count(qa_key) && predicate.parameters[0].name == obs_name)
          {
            if (qa_predicates[qa_key] != predicate)
            {
              remove_predicates.push_back(predicate);
              new_predicates.push_back(qa_predicates[qa_key]);
            }
            if (obs_name == "obs_water_visibility")
            {
              had_obs_wv = true;
            }
            if (obs_name == "obs_battery_level")
            {
              had_obs_bl = true;
            }
          }
        }
      }
      if (had_obs_wv && had_obs_bl)
      {
        break;
      }
    }

    if (!had_obs_wv && qa_predicates.count("water_visibility"))
    {
      new_predicates.push_back(qa_predicates["water_visibility"]);
    }

    if (!had_obs_bl && qa_predicates.count("battery_level"))
    {
      new_predicates.push_back(qa_predicates["battery_level"]);
    }

    if (!new_predicates.empty() || !remove_predicates.empty())
    {
      problem_expert_->updatePredicates(new_predicates, remove_predicates);
    }
  }

  std::vector<plansys2::Predicate> SuavePlansysController::add_symbolic_number(const std::string &number_str)
  {
    std::vector<plansys2::Predicate> new_predicates;
    if (!numbers_added_.insert(number_str).second)
    {
      return new_predicates;
    }
    numbers_added_.insert(number_str);

    float number_float = std::stof(number_str);
    std::string number_decimal = number_str + "_decimal";

    problem_expert_->addInstance(parser::pddl::fromStringParam(
        number_decimal, "numerical-object"));

    static const std::vector<std::pair<float, std::string>> thresholds = {
        {0.25f, "0.25_decimal"}, {0.5f, "0.5_decimal"}, {0.75f, "0.75_decimal"}, {1.0f, "1.0_decimal"}, {1.25f, "1.25_decimal"}, {2.25f, "2.25_decimal"}, {3.25f, "3.25_decimal"}};

    // Less than checks
    for (const auto &[t, t_str] : thresholds)
    {
      if (number_float < t)
      {
        new_predicates.push_back(parser::pddl::fromStringPredicate(
            "(lessthan " + number_decimal + " " + t_str + ")"));
      }
    }
    // Greater than checks
    for (const auto &[t, t_str] : thresholds)
    {
      if (number_float > t)
      {
        new_predicates.push_back(parser::pddl::fromStringPredicate(
            "(lessthan " + t_str + " " + number_decimal + ")"));
      }
    }
    return new_predicates;
  }

  SuavePlansysController::~SuavePlansysController()
  {
  }

  bool SuavePlansysController::execute_plan()
  {
    // Compute the plan
    auto domain = domain_expert_->getDomain();
    auto problem = problem_expert_->getProblem();
    auto plan = planner_client_->getPlan(domain, problem);

    if (!plan.has_value())
    {
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
    for (auto item : plan->items)
    {
      RCLCPP_INFO(get_logger(), "  Action: '%s'", item.action.c_str());
    }
    // Execute the plan
    return executor_client_->start_plan_execution(plan.value());
  }

  void SuavePlansysController::finish_controlling()
  {
    step_timer_->cancel();
    executor_client_->cancel_plan_execution();
    request_save_mission_results();
    time_limit_timer_->cancel();
  }

  void SuavePlansysController::step()
  {
    if (first_iteration_)
    {
      first_iteration_ = !execute_plan();
      return;
    }

    if (!executor_client_->execute_and_check_plan() && executor_client_->getResult())
    {
      if (executor_client_->getResult().value().success)
      {
        RCLCPP_INFO(get_logger(), "Plan execution finished with success!");
        finish_controlling();
      }
      else
      {
        RCLCPP_INFO(get_logger(), "Replanning!");
        execute_plan();
        return;
      }
    }
  }

  void SuavePlansysController::time_limit_cb()
  {
    time_limit_ = get_parameter("time_limit").as_int();
    if (guided_mode_ && (get_clock()->now() - start_time_) >= rclcpp::Duration(time_limit_, 0))
    {
      RCLCPP_INFO(get_logger(), "Time limit reached!");
      finish_controlling();
    }
  }

  bool SuavePlansysController::request_save_mission_results()
  {
    while (!save_mission_results_cli->wait_for_service(1s))
    {
      if (!rclcpp::ok())
      {
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

  void SuavePlansysController::mavros_state_cb(const mavros_msgs::msg::State &msg) 
  {
    if (msg.mode == "GUIDED" && !guided_mode_)
    {
      start_time_ = get_clock()->now();
      guided_mode_ = true;
    }
  }

} // namespace

int main(int argc, char **argv)
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
