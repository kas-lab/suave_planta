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
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <future>
#include <iomanip>
#include <sstream>
#include <stdexcept>

#include "std_msgs/msg/float32.hpp"
#include "std_srvs/srv/empty.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "suave_planta/suave_planta_controller.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace suave_planta
{

SuavePlansysController::SuavePlansysController(const std::string & node_name)
: Node(node_name), time_limit_(300)
{
}

void SuavePlansysController::init()
{
  domain_expert_ = std::make_shared<plansys2::DomainExpertClient>();
  planner_client_ = std::make_shared<plansys2::PlannerClient>();
  problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();
  executor_client_ = std::make_shared<plansys2::ExecutorClient>("suave_planta_controller_executor");

  failure_pub_ = this->create_publisher<std_msgs::msg::String>(
    "mission/control_failure", 10);

  this->declare_parameter("time_limit", 300);
  this->declare_parameter("plansys_startup_timeout", 30.0);
  plansys_startup_timeout_ = this->get_parameter(
    "plansys_startup_timeout").as_double();
  controller_start_time_ = this->get_clock()->now();
  readiness_request_started_ = controller_start_time_;
  next_readiness_check_ = controller_start_time_;

  lifecycle_cb_group_ = this->create_callback_group(
    rclcpp::CallbackGroupType::Reentrant);
  for (const auto * service_name : {
      "domain_expert/get_state",
      "problem_expert/get_state",
      "planner/get_state",
      "executor/get_state"})
  {
    lifecycle_state_clients_.push_back(
      this->create_client<LifecycleGetState>(
        service_name,
        rmw_qos_profile_services_default,
        lifecycle_cb_group_));
  }

  step_timer_cb_group_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  step_timer_ = this->create_wall_timer(
    100ms,
    [this]() {step_guarded();},
    step_timer_cb_group_);

  save_mission_results_cli =
    this->create_client<std_srvs::srv::Empty>("mission_metrics/save");

  mavros_state_sub_cb_group_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions mavros_state_sub_options;
  mavros_state_sub_options.callback_group = mavros_state_sub_cb_group_;
  mavros_state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
    "mavros/state",
    10,
    [this](const mavros_msgs::msg::State & msg) {
      mavros_state_cb_guarded(msg);
    },
    mavros_state_sub_options);

  diagnostics_sub_cb_group_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions diagnostics_sub_options;
  diagnostics_sub_options.callback_group = diagnostics_sub_cb_group_;
  diagnostics_sub_ = this->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    "/diagnostics",
    10,
    [this](const diagnostic_msgs::msg::DiagnosticArray & msg) {
      diagnostics_cb_guarded(msg);
    },
    diagnostics_sub_options);
}

void SuavePlansysController::diagnostics_cb(const diagnostic_msgs::msg::DiagnosticArray & msg)
{
  std::lock_guard<std::mutex> lock(input_mutex_);
  for (const auto & status : msg.status) {
    for (const auto & value : status.values) {
      pending_diagnostic_values_[{status.message, value.key}] = value.value;
    }
  }
}

diagnostic_msgs::msg::DiagnosticArray
SuavePlansysController::take_pending_diagnostics()
{
  std::map<DiagnosticKey, std::string> pending_values;
  {
    std::lock_guard<std::mutex> lock(input_mutex_);
    pending_values.swap(pending_diagnostic_values_);
  }

  diagnostic_msgs::msg::DiagnosticArray msg;
  std::map<std::string, diagnostic_msgs::msg::DiagnosticStatus> statuses;
  for (const auto & [key, value] : pending_values) {
    auto & status = statuses[key.first];
    status.message = key.first;
    diagnostic_msgs::msg::KeyValue key_value;
    key_value.key = key.second;
    key_value.value = value;
    status.values.push_back(key_value);
  }
  for (auto & [message, status] : statuses) {
    (void)message;
    msg.status.push_back(std::move(status));
  }
  return msg;
}

void SuavePlansysController::apply_diagnostics(
  const diagnostic_msgs::msg::DiagnosticArray & msg)
{
  if (!battery_tracking_checked_) {
    battery_tracking_checked_ = true;
    for (const auto & predicate : problem_expert_->getPredicates()) {
      if (predicate.name == "qa_has_value" && !predicate.parameters.empty() &&
        predicate.parameters[0].name == "obs_battery_level")
      {
        battery_tracking_supported_ = true;
        break;
      }
    }
  }

  std::vector<plansys2::Predicate> new_predicates;
  std::vector<plansys2::Predicate> remove_predicates;
  std::map<std::string, plansys2::Predicate> qa_predicates;
  for (const auto & status : msg.status) {
    if (status.message == "Component status") {
      for (const auto & value : status.values) {
        // The domain file spells this constant "ERROR_string", but the PDDL
        // parser lowercases identifiers while reading domain/problem files
        // (see plansys2_pddl_parser's Stringreader) whereas the client-side
        // parser::pddl::fromStringPredicate() used here does not -- it keeps
        // whatever case we write. isValidPredicate() compares this argument
        // against the parser's already-lowercased constant table with a
        // case-sensitive ==, so this literal must stay lowercase to match.
        std::string pred_str = "(c_status " + value.key + " error_string)";
        auto predicate = parser::pddl::fromStringPredicate(pred_str);
        if (value.value == "OK" || value.value == "RECOVERED") {
          if (problem_expert_->existPredicate(predicate)) {
            remove_predicates.push_back(predicate);
          }
        } else if (value.value == "ERROR") {
          new_predicates.push_back(predicate);
        }
      }
    }
    if (status.message == "QA status") {
      for (const auto & value : status.values) {
        if (value.key == "battery_level" && !battery_tracking_supported_) {
          continue;
        }
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
  for (const auto & predicate : predicates) {
    if (predicate.name == "qa_has_value") {
      for (const auto &[qa_key, obs_name] : qa_keys) {
        if (qa_predicates.count(qa_key) && predicate.parameters[0].name == obs_name) {
          if (qa_predicates[qa_key] != predicate) {
            remove_predicates.push_back(predicate);
            new_predicates.push_back(qa_predicates[qa_key]);
          }
          if (obs_name == "obs_water_visibility") {
            had_obs_wv = true;
          }
          if (obs_name == "obs_battery_level") {
            had_obs_bl = true;
          }
        }
      }
    }
    if (had_obs_wv && had_obs_bl) {
      break;
    }
  }

  if (!had_obs_wv && qa_predicates.count("water_visibility")) {
    new_predicates.push_back(qa_predicates["water_visibility"]);
  }

  if (!had_obs_bl && qa_predicates.count("battery_level")) {
    new_predicates.push_back(qa_predicates["battery_level"]);
  }

  bool diagnostics_applied = true;
  if (!new_predicates.empty() || !remove_predicates.empty()) {
    diagnostics_applied = problem_expert_->updatePredicates(new_predicates, remove_predicates);
  }
  if (!initial_water_visibility_applied_ && qa_predicates.count("water_visibility")) {
    if (!diagnostics_applied) {
      throw std::runtime_error("Failed to apply initial water visibility to the planning problem");
    }
    initial_water_visibility_applied_ = true;
    RCLCPP_INFO(get_logger(), "Initial water visibility applied to the planning problem");
  }
}

std::vector<plansys2::Predicate> SuavePlansysController::add_symbolic_number(
  const std::string & number_str)
{
  std::vector<plansys2::Predicate> new_predicates;
  if (!numbers_added_.insert(number_str).second) {
    return new_predicates;
  }
  numbers_added_.insert(number_str);

  float number_float = std::stof(number_str);
  std::string number_decimal = number_str + "_decimal";

  problem_expert_->addInstance(
    parser::pddl::fromStringParam(
      number_decimal, "numerical-object"));

  static const std::vector<std::pair<float, std::string>> thresholds = {
    {0.25f, "0.25_decimal"}, {0.5f, "0.5_decimal"}, {0.75f, "0.75_decimal"}, {1.0f, "1.0_decimal"},
    {1.25f, "1.25_decimal"}, {2.25f, "2.25_decimal"}, {3.25f, "3.25_decimal"}};

  // Less than checks
  for (const auto &[t, t_str] : thresholds) {
    if (number_float < t) {
      new_predicates.push_back(
        parser::pddl::fromStringPredicate(
          "(lessthan " + number_decimal + " " + t_str + ")"));
    }
  }
  // Greater than checks
  for (const auto &[t, t_str] : thresholds) {
    if (number_float > t) {
      new_predicates.push_back(
        parser::pddl::fromStringPredicate(
          "(lessthan " + t_str + " " + number_decimal + ")"));
    }
  }
  return new_predicates;
}

SuavePlansysController::~SuavePlansysController()
{
}

bool SuavePlansysController::has_failed() const
{
  return controller_failed_.load();
}

bool SuavePlansysController::execute_plan()
{
  if (!initial_water_visibility_applied_) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Waiting for initial water visibility before planning");
    return false;
  }
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

    std::string goal_str = "Could not find plan to reach goal " + parser::pddl::toString(
      problem_expert_->getGoal());
    RCLCPP_INFO(get_logger(), "%s", goal_str.c_str());
    return false;
  }

  RCLCPP_INFO(get_logger(), "Selected plan: ");
  for (auto item : plan->items) {
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
}

void SuavePlansysController::step()
{
  std::optional<rclcpp::Time> guided_start;
  {
    std::lock_guard<std::mutex> lock(input_mutex_);
    guided_start.swap(pending_guided_start_);
  }
  if (guided_start.has_value()) {
    start_time_ = guided_start.value();
    guided_mode_ = true;
  }

  if (!plansys_is_ready()) {
    const auto startup_elapsed =
      (this->get_clock()->now() - controller_start_time_).seconds();
    if (startup_elapsed >= plansys_startup_timeout_) {
      report_controller_failure(
        "startup",
        "PlanSys2 lifecycle nodes did not become active within " +
        std::to_string(plansys_startup_timeout_) + " seconds");
    }
    return;
  }

  auto pending_diagnostics = take_pending_diagnostics();
  if (!pending_diagnostics.status.empty()) {
    apply_diagnostics(pending_diagnostics);
  }

  if (time_limit_reached()) {
    RCLCPP_INFO(get_logger(), "Time limit reached!");
    finish_controlling();
    return;
  }

  if (first_iteration_) {
    first_iteration_ = !execute_plan();
    return;
  }

  if (!executor_client_->execute_and_check_plan() && executor_client_->getResult()) {
    if (executor_client_->getResult().value().success) {
      RCLCPP_INFO(get_logger(), "Plan execution finished with success!");
      finish_controlling();
    } else {
      RCLCPP_INFO(get_logger(), "Replanning!");
      // Preserve the retry state when replacement planning or execution
      // cannot be started; otherwise the controller can wait forever for a
      // result that will never arrive.
      first_iteration_ = !execute_plan();
      return;
    }
  }
}

void SuavePlansysController::step_guarded() noexcept
{
  if (controller_failed_.load()) {
    return;
  }
  try {
    step();
  } catch (const std::exception & exception) {
    report_controller_failure("step", exception.what());
  } catch (...) {
    report_controller_failure("step", "unknown C++ exception");
  }
}

void SuavePlansysController::diagnostics_cb_guarded(
  const diagnostic_msgs::msg::DiagnosticArray & msg) noexcept
{
  try {
    diagnostics_cb(msg);
  } catch (const std::exception & exception) {
    report_controller_failure("diagnostics_cb", exception.what());
  } catch (...) {
    report_controller_failure("diagnostics_cb", "unknown C++ exception");
  }
}

void SuavePlansysController::mavros_state_cb_guarded(
  const mavros_msgs::msg::State & msg) noexcept
{
  try {
    mavros_state_cb(msg);
  } catch (const std::exception & exception) {
    report_controller_failure("mavros_state_cb", exception.what());
  } catch (...) {
    report_controller_failure("mavros_state_cb", "unknown C++ exception");
  }
}

bool SuavePlansysController::plansys_is_ready()
{
  if (plansys_ready_.load()) {
    return true;
  }
  request_plansys_readiness();
  return plansys_ready_.load();
}

void SuavePlansysController::request_plansys_readiness()
{
  const auto now = this->get_clock()->now();
  if (now < next_readiness_check_) {
    return;
  }

  for (const auto & client : lifecycle_state_clients_) {
    if (!client->service_is_ready()) {
      next_readiness_check_ = now + rclcpp::Duration(1, 0);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Waiting for PlanSys2 lifecycle services");
      return;
    }
  }

  std::uint64_t generation;
  {
    std::lock_guard<std::mutex> lock(readiness_mutex_);
    if (readiness_request_in_flight_) {
      if ((now - readiness_request_started_).seconds() <= 2.0) {
        return;
      }
      ++readiness_generation_;
      readiness_request_in_flight_ = false;
      RCLCPP_WARN(
        get_logger(), "Timed out checking PlanSys2 lifecycle states");
    }

    readiness_request_in_flight_ = true;
    readiness_request_started_ = now;
    readiness_responses_ = 0;
    readiness_all_active_ = true;
    generation = ++readiness_generation_;
  }

  next_readiness_check_ = now + rclcpp::Duration(1, 0);
  for (const auto & client : lifecycle_state_clients_) {
    auto request = std::make_shared<LifecycleGetState::Request>();
    client->async_send_request(
      request,
      [this, generation](
        rclcpp::Client<LifecycleGetState>::SharedFuture future)
      {
        lifecycle_state_cb(generation, future);
      });
  }
}

void SuavePlansysController::lifecycle_state_cb(
  std::uint64_t generation,
  rclcpp::Client<LifecycleGetState>::SharedFuture future) noexcept
{
  bool active = false;
  try {
    active = future.get()->current_state.id ==
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(
      get_logger(), "Failed to read a PlanSys2 lifecycle state: %s",
      exception.what());
  } catch (...) {
    RCLCPP_ERROR(
      get_logger(), "Failed to read a PlanSys2 lifecycle state");
  }

  bool became_ready = false;
  {
    std::lock_guard<std::mutex> lock(readiness_mutex_);
    if (!readiness_request_in_flight_ || generation != readiness_generation_) {
      return;
    }
    readiness_all_active_ = readiness_all_active_ && active;
    ++readiness_responses_;
    if (readiness_responses_ == lifecycle_state_clients_.size()) {
      became_ready = readiness_all_active_;
      plansys_ready_.store(readiness_all_active_);
      readiness_request_in_flight_ = false;
    }
  }

  if (became_ready) {
    RCLCPP_INFO(get_logger(), "All PlanSys2 lifecycle nodes are active");
  }
}

void SuavePlansysController::report_controller_failure(
  const std::string & callback, const std::string & reason) noexcept
{
  if (controller_failed_.exchange(true)) {
    return;
  }

  const auto message = "Unhandled controller failure in " + callback +
    ": " + reason;
  RCLCPP_FATAL(get_logger(), "%s", message.c_str());
  std::fprintf(stderr, "%s\n", message.c_str());
  std::fflush(stderr);

  if (step_timer_) {
    step_timer_->cancel();
  }

  try {
    std_msgs::msg::String failure;
    failure.data = message;
    failure_pub_->publish(failure);
  } catch (...) {
    std::fprintf(stderr, "Failed to publish controller failure\n");
    std::fflush(stderr);
  }

  rclcpp::shutdown();
}

bool SuavePlansysController::time_limit_reached()
{
  time_limit_ = get_parameter("time_limit").as_int();
  return guided_mode_ &&
         (get_clock()->now() - start_time_) >= rclcpp::Duration(time_limit_, 0);
}

bool SuavePlansysController::request_save_mission_results()
{
  while (!save_mission_results_cli->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service. Exiting.");
      return false;
    }
    RCLCPP_INFO(get_logger(), "mission_metrics/save service not available, waiting again...");
  }

  auto request = std::make_shared<std_srvs::srv::Empty::Request>();
  auto response = save_mission_results_cli->async_send_request(request);
  if (response.wait_for(1s) != std::future_status::ready) {
    RCLCPP_ERROR(get_logger(), "Failed to call service mission_metrics/save");
    return false;
  }
  return true;
}

void SuavePlansysController::mavros_state_cb(const mavros_msgs::msg::State & msg)
{
  if (msg.mode == "GUIDED" && !guided_start_received_.exchange(true)) {
    std::lock_guard<std::mutex> lock(input_mutex_);
    pending_guided_start_ = get_clock()->now();
  }
}

}  // namespace suave_planta

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<suave_planta::SuavePlansysController>(
    "mission_node");

  try {
    node->init();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    executor.remove_node(node);
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(
      node->get_logger(), "Controller initialization failed: %s",
      exception.what());
    std::fprintf(
      stderr, "Controller initialization failed: %s\n", exception.what());
    std::fflush(stderr);
    rclcpp::shutdown();
    return EXIT_FAILURE;
  } catch (...) {
    RCLCPP_FATAL(
      node->get_logger(),
      "Controller initialization failed: unknown exception");
    std::fprintf(
      stderr, "Controller initialization failed: unknown exception\n");
    std::fflush(stderr);
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }

  const auto exit_code = node->has_failed() ? EXIT_FAILURE : EXIT_SUCCESS;
  rclcpp::shutdown();
  return exit_code;
}
