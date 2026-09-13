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
#include <atomic>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
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
  explicit SuavePlansysController(const std::string & node_name);

  virtual ~SuavePlansysController();

  void init();

  bool has_failed() const;

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

  bool battery_charged_ = true;
  bool initial_water_visibility_applied_ = false;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_sub_;
  rclcpp::CallbackGroup::SharedPtr diagnostics_sub_cb_group_;

  void diagnostics_cb(const diagnostic_msgs::msg::DiagnosticArray & msg);
  void apply_diagnostics(const diagnostic_msgs::msg::DiagnosticArray & msg);
  diagnostic_msgs::msg::DiagnosticArray take_pending_diagnostics();

  std::unordered_set<std::string> numbers_added_;
  std::vector<plansys2::Predicate> add_symbolic_number(const std::string & number);

  using LifecycleGetState = lifecycle_msgs::srv::GetState;
  using DiagnosticKey = std::pair<std::string, std::string>;

  std::mutex input_mutex_;
  std::map<DiagnosticKey, std::string> pending_diagnostic_values_;
  std::optional<rclcpp::Time> pending_guided_start_;
  std::atomic_bool guided_start_received_{false};

  rclcpp::CallbackGroup::SharedPtr lifecycle_cb_group_;
  std::vector<rclcpp::Client<LifecycleGetState>::SharedPtr>
  lifecycle_state_clients_;
  std::mutex readiness_mutex_;
  bool readiness_request_in_flight_ = false;
  std::size_t readiness_responses_ = 0;
  bool readiness_all_active_ = false;
  std::uint64_t readiness_generation_ = 0;
  std::atomic_bool plansys_ready_{false};
  rclcpp::Time readiness_request_started_;
  rclcpp::Time next_readiness_check_;
  rclcpp::Time controller_start_time_;
  double plansys_startup_timeout_ = 30.0;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr failure_pub_;
  std::atomic_bool controller_failed_{false};

  void step();
  void step_guarded() noexcept;
  void diagnostics_cb_guarded(
    const diagnostic_msgs::msg::DiagnosticArray & msg) noexcept;
  void mavros_state_cb_guarded(const mavros_msgs::msg::State & msg) noexcept;
  bool plansys_is_ready();
  void request_plansys_readiness();
  void lifecycle_state_cb(
    std::uint64_t generation,
    rclcpp::Client<LifecycleGetState>::SharedFuture future) noexcept;
  void report_controller_failure(
    const std::string & callback, const std::string & reason) noexcept;
  void finish_controlling();
  bool time_limit_reached();
  bool request_save_mission_results();
  void mavros_state_cb(const mavros_msgs::msg::State & msg);
};

}  // namespace suave_planta

#endif  // SUAVE_PLANTA__SUAVE_PLANTA_CONTROLLER_HPP_
