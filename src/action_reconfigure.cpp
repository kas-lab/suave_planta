// Copyright 2024 Gustavo Rezende Silva
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy at http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#include "suave_planta/action_reconfigure.hpp"

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

using namespace std::chrono_literals;
namespace suave_planta
{
using State = lifecycle_msgs::msg::State;

Reconfigure::Reconfigure(const std::string & node_name, const std::chrono::nanoseconds & rate)
: plansys2::ActionExecutorClient(node_name, rate)
{
  declare_parameter("reconfigure_timeout", 15.0);
}

Reconfigure::CallbackReturnT Reconfigure::on_activate(const rclcpp_lifecycle::State & state)
{
  const auto & args = get_arguments();
  system_ = args.front();
  node_ = system_ + "_node";
  const auto & mode = args.back();
  // The only mode strings suave_modes.yaml maps to an inactive child are
  // fd_unground (every function) and fd_all_thrusters (f_maintain_motion's
  // own inactive alias) -- everything else is some active submode.
  expect_active_ = mode != "fd_unground" && mode != "fd_all_thrusters";

  change_mode_client_ = create_client<ChangeMode>(system_ + "/change_mode");
  get_state_client_ = create_client<GetState>(node_ + "/get_state");
  get_state_future_.reset();
  requested_ = false;

  auto request = std::make_shared<ChangeMode::Request>();
  request->mode_name = mode;
  change_mode_future_ = std::make_shared<rclcpp::Client<ChangeMode>::FutureAndRequestId>(
    change_mode_client_->async_send_request(request));

  const double seconds = get_parameter("reconfigure_timeout").as_double();
  deadline_ = Clock::now() + std::chrono::duration_cast<Clock::duration>(
    std::chrono::duration<double>(seconds > 0.0 ? seconds : 1.0));

  return plansys2::ActionExecutorClient::on_activate(state);
}

void Reconfigure::do_work()
{
  if (Clock::now() >= deadline_) {
    finish(false, 0.0, "Reconfiguration timed out");
    return;
  }

  if (change_mode_future_) {
    if (change_mode_future_->wait_for(0s) != std::future_status::ready) {return;}
    auto response = change_mode_future_->get();
    change_mode_future_.reset();
    if (!response->success) {
      finish(false, 0.0, "change_mode rejected by " + system_);
      return;
    }
    requested_ = true;
    return;
  }

  if (!requested_) {return;}

  if (get_state_future_) {
    if (get_state_future_->wait_for(0s) != std::future_status::ready) {return;}
    auto response = get_state_future_->get();
    get_state_future_.reset();
    const auto id = response->current_state.id;
    const auto expected = expect_active_ ?
      State::PRIMARY_STATE_ACTIVE : State::PRIMARY_STATE_INACTIVE;
    if (id == expected) {
      finish(true, 1.0, "Reconfiguration completed");
    } else if (id == State::PRIMARY_STATE_ACTIVE || id == State::PRIMARY_STATE_INACTIVE) {
      // Settled, but not into the state this mode change asked for.
      finish(false, 0.0, "Managed node settled into an unexpected lifecycle state");
    }
    // Otherwise still transitioning: poll again next tick.
    return;
  }

  if (!get_state_client_->service_is_ready()) {return;}
  get_state_future_ = std::make_shared<rclcpp::Client<GetState>::FutureAndRequestId>(
    get_state_client_->async_send_request(std::make_shared<GetState::Request>()));
}
}  // namespace suave_planta

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<suave_planta::Reconfigure>("reconfigure", 100ms);
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
