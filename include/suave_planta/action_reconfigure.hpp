// Copyright 2024 Gustavo Rezende Silva
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy at http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#ifndef SUAVE_PLANTA__RECONFIGURE_HPP_
#define SUAVE_PLANTA__RECONFIGURE_HPP_

#include <chrono>
#include <memory>
#include <string>

#include "plansys2_executor/ActionExecutorClient.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "system_modes_msgs/srv/change_mode.hpp"

namespace suave_planta
{
class Reconfigure : public plansys2::ActionExecutorClient
{
public:
  Reconfigure(const std::string & node_name, const std::chrono::nanoseconds & rate);

protected:
  void do_work() override;
  CallbackReturnT on_activate(const rclcpp_lifecycle::State & state) override;

private:
  using Clock = std::chrono::steady_clock;
  using ChangeMode = system_modes_msgs::srv::ChangeMode;
  using GetState = lifecycle_msgs::srv::GetState;

  std::string system_;
  std::string node_;
  Clock::time_point deadline_;
  bool requested_ = false;
  bool expect_active_ = false;
  rclcpp::Client<ChangeMode>::SharedPtr change_mode_client_;
  rclcpp::Client<GetState>::SharedPtr get_state_client_;
  std::shared_ptr<rclcpp::Client<ChangeMode>::FutureAndRequestId> change_mode_future_;
  std::shared_ptr<rclcpp::Client<GetState>::FutureAndRequestId> get_state_future_;
};
}  // namespace suave_planta
#endif
