// Copyright 2023 Gustavo Rezende Silva
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
#include "plansys2_executor/ActionExecutorClient.hpp"
#include "std_msgs/msg/bool.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

class SearchPipelineAction : public plansys2::ActionExecutorClient{
public:
  SearchPipelineAction(const std::string & node_name,
    const std::chrono::nanoseconds & rate) : ActionExecutorClient(node_name, rate){
      pipeline_detection_sub_  = this->create_subscription<std_msgs::msg::Bool>(
        "/pipeline/detected",
        10,
        std::bind(&SearchPipelineAction::pipeline_detected_cb, this, _1));
  };

private:
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr pipeline_detection_sub_;

  bool _pipeline_detected = false;
  void pipeline_detected_cb(const std_msgs::msg::Bool &msg){
    _pipeline_detected = msg.data;
  };

  void do_work(){
    if(_pipeline_detected==true) finish(true, 1.0, "Pipeline found!");
  };
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SearchPipelineAction>(
    "search_pipeline", 100ms);

  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();

  rclcpp::shutdown();

  return 0;
}
