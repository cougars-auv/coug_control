// Copyright 2026 BYU FROST Lab
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

#pragma once

#include <map>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

#include "coug_control/mavros_plugin_config_parameters.hpp"

namespace coug_control {

class MavrosPluginConfigNode : public rclcpp::Node {
 public:
  explicit MavrosPluginConfigNode(const rclcpp::NodeOptions& options);

 private:
  struct PluginEntry {
    std::vector<rclcpp::Parameter> parameters;
    std::shared_ptr<rclcpp::AsyncParametersClient> client;
    bool pending{false};
    bool configured{false};
  };

  // --- Callbacks ---
  void tickCallback();

  // --- Helpers ---
  void configurePlugin(const std::string& name, PluginEntry& plugin);

  // --- ROS Interfaces ---
  rclcpp::TimerBase::SharedPtr tick_timer_;

  // --- Parameters ---
  std::shared_ptr<mavros_plugin_config_node::ParamListener> param_listener_;
  mavros_plugin_config_node::Params params_;

  // --- State ---
  std::map<std::string, PluginEntry> plugins_;
};

}  // namespace coug_control
