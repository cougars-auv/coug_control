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

#include "coug_control/mavros_plugin_config.hpp"

#include <algorithm>
#include <chrono>
#include <future>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <string>
#include <vector>

#include "coug_control/mavros_plugin_config_parameters.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

namespace coug_control {

using rcl_interfaces::msg::SetParametersResult;

MavrosPluginConfigNode::MavrosPluginConfigNode(const rclcpp::NodeOptions& options)
    : Node("mavros_plugin_config_node",
           rclcpp::NodeOptions(options).automatically_declare_parameters_from_overrides(true)) {
  param_listener_ =
      std::make_shared<mavros_plugin_config_node::ParamListener>(get_node_parameters_interface());
  params_ = param_listener_->get_params();

  for (const auto& name : list_parameters({}, 0).names) {
    const auto dot = name.find('.');
    if (dot == std::string::npos || name.rfind("qos_overrides.", 0) == 0) {
      continue;
    }
    const auto plugin_name = name.substr(0, dot);
    auto& plugin = plugins_[plugin_name];
    if (!plugin.client) {
      plugin.client =
          std::make_shared<rclcpp::AsyncParametersClient>(this, "mavros/" + plugin_name);
    }
    plugin.parameters.emplace_back(name.substr(dot + 1), get_parameter(name).get_parameter_value());
  }

  tick_timer_ = create_timer(std::chrono::duration<double>(params_.tick_period_sec),
                             [this] { tickCallback(); });

  RCLCPP_INFO(get_logger(), "Initialization complete.");
}

void MavrosPluginConfigNode::tickCallback() {
  for (auto& [name, plugin] : plugins_) {
    if (!plugin.configured && !plugin.pending && plugin.client->service_is_ready()) {
      configurePlugin(name, plugin);
    }
  }

  if (std::all_of(plugins_.begin(), plugins_.end(),
                  [](const auto& entry) { return entry.second.configured; })) {
    tick_timer_->cancel();
    RCLCPP_INFO(get_logger(), "All MAVROS plugins configured.");
  }
}

void MavrosPluginConfigNode::configurePlugin(const std::string& name, PluginEntry& plugin) {
  plugin.pending = true;
  plugin.client->set_parameters(
      plugin.parameters,
      [this, name, &plugin](const std::shared_future<std::vector<SetParametersResult>>& future) {
        plugin.pending = false;
        for (const auto& result : future.get()) {
          if (!result.successful) {
            RCLCPP_WARN(get_logger(), "Failed to configure MAVROS plugin '%s': %s.", name.c_str(),
                        result.reason.c_str());
            return;
          }
        }
        plugin.configured = true;
        RCLCPP_INFO(get_logger(), "Configured MAVROS plugin '%s'.", name.c_str());
      });
}

}  // namespace coug_control

RCLCPP_COMPONENTS_REGISTER_NODE(coug_control::MavrosPluginConfigNode)
