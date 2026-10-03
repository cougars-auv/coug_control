# Copyright 2026 BYU FROST Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from typing import Any

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.action import Action
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitution import Substitution
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node


def agent_frame(agent_ns: str | Substitution, frame: str) -> PythonExpression:
    return PythonExpression(["'", agent_ns, f"/{frame}' if '", agent_ns, f"' != '' else '{frame}'"])


def is_agent(agent_ns: LaunchConfiguration, *names: str) -> PythonExpression:
    return PythonExpression(["'", agent_ns, "' in ", str(names)])


def launch_setup(context: LaunchContext, *args: Any, **kwargs: Any) -> list[Action]:
    use_sim_time = LaunchConfiguration("use_sim_time")
    agent_ns = LaunchConfiguration("agent_ns")

    scenario_param_path = LaunchConfiguration("scenario_param_file").perform(context)

    mavros_dir = get_package_share_directory("mavros")

    apm_config_file = os.path.join(mavros_dir, "launch", "apm_config.yaml")

    fleet_param_file = PathJoinSubstitution(
        [EnvironmentVariable("CONFIG_DIR"), "fleet", "coug_control_params.yaml"]
    )
    agent_param_file = PathJoinSubstitution(
        [EnvironmentVariable("CONFIG_DIR"), [agent_ns, "_params.yaml"]]
    )
    scenario_param_file = scenario_param_path or agent_param_file

    odom_frame = agent_frame(agent_ns, "odom")
    base_link_frame = agent_frame(agent_ns, "base_link")

    return [
        Node(
            package="coug_control",
            executable="hsd_mode_converter",
            name="hsd_mode_converter_node",
            parameters=[
                fleet_param_file,
                agent_param_file,
                scenario_param_file,
                {
                    "use_sim_time": use_sim_time,
                    "map_frame": "map",
                },
            ],
        ),
        Node(
            package="mavros",
            executable="mavros_node",
            condition=IfCondition(is_agent(agent_ns, "yboat1gz")),
            parameters=[
                apm_config_file,
                fleet_param_file,
                agent_param_file,
                scenario_param_file,
                {
                    "use_sim_time": use_sim_time,
                    "map_frame": "map",
                    "odom_frame": odom_frame,
                    "base_link_frame": base_link_frame,
                },
            ],
        ),
        Node(
            package="topic_tools",
            executable="relay_field",
            name="cmd_vel_relay_node",
            condition=IfCondition(is_agent(agent_ns, "yboat1gz")),
            arguments=[
                "cmd_vel_out",
                "mavros/setpoint_raw/local",
                "mavros_msgs/msg/PositionTarget",
                (
                    "{coordinate_frame: 8, type_mask: 1479, "
                    "velocity: {x: m.twist.linear.x}, yaw_rate: m.twist.angular.z}"
                ),
                "--wait-for-start",
            ],
            parameters=[{"use_sim_time": use_sim_time}],
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
            ),
            DeclareLaunchArgument(
                "agent_ns",
                default_value="auv0",
            ),
            DeclareLaunchArgument(
                "scenario_param_file",
                default_value="",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
