/********************************************************************************
 * Copyright (c) 2025 Contributors to the Eclipse Foundation
 *
 * See the NOTICE file(s) distributed with this work for additional
 * information regarding copyright ownership.
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0
 *
 * SPDX-License-Identifier: EPL-2.0
 ********************************************************************************/

#pragma once

#include "adore_ros2_msgs/msg/caution_zone.hpp"
#include "adore_ros2_msgs/msg/traffic_participant.hpp"
#include "adore_ros2_msgs/msg/traffic_participant_set.hpp"

#include "behaviors.hpp"
#include "conditions.hpp"
#include "dynamics/comfort_settings.hpp"
#include "planning/drivable_area.hpp"
#include "planning/trajectory_planner.hpp"
#include "std_msgs/msg/bool.hpp"
#include <rclcpp/rclcpp.hpp>

namespace adore
{

class DecisionMaker : public rclcpp::Node
{
public:

  explicit DecisionMaker( const rclcpp::NodeOptions& opts );

private:

  // Driving subscribers
  rclcpp::Subscription<adore_ros2_msgs::msg::VehicleStateDynamic>::SharedPtr   subscriber_vehicle_state_dynamic;
  rclcpp::Subscription<adore_ros2_msgs::msg::Route>::SharedPtr                 subscriber_route;
  rclcpp::Subscription<adore_ros2_msgs::msg::TrafficParticipantSet>::SharedPtr subscriber_traffic_participants;

  // Vehicle subscribers
  rclcpp::Subscription<adore_ros2_msgs::msg::VehicleInfo>::SharedPtr subscriber_vehicle_info;

  // World subscribers
  rclcpp::Subscription<adore_ros2_msgs::msg::TrafficSignal>::SharedPtr  subscriber_traffic_signal;
  rclcpp::Subscription<adore_ros2_msgs::msg::SafetyCorridor>::SharedPtr subscriber_safety_corridor;
  rclcpp::Subscription<adore_ros2_msgs::msg::Trajectory>::SharedPtr     subscriber_reference_trajectory;

  // Remote operations subscribers
  rclcpp::Subscription<adore_ros2_msgs::msg::CautionZone>::SharedPtr subscriber_caution_zones;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr               subscriber_remote_operator_drive_approval;
  rclcpp::Subscription<adore_ros2_msgs::msg::Trajectory>::SharedPtr  subscriber_suggested_remote_operator_trajectory;
  // rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr       subscriber_automation_toggle;

  int v2x_id = 0;

  std::optional<adore_ros2_msgs::msg::VehicleInfo> latest_vehicle_info;

  rclcpp::Publisher<adore_ros2_msgs::msg::Trajectory>::SharedPtr         publisher_trajectory_decision;
  rclcpp::Publisher<adore_ros2_msgs::msg::TrafficParticipant>::SharedPtr publisher_v2x_traffic_participant;
  rclcpp::Publisher<adore_ros2_msgs::msg::Route>::SharedPtr              publisher_modified_route;
  rclcpp::Publisher<adore_ros2_msgs::msg::DrivableArea>::SharedPtr       publisher_drivable_area;

  // Planning
  planner::RouteSamplingPlanner              planner; // @TODO Think most of these can be removed
  dynamics::PhysicalVehicleParameters        physical_vehicle_parameters;
  std::shared_ptr<dynamics::ComfortSettings> comfort_settings;

  // Domain
  std::optional<dynamics::VehicleStateDynamic>          latest_vehicle_state_dynamic;
  std::optional<map::Route>                             latest_route;
  std::map<size_t, adore_ros2_msgs::msg::TrafficSignal> traffic_signals;
  std::optional<dynamics::Trajectory>                   suggested_remote_operator_trajectory; // A trajectory received by a remote operator
  bool                                                  remote_operator_drive_approval = false;
  std::optional<adore_ros2_msgs::msg::SafetyCorridor>   latest_safety_corridor;
  std::optional<dynamics::Trajectory>                   latest_reference_trajectory;

  dynamics::TrafficParticipantSet        traffic_participants;
  std::map<std::string, math::Polygon2d> caution_zones;

  // DecisionParams               params;
  rclcpp::TimerBase::SharedPtr timer;


  void load_parameters();
  void setup_subscribers();
  void setup_publishers();
  void timer_callback(); // main loop

  behavior::TrajectoryAndSignals           choose_and_plan_driving_behavior();
  adore_ros2_msgs::msg::TrafficParticipant make_default_participant();
};

} // namespace adore
