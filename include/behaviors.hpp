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
#include <optional>

#include "adore_map/route.hpp"
#include "adore_ros2_msgs/msg/drivable_area.hpp"
#include "adore_ros2_msgs/msg/safety_corridor.hpp"
#include "adore_ros2_msgs/msg/traffic_signal.hpp"
#include "adore_ros2_msgs/msg/trajectory.hpp"
#include "adore_ros2_msgs/msg/vehicle_signals.hpp"

#include "dynamics/traffic_participant.hpp"
#include "dynamics/trajectory.hpp"
// #include "planning/trajectory_planner.hpp"
#include "adore_dynamics_conversions.hpp"
#include "adore_map_conversions.hpp"

#include "planning/analytical_planner.hpp"
#include "planning/drivable_area.hpp"
#include "planning/planning_helpers.hpp"
#include "planning/trajectory_planner.hpp"

namespace adore
{
namespace behavior
{
struct TrajectoryAndSignals
{
  adore_ros2_msgs::msg::Trajectory     trajectory;
  adore_ros2_msgs::msg::VehicleSignals signals;
  adore_ros2_msgs::msg::Route          modified_route;
  adore_ros2_msgs::msg::DrivableArea   drivable_area;
};

TrajectoryAndSignals driving_mission( planner::RouteSamplingPlanner& planner, const dynamics::VehicleStateDynamic& vehicle_state_dynamic,
                                      const map::Route& route, const dynamics::TrafficParticipantSet& traffic_participants,
                                      const std::map<size_t, adore_ros2_msgs::msg::TrafficSignal>& traffic_signals );

TrajectoryAndSignals driving_mission_following_reference( planner::RouteSamplingPlanner&       planner,
                                                          const dynamics::VehicleStateDynamic& vehicle_state_dynamic,
                                                          const dynamics::Trajectory&          reference_trajectory );

TrajectoryAndSignals waiting_for_mission( planner::RouteSamplingPlanner&         planner,
                                          const dynamics::VehicleStateDynamic&   vehicle_state_dynamic,
                                          const dynamics::TrafficParticipantSet& traffic_participants );

TrajectoryAndSignals remote_operations( planner::RouteSamplingPlanner& planner, const dynamics::VehicleStateDynamic& vehicle_state_dynamic,
                                        const map::Route& route, const dynamics::TrafficParticipantSet& traffic_participants,
                                        const bool&                                approved_to_drive_suggested_remote_operations,
                                        const std::optional<dynamics::Trajectory>& suggested_remote_operator_trajectory );

TrajectoryAndSignals minimum_risk( planner::RouteSamplingPlanner& planner, const dynamics::VehicleStateDynamic& vehicle_state_dynamic,
                                   const map::Route& route, const dynamics::TrafficParticipantSet& traffic_participants );

TrajectoryAndSignals avoiding_safety_corridor( planner::RouteSamplingPlanner&              planner,
                                               const dynamics::VehicleStateDynamic&        vehicle_state_dynamic,
                                               const dynamics::TrafficParticipantSet&      traffic_participants,
                                               const adore_ros2_msgs::msg::SafetyCorridor& safety_corridor );

TrajectoryAndSignals emergency( planner::RouteSamplingPlanner&                      planner,
                                const std::optional<dynamics::VehicleStateDynamic>& vehicle_state_dynamic );

} // namespace behavior
} // namespace adore
