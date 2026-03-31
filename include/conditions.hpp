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
#include <array>
#include <cstdint>

#include "adore_dynamics_conversions.hpp"
#include "adore_ros2_msgs/msg/safety_corridor.hpp"

namespace adore
{
namespace conditions
{
const size_t MININUM_REFERENCE_TRAJECTORY_SIZE         = 5;
const double MAXIMUM_VEHICLE_STATE_DYNAMIC_AGE_SECONDS = 1.0;
const double MAXIMUM_REFERENCE_TRAJECTORY_AGE_SECONDS  = 1.0;
const size_t MINIMUM_ROUTE_LENGHTH_METERS              = 20;

// @TODO, improve these two functions, they are too simple currently
bool can_drive_mission( const std::optional<dynamics::VehicleStateDynamic>& vehicle_state_dynamic, const double& time_now );
bool has_mission( const std::optional<dynamics::VehicleStateDynamic>& vehicle_state_dynamic, const std::optional<map::Route>& route );
bool need_remote_operator_assitance( const std::optional<dynamics::VehicleStateDynamic>& vehicle_state_dynamic,
                                     const std::map<std::string, math::Polygon2d>&       caution_zones );
bool needs_to_avoid_safety_corridor( const std::optional<dynamics::VehicleStateDynamic>&        vehicle_state_dynamic,
                                     const std::optional<adore_ros2_msgs::msg::SafetyCorridor>& safety_corridor );

// @TODO, consider making this (inside MAD area or similar instead)
bool has_valid_remote_reference_trajectory( const std::optional<dynamics::VehicleStateDynamic>& vehicle_state_dynamic,
                                            const std::optional<dynamics::Trajectory>&          reference_trajectory );

// @TODO, needs a condition and behavior for inside of validity area, but no reference, so it can used the extended perception

} // namespace conditions
} // namespace adore
