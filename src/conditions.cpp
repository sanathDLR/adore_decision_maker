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

#include "conditions.hpp"
#include "open_odd_ros2_msgs/msg/odd_evaluation.hpp"
#include <dynamics/vehicle_state.hpp>



namespace adore
{
namespace conditions
{


bool has_localization( 
                const std::optional<dynamics::VehicleStateDynamic>& vehicle_state_dynamic, 
                const double& time_now )
{
    if ( !vehicle_state_dynamic.has_value() )
        return false;

    if ( time_now - vehicle_state_dynamic.value().time > MAXIMUM_VEHICLE_STATE_DYNAMIC_AGE_SECONDS ) // If the message is more than one second old 
        return false;

    // @TODO, add covarianve of estimate

    return true;
}

bool has_mission( 
                const std::optional<dynamics::VehicleStateDynamic>& vehicle_state_dynamic, 
                const std::optional<map::Route>& route 
            )
{
    if( !route.has_value() || !vehicle_state_dynamic.has_value() )
        return false;

    double remaining = route->get_length() - route->get_s( *vehicle_state_dynamic );
    return remaining > MINIMUM_ROUTE_LENGHTH_METERS;
}

bool needs_to_avoid_safety_corridor(
                                        const std::optional<dynamics::VehicleStateDynamic>& vehicle_state_dynamic, 
                                        const std::optional<adore_ros2_msgs::msg::SafetyCorridor>& safety_corridor 
)
{
    if ( !vehicle_state_dynamic.has_value() || !safety_corridor.has_value() )
        return false;

    // @TODO, needs to do a check if it is inside of the safety corridor

    return true;
}

bool can_drive_managed( 
                        const std::optional<dynamics::VehicleStateDynamic>& vehicle_state_dynamic, 
                        const double& time_now, 
                        const std::optional<math::Polygon2d>& managed_zone, 
                        const std::optional<dynamics::Trajectory>& managed_trajectory)
{
    if (
        !vehicle_state_dynamic.has_value() ||
        !managed_zone.has_value() ||
        !managed_trajectory.has_value()
    )
    {
        return false;
    }

    if ( !managed_zone.value().point_inside(vehicle_state_dynamic.value()) )
        return false;

    if( managed_trajectory.value().states.size() < MININUM_REFERENCE_TRAJECTORY_SIZE )
        return false;

    double age = vehicle_state_dynamic.value().time - managed_trajectory.value().states.front().time;
    return age <= MAXIMUM_REFERENCE_TRAJECTORY_AGE_SECONDS;
}

bool odd_conditions_satisfied( 
                               const std::optional<open_odd_ros2_msgs::msg::OddEvaluation>& odd, 
                               const double& time_now )
{
    if ( !odd.has_value() )
        return false;

    // if ( time_now - odd.value().time > MAXIMUM_ODD_AGE_SECONDS )
    //     return false;
    
    return odd.value().match;
}

bool road_completely_blocked(
                const std::optional<dynamics::VehicleStateDynamic>& vehicle_state_dynamic,
                const std::optional<map::Route>& route,
                const dynamics::TrafficParticipantSet& traffic_participants )
{
    if( !vehicle_state_dynamic.has_value() || !route.has_value() )
        return false;
    
    bool lane_blocked = false;
    double s_curr = route.value().get_s( vehicle_state_dynamic.value() );
    double object_distance = std::numeric_limits<double>::max();
    for( const auto& [id, participant] : traffic_participants.participants )
    {
      auto state = participant.state;
      if( state.vx > 0.2 )
        continue;

      double obj_s = route.value().get_s( state );

      double offset = adore::math::distance_2d( state, route.value().get_pose_at_s( obj_s ) );

      if( offset > 1.5 )
        continue;

      if( obj_s > s_curr )
      {
        double distance = obj_s - s_curr;

        if( distance < object_distance )
        {
          object_distance = distance;
        }
      }
    }
    // std::cerr << "object distance: " << object_distance << std::endl;
    
    // if( object_distance < 25.0 && vehicle_state_dynamic.value().vx < 0.1 )
    if( object_distance < 20.0 )
        lane_blocked = true;

    return lane_blocked;
}

bool remote_operations_is_available( const std::optional<adore_ros2_msgs::msg::RemoteOperationStatus>& remote_operation_status, const double& time_now )
{
    if ( !remote_operation_status.has_value() )
    {
        return false;
    }

    if ( time_now - remote_operation_status.value().timestamp > MAX_TIME_SINCE_LAST_REMOTE_OPERATION_STATUS )
    {
        return false;
    }

    return true;
}

bool performing_remote_operator_instrcutions( const std::optional<dynamics::Trajectory>& suggested_remote_operator_trajectory, const bool& remote_operator_wants_unstructured_driving ) 
{
    return (suggested_remote_operator_trajectory.has_value() || remote_operator_wants_unstructured_driving);
}

bool passenger_wants_vehicle_to_stop( const bool& passenger_emergency_stop, const bool& resume_ride_requested, const double& time_now )
{
    // @TODO, needs a time check to make sure the request is not super old

    if ( passenger_emergency_stop && !resume_ride_requested )
    {
        return true;
    }

    return false;
}

bool is_evacuating( const bool& evacuation_is_active, const bool& odd_conditions_satisfied, const bool& road_comletely_blocked, const bool& driving_remote_operator_instructions )
{
    return evacuation_is_active && odd_conditions_satisfied && !road_comletely_blocked && !driving_remote_operator_instructions;
}

} // namespace conditions
} // namespace adore
