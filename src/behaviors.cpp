#include "behaviors.hpp"

namespace adore
{
namespace behavior
{
TrajectoryAndSignals
driving_mission( planner::RouteSamplingPlanner& planner, const dynamics::VehicleStateDynamic& vehicle_state_dynamic,
                 const map::Route& route, const dynamics::TrafficParticipantSet& traffic_participants,
                 const std::map<size_t, adore_ros2_msgs::msg::TrafficSignal>& traffic_signals )
{
  // Go through route, and update speed at points based on traffic signal positions
  auto route_with_signal = route;
  for( auto& p : route_with_signal.reference_line )
  {
    if( std::any_of( traffic_signals.begin(), traffic_signals.end(), [&]( const auto& s ) {
          return adore::math::distance_2d( s.second, p.second ) < 3.0 && s.second.state != adore_ros2_msgs::msg::TrafficSignal::GREEN;
        } ) )
      p.second.max_speed = 0;
  }

  // dynamics::Trajectory trajectory = planner.plan_route_trajectory( route_with_signal, vehicle_state_dynamic, traffic_participants );
  auto                 result             = planner.plan_route_trajectory( route_with_signal, vehicle_state_dynamic, traffic_participants );
  dynamics::Trajectory planned_trajectory = result.trajectory.value();
  planned_trajectory.adjust_start_time( vehicle_state_dynamic.time );
  planned_trajectory.label = "driving mission";

  TrajectoryAndSignals trajectory_and_signal;
  trajectory_and_signal.drivable_area  = map::conversions::to_ros_msg( result.drivable_area.value() );
  trajectory_and_signal.modified_route = map::conversions::to_ros_msg( result.modified_route.value() );
  trajectory_and_signal.trajectory     = dynamics::conversions::to_ros_msg( planned_trajectory );

  return trajectory_and_signal;
}

TrajectoryAndSignals
driving_mission_following_reference( planner::RouteSamplingPlanner& planner, const dynamics::VehicleStateDynamic& vehicle_state_dynamic,
                                     const dynamics::Trajectory& reference_trajectory )
{
  TrajectoryAndSignals trajectory_and_signal;
  trajectory_and_signal.trajectory       = dynamics::conversions::to_ros_msg( reference_trajectory );
  trajectory_and_signal.trajectory.label = "driving mission (following reference)";

  return trajectory_and_signal;
}

TrajectoryAndSignals
waiting_for_mission( planner::RouteSamplingPlanner& planner, const dynamics::VehicleStateDynamic& vehicle_state_dynamic,
                     const dynamics::TrafficParticipantSet& traffic_participants )
{
  dynamics::Trajectory trajectory;
  trajectory.label = "waiting for mission";

  trajectory.states.push_back( vehicle_state_dynamic );

  TrajectoryAndSignals trajectory_and_signal;
  trajectory_and_signal.trajectory = dynamics::conversions::to_ros_msg( trajectory );

  return trajectory_and_signal;
}

TrajectoryAndSignals
remote_operations( planner::RouteSamplingPlanner& planner, const dynamics::VehicleStateDynamic& vehicle_state_dynamic,
                   const map::Route& route, const dynamics::TrafficParticipantSet& traffic_participants,
                   const bool&                                approved_to_drive_suggested_remote_operations,
                   const std::optional<dynamics::Trajectory>& suggested_remote_operator_trajectory )
{
  TrajectoryAndSignals trajectory_and_signals;

  if( suggested_remote_operator_trajectory.has_value() && approved_to_drive_suggested_remote_operations )
  {
    dynamics::Trajectory trajectory = suggested_remote_operator_trajectory.value();
    trajectory.label                = "remote operations (driving remote operator instructions)";

    trajectory_and_signals.trajectory = dynamics::conversions::to_ros_msg( trajectory );
    return trajectory_and_signals;
  }

  trajectory_and_signals = minimum_risk( planner, vehicle_state_dynamic, route, traffic_participants );

  trajectory_and_signals.trajectory.label = "remote operations (waiting for remote operator instructions)";
  return trajectory_and_signals;
}

TrajectoryAndSignals
minimum_risk( planner::RouteSamplingPlanner& planner, const dynamics::VehicleStateDynamic& vehicle_state_dynamic, const map::Route& route,
              const dynamics::TrafficParticipantSet& traffic_participants )
{

  double state_s            = route.get_s( vehicle_state_dynamic ).value();
  auto   cut_route          = route.get_shortened_route( state_s, 100.0 );
  auto   planned_trajectory = planner::waypoints_to_trajectory( vehicle_state_dynamic, cut_route, traffic_participants,
                                                                dynamics::PhysicalVehicleModel( planner.get_physical_vehicle_parameters() ),
                                                                0.0 /* target_speed */ );

  // planned_trajectory = planning_tools.planner.optimize_trajectory( *domain.vehicle_state, planned_trajectory );
  if( planned_trajectory.states.size() < 2 )
  {
    planned_trajectory.states.push_back( vehicle_state_dynamic );
  }
  planned_trajectory.label = "Minimum Risk Maneuver";

  TrajectoryAndSignals trajectory_and_signals;
  trajectory_and_signals.trajectory = dynamics::conversions::to_ros_msg( planned_trajectory );

  return trajectory_and_signals;
}

TrajectoryAndSignals
avoiding_safety_corridor( planner::RouteSamplingPlanner& planner, const dynamics::VehicleStateDynamic& vehicle_state_dynamic,
                          const dynamics::TrafficParticipantSet&      traffic_participants,
                          const adore_ros2_msgs::msg::SafetyCorridor& safety_corridor )
{
  auto   right_forward_points = planner::filter_points_in_front( safety_corridor.right_border, vehicle_state_dynamic );
  auto   safety_waypoints     = planner::shift_points_right( right_forward_points, planner.get_physical_vehicle_parameters().body_width );
  double target_speed         = planner::is_point_to_right_of_line( vehicle_state_dynamic, right_forward_points ) ? 0 : 2.0;

  auto planned_trajectory = planner::waypoints_to_trajectory( vehicle_state_dynamic, safety_waypoints, traffic_participants,
                                                              dynamics::PhysicalVehicleModel( planner.get_physical_vehicle_parameters() ),
                                                              target_speed );

  planned_trajectory = planner.optimize_trajectory( vehicle_state_dynamic, planned_trajectory );

  planned_trajectory.label = "avoiding safety corridor";

  TrajectoryAndSignals trajectory_and_signals;
  trajectory_and_signals.trajectory = dynamics::conversions::to_ros_msg( planned_trajectory );

  return trajectory_and_signals;
}

TrajectoryAndSignals
emergency( planner::RouteSamplingPlanner& planner, const std::optional<dynamics::VehicleStateDynamic>& vehicle_state_dynamic )
{
  dynamics::Trajectory emergency_stop_trajectory;
  if( vehicle_state_dynamic.has_value() )
    emergency_stop_trajectory.states.push_back( vehicle_state_dynamic.value() );

  emergency_stop_trajectory.label = "emergency stop";

  TrajectoryAndSignals trajectory_and_signals;
  trajectory_and_signals.trajectory = dynamics::conversions::to_ros_msg( emergency_stop_trajectory );

  return trajectory_and_signals;
}
} // namespace behavior
} // namespace adore
