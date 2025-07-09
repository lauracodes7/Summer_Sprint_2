#include "greedy_mission_planner.h"


std::string GreedyMissionPlanner::GetNextWaypoint(const std::string& currentWaypoint, const std::string& goalWaypoint) {
    
    const std::vector<std::string> waypoints = navigator_->GetAllWaypointNames();
    std::string nextWaypoint = "";
    double shortestDistanceToGoal = std::numeric_limits<double>::max();

    // determine viable waypoints
    for (const std::string& possibleWaypoint : waypoints) {
        const double waypointCost = costEstimator_->CalculateCost(currentWaypoint, possibleWaypoint);
        
        // filter current waypoint
        if (possibleWaypoint == currentWaypoint) {
            continue;
        }

        // filter inaccessible waypoints
        if (waypointCost > currentPower_) {
            continue;
        }

        // calculate euclidean distance from possible waypoint to goal waypoint
        const Point2D waypointPosition = navigator_->GetWaypointPosition(possibleWaypoint);
        const double distanceToGoal = navigator_->GetDistanceToWaypoint(waypointPosition, goalWaypoint);

        // pick move that most reduces distance to goal
        if (distanceToGoal < shortestDistanceToGoal) {
            shortestDistanceToGoal = distanceToGoal;
            nextWaypoint = possibleWaypoint;
        }

    }

    return nextWaypoint;
}