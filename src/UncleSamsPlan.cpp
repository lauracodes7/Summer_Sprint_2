#include "UncleSamsPlan.h"
#include <algorithm>

UncleSamsPlan::UncleSamsPlan(Point2D startPosition, BaseCostEstimator* costEstimator, Navigator2D* navigator): BaseMissionPlanner(startPosition, costEstimator, navigator) {}

std::string UncleSamsPlan::GetNextWaypoint(const std::string& currentWaypoint, const std::string& goalWaypoint) {
    
    //Don't let it go too far
    double maxDistance = 20.0;
    // Collect all options
    std::vector<std::string> allWaypoints = navigator_->GetAllWaypointNames();
    
    // Check if we made it???
    if (currentWaypoint == goalWaypoint) return goalWaypoint;

    Point2D ImHere = navigator_->GetWaypointPosition(currentWaypoint);
    double currDistToGoal = navigator_->GetDistanceToWaypoint(ImHere, goalWaypoint);
    double bestScore = 0;
    std::string next = "";

    for (const auto& waypoint : allWaypoints) {
        if (waypoint == currentWaypoint) continue;
        
        Point2D waypointPos = navigator_->GetWaypointPosition(waypoint);

        // Choice stats
        double distance = navigator_->GetDistanceToWaypoint(ImHere, waypoint);
        double cost = costEstimator_->CalculateCost(currentWaypoint, waypoint);
        double gain = waypointPos.power;
        double netPower = currentPower_ - cost + gain;

        // Avoid Overall loss of power
        if (netPower < 0 || distance > maxDistance) continue;

        // Avoid going backwards
        double distToGoal = navigator_->GetDistanceToWaypoint(waypointPos, goalWaypoint);
        if (distToGoal >= currDistToGoal) continue;

        if (netPower > bestScore) {
            bestScore = netPower;
            next = waypoint;
        }
    }

    return next;
}