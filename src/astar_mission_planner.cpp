#include "astar_mission_planner.h"
#include <queue>
#include <set>
#include <cmath>

struct AStar {
    std::string waypoint; // Current waypoint
    double power; // Remaining power
    double cost; // Cost from start to this waypoint
    double estimatedTotalCost; // Estimated total cost to reach the goal
    std::vector<std::string> path; // Path taken to reach this waypoint

    bool operator>(const AStar& other) const {
        return estimatedTotalCost > other.estimatedTotalCost;
    } // For priority queue to sort by estimated total cost
};

AStarMissionPlanner::AStarMissionPlanner(Point2D startPosition, BaseCostEstimator* costEstimator, Navigator2D* navigator)
    : BaseMissionPlanner(startPosition, costEstimator, navigator) {} // Constructor initializes the base class with start position, cost estimator, and navigator

    double AStarMissionPlanner::Heuristic(const std::string& from, const std::string& to) { // Heuristic function to estimate the cost from one waypoint to another
    Point2D posFrom = navigator_->GetWaypointPosition(from); // Get the position of the starting waypoint
    Point2D posTo = navigator_->GetWaypointPosition(to); // Get the position of the goal waypoint
    double dx = posFrom.x - posTo.x; // Calculate the difference in x-coordinates
    double dy = posFrom.y - posTo.y; // Calculate the difference in y-coordinates
    return std::sqrt(dx * dx + dy * dy); // Return the Euclidean distance as the heuristic cost
}
std::string AStarMissionPlanner::GetNextWaypoint(const std::string& currentWaypoint, const std::string& goalWaypoint) {
    using State = AStar;
    std::priority_queue<State, std::vector<State>, std::greater<State>> openSet; //
    std::set<std::pair<std::string, int>> visited;    // Set to track visited waypoints with power level

    openSet.push({currentWaypoint, currentPower_, 0.0, Heuristic(currentWaypoint, goalWaypoint), {currentWaypoint}}); // Push initial state

    while (!openSet.empty()) { // While there are states to explore
        State curr = openSet.top(); openSet.pop(); // Get the state with the lowest estimated total cost

        std::string wp = curr.waypoint; // Current waypoint
        int key_power = static_cast<int>(curr.power * 10); // Use power as a key for visited states

        if (visited.count({wp, key_power})) continue; // If this state has already been visited with the same power, skip it
        visited.insert({wp, key_power}); // Mark this state as visited

        if (wp == goalWaypoint) { // If we reached the goal waypoint
            if (curr.path.size() >= 2) {
                return curr.path[1]; // Return the next waypoint in the path after the current one
            } else {
                return ""; // If the path is too short, return an empty string
            }
        }

        for (const auto& neighbor : navigator_->GetAllWaypointNames()) { // Iterate through all neighbors of the current waypoint
            if (neighbor == wp) continue; // Skip the current waypoint itself

            double moveCost = costEstimator_->CalculateCost(wp, neighbor);
            if (moveCost > curr.power) continue; // If the move cost exceeds remaining power, skip this neighbor

            double newPower = curr.power - moveCost + navigator_->GetWaypointPosition(neighbor).power; // Calculate new power after moving to the neighbor
            double cost = curr.cost + moveCost; // Update cost to reach this neighbor
            double heuristic = Heuristic(neighbor, goalWaypoint); // Calculate heuristic cost from neighbor to goal waypoint
            double totalCost = cost + heuristic; // Total estimated cost to reach the goal from this neighbor

            std::vector<std::string> newPath = curr.path; // Create a new path including the current waypoint and the neighbor
            newPath.push_back(neighbor); // Add the neighbor to the path

            openSet.push({neighbor, newPower, cost, totalCost, newPath}); // Push the new state into the priority queue
        }
    }

    return ""; // If no path is found, return an empty string
}


