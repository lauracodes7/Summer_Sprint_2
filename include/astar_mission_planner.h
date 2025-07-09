#ifndef ASTAR_MISSION_PLANNER_H
#define ASTAR_MISSION_PLANNER_H

#include "base_mission_planner.h"
#include <vector>
#include <string>

class AStarMissionPlanner : public BaseMissionPlanner {
public:
    AStarMissionPlanner(Point2D startPosition, BaseCostEstimator* costEstimator, Navigator2D* navigator);

    std::string GetNextWaypoint(const std::string& currentWaypoint, const std::string& goalWaypoint) override;

private:
    double Heuristic(const std::string& from, const std::string& to);
};

#endif // ASTAR_MISSION_PLANNER_H
