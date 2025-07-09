#ifndef PATHFINDER_MISSION_PLANNER_H
#define PATHFINDER_MISSION_PLANNER_H


#include "base_mission_planner.h"


class PathfinderMissionPlanner : public BaseMissionPlanner
{
public:

    // Constructor that initializes the base class with a Navigator2D instance
    PathfinderMissionPlanner(Point2D startPosition, BaseCostEstimator* costEstimator, Navigator2D* navigator);

    std::string GetNextWaypoint(const std::string& currentWaypoint, const std::string& goalWaypoint) override;

    std::vector<std::string> PanicSearch(const std::string& currWaypoint, const std::string& goal);
    
};


#endif // PATHFINDER_MISSION_PLANNER.H