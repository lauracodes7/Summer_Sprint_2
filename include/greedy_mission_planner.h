#ifndef GREEDY_MISSION_PLANNER_H
#define GREEDY_MISSION_PLANNER_H

#include "base_mission_planner.h"


class GreedyMissionPlanner : public BaseMissionPlanner {
public:
    using BaseMissionPlanner::BaseMissionPlanner;
    std::string GetNextWaypoint(const std::string& currentWaypoint, const std::string& goalWaypoint) override;
};


#endif // GREEDY_MISSION_PLANNER_H