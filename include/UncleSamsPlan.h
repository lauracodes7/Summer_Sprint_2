#ifndef UNCLE_SAMS_PLAN_H
#define UNCLE_SAMS_PLAN_H

#include "base_mission_planner.h"

class UncleSamsPlan : public BaseMissionPlanner {
public:

    UncleSamsPlan(Point2D startPosition, BaseCostEstimator* costEstimator, Navigator2D* navigator);

    virtual ~UncleSamsPlan() = default;

    // What I need to do
    std::string GetNextWaypoint(const std::string& currentWaypoint, const std::string& goalWaypoint) override;

};

#endif