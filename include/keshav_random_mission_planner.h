#ifndef KESHAV_RANDOM_MISSION_PLANNER_H
#define KESHAV_RANDOM_MISSION_PLANNER_H
#include "base_cost_estimator.h"
#include "distance_cost_estimator.h"
#include "base_mission_planner.h"
#include "navigator_2d.h"
#include <vector>
#include <string>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <chrono>

class RandomMissionPlanner : public BaseMissionPlanner
{
public:

    RandomMissionPlanner(Point2D startPosition, BaseCostEstimator* costEstimator, Navigator2D* navigator)
        : BaseMissionPlanner(startPosition, costEstimator, navigator) {}

    ~RandomMissionPlanner()  = default;

    std::string GetNextWaypoint(const std::string& currentWaypoint, const std::string& goalWaypoint)
    {

        const auto& allWaypoints = navigator_->GetAllWaypointNames();

        std::string bestWaypoint = "";
        double bestScore = -1e9;

        for (const auto& candidate : allWaypoints) 
        {
            if (candidate == currentWaypoint || candidate == "START") continue;

            if (std::find(missionPlan_.begin(), missionPlan_.end(), candidate) != missionPlan_.end()) 
                continue;
               

            double costToCandidate = costEstimator_->CalculateCost(currentWaypoint, candidate);
            
            if (costToCandidate > currentPower_) continue;

            double distToGoal = costEstimator_->CalculateCost(candidate, goalWaypoint);
            double score = costToCandidate-distToGoal;

            if (score > bestScore) 
            {
                bestScore = score;
                bestWaypoint = candidate;
            }
        }
        return bestWaypoint;
    }
};


#endif // KESHAV_RANDOM_MISSION_PLANNER_H