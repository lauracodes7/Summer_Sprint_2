#ifndef RANDOM_MISSION_PLANNER_H
#define RANDOM_MISSION_PLANNER_H

#include "base_mission_planner.h"
#include "navigator_2d.h"
#include <random>



class RandomMissionPlanner : public BaseMissionPlanner
{
public:
    RandomMissionPlanner(Point2D startPosition, BaseCostEstimator* costEstimator, Navigator2D* navigator)
        : BaseMissionPlanner(startPosition, costEstimator, navigator) { }

    virtual std::string GetNextWaypoint(const std::string& currentWaypoint, const std::string& goalWaypoint) override
    {
        // If we can reach the goal, go for it
        double goalCost = costEstimator_->CalculateCost(currentWaypoint, goalWaypoint);
        if (goalCost <= currentPower_)
        {
            return goalWaypoint;
        }
        // Gather all valid options (not the current one)
        std::vector<std::string> options;
        for (const auto& name : navigator_->GetAllWaypointNames())
        {
            if (name != currentWaypoint)
            {
                double cost = costEstimator_->CalculateCost(currentWaypoint, name);
                if (cost <= currentPower_)  // Only go to reachable options
                {
                    options.push_back(name);
                }
            }
        }

        // If no reachable options, return empty (dead)
        if (options.empty())
        {
            return "";
        }

        // Choose randomly
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> dist(0, options.size() - 1);

        return options[dist(gen)];
    }

};

#endif // RANDOM_MISSION_PLANNER_H