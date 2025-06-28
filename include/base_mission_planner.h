#ifndef BASE_MISSION_PLANNER_H
#define BASE_MISSION_PLANNER_H

#include "base_cost_estimator.h"
#include <vector>
#include <string>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <chrono>

class BaseMissionPlanner 
{
public:

    BaseMissionPlanner(Point2D startPosition, BaseCostEstimator* costEstimator, Navigator2D* navigator) : currentPosition_(startPosition), currentWaypoint_(navigator->GetClosestWaypointName(startPosition)), costEstimator_(costEstimator), navigator_(navigator), currentPower_(startPosition.power) 
    {
        if (!costEstimator_ || !navigator_) 
        {
            throw std::runtime_error("Cost estimator and navigator must not be null.");
        }
    }

    virtual ~BaseMissionPlanner() = default;

    bool ExecuteMission(const std::string& endWaypoint)
    {
        missionPlan_.clear();
        missionPlan_.push_back(currentWaypoint_);   // Start with the initial waypoint

        while (currentWaypoint_ != endWaypoint)
        {
            std::cout << "[Vehicle] Position: (" << currentPosition_.x << ", " << currentPosition_.y << ") | Power: " << currentPower_ << std::endl;

            std::string nextWaypoint = GetNextWaypoint(currentWaypoint_, endWaypoint);

            if (nextWaypoint == "") 
            {
                std::cout << "[Mission Failed] No valid next waypoint found." << std::endl;
                return false; // No valid next waypoint, mission failed
            }

            double distance = navigator_->GetDistanceToWaypoint(currentPosition_, nextWaypoint);

            if (distance > currentPower_) 
            {
                std::cout << "[Mission Failed] Not enough power to reach " << nextWaypoint << std::endl;
                return false;
            }

            TravelTo(nextWaypoint);

            // Simulate delay proportional to travel distance
            // std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(distance * 100)));
        }

        std::cout << "[Mission Complete] Reached destination: " << endWaypoint << std::endl;
        return true;
    }

    void TravelTo(const std::string& waypointName)
    {
        double cost = costEstimator_->CalculateCost(currentWaypoint_, waypointName);

        std::cout << "Traveling from " << currentWaypoint_ << " to " << waypointName << " | Cost: " << cost << std::endl;

        currentPower_ -= cost;
        currentWaypoint_ = waypointName;
        currentPosition_ = navigator_->GetWaypointPosition(waypointName);

        missionPlan_.push_back(waypointName);

        // Recharge at the waypoint
        double recharge = navigator_->CollectWaypointPower(waypointName);
        currentPower_ += recharge;
        std::cout << "Recharged +" << recharge << " power at " << waypointName << ". New power: " << currentPower_ << std::endl;
    }

    std::vector<std::string> GetFullMissionTrace() const 
    {
        return missionPlan_;
    }

    virtual std::string GetNextWaypoint(const std::string& currentWaypoint, const std::string& goalWaypoint) = 0;

protected:
    
    Point2D currentPosition_;
    
    std::string currentWaypoint_;
    
    double currentPower_;

    BaseCostEstimator* costEstimator_;
    
    Navigator2D* navigator_;
    
    std::vector<std::string> missionPlan_;
    
};


#endif // BASE_MISSION_PLANNER_H