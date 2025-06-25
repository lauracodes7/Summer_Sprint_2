#include "base_mission_planner.h"
#include "base_cost_estimator.h"
#include "distance_cost_estimator.h"


int main()
{
    Navigator2D navigator;
    navigator.AddWaypoint("Start", Point2D(0, 0));
    
    
    // COMING SOON: Add more waypoints and 


    Point2D startPosition(0, 0, 10);
    navigator.AddWaypoint("START", startPosition);


    std::shared_ptr<BaseMissionPlanner> planner = std::make_shared<YourMissionPlanner>(startPosition, new TerrainCostEstimator(navigator), &navigator);

    planner->ExecuteMission("GOAL");

    std::vector<std::string> missionTrace = planner->GetFullMissionTrace();
    std::cout << "Mission Trace: ";
    for (const auto& waypoint : missionTrace) 
    {
        std::cout << waypoint << " ";
    }

}