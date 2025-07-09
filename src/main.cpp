// External includes
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <random>
#include <unordered_map>
#include <chrono>


// Internal Includes
#include "random_mission_planner.h"
#include "distance_cost_estimator.h"


void RenderEnvironment(Navigator2D& navigator, const std::vector<std::string>& missionTrace, std::string name = "")
{
    cv::destroyAllWindows(); // Clear any previous windows
    
    const int imgSize = 1200;
    const int padding = 50;
    const int scale = 20;  // pixels per unit
    const cv::Scalar backgroundColor(100, 100, 100); // Gray
    const cv::Scalar waypointColor(255, 165, 0);  // Orange
    const cv::Scalar startColor(0, 255, 0);  // Green
    const cv::Scalar goalColor(0, 255, 255);   // Yellow
    const cv::Scalar pathColor(0, 0, 255);   // Blue

    // Create blank canvas
    cv::Mat image(imgSize, imgSize, CV_8UC3, backgroundColor);

    auto getPixelCoord = [&](const Point2D& pt) {
        int x = static_cast<int>(pt.x * scale + padding);
        int y = static_cast<int>(pt.y * scale + padding);
        return cv::Point(x, imgSize - y); // Flip Y for display
    };

    // Draw all waypoints
    for (const auto& name : navigator.GetAllWaypointNames())
    {
        Point2D pos = navigator.GetWaypointPosition(name);
        cv::Point pt = getPixelCoord(pos);

        // Choose special colors for Start/Goal
        cv::Scalar color = waypointColor;
        if (name == "START") color = startColor;
        else if (name == "GOAL") color = goalColor;

        cv::circle(image, pt, 10, color, -1);
        cv::putText(image, name, pt + cv::Point(6, -6), cv::FONT_HERSHEY_SIMPLEX, 0.8, color, 1);
    }

    // Draw path
    if (missionTrace.size() >= 2)
    {
        for (size_t i = 1; i < missionTrace.size(); ++i)
        {
            Point2D from = navigator.GetWaypointPosition(missionTrace[i - 1]);
            Point2D to = navigator.GetWaypointPosition(missionTrace[i]);
            cv::line(image, getPixelCoord(from), getPixelCoord(to), pathColor, 2);
        }
    }

    // Show window
    cv::imshow("Mission Environment: " + name, image);
    cv::waitKey(0);
}


void AddRandomWaypoints(Navigator2D& navigator, int numWaypoints, unsigned int seed,
                        double xMin = 0, double xMax = 50,
                        double yMin = 0, double yMax = 50,
                        double powerMin = 5, double powerMax = 15)
{
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> distX(xMin, xMax);
    std::uniform_real_distribution<double> distY(yMin, yMax);
    std::uniform_real_distribution<double> distPower(powerMin, powerMax);

    for (int i = 0; i < numWaypoints; ++i)
    {
        std::ostringstream nameStream;
        nameStream << "WP" << i + 1;

        navigator.AddWaypoint(nameStream.str(), Point2D(distX(rng), distY(rng), distPower(rng)));
    }
}


int main()
{
    
    // navigator.AddWaypoint("A", Point2D(3, 4, 5));            
    // navigator.AddWaypoint("B", Point2D(6, 1, 2));         
    // navigator.AddWaypoint("C", Point2D(7, 5, 4));            
    // navigator.AddWaypoint("D", Point2D(10, 3, 5));           
    
    // navigator.AddWaypoint("E", Point2D(5, 10, 3));           
    // navigator.AddWaypoint("F", Point2D(13, 7, 5));   
    // navigator.AddWaypoint("G", Point2D(15, 5, 7));           
    // navigator.AddWaypoint("H", Point2D(17, 11, 8));         
    // navigator.AddWaypoint("I", Point2D(18, 18, 1));          
    // navigator.AddWaypoint("J", Point2D(12, 15, 6));
    
    
    
    
    unsigned int runs = 10000;
    std::unordered_map<std::string, unsigned int> successCount;
    successCount["random"] = 0;

    std::unordered_map<std::string, double> totalTimeMs;
    



    for (unsigned int i = 0; i < runs; ++i) 
    {
        Navigator2D navigator;
        AddRandomWaypoints(navigator, 20, 1 + i);
        Point2D startPosition(2, 3, 15);
        Point2D goalPosition(47, 48, 0);
        navigator.AddWaypoint("START", startPosition);
        navigator.AddWaypoint("GOAL", goalPosition);

        std::unordered_map<std::string, std::shared_ptr<BaseMissionPlanner>> planners;
        planners["random"] = (std::make_shared<RandomMissionPlanner>(startPosition, new DistanceCostEstimator(navigator), &navigator));

        for (auto it = planners.begin(); it != planners.end(); ++it) 
        {
            auto startTime = std::chrono::high_resolution_clock::now();

            bool success = (it->second)->ExecuteMission("GOAL");

            auto endTime = std::chrono::high_resolution_clock::now();
            double durationMs = std::chrono::duration<double, std::milli>(endTime - startTime).count();
            totalTimeMs[it->first] += durationMs;

            if (!success) 
            {
                std::cout << "\033[31m[Mission Planner] Mission Failed: " << it->first << "\033[0m" << std::endl;
            } 
            else 
            {
                successCount[it->first]++;
                std::cout << "\033[32m[Mission Planner] Mission Success: " << it->first << "\033[0m" << std::endl;
            }

            RenderEnvironment(navigator, it->second->GetFullMissionTrace(), it->first + " Run " + std::to_string(i + 1));

        }
    }

    std::cout << "\n\n -----========== Success Rate ==========-----" << std::endl;
    for (const auto& pair : successCount)
    {
        std::cout << pair.first << ": " << pair.second << " / " << runs << " (" << static_cast<double>(pair.second) / runs * 100.0 << "%)" << std::endl;
    }

    std::cout << "\n\n -----========== Average Runtime (ms) ==========-----" << std::endl;
    for (const auto& pair : totalTimeMs)
    {
        double avgMs = pair.second / runs;
        std::cout << pair.first << ": " << avgMs << " ms" << std::endl;
    }


    return 0;
}