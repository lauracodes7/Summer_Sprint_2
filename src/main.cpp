// External includes
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>


// Internal Includes
#include "random_mission_planner.h"
#include "distance_cost_estimator.h"


void RenderEnvironment(Navigator2D& navigator, const std::vector<std::string>& missionTrace)
{
    const int imgSize = 600;
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
    cv::imshow("Mission Environment", image);
    cv::waitKey(0);
}



int main()
{
    Navigator2D navigator;
    
    navigator.AddWaypoint("A", Point2D(3, 4, 5));            
    navigator.AddWaypoint("B", Point2D(6, 1, 2));         
    navigator.AddWaypoint("C", Point2D(7, 5, 4));            
    navigator.AddWaypoint("D", Point2D(10, 3, 5));           

    navigator.AddWaypoint("E", Point2D(5, 10, 3));           
    navigator.AddWaypoint("F", Point2D(13, 7, 5));   
    navigator.AddWaypoint("G", Point2D(15, 5, 7));           
    navigator.AddWaypoint("H", Point2D(17, 11, 8));         
    navigator.AddWaypoint("I", Point2D(18, 18, 1));          
    navigator.AddWaypoint("J", Point2D(12, 15, 6));

    Point2D startPosition(0, 0, 15);
    Point2D goalPosition(20, 20, 0);
    navigator.AddWaypoint("START", startPosition);
    navigator.AddWaypoint("GOAL", goalPosition);


    std::shared_ptr<BaseMissionPlanner> planner = std::make_shared<RandomMissionPlanner>(startPosition, new DistanceCostEstimator(navigator), &navigator);

    planner->ExecuteMission("GOAL");

    std::vector<std::string> missionTrace = planner->GetFullMissionTrace();
    std::cout << "Mission Trace: ";
    for (const auto& waypoint : missionTrace) 
    {
        std::cout << waypoint << " ";
    }

    RenderEnvironment(navigator, missionTrace);
}