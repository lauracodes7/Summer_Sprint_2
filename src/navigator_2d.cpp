#include "navigator_2d.h"


void Navigator2D::AddWaypoint(const std::string& name, const Point2D& position) 
{
    waypoints_[name] = position;
}


double Navigator2D::GetHeadingToWaypoint(const Point2D& current_position, const std::string& waypoint_name) 
{
    auto it = waypoints_.find(waypoint_name);
    if (it == waypoints_.end())
    {
        throw std::invalid_argument("[Navigator 2D] Invalid waypoint name provided.");
    }

    double heading = (180/M_PI) * atan2(it->second.y - current_position.y, it->second.x - current_position.x);

    return heading;
}


double Navigator2D::GetDistanceToWaypoint(const Point2D& current_position, const std::string& waypoint_name) 
{
    // Validate waypoint name
    auto it = waypoints_.find(waypoint_name);
    if (it == waypoints_.end())
    {
        throw std::invalid_argument("[Navigator 2D] Invalid waypoint name provided.");
    }
    
    double distance = sqrt(pow(it->second.x - current_position.x, 2) + pow(it->second.y - current_position.y, 2));

    return distance;
}


std::string Navigator2D::GetClosestWaypointName(const Point2D& current_position) 
{

    double shortestDistance = MAXFLOAT;
    std::string closestWaypointName = "";

    for (auto it = waypoints_.begin(); it != waypoints_.end(); it++)
    {
        double waypointDistance = GetDistanceToWaypoint(current_position, it->first);
        if (waypointDistance < shortestDistance)
        {
            shortestDistance = waypointDistance;
            closestWaypointName = it->first;
        }
    }

    return closestWaypointName;
}


Point2D Navigator2D::GetWaypointPosition(const std::string& name) const
{
    auto it = waypoints_.find(name);
    if (it == waypoints_.end())
    {        
        throw std::invalid_argument("[Navigator 2D] Invalid waypoint name provided.");
    }

    return it->second;
}


double Navigator2D::CollectWaypointPower(const std::string& waypoint_name)
{
    auto it = waypoints_.find(waypoint_name);
    if (it == waypoints_.end())
    {
        throw std::invalid_argument("[Navigator 2D] Invalid waypoint name provided.");
    }

    // Simulate power collection from the waypoint
    double collectedPower = it->second.power;
    it->second.power = 0;
    std::cout << "[Navigator 2D] Collected " << collectedPower << " power from waypoint: " << waypoint_name << std::endl;

    return collectedPower;
}


std::vector<std::string> Navigator2D::GetAllWaypointNames() const
{
    std::vector<std::string> names;
    for (const auto& entry : waypoints_)
    {
        names.push_back(entry.first);
    }
    return names;
}
