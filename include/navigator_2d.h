#ifndef NAVIGATOR_2D_H
#define NAVIGATOR_2D_H

#include <string>
#include <vector>
#include <map>
#include <iostream>
#include <exception>
#include <cmath>



// Represents a 2D point in Cartesian space
struct Point2D 
{
    double x;
    double y;
    double power;

    Point2D(double x_val = 0, double y_val = 0, double power_val = 0) : x(x_val), y(y_val), power(power_val) { }
};


class Navigator2D 
{
public:

    Navigator2D() = default;
    
    ~Navigator2D() = default;

    // Adds a waypoint with a given name and position to the internal map
    void AddWaypoint(const std::string& name, const Point2D& position);

    // Returns the heading (angle in degrees from the x-axis) from the current position to the named waypoint
    double GetHeadingToWaypoint(const Point2D& current_position, const std::string& waypoint_name);

    // Returns the Euclidean distance from the current position to the named waypoint
    double GetDistanceToWaypoint(const Point2D& current_position, const std::string& waypoint_name);

    // Returns the name of the closest waypoint to the given position
    std::string GetClosestWaypointName(const Point2D& current_position);

    // Returns the position (Point2D) of the named waypoint
    Point2D GetWaypointPosition(const std::string& name) const;

    double CollectWaypointPower(const std::string& waypoint_name);

    std::vector<std::string> GetAllWaypointNames() const;

private:

    std::map<std::string, Point2D> waypoints_;

};


#endif // NAVIGATOR_2D_H