#include <gtest/gtest.h>
#include "navigator_2d.h"
#include <cmath>
#include <stdexcept>

constexpr double EPSILON = 1e-4;

// Helper: compare floating point values
bool AreEqual(double a, double b, double epsilon = EPSILON) {
    return std::abs(a - b) < epsilon;
}

TEST(Navigator2DTests, AddAndGetWaypointPosition) {
    Navigator2D nav;
    Point2D p(1.0, 2.0);
    nav.AddWaypoint("Test", p);
    Point2D result = nav.GetWaypointPosition("Test");
    EXPECT_TRUE(AreEqual(result.x, 1.0));
    EXPECT_TRUE(AreEqual(result.y, 2.0));
}

TEST(Navigator2DTests, GetWaypointPositionInvalid) {
    Navigator2D nav;
    EXPECT_THROW(nav.GetWaypointPosition("Missing"), std::invalid_argument);
}

TEST(Navigator2DTests, GetDistanceToWaypoint) {
    Navigator2D nav;
    nav.AddWaypoint("A", Point2D(3.0, 4.0)); // 5.0 distance from (0,0)
    Point2D current(0.0, 0.0);
    double dist = nav.GetDistanceToWaypoint(current, "A");
    EXPECT_TRUE(AreEqual(dist, 5.0));
}

TEST(Navigator2DTests, GetHeadingToWaypoint_Horizontal) {
    Navigator2D nav;
    nav.AddWaypoint("East", Point2D(5.0, 0.0));
    Point2D origin(0.0, 0.0);
    double heading = nav.GetHeadingToWaypoint(origin, "East");
    EXPECT_TRUE(AreEqual(heading, 0.0));
}

TEST(Navigator2DTests, GetHeadingToWaypoint_VerticalUp) {
    Navigator2D nav;
    nav.AddWaypoint("North", Point2D(0.0, 5.0));
    double heading = nav.GetHeadingToWaypoint(Point2D(0.0, 0.0), "North");
    EXPECT_TRUE(AreEqual(heading, 90.0));
}

TEST(Navigator2DTests, GetHeadingToWaypoint_VerticalDown) {
    Navigator2D nav;
    nav.AddWaypoint("South", Point2D(0.0, -5.0));
    double heading = nav.GetHeadingToWaypoint(Point2D(0.0, 0.0), "South");
    EXPECT_TRUE(AreEqual(heading, 270.0) || AreEqual(heading, -90.0)); // 270 or -90 degrees
}

TEST(Navigator2DTests, GetHeadingToWaypoint_Quadrant) {
    Navigator2D nav;
    nav.AddWaypoint("Q1", Point2D(3.0, 3.0));
    double heading = nav.GetHeadingToWaypoint(Point2D(0.0, 0.0), "Q1");
    EXPECT_TRUE(AreEqual(heading, 45.0));
}

TEST(Navigator2DTests, GetClosestWaypointName_Simple) {
    Navigator2D nav;
    nav.AddWaypoint("A", Point2D(1.0, 0.0));
    nav.AddWaypoint("B", Point2D(5.0, 0.0));
    std::string closest = nav.GetClosestWaypointName(Point2D(0.0, 0.0));
    EXPECT_EQ(closest, "A");
}

TEST(Navigator2DTests, GetClosestWaypointName_ExactMatch) {
    Navigator2D nav;
    nav.AddWaypoint("Origin", Point2D(0.0, 0.0));
    std::string closest = nav.GetClosestWaypointName(Point2D(0.0, 0.0));
    EXPECT_EQ(closest, "Origin");
}

TEST(Navigator2DTests, GetHeadingToWaypoint_InvalidWaypoint) {
    Navigator2D nav;
    Point2D pos(0.0, 0.0);
    EXPECT_THROW(nav.GetHeadingToWaypoint(pos, "Missing"), std::invalid_argument);
}

TEST(Navigator2DTests, GetDistanceToWaypoint_InvalidWaypoint) {
    Navigator2D nav;
    Point2D pos(0.0, 0.0);
    EXPECT_THROW(nav.GetDistanceToWaypoint(pos, "Missing"), std::invalid_argument);
}
