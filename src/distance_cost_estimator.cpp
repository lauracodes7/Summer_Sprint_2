#include "distance_cost_estimator.h"


DistanceCostEstimator::DistanceCostEstimator(Navigator2D nav) : BaseCostEstimator(nav) { }


double DistanceCostEstimator::CalculateCost(const std::string& from, const std::string& to)
{

    Point2D fromPosition = navigator_.GetWaypointPosition(from);
    Point2D toPosition = navigator_.GetWaypointPosition(to);

    double distance = sqrt(pow(toPosition.x - fromPosition.x, 2) + pow(toPosition.y - fromPosition.y, 2));

    return distance;
}
