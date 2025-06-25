#ifndef DISTANCE_COST_ESTIMATOR_H
#define DISTANCE_COST_ESTIMATOR_H


#include "base_cost_estimator.h"



class DistanceCostEstimator : public BaseCostEstimator
{
public:

    // Constructor that initializes the base class with a Navigator2D instance
    DistanceCostEstimator(Navigator2D nav);

    // Calculates the cost as the Euclidean distance between two waypoints
    double CalculateCost(const std::string& from, const std::string& to) override;
    
};


#endif // DISTANCE_COST_ESTIMATOR_H