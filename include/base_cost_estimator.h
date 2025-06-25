#ifndef BASE_COST_ESTIMATOR_H
#define BASE_COST_ESTIMATOR_H


#include "navigator_2d.h"


class BaseCostEstimator
{
public:

    BaseCostEstimator(Navigator2D nav) : navigator_(nav) { }

    virtual double CalculateCost(const std::string& from, const std::string& to) = 0;

    virtual ~BaseCostEstimator() = default;

protected:

    Navigator2D navigator_ ;

};

#endif // BASE_COST_ESTIMATOR_H