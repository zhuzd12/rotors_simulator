#ifndef OMPL_BASE_OBJECTIVES_MANHATTAN_OPTIMIZATION_OBJECTIVE_
#define OMPL_BASE_OBJECTIVES_MANHATTAN_OPTIMIZATION_OBJECTIVE_

#include "ompl/base/OptimizationObjective.h"

namespace ompl
{
    namespace base
    {
        class ManhattanOptimizationObjective : public OptimizationObjective
        {
        public:
            ManhattanOptimizationObjective(const SpaceInformationPtr &si);

            Cost stateCost(const State *s) const override;

            Cost motionCost(const State *s1, const State *s2) const override;

            Cost motionCostHeuristic(const State *s1, const State *s2) const override;

            InformedSamplerPtr allocInformedStateSampler(const ProblemDefinitionPtr &probDefn,
                                                        unsigned int maxNumberCalls) const override;
        };
    }
}
 
#endif