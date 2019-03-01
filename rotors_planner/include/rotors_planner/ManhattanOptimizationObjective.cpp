#include "ManhattanOptimizationObjective.h"
#include <memory>
#include "ompl/base/samplers/informed/PathLengthDirectInfSampler.h"

ompl::base::ManhattanOptimizationObjective::ManhattanOptimizationObjective(const SpaceInformationPtr &si)
: ompl::base::OptimizationObjective(si)
{
    description_ = "Path Length";

    // Setup a default cost-to-go heuristics:
    setCostToGoHeuristic(base::goalRegionCostToGo);
}

ompl::base::Cost ompl::base::ManhattanOptimizationObjective::stateCost(const State *s) const
{
    return identityCost();
}

ompl::base::Cost ompl::base::ManhattanOptimizationObjective::motionCost(const State *s1, const State *s2) const
{
    return Cost(si_->distance(s1, s2));
}

ompl::base::Cost ompl::base::ManhattanOptimizationObjective::motionCostHeuristic(const State *s1,
                                                                                const State *s2) const
{
    return motionCost(s1, s2);
}

ompl::base::InformedSamplerPtr ompl::base::ManhattanOptimizationObjective::allocInformedStateSampler(
    const ProblemDefinitionPtr &probDefn, unsigned int maxNumberCalls) const
{
// Make the direct path-length informed sampler and return. If OMPL was compiled with Eigen, a direct version is
// available, if not a rejection-based technique can be used
    return std::make_shared<PathLengthDirectInfSampler>(probDefn, maxNumberCalls);
}