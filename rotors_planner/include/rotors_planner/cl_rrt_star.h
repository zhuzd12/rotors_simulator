#ifndef ROTORS_OMPL_GEOMETRIC_PLANNERS_RRT_CL_RRTstar_
#define ROTORS_OMPL_GEOMETRIC_PLANNERS_RRT_CL_RRTstar_

#include "ompl/geometric/planners/PlannerIncludes.h"
#include <ompl/control/planners/PlannerIncludes.h>

#include "ompl/base/OptimizationObjective.h"
#include "ompl/datastructures/NearestNeighbors.h"
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/base/MotionValidator.h>
#include <ompl/base/goals/GoalState.h>
#include <functional>
#include <rotors_planner/common.h>
#include <limits>
#include <vector>
#include <queue>
#include <deque>
#include <utility>
#include <list>
#include "modelmotionvalidator.h"

namespace rotors_planner_rrtstar
{
    namespace ob = ompl::base;
    namespace og = ompl::geometric;
    namespace oc = ompl::control;

    class Motion
    {
    public:
        Motion(const ob::SpaceInformationPtr &si) : state(si->allocState()), parent(nullptr), inGoal(false)
        {
        }

        ~Motion() = default;

        ob::State *state;
        
        // newly added
        // oc::Control *control{nullptr};
        unsigned int steps{0};
        bool commit_protect{false};
        double c_total{std::numeric_limits<double>::infinity()};
        double low_bound{std::numeric_limits<double>::infinity()};
        double up_bound{std::numeric_limits<double>::infinity()};

        Motion *parent;

        bool inGoal;

        ob::Cost cost;

        ob::Cost incCost;

        std::vector<Motion *> children;
    };

    typedef std::function<bool(const ob::SpaceInformationPtr &, ob::State *)> samplercleanerFn;

    class CL_RRTstar : public ob::Planner
    {
    public:
        CL_RRTstar(const ob::SpaceInformationPtr &si);

        ~CL_RRTstar() override;

        void getPlannerData(ob::PlannerData &data) const override;

        ob::PlannerStatus solve(const ob::PlannerTerminationCondition &ptc) override;

        void clear() override;

        void setup() override;

        void setSampleCleanerFn(samplercleanerFn &svc)
        {
            samplerCleanerFn_ = svc;
        }

        void setGoalBias(double goalBias)
        {
            goalBias_ = goalBias;
        }

        double getGoalBias() const
        {
            return goalBias_;
        }

        void setRange(double distance)
        {
            maxDistance_ = distance;
        }

        double getRange() const
        {
            return maxDistance_;
        }

        void setRewireFactor(double rewireFactor)
        {
            rewireFactor_ = rewireFactor;
            calculateRewiringLowerBounds();
        }

        double getRewireFactor() const
        {
            return rewireFactor_;
        }

        template <template <typename T> class NN>
        void setNearestNeighbors()
        {
            if (nn_ && nn_->size() != 0)
                OMPL_WARN("Calling setNearestNeighbors will clear all states.");
            clear();
            nn_ = std::make_shared<NN<Motion *>>();
            setup();
        }

        void setDelayCC(bool delayCC)
        {
            delayCC_ = delayCC;
        }

        bool getDelayCC() const
        {
            return delayCC_;
        }

        void setTreePruning(bool prune);

        bool getTreePruning() const
        {
            return useTreePruning_;
        }

        void setPruneThreshold(const double pp)
        {
            pruneThreshold_ = pp;
        }

        double getPruneThreshold() const
        {
            return pruneThreshold_;
        }

        void setPrunedMeasure(bool informedMeasure);

        bool getPrunedMeasure() const
        {
            return usePrunedMeasure_;
        }

        void setInformedSampling(bool informedSampling);

        bool getInformedSampling() const
        {
            return useInformedSampling_;
        }

        void setSampleRejection(bool reject);

        bool getSampleRejection() const
        {
            return useRejectionSampling_;
        }

        void setNewStateRejection(const bool reject)
        {
            useNewStateRejection_ = reject;
        }

        bool getNewStateRejection() const
        {
            return useNewStateRejection_;
        }

        void setAdmissibleCostToCome(const bool admissible)
        {
            useAdmissibleCostToCome_ = admissible;
        }

        bool getAdmissibleCostToCome() const
        {
            return useAdmissibleCostToCome_;
        }

        void setOrderedSampling(bool orderSamples);

        bool getOrderedSampling() const
        {
            return useOrderedSampling_;
        }

        void setBatchSize(unsigned int batchSize)
        {
            batchSize_ = batchSize;
        }

        unsigned int getBatchSize() const
        {
            return batchSize_;
        }

        void setFocusSearch(const bool focus)
        {
            setInformedSampling(focus);
            setTreePruning(focus);
            setPrunedMeasure(focus);
            setNewStateRejection(focus);
        }

        bool getFocusSearch() const
        {
            return getInformedSampling() && getPrunedMeasure() && getTreePruning() && getNewStateRejection();
        }

        void setKNearest(bool useKNearest)
        {
            useKNearest_ = useKNearest;
        }

        bool getKNearest() const
        {
            return useKNearest_;
        }

        void setNumSamplingAttempts(unsigned int numAttempts)
        {
            numSampleAttempts_ = numAttempts;
        }

        unsigned int getNumSamplingAttempts() const
        {
            return numSampleAttempts_;
        }

        unsigned int numIterations() const
        {
            return iterations_;
        }

        ob::Cost bestCost() const
        {
            return bestCost_;
        }

    protected:
        
        void allocSampler();

        bool sampleUniform(ob::State *statePtr);

        void freeMemory();

        // For sorting a list of costs and getting only their sorted indices
        struct CostIndexCompare
        {
            CostIndexCompare(const std::vector<ob::Cost> &costs, const ob::OptimizationObjective &opt)
            : costs_(costs), opt_(opt)
            {
            }
            bool operator()(unsigned i, unsigned j)
            {
                return opt_.isCostBetterThan(costs_[i], costs_[j]);
            }
            const std::vector<ob::Cost> &costs_;
            const ob::OptimizationObjective &opt_;
        };

        double distanceFunction(const Motion *a, const Motion *b) const
        {
            return si_->distance(a->state, b->state);
        }

        void getNeighbors(Motion *motion, std::vector<Motion *> &nbh) const;

        void removeFromParent(Motion *m);

        void updateChildCosts(Motion *m);

        int pruneTree(const ob::Cost &pruneTreeCost);

        ob::Cost solutionHeuristic(const Motion *motion) const;

        void addChildrenToList(std::queue<Motion *, std::deque<Motion *>> *motionList, Motion *motion);

        bool keepCondition(const Motion *motion, const ob::Cost &threshold) const;

        void calculateRewiringLowerBounds();

        // newly added
        // const oc::SpaceInformation *siC_;

        samplercleanerFn samplerCleanerFn_;

        ob::StateSamplerPtr sampler_;

        ob::InformedSamplerPtr infSampler_;

        std::shared_ptr<ompl::NearestNeighbors<Motion *>> nn_;

        double goalBias_{.05};

        double maxDistance_{1.};

        ompl::RNG rng_;

        bool useKNearest_{true};

        double rewireFactor_{1.1};

        double k_rrt_{0u};

        double r_rrt_{0.};

        bool delayCC_{true};

        ob::OptimizationObjectivePtr opt_;

        Motion *bestGoalMotion_{nullptr};

        std::vector<Motion *> goalMotions_;

        bool useTreePruning_{false};

        double pruneThreshold_{.05};

        bool usePrunedMeasure_{false};

        bool useInformedSampling_{false};

        bool useRejectionSampling_{false};

        bool useNewStateRejection_{false};

        bool useAdmissibleCostToCome_{true};

        unsigned int numSampleAttempts_{100u};

        bool useOrderedSampling_{false};

        unsigned int batchSize_{1u};

        std::vector<Motion *> startMotions_;

        ob::Cost bestCost_{std::numeric_limits<double>::quiet_NaN()};

        ob::Cost prunedCost_{std::numeric_limits<double>::quiet_NaN()};

        double prunedMeasure_{0.};

        unsigned int iterations_{0u};

        // Planner progress property functions
        std::string numIterationsProperty() const
        {
            return std::to_string(numIterations());
        }
        std::string bestCostProperty() const
        {
            return std::to_string(bestCost().value());
        }
    };
 }
 
 #endif