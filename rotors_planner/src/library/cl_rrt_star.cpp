#include "rotors_planner/cl_rrt_star.h"
#include <algorithm>
#include <boost/math/constants/constants.hpp>
#include <limits>
#include <vector>
#include "ompl/base/Goal.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/base/goals/GoalState.h"
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/base/samplers/InformedStateSampler.h"
#include "ompl/base/samplers/informed/RejectionInfSampler.h"
#include "ompl/base/samplers/informed/OrderedInfSampler.h"
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/util/GeometricEquations.h"

namespace rotors_planner_rrtstar {

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace oc = ompl::control;

CL_RRTstar::CL_RRTstar(const ob::SpaceInformationPtr &si)
: ob::Planner(si, "CL_RRTstar")
{
    // siC_= si.get();

    specs_.approximateSolutions = true;
    specs_.optimizingPaths = true;
    specs_.canReportIntermediateSolutions = true;

    Planner::declareParam<double>("range", this, &CL_RRTstar::setRange, &CL_RRTstar::getRange, "0.:1.:10000.");
    Planner::declareParam<double>("goal_bias", this, &CL_RRTstar::setGoalBias, &CL_RRTstar::getGoalBias, "0.:.05:1.");
    Planner::declareParam<double>("rewire_factor", this, &CL_RRTstar::setRewireFactor, &CL_RRTstar::getRewireFactor,
                                "1.0:0.01:2.0");
    Planner::declareParam<bool>("use_k_nearest", this, &CL_RRTstar::setKNearest, &CL_RRTstar::getKNearest, "0,1");
    Planner::declareParam<bool>("delay_collision_checking", this, &CL_RRTstar::setDelayCC, &CL_RRTstar::getDelayCC, "0,1");
    Planner::declareParam<bool>("tree_pruning", this, &CL_RRTstar::setTreePruning, &CL_RRTstar::getTreePruning, "0,1");
    Planner::declareParam<double>("prune_threshold", this, &CL_RRTstar::setPruneThreshold, &CL_RRTstar::getPruneThreshold,
                                "0.:.01:1.");
    Planner::declareParam<bool>("pruned_measure", this, &CL_RRTstar::setPrunedMeasure, &CL_RRTstar::getPrunedMeasure, "0,1");
    Planner::declareParam<bool>("informed_sampling", this, &CL_RRTstar::setInformedSampling, &CL_RRTstar::getInformedSampling,
                                "0,1");
    Planner::declareParam<bool>("sample_rejection", this, &CL_RRTstar::setSampleRejection, &CL_RRTstar::getSampleRejection,
                                "0,1");
    Planner::declareParam<bool>("new_state_rejection", this, &CL_RRTstar::setNewStateRejection,
                                &CL_RRTstar::getNewStateRejection, "0,1");
    Planner::declareParam<bool>("use_admissible_heuristic", this, &CL_RRTstar::setAdmissibleCostToCome,
                                &CL_RRTstar::getAdmissibleCostToCome, "0,1");
    Planner::declareParam<bool>("ordered_sampling", this, &CL_RRTstar::setOrderedSampling, &CL_RRTstar::getOrderedSampling,
                                "0,1");
    Planner::declareParam<unsigned int>("ordering_batch_size", this, &CL_RRTstar::setBatchSize, &CL_RRTstar::getBatchSize,
                                        "1:100:1000000");
    Planner::declareParam<bool>("focus_search", this, &CL_RRTstar::setFocusSearch, &CL_RRTstar::getFocusSearch, "0,1");
    Planner::declareParam<unsigned int>("number_sampling_attempts", this, &CL_RRTstar::setNumSamplingAttempts,
                                        &CL_RRTstar::getNumSamplingAttempts, "10:10:100000");

    addPlannerProgressProperty("iterations INTEGER", [this] { return numIterationsProperty(); });
    addPlannerProgressProperty("best cost REAL", [this] { return bestCostProperty(); });
}

CL_RRTstar::~CL_RRTstar()
{
    freeMemory();
}

void CL_RRTstar::setup()
{
    Planner::setup();
    ompl::tools::SelfConfig sc(si_, getName());
    sc.configurePlannerRange(maxDistance_);
    if (!si_->getStateSpace()->hasSymmetricDistance() || !si_->getStateSpace()->hasSymmetricInterpolate())
    {
        OMPL_WARN("%s requires a state space with symmetric distance and symmetric interpolation.", getName().c_str());
    }

    if (!nn_)
        nn_.reset(new ompl::NearestNeighborsLinear<Motion *>());
        // nn_.reset(ompl::tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    
    nn_->setDistanceFunction([this](const Motion *a, const Motion *b)
      {
         if(a->commit_protect || b->commit_protect)
           return std::numeric_limits<double>::infinity();
         else
           return distanceFunction(a, b);
      });

    // nn_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });

    // Setup optimization objective
    //
    // If no optimization objective was specified, then default to
    // optimizing path length as computed by the distance() function
    // in the state space.
    if (pdef_)
    {
        if (pdef_->hasOptimizationObjective())
            opt_ = pdef_->getOptimizationObjective();
        else
        {
            OMPL_INFORM("%s: No optimization objective specified. Defaulting to optimizing path length for the allowed "
                        "planning time.",
                        getName().c_str());
            opt_ = std::make_shared<ob::PathLengthOptimizationObjective>(si_);

            // Store the new objective in the problem def'n
            pdef_->setOptimizationObjective(opt_);
        }

        // Set the bestCost_ and prunedCost_ as infinite
        bestCost_ = opt_->infiniteCost();
        prunedCost_ = opt_->infiniteCost();
    }
    else
    {
        OMPL_INFORM("%s: problem definition is not set, deferring setup completion...", getName().c_str());
        setup_ = false;
    }

    // Get the measure of the entire space:
    prunedMeasure_ = si_->getSpaceMeasure();
    // std::cout<<"##########"<<prunedMeasure_<<std::endl;

    // Calculate some constants:
    calculateRewiringLowerBounds();
    std::cout<<"##########"<<k_rrt_<<std::endl;
    std::cout<<"##########"<<r_rrt_<<std::endl;
}

void CL_RRTstar::clear()
{
    setup_ = false;
    Planner::clear();
    sampler_.reset();
    infSampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();

    bestGoalMotion_ = nullptr;
    goalMotions_.clear();
    startMotions_.clear();

    iterations_ = 0;
    bestCost_ = ob::Cost(std::numeric_limits<double>::quiet_NaN());
    prunedCost_ = ob::Cost(std::numeric_limits<double>::quiet_NaN());
    prunedMeasure_ = 0.0;
}

int CL_RRTstar::onlinePruneTree(const ob::State * current_state)
{
    if(plan_loop_ == 0 or !rePropagation_flag_)
    {
        auto *motion = new Motion(si_);
        si_->copyState(motion->state, current_state);
        if(nn_)
            nn_->clear();
        // newly added
        motion->c_total = 0.0;
        motion->parent = nullptr;
        ob::Goal *goal = pdef_->getGoal().get();
        goal->isSatisfied(motion->state, &motion->low_bound);
        motion->cost = opt_->identityCost();
        motion->incCost = opt_->identityCost();
        motion->time = 0;
        motion->intime = 0.0;
        nn_->add(motion);
        startMotions_.clear();
        startMotions_.push_back(motion);
        bestGoalMotion_ = nullptr;
        bestCost_ = opt_->infiniteCost();
        prunedCost_ = opt_->infiniteCost();
        approxGoalMotion_ = nullptr;
        approxDist_= std::numeric_limits<double>::infinity();
        goalMotions_.clear();
        return 0;
    }

    // find the new root node which is nearest to current_state
    auto *cmotion = new Motion(si_);
    si_->copyState(cmotion->state, current_state);
    Motion *nmotion = nn_->nearest(cmotion);
    OMPL_INFORM("find nearest node and distance: %lf", si_->distance(current_state, nmotion->state));

    // find index of new root node in current best solution
    Motion *oldSolution = nullptr;
    if (bestGoalMotion_)
    {
        oldSolution = bestGoalMotion_;
    }
    else if (approxGoalMotion_)
    {
        oldSolution = approxGoalMotion_;
    }
    Motion *iterMotion = oldSolution;
    Motion *new_root_motion = nullptr;
    bool find_root = true;
    double min_dis = std::numeric_limits<double>::infinity();
    int solution_nodes_num = 0;
    int new_root_index = 0;
    std::vector<Motion *> solution_motions;
    while (iterMotion != nullptr)
    {
        if(si_->distance(iterMotion->state, current_state) < min_dis)
        {
            min_dis = si_->distance(iterMotion->state, current_state);
            new_root_index = solution_nodes_num;
            new_root_motion = iterMotion;
        }
        // if(iterMotion == nmotion)
        // {
        //     find_root = true;
        //     min_dis = 0.0;
        //     break;
        // }
        solution_motions.push_back(iterMotion);
        solution_nodes_num++;      
        iterMotion = iterMotion->parent;
    }
    assert(solution_motions[new_root_index] == new_root_motion);
    OMPL_INFORM("min distance between current state and old solution path: %lf, index: %d", min_dis, new_root_index);

    // clear pdef_
    OMPL_INFORM("old solution nodes num: %d", int(solution_motions.size()));
    // std::vector< ob::PlannerSolution > solutions = pdef_->getSolutions();
    pdef_->clearSolutionPaths();

    if((!find_root && min_dis > path_replan_deviation_) || new_root_index==0)
    {
        OMPL_INFORM("previous info can be discarded totally");
        freeMemory();
        int old_nn_num = nn_->size();
        if(nn_)
            nn_->clear();
        auto *total_new_motion = new Motion(si_);
        si_->copyState(total_new_motion->state, current_state);
        total_new_motion->parent = nullptr;
        total_new_motion->c_total = 0.0;
        ob::Goal *goal = pdef_->getGoal().get();
        goal->isSatisfied(total_new_motion->state, &total_new_motion->low_bound);
        total_new_motion->cost = opt_->identityCost();
        total_new_motion->incCost = opt_->identityCost();
        total_new_motion->time = 0;
        total_new_motion->intime = 0.0;
        total_new_motion->parent = nullptr;
        nn_->add(total_new_motion);
        bestGoalMotion_= nullptr;
        bestCost_ = opt_->infiniteCost();
        prunedCost_ = opt_->infiniteCost();
        approxGoalMotion_ = nullptr;
        approxDist_= std::numeric_limits<double>::infinity();
        goalMotions_.clear();
        startMotions_.clear();
        startMotions_.push_back(total_new_motion);
        return old_nn_num;
    }
    else
    {
        /* prune tree from nmotion */
        // do not change bestGoalMotion_ and approxGoalMotion_ for we assure that the agent is 
        // following the right solution after calling repropagation function
        OMPL_INFORM("previous info can be utilized partially");
        /* mark the commited portion */
        OMPL_DEBUG("time estimatation for protect region: %lf", new_root_motion->time);
        // reset new root motion info and update the whole tree
        new_root_motion->time = 0.0;
        new_root_motion->intime = 0.0;
        new_root_motion->cost = opt_->identityCost();
        new_root_motion->incCost = opt_->identityCost();
        new_root_motion->parent = nullptr;
        updateChildCosts(new_root_motion);

        // int predict_root_index = new_root_index - std::ceil(loop_plan_time_/0.1);

        // if(predict_root_index <= 0)
        // {
        //     OMPL_DEBUG("not enough for prediction, predicted root index: %d", predict_root_index);
        //     predict_root_index = 0;
        // }
        double propagation_time = loop_plan_time_;
        
        int predict_root_index = new_root_index-1;
        while(propagation_time>0 && predict_root_index>0)
        {
            propagation_time -= solution_motions[predict_root_index]->intime;
            predict_root_index--;
        }

        if(predict_root_index <= 0)
        {
            OMPL_DEBUG("not enough for prediction, predicted root index: %d", predict_root_index);
            predict_root_index = 0;
        }

        Motion *last_pro_motion = nullptr;
        assert(solution_motions[new_root_index]->parent == nullptr);
        for(int pro_iter=new_root_index; pro_iter>predict_root_index; --pro_iter)
        {
            auto *pro_motion = new Motion(si_);
            pro_motion->state = si_->allocState();
            si_->copyState(pro_motion->state, solution_motions[pro_iter]->state);
            pro_motion->time = solution_motions[pro_iter]->time;
            pro_motion->intime = solution_motions[pro_iter]->intime;
            pro_motion->cost = solution_motions[pro_iter]->cost;
            pro_motion->incCost = solution_motions[pro_iter]->incCost;
            pro_motion->parent = last_pro_motion;
            if(last_pro_motion)
                pro_motion->parent->children.push_back(pro_motion);
            protectRegionMotions_.push_back(pro_motion);
            last_pro_motion = pro_motion;
        }
        // update startmotions and real root motion
        Motion* old_start_motion = startMotions_.front();
        assert(old_start_motion->parent == nullptr);
        startMotions_.clear();
         Motion *real_root_motion;
        if(protectRegionMotions_.empty())
        {
            OMPL_DEBUG("nothing to protect!");
            startMotions_.push_back(new_root_motion);
            real_root_motion = new_root_motion;
            real_root_motion->parent = nullptr;
        }
        else
        {
            OMPL_DEBUG("temporary storage for protect region with %u node", protectRegionMotions_.size());
            startMotions_.push_back(protectRegionMotions_.front());
            real_root_motion = solution_motions[predict_root_index];
            real_root_motion->parent = last_pro_motion;
        }        

        // The queue of Motions to process:
        unsigned int numPruned = 0;
        std::queue<Motion *, std::deque<Motion *>> motionQueue;
        std::queue<Motion *, std::deque<Motion *>> leavesToPrune;
        std::queue<Motion *, std::deque<Motion *>> saveToPrune;
        OMPL_DEBUG("old nn size: %d", int(nn_->size()));
        // Clear the NN structure:
        nn_->clear();
        motionQueue.push(old_start_motion);
        // for (auto &startMotion : startMotions_)
        // {
        //     // Add to the NN
        //     // nn_->add(startMotion);
            
        //     motionQueue.push(startMotion);
        //     // Add their children to the queue:
        //     // addChildrenToList(&motionQueue, startMotion);
            
        // }
        while (motionQueue.empty() == false)
        {
            if(motionQueue.front() != real_root_motion)
            {
                // Add it's children to the queue
                addChildrenToList(&motionQueue, motionQueue.front());
                leavesToPrune.push(motionQueue.front());
            }
            // Pop the iterator, std::list::erase returns the next iterator
            motionQueue.pop();
        }
        OMPL_DEBUG("%d nodes to prune", int(leavesToPrune.size()));
        while (leavesToPrune.empty() == false)
        {
            // Erase the actual motion
            // First free the state
            si_->freeState(leavesToPrune.front()->state);
            // then delete the pointer
            delete leavesToPrune.front();
            // And finally remove it from the list, erase returns the next iterator
            leavesToPrune.pop();
            // Update our counter
            ++numPruned;
        }

        saveToPrune.push(real_root_motion);
        ob::Goal *goal = pdef_->getGoal().get();
        goalMotions_.clear();
        while (saveToPrune.empty() == false)
        {
            nn_->add(saveToPrune.front());
            double distanceFromGoal;
            if (goal->isSatisfied(saveToPrune.front()->state, &distanceFromGoal))
            {
                saveToPrune.front()->inGoal = true;
                goalMotions_.push_back(saveToPrune.front());
            }
            if(saveToPrune.front())
            addChildrenToList(&saveToPrune, saveToPrune.front());
            saveToPrune.pop();
        }
        OMPL_DEBUG("%d nodes saved", int(nn_->size()));

        //re add odsolution to pdef_
        std::vector<Motion *> mpath;
        Motion *iterMotion = oldSolution;
        while (iterMotion != nullptr)
        {
            mpath.push_back(iterMotion);
            iterMotion = iterMotion->parent;
        }
        assert(mpath.size() == (new_root_index+1));
        // set the solution path
        auto path(std::make_shared<ompl::geometric::PathGeometric>(si_));
        for (int i = mpath.size() - 1; i >= 0; --i)
            path->append(mpath[i]->state);
        // Add the solution path.
        ob::PlannerSolution psol(path);
        psol.setPlannerName(getName());
        // If we don't have a goal motion, the solution is approximate
        if (!bestGoalMotion_)
        {
            // psol.setApproximate(approxDist_);
            // std::cout<<"psol not best"<<std::endl;
        }
        // std::cout<<"appro: "<<approxDist_<<std::endl;
        // Does the solution satisfy the optimization objective?
        psol.setOptimized(opt_, oldSolution->cost, opt_->isSatisfied(bestCost_));
        pdef_->addSolutionPath(psol);
        return numPruned;

    }

}

ompl::base::PlannerStatus CL_RRTstar::solve(const ob::PlannerTerminationCondition &ptc)
{
    checkValidity();
    ob::Goal *goal = pdef_->getGoal().get();
    ob::State *goal_state = pdef_->getGoal().get()->as<ob::GoalState>()->getState();
    auto *goal_s = dynamic_cast<ob::GoalSampleableRegion *>(goal);

    bool symCost = opt_->isSymmetric();

    // every planning loop time, we get new startMotions_ from updated pdf_
    // and we start prune nn_ to get a new tree rooted from startmotion
    OMPL_INFORM("*********************************************************************************");
    const ob::State *st = pdef_->getStartState(0);
    // std::stringstream ss;
    si_->printState(st);
    int pre_plan_prune_nodes = onlinePruneTree(st);
    plan_loop_++;
    OMPL_INFORM("planning loop: %d, %d nodes pruned before planning.", plan_loop_, pre_plan_prune_nodes);

    if (nn_->size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return ob::PlannerStatus::INVALID_START;
    }

    // Allocate a sampler if necessary
    if (!sampler_ && !infSampler_)
    {
        allocSampler();
    }

    OMPL_INFORM("%s: Started planning with %u states. Seeking a solution better than %.5f.", getName().c_str(), nn_->size(), opt_->getCostThreshold().value());

    if ((useTreePruning_ || useRejectionSampling_ || useInformedSampling_ || useNewStateRejection_) &&
        !si_->getStateSpace()->isMetricSpace())
        OMPL_WARN("%s: The state space (%s) is not metric and as a result the optimization objective may not satisfy "
                "the triangle inequality. "
                "You may need to disable pruning or rejection.",
                getName().c_str(), si_->getStateSpace()->getName().c_str());

    const ob::ReportIntermediateSolutionFn intermediateSolutionCallback = pdef_->getIntermediateSolutionCallback();

    // todo: online
    // Motion *approxGoalMotion_ = nullptr;
    // double approxDist_ = std::numeric_limits<double>::infinity();

    auto *rmotion = new Motion(si_);
    ob::State *rstate = rmotion->state;
    ob::State *xstate = si_->allocState();
    // oc::Control *rctrl = rmotion->control;

    std::vector<Motion *> nbh;

    std::vector<ob::Cost> costs;
    std::vector<ob::Cost> incCosts;
    std::vector<std::size_t> sortedCostIndices;

    std::vector<int> valid;
    unsigned int rewireTest = 0;
    unsigned int statesGenerated = 0;

    if (bestGoalMotion_)
        OMPL_INFORM("%s: Starting planning with existing solution of cost %.5f", getName().c_str(),
                    bestCost_);

    if (useKNearest_)
        OMPL_INFORM("%s: Initial k-nearest value of %u", getName().c_str(),
                    (unsigned int)std::ceil(k_rrt_ * log((double)(nn_->size() + 1u))));
    else
        OMPL_INFORM(
            "%s: Initial rewiring radius of %.2f", getName().c_str(),
            std::min(maxDistance_, r_rrt_ * std::pow(log((double)(nn_->size() + 1u)) / ((double)(nn_->size() + 1u)),
                                                    1 / (double)(sample_dimension_))));

    // our functor for sorting nearest neighbors
    CostIndexCompare compareFn(costs, *opt_);

    bool solution_updated = false;
    std::pair<ob::State *, double> sample_lastValid;
    sample_lastValid.first = si_->allocState();
    while (ptc == false)
    {
        iterations_++;

        // sample random state (with goal biasing)
        // Goal samples are only sampled until maxSampleCount() goals are in the tree, to prohibit duplicate goal
        // states.
        if (goal_s && goalMotions_.size() < goal_s->maxSampleCount() && rng_.uniform01() < goalBias_ &&
            goal_s->canSample())
            goal_s->sampleGoal(rstate);
        else
        {
            // Attempt to generate a sample, if we fail (e.g., too many rejection attempts), skip the remainder of this
            // loop and return to try again
            if (!sampleUniform(rstate))
                continue;
        }
        // samplerCleanerFn_(si_, rstate);

        // find closest state in the tree
        Motion *nmotion = nn_->nearest(rmotion);

        if (intermediateSolutionCallback && si_->equalStates(nmotion->state, rstate))
            continue;

        ob::State *dstate = rstate;

        // find state to add to the tree
        double d = si_->distance(nmotion->state, rstate);
        if (d > maxDistance_)
        {
            si_->getStateSpace()->interpolate(nmotion->state, rstate, maxDistance_ / d, xstate);
            dstate = xstate;
        }
        samplerCleanerFn_(si_, dstate);

        // Check if the motion between the nearest state and the state to add is valid
        bool sample_propagation = model_mv_->checkMotion(nmotion->state, dstate, sample_lastValid);
        if (sample_propagation) // || sample_lastValid.second > 0)
        {
            // create a motion
            auto *motion = new Motion(si_);
            si_->copyState(motion->state, dstate);
            if(!sample_propagation)
            {
                OMPL_DEBUG("save last valid state as new sample motion time: %lf", sample_lastValid.second);
                // si_->printState(sample_lastValid.first);
                samplerCleanerFn_(si_, sample_lastValid.first);
                si_->copyState(motion->state, sample_lastValid.first);
            }

            motion->parent = nmotion;
            motion->incCost = opt_->motionCost(nmotion->state, motion->state);
            motion->cost = opt_->combineCosts(nmotion->cost, motion->incCost);
            motion->intime = sample_lastValid.second;
            motion->time = sample_lastValid.second + nmotion->time;

            // Find nearby neighbors of the new motion
            getNeighbors(motion, nbh);

            rewireTest += nbh.size();
            ++statesGenerated;

            // cache for distance computations
            //
            // Our cost caches only increase in size, so they're only
            // resized if they can't fit the current neighborhood
            if (costs.size() < nbh.size())
            {
                costs.resize(nbh.size());
                incCosts.resize(nbh.size());
                sortedCostIndices.resize(nbh.size());
            }

            // cache for motion validity (only useful in a symmetric space)
            //
            // Our validity caches only increase in size, so they're
            // only resized if they can't fit the current neighborhood
            if (valid.size() < nbh.size())
                valid.resize(nbh.size());
            std::fill(valid.begin(), valid.begin() + nbh.size(), 0);

            // Finding the nearest neighbor to connect to
            // By default, neighborhood states are sorted by cost, and collision checking
            // is performed in increasing order of cost
            int index_s = -1;
            if (delayCC_)
            {
                // calculate all costs and distances
                for (std::size_t i = 0; i < nbh.size(); ++i)
                {
                    incCosts[i] = opt_->motionCost(nbh[i]->state, motion->state);
                    costs[i] = opt_->combineCosts(nbh[i]->cost, incCosts[i]);
                }

                // sort the nodes
                //
                // we're using index-value pairs so that we can get at
                // original, unsorted indices
                for (std::size_t i = 0; i < nbh.size(); ++i)
                    sortedCostIndices[i] = i;
                std::sort(sortedCostIndices.begin(), sortedCostIndices.begin() + nbh.size(), compareFn);

                // collision check until a valid motion is found
                //
                // ASYMMETRIC CASE: it's possible that none of these
                // neighbors are valid. This is fine, because motion
                // already has a connection to the tree through
                // nmotion (with populated cost fields!).
                for (std::vector<std::size_t>::const_iterator i = sortedCostIndices.begin();
                    i != sortedCostIndices.begin() + nbh.size(); ++i)
                {
                    std::pair<ob::State *, double> lastValid_temp;
                    lastValid_temp.second = 0.0;
                    if (nbh[*i] == nmotion ||
                        ((!useKNearest_ || si_->distance(nbh[*i]->state, motion->state) < maxDistance_) &&
                        model_mv_->checkMotion(nbh[*i]->state, motion->state, lastValid_temp)))
                    {
                        // OMPL_DEBUG("debug: update connect node with nbh[%u]", *i);
                        // si_->printState(nbh[*i]->state);
                        // si_->printState(motion->state);
                        index_s = *i;
                        motion->incCost = incCosts[*i];
                        motion->cost = costs[*i];
                        motion->parent = nbh[*i];
                        motion->intime = lastValid_temp.second;
                        motion->time = lastValid_temp.second+nbh[*i]->time;
                        valid[*i] = 1;
                        break;
                    }
                    else
                        valid[*i] = -1;
                }

            }
            else  // if not delayCC
            {
                motion->incCost = opt_->motionCost(nmotion->state, motion->state);
                motion->cost = opt_->combineCosts(nmotion->cost, motion->incCost);
                // find which one we connect the new state to
                for (std::size_t i = 0; i < nbh.size(); ++i)
                {
                    if (nbh[i] != nmotion)
                    {
                        incCosts[i] = opt_->motionCost(nbh[i]->state, motion->state);
                        costs[i] = opt_->combineCosts(nbh[i]->cost, incCosts[i]);
                        if (opt_->isCostBetterThan(costs[i], motion->cost))
                        {
                            std::pair<ob::State *, double> lastValid_temp;
                            lastValid_temp.second = 0.0;
                            if ((!useKNearest_ || si_->distance(nbh[i]->state, motion->state) < maxDistance_) &&
                                model_mv_->checkMotion(nbh[i]->state, motion->state, lastValid_temp))
                            {
                                // OMPL_DEBUG("debug: update connect node");
                                // si_->printState(nbh[i]->state);
                                // si_->printState(motion->state);
                                motion->incCost = incCosts[i];
                                motion->cost = costs[i];
                                motion->parent = nbh[i];
                                motion->intime = lastValid_temp.second;
                                motion->time = lastValid_temp.second+nbh[i]->time;
                                valid[i] = 1;
                            }
                            else
                                valid[i] = -1;
                        }
                    }
                    else
                    {
                        incCosts[i] = motion->incCost;
                        costs[i] = motion->cost;
                        valid[i] = 1;
                    }
                }
            }
            
            Motion *connected_motion = motion->parent;
            assert(index_s != -1);
            assert(connected_motion == nbh[index_s]);
            // OMPL_DEBUG("nbh size: %u", nbh.size());
            if (!useNewStateRejection_ || (useNewStateRejection_ && opt_->isCostBetterThan(solutionHeuristic(motion), bestCost_)))
            {
                // as we have found motion->parent namely nbh[i] as the connected node
                // we use checkMotion function to add all intermediate nodes to the nn_ and update motion->parent
                if(useTrejectoryExpansion_)
                {
                    tempTrajectoryMotions_.clear();
                    std::vector<ob::State *> trajectory_states;
                    std::vector<double> time_stamps;
                    bool recheck = model_mv_->checkMotion(connected_motion->state, motion->state, trajectory_states, time_stamps);
                    if(!recheck)
                    {
                        si_->freeState(motion->state);
                        delete motion;
                        OMPL_WARN("trajectory propagation failed");
                        continue;
                    }
                    if(trajectory_states.empty())
                    {
                        nn_->add(motion);
                        motion->parent->children.push_back(motion);
                    }
                    else
                    {
                        Motion* last_motion = connected_motion;
                        for (std::size_t i = 0; i < trajectory_states.size(); ++i)
                        {
                            Motion *inter_motion = new Motion(si_);
                            si_->copyState(inter_motion->state, trajectory_states[i]);
                            inter_motion->incCost = opt_->motionCost(last_motion->state, inter_motion->state);
                            inter_motion->cost = opt_->combineCosts(last_motion->cost, inter_motion->incCost);
                            inter_motion->intime = time_stamps[i];
                            inter_motion->parent = last_motion;
                            inter_motion->parent->children.push_back(inter_motion);
                            tempTrajectoryMotions_.push_back(inter_motion);
                            nn_->add(inter_motion);
                            last_motion = inter_motion;                     
                        }
                        // do not add motion to tree as the last node
                        si_->freeState(motion->state);
                        // warn
                        delete motion;
                        motion = last_motion;
                    }               
                }
                else
                {
                    nn_->add(motion);
                    motion->parent->children.push_back(motion);
                }
            }
            else
            {
                // add motion to the tree
                si_->freeState(motion->state);
                delete motion;
                continue;
            }

            bool checkForSolution = false;
            std::pair<ob::State *, double> lastValid_rewire;
            // lastValid_rewire.first = si_->allocState();
            for (std::size_t i = 0; i < nbh.size(); ++i)
            {   
                if (nbh[i] != connected_motion)
                {
                    ob::Cost nbhIncCost;
                    if (symCost)
                        nbhIncCost = incCosts[i];
                    else
                        nbhIncCost = opt_->motionCost(motion->state, nbh[i]->state);
                    ob::Cost nbhNewCost = opt_->combineCosts(motion->cost, nbhIncCost);
                    if (opt_->isCostBetterThan(nbhNewCost, nbh[i]->cost))
                    {
                        bool motionValid;
                        if (valid[i] == 0)
                        {
                            motionValid =
                                (!useKNearest_ || si_->distance(nbh[i]->state, motion->state) < maxDistance_) &&
                                model_mv_->checkMotion(motion->state, nbh[i]->state, lastValid_rewire);
                        }
                        else
                        {
                            motionValid = (valid[i] == 1);
                        }

                        if (motionValid)
                        {
                            // Remove this node from its parent list
                            removeFromParent(nbh[i]);

                            // Add this node to the new parent
                            // todo: add all intermediate trajectory nodes form motion->state to nbh[i]->state to nn_
                            // and update nbh[i]->parent = last node of intermediate trajectory
                            if(useTrejectoryExpansion_)
                            {
                                // OMPL_INFORM("debug4 !");
                                Motion *connected_motion = motion->parent;
                                std::vector<ob::State *> trajectory_states;
                                std::vector<double> time_stamps;
                                // ob::MotionValidatorPtr mv = si_->getMotionValidator();
                                // const std::shared_ptr<ob::ModelMotionValidator> model_mv = std::dynamic_pointer_cast<ob::ModelMotionValidator>(mv);
                                // bool fake_check = model_mv_->checkMotion(motion->state, nbh[i]->state);
                                // bool fake_check_2 = model_mv_->checkMotion(motion->state, nbh[i]->state, lastValid_rewire);
                                bool recheck = model_mv_->checkMotion(motion->state, nbh[i]->state, trajectory_states, time_stamps);
                                if(!recheck)
                                {
                                    // si_->freeStates(trajectory_states);
                                    // OMPL_DEBUG("reverse propagation failed");
                                    continue;
                                }
                                if(trajectory_states.empty())
                                {
                                    nbh[i]->parent = motion;
                                    nbh[i]->incCost = nbhIncCost;
                                    nbh[i]->cost = nbhNewCost;
                                    nbh[i]->parent->children.push_back(nbh[i]);
                                    nbh[i]->intime = lastValid_rewire.second;
                                    nbh[i]->time = lastValid_rewire.second + motion->time;
                                }
                                else
                                {
                                    std::vector<Motion *> intermotions;
                                    Motion* last_motion = motion;
                                    for (std::size_t i = 0; i < trajectory_states.size(); ++i)
                                    {
                                        Motion *inter_motion = new Motion(si_);
                                        si_->copyState(inter_motion->state, trajectory_states[i]);
                                        inter_motion->incCost = opt_->motionCost(last_motion->state, inter_motion->state);
                                        inter_motion->cost = opt_->combineCosts(last_motion->cost, inter_motion->incCost);
                                        inter_motion->intime = time_stamps[i];
                                        inter_motion->time = inter_motion->intime + last_motion->time;
                                        inter_motion->parent = last_motion;
                                        inter_motion->parent->children.push_back(inter_motion);
                                        intermotions.push_back(inter_motion);
                                        nn_->add(inter_motion);
                                        last_motion = inter_motion;                     
                                    }
                                    nbh[i]->parent = last_motion;
                                    nbh[i]->incCost = opt_->identityCost();
                                    nbh[i]->cost = last_motion->cost;
                                    nbh[i]->intime = 0.0;
                                    nbh[i]->time = last_motion->time;
                                    nbh[i]->parent->children.push_back(nbh[i]);
                                }               
                            }
                            else{
                                nbh[i]->parent = motion;
                                nbh[i]->incCost = nbhIncCost;
                                nbh[i]->cost = nbhNewCost;
                                nbh[i]->intime = lastValid_rewire.second;
                                nbh[i]->time = motion->time + nbh[i]->intime;
                                nbh[i]->parent->children.push_back(nbh[i]);
                            }

                            // Update the costs of the node's children
                            updateChildCosts(nbh[i]);

                            checkForSolution = true;
                        }
                    }
                }
            }

            // Add the new motion to the goalMotion_ list, if it satisfies the goal
            double distanceFromGoal;
            if (goal->isSatisfied(motion->state, &distanceFromGoal))
            {
                motion->inGoal = true;
                goalMotions_.push_back(motion);
                checkForSolution = true;
            }

            // Checking for solution or iterative improvement
            if (checkForSolution)
            {
                bool updatedSolution = false;
                if (!bestGoalMotion_ && !goalMotions_.empty())
                {
                    // We have found our first solution, store it as the best. We only add one
                    // vertex at a time, so there can only be one goal vertex at this moment.
                    bestGoalMotion_ = goalMotions_.front();
                    bestCost_ = bestGoalMotion_->cost;
                    updatedSolution = true;
                    solution_updated = true;

                    OMPL_INFORM("%s: Found an initial solution with a cost of %.2f in %u iterations (%u "
                                "vertices in the graph)",
                                getName().c_str(), bestCost_, iterations_, nn_->size());
                }
                else
                {
                    // We already have a solution, iterate through the list of goal vertices
                    // and see if there's any improvement.
                    for (auto &goalMotion : goalMotions_)
                    {
                        // Is this goal motion better than the (current) best?
                        if (opt_->isCostBetterThan(goalMotion->cost, bestCost_))
                        {
                            bestGoalMotion_ = goalMotion;
                            bestCost_ = bestGoalMotion_->cost;
                            updatedSolution = true;
                            solution_updated = true;

                            // Check if it satisfies the optimization objective, if it does, break the for loop
                            if (opt_->isSatisfied(bestCost_))
                            {
                                break;
                            }
                        }
                    }
                }

                if (updatedSolution)
                {
                    if (useTreePruning_)
                    {
                        pruneTree(bestCost_);
                    }

                    if (intermediateSolutionCallback)
                    {
                        std::vector<const ob::State *> spath;
                        Motion *intermediate_solution =
                            bestGoalMotion_->parent;  // Do not include goal state to simplify code.

                        // Push back until we find the start, but not the start itself
                        while (intermediate_solution->parent != nullptr)
                        {
                            spath.push_back(intermediate_solution->state);
                            intermediate_solution = intermediate_solution->parent;
                        }

                        intermediateSolutionCallback(this, spath, bestCost_);
                    }
                }
            }

            // Checking for approximate solution (closest state found to the goal)
            if (goalMotions_.size() == 0 && distanceFromGoal < approxDist_)
            {
                approxGoalMotion_ = motion;
                approxDist_ = distanceFromGoal;
                solution_updated = true;
            }
        }

        // terminate if a sufficient solution is found
        if (bestGoalMotion_ && opt_->isSatisfied(bestCost_))
            break;
    }

    // add protect regions to nn_
    for(int g=0; g<protectRegionMotions_.size(); g++)
    {
        nn_->add(protectRegionMotions_[g]);
    }
    if(!protectRegionMotions_.empty())
    {
        assert(protectRegionMotions_.front()->parent == nullptr);
    }
    protectRegionMotions_.clear();

    // Add our solution (if it exists)
    Motion *newSolution = nullptr;
    if(solution_updated)
    {
        if (bestGoalMotion_)
        {
            // We have an exact solution
            newSolution = bestGoalMotion_;
        }
        else if (approxGoalMotion_)
        {
            // We don't have a solution, but we do have an approximate solution
            newSolution = approxGoalMotion_;
        }   
    }
    // No else, we have nothing

    // Add what we found
    if (newSolution)
    {
        ptc.terminate();
        // construct the solution path
        std::vector<Motion *> mpath;
        Motion *iterMotion = newSolution;
        while (iterMotion != nullptr)
        {
            mpath.push_back(iterMotion);
            iterMotion = iterMotion->parent;
        }

        // set the solution path
        auto path(std::make_shared<ompl::geometric::PathGeometric>(si_));
        for (int i = mpath.size() - 1; i >= 0; --i)
            path->append(mpath[i]->state);

        // Add the solution path.
        ob::PlannerSolution psol(path);
        psol.setPlannerName(getName());

        // If we don't have a goal motion, the solution is approximate
        if (!bestGoalMotion_)
            psol.setApproximate(approxDist_);

        // Does the solution satisfy the optimization objective?
        psol.setOptimized(opt_, newSolution->cost, opt_->isSatisfied(bestCost_));
        pdef_->addSolutionPath(psol);
    }
    // No else, we have nothing

    si_->freeState(xstate);
    si_->freeState(sample_lastValid.first);
    if (rmotion->state)
        si_->freeState(rmotion->state);
    delete rmotion;

    OMPL_INFORM("%s: Created %u new states. Checked %u rewire options. %u goal states in tree. Final solution cost "
                "%.3f",
                getName().c_str(), statesGenerated, rewireTest, goalMotions_.size(), bestCost_.value());

    // We've added a solution if newSolution == true, and it is an approximate solution if bestGoalMotion_ == false
    return ob::PlannerStatus(approxGoalMotion_ != nullptr, bestGoalMotion_ == nullptr);
}

bool CL_RRTstar::rePropagation(ob::State * current_state, std::shared_ptr<ompl::geometric::PathGeometric> &best_path, std::vector<double> &time_stamps)
{
    bool result = false;
    time_stamps.clear();
    if(!pdef_->hasSolution() || (!approxGoalMotion_ && !bestGoalMotion_))
    {
        OMPL_INFORM("no solution to repropagate");
        return result;
    }

    // GoalMotionCompare goalmotionFn(*opt_);
    std::priority_queue<Motion*,std::vector<Motion*>, GoalMotionCompare> goal_motion_que;
    if(!bestGoalMotion_)
    {
        goal_motion_que.push(approxGoalMotion_);
    }
    else
    {
        for (auto &goalMotion : goalMotions_)
        {
            goal_motion_que.push(goalMotion);
            // OMPL_DEBUG("solution cost: %lf", goalMotion->cost.value());
        }
        // OMPL_DEBUG("best cost: %lf", bestGoalMotion_->cost.value());
        // OMPL_DEBUG("top cost: %lf", goal_motion_que.top()->cost.value());
        goalMotions_.clear();

    }
    

    // OMPL_INFORM("%d solutions ready to repropagate", pdef_->getSolutionCount());
    OMPL_INFORM("%d solutions ready to repropagate", goal_motion_que.size());
    // std::vector< ob::PlannerSolution > solutions = pdef_->getSolutions();
    pdef_->clearSolutionPaths();
    // for(int i=0 ; i<solutions.size(); i++)
    int i = 0;
    if(bestGoalMotion_)
    {
        assert(bestGoalMotion_->cost.value() == goal_motion_que.top()->cost.value());
    }
    while(!goal_motion_que.empty())
    {
        i++;
        Motion *current_motion = goal_motion_que.top();
        if(bestGoalMotion_)
        {
            bestGoalMotion_ = goal_motion_que.top();
            bestCost_ = bestGoalMotion_->cost;
        }
        
        Motion *iter_motion = current_motion;
        double approach_dis{std::numeric_limits<double>::infinity()};
        ob::State *solution_nearst_state;
        int index = 0;
        while(iter_motion)
        {
            if(si_->distance(current_state, iter_motion->state) < approach_dis)
            {
                approach_dis = si_->distance(current_state, iter_motion->state);
                solution_nearst_state = iter_motion->state;
                index ++;
            }
            iter_motion = iter_motion->parent;
        }

        OMPL_DEBUG("%d th solution nearest index %d, dis: %lf", i, index, approach_dis);
        bool path_validity = true;
        if(approach_dis < 0.5 || model_mv_->checkMotion(current_state, solution_nearst_state))
        {
            // set the solution path
            std::vector<Motion *> mpath;
            Motion *iterMotion = current_motion;
            while (iterMotion->state != solution_nearst_state)
            {         
                if(!si_->isValid(iterMotion->state))
                {
                    // si_->printState(iterMotion->state);
                    path_validity = false;
                    OMPL_DEBUG("%d th solution recheck failed", i);
                    break;
                }
                mpath.push_back(iterMotion);
                iterMotion = iterMotion->parent;
            }
            
            if(path_validity && !result)
            {
                time_stamps.clear();
                best_path = std::make_shared<ompl::geometric::PathGeometric>(si_);
                for (int i = mpath.size() - 1; i >= 0; --i)
                {
                    best_path->append(mpath[i]->state);
                    time_stamps.push_back(0.1);
                }
                // Add the solution path.
                ob::PlannerSolution psol(best_path);
                pdef_->addSolutionPath(psol);
                result = true;
                rePropagation_flag_ = true;
                // return result;
            }
            if(path_validity)
                goalMotions_.push_back(current_motion);
            
        }
        else
        {
            OMPL_INFORM("%d th solution repropagation failed.", i);
        }
        goal_motion_que.pop();

    }
    if(result)
    {
        OMPL_INFORM("repropagation success");
        return result;
    }
    else
    {
        OMPL_WARN("repropagation failed and send stop trajecotry");
        best_path = std::make_shared<ompl::geometric::PathGeometric>(si_);
        best_path->append(current_state);
        // Add the solution path.
        ob::PlannerSolution psol(best_path);
        pdef_->addSolutionPath(psol);
        rePropagation_flag_ = false;
        return result;
    }
    // bestGoalMotion_ = nullptr;
    // approxGoalMotion_ = nullptr;
    // approxDist_= std::numeric_limits<double>::infinity();

    // return result;
}

void CL_RRTstar::getNeighbors(Motion *motion, std::vector<Motion *> &nbh) const
{
    auto cardDbl = static_cast<double>(nn_->size() + 1u);
    if (useKNearest_)
    {
        //- k-nearest RRT*
        unsigned int k = std::ceil(k_rrt_ * log(cardDbl));
        nn_->nearestK(motion, k, nbh);
    }
    else
    {
        double r = std::min(
            maxDistance_, r_rrt_ * std::pow(log(cardDbl) / cardDbl, 1 / static_cast<double>(sample_dimension_)));
        nn_->nearestR(motion, r, nbh);
    }
}

void CL_RRTstar::removeFromParent(Motion *m)
{
    for (auto it = m->parent->children.begin(); it != m->parent->children.end(); ++it)
    {
        if (*it == m)
        {
            m->parent->children.erase(it);
            break;
        }
    }
}

void CL_RRTstar::updateChildCosts(Motion *m)
{
    for (std::size_t i = 0; i < m->children.size(); ++i)
    {
        m->children[i]->cost = opt_->combineCosts(m->cost, m->children[i]->incCost);
        m->children[i]->time = m->time + m->children[i]->intime;
        updateChildCosts(m->children[i]);
    }
}

void CL_RRTstar::freeMemory()
{
    if (nn_)
    {
        std::vector<Motion *> motions;
        nn_->list(motions);
        for (auto &motion : motions)
        {
            if (motion->state)
                si_->freeState(motion->state);
            delete motion;
        }
    }
}

void CL_RRTstar::getPlannerData(ob::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion *> motions;
    if (nn_)
        nn_->list(motions);

    if (bestGoalMotion_)
        data.addGoalVertex(ob::PlannerDataVertex(bestGoalMotion_->state));

    for (auto &motion : motions)
    {
        if (motion->parent == nullptr)
            data.addStartVertex(ob::PlannerDataVertex(motion->state));
        else
            data.addEdge(ob::PlannerDataVertex(motion->parent->state), ob::PlannerDataVertex(motion->state));
    }
}

int CL_RRTstar::pruneTree(const ob::Cost &pruneTreeCost)
{
    // Variable
    // The percent improvement (expressed as a [0,1] fraction) in cost
    double fracBetter;
    // The number pruned
    int numPruned = 0;

    if (opt_->isFinite(prunedCost_))
    {
        fracBetter = std::abs((pruneTreeCost.value() - prunedCost_.value()) / prunedCost_.value());
    }
    else
    {
        fracBetter = 1.0;
    }

    if (fracBetter > pruneThreshold_)
    {
        // We are only pruning motions if they, AND all descendents, have a estimated cost greater than pruneTreeCost
        // The easiest way to do this is to find leaves that should be pruned and ascend up their ancestry until a
        // motion is found that is kept.
        // To avoid making an intermediate copy of the NN structure, we process the tree by descending down from the
        // start(s).
        // In the first pass, all Motions with a cost below pruneTreeCost, or Motion's with children with costs below
        // pruneTreeCost are added to the replacement NN structure,
        // while all other Motions are stored as either a 'leaf' or 'chain' Motion. After all the leaves are
        // disconnected and deleted, we check
        // if any of the the chain Motions are now leaves, and repeat that process until done.
        // This avoids (1) copying the NN structure into an intermediate variable and (2) the use of the expensive
        // NN::remove() method.

        // Variable
        // The queue of Motions to process:
        std::queue<Motion *, std::deque<Motion *>> motionQueue;
        // The list of leaves to prune
        std::queue<Motion *, std::deque<Motion *>> leavesToPrune;
        // The list of chain vertices to recheck after pruning
        std::list<Motion *> chainsToRecheck;

        // Clear the NN structure:
        nn_->clear();

        // Put all the starts into the NN structure and their children into the queue:
        // We do this so that start states are never pruned.
        for (auto &startMotion : startMotions_)
        {
            // Add to the NN
            nn_->add(startMotion);

            // Add their children to the queue:
            addChildrenToList(&motionQueue, startMotion);
        }

        while (motionQueue.empty() == false)
        {
            // Test, can the current motion ever provide a better solution?
            if (keepCondition(motionQueue.front(), pruneTreeCost))
            {
                // Yes it can, so it definitely won't be pruned
                // Add it back into the NN structure
                nn_->add(motionQueue.front());

                // Add it's children to the queue
                addChildrenToList(&motionQueue, motionQueue.front());
            }
            else
            {
                // No it can't, but does it have children?
                if (motionQueue.front()->children.empty() == false)
                {
                    // Yes it does.
                    // We can minimize the number of intermediate chain motions if we check their children
                    // If any of them won't be pruned, then this motion won't either. This intuitively seems
                    // like a nice balance between following the descendents forever.

                    // Variable
                    // Whether the children are definitely to be kept.
                    bool keepAChild = false;

                    // Find if any child is definitely not being pruned.
                    for (unsigned int i = 0u; keepAChild == false && i < motionQueue.front()->children.size(); ++i)
                    {
                        // Test if the child can ever provide a better solution
                        keepAChild = keepCondition(motionQueue.front()->children.at(i), pruneTreeCost);
                    }

                    // Are we *definitely* keeping any of the children?
                    if (keepAChild)
                    {
                        // Yes, we are, so we are not pruning this motion
                        // Add it back into the NN structure.
                        nn_->add(motionQueue.front());
                    }
                    else
                    {
                        // No, we aren't. This doesn't mean we won't though
                        // Move this Motion to the temporary list
                        chainsToRecheck.push_back(motionQueue.front());
                    }

                    // Either way. add it's children to the queue
                    addChildrenToList(&motionQueue, motionQueue.front());
                }
                else
                {
                    // No, so we will be pruning this motion:
                    leavesToPrune.push(motionQueue.front());
                }
            }

            // Pop the iterator, std::list::erase returns the next iterator
            motionQueue.pop();
        }

        // We now have a list of Motions to definitely remove, and a list of Motions to recheck
        // Iteratively check the two lists until there is nothing to to remove
        while (leavesToPrune.empty() == false)
        {
            // First empty the current leaves-to-prune
            while (leavesToPrune.empty() == false)
            {
                // If this leaf is a goal, remove it from the goal set
                if (leavesToPrune.front()->inGoal == true)
                {
                    // Warn if pruning the _best_ goal
                    if (leavesToPrune.front() == bestGoalMotion_)
                    {
                        OMPL_ERROR("%s: Pruning the best goal.", getName().c_str());
                    }
                    // Remove it
                    goalMotions_.erase(std::remove(goalMotions_.begin(), goalMotions_.end(), leavesToPrune.front()),
                                    goalMotions_.end());
                }

                // Remove the leaf from its parent
                removeFromParent(leavesToPrune.front());

                // Erase the actual motion
                // First free the state
                si_->freeState(leavesToPrune.front()->state);
                // siC_->freeControl(leavesToPrune.front()->control);

                // then delete the pointer
                delete leavesToPrune.front();

                // And finally remove it from the list, erase returns the next iterator
                leavesToPrune.pop();

                // Update our counter
                ++numPruned;
            }

            // Now, we need to go through the list of chain vertices and see if any are now leaves
            auto mIter = chainsToRecheck.begin();
            while (mIter != chainsToRecheck.end())
            {
                // Is the Motion a leaf?
                if ((*mIter)->children.empty() == true)
                {
                    // It is, add to the removal queue
                    leavesToPrune.push(*mIter);

                    // Remove from this queue, getting the next
                    mIter = chainsToRecheck.erase(mIter);
                }
                else
                {
                    // Is isn't, skip to the next
                    ++mIter;
                }
            }
        }

        // Now finally add back any vertices left in chainsToReheck.
        // These are chain vertices that have descendents that we want to keep
        for (const auto &r : chainsToRecheck)
            // Add the motion back to the NN struct:
            nn_->add(r);

        // All done pruning.
        // Update the cost at which we've pruned:
        prunedCost_ = pruneTreeCost;

        // And if we're using the pruned measure, the measure to which we've pruned
        if (usePrunedMeasure_)
        {
            prunedMeasure_ = infSampler_->getInformedMeasure(prunedCost_);

            if (useKNearest_ == false)
            {
                calculateRewiringLowerBounds();
            }
        }
        // No else, prunedMeasure_ is the si_ measure by default.
    }

    return numPruned;
}

void CL_RRTstar::addChildrenToList(std::queue<Motion *, std::deque<Motion *>> *motionList, Motion *motion)
{
    for (auto &child : motion->children)
    {
        motionList->push(child);
    }
}

bool CL_RRTstar::keepCondition(const Motion *motion, const ob::Cost &threshold) const
{
    // We keep if the cost-to-come-heuristic of motion is <= threshold, by checking
    // if !(threshold < heuristic), as if b is not better than a, then a is better than, or equal to, b
    if (bestGoalMotion_ && motion == bestGoalMotion_)
    {
        // If the threshold is the theoretical minimum, the bestGoalMotion_ will sometimes fail the test due to floating point precision. Avoid that.
        return true;
    }

    return !opt_->isCostBetterThan(threshold, solutionHeuristic(motion));
}

ompl::base::Cost CL_RRTstar::solutionHeuristic(const Motion *motion) const
{
    ob::Cost costToCome;
    if (useAdmissibleCostToCome_)
    {
        // Start with infinite cost
        costToCome = opt_->infiniteCost();

        // Find the min from each start
        for (auto &startMotion : startMotions_)
        {
            costToCome = opt_->betterCost(
                costToCome, opt_->motionCost(startMotion->state,
                                            motion->state));  // lower-bounding cost from the start to the state
        }
    }
    else
    {
        costToCome = motion->cost;  // current cost from the state to the goal
    }

    const ob::Cost costToGo =
        opt_->costToGo(motion->state, pdef_->getGoal().get());  // lower-bounding cost from the state to the goal
    return opt_->combineCosts(costToCome, costToGo);            // add the two costs
}

void CL_RRTstar::setTreePruning(const bool prune)
{
    if (static_cast<bool>(opt_) == true)
    {
        if (opt_->hasCostToGoHeuristic() == false)
        {
            OMPL_INFORM("%s: No cost-to-go heuristic set. Informed techniques will not work well.", getName().c_str());
        }
    }

    // If we just disabled tree pruning, but we wee using prunedMeasure, we need to disable that as it required myself
    if (prune == false && getPrunedMeasure() == true)
    {
        setPrunedMeasure(false);
    }

    // Store
    useTreePruning_ = prune;
}

void CL_RRTstar::setPrunedMeasure(bool informedMeasure)
{
    if (static_cast<bool>(opt_) == true)
    {
        if (opt_->hasCostToGoHeuristic() == false)
        {
            OMPL_INFORM("%s: No cost-to-go heuristic set. Informed techniques will not work well.", getName().c_str());
        }
    }

    // This option only works with informed sampling
    if (informedMeasure == true && (useInformedSampling_ == false || useTreePruning_ == false))
    {
        OMPL_ERROR("%s: InformedMeasure requires InformedSampling and TreePruning.", getName().c_str());
    }

    // Check if we're changed and update parameters if we have:
    if (informedMeasure != usePrunedMeasure_)
    {
        // Store the setting
        usePrunedMeasure_ = informedMeasure;

        // Update the prunedMeasure_ appropriately, if it has been configured.
        if (setup_ == true)
        {
            if (usePrunedMeasure_)
            {
                prunedMeasure_ = infSampler_->getInformedMeasure(prunedCost_);
            }
            else
            {
                prunedMeasure_ = si_->getSpaceMeasure();
            }
        }

        // And either way, update the rewiring radius if necessary
        if (useKNearest_ == false)
        {
            calculateRewiringLowerBounds();
        }
    }
}

void CL_RRTstar::setInformedSampling(bool informedSampling)
{
    if (static_cast<bool>(opt_) == true)
    {
        if (opt_->hasCostToGoHeuristic() == false)
        {
            OMPL_INFORM("%s: No cost-to-go heuristic set. Informed techniques will not work well.", getName().c_str());
        }
    }

    // This option is mutually exclusive with setSampleRejection, assert that:
    if (informedSampling == true && useRejectionSampling_ == true)
    {
        OMPL_ERROR("%s: InformedSampling and SampleRejection are mutually exclusive options.", getName().c_str());
    }

    // If we just disabled tree pruning, but we are using prunedMeasure, we need to disable that as it required myself
    if (informedSampling == false && getPrunedMeasure() == true)
    {
        setPrunedMeasure(false);
    }

    // Check if we're changing the setting of informed sampling. If we are, we will need to create a new sampler, which
    // we only want to do if one is already allocated.
    if (informedSampling != useInformedSampling_)
    {
        // If we're disabled informedSampling, and prunedMeasure is enabled, we need to disable that
        if (informedSampling == false && usePrunedMeasure_ == true)
        {
            setPrunedMeasure(false);
        }

        // Store the value
        useInformedSampling_ = informedSampling;

        // If we currently have a sampler, we need to make a new one
        if (sampler_ || infSampler_)
        {
            // Reset the samplers
            sampler_.reset();
            infSampler_.reset();

            // Create the sampler
            allocSampler();
        }
    }
}

void CL_RRTstar::setSampleRejection(const bool reject)
{
    if (static_cast<bool>(opt_) == true)
    {
        if (opt_->hasCostToGoHeuristic() == false)
        {
            OMPL_INFORM("%s: No cost-to-go heuristic set. Informed techniques will not work well.", getName().c_str());
        }
    }

    // This option is mutually exclusive with setInformedSampling, assert that:
    if (reject == true && useInformedSampling_ == true)
    {
        OMPL_ERROR("%s: InformedSampling and SampleRejection are mutually exclusive options.", getName().c_str());
    }

    // Check if we're changing the setting of rejection sampling. If we are, we will need to create a new sampler, which
    // we only want to do if one is already allocated.
    if (reject != useRejectionSampling_)
    {
        // Store the setting
        useRejectionSampling_ = reject;

        // If we currently have a sampler, we need to make a new one
        if (sampler_ || infSampler_)
        {
            // Reset the samplers
            sampler_.reset();
            infSampler_.reset();

            // Create the sampler
            allocSampler();
        }
    }
}

void CL_RRTstar::setOrderedSampling(bool orderSamples)
{
    // Make sure we're using some type of informed sampling
    if (useInformedSampling_ == false && useRejectionSampling_ == false)
    {
        OMPL_ERROR("%s: OrderedSampling requires either informed sampling or rejection sampling.", getName().c_str());
    }

    // Check if we're changing the setting. If we are, we will need to create a new sampler, which we only want to do if
    // one is already allocated.
    if (orderSamples != useOrderedSampling_)
    {
        // Store the setting
        useOrderedSampling_ = orderSamples;

        // If we currently have a sampler, we need to make a new one
        if (sampler_ || infSampler_)
        {
            // Reset the samplers
            sampler_.reset();
            infSampler_.reset();

            // Create the sampler
            allocSampler();
        }
    }
}

void CL_RRTstar::allocSampler()
{
    // Allocate the appropriate type of sampler.
    if (useInformedSampling_)
    {
        // We are using informed sampling, this can end-up reverting to rejection sampling in some cases
        OMPL_INFORM("%s: Using informed sampling.", getName().c_str());
        infSampler_ = opt_->allocInformedStateSampler(pdef_, numSampleAttempts_);
    }
    else if (useRejectionSampling_)
    {
        // We are explicitly using rejection sampling.
        OMPL_INFORM("%s: Using rejection sampling.", getName().c_str());
        infSampler_ = std::make_shared<ob::RejectionInfSampler>(pdef_, numSampleAttempts_);
    }
    else
    {
        // We are using a regular sampler
        sampler_ = si_->allocStateSampler();
    }

    // Wrap into a sorted sampler
    if (useOrderedSampling_ == true)
    {
        infSampler_ = std::make_shared<ob::OrderedInfSampler>(infSampler_, batchSize_);
    }
    // No else
}

bool CL_RRTstar::sampleUniform(ob::State *statePtr)
{
    // Use the appropriate sampler
    if (useInformedSampling_ || useRejectionSampling_)
    {
        // Attempt the focused sampler and return the result.
        // If bestCost is changing a lot by small amounts, this could
        // be prunedCost_ to reduce the number of times the informed sampling
        // transforms are recalculated.
        return infSampler_->sampleUniform(statePtr, bestCost_);
    }
    else
    {
        // Simply return a state from the regular sampler
        sampler_->sampleUniform(statePtr);

        // Always true
        return true;
    }
}

void CL_RRTstar::calculateRewiringLowerBounds()
{
    // const auto dimDbl = static_cast<double>(si_->getStateDimension());
    double dimDbl = static_cast<double>(sample_dimension_);

    // k_rrt > 2^(d + 1) * e * (1 + 1 / d).  K-nearest RRT*
    k_rrt_ = rewireFactor_ * (std::pow(2, dimDbl + 1) * boost::math::constants::e<double>() * (1.0 + 1.0 / dimDbl));

    // r_rrt > (2*(1+1/d))^(1/d)*(measure/ballvolume)^(1/d)
    // If we're not using the informed measure, prunedMeasure_ will be set to si_->getSpaceMeasure();
    r_rrt_ =
        rewireFactor_ *
        std::pow(2 * (1.0 + 1.0 / dimDbl) * (prunedMeasure_ / ompl::unitNBallMeasure(sample_dimension_)), 1.0 / dimDbl);
}

}