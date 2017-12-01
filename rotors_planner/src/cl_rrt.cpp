#include "cl_rrt.h"
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/tools/config/SelfConfig.h>
#include <limits>


CL_rrt::CL_rrt(const oc::SpaceInformationPtr &si) : ob::Planner(si,"CL_rrt")
{
  specs_.approximateSolutions = true;
  // specs_.directed = false;
  siC_= si.get();

  Planner::declareParam<double>("range", this, &CL_rrt::setRange, &CL_rrt::getRange, "0.:1.:10000." );
  Planner::declareParam<double>("goal_bias",this, &CL_rrt::setGoalBias, &CL_rrt::getGoalBias, "0.:.05:1.");

}

CL_rrt::~CL_rrt()
{
  freeMemory();
}

void CL_rrt::clear()
{
  Planner::clear();
  sampler_.reset();
  freeMemory();
  if(!nn_)
    nn_->clear();
  lastGoalMotion= nullptr;
  // something else declare in this class need to be cleaned
}

void CL_rrt::setup()
{
  Planner::setup();
  if (!nn_)
    nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));

  nn_->setDistanceFunction([this](const Motion *a, const Motion *b)
      {
         return distanceFunction(a, b);
      });

  //ompl::tools::SelfConfig sc(si_,getName());
  //sc.configurePlannerRange(maxDistance_);
}

void CL_rrt::freeMemory()
{
  if(nn_)
  {
    std::vector<Motion *> motions;
    nn_->list(motions);
    for (auto &motion : motions)
    {
      if(motion->state)
        si_->freeState(motion->state);
      if(motion->control)
        siC_->freeControl(motion->control);
      delete motion;
    }
  }

}

bool CL_rrt::propagatewhilestop(const ompl::base::State *state, const ompl::control::Control *control, std::vector<ompl::base::State *> &result) const
{

}

ob::PlannerStatus CL_rrt::solve(const ob::PlannerTerminationCondition &ptc)
{
  checkValidity();
  ob::Goal *goal = pdef_->getGoal().get();
  // ob::State *goal_state = pdef_->getGoal()->as<ob::GoalState>()->getState();
  auto *goal_s = dynamic_cast <ob::GoalSampleableRegion *>(goal);

  while(const ob::State *st = pis_.nextStart())
  {
    auto *motion = new Motion(siC_);
    si_->copyState(motion->state, st);
    siC_->nullControl(motion->control);
    nn_->add(motion);
  }

  if(nn_->size() == 0)
  {
    OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
    return ob::PlannerStatus::INVALID_START;
  }

  if(!sampler_)
    sampler_ = si_->allocStateSampler();
  OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());

  Motion *solution = nullptr;
  Motion *approxsol = nullptr;
  double approxdif = std::numeric_limits<double>::infinity();

  auto *rmotion = new Motion(siC_);
  ob::State *rstate = rmotion->state;
  oc::Control *rctrl = rmotion->control;
  ob::State *xstate = si_->allocState();

  while(ptc == false)
  {
    /* sample random state based on current state */
    if(goal_s && rng_.uniform01() < goalBias_ && goal_s->canSample())
      goal_s->sampleGoal(rstate);
    else
      sampler_->sampleUniform(rstate);

    /* find  closest state in the tree  */
    // we can choose R closest states either

    Motion *nmotion = nn_->nearest(rmotion);
    /*std::vector<Motion *> Nmotion ;
    nn_->nearestR(rmotion,radius,Nmotion); */

    /* get controller output using Controller function */
    while(!Controlfn_(nmotion->state,rctrl))
    {
      // OMPL_DEBUG("invalid sample state");
      sampler_->sampleUniform(rstate);
    }

    std::vector<ob::State *> pstates;
    if(propagateuntilstop(nmotion->state, rctrl, pstates))
    {
      if(pstates.size() >= siC_->getMinControlDuration()){

      Motion *lastmotion = nmotion;
      bool solved = false;
      size_t p = 0;
      for(; p < pstates.size(); ++p)
      {
        auto *motion = new Motion();
        motion->state = pstates[p];
        // we need multiple copies of rctrl
        motion->control = siC_->allocControl();
        siC_->copyControl(motion->control, rctrl);
        motion->steps = 1;
        motion->parent = lastmotion;
        //motion->low_bound = si_->distance(motion->state, goal_state);

        double dist = 0.0;
        solved = goal->isSatisfied(motion->state, &dist);
        if(solved)
          goal_solve = true;
        motion->low_bound = dist;
        lastmotion = motion;
        nn_->add(motion);

        if(motion->low_bound < approxdif)
        {
          approxdif = dist;
          solution = motion;

        }

        /* for each newly added nodes form C_UB and add goal node to tree */

      }
      while((++p < pstates.size()))
      {
        si_->freeState(pstates[p]);
      }
      if(solved)
        break;
    }
      else
        for (auto &pstate :pstates)
          si_->freeState(pstate);
    }
  }

  bool approximate =false;
  if(!goal_solve)
    approximate = true;

  if(solution != nullptr){

  lastGoalMotion_ = solution;

  /* construct the solution path */
  std::vector<Motion *> mpath;
  while(solution != nullptr)
  {
    mpath.push_back(solution);
    solution = solution->parent;
  }

  /* set the solution path */
  auto path(std::make_shared<oc::PathControl>(si_));
  for (int i = mpath.size()-1; i >= 0; --i)
    if(mpath[i]->parent)
      path->append(mpath[i]->state,mpath[i]->control, mpath[i]->steps*siC_->getPropagationStepSize());
  else
      path->append(mpath[i]->state);
  pdef_->addSolutionPath(path, approximate, approxdif, getName());

  }

  if(rmotion->state)
    si_->freeState(rmotion->state);
  if(rmotion->control)
    siC_->freeControl(rmotion->control);
  delete rmotion;
  si_->freeState(xstate);
  OMPL_INFORM("%s: Created %u states", getName().c_str(), nn_->size());
  return base::PlannerStatus(solved, approximate);




}

void CL_rrt::getPlannerData(ob::PlannerData &data) const
{
  Planner::getPlannerData(data);

  std::vector<Motion *> motions;
  if(nn_)
    nn_->list(motions);

  double delta = siC_->getPropagationStepSize();

  if(lastGoalMotion_)
    data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));

  for(auto m : motions)
  {
    if(m->parent)
    {
      if(data.hasControls())
        data.addEdge(ob::PlannerDataVertex(m->parent->state), ob::PlannerDataVertex(m->state), oc::PlannerDataEdgeControl(m->control, m->steps*delta));
      else
        data.addEdge(ob::PlannerDataVertex(m->parent->state),ob::PlannerDataVertex(m->state));
    }
    else
      data.addStartVertex(ob::PlannerDataVertex(m->state));
    }
  }

}
