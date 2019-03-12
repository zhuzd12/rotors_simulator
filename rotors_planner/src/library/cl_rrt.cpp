#include "rotors_planner/cl_rrt.h"
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/tools/config/SelfConfig.h>
#include <limits>
#include <ompl/control/planners/PlannerIncludes.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/base/goals/GoalState.h>
#include <functional>
#include <time.h>
#include <stack>
#include <unistd.h>

namespace rotors_planner {

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace oc = ompl::control;

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
  if(nn_)
    nn_->clear();
  lastGoalMotion_= nullptr;
  // something else declare in this class need to be cleaned
}

void CL_rrt::setup()
{
  Planner::setup();
  if(goal_solve)
    goal_solve = false;

  if (!nn_)
  {
    nn_.reset(new ompl::NearestNeighborsLinear<Motion *>());
    //nn_.reset(ompl::tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
  }

  nn_->setDistanceFunction([this](const Motion *a, const Motion *b)
      {
         if(a->commit_protect || b->commit_protect)
           return std::numeric_limits<double>::infinity();
         else
           return distanceFunction(a, b);
      });

  ompl::tools::SelfConfig sc(si_,getName());
  sc.configurePlannerRange(maxDistance_);
}

void CL_rrt::freeMemory()
{
  // OMPL_INFORM("begin to freeMemory, size: %d", nn_->size());
  if(nn_)
  {
    std::vector<Motion *> motions;
    nn_->list(motions);
    for (auto &motion : motions)
    {
      if(motion->state)
      {
        si_->freeState(motion->state);
      }
      if(motion->control)
        siC_->freeControl(motion->control);

      delete motion;
    }
  }
  // OMPL_INFORM("freeMemory done");
}

bool CL_rrt::propagateuntilstop(const ob::State *state, const ob::State *heading_state, std::vector<ob::State *> &state_sequence, std::vector<oc::Control *> &control_sequence, std::vector<double>& cum_distance,  std::vector<int>& cum_steps) const
{
  // OMPL_INFORM("propagate begins");
  double dis_to_target_ = std::numeric_limits<double>::infinity();
  int st = 0;
  int st_loop = 0;
  int action_cost_steps = 0;
  double accumulate_distance_ = 0.0;

  oc::Control *rctrl = siC_->allocControl();
  ob::State *current_State = si_->allocState();
  ob::State *future_State = si_->allocState();
  si_->copyState(current_State, state);
  while(dis_to_target_ >= path_deviation){
    Controlfn_(current_State, heading_state, rctrl);
    siC_->getStatePropagator()->propagate(current_State, rctrl, siC_->getPropagationStepSize(), future_State);

    /*
    const ob::CompoundStateSpace::StateType *s = result[st]->as<ob::CompoundStateSpace::StateType>();
    std::cout<<"result"<<" "<<st<<": "<<std::endl<<s->as<ob::RealVectorStateSpace::StateType>(0)->values[0]<<" "
            <<s->as<ob::RealVectorStateSpace::StateType>(0)->values[1]<<" "
           <<s->as<ob::RealVectorStateSpace::StateType>(0)->values[2]<<std::endl;
    std::cout<<s->as<ob::RealVectorStateSpace::StateType>(1)->values[0]<<" "
              <<s->as<ob::RealVectorStateSpace::StateType>(1)->values[1]<<" "
             <<s->as<ob::RealVectorStateSpace::StateType>(1)->values[2]<<std::endl;
    std::cout<<s->as<ob::RealVectorStateSpace::StateType>(2)->values[0]<<" "
              <<s->as<ob::RealVectorStateSpace::StateType>(2)->values[1]<<" "
             <<s->as<ob::RealVectorStateSpace::StateType>(2)->values[2]<<std::endl;
    std::cout<<s->as<ob::RealVectorStateSpace::StateType>(3)->values[0]<<" "
              <<s->as<ob::RealVectorStateSpace::StateType>(3)->values[1]<<" "
             <<s->as<ob::RealVectorStateSpace::StateType>(3)->values[2]<<std::endl;
    */

    if(!si_->isValid(future_State))
    {
      si_->freeState(future_State);
      si_->freeState(current_State);
      return false;
    }

    accumulate_distance_ = accumulate_distance_ + si_->distance(current_State, future_State);
    action_cost_steps ++;
    dis_to_target_ = si_->distance(future_State, heading_state);
    if(accumulate_distance_ >= path_resolution || dis_to_target_ < path_deviation)
    {
      st++;
      control_sequence.resize(st);
      state_sequence.resize(st);
      control_sequence[st-1] = siC_->allocControl();
      siC_->copyControl(control_sequence[st-1], rctrl);
      state_sequence[st-1] = si_->allocState();
      si_->copyState(state_sequence[st-1], future_State);
      cum_distance.resize(st);
      cum_distance[st-1] = accumulate_distance_;
      cum_steps.resize(st);
      cum_steps[st-1] = action_cost_steps;

      accumulate_distance_ = 0.0;
      action_cost_steps = 0;

    }

    si_->copyState(current_State, future_State);
    ++st_loop;
    if(st_loop > 300)
    {
      OMPL_INFORM("propagate endless loop");
      si_->freeState(current_State);
      si_->freeState(future_State);
      return false;
    }
  }
  si_->freeState(current_State);
  si_->freeState(future_State);
  return true;
}

ob::PlannerStatus CL_rrt::solve(const ob::PlannerTerminationCondition &ptc)
{
  checkValidity();
  ob::Goal *goal = pdef_->getGoal().get();
  ob::State *goal_state = pdef_->getGoal().get()->as<ob::GoalState>()->getState();
  // auto *goal_s = dynamic_cast <ob::GoalSampleableRegion *>(goal);

  while(const ob::State *st = pis_.nextStart())
  {
    auto *motion = new Motion(siC_);
    si_->copyState(motion->state, st);
    siC_->nullControl(motion->control);
    nn_->add(motion);
    motion->c_total = 0.0;
    goal->isSatisfied(motion->state, &motion->low_bound);
    current_root = motion;
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
  //Motion *approxsol = nullptr;
  double approxdif = std::numeric_limits<double>::infinity();

  auto *rmotion = new Motion(siC_);
  ob::State *rstate = rmotion->state;
  oc::Control *rctrl = rmotion->control;
  ob::State *xstate = si_->allocState();

  int loop_times = 0;
  /* keep expanding the tree until time up, enven when we hit the goal */
  while(ptc == false)
  {
    loop_times++;
    /* sample random state based on current state */
    // if(goal_s && rng_.uniform01() < goalBias_ && goal_s->canSample())
    //   goal_s->sampleGoal(rstate);
    // else
    //   sampler_->sampleUniform(rstate);
    sampler_->sampleUniform(rstate);

    /* find  closest state in the tree  */
    Motion *nmotion = nn_->nearest(rmotion);
    ob::State *dstate = rstate;

    /* we can choose R closest states either */
    // std::vector<Motion *> Nmotion ;
    // nn_->nearestR(rmotion,radius,Nmotion); 

    double d = si_->distance(nmotion->state, rstate);
    if(d > maxDistance_)
    {
      si_->getStateSpace()->interpolate(nmotion->state, rstate, maxDistance_/d, xstate);
      dstate = xstate;
    }

    /* get controller output using Controller function */
    while(!Controlfn_(nmotion->state, dstate, rctrl) || (!goal_solve && si_->distance(nmotion->state, dstate) < minDistance_))
    {
      // OMPL_DEBUG("invalid sample state");
      sampler_->sampleUniform(rstate);
      d = si_->distance(nmotion->state, rstate);
      if(d > maxDistance_)
      {
        si_->getStateSpace()->interpolate(nmotion->state, rstate, maxDistance_/d, xstate);
        dstate = xstate;
      }
    }

    /* here we set velocities and angular velocities to 0 */
    ob::CompoundStateSpace::StateType& s = *dstate->as<ob::CompoundStateSpace::StateType>();
    double *check_angular= s.as<ob::RealVectorStateSpace::StateType>(0)->values;
    double *check_position= s.as<ob::RealVectorStateSpace::StateType>(1)->values;
    double *check_velocity = s.as<ob::RealVectorStateSpace::StateType>(2)->values;
    double *check_angular_vel = s.as<ob::RealVectorStateSpace::StateType>(3)->values;
    check_angular[0] = 0.0; check_angular[1] = 0.0;
    //check_angular[2] = 0.0;
    check_velocity[0] = 0.0; check_velocity[1] = 0.0; check_velocity[2] = 0.0;
    check_angular_vel[0] = 0.0; check_angular_vel[1] = 0.0; check_angular_vel[2] = 0.0;
    // OMPL_INFORM("%d loop : sampled position and yaw: %lf %lf %lf %lf ", loop_times, check_position[0], check_position[1], check_position[2], check_angular[2]);

    si_->copyState(rmotion->state, dstate);
    std::vector<ob::State *> pstates;  // every expand loop, pstates point to the newly added tree nodes
    std::vector<oc::Control *> pcontrols;
    std::vector<double> cum_distance;
    std::vector<int> cum_steps;
    if(propagateuntilstop(nmotion->state, rmotion->state, pstates, pcontrols, cum_distance, cum_steps))
    {
      // OMPL_INFORM("propagate success");
      if(pstates.size() >= siC_->getMinControlDuration()){

      Motion *lastmotion = nmotion;
      bool solved = false;
      size_t p = 0;
      double dist = 0.0;
  
      for(; p < pstates.size(); ++p)
      {
        auto *motion = new Motion();
        motion->state = pstates[p];
        motion->control = pcontrols[p];
        motion->steps = cum_steps[p];
        motion->c_total = lastmotion->c_total + cum_distance[p];
        motion->parent = lastmotion;
        motion->low_bound = si_->distance(motion->state, goal_state);

        solved = goal->isSatisfied(motion->state, &dist);
        motion->low_bound = dist;
        if(solved)
        {
          OMPL_INFORM("Found solution in stage 1, appro: %lf", dist);
          goal_solve = true;
          motion->up_bound = motion->low_bound;
         // back to root and update upper_bound
          Motion *back_motion = motion;
          Motion *father_motion = motion;
          bool is_optimal = true;
          while(back_motion->parent != nullptr)
          {
            father_motion = back_motion->parent;
            if(father_motion->up_bound > back_motion->up_bound + back_motion->c_total - father_motion->c_total)
            {
              father_motion->up_bound = back_motion->up_bound + back_motion->c_total - father_motion->c_total;
              back_motion = father_motion;
            }
            else
            {
              is_optimal = false;
              break;
            }
          }
          if(is_optimal)
          {
            solution = motion;
            approxdif = solution->low_bound;
          }

          // std::vector<Motion *> motions;
          // nn_->list(motions);
          // OMPL_INFORM("DEBUG: after stage 1 nn_ size: %d, motions size: %d", nn_->size(), motions.size());
        }

        else
        {
          std::vector<ob::State *> togo_states;
          std::vector<oc::Control *> togo_controls;
          std::vector<double> togo_distance;
          std::vector<int> togo_steps;
          if(propagateuntilstop(motion->state, goal_state, togo_states, togo_controls, togo_distance, togo_steps))
          {
            // OMPL_INFORM("togo propagate success");
            // OMPL_INFORM("Found solution in stage 2, appro: %lf", si_->distance(togo_states[togo_states.size()-1], goal_state));
            goal_solve = true;
            // add the path to nn_
            size_t k = 0;
            Motion *togo_lastmotion = motion;
            // OMPL_INFORM("porpagate to goal steps: %d ", togo_states.size());
            for(; k < togo_states.size(); ++k)
            {
              auto *togo_motion = new Motion();
              togo_motion->state = togo_states[k];
              togo_motion->control = togo_controls[k];
              togo_motion->steps = togo_steps[k];
              togo_motion->c_total = togo_lastmotion->c_total + togo_distance[k];
              togo_motion->parent = togo_lastmotion;
              togo_motion->low_bound = si_->distance(togo_motion->state, goal_state);

              togo_lastmotion->children.push_back(togo_motion);
              togo_lastmotion = togo_motion;

              //nn_->rebuildDataStructure();
              nn_->add(togo_motion);
            }
            togo_lastmotion->up_bound = si_->distance(togo_lastmotion->state, goal_state);

            // back to root and update upper_bound
            Motion *back_motion;
            Motion *father_motion;
            back_motion = togo_lastmotion;
            bool is_optimal = true;

            while(back_motion->parent != nullptr)
            {
              father_motion = back_motion->parent;
              if(father_motion->up_bound > back_motion->up_bound + back_motion->c_total - father_motion->c_total)
              {
                father_motion->up_bound = back_motion->up_bound + back_motion->c_total - father_motion->c_total;
                back_motion = father_motion;
              }
              else
              {
                is_optimal = false;
                break;
              }

            }

            if(is_optimal)
            {
              // OMPL_INFORM("find more optimal path, appro: %lf", si_->distance(togo_lastmotion->state, goal_state));
              solution = togo_lastmotion;
              approxdif = solution->low_bound;
            }

            // std::vector<Motion *> motions;
            // nn_->list(motions);
            // OMPL_INFORM("DEBUG: after stage 2 nn_ size: %d, motions size: %d", nn_->size(), motions.size());
          }
          else
          {
            // clear the togo path
            size_t p_t = 0;
            while(p_t < togo_states.size())
            {
              si_->freeState(togo_states[p_t]);
              p_t++;
            }
            p_t = 0;
            while(p_t < togo_controls.size())
            {
              siC_->freeControl(togo_controls[p_t]);
              p_t++;
            }
            togo_distance.clear();
            togo_steps.clear();

            // std::vector<Motion *> motions;
            // nn_->list(motions);
            // OMPL_INFORM("DEBUG: after stage 3 nn_ size: %d, motions size: %d", nn_->size(), motions.size());
          }
        }

        // motion->c_total = lastmotion->c_total + si_->distance(motion->state, lastmotion->state);
        /* build parent-children relationship between lastmotion and currentmotion */
        lastmotion->children.push_back(motion);
        lastmotion = motion;
        nn_->add(motion);

        if(!goal_solve && motion->low_bound < approxdif)
        {
          /*we always have a path, but it doesn't mean we haved arrived the goal state.
          before we find a solution, we choose path according to the low_bound */
          // OMPL_INFORM("update solution before found");
          approxdif = motion->low_bound;
          solution = motion;
        }
      }

      // OMPL_INFORM("%d loop : approxdif : %lf; node_num: %d ", loop_times, approxdif, pstates.size());
    }
    /* no need to grow tree because control steps is too small */
    else  
    {
      for (auto &pstate :pstates)
        si_->freeState(pstate);
      for (auto &pcontrol :pcontrols)
        siC_->freeControl(pcontrol);
      cum_distance.clear();
      cum_steps.clear();
    }
    }
    else
    {
      // OMPL_INFORM("propagate failed");
      size_t p_d = 0;
      while(p_d < pstates.size())
      {
        si_->freeState(pstates[p_d]);
        p_d++;
      }
      p_d = 0;
      while(p_d < pcontrols.size())
      {
        siC_->freeControl(pcontrols[p_d]);
        p_d++;
      }
      cum_distance.clear();
      cum_steps.clear();
    }
  }

  OMPL_INFORM("%s time out: %u states already in datastructure", getName().c_str(), nn_->size());
  std::vector<Motion *> motions;
  nn_->list(motions);
  OMPL_INFORM("motions size: %d", motions.size());

  /*when time up, we update the best solution                                                                                                                                                        */

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
  solution_path = mpath;
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
  return ob::PlannerStatus(goal_solve, approximate);

}

bool CL_rrt::prune(Motion *new_root)
{
  if(new_root == current_root)
    return true;

  std::vector<Motion *> new_motion_tree;
  std::stack<Motion *> new_motion_box;
  new_motion_box.push(new_root);

  while (!new_motion_box.empty()) {
    Motion *current_addmotion = new_motion_box.top();
    new_motion_tree.push_back(current_addmotion);
    new_motion_box.pop();
    for(auto &new_addmotion :current_addmotion->children)
    {
       new_motion_box.push(new_addmotion);
    }
  }
  //freememory for delete nodes
  std::stack<Motion *> dmotions;
  //dmotions.reserve(nn_->size());
  dmotions.push(current_root);
  while(!dmotions.empty())
  {
    Motion *current_dmotion = dmotions.top();
    dmotions.pop();
    for(auto &new_dmotion :current_dmotion->children)
    {
      if(new_dmotion != new_root)
        dmotions.push(new_dmotion);
    }
    // delete the current_dmoiton
   // nn_->remove(current_dmotion);
    si_->freeState(current_dmotion->state);
    siC_->freeControl(current_dmotion->control);
    delete current_dmotion;
  }

  nn_->clear();
  nn_->add(new_motion_tree);

  new_root->parent = nullptr;
  new_root->steps = 0;

  current_root = new_root;
  return true;  // we can set a timer here to make sure that we prune in assumed duration
}

bool CL_rrt::prune_path(const ob::State *state_rt)
{
  // ob::State *goal_state = pdef_->getGoal().get()->as<ob::GoalState>()->getState();
  std::shared_ptr<ompl::NearestNeighbors<Motion *>> nn_t;
  // nn_t.reset(ompl::tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
  nn_t.reset(new ompl::NearestNeighborsLinear<Motion *>());
  nn_t->setDistanceFunction([this](const Motion *a, const Motion *b)
      {
         return distanceFunction2(a, b);
      });
  Motion *Solution = lastGoalMotion_;
  while(Solution != nullptr)
   {
     nn_t->add(Solution);
     Solution = Solution->parent;
   }
  auto *current_motion = new Motion(siC_);
  si_->copyState(current_motion->state, state_rt);
  siC_->nullControl(current_motion->control);
  Motion *new_rootmotion = nn_t->nearest(current_motion);
  waypoints.clear();
  // double temp_t = 1.0;
  double temp_t = siC_->getPropagationStepSize();

  /* connect current state to current path */
  if(si_->distance(current_motion->state, new_rootmotion->state) > path_resolution)
  {
    std::vector<ob::State *> cont_states;
    std::vector<oc::Control *> cont_controls;
    std::vector<double> cont_distance;
    std::vector<int> cont_steps;

    if(propagateuntilstop(current_motion->state, new_rootmotion->state, cont_states, cont_controls, cont_distance, cont_steps))
    {
      // OMPL_INFORM("connect propagate success");
      size_t k = 0;
      // OMPL_INFORM("size of path connecting current state to nearest last optimal trajectory point : %d ", togo_states.size());
      for(; k < cont_states.size(); ++k)
      {
        ob::CompoundStateSpace::StateType& hs = *cont_states[k]->as<ob::CompoundStateSpace::StateType>();
        double *angular= hs.as<ob::RealVectorStateSpace::StateType>(0)->values;
        double *position = hs.as<ob::RealVectorStateSpace::StateType>(1)->values;
        waypoints.push_back(WaypointWithTime(temp_t, position[0], position[1], position[2], angular[2]));
      }
      // cont_lastmotion->up_bound = si_->distance(cont_lastmotion->state, goal_state);
      // OMPL_INFORM("add %d waypoint to connect", cont_states.size());
      size_t p_t = 0;
      while(p_t < cont_states.size())
      {
        si_->freeState(cont_states[p_t]);
        p_t++;
      }
      p_t = 0;
      while(p_t < cont_controls.size())
      {
        siC_->freeControl(cont_controls[p_t]);
        p_t++;
      }
      cont_distance.clear();
      cont_steps.clear();
    }
    else
    {
      // OMPL_INFORM("connect failed!");
      return false;
    }
  }

  int index = solution_path.size()-1;
  for (int i = solution_path.size()-1; i >= 0; --i)
    if(solution_path[i] == new_rootmotion)
    {
      index = i;
      break;
    }
  OMPL_INFORM("find nearest path node, index: %d distance: %lf",solution_path.size()-1-index, si_->distance(new_rootmotion->state, current_motion->state));
  
  for (signed int i = index-1 >= 0? index-1 : 0 ; i >= 0 ; i--)
  {
    if(!si_->isValid(solution_path[i]->state))
    {
      OMPL_INFORM("path unvalid!");
      return false;
    }
    ob::CompoundStateSpace::StateType& hs = *solution_path[i]->state->as<ob::CompoundStateSpace::StateType>();
    double *angular= hs.as<ob::RealVectorStateSpace::StateType>(0)->values;
    double *position = hs.as<ob::RealVectorStateSpace::StateType>(1)->values;
    waypoints.push_back(WaypointWithTime(temp_t, position[0], position[1], position[2], angular[2]));
  }

  OMPL_INFORM("prune path complete");
  if(current_motion->state)
    si_->freeState(current_motion->state);
  return true;
}



ob::PlannerStatus CL_rrt::loop_solve(const ob::PlannerTerminationCondition &ptc, const ob::State *current_state)
{

  ob::Goal *goal = pdef_->getGoal().get();
  ob::State *goal_state = pdef_->getGoal().get()->as<ob::GoalState>()->getState();

  if(lastGoalMotion_) // not the first loop
  {
    Motion *Solution = lastGoalMotion_;
    std::shared_ptr<ompl::NearestNeighbors<Motion *>> nn_t;
    nn_t.reset(new ompl::NearestNeighborsLinear<Motion *>());
    nn_t->setDistanceFunction([this](const Motion *a, const Motion *b)
        {
          return distanceFunction2(a, b);
        });

    while(Solution != nullptr)
    {
      nn_t->add(Solution);
      Solution = Solution->parent;
    }

    OMPL_INFORM("%s: current total %u states", getName().c_str(), nn_->size());
    OMPL_INFORM("construct the nearestneighbor path with %u states", nn_t->size());

    /* find closest state in the solution path  */
    auto *current_motion = new Motion(siC_);
    si_->copyState(current_motion->state, current_state);
    siC_->nullControl(current_motion->control);
    Motion *new_rootmotion = nn_t->nearest(current_motion);
    OMPL_INFORM("find nearest node and distance: %lf", si_->distance(current_motion->state, new_rootmotion->state));
    int index = 0;
    for (int i = solution_path.size()-1; i >= 0; --i)
      if(solution_path[i] == new_rootmotion)
      {
        index = i;
        break;
      }
    OMPL_INFORM("index: %d offset: %d", index, int(floor(deltaT/siC_->getPropagationStepSize())) );
    if((!goal_solve && (index - int(floor(deltaT/siC_->getPropagationStepSize()))) <= 0) || si_->distance(new_rootmotion->state, current_state) >= std::numeric_limits<double>::infinity() )
    {
      OMPL_INFORM("previous info can be discarded totally");
      goal_solve = false;
      freeMemory();
      if(nn_)
        nn_->clear();

      lastGoalMotion_= nullptr;
      auto *total_new_motion = new Motion(siC_);
      si_->copyState(total_new_motion->state, current_state);
      siC_->nullControl(total_new_motion->control);
      total_new_motion->c_total = 0.0;
      total_new_motion->parent = nullptr;
      total_new_motion->steps = 0;
      goal->isSatisfied(total_new_motion->state, &total_new_motion->low_bound);
      current_root = total_new_motion;
      nn_->add(total_new_motion);
    }
    else
    {
      //new_rootmotion = solution_path[index - int(floor(deltaT/siC_->getPropagationStepSize()))];
      new_rootmotion = solution_path[index];
      OMPL_INFORM("find new root %d and get ready to prune", index);

      clock_t start = std::clock();
      prune(new_rootmotion);
      clock_t ends = std::clock();
      OMPL_INFORM("prune spend time: %lf", (double)(ends - start)/CLOCKS_PER_SEC);
      OMPL_INFORM("prune complete and current states number: %d", nn_->size());

      /* mark the commited portion */
      if(!(goal_solve && (index - int(floor(deltaT/siC_->getPropagationStepSize()))) <= 0))
      {
      Motion *commit_portion = solution_path[std::max(0, index- int(floor(deltaT/siC_->getPropagationStepSize())))];
      commit_portion->commit_protect = true;
      while(commit_portion != new_rootmotion)
      {
        commit_portion = commit_portion->parent;
        commit_portion->commit_protect = true;
      }
      }
      if(ptc == true)
        OMPL_INFORM("prune run out of time!");
      current_root = new_rootmotion;
    }
    si_->freeState(current_motion->state);
    nn_t->clear();
  }
  else{
      OMPL_INFORM("no previous solution, previous info can be discarded totally");
      goal_solve = false;
      freeMemory();
      if(nn_)
        nn_->clear();
      lastGoalMotion_= nullptr;
      auto *total_new_motion = new Motion(siC_);
      si_->copyState(total_new_motion->state, current_state);
      siC_->nullControl(total_new_motion->control);
      total_new_motion->c_total = 0.0;
      total_new_motion->parent = nullptr;
      total_new_motion->steps = 0;
      goal->isSatisfied(total_new_motion->state, &total_new_motion->low_bound);
      current_root = total_new_motion;
      nn_->add(total_new_motion);
  }

  /* below are almost the same as solve function */
  if(nn_->size() == 0)
  {
    OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
    return ob::PlannerStatus::INVALID_START;
  }

  if(!sampler_)
    sampler_ = si_->allocStateSampler();
  OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());

  Motion *solution = lastGoalMotion_;
  //Motion *approxsol = nullptr;
  double approxdif;
  if(solution == nullptr)
    approxdif = std::numeric_limits<double>::infinity();
  else
    approxdif = solution->low_bound;

  auto *rmotion = new Motion(siC_);
  ob::State *rstate = rmotion->state;
  oc::Control *rctrl = rmotion->control;
  ob::State *xstate = si_->allocState();

  int loop_times = 0;
  /* keep expanding thr tree until time up, enven when we hit the goal */
  while(ptc == false)
  {
    loop_times++;
    /* sample random state based on current state */
    // if(goal_s && rng_.uniform01() < goalBias_ && goal_s->canSample())
    //   goal_s->sampleGoal(rstate);
    // else
    //   sampler_->sampleUniform(rstate);
    sampler_->sampleUniform(rstate);

    /* find  closest state in the tree  */
    Motion *nmotion = nn_->nearest(rmotion);
    ob::State *dstate = rstate;
    /* we can choose R closest states either */
    // std::vector<Motion *> Nmotion ;
    // nn_->nearestR(rmotion,radius,Nmotion); 

    double d = si_->distance(nmotion->state, rstate);
    if(d > maxDistance_)
    {
      si_->getStateSpace()->interpolate(nmotion->state, rstate, maxDistance_/d, xstate);
      dstate = xstate;
    }

    /* get controller output using Controller function */
    while(!Controlfn_(nmotion->state, dstate, rctrl) || (!goal_solve && si_->distance(nmotion->state, dstate) < minDistance_))
    {
      // OMPL_DEBUG("invalid sample state");
      sampler_->sampleUniform(rstate);
      d = si_->distance(nmotion->state, rstate);
      if(d > maxDistance_)
      {
        si_->getStateSpace()->interpolate(nmotion->state, rstate, maxDistance_/d, xstate);
        dstate = xstate;
      }
    }

    /* here we set velocities and angular velocities to 0 */
    ob::CompoundStateSpace::StateType& s = *dstate->as<ob::CompoundStateSpace::StateType>();
    double *check_angular= s.as<ob::RealVectorStateSpace::StateType>(0)->values;
    double *check_position= s.as<ob::RealVectorStateSpace::StateType>(1)->values;
    double *check_velocity = s.as<ob::RealVectorStateSpace::StateType>(2)->values;
    double *check_angular_vel = s.as<ob::RealVectorStateSpace::StateType>(3)->values;
    check_angular[0] = 0.0; check_angular[1] = 0.0;
    //check_angular[2] = 0.0;
    check_velocity[0] = 0.0; check_velocity[1] = 0.0; check_velocity[2] = 0.0;
    check_angular_vel[0] = 0.0; check_angular_vel[1] = 0.0; check_angular_vel[2] = 0.0;
    // OMPL_INFORM("%d loop : sampled position and yaw: %lf %lf %lf %lf ", loop_times, check_position[0], check_position[1], check_position[2], check_angular[2]);

    si_->copyState(rmotion->state, dstate);
    std::vector<ob::State *> pstates;  // every expand loop, pstates point to the newly added tree nodes
    std::vector<oc::Control *> pcontrols;
    std::vector<double> cum_distance;
    std::vector<int> cum_steps;
    if(propagateuntilstop(nmotion->state, rmotion->state, pstates, pcontrols, cum_distance, cum_steps))
    {
      //OMPL_INFORM("propagate success");
      if(pstates.size() >= siC_->getMinControlDuration()){

      Motion *lastmotion = nmotion;
      bool solved = false;
      size_t p = 0;
      double dist = 0.0;
      for(; p < pstates.size(); ++p)
      {
        auto *motion = new Motion();
        motion->state = pstates[p];
        motion->control = pcontrols[p];
        motion->steps = cum_steps[p];
        motion->c_total = lastmotion->c_total + cum_distance[p];
        motion->parent = lastmotion;
        motion->low_bound = si_->distance(motion->state, goal_state);

        solved = goal->isSatisfied(motion->state, &dist);
        motion->low_bound = dist;
        if(solved)
        {
          // OMPL_INFORM("Found solution in stage 1, appro: %lf", dist);
          goal_solve = true;
          motion->up_bound = motion->low_bound;
         // back to root and update upper_bound
          Motion *back_motion = motion;
          Motion *father_motion = motion;
          bool is_optimal = true;
          while(back_motion->parent != nullptr)
          {
            father_motion = back_motion->parent;
            if(father_motion->up_bound > back_motion->up_bound + back_motion->c_total - father_motion->c_total)
            {
              father_motion->up_bound = back_motion->up_bound + back_motion->c_total - father_motion->c_total;
              back_motion = father_motion;
            }
            else
            {
              is_optimal = false;
              break;
            }
          }
          if(is_optimal)
          {
            OMPL_INFORM("update solution");
            solution = motion;
            approxdif = solution->low_bound;
          }
        }

        else
        {
          std::vector<ob::State *> togo_states;
          std::vector<oc::Control *> togo_controls;
          std::vector<double> togo_distance;
          std::vector<int> togo_steps;
          if(propagateuntilstop(motion->state, goal_state, togo_states, togo_controls, togo_distance, togo_steps))
          {
            // OMPL_INFORM("togo propagate success");
            // OMPL_INFORM("Found solution in stage 2, appro: %lf", si_->distance(togo_states[togo_states.size()-1], goal_state));
            goal_solve = true;
            // add the path to nn_
            size_t k = 0;
            Motion *togo_lastmotion = motion;
            // OMPL_INFORM("porpagate to goal steps: %d ", togo_states.size());
            for(; k < togo_states.size(); ++k)
            {
              auto *togo_motion = new Motion();
              togo_motion->state = togo_states[k];
              togo_motion->control = togo_controls[k];
              togo_motion->steps = togo_steps[k];
              togo_motion->c_total = togo_lastmotion->c_total + togo_distance[k];
              togo_motion->parent = togo_lastmotion;
              togo_motion->low_bound = si_->distance(togo_motion->state, goal_state);
              nn_->add(togo_motion);
              togo_lastmotion->children.push_back(togo_motion);
              togo_lastmotion = togo_motion;
            }
            togo_lastmotion->up_bound = si_->distance(togo_lastmotion->state, goal_state);

            // back to root and update upper_bound
            Motion *back_motion;
            Motion *father_motion;
            back_motion = togo_lastmotion;
            bool is_optimal = true;

            while(back_motion->parent != nullptr)
            {
              father_motion = back_motion->parent;
              if(father_motion->up_bound > back_motion->up_bound + back_motion->c_total - father_motion->c_total)
              {
                father_motion->up_bound = back_motion->up_bound + back_motion->c_total - father_motion->c_total;
                back_motion = father_motion;
              }
              else
              {
                is_optimal = false;
                break;
              }

            }

            if(is_optimal)
            {
              // OMPL_INFORM("find more optimal path, appro: %lf", si_->distance(togo_lastmotion->state, goal_state));
              solution = togo_lastmotion;
              approxdif = solution->low_bound;
            }
          }
          else
          {
            // clear the togo path
            size_t p_t = 0;
            while(p_t < togo_states.size())
            {
              si_->freeState(togo_states[p_t]);
              p_t++;
            }
            p_t = 0;
            while(p_t < togo_controls.size())
            {
              siC_->freeControl(togo_controls[p_t]);
              p_t++;
            }
            togo_distance.clear();
            togo_steps.clear();
          }
        }

        /* build parent-children relationship between lastmotion and currentmotion */
        lastmotion->children.push_back(motion);
        lastmotion = motion;
        nn_->add(motion);

        if(!goal_solve && motion->low_bound < approxdif)
        {
          //we always have a path, but it doesn't mean we haved arrived the goal state.
          //before we find a solution, we choose path according to the low_bound
          // OMPL_INFORM("update solution before found");
          approxdif = motion->low_bound;
          solution = motion;
        }
      }
      // OMPL_INFORM("%d loop : approxdif : %lf; node_num: %d ", loop_times, approxdif, pstates.size());
    }
    else /* no need to grow tree because control steps is too small */
    {
      for (auto &pstate :pstates)
        si_->freeState(pstate);
      for (auto &pcontrol :pcontrols)
        siC_->freeControl(pcontrol);
      cum_distance.clear();
      cum_steps.clear();
    }
    }
    else
    {
      // OMPL_INFORM("propagate failed");
      size_t p_d = 0;
      while(p_d < pstates.size())
      {
        si_->freeState(pstates[p_d]);
        p_d++;
      }
      p_d = 0;
      while(p_d < pcontrols.size())
      {
        siC_->freeControl(pcontrols[p_d]);
        p_d++;
      }
      cum_distance.clear();
      cum_steps.clear();
    }
  }

  OMPL_INFORM("%s time out: %u states already in datastructure", getName().c_str(), nn_->size());
  std::vector<Motion *> motions;
  nn_->list(motions);
  OMPL_INFORM("motions size: %d", motions.size());
  /*when time up, we update the best solution                                                                                                                                                        */

  bool approximate =false;
  if(!goal_solve)
  {
    OMPL_INFORM("solution not found!");
    approximate = true;
  }
  if(solution != nullptr){

  lastGoalMotion_ = solution;
  /* construct the solution path */
  std::vector<Motion *> mpath;
  OMPL_INFORM("construct the solution path");
  //int steps = 0;
  while(solution != nullptr)
  {
    mpath.push_back(solution);
    solution = solution->parent;
  }
  solution_path = mpath;
   OMPL_INFORM("construct solution path complete: %d", mpath.size());

  /* set the solution path */
  auto path(std::make_shared<oc::PathControl>(si_));
  for (int i = mpath.size()-1; i >= 0; --i)
    if(mpath[i]->parent)
      path->append(mpath[i]->state, mpath[i]->control, mpath[i]->steps*siC_->getPropagationStepSize());
  else
      path->append(mpath[i]->state);
  // OMPL_INFORM("complete set solution path");
  pdef_->clearSolutionPaths();
  // OMPL_INFORM("delete last solution");
  pdef_->addSolutionPath(path, approximate, approxdif, getName());

  }
  else
    return ob::PlannerStatus::CRASH;

  if(rmotion->state)
    si_->freeState(rmotion->state);
  if(rmotion->control)
    siC_->freeControl(rmotion->control);
  delete rmotion;
  si_->freeState(xstate);
  OMPL_INFORM("%s: Created %u states", getName().c_str(), nn_->size());
  return ob::PlannerStatus(goal_solve, approximate);
}

void CL_rrt::recordSolution()
{
    if (!si_ || !pdef_->hasSolution())
        return;
    waypoints.clear();
    og::PathGeometric &p = *(pdef_->getSolutionPath()->as<og::PathGeometric>());
   // std::static_pointer_cast<og::PathGeometric> p = pdf_->getSolutionPath();
    p.interpolate();
    double temp_t = 1.0;
    // double temp_t = 10*siC_->getPropagationStepSize();
    //const float DEG_2_RAD = M_PI / 180.0;
    for (std::size_t i = 0 ; i < p.getStateCount() ; ++i)
          {
              ob::CompoundStateSpace::StateType& hs = *p.getState(i)->as<ob::CompoundStateSpace::StateType>();
              double *angular= hs.as<ob::RealVectorStateSpace::StateType>(0)->values;
              double *position = hs.as<ob::RealVectorStateSpace::StateType>(1)->values;
              waypoints.push_back(WaypointWithTime(temp_t, position[0], position[1], position[2], angular[2]));
              //temp_t = temp_t +1;
          }

}

void CL_rrt::getPlannerData(ob::PlannerData &data) const
{
  Planner::getPlannerData(data);

  std::vector<Motion *> motions;
  if(nn_)
    nn_->list(motions);

  double delta = siC_->getPropagationStepSize();

  if(lastGoalMotion_)
    data.addGoalVertex(ob::PlannerDataVertex(lastGoalMotion_->state));

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
