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
  OMPL_INFORM("begin to freeMemory, size: %d", nn_->size());
  if(nn_)
  {
    std::vector<Motion *> motions;
    nn_->list(motions);
//    OMPL_INFORM("motions size: %d", motions.size());
//    int i = 0;
    for (auto &motion : motions)
    {
//      i++;
//      std::cout<<"i: "<<i<<std::endl;
//      if(motion == nullptr)
//        std::cout<<"null"<<std::endl;
      if(motion->state)
      {
        //OMPL_INFORM("test point 1");
        si_->freeState(motion->state);
       // OMPL_INFORM("test point 2");
      }
      if(motion->control)
        siC_->freeControl(motion->control);

      delete motion;
    }
  }
  OMPL_INFORM("freeMemory done");
}

bool CL_rrt::propagateuntilstop(const ob::State *state, const ob::State *heading_state, std::vector<ob::State *> &result, std::vector<oc::Control *> &control_result, std::vector<double>& cum_distance,  std::vector<int>& cum_steps) const
{
  //OMPL_INFORM("propagate begins");
  double dv = std::numeric_limits<double>::infinity();
  int st = 0;
  int st_loop = 0;
  int steps = 0;
  double distance_ = 0.0;

  oc::Control *rctrl = siC_->allocControl();
  ob::State *current_State = si_->allocState();
  ob::State *future_State = si_->allocState();
  si_->copyState(current_State, state);
  while(dv >= path_deviation){
    Controlfn_(current_State, heading_state, rctrl);
    //control_result.resize(st +1);
    //control_result[st] = siC_->allocControl();
    //siC_->copyControl(control_result[st], rctrl);
//    std::cout<<"control output: "<<rctrl->as<oc::RealVectorControlSpace::ControlType>()->values[0]<<" "
//            <<rctrl->as<oc::RealVectorControlSpace::ControlType>()->values[1]<<" "<<rctrl->as<oc::RealVectorControlSpace::ControlType>()->values[2]
//           <<" "<<rctrl->as<oc::RealVectorControlSpace::ControlType>()->values[3]<<std::endl;
    //result.resize(st + 1);
    //result[st] = si_->allocState();
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
      //siC_->freeControl(control_result[st]);
      //result.resize(st);
      //control_result.resize(st);
      // std::cout<<"invalid state "<<st<<std::endl;
      return false;
    }
    //std::cout<<"valid state "<<st<<std::endl;
    distance_ = distance_ + si_->distance(current_State, future_State);
    steps++;
    dv = si_->distance(future_State, heading_state);
//    std::cout<<"distance: "<<distance_<<std::endl;
//    std::cout<<"steps: "<<steps<<std::endl;
    if(distance_ >= path_resolution || dv < path_deviation)
    {
      st++;
      control_result.resize(st);
      result.resize(st);
      control_result[st-1] = siC_->allocControl();
      //result[st-1] = si_->allocState();
      siC_->copyControl(control_result[st-1], rctrl);
      result[st-1] = si_->allocState();
      si_->copyState(result[st-1], future_State);
      cum_distance.resize(st);
      cum_distance[st-1] = distance_;
      cum_steps.resize(st);
      cum_steps[st-1] = steps;

      distance_ = 0.0;
      steps = 0;

    }


    si_->copyState(current_State, future_State);
    //si_->copyState(current_State, result[st]);
    //dv = si_->distance(current_State, heading_state);
    //std::cout<<"distance: "<<dv<<std::endl;
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
  //std::cout<<std::endl;
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
  /* keep expanding thr tree until time up, enven when we hit the goal */
  while(ptc == false)
  {
    loop_times++;
    /* sample random state based on current state */
//    if(goal_s && rng_.uniform01() < goalBias_ && goal_s->canSample())
//      goal_s->sampleGoal(rstate);
//    else
//      sampler_->sampleUniform(rstate);
    sampler_->sampleUniform(rstate);

    /* find  closest state in the tree  */
    // we can choose R closest states either
    Motion *nmotion = nn_->nearest(rmotion);
    ob::State *dstate = rstate;
    /*std::vector<Motion *> Nmotion ;
    nn_->nearestR(rmotion,radius,Nmotion); */

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
    OMPL_INFORM("%d loop : sampled position and yaw: %lf %lf %lf %lf ", loop_times, check_position[0], check_position[1], check_position[2], check_angular[2]);

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
     // Motion *inter_motion = new Motion();
      /*
      int inter_steps = 0;
      double cum_distance[pstates.size()];
      cum_distance[p] = si_->distance(pstates[p], nmotion);
      for(; p < pstates.size(); ++p)
      {
        cum_distance[p] = cum_distance[p-1] + si_->distance(pstates[p], pstates[p-1]);
      }
      p = 0;
      double last_cum_distance = 0.0;  */
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


          lastmotion->children.push_back(motion);
          lastmotion = motion;
          nn_->add(motion);

//          std::vector<Motion *> motions;
//          nn_->list(motions);
//          OMPL_INFORM("after stage 1 nn_ size: %d, motions size: %d", nn_->size(), motions.size());
        }

        else
        {
          std::vector<ob::State *> togo_states;
          std::vector<oc::Control *> togo_controls;
          std::vector<double> togo_distance;
          std::vector<int> togo_steps;
          if(propagateuntilstop(motion->state, goal_state, togo_states, togo_controls, togo_distance, togo_steps))
          {
            //OMPL_INFORM("togo propagate success");
            OMPL_INFORM("Found solution in stage 2, appro: %lf", si_->distance(togo_states[togo_states.size()-1], goal_state));
            goal_solve = true;
            // add the path to nn_
            size_t k = 0;
            Motion *togo_lastmotion = motion;
            OMPL_INFORM("test size: %d ", togo_states.size());
            for(; k < togo_states.size(); ++k)
            {
             // OMPL_INFORM("test steps: %d", k);
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

//              std::vector<Motion *> motions;
//              nn_->list(motions);
//              OMPL_INFORM("testing stage 2 nn_ size: %d, motions size: %d", nn_->size(), motions.size());


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
              OMPL_INFORM("is optimal, appro: %lf", si_->distance(togo_lastmotion->state, goal_state));
              solution = togo_lastmotion;
              approxdif = solution->low_bound;
            }

            lastmotion->children.push_back(motion);
            lastmotion = motion;
            nn_->add(motion);

//            std::vector<Motion *> motions;
//            nn_->list(motions);
//            OMPL_INFORM("after stage 2 nn_ size: %d, motions size: %d", nn_->size(), motions.size());
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
            lastmotion->children.push_back(motion);
            lastmotion = motion;
            nn_->add(motion);

            std::vector<Motion *> motions;
            nn_->list(motions);
            OMPL_INFORM("after stage 3 nn_ size: %d, motions size: %d", nn_->size(), motions.size());
          }
        }

        //motion->c_total = lastmotion->c_total + si_->distance(motion->state, lastmotion->state);
//        lastmotion->children.push_back(motion);
//        lastmotion = motion;
//        nn_->add(motion);

        if(!goal_solve && motion->low_bound < approxdif)
        {
          //we always have a path, but it doesn't mean we haved arrived the goal state.
          //before we find a solution, we choose path according to the low_bound
          OMPL_INFORM("update solution before found");
          approxdif = motion->low_bound;
          solution = motion;
        }
        /*
        inter_steps++;
        if(cum_distance[p] >= path_resolution + last_cum_distance)
        {
          last_cum_distance = cum_distance[p];
          inter_motion->state = pstates[p];
          inter_motion->control = pcontrols[p];
          inter_motion->steps = inter_steps;
          inter_motion->parent = lastmotion;
          inter_motion->c_total = lastmotion->c_total + cum_distance[p];
          solved = goal->isSatisfied(inter_motion->state, &dist);
          inter_motion->low_bound = dist;
          if(solved)
          {
            inter_motion->up_bound = dist;
            //  back to root and update upper_bound
            ...
          }
          else
          {
            std::vector<ob::State *> togo_states;
            std::vector<oc::Control *> togo_controls;
            if(propagateuntilstop(inter_motion->state, goal_state, togo_states, togo_controls))
            {
              // add inter point to nn_

            }
          }

        } */


      }

      OMPL_INFORM("%d loop : approxdif : %lf; node_num: %d ", loop_times, approxdif, pstates.size());

      /*
      while((++p < pstates.size()))
      {
        si_->freeState(pstates[p]);
      }  */
    }
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
      OMPL_INFORM("propagate failed");
      size_t p_d = 0;
      while(p_d < pstates.size())
      {
        si_->freeState(pstates[p_d]);
        //std::cout<<"free state "<<p_d<<std::endl;
        p_d++;
      }
      p_d = 0;
      while(p_d < pcontrols.size())
      {
        siC_->freeControl(pcontrols[p_d]);
        //std::cout<<"free control "<<p_d<<std::endl;
        p_d++;
      }
      cum_distance.clear();
      cum_steps.clear();
      //std::cout<<"free done"<<std::endl;
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
  //new_motion_tree.reserve(nn_->size());
  // set the new root as current root node

  /*
  auto *root_motion = new Motion(siC_);
  root_motion->parent = nullptr;
  //root_motion->children = new_root->children;
  std::copy(new_root->children.begin(), new_root->children.end(), std::back_inserter(root_motion->children));
  siC_->nullControl(root_motion->control);
  root_motion->low_bound = new_root->low_bound;
  root_motion->steps = 0;
  si_->copyState(root_motion->state, new_root->state);
  for(auto &new_childmotion :new_root->children)
    new_childmotion->parent = root_motion;  */

//  new_root->parent = nullptr;
//  new_root->steps = 0;

  std::stack<Motion *> box;
  box.push(new_root);

  while (!box.empty()) {
    Motion *current_addmotion = box.top();

//    Motion *deep_copy = new Motion(siC_);
//    si_->copyState(deep_copy->state, current_addmotion->state);
//    siC_->copyControl(deep_copy->control, current_addmotion->control);
//    deep_copy->children = current_addmotion->children;
//    deep_copy->low_bound = current_addmotion->low_bound;
//    deep_copy->steps = current_addmotion->steps;
//    deep_copy->parent = current_addmotion->parent;

    new_motion_tree.push_back(current_addmotion);
    box.pop();
    for(auto &new_addmotion :current_addmotion->children)
    {
       box.push(new_addmotion);
    }
  }
  //freeMemory();
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
//      else
//      {
//        // delete the candidate root node
//        si_->freeState(new_dmotion->state);
//        siC_->freeControl(new_dmotion->control);
//        delete new_dmotion;
//      }
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
/*
  if(!new_root)
    return false;
  else
  {
    std::vector<Motion *> dmotions;
    dmotions.reserve(nn_->size());
    dmotions.push_back(current_root);
   // Motion *current_dmotion = current_root;
    while(!dmotions.empty())
    {
      Motion *current_dmotion = dmotions.back();
      dmotions.pop_back();
      // add current_dmotion children to the delete vector
//      for(auto iter = current_dmotion->children.begin(); iter!=current_dmotion->children.end(); ++iter)
//      {
//        if(current_dmotion->children.back() != new_root)
//        {
//          dmotions.push_back(current_dmotion->children.back());
//        }
//      }
      //OMPL_INFORM("%d children", current_dmotion->children.size());
      for(auto &new_dmotion :current_dmotion->children)
      {
        if(current_dmotion->children.back() != new_root)
          dmotions.push_back(new_dmotion);
      }
      // delete the current_dmoiton
      nn_->remove(current_dmotion);
      //OMPL_INFORM("delete the current_dmotion from neighbor tree");
      si_->freeState(current_dmotion->state);
      siC_->freeControl(current_dmotion->control);
    }

    // set the new root as current root node
    auto *root_motion = new Motion(siC_);
    root_motion->parent = nullptr;
    root_motion->children = new_root->children;
    siC_->nullControl(root_motion->control);
    root_motion->low_bound = new_root->low_bound;
    nn_->remove(new_root);
    OMPL_INFORM("test point 4");
    nn_->add(root_motion);
    OMPL_INFORM("test point 5");
    current_root = root_motion;
  } */
}

bool CL_rrt::prune_path(const ob::State *state_rt)
{
  //ob::State *goal_state = pdef_->getGoal().get()->as<ob::GoalState>()->getState();
  std::shared_ptr<ompl::NearestNeighbors<Motion *>> nn_t;
  //nn_t.reset(ompl::tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
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
  double temp_t = 1.0;

  /* connect current state to current path */
  if(si_->distance(current_motion->state, new_rootmotion->state) > path_resolution)
  {
    std::vector<ob::State *> cont_states;
    std::vector<oc::Control *> cont_controls;
    std::vector<double> cont_distance;
    std::vector<int> cont_steps;

    if(propagateuntilstop(current_motion->state, new_rootmotion->state, cont_states, cont_controls, cont_distance, cont_steps))
    {
      OMPL_INFORM("connect propagate success");
      size_t k = 0;
      //Motion *cont_lastmotion = current_motion;
      // OMPL_INFORM("test size: %d ", togo_states.size());
      for(; k < cont_states.size(); ++k)
      {
       // OMPL_INFORM("test steps: %d", k);
//        auto *cont_motion = new Motion();
//        cont_motion->state = cont_states[k];
//        cont_motion->control = cont_controls[k];
//        cont_motion->steps = cont_steps[k];
//        cont_motion->c_total = cont_lastmotion->c_total + cont_distance[k];
//        cont_motion->parent = cont_lastmotion;
//        cont_motion->low_bound = si_->distance(cont_motion->state, goal_state);
//        nn_->add(cont_motion);
//        cont_lastmotion->children.push_back(cont_motion);
//        cont_lastmotion = cont_motion;
        ob::CompoundStateSpace::StateType& hs = *cont_states[k]->as<ob::CompoundStateSpace::StateType>();
        double *angular= hs.as<ob::RealVectorStateSpace::StateType>(0)->values;
        double *position = hs.as<ob::RealVectorStateSpace::StateType>(1)->values;
        waypoints.push_back(WaypointWithTime(temp_t, position[0], position[1], position[2], angular[2]));
      }
      //cont_lastmotion->up_bound = si_->distance(cont_lastmotion->state, goal_state);
      OMPL_INFORM("add %d waypoint to connect", cont_states.size());
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
     OMPL_INFORM("connect failed!");
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
  //OMPL_INFORM("new root index: %d", index);
  // prune the path and update waypoints
  /*
  std::vector<Motion *> mpath;
  //OMPL_INFORM("construct the solution path");
  double approxdif = lastGoalMotion_->low_bound;
  lastGoalMotion_ = new_rootmotion;
  Solution = lastGoalMotion_;
  while(Solution != nullptr)
  {
    mpath.push_back(Solution);
    Solution = Solution->parent;
  }
  solution_path = mpath;
  // OMPL_INFORM("construct solution path complete: %d", mpath.size());

  auto path(std::make_shared<oc::PathControl>(si_));
  for (int i = mpath.size()-1; i >= 0; --i)
    if(mpath[i]->parent)
      path->append(mpath[i]->state,mpath[i]->control, mpath[i]->steps*siC_->getPropagationStepSize());
  else
      path->append(mpath[i]->state);
  //OMPL_INFORM("complete set solution path");
  pdef_->clearSolutionPaths();
  //OMPL_INFORM("delete last solution");
  pdef_->addSolutionPath(path, !goal_solve, approxdif, getName());
  */
 // waypoints.clear();

  //const float DEG_2_RAD = M_PI / 180.0;
  //std::cout<<"index: "<<index<<std::endl;
  for (signed int i = index-1>=0?index-1:0 ; i >= 0 ; i--)
  {
    //std::cout<<"i: "<<i<<std::endl;
    if(!si_->isValid(solution_path[i]->state))
    {
      OMPL_INFORM("path unvalid!");
      return false;
    }
    ob::CompoundStateSpace::StateType& hs = *solution_path[i]->state->as<ob::CompoundStateSpace::StateType>();
    double *angular= hs.as<ob::RealVectorStateSpace::StateType>(0)->values;
    double *position = hs.as<ob::RealVectorStateSpace::StateType>(1)->values;
    waypoints.push_back(WaypointWithTime(temp_t, position[0], position[1], position[2], angular[2]));
    //temp_t = temp_t +1;
//    std::cout << "Waypoint position: " <<  position[0] << " "
//              <<   position[1] <<  " " <<   position[2]  << std::endl;
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

  Motion *Solution = lastGoalMotion_;
  std::shared_ptr<ompl::NearestNeighbors<Motion *>> nn_t;
  //nn_t.reset(ompl::tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
  nn_t.reset(new ompl::NearestNeighborsLinear<Motion *>());
  nn_t->setDistanceFunction([this](const Motion *a, const Motion *b)
      {
         return distanceFunction2(a, b);
      });
//  int num = 0;
  while(Solution != nullptr)
   {
     nn_t->add(Solution);
     Solution = Solution->parent;
//     num++;
//     OMPL_INFORM("test point %d", num);
   }

  OMPL_INFORM("%s: current total %u states", getName().c_str(), nn_->size());
  OMPL_INFORM("construct the nearestneighbor path with %d states", nn_t->size());
  /* find  closest state in the solution path  */
  auto *current_motion = new Motion(siC_);
  si_->copyState(current_motion->state, current_state);
  //current_motion->state = current_state;
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
    //new_rootmotion = lastGoalMotion_;
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
//  else if(goal_solve && (index - int(floor(deltaT/siC_->getPropagationStepSize()))) <= 0)
//  {
//    OMPL_INFORM("no need to plan");
//    while(!ptc)
//    {
//       sleep(0.1);
//    }
//    return ob::PlannerStatus(goal_solve, false);
//  }
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
   // int pre_nodenum = nn_->size();
    if(ptc == true)
      OMPL_INFORM("prune run out of time!");
    current_root = new_rootmotion;
    //si_->freeState(current_motion->state);
    std::vector<Motion *> motions;
    nn_->list(motions);
    OMPL_INFORM("after prune motions size: %d", motions.size());

  }

  si_->freeState(current_motion->state);
  nn_t->clear();

  //below are almost the same as solve function

 // auto *goal_s = dynamic_cast <ob::GoalSampleableRegion *>(goal);


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
//    if(goal_s && rng_.uniform01() < goalBias_ && goal_s->canSample())
//      goal_s->sampleGoal(rstate);
//    else
//      sampler_->sampleUniform(rstate);
    sampler_->sampleUniform(rstate);

    /* find  closest state in the tree  */
    // we can choose R closest states either
    Motion *nmotion = nn_->nearest(rmotion);
    ob::State *dstate = rstate;
    /*std::vector<Motion *> Nmotion ;
    nn_->nearestR(rmotion,radius,Nmotion); */

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
    OMPL_INFORM("%d loop : sampled position and yaw: %lf %lf %lf %lf ", loop_times, check_position[0], check_position[1], check_position[2], check_angular[2]);

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
            OMPL_INFORM("update solution");
            solution = motion;
            approxdif = solution->low_bound;
          }


          lastmotion->children.push_back(motion);
          lastmotion = motion;
          nn_->add(motion);
        }

        else
        {
          std::vector<ob::State *> togo_states;
          std::vector<oc::Control *> togo_controls;
          std::vector<double> togo_distance;
          std::vector<int> togo_steps;
          if(propagateuntilstop(motion->state, goal_state, togo_states, togo_controls, togo_distance, togo_steps))
          {
            //OMPL_INFORM("togo propagate success");
            OMPL_INFORM("Found solution in stage 2, appro: %lf", si_->distance(togo_states[togo_states.size()-1], goal_state));
            goal_solve = true;
            // add the path to nn_
            size_t k = 0;
            Motion *togo_lastmotion = motion;
           // OMPL_INFORM("test size: %d ", togo_states.size());
            for(; k < togo_states.size(); ++k)
            {
             // OMPL_INFORM("test steps: %d", k);
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
              OMPL_INFORM("is optimal, appro: %lf", si_->distance(togo_lastmotion->state, goal_state));
              solution = togo_lastmotion;
              approxdif = solution->low_bound;
            }

            lastmotion->children.push_back(motion);
            lastmotion = motion;
            nn_->add(motion);
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

            lastmotion->children.push_back(motion);
            lastmotion = motion;
            nn_->add(motion);
          }
        }

        if(!goal_solve && motion->low_bound < approxdif)
        {
          //we always have a path, but it doesn't mean we haved arrived the goal state.
          //before we find a solution, we choose path according to the low_bound
          OMPL_INFORM("update solution before found");
          approxdif = motion->low_bound;
          solution = motion;
        }

      }

      OMPL_INFORM("%d loop : approxdif : %lf; node_num: %d ", loop_times, approxdif, pstates.size());
    }
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
      OMPL_INFORM("propagate failed");
      size_t p_d = 0;
      while(p_d < pstates.size())
      {
        si_->freeState(pstates[p_d]);
        //std::cout<<"free state "<<p_d<<std::endl;
        p_d++;
      }
      p_d = 0;
      while(p_d < pcontrols.size())
      {
        siC_->freeControl(pcontrols[p_d]);
        //std::cout<<"free control "<<p_d<<std::endl;
        p_d++;
      }
      cum_distance.clear();
      cum_steps.clear();
      //std::cout<<"free done"<<std::endl;
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
   // steps++;
    //std::cout<<"steps: "<<steps<<" diff: "<<solution->low_bound<<std::endl;
//    if(solution == new_rootmotion)
//    {
//       std::cout<<"arrive the root node!"<<std::endl;
//       if(solution->parent)
//            std::cout<<"bug!"<<std::endl;
//    }
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
  OMPL_INFORM("complete set solution path");
  pdef_->clearSolutionPaths();
  OMPL_INFORM("delete last solution");
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
