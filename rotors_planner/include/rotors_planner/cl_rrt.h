#ifndef CL_RRT_H
#define CL_RRT_H

#include <ompl/control/planners/PlannerIncludes.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/base/goals/GoalState.h>
#include <functional>
#include <rotors_planner/common.h>

namespace rotors_planner {


namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace oc = ompl::control;


typedef std::function<bool(const ob::State *, const ob::State *, oc::Control *)> ControllerFn;

class Motion
{
public:
  Motion() = default;

  Motion(const oc::SpaceInformation *si)
    : state(si->allocState()),control(si->allocControl())
  {
  }

  ~Motion() = default;

  ob::State *state{nullptr};
  oc::Control *control{nullptr};
  unsigned int steps{0};
  bool commit_protect{false};
  double c_total{std::numeric_limits<double>::infinity()};
  double low_bound{std::numeric_limits<double>::infinity()};
  double up_bound{std::numeric_limits<double>::infinity()};
  Motion *parent{nullptr};
  std::vector<Motion *> children;
};

class CL_rrt : public ob::Planner
{
public:
  CL_rrt(const oc::SpaceInformationPtr &si);

  ~CL_rrt() override;

  void setup();

  void getPlannerData(ob::PlannerData &data) const override;

  ob::PlannerStatus solve(const ob::PlannerTerminationCondition &ptc) override;

  ob::PlannerStatus loop_solve(const ob::PlannerTerminationCondition &ptc, const ob::State *current_state);

  bool prune(Motion *new_root);

  bool prune_path(const ob::State *state_rt);

  std::vector<Motion *> get_solution_motions()
  {
    return solution_path;
  }

  void clear() override;

  void setGoalBias( double goalBias)
  {
    goalBias_ = goalBias;
  }

  double getGoalBias() const
  {
    return goalBias_;
  }

  void setRange(double distance) //need recover to 3d range
  {
    maxDistance_ = distance;
  }

  double getRange() const
  {
    return maxDistance_;
  }

  void setMinRange(double distance) //need recover to 3d range
  {
    minDistance_ = distance;
  }

  void setPathdeviation( double pathdeviation)
  {
    path_deviation = pathdeviation;
  }

  double getPathdeviation() const
  {
    return path_deviation;
  }

  void setPathresolution( double pathresoluton)
  {
    path_resolution = pathresoluton;
  }

  Motion * get_root_node()
  {
    return current_root;
  }
  template <template <typename T> class NN>
  void setNearestNeighbors()
  {
    if(nn_ && nn_->size() != 0)
      OMPL_WARN("Calling setNearestNeighbors will clear all states.");
    clear();
     nn_ = std::make_shared<NN<Motion *>>();
    setup();
  }

  void setController(const ControllerFn &svc)
  {
    Controlfn_ = svc;
  }

  bool propagateuntilstop(const ob::State *state, const ob::State *heading_state, std::vector<ob::State *> &result, std::vector<oc::Control *> &control_result, std::vector<double>& cum_distance, std::vector<int>& cum_steps) const;

  bool provideControl(const ob::State *state, const ob::State *heading_state,  oc::Control *control)
  {
    return Controlfn_(state, heading_state, control);
  }

  const ControllerFn& getController() const
  {
    return Controlfn_;
  }

  void recordSolution();

protected:
  void freeMemory();

  // a is the start point
  double distanceFunction(const Motion *a, const Motion *b) const
  {
    if(goal_solve)
      //return std::min(a->c_total, b->c_total);
      return std::min(a->c_total + si_->distance(a->state, b->state), b->c_total + si_->distance(a->state, b->state));
    else
      return si_->distance(a->state, b->state);
  }

  double distanceFunction2(const Motion *a, const Motion *b) const
  {
    return si_->distance(a->state, b->state);
  }

  ob::StateSamplerPtr sampler_;
  const oc::SpaceInformation *siC_;
  std::shared_ptr<ompl::NearestNeighbors<Motion *>> nn_;
  double goalBias_{.05};
  double maxDistance_{100.};
  double minDistance_{1.0};
  double deltaT{0.1};
  //double global_low_bound{std::numeric_limits<double>::infinity()};
  ompl::RNG rng_;
  Motion *lastGoalMotion_{nullptr};
  std::vector<Motion *> solution_path;
  Motion *current_root{nullptr};
  // double nearst_radius{50};
  bool goal_solve{false};
  double path_deviation{.1};
  double path_resolution{.25};
  ControllerFn Controlfn_;
public:
    std::vector<WaypointWithTime> waypoints;
};

#endif // CL_RRT_H

}
