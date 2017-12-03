#ifndef CL_RRT_H
#define CL_RRT_H

#include <ompl/control/planners/PlannerIncludes.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/base/goals/GoalState.h>
#include <functional>

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace oc = ompl::control;

typedef std::function<bool(const ob::State *, oc::Control *)> ControllerFn;

class CL_rrt : public ob::Planner
{
public:
  CL_rrt(const oc::SpaceInformationPtr &si);

  ~CL_rrt() override;

  void getPlannerData(ob::PlannerData &data) const override;

  ob::PlannerStatus solve(const ob::PlannerTerminationCondition &ptc) override;

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

  template <template <typename T> class NN>
  void setNearestNeighbors()
  {
    if(nn_ && nn_->size() != 0)
      OMPL_WARN("Calling setNearestNeighbors will clear all states.");
    clear();
     nn_ = std::make_shared<NN<Motion *>>();
    setup();
  }

protected:
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
    double low_bound{0.0};
    // double up_bound(0.0);
    Motion *parent{nullptr};
  };

  bool provideControl(const ob::State *state, oc::Control *control)
  {
    return Controlfn_(state,control);
  }

  void setController(const ControllerFn &svc)
  {
    Controlfn_ = svc;
  }

  const ControllerFn& getController() const
  {
    return Controlfn_;
  }

  bool propagateuntilstop(const ob::State *state, const oc::Control *control, const ob::State *heading_state, std::vector<ob::State *> &result) const;

  void freeMemory();

  double distanceFunction(const Motion *a, const Motion *b) const
  {
    return si_->distance(a->state, b->state);
  }

  ob::StateSamplerPtr sampler_;
  const oc::SpaceInformation *siC_;
  std::shared_ptr<ompl::NearestNeighbors<Motion *>> nn_;
  double goalBias_{.05};
  double maxDistance_{0.};
  //double global_low_bound{std::numeric_limits<double>::infinity()};
  ompl::RNG rng_;
  Motion *lastGoalMotion_{nullptr};
  ob::State *current_state{nullptr};
  // double nearst_radius{50};
  bool goal_solve{false};
  double path_deviation{.1};
protected:
  ControllerFn Controlfn_;

};

#endif // CL_RRT_H
