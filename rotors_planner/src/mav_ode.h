#include <ompl/config.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <boost/math/constants/constants.hpp>
#include <ompl/base/StateSpaceTypes.h>
#include <ompl/base/spaces/SE3StateSpace.h>

#include "rotors_control/parameters_ros.h"


namespace oc = ompl::control;
namespace ob = ompl::base;

inline void Quadrotorpropagate(const oc::SpaceInformation *si, const ob::State *start, const oc::Control *control, const double duration, ob::State *result)
{
    static double timestep = .01;
    int nsteps = ceil(duration / timestep);
    double dt = duration / nsteps;

    const double *U = control->as<oc::RealVectorControlSpace::ControlType>()->values;
    ob::CompoundStateSpace::StateType *s = result->as<ob::CompoundStateSpace::StateType>();
    ob::RealVectorStateSpace::StateType& Angular = s->as<ob::RealVectorStateSpace::StateType>(0);
    ob::RealVectorStateSpace::StateType& Position = s->as<ob::RealVectorStateSpace::StateType>(1);
    ob::RealVectorStateSpace::StateType& velocity = s->as<ob::RealVectorStateSpace::StateType>(2);
    ob::RealVectorStateSpace::StateType& Angular_vel = s->as<ob::RealVectorStateSpace::StateType>(3);

    const double rot = s->as<oc::RealVectorControlSpace::ControlType>()->values;
    si->getStateSpace()->copyState(result, start);
    for(int i=0; i<nsteps; i++)
    {
      Position.values[0] = Position.values[0] + velocity.values[0] * dt;
      Position.values[1] = Position.values[1] + velocity.values[1] * dt;
      Position.values[2] = Position.values[2] + velocity.values[2] * dt;

      Angular.values[0] = Angular.values[0] + Angular_vel.values[0] * dt;
      Angular.values[1] = Angular.values[1] + Angular_vel.values[1] * dt;
      Angular.values[2] = Angular.values[2] + Angular_vel.values[2] * dt;


    }


//    result->as<ob::SE3StateSpace::StateType>()->setXYZ(
//        pos[0] + ctrl[0] * duration * cos(rot),
//        pos[1] + ctrl[0] * duration * sin(rot));\,
//        pos[2] +
//    result->as<ob::SE3StateSpace::StateType>()->setYaw(
//        rot    + ctrl[1] * duration);


}

/*
void SimpleQuadrotorODE(const oc::ODESolver::StateType& q, const oc::Control* c, oc::ODESolver::StateType& qdot)
{
    // Retrieve control values.  Velocity is the first entry, steering angle is second.
    const double *u = c->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
    const double velocity = u[0];
    const double steeringAngle = u[1];
    // Retrieve the current orientation of the car.  The memory for ompl::base::SE2StateSpace is mapped as:
    // 0: x
    // 1: y
    // 2: theta
    const double theta = q[2];
    // Ensure qdot is the same size as q.  Zero out all values.
    qdot.resize(q.size(), 0);
    qdot[0] = velocity * cos(theta);            // x-dot
    qdot[1] = velocity * sin(theta);            // y-dot
    qdot[2] = velocity * tan(steeringAngle);    // theta-dot
}
*/
