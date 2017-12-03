#include <ros/ros.h>
#include <Eigen/Geometry>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <sensor_msgs/Imu.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

// octomap library header
#include <octomap/octomap_utils.h>
#include <octomap/OcTree.h>
#include <octomap/octomap.h>
#include <octomap/OcTreeNode.h>

#include <octomap_msgs/conversions.h>
#include <rotors_comm/Octomap.h>


// ompl library header
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/config.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <boost/math/constants/constants.hpp>
#include <ompl/base/StateSpaceTypes.h>



//#include <fcl/common/detail/profiler.h>
#include <iostream>
#include <cstring>
#include <boost/filesystem.hpp>

#include "cl_rrt.h"
//#include "mav_ode.h"
//#include <rotors_control/common.h>
//#include <rotors_control/parameters.h>
#include <rotors_control/parameters_ros.h>
#include <rotors_control/lee_position_controller.h>

#define pi 3.1415926

namespace oc = ompl::control;
namespace ob = ompl::base;
namespace og = ompl::geometric;



void Quadrotorpropagate(Eigen::Matrix4Xd allocation_matrix, rotors_control::VehicleParameters vehicle_parameters_, const oc::SpaceInformation *si, const ob::State *start, const oc::Control *control, const double duration, ob::State *result)
{
    static double timestep = .01;
    int nsteps = ceil(duration / timestep);
    double dt = duration / nsteps;

    double *input_rotors_control = control->as<oc::RealVectorControlSpace::ControlType>()->values;
    Eigen::Map<Eigen::Vector4d> U(input_rotors_control);
    U = allocation_matrix * U;
    ob::CompoundStateSpace::StateType& s = *result->as<ob::CompoundStateSpace::StateType>();
    ob::RealVectorStateSpace::StateType& Angular = *s.as<ob::RealVectorStateSpace::StateType>(0);
    ob::RealVectorStateSpace::StateType& Position = *s.as<ob::RealVectorStateSpace::StateType>(1);
    ob::RealVectorStateSpace::StateType& velocity = *s.as<ob::RealVectorStateSpace::StateType>(2);
    ob::RealVectorStateSpace::StateType& Angular_vel = *s.as<ob::RealVectorStateSpace::StateType>(3);


    si->getStateSpace()->copyState(result, start);
    for(int i=0; i<nsteps; i++)
    {
      Position.values[0] = Position.values[0] + velocity.values[0] * dt;
      Position.values[1] = Position.values[1] + velocity.values[1] * dt;
      Position.values[2] = Position.values[2] + velocity.values[2] * dt;

      Angular.values[0] = Angular.values[0] + Angular_vel.values[0] * dt;
      Angular.values[1] = Angular.values[1] + Angular_vel.values[1] * dt;
      Angular.values[2] = Angular.values[2] + Angular_vel.values[2] * dt;

      velocity.values[0] = velocity.values[0] + dt * U[3] * (cos(Angular.values[0]) * cos(Angular.values[2]) * sin(Angular.values[1])
          + sin(Angular.values[0]) * sin(Angular.values[2])) / vehicle_parameters_.mass_;

      velocity.values[1] = velocity.values[1] + dt * U[3] * (cos(Angular.values[0]) * cos(Angular.values[2]) * sin(Angular.values[1])
          - sin(Angular.values[0]) * sin(Angular.values[2])) / vehicle_parameters_.mass_;

      velocity.values[2] = velocity.values[2] + dt * (U[3] * cos(Angular.values[0]) * cos(Angular.values[1])/vehicle_parameters_.mass_ - vehicle_parameters_.gravity_);

      //Eigen::Vector3d angular_acc = vehicle_parameters_.inertia_.
      Angular_vel[0] = Angular_vel[0] + dt * (Angular_vel[1] * Angular_vel[2] * (vehicle_parameters_.inertia_(1,1) - vehicle_parameters_.inertia_(2,2))/vehicle_parameters_.inertia_(0,0)
          + U[0]/vehicle_parameters_.inertia_(0,0));

      Angular_vel[1] = Angular_vel[1] + dt * (Angular_vel[0] * Angular_vel[2] * (vehicle_parameters_.inertia_(2,2) - vehicle_parameters_.inertia_(0,0))/vehicle_parameters_.inertia_(1,1)
          + U[1]/vehicle_parameters_.inertia_(1,1));

      Angular_vel[2] = Angular_vel[2] + dt * (Angular_vel[0] * Angular_vel[1] * (vehicle_parameters_.inertia_(0,0) - vehicle_parameters_.inertia_(1,1))/vehicle_parameters_.inertia_(2,2)
          + U[2]/vehicle_parameters_.inertia_(2,2));

      if(!si->satisfiesBounds(result))
        return ;

    }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "CL_RRT_test");
  ros::NodeHandle nh;

  rotors_control::VehicleParameters vehicle_parameters_;
  rotors_control::GetVehicleParameters(nh, &vehicle_parameters_);
  Eigen::Matrix4Xd* allocation_matrix;
  rotors_control::calculateAllocationMatrix(vehicle_parameters_.rotor_configuration_, allocation_matrix);

  ob::StateSpacePtr SO3(new ob::RealVectorStateSpace());
  SO3->setName("Angular");
  ob::StateSpacePtr POS(new ob::RealVectorStateSpace(3));
  POS->setName("Position");
  ob::StateSpacePtr velocity(new ob::RealVectorStateSpace(3));
  velocity->setName("Velocity");
  ob::StateSpacePtr angular_vel(new ob::RealVectorStateSpace(3));
  angular_vel->setName("Angular_vel");
  //ob::StateSpacePtr stateSpace = SO3 + POS + velocity + angular_vel;

  ob::CompoundStateSpace * stateSpace = new ob::CompoundStateSpace();
  stateSpace->addSubspace(SO3, 0.0);
  stateSpace->addSubspace(POS, 1.0);
  stateSpace->addSubspace(velocity, 0.0);
  stateSpace->addSubspace(angular_vel, 0.0);


  // set the bounds for the position space
  ob::RealVectorBounds bounds(3);
  bounds.setLow(0);
  bounds.setHigh(2*pi);
  SO3->as<ob::RealVectorStateSpace>()->setBounds(bounds);

  //ob::RealVectorBounds bounds(3);
  bounds.setLow(-50);
  bounds.setHigh(50);
  POS->as<ob::RealVectorStateSpace>()->setBounds(bounds);

  //ob::RealVectorBounds bounds(3);
  bounds.setLow(-5);
  bounds.setHigh(5);
  velocity->as<ob::RealVectorStateSpace>()->setBounds(bounds);

 // ob::RealVectorBounds bounds(3);
  bounds.setLow(-5);
  bounds.setHigh(5);
  angular_vel->as<ob::RealVectorStateSpace>()->setBounds(bounds);

  ob::ScopedState<ob::RealVectorStateSpace> start(POS);
  ob::ScopedState<ob::CompoundStateSpace> goal(start);

  oc::ControlSpacePtr cspace(new oc::RealVectorControlSpace(*stateSpace, 4));

  // set the bounds for the control space
  ob::RealVectorBounds cbounds(2);
  cbounds.setLow(-0.3);
  cbounds.setHigh(0.3);

  cspace->as<oc::RealVectorControlSpace>()->setBounds(cbounds);
  oc::SpaceInformationPtr si(new oc::SpaceInformation(stateSpace, cspace));

  si->setStatePropagator(std::bind(&Quadrotorpropagate, allocation_matrix, vehicle_parameters_, si, std::placeholders::_1,
          std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));

 /*
  ob::State *xstate = stateSpace->allocState();
  ob::CompoundStateSpace::StateType *s = xstate->as<ob::CompoundStateSpace::StateType>();
  ob::RealVectorStateSpace::StateType *Angular = s->as<ob::RealVectorStateSpace::StateType>(0);
  Angular->values[0] = 1;   */







  ROS_INFO("Hello world!");
}
