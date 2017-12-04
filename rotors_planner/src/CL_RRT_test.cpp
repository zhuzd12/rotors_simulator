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

#include "rotors_planner/cl_rrt.h"
#include <rotors_control/parameters_ros.h>
#include <rotors_control/lee_position_controller.h>


#define pi 3.1415926

namespace oc = ompl::control;
namespace ob = ompl::base;
namespace og = ompl::geometric;



void Quadrotorpropagate(const Eigen::Matrix4Xd allocation_matrix, rotors_control::VehicleParameters vehicle_parameters_, const oc::SpaceInformationPtr si, const ob::State *start, const oc::Control *control, const double duration, ob::State *result)
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


/* define the controller fucntion */
bool Lee_controller(rotors_control::LeePositionController lee_position_controller_ , const ob::State *state, const ob::State *heading_state, oc::Control *output)
{
   mav_msgs::EigenTrajectoryPoint trajectory_point;
   rotors_control::EigenOdometry odometry_;
   const ob::CompoundStateSpace::StateType& hs = *heading_state->as<ob::CompoundStateSpace::StateType>();
   const ob::CompoundStateSpace::StateType& s = *state->as<ob::CompoundStateSpace::StateType>();
   const Eigen::Map<Eigen::Vector3d> position(hs.as<ob::RealVectorStateSpace::StateType>(1)->values);
   const double yaw = hs.as<ob::RealVectorStateSpace::StateType>(0)->values[3];

   Eigen::Quaterniond orientation;
   Eigen::Matrix3d Rx;

   Eigen::Map<Eigen::Vector3d> current_angular(s.as<ob::RealVectorStateSpace::StateType>(0)->values);
   Eigen::Map<Eigen::Vector3d> current_position(s.as<ob::RealVectorStateSpace::StateType>(1)->values);
   Eigen::Map<Eigen::Vector3d> current_velocity(s.as<ob::RealVectorStateSpace::StateType>(2)->values);
   Eigen::Map<Eigen::Vector3d> current_angular_vel(s.as<ob::RealVectorStateSpace::StateType>(3)->values);
   Rx = Eigen::AngleAxisd(current_angular[0], Eigen::Vector3d::UnitZ())
       * Eigen::AngleAxisd(current_angular[1], Eigen::Vector3d::UnitY())
       * Eigen::AngleAxisd(current_angular[2], Eigen::Vector3d::UnitX());
   orientation = Rx;

   odometry_.position = current_position;
   odometry_.orientation = orientation;
   odometry_.velocity = current_velocity;
   odometry_.angular_velocity = current_angular_vel;
   Eigen::VectorXd *rotor_velocities = new(Eigen::VectorXd);

   trajectory_point.position_W = position;
   trajectory_point.setFromYaw(yaw);
   lee_position_controller_.SetOdometry(odometry_);
   lee_position_controller_.SetTrajectoryPoint(trajectory_point);
   lee_position_controller_.CalculateRotorVelocities(rotor_velocities);

   output->as<oc::RealVectorControlSpace::ControlType>()->values[0] = (*rotor_velocities)(0);
   output->as<oc::RealVectorControlSpace::ControlType>()->values[1] = (*rotor_velocities)(1);
   output->as<oc::RealVectorControlSpace::ControlType>()->values[2] = (*rotor_velocities)(2);
   output->as<oc::RealVectorControlSpace::ControlType>()->values[3] = (*rotor_velocities)(3);

}

bool isStateValid(const oc::SpaceInformation *si, const ob::State *state)
{
//    //    ob::ScopedState<ob::SE2StateSpace>
//    // cast the abstract state type to the type we expect
//    const ob::SE2StateSpace::StateType *se2state = state->as<ob::SE2StateSpace::StateType>();

//    // extract the first component of the state and cast it to what we expect
//    const ob::RealVectorStateSpace::StateType *pos = se2state->as<ob::RealVectorStateSpace::StateType>(0);

//    // extract the second component of the state and cast it to what we expect
//    const ob::SO2StateSpace::StateType *rot = se2state->as<ob::SO2StateSpace::StateType>(1);

//    // check validity of state defined by pos & rot


//    // return a value that is always true but uses the two variables we define, so we avoid compiler warnings
//    return si->satisfiesBounds(state) && (const void*)rot != (const void*)pos;
}





int main(int argc, char **argv)
{
  ros::init(argc, argv, "CL_RRT_test");
  ros::NodeHandle nh;

  //rotors_control::VehicleParameters vehicle_parameters_;
  //rotors_control::GetVehicleParameters(nh, &vehicle_parameters_);
  //Eigen::Matrix4Xd* allocation_matrix;
  //rotors_control::calculateAllocationMatrix(vehicle_parameters_.rotor_configuration_, allocation_matrix);
  rotors_control::LeePositionController lee_position_controller_;

  // initialize the lee_controller
  rotors_control::GetRosParameter(nh, "position_gain/x",
                  lee_position_controller_.controller_parameters_.position_gain_.x(),
                  &lee_position_controller_.controller_parameters_.position_gain_.x());
  rotors_control::GetRosParameter(nh, "position_gain/y",
                  lee_position_controller_.controller_parameters_.position_gain_.y(),
                  &lee_position_controller_.controller_parameters_.position_gain_.y());
  rotors_control::GetRosParameter(nh, "position_gain/z",
                  lee_position_controller_.controller_parameters_.position_gain_.z(),
                  &lee_position_controller_.controller_parameters_.position_gain_.z());
  rotors_control::GetRosParameter(nh, "velocity_gain/x",
                  lee_position_controller_.controller_parameters_.velocity_gain_.x(),
                  &lee_position_controller_.controller_parameters_.velocity_gain_.x());
  rotors_control::GetRosParameter(nh, "velocity_gain/y",
                  lee_position_controller_.controller_parameters_.velocity_gain_.y(),
                  &lee_position_controller_.controller_parameters_.velocity_gain_.y());
  rotors_control::GetRosParameter(nh, "velocity_gain/z",
                  lee_position_controller_.controller_parameters_.velocity_gain_.z(),
                  &lee_position_controller_.controller_parameters_.velocity_gain_.z());
  rotors_control::GetRosParameter(nh, "attitude_gain/x",
                  lee_position_controller_.controller_parameters_.attitude_gain_.x(),
                  &lee_position_controller_.controller_parameters_.attitude_gain_.x());
  rotors_control::GetRosParameter(nh, "attitude_gain/y",
                  lee_position_controller_.controller_parameters_.attitude_gain_.y(),
                  &lee_position_controller_.controller_parameters_.attitude_gain_.y());
  rotors_control::GetRosParameter(nh, "attitude_gain/z",
                  lee_position_controller_.controller_parameters_.attitude_gain_.z(),
                  &lee_position_controller_.controller_parameters_.attitude_gain_.z());
  rotors_control::GetRosParameter(nh, "angular_rate_gain/x",
                  lee_position_controller_.controller_parameters_.angular_rate_gain_.x(),
                  &lee_position_controller_.controller_parameters_.angular_rate_gain_.x());
  rotors_control::GetRosParameter(nh, "angular_rate_gain/y",
                  lee_position_controller_.controller_parameters_.angular_rate_gain_.y(),
                  &lee_position_controller_.controller_parameters_.angular_rate_gain_.y());
  rotors_control::GetRosParameter(nh, "angular_rate_gain/z",
                  lee_position_controller_.controller_parameters_.angular_rate_gain_.z(),
                  &lee_position_controller_.controller_parameters_.angular_rate_gain_.z());
  rotors_control::GetVehicleParameters(nh, &lee_position_controller_.vehicle_parameters_);
  lee_position_controller_.InitializeParameters();


  ob::StateSpacePtr SO3(new ob::RealVectorStateSpace());
  SO3->setName("Angular");
  ob::StateSpacePtr POS(new ob::RealVectorStateSpace(3));
  POS->setName("Position");
  ob::StateSpacePtr velocity(new ob::RealVectorStateSpace(3));
  velocity->setName("Velocity");
  ob::StateSpacePtr angular_vel(new ob::RealVectorStateSpace(3));
  angular_vel->setName("Angular_vel");
  //ob::StateSpacePtr stateSpace = SO3 + POS + velocity + angular_vel;

  std::shared_ptr<ob::CompoundStateSpace> stateSpace(new ob::CompoundStateSpace());
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



  oc::ControlSpacePtr cspace(new oc::RealVectorControlSpace(stateSpace, 4));
  // set the bounds for the control space
  ob::RealVectorBounds cbounds(2);
  cbounds.setLow(-0.3);
  cbounds.setHigh(0.3);
  cspace->as<oc::RealVectorControlSpace>()->setBounds(cbounds);

  // construct an instance of  space information from this control space
  oc::SpaceInformationPtr si(new oc::SpaceInformation(stateSpace, cspace));

  // set state validity checking for this space
  si->setStateValidityChecker(std::bind(&isStateValid, si.get(),  std::placeholders::_1));

  // set the state propagation routine
  si->setStatePropagator(std::bind(&Quadrotorpropagate, lee_position_controller_.controller_parameters_.allocation_matrix_, lee_position_controller_.vehicle_parameters_, si, std::placeholders::_1,
          std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));

 /*
  ob::State *xstate = stateSpace->allocState();
  ob::CompoundStateSpace::StateType *s = xstate->as<ob::CompoundStateSpace::StateType>();
  ob::RealVectorStateSpace::StateType *Angular = s->as<ob::RealVectorStateSpace::StateType>(0);
  Angular->values[0] = 1;   */

  // create a start state
  ob::ScopedState<ob::RealVectorStateSpace> start(stateSpace);

  // create a goal state
  ob::ScopedState<ob::CompoundStateSpace> goal(start);

  // create a problem instance
  ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));

  // set the start and goal states
  pdef->setStartAndGoalStates(start, goal, 0.1);

  // create a planner for the defined space
  ob::PlannerPtr planner(new rotors_planner::CL_rrt(si));

  // set the problem we are trying to solve for the planner
  planner->setProblemDefinition(pdef);

  // perform setup steps for the planner
  planner->setup();

  // print the settings for this space
  si->printSettings(std::cout);
  // print the problem settings
  pdef->print(std::cout);

  // attempt to solve the problem within one second of planning time
  ob::PlannerStatus solved = planner->solve(10.0);

  if (solved)
      {
          // get the goal representation from the problem definition (not the same as the goal state)
          // and inquire about the found path
          ob::PathPtr path = pdef->getSolutionPath();
          std::cout << "Found solution:" << std::endl;

          // print the path to screen
          path->print(std::cout);
      }
      else
          std::cout << "No solution found" << std::endl;

  ros::shutdown();

  return 0;
}

