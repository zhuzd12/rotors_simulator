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


#include <fcl/config.h>
#include <fcl/geometry/octree/octree.h>
#include <fcl/geometry/shape/box.h>
#include <fcl/broadphase/broadphase_collision_manager.h>
#include <fcl/narrowphase/collision.h>
#include <fcl/fcl.h>


#include <iostream>
#include <cstring>
#include <boost/filesystem.hpp>

#include "rotors_planner/cl_rrt.h"
#include <rotors_control/parameters_ros.h>
#include <rotors_control/lee_position_controller.h>

#include <Eigen/Dense>

#define pi 3.1415926

namespace oc = ompl::control;
namespace ob = ompl::base;
namespace og = ompl::geometric;

template <typename S>
void eulerToMatrix(S a, S b, S c, Eigen::Matrix<S, 3, 3>& R)
{
  auto c1 = std::cos(a);
  auto c2 = std::cos(b);
  auto c3 = std::cos(c);
  auto s1 = std::sin(a);
  auto s2 = std::sin(b);
  auto s3 = std::sin(c);

  R << c1 * c2, - c2 * s1, s2,
      c3 * s1 + c1 * s2 * s3, c1 * c3 - s1 * s2 * s3, - c2 * s3,
      s1 * s3 - c1 * c3 * s2, c3 * s1 * s2 + c1 * s3, c2 * c3;
}


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
//   Rx = Eigen::AngleAxisd(current_angular[0], Eigen::Vector3d::UnitZ())
//       * Eigen::AngleAxisd(current_angular[1], Eigen::Vector3d::UnitY())
//       * Eigen::AngleAxisd(current_angular[2], Eigen::Vector3d::UnitX());
   eulerToMatrix<double>(current_angular(0), current_angular(1), current_angular(2), Rx);
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

   delete rotor_velocities;

}


bool isStateValid(const oc::SpaceInformation *si, const octomap::OcTree *octree_, const ob::State *state)
{
  // extract the position and construct the UAV box
  const ob::CompoundStateSpace::StateType& s = *state->as<ob::CompoundStateSpace::StateType>();
  Eigen::Map<Eigen::Vector3d> current_angular(s.as<ob::RealVectorStateSpace::StateType>(0)->values);
  Eigen::Map<Eigen::Vector3d> current_position(s.as<ob::RealVectorStateSpace::StateType>(1)->values);
 // std::cout <<"current position X:" <<current_position(0) <<"Y: "<<current_position(1)<<"Z: "<<current_position(2)<<std::endl;
  //fcl::Box uav_box(0.3,0.3,0.3);
  //std::shared_ptr<fcl::Box<float>> uav_box(new fcl::Box<float>(0.1,0.1,0.1));
  std::shared_ptr<fcl::CollisionGeometry<double>> boxGeometry (new fcl::Box<double> (0.1, 0.1, 0.1));
//  fcl::BVHModel<fcl::OBBRSS<float>> uav_model;
//  fcl::BVHModel<fcl::OBBRSS<float>> *uav_model_ptr = &uav_model;
  fcl::Matrix3d R;
//  R = Eigen::AngleAxisd(current_angular[0], Eigen::Vector3d::UnitZ())
//      * Eigen::AngleAxisd(current_angular[1], Eigen::Vector3d::UnitY())
//      * Eigen::AngleAxisd(current_angular[2], Eigen::Vector3d::UnitX());
  eulerToMatrix<double>(current_angular(0), current_angular(1), current_angular(2), R);
  // Eigen::Map<Eigen::Matrix<float, 3, 1>> current_pos(current_position);
  // fcl::generateBVHModel(uav_model, uav_box, fcl::Transform3f(R, current_position));
  // fcl::Transform3f tf(R, current_position);
  fcl::Transform3d tf;
  tf.setIdentity();
  tf.linear() = R;
  tf.translation() = current_position;
  fcl::CollisionObjectd* uav_obj = new fcl::CollisionObjectd(boxGeometry, tf);

  fcl::OcTree<double>* tree = new fcl::OcTree<double>(std::shared_ptr<const octomap::OcTree>(octree_));
  std::shared_ptr<fcl::CollisionGeometry<double>> tree_ptr(tree);

  //fcl::Transform3f tf2(Eigen::Vector3f::Zeros(), Eigen::MatrixXf::Identity(3,3));
  fcl::CollisionObjectd* env_obj = new fcl::CollisionObjectd(tree_ptr, fcl::Transform3d::Identity());

  //fcl::BroadPhaseCollisionManager* manager1 = new fcl::DynamicAABBTreeCollisionManager(); //represent octomap
  //fcl::BroadPhaseCollisionManager* manager2 = new fcl::DynamicAABBTreeCollisionManager();

  fcl::CollisionRequest<double> request;
  fcl::CollisionResult<double> result;
  request.num_max_contacts = 5;

  fcl::collide(env_obj, uav_obj, request, result);

  return si->satisfiesBounds(state) && !result.isCollision();
  //return !result.isCollision();


}

bool simple_isStateValid(const ob::State *state, const octomap::OcTree *octomap_)
{
  const ob::CompoundStateSpace::StateType& s = *state->as<ob::CompoundStateSpace::StateType>();
  Eigen::Map<Eigen::Vector3d> current_position(s.as<ob::RealVectorStateSpace::StateType>(1)->values);
    const double x = current_position(0);
    const double y = current_position(1);
    const double z = current_position(2);
    octomap::OcTreeNode *state_node;
    try{
      state_node = octomap_->search(x, y, z);
    }
    catch(...)
    {
      std::cout<<"search failed"<<std::endl;
    }
    return  !octomap_->isNodeOccupied(state_node);
}

bool sim_running = false;

void callback(const sensor_msgs::ImuPtr& msg) {
  sim_running = true;
}




int main(int argc, char **argv)
{
  ros::init(argc, argv, "CL_RRT_test");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("/firefly/lee_position_controller_node");

  // The IMU is used, to determine if the simulator is running or not.
  ros::Subscriber sub = nh.subscribe("imu", 10, &callback);

  ros::Publisher octomap_pub = nh.advertise<octomap_msgs::Octomap>("/octo_test", 10);

  ROS_INFO("Wait for simulation to become ready...");

  while (!sim_running && ros::ok()) {
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }

  // Wait for 15s such that everything can settle and the mav flies to the initial position.
  ros::Duration(15).sleep();


  //rotors_control::VehicleParameters vehicle_parameters_;
  //rotors_control::GetVehicleParameters(nh, &vehicle_parameters_);
  //Eigen::Matrix4Xd* allocation_matrix;
  //rotors_control::calculateAllocationMatrix(vehicle_parameters_.rotor_configuration_, allocation_matrix);
  rotors_control::LeePositionController lee_position_controller_;

  // initialize the lee_controller
  rotors_control::GetRosParameter(pnh, "position_gain/x",
                  lee_position_controller_.controller_parameters_.position_gain_.x(),
                  &lee_position_controller_.controller_parameters_.position_gain_.x());
  rotors_control::GetRosParameter(pnh, "position_gain/y",
                  lee_position_controller_.controller_parameters_.position_gain_.y(),
                  &lee_position_controller_.controller_parameters_.position_gain_.y());
  rotors_control::GetRosParameter(pnh, "position_gain/z",
                  lee_position_controller_.controller_parameters_.position_gain_.z(),
                  &lee_position_controller_.controller_parameters_.position_gain_.z());
  rotors_control::GetRosParameter(pnh, "velocity_gain/x",
                  lee_position_controller_.controller_parameters_.velocity_gain_.x(),
                  &lee_position_controller_.controller_parameters_.velocity_gain_.x());
  rotors_control::GetRosParameter(pnh, "velocity_gain/y",
                  lee_position_controller_.controller_parameters_.velocity_gain_.y(),
                  &lee_position_controller_.controller_parameters_.velocity_gain_.y());
  rotors_control::GetRosParameter(pnh, "velocity_gain/z",
                  lee_position_controller_.controller_parameters_.velocity_gain_.z(),
                  &lee_position_controller_.controller_parameters_.velocity_gain_.z());
  rotors_control::GetRosParameter(pnh, "attitude_gain/x",
                  lee_position_controller_.controller_parameters_.attitude_gain_.x(),
                  &lee_position_controller_.controller_parameters_.attitude_gain_.x());
  rotors_control::GetRosParameter(pnh, "attitude_gain/y",
                  lee_position_controller_.controller_parameters_.attitude_gain_.y(),
                  &lee_position_controller_.controller_parameters_.attitude_gain_.y());
  rotors_control::GetRosParameter(pnh, "attitude_gain/z",
                  lee_position_controller_.controller_parameters_.attitude_gain_.z(),
                  &lee_position_controller_.controller_parameters_.attitude_gain_.z());
  rotors_control::GetRosParameter(pnh, "angular_rate_gain/x",
                  lee_position_controller_.controller_parameters_.angular_rate_gain_.x(),
                  &lee_position_controller_.controller_parameters_.angular_rate_gain_.x());
  rotors_control::GetRosParameter(pnh, "angular_rate_gain/y",
                  lee_position_controller_.controller_parameters_.angular_rate_gain_.y(),
                  &lee_position_controller_.controller_parameters_.angular_rate_gain_.y());
  rotors_control::GetRosParameter(pnh, "angular_rate_gain/z",
                  lee_position_controller_.controller_parameters_.angular_rate_gain_.z(),
                  &lee_position_controller_.controller_parameters_.angular_rate_gain_.z());
  rotors_control::GetVehicleParameters(pnh, &lee_position_controller_.vehicle_parameters_);
  lee_position_controller_.InitializeParameters();


  ob::StateSpacePtr SO3(new ob::RealVectorStateSpace(3));
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
  bounds.setLow(0, -15); bounds.setLow(1, -35); bounds.setLow(2, 0);
  bounds.setHigh(0, 35); bounds.setHigh(1, 15); bounds.setHigh(2, 50);
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
  ob::RealVectorBounds cbounds(4);
  cbounds.setLow(-0.3);
  cbounds.setHigh(0.3);
  cspace->as<oc::RealVectorControlSpace>()->setBounds(cbounds);

  // construct an instance of  space information from this control space
  oc::SpaceInformationPtr si(new oc::SpaceInformation(stateSpace, cspace));


  octomap::OcTree *octomap_;
  std::string octo_file ;

  if (argc == 2 ) {
    octo_file = argv[1];
    ROS_INFO("read octomap file: %s", argv[1]);
  }
  octomap_ = new octomap::OcTree(octo_file);
  double tree_metricx, tree_metricy, tree_metricz;
  octomap_->getMetricMax(tree_metricx, tree_metricy, tree_metricz);
    std::cout<< "octotree nodes: "<< octomap_->calcNumNodes()<<std::endl;
    std::cout<< "occupancy threshold: "<< octomap_->getOccupancyThres()<<std::endl;
    std::cout<< "octotree resolution: "<< octomap_->getResolution()<<std::endl;
    std::cout<< "octotree max metrix: "<< tree_metricx << " " << tree_metricy << " " << tree_metricz <<std::endl;
    octomap_->getMetricMin(tree_metricx, tree_metricy, tree_metricz);
    std::cout<< "octotree min metrix: "<< tree_metricx << " " << tree_metricy << " " << tree_metricz <<std::endl;
  ROS_INFO("octomap file read successfully");
  // set state validity checking for this space
  si->setStateValidityChecker(std::bind(&isStateValid, si.get(), octomap_, std::placeholders::_1));

  /* statevaliditychecker check */
  ob::State *check_state = si->allocState();
  ob::CompoundStateSpace::StateType& s = *check_state->as<ob::CompoundStateSpace::StateType>();
  double *check_angular= s.as<ob::RealVectorStateSpace::StateType>(0)->values;
  double *check_position = s.as<ob::RealVectorStateSpace::StateType>(1)->values;
  double *check_velocity = s.as<ob::RealVectorStateSpace::StateType>(2)->values;
  double *check_angular_vel = s.as<ob::RealVectorStateSpace::StateType>(3)->values;
  check_angular[0] = 0; check_angular[1] = 0; check_angular[2] = 0;
  check_velocity[0] = 0; check_velocity[1] = 0; check_velocity[2] = 0;
  check_angular_vel[0] = 0; check_angular_vel[1] = 0; check_angular_vel[2] = 0;
  double check_posion_x[5] = {0, 0, 0, 0, 0};
  double check_posion_y[5] = {-3.15, -3.2, -3.25, -3.3, -3.5};
  double check_posion_z[5] = {0, 1, 1, 1, 1};

  for(int i = 0; i < 5; ++i)
  {
  check_position[0] = check_posion_x[i];
  check_position[1] = check_posion_y[i];
  check_position[2] = check_posion_z[i];

  std::cout << "set state position: " << check_position[0] <<" "
            << check_position[1] <<" " <<check_position[2]<<std::endl;
  std::cout << "check state position: " << s.as<ob::RealVectorStateSpace::StateType>(1)->values[0] <<" "
            << s.as<ob::RealVectorStateSpace::StateType>(1)->values[1] <<" " <<s.as<ob::RealVectorStateSpace::StateType>(1)->values[2]<<std::endl;

  if(si->isValid(check_state))
    std::cout << "check result: valid state" <<std::endl;
  else
    std::cout << "check result: invalid state" <<std::endl;

  if(simple_isStateValid(check_state, octomap_))
    std::cout << "simple check result: valid state" <<std::endl;
  else
    std::cout << "simple check result: invalid state" <<std::endl;

  }


  // set the state propagation routine
  si->setStatePropagator(std::bind(&Quadrotorpropagate, lee_position_controller_.controller_parameters_.allocation_matrix_, lee_position_controller_.vehicle_parameters_, si, std::placeholders::_1,
          std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));

 /*
  ob::State *xstate = stateSpace->allocState();
  ob::CompoundStateSpace::StateType *s = xstate->as<ob::CompoundStateSpace::StateType>();
  ob::RealVectorStateSpace::StateType *Angular = s->as<ob::RealVectorStateSpace::StateType>(0);
  Angular->values[0] = 1;   */

  /*
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

   */
 // octomap::OcTree *octomap_ = new octomap::OcTree(octo_file);
  ros::spinOnce();
  octomap_msgs::Octomap octo_msg;
  octo_msg.header.frame_id = "/world";
  octomap_msgs::binaryMapToMsg(*octomap_,octo_msg);

  ros::Rate rate(10);
  while(ros::ok()){
    octomap_pub.publish(octo_msg);
    rate.sleep();
  }
  //octomap_->read(octo_file);
  ros::shutdown();

  return 0;
}

