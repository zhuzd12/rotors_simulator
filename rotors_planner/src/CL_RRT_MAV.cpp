#include <ros/ros.h>
#include <Eigen/Geometry>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <sensor_msgs/Imu.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <visualization_msgs/Marker.h>

// octomap library header
#include <octomap/octomap_utils.h>
#include <octomap/OcTree.h>
#include <octomap/octomap.h>
#include <octomap/OcTreeNode.h>

#include <octomap_msgs/conversions.h>
#include <rotors_comm/Octomap.h>


// ompl library header
#include <ompl/base/SpaceInformation.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/config.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/ODESolver.h>
#include <boost/math/constants/constants.hpp>
#include <ompl/base/StateSpaceTypes.h>


#include <fcl/config.h>
#include <fcl/geometry/octree/octree.h>
#include <fcl/geometry/shape/box.h>
#include <fcl/broadphase/broadphase_collision_manager.h>
#include <fcl/narrowphase/collision.h>
#include <fcl/fcl.h>


#include <iostream>
#include <string>
#include <boost/filesystem.hpp>

#include "rotors_planner/cl_rrt.h"
#include <rotors_control/parameters_ros.h>
#include <rotors_control/lee_position_controller.h>

#include "rotors_planner/common.h"

#include <Eigen/Dense>
//#define pi 3.14159265359

namespace oc = ompl::control;
namespace ob = ompl::base;
namespace og = ompl::geometric;

double angluar_normalization(double angular)
{
  double temp = std::fmod(angular, 2.0 * M_PI);
  if(temp < -M_PI)
    temp += 2.0 * M_PI;
  else if(temp >= M_PI)
    temp -= 2 * M_PI;

  return temp;
}

/* define Hexacopter dynamic model */
void Hexacopterpropagate(const Eigen::Matrix4Xd allocation_matrix, rotors_control::VehicleParameters vehicle_parameters_, const oc::SpaceInformationPtr si, const ob::State *start, const oc::Control *control, const double duration, ob::State *result)
{
    static double timestep = .01;
    int nsteps = ceil(duration / timestep);
    double dt = duration / nsteps;

    double *input_rotors_control = control->as<oc::RealVectorControlSpace::ControlType>()->values;
    Eigen::VectorXd rotors_(vehicle_parameters_.rotor_configuration_.rotors.size());
    for (int k=0; k<vehicle_parameters_.rotor_configuration_.rotors.size(); ++k)
    {
      rotors_(k) = input_rotors_control[k] * input_rotors_control[k];
    }
    Eigen::Vector4d U;
    U = allocation_matrix * rotors_;
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

      velocity.values[0] = velocity.values[0] + dt * U[3] * (cos(Angular.values[0]) * cos(Angular.values[2]) * sin(Angular.values[1])
          + sin(Angular.values[0]) * sin(Angular.values[2])) / vehicle_parameters_.mass_;

      velocity.values[1] = velocity.values[1] + dt * U[3] * (cos(Angular.values[0]) * sin(Angular.values[2]) * sin(Angular.values[1])
          - sin(Angular.values[0]) * cos(Angular.values[2])) / vehicle_parameters_.mass_;

      velocity.values[2] = velocity.values[2] + dt * (U[3] * cos(Angular.values[0]) * cos(Angular.values[1])/vehicle_parameters_.mass_ - vehicle_parameters_.gravity_);

      Eigen::Matrix3d T_eb;
      T_eb<< 1.0, sin(Angular.values[0])*tan(Angular.values[1]), cos(Angular.values[0])*tan(Angular.values[1]),
          0.0, cos(Angular.values[0]), -sin(Angular.values[0]),
          0.0,  sin(Angular.values[0])/cos(Angular.values[1]), cos(Angular.values[0])/cos(Angular.values[1]);
      Eigen::Map<Eigen::Vector3d> angular_vel_vector(Angular_vel.values);
     
      Eigen::Vector3d T_eb_pqr = T_eb * angular_vel_vector;

      Angular.values[0] = angluar_normalization(Angular.values[0] + T_eb_pqr(0) * dt);
      Angular.values[1] = angluar_normalization(Angular.values[1] + T_eb_pqr(1) * dt);
      Angular.values[2] = angluar_normalization(Angular.values[2] + T_eb_pqr(2) * dt);


      double temp1 = Angular_vel[1] * Angular_vel[2] * (vehicle_parameters_.inertia_(1,1) - vehicle_parameters_.inertia_(2,2))/vehicle_parameters_.inertia_(0,0)
          + U[0]/vehicle_parameters_.inertia_(0,0);
      double temp2 = Angular_vel[0] * Angular_vel[2] * (vehicle_parameters_.inertia_(2,2) - vehicle_parameters_.inertia_(0,0))/vehicle_parameters_.inertia_(1,1)
          + U[1]/vehicle_parameters_.inertia_(1,1);
      double temp3 = Angular_vel[0] * Angular_vel[1] * (vehicle_parameters_.inertia_(0,0) - vehicle_parameters_.inertia_(1,1))/vehicle_parameters_.inertia_(2,2)
          + U[2]/vehicle_parameters_.inertia_(2,2);

      Angular_vel[0] = Angular_vel[0] + dt * temp1;

      Angular_vel[1] = Angular_vel[1] + dt * temp2;

      Angular_vel[2] = Angular_vel[2] + dt * temp3;

      if(!si->satisfiesBounds(result))
      {
        return;
      }

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
    const double yaw = hs.as<ob::RealVectorStateSpace::StateType>(0)->values[2];

    Eigen::Quaterniond orientation;
    Eigen::Matrix3d Rx;

    Eigen::Map<Eigen::Vector3d> current_angular(s.as<ob::RealVectorStateSpace::StateType>(0)->values);
    Eigen::Map<Eigen::Vector3d> current_position(s.as<ob::RealVectorStateSpace::StateType>(1)->values);
    Eigen::Map<Eigen::Vector3d> current_velocity(s.as<ob::RealVectorStateSpace::StateType>(2)->values);
    Eigen::Map<Eigen::Vector3d> current_angular_vel(s.as<ob::RealVectorStateSpace::StateType>(3)->values);
    Rx = Eigen::AngleAxisd(current_angular[2], Eigen::Vector3d::UnitZ())
        * Eigen::AngleAxisd(current_angular[1], Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(current_angular[0], Eigen::Vector3d::UnitX());
    orientation = Rx;

    odometry_.position = current_position;
    odometry_.orientation = orientation;
    odometry_.velocity = Rx.inverse()*current_velocity;
    odometry_.angular_velocity = current_angular_vel;
    Eigen::VectorXd *rotor_velocities = new(Eigen::VectorXd);

    trajectory_point.position_W = position;
    trajectory_point.setFromYaw(yaw);
    //trajectory_point.time_from_start_ns = 0;
    lee_position_controller_.SetOdometry(odometry_);
    lee_position_controller_.SetTrajectoryPoint(trajectory_point);
    lee_position_controller_.CalculateRotorVelocities(rotor_velocities);

    for(int i = 0; i < rotor_velocities->size(); i++)
    {
        output->as<oc::RealVectorControlSpace::ControlType>()->values[i] = (*rotor_velocities)(i);
    }
    delete rotor_velocities;
    return true;

}


bool isStateValid(const oc::SpaceInformation *si, const octomap::OcTree *octree_, const ob::State *state)
{
  // extract the position and construct the MAV box
  const ob::CompoundStateSpace::StateType& s = *state->as<ob::CompoundStateSpace::StateType>();
  Eigen::Map<Eigen::Vector3d> current_angular(s.as<ob::RealVectorStateSpace::StateType>(0)->values);
  Eigen::Map<Eigen::Vector3d> current_position(s.as<ob::RealVectorStateSpace::StateType>(1)->values);
  std::shared_ptr<fcl::CollisionGeometry<double>> boxGeometry (new fcl::Box<double> (0.5, 0.5, 0.3));
  fcl::Matrix3d R;
  R = Eigen::AngleAxisd(current_angular[2], Eigen::Vector3d::UnitZ())
      * Eigen::AngleAxisd(current_angular[1], Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(current_angular[0], Eigen::Vector3d::UnitX());
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
rotors_control::EigenOdometry current_odometry;



void callback(const sensor_msgs::ImuPtr& msg) {
  sim_running = true;
}

void OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg) {
  //std::cout<<"odometry!"<<std::endl;
  rotors_control::eigenOdometryFromMsg(odometry_msg, &current_odometry);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "CL_RRT_MAV");
  ros::NodeHandle nh, pnh("~");
  std::string mav_name_;
  bool ifget_ = ros::param::get("mav_name", mav_name_);

  // The IMU is used, to determine if the simulator is running or not.
  ros::Subscriber sub = nh.subscribe("imu", 10, &callback);

  ros::Subscriber odometry_sub_ = nh.subscribe("ground_truth/odometry", 1,
                               OdometryCallback);

  ros::Publisher trajectory_pub =
  nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
  mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);

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
  // nh.getParam("/"+mav_name_+"/position_gain/x", lee_position_controller_.controller_parameters_.position_gain_.x());
  // nh.getParam("/"+mav_name_+"/position_gain/y", lee_position_controller_.controller_parameters_.position_gain_.y());
  // nh.getParam("/"+mav_name_+"/position_gain/z", lee_position_controller_.controller_parameters_.position_gain_.z());
  // nh.getParam("/"+mav_name_+"/velocity_gain/x", lee_position_controller_.controller_parameters_.velocity_gain_.x());
  // nh.getParam("/"+mav_name_+"/velocity_gain/y", lee_position_controller_.controller_parameters_.velocity_gain_.y());
  // nh.getParam("/"+mav_name_+"/velocity_gain/z", lee_position_controller_.controller_parameters_.velocity_gain_.z());
  // nh.getParam("/"+mav_name_+"/attitude_gain/x", lee_position_controller_.controller_parameters_.attitude_gain_.x());
  // nh.getParam("/"+mav_name_+"/attitude_gain/y", lee_position_controller_.controller_parameters_.attitude_gain_.y());
  // nh.getParam("/"+mav_name_+"/attitude_gain/z", lee_position_controller_.controller_parameters_.attitude_gain_.z());
  // nh.getParam("/"+mav_name_+"/angular_rate_gain/x", lee_position_controller_.controller_parameters_.angular_rate_gain_.x());
  // nh.getParam("/"+mav_name_+"/angular_rate_gain/y", lee_position_controller_.controller_parameters_.angular_rate_gain_.y());
  // nh.getParam("/"+mav_name_+"/angular_rate_gain/z", lee_position_controller_.controller_parameters_.angular_rate_gain_.z());
  // rotors_control::GetVehicleParameters(nh, &lee_position_controller_.vehicle_parameters_);
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
  bounds.setLow(-M_PI);
  bounds.setHigh(M_PI);
  SO3->as<ob::RealVectorStateSpace>()->setBounds(bounds);

  bounds.setLow(0, -15); bounds.setLow(1, -35); bounds.setLow(2, 0);
  bounds.setHigh(0, 35); bounds.setHigh(1, 15); bounds.setHigh(2, 50);
  POS->as<ob::RealVectorStateSpace>()->setBounds(bounds);

  bounds.setLow(-100);
  bounds.setHigh(100);
  velocity->as<ob::RealVectorStateSpace>()->setBounds(bounds);

  bounds.setLow(-100);
  bounds.setHigh(100);
  angular_vel->as<ob::RealVectorStateSpace>()->setBounds(bounds);
  
  size_t rotors_size = lee_position_controller_.vehicle_parameters_.rotor_configuration_.rotors.size();
  ROS_INFO("rotor size: %d", rotors_size);
  oc::ControlSpacePtr cspace(new oc::RealVectorControlSpace(stateSpace, rotors_size));
  // set the bounds for the control space
  ob::RealVectorBounds cbounds(rotors_size);
  cbounds.setLow(0.0);
  cbounds.setHigh(1600);
  cspace->as<oc::RealVectorControlSpace>()->setBounds(cbounds);

  // construct an instance of  space information from this control space
  oc::SpaceInformationPtr si(new oc::SpaceInformation(stateSpace, cspace));

  octomap::OcTree *octomap_;
  std::string octo_file ;

  // if (argc == 2 ) {
  //   octo_file = argv[1];
  //   ROS_INFO("read octomap file: %s", argv[1]);
  // }
  if (!pnh.getParam("map_path", octo_file)) {
    ROS_ERROR("map is not loaded from ros parameter server");
    abort();
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

  /* set start point */
  ob::State *check_state = si->allocState();
  ob::CompoundStateSpace::StateType& s = *check_state->as<ob::CompoundStateSpace::StateType>();
  double *check_angular= s.as<ob::RealVectorStateSpace::StateType>(0)->values;
  double *check_position = s.as<ob::RealVectorStateSpace::StateType>(1)->values;
  double *check_velocity = s.as<ob::RealVectorStateSpace::StateType>(2)->values;
  double *check_angular_vel = s.as<ob::RealVectorStateSpace::StateType>(3)->values;
  check_angular[0] = 0.0; check_angular[1] = 0.0; check_angular[2] = 0.0;
  check_velocity[0] = 0.0; check_velocity[1] = 0.0; check_velocity[2] = 0.0;
  check_angular_vel[0] = 0.0; check_angular_vel[1] = 0.0; check_angular_vel[2] = 0.0;
  check_position[0] = 0.0;
  check_position[1] = 0.0;
  check_position[2] = 1.0;

  // set the state propagation routine
  si->setStatePropagator(std::bind(&Hexacopterpropagate, lee_position_controller_.controller_parameters_.allocation_matrix_, lee_position_controller_.vehicle_parameters_, si, std::placeholders::_1,
          std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
  si->setPropagationStepSize(0.02);
  si->setMinMaxControlDuration(1, 10);
  si->setStateValidityCheckingResolution(0.25);
  si->setup();

  ob::State *heading_state = stateSpace->allocState();
  ob::CompoundStateSpace::StateType& hs = *heading_state->as<ob::CompoundStateSpace::StateType>();
  double *heading_angular= hs.as<ob::RealVectorStateSpace::StateType>(0)->values;
  double *heading_position = hs.as<ob::RealVectorStateSpace::StateType>(1)->values;
  double *heading_velocity = hs.as<ob::RealVectorStateSpace::StateType>(2)->values;
  double *heading_angular_vel = hs.as<ob::RealVectorStateSpace::StateType>(3)->values;
  heading_angular[0] = 0.0; heading_angular[1] = 0.0; heading_angular[2] = 0.0;
  heading_position[0] = 2.0; heading_position[1] = -27.0; heading_position[2] = 15.0;
  heading_velocity[0] = 0.0; heading_velocity[1] = 0.0; heading_velocity[2] = 0.0;
  heading_angular_vel[0] = 0.0; heading_angular_vel[1] = 0.0; heading_angular_vel[2] = 0.0;
  ob::ScopedState<ob::CompoundStateSpace> start(stateSpace);
  start = *check_state;
  // start.random();
  ob::ScopedState<ob::CompoundStateSpace> goal(stateSpace);
  // goal.random();
  goal = *heading_state;
  // create a problem instance
  ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));
  // set the start and goal states
  pdef->setStartAndGoalStates(start, goal, 0.2);

  std::shared_ptr<rotors_planner::CL_rrt> planner(new rotors_planner::CL_rrt(si));
  planner->setGoalBias(0.1);
  planner->setName("CL_RRT");
  planner->setPathdeviation(0.2);
  planner->setPathresolution(0.25);
  planner->setRange(3);
  planner->setMinRange(0.20);
  planner->setController(std::bind(&Lee_controller, lee_position_controller_, std::placeholders::_1, std::placeholders::_2,std::placeholders::_3));
  planner->setProblemDefinition(pdef);
  planner->setup();

  // print the settings for this space
  si->printSettings(std::cout);
  // print the problem settings
  pdef->print(std::cout);
  // ob::State *goal_state = pdef->getGoal()->as<ob::GoalState>()->getState();
  // ob::CompoundStateSpace::StateType& goals = *goal_state->as<ob::CompoundStateSpace::StateType>();
  // double *test_goal = goals.as<ob::RealVectorStateSpace::StateType>(1)->values;
  // std::cout<<"goal_state: "<<test_goal[0]<<" "<<test_goal[1]<<" "<<test_goal[2]<<std::endl;

  // attempt to solve the problem within one second of planning time
  ob::PlannerTerminationCondition ptc = ob::timedPlannerTerminationCondition(1.0);
  ob::PlannerStatus solved = planner->solve(ptc);

  if (solved)
      {
        // get the goal representation from the problem definition (not the same as the goal state)
        // and inquire about the found path
        // ob::PathPtr path = pdef->getSolutionPath();
        std::cout << "Found solution:" << std::endl;
        // print the path to screen
        // path->print(std::cout);
      }
   else
      std::cout << "No solution found" << std::endl;
  std::vector<rotors_planner::Motion *> solution_path;
  solution_path = planner->get_solution_motions();

  ROS_INFO("Start publishing trajectory_pub planned trajectory.");

  planner->recordSolution();
  //std::vector<WaypointWithTime> waypoints = planner->waypoints;
  ROS_INFO("Read %d waypoints.", (int) planner->waypoints.size());
  trajectory_msgs::MultiDOFJointTrajectoryPtr msg(new trajectory_msgs::MultiDOFJointTrajectory);
  msg->header.stamp = ros::Time::now();
  msg->points.resize(planner->waypoints.size());
  msg->joint_names.push_back("base_link");
  int64_t time_from_start_ns = 0;
  for (size_t i = 0; i < planner->waypoints.size(); ++i) {
    WaypointWithTime& wp = planner->waypoints[i];

    mav_msgs::EigenTrajectoryPoint trajectory_point;
    trajectory_point.position_W = wp.position;
    trajectory_point.setFromYaw(wp.yaw);
    trajectory_point.time_from_start_ns = time_from_start_ns;

//    std::cout << "Waypoint XYZ: " << wp.position(0) << " "
//              <<  wp.position(1) <<  " " <<  wp.position(2)  << std::endl;

    time_from_start_ns += static_cast<int64_t>(wp.waiting_time * kNanoSecondsInSecond);
    mav_msgs::msgMultiDofJointTrajectoryPointFromEigen(trajectory_point, &msg->points[i]);
  }

  trajectory_pub.publish(msg);
  ros::spinOnce();

  /* rviz interface */
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  visualization_msgs::Marker line_strip, points, line_actual, line_protect, line_tree;
  line_protect.header.frame_id = line_tree.header.frame_id = points.header.frame_id = line_strip.header.frame_id = line_actual.header.frame_id = "/world";
  line_protect.header.stamp = line_tree.header.stamp = points.header.stamp = line_strip.header.stamp = line_actual.header.stamp = ros::Time::now();
  line_protect.ns = line_tree.ns = points.ns = line_strip.ns = line_actual.ns = "planning_trajectory";
  line_protect.action = line_tree.action = points.action = line_strip.action = line_actual.action = visualization_msgs::Marker::ADD;
  //line_strip.action = visualization_msgs::Marker::ARROW;
  line_protect.pose.orientation.w = line_tree.pose.orientation.w = points.pose.orientation.w = line_strip.pose.orientation.w = line_actual.pose.orientation.w = 1.0;

  points.id = 0;
  line_strip.id = 1;
  line_actual.id = 2;
  line_protect.id = 3;
  line_tree.id = 4;
  points.type = visualization_msgs::Marker::POINTS;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  line_actual.type = visualization_msgs::Marker::LINE_STRIP;
  line_protect.type = visualization_msgs::Marker::LINE_STRIP;
  line_tree.type = visualization_msgs::Marker::LINE_LIST;

  points.scale.x = 1;
  points.scale.y = 1;
  points.color.r = 0.0f/255.0f;
  points.color.g = 255.0f/255.0f;
  points.color.b = 154.0f/255.0f;
  points.color.a = 1.0;

  line_strip.scale.x = 0.2;
  line_strip.color.r = 34.0f/255.0f;
  line_strip.color.g = 139.0f/255.0f;
  line_strip.color.b = 34.0f/255.0f;
  line_strip.color.a = 1.0;

  line_actual.scale.x = 0.2;
  line_actual.color.r = 0.0f/255.0f;
  line_actual.color.g = 191.0f/255.0f;
  line_actual.color.b = 255.0f/255.0f;
  line_actual.color.a = 1.0;

  line_protect.scale.x = 0.25;
  line_protect.color.r = 255.0f/255.0f;
  line_protect.color.g = 99.0f/255.0f;
  line_protect.color.b = 71.0f/255.0f;
  line_protect.color.a = 1.0;

  line_tree.scale.x = 0.05;
  line_tree.color.r = 184.0f/255.0f;
  line_tree.color.g = 134.0f/255.0f;
  line_tree.color.b = 11.0f/255.0f;
  line_tree.color.a = 1.0;

  geometry_msgs::Point goal_point;
  goal_point.x = heading_position[0];
  goal_point.y = heading_position[1];
  goal_point.z = heading_position[2];
  points.points.push_back(goal_point);

  for(int i = solution_path.size()-1; i >= 0; --i)
  {
    ob::CompoundStateSpace::StateType& ps = *solution_path[i]->state->as<ob::CompoundStateSpace::StateType>();
    double *point_position = ps.as<ob::RealVectorStateSpace::StateType>(1)->values;
    geometry_msgs::Point p;
    p.x = point_position[0];
    p.y = point_position[1];
    p.z = point_position[2];
    // std::cout<<"point: "<<p.x<<" "<<p.y<<" "<<p.z<<std::endl;
    line_strip.points.push_back(p);
  }

  octomap_msgs::Octomap octo_msg;
  octo_msg.header.frame_id = "/world";
  octomap_msgs::binaryMapToMsg(*octomap_,octo_msg);
  ob::State *update_state = stateSpace->allocState();
  ob::CompoundStateSpace::StateType& us = *update_state->as<ob::CompoundStateSpace::StateType>();
  double *update_angular= us.as<ob::RealVectorStateSpace::StateType>(0)->values;
  double *update_position = us.as<ob::RealVectorStateSpace::StateType>(1)->values;
  double *update_velocity = us.as<ob::RealVectorStateSpace::StateType>(2)->values;
  double *update_angular_vel = us.as<ob::RealVectorStateSpace::StateType>(3)->values;
  update_angular[0] = 0.0; update_angular[1] = 0.0; update_angular[2] = 0.0;

  update_velocity[0] = 0.0; update_velocity[1] = 0.0; update_velocity[2] = 0.0;
  update_angular_vel[0] = 0.0; update_angular_vel[1] = 0.0; update_angular_vel[2] = 0.0;
  Eigen::Map<Eigen::Vector3d> target_pos(heading_position);

  // ros::Rate rate(0.5);
  // begin real time plan
  while(ros::ok()){
    // update current mav state
    ros::spinOnce();
    Eigen::Vector3d update_position_vector = current_odometry.position;
    update_position[0] = update_position_vector[0]; update_position[1] = update_position_vector[1]; update_position[2] = update_position_vector[2];
    geometry_msgs::Point p_f;
    p_f.x = update_position_vector[0];
    p_f.y = update_position_vector[1];
    p_f.z = update_position_vector[2];
    line_actual.points.push_back(p_f);
    OMPL_INFORM("before planning state: %lf %lf %lf",update_position_vector[0], update_position_vector[1], update_position_vector[2]);
    Eigen::Vector3d position_error = current_odometry.position -target_pos;
    if(std::sqrt(position_error.dot(position_error)) < 2*planner->getPathdeviation())
      break;
    
    ob::PlannerTerminationCondition ptc_rt = ob::timedPlannerTerminationCondition(1.0);
    solved = planner->loop_solve(ptc_rt, update_state);

    /* propagate again
       attention: current state is update again
       and evaluate delta_t meanwhile
       prune the solution path according current state */
    ros::spinOnce();
    update_position_vector = current_odometry.position;
    update_position[0] = update_position_vector[0]; update_position[1] = update_position_vector[1]; update_position[2] = update_position_vector[2];
    // planner->recordSolution();
    OMPL_INFORM("after plannnig state: %lf %lf %lf",update_position_vector[0], update_position_vector[1], update_position_vector[2]);
    if(planner->prune_path(update_state))
    {
    solution_path.clear();
    solution_path = planner->get_solution_motions();
    // waypoints interface
    // std::vector<WaypointWithTime> update_waypoints = planner->waypoints;
    ROS_INFO("Read %d waypoints.", (int) planner->waypoints.size());
    trajectory_msgs::MultiDOFJointTrajectoryPtr rt_msg(new trajectory_msgs::MultiDOFJointTrajectory);
    rt_msg->header.stamp = ros::Time::now();
    rt_msg->points.resize(planner->waypoints.size());
    rt_msg->joint_names.push_back("base_link");
    int64_t time_from_start_ns = 0;
    for (size_t i = 0; i < planner->waypoints.size(); ++i) {
      WaypointWithTime& wp = planner->waypoints[i];
      mav_msgs::EigenTrajectoryPoint trajectory_point;
      trajectory_point.position_W = wp.position;
      trajectory_point.setFromYaw(wp.yaw);
      trajectory_point.time_from_start_ns = time_from_start_ns;
      time_from_start_ns += static_cast<int64_t>(wp.waiting_time * kNanoSecondsInSecond);
      mav_msgs::msgMultiDofJointTrajectoryPointFromEigen(trajectory_point, &rt_msg->points[i]);
    }
    ROS_INFO("waypoint msg complete");

    //update line_strip
    line_protect.points.clear();
    line_strip.points.clear();
    line_tree.points.clear();
    bool is_pro_neighbor = false;
    for(int i = solution_path.size()-1; i >= 0; --i)
    {
      ob::CompoundStateSpace::StateType& ps = *solution_path[i]->state->as<ob::CompoundStateSpace::StateType>();
      double *point_position = ps.as<ob::RealVectorStateSpace::StateType>(1)->values;
      geometry_msgs::Point p;
      p.x = point_position[0];
      p.y = point_position[1];
      p.z = point_position[2];
      // std::cout<<"point: "<<p.x<<" "<<p.y<<" "<<p.z<<std::endl;
      if(is_pro_neighbor && solution_path[i]->commit_protect)
        line_protect.points.push_back(p);
      else
      {
        if(is_pro_neighbor)
        {
          line_protect.points.push_back(p);
          is_pro_neighbor = false;
        }
        line_strip.points.push_back(p);
      }
    }
    // ROS_INFO("line strip complete");
    std::stack<rotors_planner::Motion *> box;
    box.push(planner->get_root_node());

    while (!box.empty()) {
      rotors_planner::Motion *current_addmotion = box.top();
      box.pop();
      ob::CompoundStateSpace::StateType& ps = *current_addmotion->state->as<ob::CompoundStateSpace::StateType>();
      double *point_position = ps.as<ob::RealVectorStateSpace::StateType>(1)->values;
      geometry_msgs::Point p;
      p.x = point_position[0];
      p.y = point_position[1];
      p.z = point_position[2];

      for(auto &new_addmotion :current_addmotion->children)
      {
         ob::CompoundStateSpace::StateType& ps_c = *new_addmotion->state->as<ob::CompoundStateSpace::StateType>();
         double *point_position_c = ps_c.as<ob::RealVectorStateSpace::StateType>(1)->values;
         geometry_msgs::Point p_c;
         p_c.x = point_position_c[0];
         p_c.y = point_position_c[1];
         p_c.z = point_position_c[2];
         line_tree.points.push_back(p);
         line_tree.points.push_back(p_c);
         box.push(new_addmotion);
      }
    }

    // publish info
    trajectory_pub.publish(rt_msg);
    marker_pub.publish(points);
    marker_pub.publish(line_strip);
    marker_pub.publish(line_protect);
    marker_pub.publish(line_tree);
   }
    else // no need to plan
    {
      trajectory_msgs::MultiDOFJointTrajectoryPtr rt_msg(new trajectory_msgs::MultiDOFJointTrajectory);
      rt_msg->header.stamp = ros::Time::now();
      rt_msg->points.resize(1);
      rt_msg->joint_names.push_back("base_link");
      mav_msgs::EigenTrajectoryPoint trajectory_point;
      trajectory_point.position_W = update_position_vector;
      trajectory_point.setFromYaw(0);
      trajectory_point.time_from_start_ns = 0.0;
      mav_msgs::msgMultiDofJointTrajectoryPointFromEigen(trajectory_point, &rt_msg->points[0]);
      trajectory_pub.publish(rt_msg);

//      line_strip.points.clear();
//      line_strip.color.r = 255.0f/255.0f;
//      line_strip.color.g = 99.0f/255.0f;
//      line_strip.color.b = 71.0f/255.0f;
//      geometry_msgs::Point p_s;
//      p_s.x = update_position_vector[0];
//      p_s.y = update_position_vector[1];
//      p_s.z = update_position_vector[2];
//      line_strip.points.push_back(p_s);
//      line_strip.points.push_back(p_s);
      marker_pub.publish(points);
     // marker_pub.publish(line_strip);
    }
    geometry_msgs::Point p_e;
    p_e.x = update_position_vector[0];
    p_e.y = update_position_vector[1];
    p_e.z = update_position_vector[2];
    line_actual.points.push_back(p_e);

    marker_pub.publish(line_actual);
    octomap_pub.publish(octo_msg);
    //rate.sleep();
  }

  if(check_state)
    si->freeState(check_state);
  if(heading_state)
    si->freeState(heading_state);
  //octomap_->read(octo_file);
  ros::shutdown();

  return 0;
}

