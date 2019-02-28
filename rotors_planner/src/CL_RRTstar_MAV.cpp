#include <ros/ros.h>
#include <Eigen/Geometry>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <sensor_msgs/Imu.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <visualization_msgs/Marker.h>
#include<algorithm>

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
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/DiscreteMotionValidator.h>

#include <fcl/config.h>
#include <fcl/geometry/octree/octree.h>
#include <fcl/geometry/shape/box.h>
#include <fcl/broadphase/broadphase_collision_manager.h>
#include <fcl/narrowphase/collision.h>
#include <fcl/fcl.h>


#include <iostream>
#include <string>
#include <boost/filesystem.hpp>
#include <math.h>

#include "rotors_planner/cl_rrt_star.h"
#include "rotors_planner/discretedmotionvalidator.h"
#include <rotors_control/parameters_ros.h>
#include <rotors_control/lee_position_controller.h>

#include "rotors_planner/common.h"

#include <Eigen/Dense>

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
void Hexacopterpropagate(double step_size, const Eigen::Matrix4Xd allocation_matrix, rotors_control::VehicleParameters vehicle_parameters_, const ob::SpaceInformationPtr si, const ob::State *start, const Eigen::VectorXd *control, const double duration, ob::State *result)
{
    static double timestep = step_size;
    int nsteps = ceil(duration / timestep);
    double dt = duration / nsteps;

    // double *input_rotors_control = control->as<oc::RealVectorControlSpace::ControlType>()->values;
    Eigen::VectorXd rotors_(vehicle_parameters_.rotor_configuration_.rotors.size());
    for (int k=0; k<vehicle_parameters_.rotor_configuration_.rotors.size(); ++k)
    {
      rotors_(k) = (*control)(k) * (*control)(k);
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
bool Lee_controller(rotors_control::LeePositionController lee_position_controller_ , const ob::State *state, const ob::State *heading_state, Eigen::VectorXd *rotor_velocities)
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
    // Eigen::VectorXd *rotor_velocities = new(Eigen::VectorXd);

    trajectory_point.position_W = position;
    trajectory_point.setFromYaw(yaw);
    //trajectory_point.time_from_start_ns = 0;
    lee_position_controller_.SetOdometry(odometry_);
    lee_position_controller_.SetTrajectoryPoint(trajectory_point);
    lee_position_controller_.CalculateRotorVelocities(rotor_velocities);

    // for(int i = 0; i < rotor_velocities->size(); i++)
    // {
    //     output->as<oc::RealVectorControlSpace::ControlType>()->values[i] = (*rotor_velocities)(i);
    // }
    // delete rotor_velocities;
    return true;

}

/* define model controller combined propagation function */
typedef std::function<bool(const ob::State *, const ob::State *, Eigen::VectorXd *)> ControllerFn;
typedef std::function<void(const ob::State *, const Eigen::VectorXd *, const double, ob::State *)> StatePropagatorFn;
bool propagationFn(StatePropagatorFn model, ControllerFn controller, const ob::State * start_s, const ob::State * end_s, const double duration, ob::State * next_s)
{
    // Eigen::VectorXd cmd;
    Eigen::VectorXd *cmd = new(Eigen::VectorXd);
    controller(start_s, end_s, cmd);
    model(start_s, cmd, duration, next_s);
    return true;
}

/* define the state validation fucntion */
bool isStateValid(const ob::SpaceInformation *si, const octomap::OcTree *octree_, const ob::State *state)
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

/* define a simpler state validation fucntion */
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

ob::OptimizationObjectivePtr getPathLengthObjective(const ob::SpaceInformationPtr& si)
{
  return ob::OptimizationObjectivePtr(new ob::PathLengthOptimizationObjective(si));
}

// set sampler clean function
bool ODEstate_to_position_sampler(const ob::SpaceInformationPtr &si, ob::State * rstate)
{
if(rstate == nullptr)
    return false;
ob::CompoundStateSpace::StateType& hs = *rstate->as<ob::CompoundStateSpace::StateType>();
double *angular= hs.as<ob::RealVectorStateSpace::StateType>(0)->values;
double *velocity = hs.as<ob::RealVectorStateSpace::StateType>(2)->values;
double *angular_vel = hs.as<ob::RealVectorStateSpace::StateType>(3)->values;
std::fill(angular, angular+3, 0.0);
std::fill(velocity, velocity+3, 0.0);
std::fill(angular_vel, angular_vel+3, 0.0);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "CL_RRTstar_MAV");
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

  // Wait for 10s such that everything can settle and the mav flies to the initial position.
  ros::Duration(10).sleep();

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


  // define the state space
  ob::StateSpacePtr SO3(new ob::RealVectorStateSpace(3));
  SO3->setName("Angular");
  ob::StateSpacePtr POS(new ob::RealVectorStateSpace(3));
  POS->setName("Position");
  ob::StateSpacePtr velocity(new ob::RealVectorStateSpace(3));
  velocity->setName("Velocity");
  ob::StateSpacePtr angular_vel(new ob::RealVectorStateSpace(3));
  angular_vel->setName("Angular_vel");

  std::shared_ptr<ob::CompoundStateSpace> stateSpace(new ob::CompoundStateSpace());
  stateSpace->addSubspace(SO3, 0.0);
  stateSpace->addSubspace(POS, 1.0);
  stateSpace->addSubspace(velocity, 0.0);
  stateSpace->addSubspace(angular_vel, 0.0);

  // set the bounds for the angular subspace
  ob::RealVectorBounds bounds(3);
  bounds.setLow(-M_PI);
  bounds.setHigh(M_PI);
  SO3->as<ob::RealVectorStateSpace>()->setBounds(bounds);
  
  // set the bounds for the position subspace
  double workspace_x_min, workspace_x_max, workspace_y_min, workspace_y_max, workspace_z_min, workspace_z_max;
  pnh.getParam("workspace/min_x", workspace_x_min);
  pnh.getParam("workspace/max_x", workspace_x_max);
  pnh.getParam("workspace/min_y", workspace_y_min);
  pnh.getParam("workspace/max_y", workspace_y_max);
  pnh.getParam("workspace/min_z", workspace_z_min);
  pnh.getParam("workspace/max_z", workspace_z_max);
  bounds.setLow(0, workspace_x_min); bounds.setLow(1, workspace_y_min); bounds.setLow(2, workspace_z_min);
  bounds.setHigh(0, workspace_x_max); bounds.setHigh(1, workspace_y_max); bounds.setHigh(2, workspace_z_max);
  POS->as<ob::RealVectorStateSpace>()->setBounds(bounds);
  // POS->setValidSegmentCountFactor(1.0);
//   double space_extent= sqrt(pow(workspace_z_max-workspace_z_min, 2)+pow(workspace_y_max-workspace_y_min, 2)+pow(workspace_x_max-workspace_x_min, 2));
//   double longestValidSegment;
//   pnh.getParam("planner/path_resolution", longestValidSegment);
//   POS->setLongestValidSegmentFraction(0.1*longestValidSegment/space_extent);
  
  // set the bounds for the velocity subspace
  bounds.setLow(-10);
  bounds.setHigh(10);
  velocity->as<ob::RealVectorStateSpace>()->setBounds(bounds);
  
  // set the bounds for the angular_vel subspace
  bounds.setLow(-5);
  bounds.setHigh(5);
  angular_vel->as<ob::RealVectorStateSpace>()->setBounds(bounds);

  size_t rotors_size = lee_position_controller_.vehicle_parameters_.rotor_configuration_.rotors.size();
  ROS_INFO("rotor size: %d", int (rotors_size));
  oc::ControlSpacePtr cspace(new oc::RealVectorControlSpace(stateSpace, rotors_size));
  // set the bounds for the control space
  ob::RealVectorBounds cbounds(rotors_size);
  cbounds.setLow(0.0);
  cbounds.setHigh(1600);
  cspace->as<oc::RealVectorControlSpace>()->setBounds(cbounds);

  // construct an instance of space information from this control space
  //   oc::SpaceInformationPtr si(new oc::SpaceInformation(stateSpace, cspace));
  ob::SpaceInformationPtr si(new ob::SpaceInformation(stateSpace));
  

  // load octomap 
  octomap::OcTree *octomap_;
  std::string octo_file ;
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

  // set state validity checking for state space
  si->setStateValidityChecker(std::bind(&isStateValid, si.get(), octomap_, std::placeholders::_1));
  

  // set start point 
  double target_x, target_y, target_z, target_yaw;
  pnh.getParam("target_x", target_x);
  pnh.getParam("target_y", target_y);
  pnh.getParam("target_z", target_z);
  pnh.getParam("target_yaw", target_yaw);
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
  double step_size, waypoint_interval, max_loop;
  pnh.getParam("propagation/step_size", step_size);
  pnh.getParam("propagation/waypoint_interval", waypoint_interval);
  pnh.getParam("propagation/max_loop", max_loop);
//   si->setStatePropagator(std::bind(&Hexacopterpropagate, step_size, lee_position_controller_.controller_parameters_.allocation_matrix_, lee_position_controller_.vehicle_parameters_, si, std::placeholders::_1,
//           std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
//   si->setPropagationStepSize(waypoint_interval);
//   si->setMinMaxControlDuration(1, 10);

  double path_deviation, initial_plan_time, path_resolution, sample_max_range, loop_plan_time, propagate_time;
  bool use_propagation, useKNearest;
  double path_replan_deviation;
  bool useTrejectoryExpansion;
  pnh.getParam("planner/path_deviation", path_deviation);
  pnh.getParam("planner/path_resolution", path_resolution);
  pnh.getParam("planner/sample_connect_range", sample_max_range);
  pnh.getParam("planner/initial_plan_time", initial_plan_time);
  pnh.getParam("planner/loop_plan_time", loop_plan_time);
  pnh.getParam("propagation/use_propagation", use_propagation);
  pnh.getParam("planner/setKNearest", useKNearest);
  pnh.getParam("planner/path_replan_deviation", path_replan_deviation);
  pnh.getParam("planner/useTrejectoryExpansion", useTrejectoryExpansion);
  
  StatePropagatorFn model = std::bind(&Hexacopterpropagate, step_size, lee_position_controller_.controller_parameters_.allocation_matrix_, lee_position_controller_.vehicle_parameters_, si, std::placeholders::_1,
        std::placeholders::_2, std::placeholders::_3, std::placeholders::_4);
  ControllerFn controller = std::bind(&Lee_controller, lee_position_controller_, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3); 

  if(use_propagation == 1)
  {
    std::shared_ptr<ob::ModelMotionValidator> mv(new ob::ModelMotionValidator(si, path_resolution, max_loop, waypoint_interval, std::bind(&propagationFn, model, controller, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4)));
    si->setMotionValidator(mv);
    OMPL_INFORM("use propagation");
  }
  else
  {
    ob::MotionValidatorPtr mv(new ob::DiscretedMotionValidator(si));
    si->setMotionValidator(mv);
    OMPL_INFORM("do not use propagation");
  }

  si->setStateValidityCheckingResolution(path_resolution);
  si->setup();

  ob::State *heading_state = stateSpace->allocState();
  ob::CompoundStateSpace::StateType& hs = *heading_state->as<ob::CompoundStateSpace::StateType>();
  double *heading_angular= hs.as<ob::RealVectorStateSpace::StateType>(0)->values;
  double *heading_position = hs.as<ob::RealVectorStateSpace::StateType>(1)->values;
  double *heading_velocity = hs.as<ob::RealVectorStateSpace::StateType>(2)->values;
  double *heading_angular_vel = hs.as<ob::RealVectorStateSpace::StateType>(3)->values;
  heading_angular[0] = 0.0; heading_angular[1] = 0.0; heading_angular[2] = target_yaw;
  heading_position[0] = target_x; heading_position[1] = target_y; heading_position[2] = target_z;
  heading_velocity[0] = 0.0; heading_velocity[1] = 0.0; heading_velocity[2] = 0.0;
  heading_angular_vel[0] = 0.0; heading_angular_vel[1] = 0.0; heading_angular_vel[2] = 0.0;
  
  ob::ScopedState<ob::CompoundStateSpace> start(stateSpace);
  start = *check_state;

  ob::ScopedState<ob::CompoundStateSpace> goal(stateSpace);
  goal = *heading_state;

//   ob::OptimizationObjectivePtr getPathLengthObjective(const ob::SpaceInformationPtr& si)
//   {
//     return ob::OptimizationObjectivePtr(new ob::PathLengthOptimizationObjective(si));
//   }

  // create a problem instance
  ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));
  pdef->setStartAndGoalStates(start, goal, path_deviation);
  // set the problem definition
  pdef->setOptimizationObjective(getPathLengthObjective(si));

  std::shared_ptr<rotors_planner_rrtstar::CL_RRTstar> planner(new rotors_planner_rrtstar::CL_RRTstar(si));
  planner->setGoalBias(path_deviation);
  planner->setName("CL_RRT_star");
//   planner->setPathdeviation(path_deviation);
//   planner->setPathresolution(path_resolution);
  rotors_planner_rrtstar::samplercleanerFn svc = ODEstate_to_position_sampler;
  planner->setSampleCleanerFn(svc);
  planner->setRange(sample_max_range);
  planner->setKNearest(useKNearest);
  planner->setTrejectoryExpansion(useTrejectoryExpansion);
  planner->setReplanPathDeviation(path_replan_deviation);
  planner->setSampleDimension(3);
  std::shared_ptr<ob::ModelMotionValidator> m_v(new ob::ModelMotionValidator(si, path_resolution, max_loop, waypoint_interval, std::bind(&propagationFn, model, controller, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4)));
  planner->setMV(m_v);
  // planner->setMinRange(0.20);
  // planner->setController(std::bind(&Lee_controller, lee_position_controller_, std::placeholders::_1, std::placeholders::_2,std::placeholders::_3));
  planner->setProblemDefinition(pdef);
  planner->setup();
  // print the settings for this space
  si->printSettings(std::cout);
  // print the problem settings
  pdef->print(std::cout);

  /*
  ob::PlannerTerminationCondition ptc = ob::timedPlannerTerminationCondition(initial_plan_time);
  ob::PlannerStatus solved = planner->solve(ptc);

  if (solved)
      {
        // get the goal representation from the problem definition (not the same as the goal state)
        // and inquire about the found path
        ob::PathPtr path = pdef->getSolutionPath();
        std::cout << "Found solution:" << std::endl;
        // print the path to screen
        // path->print(std::cout);
      }
   else
      std::cout << "No solution found" << std::endl;

  //   std::vector<rotors_planner_rrtstar::Motion *> solution_path;
  //   solution_path = planner->get_solution_motions();

  ROS_INFO("Start publishing trajectory_pub planned trajectory.");

  //   auto best_path = std::dynamic_pointer_cast<og::PathGeometric>(pdef->getSolutionPath());
  ob::PathPtr path = pdef->getSolutionPath();
  og::PathGeometric *best_path = path->as<og::PathGeometric>();
  //   ob::PlannerSolution best_solution;
  //   pdef->getSolution(best_solution);

  std::vector< ob::State *> solution_states = best_path->getStates();

  // get trajectory msg from solution path
  ROS_INFO("Read %d waypoints.", int(solution_states.size()));
  trajectory_msgs::MultiDOFJointTrajectoryPtr msg(new trajectory_msgs::MultiDOFJointTrajectory);
  msg->header.stamp = ros::Time::now();
  msg->points.resize(solution_states.size());
  msg->joint_names.push_back("base_link");
  int64_t time_from_start_ns = 0;
  for (size_t i = 0; i < solution_states.size(); ++i) {
    ob::State * current_s = solution_states[i];
    ob::CompoundStateSpace::StateType& c_s = *current_s->as<ob::CompoundStateSpace::StateType>();
    const Eigen::Map<Eigen::Vector3d> c_position(c_s.as<ob::RealVectorStateSpace::StateType>(1)->values);
    const double c_yaw = c_s.as<ob::RealVectorStateSpace::StateType>(0)->values[2];
    
    mav_msgs::EigenTrajectoryPoint trajectory_point;
    trajectory_point.position_W = c_position;
    trajectory_point.setFromYaw(c_yaw);
    // std::cout <<"waypoint position: "<<c_position(0) <<" "<<c_position(1)<<" "<<c_position(2)<<std::endl;
    // std::cout<<"waypoint yaw: "<<c_yaw<<std::endl;
    trajectory_point.time_from_start_ns = time_from_start_ns;

    time_from_start_ns += 1.0;
    mav_msgs::msgMultiDofJointTrajectoryPointFromEigen(trajectory_point, &msg->points[i]);
  }

//   trajectory_pub.publish(msg);
//   ros::spinOnce();

*/

  /* rviz interface */
  bool show_tree;
  pnh.getParam("rviz/show_tree", show_tree);
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
  line_tree.color.r = 90.0f/255.0f;
  line_tree.color.g = 90.0f/255.0f;
  line_tree.color.b = 90.0f/255.0f;
  line_tree.color.a = 1.0;

  geometry_msgs::Point goal_point;
  goal_point.x = heading_position[0];
  goal_point.y = heading_position[1];
  goal_point.z = heading_position[2];
  points.points.push_back(goal_point);

  /*
  for (size_t i = 0; i < solution_states.size(); ++i)
  {
    ob::State * current_s = solution_states[i];
    ob::CompoundStateSpace::StateType& ps = *current_s->as<ob::CompoundStateSpace::StateType>();
    // ob::CompoundStateSpace::StateType& ps = *solution_path[i]->state->as<ob::CompoundStateSpace::StateType>();
    double *point_position = ps.as<ob::RealVectorStateSpace::StateType>(1)->values;
    geometry_msgs::Point p;
    p.x = point_position[0];
    p.y = point_position[1];
    p.z = point_position[2];
    line_strip.points.push_back(p);
  }
  */
  octomap_msgs::Octomap octo_msg;
  octo_msg.header.frame_id = "/world";
  octomap_msgs::binaryMapToMsg(*octomap_,octo_msg);

  

  // publish msg
 //  trajectory_pub.publish(msg);
//   marker_pub.publish(points);
//   marker_pub.publish(line_strip);
  octomap_pub.publish(octo_msg);


  // real-time state for loop planning
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

  std::shared_ptr<ompl::geometric::PathGeometric> real_time_best_path;
  ros::spinOnce();
   while (ros::ok()) {
    
    // trajectory_pub.publish(msg);
    // ros::spinOnce();
    Eigen::Vector3d update_position_vector = current_odometry.position;
    update_position[0] = update_position_vector[0]; update_position[1] = update_position_vector[1]; update_position[2] = update_position_vector[2];
    
    geometry_msgs::Point p_f;
    p_f.x = update_position_vector[0];
    p_f.y = update_position_vector[1];
    p_f.z = update_position_vector[2];
    line_actual.points.push_back(p_f);

    OMPL_INFORM("before planning state: %lf %lf %lf",update_position_vector[0], update_position_vector[1], update_position_vector[2]);
    Eigen::Vector3d position_error = current_odometry.position -target_pos;
    if(std::sqrt(position_error.dot(position_error)) < 2*path_deviation)
      break;

    ob::PlannerTerminationCondition ptc_rt = ob::timedPlannerTerminationCondition(loop_plan_time);
    pdef->clearStartStates();
    pdef->addStartState(update_state);
    planner->setProblemDefinition(pdef);
    ob::PlannerStatus solved = planner->solve(ptc_rt);

    ros::spinOnce();
    update_position_vector = current_odometry.position;
    update_position[0] = update_position_vector[0]; update_position[1] = update_position_vector[1]; update_position[2] = update_position_vector[2];
    OMPL_INFORM("after plannnig state: %lf %lf %lf",update_position_vector[0], update_position_vector[1], update_position_vector[2]);

    std::vector<double> time_stamps;
    bool rePropagation_success = planner->rePropagation(update_state, real_time_best_path, time_stamps);
    // if(true)
    // {
    std::vector< ob::State *> solution_states = real_time_best_path->getStates();
    // get trajectory msg from solution path
    ROS_INFO("Read %d waypoints.", int(solution_states.size()));
    trajectory_msgs::MultiDOFJointTrajectoryPtr traj_msg(new trajectory_msgs::MultiDOFJointTrajectory);
    traj_msg->header.stamp = ros::Time::now();
    traj_msg->points.resize(solution_states.size());
    traj_msg->joint_names.push_back("base_link");
    int64_t time_from_start_ns = 0;
    for (size_t i = 0; i < solution_states.size(); ++i) {
        ob::State * current_s = solution_states[i];
        ob::CompoundStateSpace::StateType& c_s = *current_s->as<ob::CompoundStateSpace::StateType>();
        const Eigen::Map<Eigen::Vector3d> c_position(c_s.as<ob::RealVectorStateSpace::StateType>(1)->values);
        const double c_yaw = c_s.as<ob::RealVectorStateSpace::StateType>(0)->values[2];
        
        mav_msgs::EigenTrajectoryPoint trajectory_point;
        trajectory_point.position_W = c_position;
        trajectory_point.setFromYaw(c_yaw);
        // if(i<30)
        // {
        //     std::cout <<"waypoint position: "<<c_position(0) <<" "<<c_position(1)<<" "<<c_position(2)<<std::endl;
        //     std::cout<<"waypoint yaw: "<<c_yaw<<std::endl;
        // }
        trajectory_point.time_from_start_ns = time_from_start_ns;
        time_from_start_ns += static_cast<int64_t>(waypoint_interval * kNanoSecondsInSecond);
        mav_msgs::msgMultiDofJointTrajectoryPointFromEigen(trajectory_point, &traj_msg->points[i]);
    }

    //update marker line_strip
    line_protect.points.clear();
    line_strip.points.clear();
    line_tree.points.clear();
    for (size_t i = 0; i < solution_states.size(); ++i)
    {
        ob::State * current_s = solution_states[i];
        ob::CompoundStateSpace::StateType& ps = *current_s->as<ob::CompoundStateSpace::StateType>();
        double *point_position = ps.as<ob::RealVectorStateSpace::StateType>(1)->values;
        geometry_msgs::Point p;
        p.x = point_position[0];
        p.y = point_position[1];
        p.z = point_position[2];
        // if(i<3)
        // {
        //     std::cout <<"marker position: "<<p.x <<" "<<p.y<<" "<<p.z<<std::endl;
        // }
        line_strip.points.push_back(p);
    }

    //update marker line_tree
    std::stack<rotors_planner_rrtstar::Motion *> box;
    box.push(planner->get_root_node());
    while (!box.empty()) {
    rotors_planner_rrtstar::Motion *current_addmotion = box.top();
    box.pop();
    ob::CompoundStateSpace::StateType& ps = *current_addmotion->state->as<ob::CompoundStateSpace::StateType>();
    double *point_position = ps.as<ob::RealVectorStateSpace::StateType>(1)->values;
    geometry_msgs::Point p;
    p.x = point_position[0];
    p.y = point_position[1];
    p.z = point_position[2];

    if(show_tree)
    {
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
    }
    // }
    // else // plan failed and send stop trajectory
    // {
    //     ROS_ERROR("planning failed");
    //     trajectory_msgs::MultiDOFJointTrajectoryPtr rt_msg(new trajectory_msgs::MultiDOFJointTrajectory);
    //     rt_msg->header.stamp = ros::Time::now();
    //     rt_msg->points.resize(1);
    //     rt_msg->joint_names.push_back("base_link");
    //     mav_msgs::EigenTrajectoryPoint trajectory_point;
    //     trajectory_point.position_W = update_position_vector;
    //     trajectory_point.setFromYaw(0);
    //     trajectory_point.time_from_start_ns = 0.0;
    //     mav_msgs::msgMultiDofJointTrajectoryPointFromEigen(trajectory_point, &rt_msg->points[0]);
    //     trajectory_pub.publish(rt_msg);
    // }

    // update marker line actual
    geometry_msgs::Point p_e;
    p_e.x = update_position_vector[0];
    p_e.y = update_position_vector[1];
    p_e.z = update_position_vector[2];
    line_actual.points.push_back(p_e);

    // publish msg
    trajectory_pub.publish(traj_msg);
    marker_pub.publish(line_actual);
    marker_pub.publish(points);
    marker_pub.publish(line_strip);
    if(show_tree)
        marker_pub.publish(line_tree);    
    // octomap_pub.publish(octo_msg);
    ros::spinOnce();
  }
  
  // free memory
  if(check_state)
    si->freeState(check_state);
  if(heading_state)
    si->freeState(heading_state);
    if(update_state)
    si->freeState(update_state);

  ros::shutdown();
  return 0;
}

