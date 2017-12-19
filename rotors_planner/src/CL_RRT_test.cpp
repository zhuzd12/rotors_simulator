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
#include <cstring>
#include <boost/filesystem.hpp>

#include "rotors_planner/cl_rrt.h"
#include <rotors_control/parameters_ros.h>
#include <rotors_control/lee_position_controller.h>

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

void Quadrotorpropagate(const Eigen::Matrix4Xd allocation_matrix, rotors_control::VehicleParameters vehicle_parameters_, const oc::SpaceInformationPtr si, const ob::State *start, const oc::Control *control, const double duration, ob::State *result)
{
    static double timestep = .01;
    int nsteps = ceil(duration / timestep);
    double dt = duration / nsteps;

    double *input_rotors_control = control->as<oc::RealVectorControlSpace::ControlType>()->values;
    Eigen::Map<Eigen::Vector4d> U(input_rotors_control);
    for (int k=0; k<4; ++k)
    {
      U(k) = U(k) * U(k);
    }
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
      /*
      Eigen::Matrix3d T_be;
      T_be<< 1.0, 0.0, -sin(Angular.values[1]),
          0.0, cos(Angular.values[0]), sin(Angular.values[0])*cos(Angular.values[1]),
          0.0,  -sin(Angular.values[0]), cos(Angular.values[0])*cos(Angular.values[1]);
      Eigen::Vector3d T_eb_pqr = T_be.inverse() * angular_vel_vector;
      Eigen::Matrix3d T_be;
      T_be<< cos(Angular.values[1]), 0.0, -cos(Angular.values[0])*sin(Angular.values[1]),
          0.0, 1, sin(Angular.values[0]),
          sin(Angular.values[1]),  0.0, cos(Angular.values[0])*cos(Angular.values[1]);
      Eigen::Vector3d T_eb_pqr = T_be.inverse() * angular_vel_vector;  */
      Eigen::Vector3d T_eb_pqr = T_eb * angular_vel_vector;

      Angular.values[0] = angluar_normalization(Angular.values[0] + T_eb_pqr(0) * dt);
      Angular.values[1] = angluar_normalization(Angular.values[1] + T_eb_pqr(1) * dt);
      Angular.values[2] = angluar_normalization(Angular.values[2] + T_eb_pqr(2) * dt);

      /*
      Angular.values[0] = angluar_normalization(Angular.values[0] + Angular_vel.values[0] * dt);
      Angular.values[1] = angluar_normalization(Angular.values[1] + Angular_vel.values[1] * dt);
      Angular.values[2] = angluar_normalization(Angular.values[2] + Angular_vel.values[2] * dt);  */


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

void QuadrotorODE(const Eigen::Matrix4Xd allocation_matrix, rotors_control::VehicleParameters vehicle_parameters_,const oc::ODESolver::StateType& q, const oc::Control* u, oc::ODESolver::StateType& qdot)
{
  double *input_rotors_control = u->as<oc::RealVectorControlSpace::ControlType>()->values;
  Eigen::Map<Eigen::Vector4d> U(input_rotors_control);
  for (int k=0; k<4; ++k)
  {
    U(k) = U(k) * U(k);
  }
  U = allocation_matrix * U;

//  qdot[0] = q[9] + sin(q[0])*tan(q[1])*q[10] + cos(q[0])*tan(q[1])*q[11];
//  qdot[1] = cos(q[0])*q[10] - sin(q[0])*q[11];
//  qdot[2] = q[10]*sin(q[0])/cos(q[1]) + q[11]*cos(q[0])/cos(q[1]);
  qdot[0] = q[9];
  qdot[1] = q[10];
  qdot[2] = q[11];

  qdot[3] = q[6];
  qdot[4] = q[7];
  qdot[5] = q[8];

  qdot[6] = U[3] * (cos(q[0]) * cos(q[2]) * sin(q[1])
      + sin(q[0]) * sin(q[2])) / vehicle_parameters_.mass_;

  qdot[7] = U[3] * (cos(q[0]) * sin(q[2]) * sin(q[1])
      - sin(q[0]) * cos(q[2])) / vehicle_parameters_.mass_;

  qdot[8] = U[3] * cos(q[0]) * cos(q[1])/vehicle_parameters_.mass_ - vehicle_parameters_.gravity_;

  qdot[9] = q[10] * q[11] * (vehicle_parameters_.inertia_(1,1) - vehicle_parameters_.inertia_(2,2))/vehicle_parameters_.inertia_(0,0)
      + U[0]/vehicle_parameters_.inertia_(0,0);
  qdot[10] = q[9] * q[11] * (vehicle_parameters_.inertia_(2,2) - vehicle_parameters_.inertia_(0,0))/vehicle_parameters_.inertia_(1,1)
      + U[1]/vehicle_parameters_.inertia_(1,1);
  qdot[11] = q[9] * q[10] * (vehicle_parameters_.inertia_(0,0) - vehicle_parameters_.inertia_(1,1))/vehicle_parameters_.inertia_(2,2)
      + U[2]/vehicle_parameters_.inertia_(2,2);

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
//   orientation.w() = cos(current_angular(0)/2.0)*cos(current_angular(1)/2.0)*cos(current_angular(2)/2.0)+
//       sin(current_angular(0)/2.0)*sin(current_angular(1)/2.0)*sin(current_angular(2)/2.0);
//   orientation.x() = sin(current_angular(0)/2.0)*cos(current_angular(1)/2.0)*cos(current_angular(2)/2.0)-
//       cos(current_angular(0)/2.0)*sin(current_angular(1)/2.0)*sin(current_angular(2)/2.0);
//   orientation.y() = cos(current_angular(0)/2.0)*sin(current_angular(1)/2.0)*cos(current_angular(2)/2.0)+
//       sin(current_angular(0)/2.0)*cos(current_angular(1)/2.0)*sin(current_angular(2)/2.0);
//   orientation.z() = cos(current_angular(0)/2.0)*cos(current_angular(1)/2.0)*sin(current_angular(2)/2.0)-
//       sin(current_angular(0)/2.0)*sin(current_angular(1)/2.0)*cos(current_angular(2)/2.0);

   //std::cout<<"orientation : "<<orientation.w()<<" "<<orientation.x()<<" "<<orientation.y()<<" "<<orientation.z()<<std::endl;
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

   output->as<oc::RealVectorControlSpace::ControlType>()->values[0] = (*rotor_velocities)(0);
   output->as<oc::RealVectorControlSpace::ControlType>()->values[1] = (*rotor_velocities)(1);
   output->as<oc::RealVectorControlSpace::ControlType>()->values[2] = (*rotor_velocities)(2);
   output->as<oc::RealVectorControlSpace::ControlType>()->values[3] = (*rotor_velocities)(3);

   delete rotor_velocities;
   return true;

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
  R = Eigen::AngleAxisd(current_angular[2], Eigen::Vector3d::UnitZ())
      * Eigen::AngleAxisd(current_angular[1], Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(current_angular[0], Eigen::Vector3d::UnitX());
  //eulerToMatrix<double>(current_angular(0), current_angular(1), current_angular(2), R);
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
  ros::NodeHandle pnh("/pelican/lee_position_controller_node");

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
  stateSpace->addSubspace(SO3, 1.0);
  stateSpace->addSubspace(POS, 1.0);
  stateSpace->addSubspace(velocity, 0.0);
  stateSpace->addSubspace(angular_vel, 0.0);


  // set the bounds for the position space
  ob::RealVectorBounds bounds(3);
  bounds.setLow(-M_PI);
  bounds.setHigh(M_PI);
  SO3->as<ob::RealVectorStateSpace>()->setBounds(bounds);

  //ob::RealVectorBounds bounds(3);
  bounds.setLow(0, -15); bounds.setLow(1, -35); bounds.setLow(2, 0);
  bounds.setHigh(0, 35); bounds.setHigh(1, 15); bounds.setHigh(2, 50);
//  bounds.setLow(0, -150); bounds.setLow(1, -350); bounds.setLow(2, 0);
//  bounds.setHigh(0, 350); bounds.setHigh(1, 150); bounds.setHigh(2, 500);
  POS->as<ob::RealVectorStateSpace>()->setBounds(bounds);

  //ob::RealVectorBounds bounds(3);
  bounds.setLow(-100);
  bounds.setHigh(100);
  velocity->as<ob::RealVectorStateSpace>()->setBounds(bounds);

 // ob::RealVectorBounds bounds(3);
  bounds.setLow(-100);
  bounds.setHigh(100);
  angular_vel->as<ob::RealVectorStateSpace>()->setBounds(bounds);



  oc::ControlSpacePtr cspace(new oc::RealVectorControlSpace(stateSpace, 4));
  // set the bounds for the control space
  ob::RealVectorBounds cbounds(4);
  cbounds.setLow(0.0);
  cbounds.setHigh(1600);
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
  check_angular[0] = 0.0; check_angular[1] = 0.0; check_angular[2] = 0.0;
  check_velocity[0] = 0.0; check_velocity[1] = 0.0; check_velocity[2] = 0.0;
  check_angular_vel[0] = 0.0; check_angular_vel[1] = 0.0; check_angular_vel[2] = 0.0;

  /*
  double check_posion_x[14] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  double check_posion_y[14] = {-3.2, -3.3, -3.35, -3.4, -3.45, -3.55, -3.65, -3.75, -3.8, -3.85, -3.9, -4, -4.1, -4.2};
  double check_posion_z[14] = {0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};

  for(int i = 0; i < 14; ++i)
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
  */

  check_position[0] = 0.0;
  check_position[1] = 0.0;
  check_position[2] = 1.0;

  // set the state propagation routine
  si->setStatePropagator(std::bind(&Quadrotorpropagate, lee_position_controller_.controller_parameters_.allocation_matrix_, lee_position_controller_.vehicle_parameters_, si, std::placeholders::_1,
          std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
//  ompl::control::ODESolverPtr odeSolver (new oc::ODEBasicSolver<> (si, std::bind(&QuadrotorODE, lee_position_controller_.controller_parameters_.allocation_matrix_, lee_position_controller_.vehicle_parameters_, std::placeholders::_1,
//                                                                                             std::placeholders::_2, std::placeholders::_3)));
//  si->setStatePropagator(oc::ODESolver::getStatePropagator(odeSolver));
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
 // std::vector<ob::State *> pstates;
 // ob::PlannerPtr planner(new rotors_planner::CL_rrt(si));
  ob::ScopedState<ob::CompoundStateSpace> start(stateSpace);
  start = *check_state;
  //start.random();
  ob::ScopedState<ob::CompoundStateSpace> goal(stateSpace);
  //goal.random();
  goal = *heading_state;
  // create a problem instance
  ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));

  // set the start and goal states
  pdef->setStartAndGoalStates(start, goal, 0.2);

  std::shared_ptr<rotors_planner::CL_rrt> planner(new rotors_planner::CL_rrt(si));
  planner->setGoalBias(0.2);
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
//  ob::State *goal_state = pdef->getGoal()->as<ob::GoalState>()->getState();
//  ob::CompoundStateSpace::StateType& goals = *goal_state->as<ob::CompoundStateSpace::StateType>();
//  double *test_goal = goals.as<ob::RealVectorStateSpace::StateType>(1)->values;
//  std::cout<<"goal_state: "<<test_goal[0]<<" "<<test_goal[1]<<" "<<test_goal[2]<<std::endl;


  // attempt to solve the problem within one second of planning time
  ob::PlannerTerminationCondition ptc = ob::timedPlannerTerminationCondition(20.0);
  ob::PlannerStatus solved = planner->solve(ptc);

  if (solved)
      {
          // get the goal representation from the problem definition (not the same as the goal state)
          // and inquire about the found path
          ob::PathPtr path = pdef->getSolutionPath();
          std::cout << "Found solution:" << std::endl;

          // print the path to screen
          path->print(std::cout);
         // std::cout<<"first loop find solution"<<std::endl;
      }
   else
      std::cout << "No solution found" << std::endl;
  std::vector<rotors_planner::Motion *> solution_path;
  solution_path = planner->get_solution_motions();
  //int index = solution_path.size()- 100;
//  std::cout<<"solution states: "<<solution_path.size()<<std::endl;
//  std::cout<<"new current state index: "<<index<<std::endl;
//  ob::PlannerTerminationCondition ptc2 = ob::timedPlannerTerminationCondition(2.0);


  /*
  solved = planner->loop_solve(ptc2, solution_path[index]->state);
  if (solved)
      {
          // get the goal representation from the problem definition (not the same as the goal state)
          // and inquire about the found path
         // ob::PathPtr path = pdef->getSolutionPath();
          std::cout << "Found solution:" << std::endl;

          // print the path to screen
          //path->print(std::cout);

      }
   else
      std::cout << "No solution found" << std::endl;
  */

  /*
  std::vector<ob::State *> pstates;
  std::vector<oc::Control *> pcontrols;
  if(planner->propagateuntilstop(check_state, heading_state, pstates, pcontrols))
  {
    size_t p = 0;
    for(; p < pstates.size(); ++p)
    {
      ob::CompoundStateSpace::StateType *ps = pstates[p]->as<ob::CompoundStateSpace::StateType>();
      std::cout<<p<<" state position: "<<ps->as<ob::RealVectorStateSpace::StateType>(1)->values[0] <<" "
              << ps->as<ob::RealVectorStateSpace::StateType>(1)->values[1] <<" " <<ps->as<ob::RealVectorStateSpace::StateType>(1)->values[2]<<std::endl;
    }
  }
  */

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


  /* rviz interface */

  ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  visualization_msgs::Marker line_strip, points;
  points.header.frame_id = line_strip.header.frame_id = "/world";
  points.header.stamp = line_strip.header.stamp = ros::Time::now();
  points.ns = line_strip.ns = "planning_trajectory";
  points.action = line_strip.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = line_strip.pose.orientation.w = 1.0;

  points.id = 0;
  line_strip.id = 1;
  points.type = visualization_msgs::Marker::POINTS;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;

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
    std::cout<<"point: "<<p.x<<" "<<p.y<<" "<<p.z<<std::endl;
    line_strip.points.push_back(p);
  }



  ros::spinOnce();
  octomap_msgs::Octomap octo_msg;
  octo_msg.header.frame_id = "/world";
  octomap_msgs::binaryMapToMsg(*octomap_,octo_msg);

  ros::Rate rate(10);
  while(ros::ok()){
    octomap_pub.publish(octo_msg);
    marker_pub.publish(points);
    marker_pub.publish(line_strip);

    rate.sleep();
  }
  //octomap_->read(octo_file);
  ros::shutdown();

  return 0;
}

