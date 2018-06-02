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

#include "rotors_planner/common.h"

#include <Eigen/Dense>

#include <std_msgs/String.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/SetBoolRequest.h>
#include <std_srvs/SetBoolResponse.h>

bool isStateValid(const rotors_control::EigenOdometry& odometry, const octomap::OcTree *octree_)
{
    //std::cout<<"get in isStateValid"<<std::endl;
  // extract the position and construct the UAV box
  //Eigen::Quaterniond orientation = odometry.orientation;
  //Eigen::Map<Eigen::Vector3d> current_angular(odometry.orientation);
  Eigen::Vector3d current_position = odometry.position;
  const Eigen::Matrix3d R_W_I = odometry.orientation.toRotationMatrix();
 // std::cout <<"current position X:" <<current_position(0) <<"Y: "<<current_position(1)<<"Z: "<<current_position(2)<<std::endl;
  //fcl::Box uav_box(0.3,0.3,0.3);
  //std::shared_ptr<fcl::Box<float>> uav_box(new fcl::Box<float>(0.1,0.1,0.1));
  std::shared_ptr<fcl::CollisionGeometry<double>> boxGeometry (new fcl::Box<double> (0.5, 0.5, 0.5));
//  fcl::BVHModel<fcl::OBBRSS<float>> uav_model;
//  fcl::BVHModel<fcl::OBBRSS<float>> *uav_model_ptr = &uav_model;
  fcl::Matrix3d R = R_W_I;
  //std::cout<<R_W_I(0,0)<<" "<<R_W_I(0,1)<<" "<<R_W_I(0,2)<<" "<<R_W_I(1,0)<<" "<<R_W_I(1,1)<<" "<<R_W_I(1,2)<<" "<<R_W_I(2,0)<<" "<<R_W_I(2,1)<<" "<<R_W_I(2,2)<<std::endl;
  //R = Eigen::AngleAxisd(current_angular[2], Eigen::Vector3d::UnitZ())
      //* Eigen::AngleAxisd(current_angular[1], Eigen::Vector3d::UnitY())
      //* Eigen::AngleAxisd(current_angular[0], Eigen::Vector3d::UnitX());
  //eulerToMatrix<double>(current_angular(0), current_angular(1), current_angular(2), R);
  // Eigen::Map<Eigen::Matrix<float, 3, 1>> current_pos(current_position);
  // fcl::generateBVHModel(uav_model, uav_box, fcl::Transform3f(R, current_position));
  // fcl::Transform3f tf(R, current_position);
  fcl::Transform3d tf;
  tf.setIdentity();
  tf.linear() = R_W_I;
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
  //std::cout<<"begin to use fcl"<<std::endl;
  fcl::collide(env_obj, uav_obj, request, result);

  bool is_in_box = false;
  if(current_position[0] > -15 && current_position[0] < 35 && current_position[1] < 15 && current_position[1] >-35 && current_position[2]<50 && current_position[2]>0){
      std::cout<<"mav not out of range"<<std::endl;
      is_in_box = true;
  }
  if(result.isCollision()){
      std::cout<<"mav in collision"<<std::endl;
  }
  return is_in_box && !result.isCollision();
  //return !result.isCollision();
}

octomap::OcTree *octomap_;
rotors_control::EigenOdometry current_odometry;

bool check_valid_state(std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &res){
    if(req.data){
        res.success = true;
        //std::cout<<"res.data is true"<<std::endl;
        if (isStateValid(current_odometry, octomap_)){
            res.message = "valid_state";
           // std::cout<<"valid_state"<<std::endl;
        }
        else
        {
            res.message = "invalid_state";
            //std::cout<<"invalid_state"<<std::endl;
        }
    }
    else{
        res.success = true;
        res.message = "";
    }
    return true;
}



void OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg) {
  //std::cout<<"odometry!"<<std::endl;
  rotors_control::eigenOdometryFromMsg(odometry_msg, &current_odometry);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "statevalid");
  ros::NodeHandle nh;

  // The IMU is used, to determine if the simulator is running or not.
  //ros::Subscriber sub = nh.subscribe("imu", 10, &callback);

  ros::Subscriber odometry_sub_ = nh.subscribe("odometry_sensor1/odometry", 1,
                               OdometryCallback);

  ros::Publisher octomap_pub = nh.advertise<octomap_msgs::Octomap>("/octomap", 10);

  // get the octomap of the world

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
  octomap_msgs::Octomap octo_msg;
  octo_msg.header.frame_id = "/world";
  octomap_msgs::binaryMapToMsg(*octomap_,octo_msg);

  ros::ServiceServer cmd_quit_motor_server = nh.advertiseService("check_valid_state", &check_valid_state);
  //ros::spinOnce();
  //ros::Rate rate(10);
  //while(ros::ok()){
    //octomap_pub.publish(octo_msg);
    //rate.sleep();
 // }
  //ros::shutdown();
  ros::spin();
  return 0;

}
