#ifndef LEE_POSITION_CONTROLLER_FIXNODE_H
#define LEE_POSITION_CONTROLLER_FIXNODE_H
#include <boost/bind.hpp>
#include <Eigen/Eigen>
#include <stdio.h>
#include <std_msgs/String.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/SetBoolRequest.h>
#include <std_srvs/SetBoolResponse.h>
#include <string.h>
#include <geometry_msgs/PoseStamped.h>
#include <mav_msgs/Actuators.h>
#include <mav_msgs/AttitudeThrust.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <nav_msgs/Odometry.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include "rotors_control/common.h"
#include "rotors_control/lee_position_controller.h"

namespace rotors_control {

class LeePositionControllerFixNode {
 public:
  LeePositionControllerFixNode();
  ~LeePositionControllerFixNode();

  void InitializeParams();
  void Publish();

 private:

  LeePositionController lee_position_controller_;

  std::string namespace_;

  // subscribers
  ros::Subscriber cmd_trajectory_sub_;
  ros::Subscriber cmd_multi_dof_joint_trajectory_sub_;
  ros::Subscriber cmd_pose_sub_;
  ros::Subscriber odometry_sub_;
  ros::Subscriber cmd_quit_motor_sub_;

  ros::Publisher motor_velocity_reference_pub_;

  std::string motor_stop = "normal";

  mav_msgs::EigenTrajectoryPointDeque commands_;
  std::deque<ros::Duration> command_waiting_times_;
  ros::Timer command_timer_;

  void TimedCommandCallback(const ros::TimerEvent& e);

  void MultiDofJointTrajectoryCallback(
      const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& trajectory_reference_msg);

  void CommandPoseCallback(
      const geometry_msgs::PoseStampedConstPtr& pose_msg);

  void OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg);

  void GymRenderCallback(const std_msgs::String::ConstPtr& gymrender_msg);
public:
  bool set_motor_authority(std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &res);
};
}
#endif // LEE_POSITION_CONTROLLER_FIXNODE_H
