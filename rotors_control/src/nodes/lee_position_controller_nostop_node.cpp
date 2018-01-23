
#include <ros/ros.h>
#include <mav_msgs/default_topics.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include "lee_position_controller_nostop_node.h"

#include "rotors_control/parameters_ros.h"


namespace rotors_control {

double angluar_normalization(double angular)
{
  double temp = std::fmod(angular, 2.0 * M_PI);
  if(temp < -M_PI)
    temp += 2.0 * M_PI;
  else if(temp >= M_PI)
    temp -= 2 * M_PI;

  return temp;
}

LeePositionControllerNostopNode::LeePositionControllerNostopNode()
{
  InitializeParams();

  ros::NodeHandle nh;

  cmd_pose_sub_ = nh.subscribe(
      mav_msgs::default_topics::COMMAND_POSE, 1,
      &LeePositionControllerNostopNode::CommandPoseCallback, this);

  cmd_multi_dof_joint_trajectory_sub_ = nh.subscribe(
      mav_msgs::default_topics::COMMAND_TRAJECTORY, 1,
      &LeePositionControllerNostopNode::MultiDofJointTrajectoryCallback, this);

  odometry_sub_ = nh.subscribe(mav_msgs::default_topics::ODOMETRY, 1,
                               &LeePositionControllerNostopNode::OdometryCallback, this);

  motor_velocity_reference_pub_ = nh.advertise<mav_msgs::Actuators>(
      mav_msgs::default_topics::COMMAND_ACTUATORS, 1);

}

LeePositionControllerNostopNode::~LeePositionControllerNostopNode() {}

void LeePositionControllerNostopNode::InitializeParams() {
  ros::NodeHandle pnh("~");

  // Read parameters from rosparam.
  GetRosParameter(pnh, "position_gain/x",
                  lee_position_controller_.controller_parameters_.position_gain_.x(),
                  &lee_position_controller_.controller_parameters_.position_gain_.x());
  GetRosParameter(pnh, "position_gain/y",
                  lee_position_controller_.controller_parameters_.position_gain_.y(),
                  &lee_position_controller_.controller_parameters_.position_gain_.y());
  GetRosParameter(pnh, "position_gain/z",
                  lee_position_controller_.controller_parameters_.position_gain_.z(),
                  &lee_position_controller_.controller_parameters_.position_gain_.z());
  GetRosParameter(pnh, "velocity_gain/x",
                  lee_position_controller_.controller_parameters_.velocity_gain_.x(),
                  &lee_position_controller_.controller_parameters_.velocity_gain_.x());
  GetRosParameter(pnh, "velocity_gain/y",
                  lee_position_controller_.controller_parameters_.velocity_gain_.y(),
                  &lee_position_controller_.controller_parameters_.velocity_gain_.y());
  GetRosParameter(pnh, "velocity_gain/z",
                  lee_position_controller_.controller_parameters_.velocity_gain_.z(),
                  &lee_position_controller_.controller_parameters_.velocity_gain_.z());
  GetRosParameter(pnh, "attitude_gain/x",
                  lee_position_controller_.controller_parameters_.attitude_gain_.x(),
                  &lee_position_controller_.controller_parameters_.attitude_gain_.x());
  GetRosParameter(pnh, "attitude_gain/y",
                  lee_position_controller_.controller_parameters_.attitude_gain_.y(),
                  &lee_position_controller_.controller_parameters_.attitude_gain_.y());
  GetRosParameter(pnh, "attitude_gain/z",
                  lee_position_controller_.controller_parameters_.attitude_gain_.z(),
                  &lee_position_controller_.controller_parameters_.attitude_gain_.z());
  GetRosParameter(pnh, "angular_rate_gain/x",
                  lee_position_controller_.controller_parameters_.angular_rate_gain_.x(),
                  &lee_position_controller_.controller_parameters_.angular_rate_gain_.x());
  GetRosParameter(pnh, "angular_rate_gain/y",
                  lee_position_controller_.controller_parameters_.angular_rate_gain_.y(),
                  &lee_position_controller_.controller_parameters_.angular_rate_gain_.y());
  GetRosParameter(pnh, "angular_rate_gain/z",
                  lee_position_controller_.controller_parameters_.angular_rate_gain_.z(),
                  &lee_position_controller_.controller_parameters_.angular_rate_gain_.z());
  GetVehicleParameters(pnh, &lee_position_controller_.vehicle_parameters_);
  lee_position_controller_.InitializeParameters();
  waypoint_deviation = 0.2;
  yaw_deviation = 360.0 * M_PI/180.0;
}

void LeePositionControllerNostopNode::Publish() {
}

void LeePositionControllerNostopNode::CommandPoseCallback(
    const geometry_msgs::PoseStampedConstPtr& pose_msg) {
  // Clear all pending commands.
  commands_.clear();

  mav_msgs::EigenTrajectoryPoint eigen_reference;
  mav_msgs::eigenTrajectoryPointFromPoseMsg(*pose_msg, &eigen_reference);
  commands_.push_front(eigen_reference);
  target_state = eigen_reference;

  lee_position_controller_.SetTrajectoryPoint(commands_.front());
  commands_.pop_front();
}

void LeePositionControllerNostopNode::MultiDofJointTrajectoryCallback(
    const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& msg) {
  // Clear all pending commands.
  commands_.clear();

  const size_t n_commands = msg->points.size();

  if(n_commands < 1){
    ROS_WARN_STREAM("Got MultiDOFJointTrajectory message, but message has no points.");
    return;
  }

  mav_msgs::EigenTrajectoryPoint eigen_reference;
  mav_msgs::eigenTrajectoryPointFromMsg(msg->points.front(), &eigen_reference);
  commands_.push_front(eigen_reference);

  for (size_t i = 1; i < n_commands; ++i) {
    //const trajectory_msgs::MultiDOFJointTrajectoryPoint& reference_before = msg->points[i-1];
    const trajectory_msgs::MultiDOFJointTrajectoryPoint& current_reference = msg->points[i];

    mav_msgs::eigenTrajectoryPointFromMsg(current_reference, &eigen_reference);

    commands_.push_back(eigen_reference);

  }

  // We can trigger the first command immediately.
  lee_position_controller_.SetTrajectoryPoint(commands_.front());
  target_state = commands_.front();
  commands_.pop_front();
}

void LeePositionControllerNostopNode::OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg) {

  ROS_INFO_ONCE("nostop_LeePositionController got first odometry message.");

  EigenOdometry odometry;
  eigenOdometryFromMsg(odometry_msg, &odometry);
  lee_position_controller_.SetOdometry(odometry);

  // judge if arrive the current goal position
  Eigen::Vector3d current_position = odometry.position;
  Eigen::Vector3d target_position = target_state.position_W;
  double target_yaw = angluar_normalization(target_state.getYaw());
  Eigen::Matrix3d R = odometry.orientation.toRotationMatrix();
  Eigen::Vector3d euler = R.eulerAngles(0,1,2);
  double current_yaw = angluar_normalization(euler[2]);
  //std::cout<<"current position: "<<current_position[0]<<" "<<current_position[1]<<" "<<current_position[2]<<" "<<current_yaw<<std::endl;
  //std::cout<<"target position: "<<target_position[0]<<" "<<target_position[1]<<" "<<target_position[2]<<" "<<target_yaw<<std::endl;

  Eigen::Vector3d position_error = current_position - target_position;
  if(std::sqrt(position_error.dot(position_error)) < waypoint_deviation && std::abs(current_yaw - target_yaw) < yaw_deviation)
  {
    // update the goal position
    if(!commands_.empty()){
    lee_position_controller_.SetTrajectoryPoint(commands_.front());
    target_state = commands_.front();
    commands_.pop_front();
    }
    else
      ROS_WARN("Commands empty, this should not happen here");
  }

  Eigen::VectorXd ref_rotor_velocities;
  lee_position_controller_.CalculateRotorVelocities(&ref_rotor_velocities);

  // Todo(ffurrer): Do this in the conversions header.
  mav_msgs::ActuatorsPtr actuator_msg(new mav_msgs::Actuators);

  actuator_msg->angular_velocities.clear();
  for (int i = 0; i < ref_rotor_velocities.size(); i++)
    actuator_msg->angular_velocities.push_back(ref_rotor_velocities[i]);
  actuator_msg->header.stamp = odometry_msg->header.stamp;

  motor_velocity_reference_pub_.publish(actuator_msg);
}

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "lee_position_controller_nostop_node");

  rotors_control::LeePositionControllerNostopNode lee_position_controller_nostop_node;

  ros::spin();

  return 0;
}


