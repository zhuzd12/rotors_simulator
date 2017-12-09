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
//#include <ompl/base/spaces/SO3StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/config.h>


//#include <fcl/common/detail/profiler.h>
#include <iostream>
#include <cstring>
#include <boost/filesystem.hpp>

namespace ob = ompl::base;
namespace og = ompl::geometric;

static const int64_t kNanoSecondsInSecond = 1000000000;

class WaypointWithTime {
 public:
  WaypointWithTime()
      : waiting_time(0), yaw(0.0) {
  }

  WaypointWithTime(double t, float x, float y, float z, float _yaw)
      : position(x, y, z), yaw(_yaw), waiting_time(t) {
  }

  Eigen::Vector3d position;
  double yaw;
  double waiting_time;
};

template<class TreeType>
  void copy_readTree(TreeType* octree, const octomap_msgs::Octomap& msg){
    std::stringstream datastream;
    if (msg.data.size() > 0){
      datastream.write((const char*) &msg.data[0], msg.data.size());
      octree->readBinaryData(datastream);
    }
  }


class Plane3DEnvironment
{
public:

    ~Plane3DEnvironment(){
        delete octomap_;
    }

    Plane3DEnvironment(const std::string octomap_file)
    {
        bool ok = false;
        try
        {
            //std::istream stream_octo;
            //stream_octo.read(octomap_file.c_str());
            octomap_ = new octomap::OcTree(octomap_file);
            ok = true;
        }
        catch(ompl::Exception &ex)
        {
            ROS_INFO("Unable to load %s.\n%s", octomap_file.c_str(), ex.what());
        }
        if (ok)
        {
            ob::RealVectorStateSpace *space = new ob::RealVectorStateSpace();
            double metricMax_x, metricMax_y, metricMax_z;
            double metricMin_x, metricMin_y, metricMin_z;
            octomap_->getMetricMax(metricMin_x, metricMax_y, metricMax_z);
            octomap_->getMetricMin(metricMin_x, metricMin_y, metricMin_z);
            space->addDimension(metricMin_x,metricMax_x);
            space->addDimension(metricMin_y,metricMax_y);
            space->addDimension(metricMin_z,metricMax_z);
            //ss_.reset(new og::SimpleSetup(ob::StateSpacePtr(space)));
            //si_ = std::make_shared<ob::SpaceInformation>(space);
            //ob::SpaceInformationPtr si(new ob::SpaceInformation(space));
            //std::shared_ptr<ob::SpaceInformationPtr> si(std::make_shared<ob::SpaceInformation>(space));

            //si_ = si;

            si_.reset(new ob::SpaceInformation(ob::StateSpacePtr(space)));
            // set state validity checking for this space
            si_->setStateValidityChecker(std::bind(&Plane3DEnvironment::isStateValid, this, std::placeholders::_1));
            // space->setup();
            si_->setStateValidityCheckingResolution(1.0 / space->getMaximumExtent());
            //ss_->getSpaceInformation()->setStateValidityCheckingResolution(1.0 / space->getMaximumExtent());
        }
    }

    Plane3DEnvironment(const octomap_msgs::Octomap octomap_message)
    {
        bool ok = false;
        try
        {

           // octomap_ = (octomap::OcTree*)octomap_msgs::msgToMap(octomap_message);
            octomap_ = new octomap::OcTree(octomap_message.resolution);
            copy_readTree(octomap_, octomap_message);
            ok = true;
        }
        catch(ompl::Exception &ex)
        {
            OMPL_ERROR("Unable to get octomap .\n%s", ex.what());
        }
        if (ok)
        {
            ob::RealVectorStateSpace *space = new ob::RealVectorStateSpace();
            double metricMax_x, metricMax_y, metricMax_z;
            double metricMin_x, metricMin_y, metricMin_z;
            octomap_->getMetricMax(metricMin_x, metricMax_y, metricMax_z);
            octomap_->getMetricMin(metricMin_x, metricMin_y, metricMin_z);
            space->addDimension(metricMin_x,metricMax_x);
            space->addDimension(metricMin_y,metricMax_y);
            space->addDimension(metricMin_z,metricMax_z);
            //ss_.reset(new og::SimpleSetup(ob::StateSpacePtr(space)));
            //si_ = std::make_shared<ob::SpaceInformation>(space);
            //ob::SpaceInformationPtr si(new ob::SpaceInformation(space));
            //std::shared_ptr<ob::SpaceInformationPtr> si(std::make_shared<ob::SpaceInformation>(space));

            //si_ = si;

            si_.reset(new ob::SpaceInformation(ob::StateSpacePtr(space)));
            // set state validity checking for this space
            si_->setStateValidityChecker(std::bind(&Plane3DEnvironment::isStateValid, this, std::placeholders::_1));
            // space->setup();
            si_->setStateValidityCheckingResolution(1.0 / space->getMaximumExtent());
            //ss_->getSpaceInformation()->setStateValidityCheckingResolution(1.0 / space->getMaximumExtent());
        }
    }

    bool plan(unsigned int start_x, unsigned int start_y, unsigned int start_z, unsigned int goal_x, unsigned int goal_y, unsigned  int goal_z)
    {
        if (!si_)
            return false;
        ob::ScopedState<> start(si_->getStateSpace());
        start[0] = start_x;
        start[1] = start_y;
        start[2] = start_z;
        ob::ScopedState<> goal(si_->getStateSpace());
        goal[0] = goal_x;
        goal[1] = goal_y;
        goal[2] = goal_z;

        pdf_ = std::make_shared<ob::ProblemDefinition>(si_);
        pdf_->setStartAndGoalStates(start, goal);
        auto planner(std::make_shared<og::RRT>(si_));
        planner->setProblemDefinition(pdf_);
        planner->setup();

        ob::PlannerStatus solved = planner->ob::Planner::solve(10.0);

        if (solved)
            {
                // get the goal representation from the problem definition (not the same as the goal state)
                // and inquire about the found path
               // ob::PathPtr path = pdf_->getSolutionPath();
                std::cout << "Found solution:" << std::endl;
                // print the path to screen
               // path->print(std::cout);
                return true;
            }
        else
        {
          return false;
        }
    }



    void getsolution(ob::PathPtr path) const{
      path = pdf_->getSolutionPath();
      return ;

    }

    int getsolutioncount() const {return pdf_->getSolutionCount();}

    void recordSolution()
    {
        if (!si_ || !pdf_->hasSolution())
            return;
        og::PathGeometric &p = *(pdf_->getSolutionPath()->as<og::PathGeometric>());
       // std::static_pointer_cast<og::PathGeometric> p = pdf_->getSolutionPath();
        p.interpolate();
        double temp_t = 5.0;
        const float DEG_2_RAD = M_PI / 180.0;
        for (std::size_t i = 0 ; i < p.getStateCount() ; ++i)
              {
                  const double x = (double)p.getState(i)->as<ob::RealVectorStateSpace::StateType>()->values[0];
                  const double y = (double)p.getState(i)->as<ob::RealVectorStateSpace::StateType>()->values[1];
                  const double z = (double)p.getState(i)->as<ob::RealVectorStateSpace::StateType>()->values[2];

                  waypoints.push_back(WaypointWithTime(temp_t, x, y, z, 0.0 * DEG_2_RAD));
                  temp_t = temp_t +1;
              }

    }

    void save(const std::string filename)
    {
        if (!si_)
            return;
        octomap_->writeBinary(filename);
    }

private:

    bool isStateValid(const ob::State *state) const
    {
        const double x = (double)state->as<ob::RealVectorStateSpace::StateType>()->values[0];
        const double y = (double)state->as<ob::RealVectorStateSpace::StateType>()->values[1];
        const double z = (double)state->as<ob::RealVectorStateSpace::StateType>()->values[2];
        octomap::OcTreeNode *state_node = octomap_->search(x, y, z);
        return !octomap_->isNodeOccupied(state_node);
    }

    ob::SpaceInformationPtr si_;
    ob::ProblemDefinitionPtr pdf_;
    octomap::OcTree *octomap_;

public:
    std::vector<WaypointWithTime> waypoints;
};

bool sim_running = false;

void callback(const sensor_msgs::ImuPtr& msg) {
  sim_running = true;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "RRT_planner");
  ros::NodeHandle nh;

  ros::Publisher trajectory_pub =
  nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
  mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);

  ros::Publisher octomap_pub = nh.advertise<octomap_msgs::Octomap>("/octo_test", 10);

  // The IMU is used, to determine if the simulator is running or not.
  ros::Subscriber sub = nh.subscribe("imu", 10, &callback);

  // The octomap src client
  ros::ServiceClient octo_client = nh.serviceClient<rotors_comm::Octomap>("/world/get_octomap");
  rotors_comm::Octomap octo_src;
  octo_src.request.bounding_box_lengths.x = 60 ;
  octo_src.request.bounding_box_lengths.y = 60 ;
  octo_src.request.bounding_box_lengths.z = 60 ;
  octo_src.request.bounding_box_origin.x = 20;
  octo_src.request.bounding_box_origin.y = 20;
  octo_src.request.bounding_box_origin.z = 20;
  octo_src.request.publish_octomap = true;
  octo_src.request.leaf_size = 0.25;
  octo_src.request.filename = "powerplant";  // wait for generic map name
  if(octo_client.call(octo_src)){
    ROS_INFO("receive octomap %s",octo_src.request.filename.c_str());
  }
  else{
    ROS_INFO("Failed to receive octomap");
    //return 1;
  }

  ros::V_string args;
  //ros::removeROSArgs(argc, argv, args);


  ROS_INFO("Wait for simulation to become ready...");

  while (!sim_running && ros::ok()) {
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }

  ROS_INFO("basic RRT planner begins");

  // Wait for 30s such that everything can settle and the mav flies to the initial position.
  ros::Duration(10).sleep();

  std::cout << "OMPL version: " << OMPL_VERSION << std::endl;

  //boost::filesystem::path path("$(find rotors_planner)/octoworld/powerplant.bt");
  std::string octo_file ;

  if (argc == 2 ) {
    //ROS_ERROR("");
    //return -1;
    octo_file = argv[1];
    ROS_INFO("read octomap file: %s", argv[1]);
  }

   Plane3DEnvironment env(octo_file);
  //Plane3DEnvironment env(octo_src.response.map);
    ROS_INFO("instantiate planner");

  if (env.plan(0, 0, 1, 0, 1, 7))
  {
      //env.recordSolution();

      env.save("result_demo.bt");
  }
  else {
    ROS_INFO("failed to get solution");
    return 2;
  }
  ob::PathPtr path;
  env.getsolution(path);
  ROS_INFO("Start publishing trajectory_pub planned trajectory.");

  env.recordSolution();
  std::vector<WaypointWithTime> waypoints = env.waypoints;
  ROS_INFO("Read %d waypoints.", (int) waypoints.size());
  trajectory_msgs::MultiDOFJointTrajectoryPtr msg(new trajectory_msgs::MultiDOFJointTrajectory);
  msg->header.stamp = ros::Time::now();
  msg->points.resize(waypoints.size());
  msg->joint_names.push_back("base_link");
  int64_t time_from_start_ns = 0;
  for (size_t i = 0; i < waypoints.size(); ++i) {
    WaypointWithTime& wp = waypoints[i];

    mav_msgs::EigenTrajectoryPoint trajectory_point;
    trajectory_point.position_W = wp.position;
    trajectory_point.setFromYaw(wp.yaw);
    trajectory_point.time_from_start_ns = time_from_start_ns;

    std::cout << "Waypoint XYZ: " << wp.position(0) << " "
              <<  wp.position(1) <<  " " <<  wp.position(2)  << std::endl;

    time_from_start_ns += static_cast<int64_t>(wp.waiting_time * kNanoSecondsInSecond);

    mav_msgs::msgMultiDofJointTrajectoryPointFromEigen(trajectory_point, &msg->points[i]);
  }

//  ros::Rate rate(10);
//      while(ros::ok()){
//        trajectory_pub.publish(msg);
//        rate.sleep();
//      }

  trajectory_pub.publish(msg);

  ros::spinOnce();

    octomap::OcTree *octomap_ = new octomap::OcTree(octo_file);
    octomap_msgs::Octomap octo_msg;
    octo_msg.header.frame_id = "/world";
    octomap_msgs::binaryMapToMsg(*octomap_,octo_msg);

    ros::Rate rate(10);
    while(ros::ok()){
      octomap_pub.publish(octo_msg);
      rate.sleep();
    }
    octomap_->read(octo_file);

  ros::shutdown();

  return 0;

}
