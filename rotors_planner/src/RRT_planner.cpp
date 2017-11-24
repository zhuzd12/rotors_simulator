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

// ompl library header
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SO3StateSpace.h>
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

class Plane3DEnvironment
{
public:

    Plane3DEnvironment(const std::string octomap_file)
    {
        bool ok = false;
        try
        {
            //std::istream stream_octo;
            //stream_octo.read(octomap_file.c_str());
            octomap_->readBinary(octomap_file);
            ok = true;
        }
        catch(ompl::Exception &ex)
        {
            OMPL_ERROR("Unable to load %s.\n%s", octomap_file.c_str(), ex.what());
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
            //maxWidth_ = ppm_.getWidth() - 1;
            //maxHeight_ = ppm_.getHeight() - 1;
            //ss_.reset(new og::SimpleSetup(ob::StateSpacePtr(space)));
            //si_ = std::make_shared<ob::SpaceInformation>(space);
            ob::SpaceInformationPtr si(new ob::SpaceInformation(space));

            //std::shared_ptr<ob::SpaceInformationPtr> si(std::make_shared<ob::SpaceInformation>(space));
            si_ = si;
            delete si;
            // set state validity checking for this space
            si_->setStateValidityChecker(std::bind(&Plane3DEnvironment::isStateValid, this, std::placeholders::_1));
            // space->setup();
            si_->setStateValidityCheckingResolution(1.0 / space->getMaximumExtent());
            //ss_->getSpaceInformation()->setStateValidityCheckingResolution(1.0 / space->getMaximumExtent());
            //      ss_->setPlanner(ob::PlannerPtr(new og::RRTConnect(ss_->getSpaceInformation())));
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

        ob::PlannerStatus solved = planner->ob::Planner::solve(1.0);

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

        //ss_->setStartAndGoalStates(start, goal);
        // generate a few solutions; all will be added to the goal;

//        const std::size_t ns = ss_->getProblemDefinition()->getSolutionCount();
//        OMPL_INFORM("Found %d solutions", (int)ns);
//        if (ss_->haveSolutionPath())
//        {
//            ss_->simplifySolution();
//            og::PathGeometric &p = ss_->getSolutionPath();
//            ss_->getPathSimplifier()->simplifyMax(p);
//            ss_->getPathSimplifier()->smoothBSpline(p);
//            return true;
//        }
//        else
//            return false;
//    }

    void getsolution(ob::PathPtr path) const{
      path = pdf_->getSolutionPath();
      return ;

    }

    int getsolutioncount() const {return pdf_->getSolutionCount();}

    void recordSolution()
    {
        if (!si_ || !pdf_->hasSolution())
            return;
//        og::PathGeometric &p = pdf_->getSolutionPath();
//        p.interpolate();
//        for (std::size_t i = 0 ; i < p.getStateCount() ; ++i)
//        {
//            const int w = std::min(maxWidth_, (int)p.getState(i)->as<ob::RealVectorStateSpace::StateType>()->values[0]);
//            const int h = std::min(maxHeight_, (int)p.getState(i)->as<ob::RealVectorStateSpace::StateType>()->values[1]);
//            ompl::PPM::Color &c = ppm_.getPixel(h, w);
//            c.red = 255;
//            c.green = 0;
//            c.blue = 0;
//        }
    }

    void save(const std::string filename)
    {
        if (!si_)
            return;
        octomap_->write(filename);
    }

private:

    bool isStateValid(const ob::ScopedState<> *state) const
    {
//        const int w = std::min((int)state->as<ob::RealVectorStateSpace::StateType>()->values[0], maxWidth_);
//        const int h = std::min((int)state->as<ob::RealVectorStateSpace::StateType>()->values[1], maxHeight_);

//        const ompl::PPM::Color &c = ppm_.getPixel(h, w);
//        return c.red > 127 && c.green > 127 && c.blue > 127;

        octomap::OcTreeNode *state_node = octomap_->search(*state[0], *state[1], *state[2]);
        return !octomap_->isNodeOccupied(state_node);
    }

    //og::SimpleSetupPtr ss_;
    ob::SpaceInformationPtr si_;
    ob::ProblemDefinitionPtr pdf_;
    //int maxWidth_;
    //int maxHeight_;
    //ompl::PPM ppm_;
    octomap::OcTree *octomap_;

};

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


int main(int argc, char **argv)
{
  ros::init(argc, argv, "RRT_planner");
  ros::NodeHandle nh;

  ros::Publisher trajectory_pub =
      nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
      mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);


  std::cout << "OMPL version: " << OMPL_VERSION << std::endl;

  //boost::filesystem::path path("$(find rotors_planner)/octoworld/powerplant.bt");
  std::string octo_file= "../resource/power_plant.bt";
  Plane3DEnvironment env(octo_file);

  if (env.plan(0, 0, 1, 25, 65,45))
  {
      //env.recordSolution();

      env.save("result_demo.bt");
  }

  ob::PathPtr path;
  env.getsolution(path);
  std::vector< ob::State * > state_waypoints =path->as;
  int num_waypoints = 0;
  num_waypoints = env.getsolutioncount();
  ROS_INFO("Start publishing planned trajectory.");

  std::vector<WaypointWithTime> waypoints;
  const float DEG_2_RAD = M_PI / 180.0;
  double temp_t = 5.0;
  for(int i=0;i<num_waypoints;i++){
    //ob::State *current_state = std::make_shared<ob::State>(state_waypoints[i]);
    ob::State current_state = state_waypoints[i];
    waypoints.push_back(WaypointWithTime(temp_t,current_state[0],current_state[1], current_state[2], 0.0 * DEG_2_RAD));
    temp_t = temp_t +5;
  }

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

    time_from_start_ns += static_cast<int64_t>(wp.waiting_time * kNanoSecondsInSecond);

    mav_msgs::msgMultiDofJointTrajectoryPointFromEigen(trajectory_point, &msg->points[i]);
  }
  trajectory_pub.publish(msg);

  ros::spinOnce();
  ros::shutdown();

  return 0;

}
