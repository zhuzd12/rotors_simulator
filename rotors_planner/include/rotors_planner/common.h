#ifndef COMMON_H
#define COMMON_H

#include <Eigen/Dense>

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

#endif // COMMON_H
