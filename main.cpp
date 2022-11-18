#include <iostream>
#include "Planning2D.h"
#include <pybind11/pybind11.h>

int main() {
  double delta_t = 0.4;
  int total_time_step = 5;
  Pose2 start_pose = Pose2(5, 5, 0);
  Vector start_vel = Vector3(0.0, 0.0, 0.0);

  Pose2 end_pose = Pose2(45, 45, 0);
  Vector end_vel = Vector3(0.0, 0.0, 0.0);

  Vector avg_vel = Vector3((end_pose.x() - start_pose.x())/delta_t, (end_pose.y() - start_pose.y())/delta_t, (end_pose.theta() - start_pose.theta())/delta_t);
  vector <Pose2> ps;
  vector <Vector> vs;
  for (int i = 0; i<=5; i++){
      Pose2 pose = Pose2(start_pose.x() * (total_time_step-i)/total_time_step + end_pose.x() * i/total_time_step, start_pose.y() * (total_time_step-i)/total_time_step + end_pose.y() * i/total_time_step, start_pose.theta() * (total_time_step-i)/total_time_step + end_pose.theta() * i/total_time_step);
      ps.push_back(pose);
      vs.push_back(avg_vel);
  }
  Planning2D p2d;
  p2d.buildMap(1,Point2(0,0), 50,50);
  p2d.optimize(ps, vs, delta_t);

  return 0;
}
