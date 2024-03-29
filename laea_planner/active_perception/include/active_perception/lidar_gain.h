
#ifndef LIDAR_GAIN_H
#define LIDAR_GAIN_H

#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <memory>
#include <ros/console.h>

#include <math.h>
#include <vector>
#include <Eigen/Eigen>

#include <tf/tf.h>
#include <visualization_msgs/Marker.h>
#include <plan_env/raycast.h>

// #include <opencv2/opencv.hpp>
// using namespace cv;
using namespace std;
using namespace Eigen;

class Lidar_gain {
private:
  #define ang2rad_rate 0.01745329
  
  double max_range_lidar, max_range_depth, h_fov_depth, subdivide_rate;
  // dt_num = round(dt_temp/map.resolution)
  double dt_temp;
  int dt_num;


  int occupied_thresh = 1;
  int free_thresh = 0;

  nav_msgs::OccupancyGrid OccGrid;
  double yaw;

  Eigen::Matrix<double,2,2> R;
  Eigen::Matrix<double,2,1> T;

  unique_ptr<RayCaster> raycaster;

public:
  Lidar_gain(/* args */) {
  }
  ~Lidar_gain() {
  } 
  bool map_flag;
  void Init();
  void setFov_Params(const double& lidar_range, const double& depth_range, const double& depth_fov_h,const double& divide_rate);
  void Update_Occgrid_data(const nav_msgs::OccupancyGrid::ConstPtr& msg);

  enum OCCUPANCY { UNKNOWN, FREE, OCCUPIED };
  int Get_One_Raycast(Vector3d& ray_start_, Vector3d& ray_end_, vector<Eigen::Vector3d>& ray_visual_);
  int Get_One_Raycast(Vector3d& ray_start_, Vector3d& ray_end_);

  int Get_Multi_Raycast(vector<Eigen::Vector3d> ray_start, vector<Eigen::Vector3d> ray_end);
  int Get_Multi_Raycast(vector<Eigen::Vector3d> ray_start, vector<Eigen::Vector3d> ray_end, vector<Eigen::Vector3d>& ray_visual_);

  bool If_Count_LidarGains(const Vector3i& idx);

  void World2Map_Transform(Vector3d& world_pos, Vector3d& map_pos);
  // int idx
  void Map2World_Transform(Vector3i& map_pos, Vector3d& world_pos);
  void Get_Raycast_Pos(Vector3d pos, double yaw, vector<Eigen::Vector3d>& ray_start, vector<Eigen::Vector3d>& ray_end);
  int Count_Lidar_Gains(Vector3d pos, double yaw);
  int Count_Lidar_Gains_Visualization(Vector3d pos, double yaw, vector<Eigen::Vector3d>& lidar_FOV, 
    vector<Eigen::Vector3d>& lidar_Gains, vector<Eigen::Vector3d>& lidar_raycast);

  inline bool isInMap(const Eigen::Vector3i& idx);
  inline int getOccupancy(const Eigen::Vector3i& id);

};


#endif //ASTAR_H
