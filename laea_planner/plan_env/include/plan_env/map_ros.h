#ifndef _MAP_ROS_H
#define _MAP_ROS_H

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>
#include <pcl_conversions/pcl_conversions.h>

#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

#include <memory>
#include <random>

#include <nav_msgs/OccupancyGrid.h>

#include <mutex>
#include <thread>
#include <condition_variable>

using std::shared_ptr;
using std::normal_distribution;
using std::default_random_engine;

namespace fast_planner {
class SDFMap;

class MapROS {
public:
  MapROS();
  ~MapROS();
  void setMap(SDFMap* map);
  void init(); 
  // get bounding_box_unknown
  bool Got_Unknown_Boundingbox_(vector<Eigen::Vector3d>& boundingbox);

  ros::Subscriber lidar_map_sub_;
  void Lidar_Occ_MapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);


private:
  void depthPoseCallback(const sensor_msgs::ImageConstPtr& img,
                         const geometry_msgs::PoseStampedConstPtr& pose);
  void cloudPoseCallback(const sensor_msgs::PointCloud2ConstPtr& msg,
                         const geometry_msgs::PoseStampedConstPtr& pose);
  void updateESDFCallback(const ros::TimerEvent& /*event*/);
  void visCallback(const ros::TimerEvent& /*event*/);

  void publishMapAll();
  void publishMapLocal();
  void publishESDF();
  void publishUpdateRange();
  void publishUnknown();
  void publishDepth();

  void proessDepthImage();

  void process_2d_map(const nav_msgs::OccupancyGrid& gridmap_2d);

  void displaySthWithColor(vector<Eigen::Vector3d> path, double resolution, Eigen::Vector4d color,
                          int id);
  void displaySthWithColor_Contours(vector<Eigen::Vector3d> path, double resolution, Eigen::Vector4d color,
                          int id);

  void displaySthWithColor_Point(vector<Eigen::Vector3d> path, double resolution, Eigen::Vector4d color,
                          int id);
  void quaternionTrans(Eigen::Vector3d p1,Eigen::Vector3d &p2);
  

  SDFMap* map_;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, geometry_msgs::PoseStamped>
      SyncPolicyImagePose;
  typedef shared_ptr<message_filters::Synchronizer<SyncPolicyImagePose>> SynchronizerImagePose;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2,
                                                          geometry_msgs::PoseStamped>
      SyncPolicyCloudPose;
  typedef shared_ptr<message_filters::Synchronizer<SyncPolicyCloudPose>> SynchronizerCloudPose;

  ros::NodeHandle node_;
  shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> depth_sub_;
  shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> cloud_sub_;
  shared_ptr<message_filters::Subscriber<geometry_msgs::PoseStamped>> pose_sub_;
  SynchronizerImagePose sync_image_pose_;
  SynchronizerCloudPose sync_cloud_pose_;

  ros::Publisher map_local_pub_, map_local_inflate_pub_, esdf_pub_, map_all_pub_, unknown_pub_,
      update_range_pub_, depth_pub_;
  ros::Publisher occ_2d_map_pub_,occ_2d_map_pub_downsample_;
  ros::Timer esdf_timer_, vis_timer_;

  Eigen::Vector3d pos_origin;
  Eigen::Quaterniond origin_q;
  Eigen::Matrix3d origin_r;



  double qw_box,qx_box,qy_box,qz_box;
  vector<Eigen::Vector3d> bounding_box_unknown;

  nav_msgs::OccupancyGrid m_gridmap;
  nav_msgs::OccupancyGrid m_gridmap_downsample;
  nav_msgs::OccupancyGrid m_gridmap_cv;

  // lidar_map
  nav_msgs::OccupancyGrid lidar_map, hybird_map;
  ros::Publisher hybird_map_pub;
  int contours_thesh_;

  ros::Publisher marker_bb_pub;
  
  double qw_,qx_,qy_,qz_;
  double downsample_ratio;
  bool pub_2d_map_normal;
  bool pub_2d_map_downsample;
  bool pub_bbox_marker;
  double downsample_2d_resolution;
  int bounding_small,bounding_big;
  bool use_hybird_map;

  std::thread t_hybrid;
  std::mutex mtx_plan_hybrid;
  void new_thread_hybrid(); 
  std::condition_variable condition_;  
  bool hybrid_map_generated = false;  

  bool use_new_thread = false;

  bool generate_hybrid_map = false;
  bool detect_hybrid_map = true;
  void detect_boundingbox(nav_msgs::OccupancyGrid& hybird_map_, vector<Eigen::Vector3d>& bounding_box_unknown_);

  double cx_, cy_, fx_, fy_;
  double depth_filter_maxdist_, depth_filter_mindist_;
  int depth_filter_margin_;
  double k_depth_scaling_factor_;
  int skip_pixel_;
  string frame_id_;
  // msg publication
  double esdf_slice_height_;
  double visualization_truncate_height_, visualization_truncate_low_;
  bool show_esdf_time_, show_occ_time_;
  bool show_all_map_;

  // data
  // flags of map state
  bool local_updated_, esdf_need_update_;
  // input
  Eigen::Vector3d camera_pos_;
  Eigen::Quaterniond camera_q_;
  unique_ptr<cv::Mat> depth_image_;
  vector<Eigen::Vector3d> proj_points_;
  int proj_points_cnt;
  double fuse_time_, esdf_time_, max_fuse_time_, max_esdf_time_;
  int fuse_num_, esdf_num_;
  pcl::PointCloud<pcl::PointXYZ> point_cloud_;

  normal_distribution<double> rand_noise_;
  default_random_engine eng_;

  ros::Time map_start_time_;

  friend SDFMap;
};
}

#endif