#include <plan_env/sdf_map.h>
#include <plan_env/map_ros.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <visualization_msgs/Marker.h>

#include <fstream>

#include <opencv2/opencv.hpp>
#include <vector>
using namespace cv;

namespace fast_planner {
MapROS::MapROS() {
}

MapROS::~MapROS() {
}

void MapROS::setMap(SDFMap* map) {
  this->map_ = map;
}

void MapROS::init() {
  node_.param("map_ros/fx", fx_, -1.0);
  node_.param("map_ros/fy", fy_, -1.0);
  node_.param("map_ros/cx", cx_, -1.0);
  node_.param("map_ros/cy", cy_, -1.0);
  node_.param("map_ros/depth_filter_maxdist", depth_filter_maxdist_, -1.0);
  node_.param("map_ros/depth_filter_mindist", depth_filter_mindist_, -1.0);
  node_.param("map_ros/depth_filter_margin", depth_filter_margin_, -1);
  node_.param("map_ros/k_depth_scaling_factor", k_depth_scaling_factor_, -1.0);
  node_.param("map_ros/skip_pixel", skip_pixel_, -1);

  node_.param("map_ros/esdf_slice_height", esdf_slice_height_, -0.1);
  node_.param("map_ros/visualization_truncate_height", visualization_truncate_height_, -0.1);
  node_.param("map_ros/visualization_truncate_low", visualization_truncate_low_, -0.1);
  node_.param("map_ros/show_occ_time", show_occ_time_, false);
  node_.param("map_ros/show_esdf_time", show_esdf_time_, false);
  node_.param("map_ros/show_all_map", show_all_map_, false);
  node_.param("map_ros/frame_id", frame_id_, string("world"));

  // x 180 z -90
  // 0.0  0.707  0.707  0.0 
  node_.param("map_ros/qw", qw_, 0.0);
  node_.param("map_ros/qx", qx_, 0.707);
  node_.param("map_ros/qy", qy_, 0.707);
  node_.param("map_ros/qz", qz_, 0.0);
  node_.param("map_ros/pub_normal_2d", pub_2d_map_normal, false);
  node_.param("map_ros/pub_downsample_2d", pub_2d_map_downsample, false);
  node_.param("map_ros/downsample_2d_resolution", downsample_2d_resolution, 0.15);
  // Limits the area of the connected area to: bounding_small~bounding_big
  node_.param("map_ros/bounding_small", bounding_small, 10);
  node_.param("map_ros/bounding_big", bounding_big, 5000);
  // for testing
  node_.param("map_ros/pub_bbox_marker", pub_bbox_marker, false);
  // Whether to use hybird-map for isolated area detection
  node_.param("map_ros/use_hybird_map", use_hybird_map, false);
  // Whether to use multi-threading for connected region detection
  node_.param("map_ros/use_new_thread", use_new_thread, false);

  proj_points_.resize(640 * 480 / (skip_pixel_ * skip_pixel_));
  point_cloud_.points.resize(640 * 480 / (skip_pixel_ * skip_pixel_));
  // proj_points_.reserve(640 * 480 / map_->mp_->skip_pixel_ / map_->mp_->skip_pixel_);
  proj_points_cnt = 0;

  local_updated_ = false;
  esdf_need_update_ = false;
  fuse_time_ = 0.0;
  esdf_time_ = 0.0;
  max_fuse_time_ = 0.0;
  max_esdf_time_ = 0.0;
  fuse_num_ = 0;
  esdf_num_ = 0;
  depth_image_.reset(new cv::Mat);

  rand_noise_ = normal_distribution<double>(0, 0.1);
  random_device rd;
  eng_ = default_random_engine(rd());

  esdf_timer_ = node_.createTimer(ros::Duration(0.05), &MapROS::updateESDFCallback, this);
  vis_timer_ = node_.createTimer(ros::Duration(0.05), &MapROS::visCallback, this);

  map_all_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/occupancy_all", 10);
  map_local_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/occupancy_local", 10);
  map_local_inflate_pub_ =
      node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/occupancy_local_inflate", 10);
  unknown_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/unknown", 10);
  esdf_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/esdf", 10);
  update_range_pub_ = node_.advertise<visualization_msgs::Marker>("/sdf_map/update_range", 10);
  depth_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/depth_cloud", 10);
  
  marker_bb_pub = node_.advertise<visualization_msgs::Marker>("/sdf_map/boundingbox", 10);

  lidar_map_sub_ = node_.subscribe("/projected_map/cv", 100, &MapROS::Lidar_Occ_MapCallback, this);
  hybird_map_pub = node_.advertise<nav_msgs::OccupancyGrid>("/sdf_map/hybrid_2d", 10);
  node_.param("map_ros/contours_thesh_", contours_thesh_, 50);

  occ_2d_map_pub_ = node_.advertise<nav_msgs::OccupancyGrid>("/sdf_map/project_2d", 10);
  // init
  m_gridmap.info.resolution = map_->getResolution();
  m_gridmap.header.frame_id = frame_id_;
  m_gridmap.info.origin.orientation.w = qw_;
  m_gridmap.info.origin.orientation.x = qx_;
  m_gridmap.info.origin.orientation.y = qy_;
  m_gridmap.info.origin.orientation.z = qz_;

  // origin_q = Eigen::Quaterniond(qw_box, qx_box, qy_box, qz_box);
  origin_q = Eigen::Quaterniond(qw_, qx_, qy_, qz_);
  origin_r = origin_q.toRotationMatrix();
  // std::cout << "origin_q: " << origin_q.w()  << origin_q.x() << origin_q.y() << origin_q.z()<< std::endl;

  // downsample
  occ_2d_map_pub_downsample_ = node_.advertise<nav_msgs::OccupancyGrid>("/sdf_map/project_2d_downsample", 10);
  m_gridmap_downsample.info.resolution = downsample_2d_resolution;
  m_gridmap_downsample.header.frame_id = frame_id_;
  downsample_ratio = m_gridmap.info.resolution / m_gridmap_downsample.info.resolution;


  depth_sub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(node_, "/map_ros/depth", 50));
  cloud_sub_.reset(
      new message_filters::Subscriber<sensor_msgs::PointCloud2>(node_, "/map_ros/cloud", 50));
  pose_sub_.reset(
      new message_filters::Subscriber<geometry_msgs::PoseStamped>(node_, "/map_ros/pose", 25));

  sync_image_pose_.reset(new message_filters::Synchronizer<MapROS::SyncPolicyImagePose>(
      MapROS::SyncPolicyImagePose(100), *depth_sub_, *pose_sub_));
  sync_image_pose_->registerCallback(boost::bind(&MapROS::depthPoseCallback, this, _1, _2));
  sync_cloud_pose_.reset(new message_filters::Synchronizer<MapROS::SyncPolicyCloudPose>(
      MapROS::SyncPolicyCloudPose(100), *cloud_sub_, *pose_sub_));
  sync_cloud_pose_->registerCallback(boost::bind(&MapROS::cloudPoseCallback, this, _1, _2));

  map_start_time_ = ros::Time::now();

  if(use_new_thread){
    t_hybrid = std::thread(&MapROS::new_thread_hybrid, this);
    t_hybrid.detach();
  }

}

// A separate thread that detects the generation of boundingboxes
void MapROS::new_thread_hybrid() {
  
  while (ros::ok() && use_new_thread)
  { 
    nav_msgs::OccupancyGrid hybird_map_;
    vector<Eigen::Vector3d> bounding_box_unknown_;
    hybird_map_.data.clear();
    {
      std::unique_lock<std::mutex> lock(mtx_plan_hybrid);
      condition_.wait(lock, [this] {return hybrid_map_generated; });
      hybird_map_ = hybird_map;
      hybrid_map_generated = false;
    }

    detect_boundingbox(hybird_map_, bounding_box_unknown_);
    {
      std::unique_lock<std::mutex> lock(mtx_plan_hybrid);
      bounding_box_unknown.clear();
      for (size_t i = 0; i < bounding_box_unknown_.size(); i++)
      {
        bounding_box_unknown.push_back(bounding_box_unknown_.at(i));
      }
    }
  }
}

// Sometimes it dies 
void MapROS::detect_boundingbox(nav_msgs::OccupancyGrid& hybird_map_, vector<Eigen::Vector3d>& bounding_box_unknown_){

    if(true){
      int height = hybird_map_.info.height;
      int width = hybird_map_.info.width;
      Eigen::Vector3d origin_hybird{hybird_map_.info.origin.position.x,hybird_map_.info.origin.position.y,0};

      int OccProb;
      Mat Map_hybird(height, width, CV_8UC1);
      for(int i=0; i<height; i++){
        for(int j=0;j<width;j++){
          OccProb = hybird_map_.data[i * width + j];
          OccProb = (OccProb == 100) ? 255 : 0; 
          // OccProb = (OccProb == 1 || OccProb == -1) ? 0 : 255; 

          // The origin of the OccGrid is on the bottom left corner of the map
          Map_hybird.at<uchar>(height-i-1, j) = OccProb;
        }
      }
      Mat _Map = Map_hybird.clone();
      if(_Map.channels() == 3)
        cvtColor(_Map.clone(), _Map, COLOR_BGR2GRAY);
      threshold(_Map.clone(), _Map, 125, 255, cv::THRESH_BINARY);

      std::vector<Vec4i> hierarchy;
      std::vector<std::vector<cv::Point>> contours;
      cv::findContours(_Map, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
      

      if(true)
        bounding_box_unknown_.clear();

      vector<Eigen::Vector3d> centroids_;
      centroids_.clear();

      for (int i = 0; i < contours.size(); i++) {
        if(contours.size()<3)
          break;

      if(hierarchy[i][2] != -1)
        continue;

      if(hierarchy[i][3] == -1)
        continue;
              
        int area_size = cv::contourArea(contours[i]);

        if(area_size > bounding_small && area_size < bounding_big){

          RotatedRect minRect = minAreaRect(contours[i]);
          vector<Point2f> boxPts(4);
          minRect.points(boxPts.data());

          for (size_t j = 0; j < boxPts.size(); j++)
          { 
            Eigen::Vector3d pos_temp_,pos_temp__;

            boxPts.at(j).y = height -1 - boxPts.at(j).y;
            pos_temp_(0) = (boxPts.at(j).x + 0.5) * hybird_map_.info.resolution;
            pos_temp_(1) = (boxPts.at(j).y + 0.5) * hybird_map_.info.resolution;
            
            pos_temp__ = pos_temp_ + origin_hybird;
            pos_temp__(2) = 2;

            if(true)
              bounding_box_unknown_.push_back(pos_temp__);

          }
          
        }
        
      }
    }
  
}


void MapROS::visCallback(const ros::TimerEvent& e) {
  publishMapLocal();
  if (show_all_map_) {
    // Limit the frequency of all map
    static double tpass = 0.0;
    tpass += (e.current_real - e.last_real).toSec();
    if (tpass > 0.1) {
      publishMapAll();
      tpass = 0.0;
    }
  }
  // publishUnknown();
  // publishESDF();

  // publishUpdateRange();
  // publishDepth();
}


void MapROS::Lidar_Occ_MapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    lidar_map = *msg;
    Eigen::Vector3d temp_{0.5,0.5,0}, origin_{lidar_map.info.origin.position.x,lidar_map.info.origin.position.y,0};

    // **************************************** Publish an adaptive-sized raster map ************************************
    Eigen::Vector3d origin_sdf{m_gridmap.info.origin.position.x, m_gridmap.info.origin.position.y, 0};

    Eigen::Vector3d origin_sdf_idx;  
    origin_sdf_idx = (origin_sdf-origin_)/lidar_map.info.resolution + temp_;

    Eigen::Vector3i sdf_origin_idx{floor(origin_sdf_idx(0)), floor(origin_sdf_idx(1)), 0};

    Eigen::Vector3d diagon_sdf_idx; 
    map_->indexToPos(Eigen::Vector3i(map_->mp_->box_max_(0), map_->mp_->box_max_(1), map_->mp_->box_max_(2)), diagon_sdf_idx);
    
    diagon_sdf_idx = (diagon_sdf_idx-origin_)/lidar_map.info.resolution + temp_;

    Eigen::Vector3i sdf_diagon_idx{floor(diagon_sdf_idx(0)), floor(diagon_sdf_idx(1)), 0};

    Eigen::Matrix<int, 4, 1> sdf_idx_wwhh{sdf_origin_idx(0),sdf_diagon_idx(0),sdf_origin_idx(1),sdf_diagon_idx(1)}; 
    Eigen::Matrix<int, 4, 1> idx_wwhh{0,lidar_map.info.width-1,0,lidar_map.info.height-1};   

    Eigen::Matrix<int, 4, 1> bound_constrains{0,0,0,0};   

    if(sdf_idx_wwhh(0) > idx_wwhh(0)){  
      idx_wwhh(0) = sdf_idx_wwhh(0);
      bound_constrains(0) = 1;
    } 
    if(sdf_idx_wwhh(1) < idx_wwhh(1)){   
      idx_wwhh(1) = sdf_idx_wwhh(1);
      bound_constrains(1) = 1;
    } 

    if(sdf_idx_wwhh(2) > idx_wwhh(2)){  
      idx_wwhh(2) = sdf_idx_wwhh(2);
      bound_constrains(2) = 1;
    } 
    if(sdf_idx_wwhh(3) < idx_wwhh(3)){   
      idx_wwhh(3) = sdf_idx_wwhh(3);
      bound_constrains(3) = 1;
    } 

    hybird_map.info = lidar_map.info;
    hybird_map.info.origin.position.x = origin_(0) + idx_wwhh(0)*lidar_map.info.resolution;
    hybird_map.info.origin.position.y = origin_(1) + idx_wwhh(2)*lidar_map.info.resolution;

    hybird_map.info.width = idx_wwhh(1)-idx_wwhh(0) + 1;
    hybird_map.info.height = idx_wwhh(3)-idx_wwhh(2) + 1;

  
    hybird_map.data.clear();
    for (int x = 0; x < hybird_map.info.height; x++)
    {
      for (int y = 0; y < hybird_map.info.width; y++)
      { 
        Eigen::Vector3d pos_in_world;
        Eigen::Vector3d idx{x+idx_wwhh(2), y+idx_wwhh(0), 0};

        // 2d occ -> world
        pos_in_world = origin_r*(idx + temp_)*lidar_map.info.resolution + origin_;
        
        int occ_, occ;
        int occ_idx_1 = (x + idx_wwhh(2)) * lidar_map.info.width + (y + idx_wwhh(0));

        for (double hgt = 0.45; hgt < 1.5; hgt += map_->getResolution())
        {
          pos_in_world(2) =  hgt;
          if(!map_->isInMap(pos_in_world)){
            occ_ = 100;
            break;
          }

          occ = map_->getOccupancy(pos_in_world);
          if(occ == map_->OCCUPIED){
            occ_ = 100;
            break;
          }else if(occ == map_->UNKNOWN) {
            // if
            occ_ = lidar_map.data[occ_idx_1];
            break;
          }else{
            occ_ = 100;
          }
        }
        hybird_map.data.push_back(occ_);
      }
    }

    hybird_map_pub.publish(hybird_map);


    if(use_new_thread){
      {
        std::lock_guard<std::mutex> lock(mtx_plan_hybrid);
        hybrid_map_generated = true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      condition_.notify_one();
    }
   

    if(!use_new_thread){ 
    int height = hybird_map.info.height;
    int width = hybird_map.info.width;
    Eigen::Vector3d origin_hybird{hybird_map.info.origin.position.x,hybird_map.info.origin.position.y,0};
    int OccProb;
    Mat Map_hybird(height, width, CV_8UC1);
    for(int i=0; i<height; i++){
      for(int j=0;j<width;j++){
        OccProb = hybird_map.data[i * width + j];
        
        OccProb = (OccProb == 100) ? 255 : 0; 
        // OccProb = (OccProb == 1 || OccProb == -1) ? 0 : 255; 

        // The origin of the OccGrid is on the bottom left corner of the map
        Map_hybird.at<uchar>(height-i-1, j) = OccProb;
      }
    }

    Mat _Map = Map_hybird.clone();
    if(_Map.channels() == 3)
      cvtColor(_Map.clone(), _Map, COLOR_BGR2GRAY);
    // THRESH_BINARY : occupied -> 1   free&unknown -> 0
    threshold(_Map.clone(), _Map, 125, 255, cv::THRESH_BINARY);

    std::vector<Vec4i> hierarchy;
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(_Map, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    
    if(use_hybird_map)
      bounding_box_unknown.clear();

    vector<Eigen::Vector3d> centroids_;
    centroids_.clear();

    for (int i = 0; i < contours.size(); i++) {
      if(contours.size()<3)
        break;

      if(hierarchy[i][2] != -1)
        continue;

      if(hierarchy[i][3] == -1)
        continue;

      int area_size = cv::contourArea(contours[i]);

      vector<Eigen::Vector3d> boundingbox_;
      boundingbox_.clear();
      if(area_size > bounding_small && area_size < bounding_big){

        RotatedRect minRect = minAreaRect(contours[i]);
        vector<Point2f> boxPts(4);
        minRect.points(boxPts.data());

        for (size_t j = 0; j < boxPts.size(); j++)
        { 
          Eigen::Vector3d pos_temp_,pos_temp__;

          boxPts.at(j).y = height -1 - boxPts.at(j).y;
          pos_temp_(0) = (boxPts.at(j).x + 0.5) * hybird_map.info.resolution;
          pos_temp_(1) = (boxPts.at(j).y + 0.5) * hybird_map.info.resolution;
          
          pos_temp__ = pos_temp_ + origin_hybird;
          pos_temp__(2) = 2;
          boundingbox_.push_back(pos_temp__);

          if(use_hybird_map)
            bounding_box_unknown.push_back(pos_temp__);
        }
        
      }
      
    }

    }

  }


// ok
void MapROS::process_2d_map(const nav_msgs::OccupancyGrid& gridmap_2d){

    m_gridmap_cv = gridmap_2d;
    // Get map
    int height = m_gridmap_cv.info.height;
    int width = m_gridmap_cv.info.width;
    int OccProb, OccProb_temp;
    Mat Map(height, width, CV_8UC1);
    for(int i=0; i<height; i++){
      for(int j=0;j<width;j++){
        OccProb = m_gridmap_cv.data[i * width + j];
        OccProb = (OccProb < 0) ? 0 : 255; 

        // The origin of the OccGrid is on the bottom left corner of the map
        Map.at<uchar>(height-i-1, j) = OccProb;
        // Map.at<uchar>(height-i-1, j) = 255 - round(OccProb * 255.0 / 100.0);
      }
    }

    Mat _Map = Map.clone();
    if(_Map.channels() == 3)
    {
      cvtColor(_Map.clone(), _Map, COLOR_BGR2GRAY);
    } 
    
    
    threshold(_Map.clone(), _Map, 125, 255, cv::THRESH_BINARY_INV);

    std::vector<std::vector<cv::Point>> contours;
    std::vector<Vec4i> hierarchy;
    
    cv::findContours(_Map, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    // std::cout << " Contours_size: " << contours.size() << std::endl;
    bounding_box_unknown.clear();
    for (int i = 0; i < contours.size(); i++) {
      
      if(contours.size()<3)
        break;

      if(hierarchy[i][2] != -1)
        continue;

      int area_size = cv::contourArea(contours[i]);
      
      vector<Eigen::Vector3d> boundingbox_;
      boundingbox_.clear();
      if(area_size > bounding_small && area_size <bounding_big){
        RotatedRect minRect = minAreaRect(contours[i]);
        vector<Point2f> boxPts(4);
        minRect.points(boxPts.data());
        
        for (size_t j = 0; j < boxPts.size(); j++)
        { 
          Eigen::Vector3d pos_temp_,pos_temp__;
          
          boxPts.at(j).y = height -1 - boxPts.at(j).y;
          pos_temp_(0) = (boxPts.at(j).x + 0.5) * m_gridmap_downsample.info.resolution;
          pos_temp_(1) = (boxPts.at(j).y + 0.5) * m_gridmap_downsample.info.resolution;
          
          pos_temp__ =  origin_r*pos_temp_ + pos_origin;
          pos_temp__(2) = 2;
          boundingbox_.push_back(pos_temp__);

          bounding_box_unknown.push_back(pos_temp__);
        }
      }
      if(pub_bbox_marker)
        displaySthWithColor(boundingbox_, 0.15, Eigen::Vector4d(1, 0, 0, 1), i);

    }
  }

void MapROS::updateESDFCallback(const ros::TimerEvent& /*event*/) {
  if (!esdf_need_update_) return;
  auto t1 = ros::Time::now();

  map_->updateESDF3d();
  esdf_need_update_ = false;

  auto t2 = ros::Time::now();
  esdf_time_ += (t2 - t1).toSec();
  max_esdf_time_ = max(max_esdf_time_, (t2 - t1).toSec());
  esdf_num_++;
  if (show_esdf_time_)
    ROS_WARN("ESDF t: cur: %lf, avg: %lf, max: %lf", (t2 - t1).toSec(), esdf_time_ / esdf_num_,
             max_esdf_time_);
}

void MapROS::depthPoseCallback(const sensor_msgs::ImageConstPtr& img,
                               const geometry_msgs::PoseStampedConstPtr& pose) {
  camera_pos_(0) = pose->pose.position.x;
  camera_pos_(1) = pose->pose.position.y;
  camera_pos_(2) = pose->pose.position.z;
  if (!map_->isInMap(camera_pos_))  // exceed mapped region
    return;

  camera_q_ = Eigen::Quaterniond(pose->pose.orientation.w, pose->pose.orientation.x,
                                 pose->pose.orientation.y, pose->pose.orientation.z);
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, img->encoding);
  if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
    (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, k_depth_scaling_factor_);
  cv_ptr->image.copyTo(*depth_image_);

  auto t1 = ros::Time::now();

  // generate point cloud, update map
  proessDepthImage();
  map_->inputPointCloud(point_cloud_, proj_points_cnt, camera_pos_);
  if (local_updated_) {
    map_->clearAndInflateLocalMap();
    esdf_need_update_ = true;
    local_updated_ = false;
  }

  auto t2 = ros::Time::now();
  fuse_time_ += (t2 - t1).toSec();
  max_fuse_time_ = max(max_fuse_time_, (t2 - t1).toSec());
  fuse_num_ += 1;
  if (show_occ_time_)
    ROS_WARN("Fusion t: cur: %lf, avg: %lf, max: %lf", (t2 - t1).toSec(), fuse_time_ / fuse_num_,
             max_fuse_time_);
}

void MapROS::cloudPoseCallback(const sensor_msgs::PointCloud2ConstPtr& msg,
                               const geometry_msgs::PoseStampedConstPtr& pose) {
  camera_pos_(0) = pose->pose.position.x;
  camera_pos_(1) = pose->pose.position.y;
  camera_pos_(2) = pose->pose.position.z;
  camera_q_ = Eigen::Quaterniond(pose->pose.orientation.w, pose->pose.orientation.x,
                                 pose->pose.orientation.y, pose->pose.orientation.z);
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(*msg, cloud);
  int num = cloud.points.size();

  map_->inputPointCloud(cloud, num, camera_pos_);

  if (local_updated_) {
    map_->clearAndInflateLocalMap();
    esdf_need_update_ = true;
    local_updated_ = false;
  }
}

void MapROS::proessDepthImage() {
  proj_points_cnt = 0;

  uint16_t* row_ptr;
  int cols = depth_image_->cols;
  int rows = depth_image_->rows;
  double depth;
  Eigen::Matrix3d camera_r = camera_q_.toRotationMatrix();
  Eigen::Vector3d pt_cur, pt_world;
  const double inv_factor = 1.0 / k_depth_scaling_factor_;

  for (int v = depth_filter_margin_; v < rows - depth_filter_margin_; v += skip_pixel_) {
    row_ptr = depth_image_->ptr<uint16_t>(v) + depth_filter_margin_;
    for (int u = depth_filter_margin_; u < cols - depth_filter_margin_; u += skip_pixel_) {
      depth = (*row_ptr) * inv_factor;
      row_ptr = row_ptr + skip_pixel_;

      // // filter depth
      // if (depth > 0.01)
      //   depth += rand_noise_(eng_);

      // TODO: simplify the logic here
      if (*row_ptr == 0 || depth > depth_filter_maxdist_)
        depth = depth_filter_maxdist_;
      else if (depth < depth_filter_mindist_)
        continue;

      pt_cur(0) = (u - cx_) * depth / fx_;
      pt_cur(1) = (v - cy_) * depth / fy_;
      pt_cur(2) = depth;
      pt_world = camera_r * pt_cur + camera_pos_;
      auto& pt = point_cloud_.points[proj_points_cnt++];
      pt.x = pt_world[0];
      pt.y = pt_world[1];
      pt.z = pt_world[2];
    }
  }

  publishDepth();
}


void MapROS::publishMapAll() {

  map_->indexToPos(Eigen::Vector3i(map_->mp_->box_min_(0), map_->mp_->box_min_(1), map_->mp_->box_min_(2)), pos_origin);
  m_gridmap.info.origin.position.x = pos_origin(0);
  m_gridmap.info.origin.position.y = pos_origin(1);
  m_gridmap.info.origin.position.z = 0;
  
  int height_temp = map_->mp_->box_max_(0)-map_->mp_->box_min_(0);
  int width_temp = map_->mp_->box_max_(1)-map_->mp_->box_min_(1);
  // 这个算是正确的
  m_gridmap.info.height = height_temp;
  m_gridmap.info.width = width_temp;

  uint8_t occ_value;
  int local_idx;
  m_gridmap.data.clear();

  bool exist_occupied_z = false;

  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud1, cloud2;
  for (int x = map_->mp_->box_min_(0) /* + 1 */; x < map_->mp_->box_max_(0); ++x)
    for (int y = map_->mp_->box_min_(1) /* + 1 */; y < map_->mp_->box_max_(1); ++y){
      occ_value = -1;
      exist_occupied_z = false;
      for (int z = map_->mp_->box_min_(2) /* + 1 */; z < map_->mp_->box_max_(2); ++z) {
        Eigen::Vector3d pos;
        map_->indexToPos(Eigen::Vector3i(x, y, z), pos);
        if (pos(2) > visualization_truncate_height_) continue;
        if (pos(2) < visualization_truncate_low_) continue;
        double occ = map_->md_->occupancy_buffer_[map_->toAddress(x, y, z)];
        if (occ > map_->mp_->min_occupancy_log_) {
          occ_value = 100;
          
          if(pos(2) > 0.35)
            exist_occupied_z = true;
          
          pt.x = pos(0);
          pt.y = pos(1);
          pt.z = pos(2);
          cloud1.push_back(pt);
        }
        // box unknown
        else if (occ < map_->mp_->clamp_min_log_ - 1e-3 && !exist_occupied_z) {
          occ_value = -1;
        }
        // box free
        else if(!exist_occupied_z) {
          occ_value = 1;
        }
      }
      m_gridmap.data.push_back(occ_value);
    }

  cloud1.width = cloud1.points.size();
  cloud1.height = 1;
  cloud1.is_dense = true;
  cloud1.header.frame_id = frame_id_;
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud1, cloud_msg);
  map_all_pub_.publish(cloud_msg);

  // Output time and known volumn
  double time_now = (ros::Time::now() - map_start_time_).toSec();
  double known_volumn = 0;

  // for coverage curve
  bool record_known_volumn = true;
  if(record_known_volumn){
  for (int x = map_->mp_->box_min_(0) /* + 1 */; x < map_->mp_->box_max_(0); ++x)
    for (int y = map_->mp_->box_min_(1) /* + 1 */; y < map_->mp_->box_max_(1); ++y)
      for (int z = map_->mp_->box_min_(2) /* + 1 */; z < map_->mp_->box_max_(2); ++z) {
        if (map_->md_->occupancy_buffer_[map_->toAddress(x, y, z)] > map_->mp_->clamp_min_log_ - 1e-3)
          known_volumn += 0.1 * 0.1 * 0.1;
      }
  
  ofstream file("/home/pzz/algorithm/Data/coverage.txt",
                ios::app);
  file << time_now << "," << known_volumn << std::endl;
  }

  if(pub_2d_map_normal){
    m_gridmap.header.stamp = ros::Time::now();
    m_gridmap.header.seq +=1;
    occ_2d_map_pub_.publish(m_gridmap);
  }
  
  // backup for pub occupied only
  if(false){
  for (int x = map_->mp_->box_min_(0) /* + 1 */; x < map_->mp_->box_max_(0); ++x)
    for (int y = map_->mp_->box_min_(1) /* + 1 */; y < map_->mp_->box_max_(1); ++y){
      occ_value = 1;
      local_idx = x * width_temp + y;  // 索引其实没用
      for (int z = map_->mp_->box_min_(2) /* + 1 */; z < map_->mp_->box_max_(2); ++z) {
        // 状态为占用
        if (map_->md_->occupancy_buffer_[map_->toAddress(x, y, z)] > map_->mp_->min_occupancy_log_) {
          Eigen::Vector3d pos;
          map_->indexToPos(Eigen::Vector3i(x, y, z), pos);
          // 限z轴的高
          if (pos(2) > visualization_truncate_height_) continue;
          if (pos(2) < visualization_truncate_low_) continue;
          // if (pos(2) > 0.1) continue;
          // if (pos(2) < 1.5) continue;
          occ_value = 100;
        }
      }
      m_gridmap.data.push_back(occ_value);
    }
  }

}

void MapROS::publishMapLocal() {
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::PointCloud<pcl::PointXYZ> cloud2;
  Eigen::Vector3i min_cut = map_->md_->local_bound_min_;
  Eigen::Vector3i max_cut = map_->md_->local_bound_max_;
  map_->boundIndex(min_cut);
  map_->boundIndex(max_cut);

  // for (int z = min_cut(2); z <= max_cut(2); ++z)
  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = map_->mp_->box_min_(2); z < map_->mp_->box_max_(2); ++z) {
        if (map_->md_->occupancy_buffer_[map_->toAddress(x, y, z)] > map_->mp_->min_occupancy_log_) {
          // Occupied cells
          Eigen::Vector3d pos;
          map_->indexToPos(Eigen::Vector3i(x, y, z), pos);
          if (pos(2) > visualization_truncate_height_) continue;
          if (pos(2) < visualization_truncate_low_) continue;

          pt.x = pos(0);
          pt.y = pos(1);
          pt.z = pos(2);
          cloud.push_back(pt);
        }
        // else if (map_->md_->occupancy_buffer_inflate_[map_->toAddress(x, y, z)] == 1)
        // {
        //   // Inflated occupied cells
        //   Eigen::Vector3d pos;
        //   map_->indexToPos(Eigen::Vector3i(x, y, z), pos);
        //   if (pos(2) > visualization_truncate_height_)
        //     continue;
        //   if (pos(2) < visualization_truncate_low_)
        //     continue;

        //   pt.x = pos(0);
        //   pt.y = pos(1);
        //   pt.z = pos(2);
        //   cloud2.push_back(pt);
        // }
      }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = frame_id_;
  cloud2.width = cloud2.points.size();
  cloud2.height = 1;
  cloud2.is_dense = true;
  cloud2.header.frame_id = frame_id_;
  sensor_msgs::PointCloud2 cloud_msg;

  pcl::toROSMsg(cloud, cloud_msg);
  map_local_pub_.publish(cloud_msg);
  pcl::toROSMsg(cloud2, cloud_msg);
  map_local_inflate_pub_.publish(cloud_msg);
}

void MapROS::publishUnknown() {
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;
  Eigen::Vector3i min_cut = map_->md_->local_bound_min_;
  Eigen::Vector3i max_cut = map_->md_->local_bound_max_;
  map_->boundIndex(max_cut);
  map_->boundIndex(min_cut);

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z) {
        if (map_->md_->occupancy_buffer_[map_->toAddress(x, y, z)] < map_->mp_->clamp_min_log_ - 1e-3) {
          Eigen::Vector3d pos;
          map_->indexToPos(Eigen::Vector3i(x, y, z), pos);
          if (pos(2) > visualization_truncate_height_) continue;
          if (pos(2) < visualization_truncate_low_) continue;
          pt.x = pos(0);
          pt.y = pos(1);
          pt.z = pos(2);
          cloud.push_back(pt);
        }
      }
  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = frame_id_;
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  unknown_pub_.publish(cloud_msg);
}

void MapROS::publishDepth() {
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;
  for (int i = 0; i < proj_points_cnt; ++i) {
    cloud.push_back(point_cloud_.points[i]);
  }
  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = frame_id_;
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  depth_pub_.publish(cloud_msg);
}

void MapROS::publishUpdateRange() {
  Eigen::Vector3d esdf_min_pos, esdf_max_pos, cube_pos, cube_scale;
  visualization_msgs::Marker mk;
  map_->indexToPos(map_->md_->local_bound_min_, esdf_min_pos);
  map_->indexToPos(map_->md_->local_bound_max_, esdf_max_pos);

  cube_pos = 0.5 * (esdf_min_pos + esdf_max_pos);
  cube_scale = esdf_max_pos - esdf_min_pos;
  mk.header.frame_id = frame_id_;
  mk.header.stamp = ros::Time::now();
  mk.type = visualization_msgs::Marker::CUBE;
  mk.action = visualization_msgs::Marker::ADD;
  mk.id = 0;
  mk.pose.position.x = cube_pos(0);
  mk.pose.position.y = cube_pos(1);
  mk.pose.position.z = cube_pos(2);
  mk.scale.x = cube_scale(0);
  mk.scale.y = cube_scale(1);
  mk.scale.z = cube_scale(2);
  mk.color.a = 0.3;
  mk.color.r = 1.0;
  mk.color.g = 0.0;
  mk.color.b = 0.0;
  mk.pose.orientation.w = 1.0;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;

  update_range_pub_.publish(mk);
}

void MapROS::publishESDF() {
  double dist;
  pcl::PointCloud<pcl::PointXYZI> cloud;
  pcl::PointXYZI pt;

  const double min_dist = 0.0;
  const double max_dist = 3.0;

  Eigen::Vector3i min_cut = map_->md_->local_bound_min_ - Eigen::Vector3i(map_->mp_->local_map_margin_,
                                                                          map_->mp_->local_map_margin_,
                                                                          map_->mp_->local_map_margin_);
  Eigen::Vector3i max_cut = map_->md_->local_bound_max_ + Eigen::Vector3i(map_->mp_->local_map_margin_,
                                                                          map_->mp_->local_map_margin_,
                                                                          map_->mp_->local_map_margin_);
  map_->boundIndex(min_cut);
  map_->boundIndex(max_cut);

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y) {
      Eigen::Vector3d pos;
      map_->indexToPos(Eigen::Vector3i(x, y, 1), pos);
      pos(2) = esdf_slice_height_;
      dist = map_->getDistance(pos);
      dist = min(dist, max_dist);
      dist = max(dist, min_dist);
      pt.x = pos(0);
      pt.y = pos(1);
      pt.z = -0.2;
      pt.intensity = (dist - min_dist) / (max_dist - min_dist);
      cloud.push_back(pt);
    }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = frame_id_;
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);

  esdf_pub_.publish(cloud_msg);

  // ROS_INFO("pub esdf");
}

void MapROS::displaySthWithColor(vector<Eigen::Vector3d> path, double resolution, Eigen::Vector4d color,
                          int id) {
  visualization_msgs::Marker mk;
  // mk.header.frame_id = "world";
  mk.header.frame_id = frame_id_;
  mk.header.stamp = ros::Time::now();
  mk.type = visualization_msgs::Marker::LINE_STRIP;
  mk.action = visualization_msgs::Marker::DELETE;
  mk.id = id;
  mk.points.clear();
  marker_bb_pub.publish(mk);

  mk.action = visualization_msgs::Marker::ADD;
  if(path.size()){
    mk.pose.orientation.x = 0.0;
    mk.pose.orientation.y = 0.0;
    mk.pose.orientation.z = 0.0;
    mk.pose.orientation.w = 1.0;
    mk.color.r = color(0);
    mk.color.g = color(1);
    mk.color.b = color(2);
    mk.color.a = color(3);
    mk.scale.x = resolution;
    mk.scale.y = resolution;
    mk.scale.z = resolution;
    
    geometry_msgs::Point pt;
    for (int i = 0; i < int(path.size()); i++) {
      if(i<4){
        pt.x = path[i](0);
        pt.y = path[i](1);
        pt.z = path[i](2);
        mk.points.push_back(pt);
      }
    }
    pt.x = path[0](0);
    pt.y = path[0](1);
    pt.z = path[0](2);
    mk.points.push_back(pt);
    // std::cout << "visual size: " << path.size() << std::endl;
    marker_bb_pub.publish(mk);
    ros::Duration(0.001).sleep();
  }
}

void MapROS::displaySthWithColor_Point(vector<Eigen::Vector3d> path, double resolution, Eigen::Vector4d color,
                          int id) {
  visualization_msgs::Marker mk;
  // mk.header.frame_id = "world";
  mk.header.frame_id = frame_id_;
  mk.header.stamp = ros::Time::now();
  mk.type = visualization_msgs::Marker::SPHERE_LIST;
  mk.action = visualization_msgs::Marker::DELETE;
  mk.id = id;
  mk.points.clear();
  marker_bb_pub.publish(mk);

  if(path.size()){
    mk.action = visualization_msgs::Marker::ADD;
    mk.pose.orientation.x = 0.0;
    mk.pose.orientation.y = 0.0;
    mk.pose.orientation.z = 0.0;
    mk.pose.orientation.w = 1.0;
    mk.color.r = color(0);
    mk.color.g = color(1);
    mk.color.b = color(2);
    mk.color.a = color(3);
    mk.scale.x = resolution;
    mk.scale.y = resolution;
    mk.scale.z = resolution;
    geometry_msgs::Point pt;
    for (int i = 0; i < int(path.size()); i++) {
      pt.x = path[i](0);
      pt.y = path[i](1);
      pt.z = path[i](2);
      mk.points.push_back(pt);
    }
    // std::cout << "visual size: " << path.size() << std::endl;
    marker_bb_pub.publish(mk);
    ros::Duration(0.001).sleep();
  }
}


void MapROS::displaySthWithColor_Contours(vector<Eigen::Vector3d> path, double resolution, Eigen::Vector4d color,
                          int id) {
  visualization_msgs::Marker mk;
  // mk.header.frame_id = "world";
  mk.header.frame_id = frame_id_;
  mk.header.stamp = ros::Time::now();
  mk.type = visualization_msgs::Marker::LINE_STRIP;
  mk.action = visualization_msgs::Marker::DELETE;
  mk.id = id;
  mk.points.clear();
  marker_bb_pub.publish(mk);

  mk.action = visualization_msgs::Marker::ADD;
  if(path.size()){
    mk.pose.orientation.x = 0.0;
    mk.pose.orientation.y = 0.0;
    mk.pose.orientation.z = 0.0;
    mk.pose.orientation.w = 1.0;
    mk.color.r = color(0);
    mk.color.g = color(1);
    mk.color.b = color(2);
    mk.color.a = color(3);
    mk.scale.x = resolution;
    mk.scale.y = resolution;
    mk.scale.z = resolution;
    geometry_msgs::Point pt;
    for (int i = 0; i < int(path.size()); i++) {
      // if(i<4){
        pt.x = path[i](0);
        pt.y = path[i](1);
        pt.z = path[i](2);
        mk.points.push_back(pt);
      // }
    }
    pt.x = path[0](0);
    pt.y = path[0](1);
    pt.z = path[0](2);
    mk.points.push_back(pt);
    // std::cout << "visual size: " << path.size() << std::endl;
    marker_bb_pub.publish(mk);
    ros::Duration(0.001).sleep();
  }
}

bool MapROS::Got_Unknown_Boundingbox_(vector<Eigen::Vector3d>& boundingbox){
  boundingbox.clear();
  int box_num;
  mtx_plan_hybrid.lock();
  box_num = bounding_box_unknown.size();
  mtx_plan_hybrid.unlock();

  if(box_num){
    mtx_plan_hybrid.lock();
    for (size_t i = 0; i < box_num; i++){ 
      boundingbox.push_back(bounding_box_unknown.back());
      bounding_box_unknown.pop_back();
    }
    mtx_plan_hybrid.unlock();

    return true;
  }else
    return false;
}

}
