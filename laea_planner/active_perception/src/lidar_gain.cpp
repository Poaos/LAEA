#include <iostream>
#include <active_perception/lidar_gain.h>
#include <Eigen/Eigen>

// frontier_finder  init   // no use
void Lidar_gain::setFov_Params(const double& lidar_range, const double& depth_range, const double& depth_fov_h,const double& divide_rate){
    
    raycaster.reset(new RayCaster);
    max_range_lidar = lidar_range;
    max_range_depth = depth_range;
    h_fov_depth = depth_fov_h;
    subdivide_rate = divide_rate;

    dt_temp = h_fov_depth*ang2rad_rate*max_range_depth/subdivide_rate;
}

void Lidar_gain::Init(){
    raycaster.reset(new RayCaster);
}

void Lidar_gain::Update_Occgrid_data(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    bool print_tag = false;
    if(OccGrid.info.origin.position != msg->info.origin.position){
        print_tag = true;
    }
    // std::cout << "lidar_gain Got map " << std::endl;
    OccGrid.info.resolution = msg->info.resolution;
    OccGrid.info.height = msg->info.height;
    OccGrid.info.width = msg->info.width;
    // origin
    OccGrid.info.origin.orientation = msg->info.origin.orientation;
    OccGrid.info.origin.position = msg->info.origin.position;
    // data
    OccGrid.data = msg->data;
    
    double roll, pitch, yaw_;
    geometry_msgs::Quaternion q = OccGrid.info.origin.orientation;
    tf::Quaternion quat(q.x, q.y, q.z, q.w); // x, y, z, w
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    yaw = yaw_;

    // Calculate R, t
    R(0, 0) = OccGrid.info.resolution*cos(yaw);
    R(0, 1) = OccGrid.info.resolution*sin(-yaw);
    R(1, 0) = OccGrid.info.resolution*sin(yaw);
    R(1, 1) = OccGrid.info.resolution*cos(yaw);

    T(0,0) = OccGrid.info.origin.position.x;
    T(1,0) = OccGrid.info.origin.position.y;

    // dt_num = round(dt_temp/OccGrid.info.resolution);
    
    print_tag = false;
    if(print_tag){
        std::cout << "***** Update OccGrid Info Below *****" << std::endl; 
        std::cout << "Resolution: " << OccGrid.info.resolution << "\tHeight: " 
            << OccGrid.info.height << "\tWidth: " << OccGrid.info.width << std::endl;
        std::cout << "Rotation: " << R(0,0) <<"\t" << R(0,1) <<"\t" 
            << R(1,0) << "\t"<< R(1,1) << std::endl;
        std::cout << "Translation: " << T(0,0) <<"\t" << T(1,0) << std::endl;
        // std::cout << "dt_Temp: "<< dt_temp << " dt_num: " << dt_num << std::endl;
        print_tag = false;
    }

    // Set flag
    map_flag = true;

    // update raycaster
    Vector3d origin_(OccGrid.info.origin.position.x,OccGrid.info.origin.position.y,OccGrid.info.origin.position.z);
    raycaster->setParams(OccGrid.info.resolution, origin_);
}

void Lidar_gain::World2Map_Transform(Vector3d& world_pos, Vector3d& map_pos){
    Eigen::MatrixXd world_(2, 1);
    world_(0,0) = world_pos[0];
    world_(1,0) = world_pos[1];

    Eigen::MatrixXd map_temp(2, 1);
    map_temp = (R.inverse())*(world_ - T);

    map_pos[0] = round(map_temp(0,0));
    map_pos[1] = round(map_temp(1,0));
}

void Lidar_gain::Map2World_Transform(Vector3i& map_pos, Vector3d& world_pos){
    Eigen::MatrixXd map_(2, 1);
    map_(0,0) = map_pos[0];
    map_(1,0) = map_pos[1];

    Eigen::MatrixXd world_temp(2, 1);
    world_temp = R*map_ + T;

    world_pos[0] = world_temp(0,0);
    world_pos[1] = world_temp(1,0);

}


int Lidar_gain::Count_Lidar_Gains_Visualization(Vector3d pos, double yaw, vector<Eigen::Vector3d>& lidar_FOV, 
    vector<Eigen::Vector3d>& lidar_Gains, vector<Eigen::Vector3d>& lidar_raycast){
    
    dt_num = round(dt_temp/OccGrid.info.resolution);
    
    vector<Eigen::Vector3d> ray_0,ray_1;
    Get_Raycast_Pos(pos,yaw,ray_0,ray_1);

    lidar_FOV.clear();
    lidar_Gains.clear();
    lidar_raycast.clear();

    int count = 0, d_count = 0;
    // std::cout << "Ray_size " << ray_0.size();

    while (ray_0.size())
    {
        Vector3d ray0 = ray_0.back();
        Vector3d ray1 = ray_1.back();
        lidar_FOV.push_back(ray0);
        lidar_FOV.push_back(ray1);
        ray_0.pop_back();
        ray_1.pop_back();

        raycaster->input(ray0,ray1);
        // int 
        Vector3i idx;
        Eigen::Vector3d pos_;

        d_count = 0;
        while (raycaster->next_Pos_idx(pos_,idx)){
            if(getOccupancy(idx)==OCCUPIED || getOccupancy(idx)==-1){
                // std::cout << "Break " << std::endl;
                break;
            }
            d_count ++;
            if(d_count%3==2){
                lidar_Gains.push_back(pos_);
            }
        }
        count = count + d_count;

        // just for lidar_raycast 
        raycaster->input(ray0,ray1);
        while (raycaster->nextId(idx)){
            Eigen::Vector3d pos_;
            if(d_count%7==6){
                Map2World_Transform(idx,pos_);
                lidar_raycast.push_back(pos_);
            }
        }
    }
    std::cout << " All Counts " << count << std::endl;
    return count;
}


int Lidar_gain::Get_One_Raycast(Vector3d& ray_start_, Vector3d& ray_end_, vector<Eigen::Vector3d>& ray_visual_){

    int d_count = 0;
    ray_visual_.clear();
    raycaster->input(ray_start_,ray_end_);
    ray_visual_.push_back(ray_start_);
    ray_visual_.push_back(ray_end_);
    Vector3i idx;
    Vector3d pos_;
    d_count = 0;
    
    while (raycaster->next_Pos_idx(pos_,idx)){
        if(getOccupancy(idx)==OCCUPIED || getOccupancy(idx)==-1){
            // std::cout << "Break " << std::endl;
            break;
        }
        d_count ++;

        if(d_count%3==2){
            ray_visual_.push_back(pos_);
        }
    }

    return d_count;
}

int Lidar_gain::Get_One_Raycast(Vector3d& ray_start_, Vector3d& ray_end_){
    
    int d_count = 0;
    raycaster->input(ray_start_,ray_end_);

    Vector3i idx;
    Vector3d pos_;
    d_count = 0;
    while (raycaster->next_Pos_idx(pos_,idx)){
        if(getOccupancy(idx)==OCCUPIED || getOccupancy(idx)==-1){
            // std::cout << "Break " << std::endl;
            break;
        }
        d_count ++;
        
    }
    // std::cout << " dCounts " << d_count << std::endl;
    return d_count;
}

// 
bool Lidar_gain::If_Count_LidarGains(const Vector3i& idx){

    if(getOccupancy(idx)==OCCUPIED || getOccupancy(idx)==-1){
        return 0;
    }else
        return 1;

}

// rostopic echo /project_map
// -1 unknown | 0 free | 100 occupied
inline int Lidar_gain::getOccupancy(const Eigen::Vector3i& id) {
    if (!isInMap(id)) return -1;
    // idex = y*width + x
    int idex = id[1]*OccGrid.info.width + id[0];
    int occ = OccGrid.data[idex];
    // clamp_min_log_ p_min_ 0.12
    if (occ < free_thresh) return UNKNOWN;
    // min_occupancy_log_  p_occ_  0.80
    if (occ > occupied_thresh) return OCCUPIED;
    return FREE;
}

inline bool Lidar_gain::isInMap(const Eigen::Vector3i& idx) {
    if (idx(0) < 0 || idx(1) < 0 || idx(2) < 0) return false;
    if (idx(0) > OccGrid.info.width - 1 || idx(1) > OccGrid.info.height - 1)
        return false;
    return true;
}


int Lidar_gain::Get_Multi_Raycast(vector<Eigen::Vector3d> ray_start, vector<Eigen::Vector3d> ray_end){

    int count = 0, d_count = 0;
    while (ray_start.size())
    {
        Vector3d ray0 = ray_start.back();
        Vector3d ray1 = ray_end.back();
        ray_start.pop_back();
        ray_end.pop_back();

        ray0[2] = 0; ray1[2] = 0;
        

        raycaster->input(ray0,ray1);
        // int 
        Vector3i idx;
        d_count = 0;
        while (raycaster->nextId(idx)){
            if(getOccupancy(idx)==OCCUPIED || getOccupancy(idx)==-1){
                // std::cout << "Break " << std::endl;
                break;
            }
            d_count ++;
        }
        count = count + d_count;
        // std::cout << "dCounts " << d_count << std::endl;
        // std::cout << "raycast_gains " << raycast_gains.size() << std::endl;
    }
    std::cout << " All Counts " << count << std::endl;
    return count;
}

int Lidar_gain::Get_Multi_Raycast(vector<Eigen::Vector3d> ray_start, vector<Eigen::Vector3d> ray_end, vector<Eigen::Vector3d>& ray_visual_){
    
    ray_visual_.clear();
    int count = 0, d_count = 0;
    while (ray_start.size())
    {
        Vector3d ray0 = ray_start.back();
        Vector3d ray1 = ray_end.back();
        ray_start.pop_back();
        ray_end.pop_back();
        ray0[2] = 0; ray1[2] = 0;

        raycaster->input(ray0,ray1);
        // int 
        Vector3i idx;
        Vector3d pos_;
        d_count = 0;
        while (raycaster->next_Pos_idx(pos_,idx)){
            if(getOccupancy(idx)==OCCUPIED || getOccupancy(idx)==-1){
                // std::cout << "Break " << std::endl;
                break;
            }
            d_count ++;
            if(d_count%8==7)
                ray_visual_.push_back(pos_);
        }
        count = count + d_count;
       
    }
    // std::cout << " All Counts " << count << std::endl;
    return count;
}


// ************************ old **************************
void Lidar_gain::Get_Raycast_Pos(Vector3d pos, double yaw, vector<Eigen::Vector3d>& ray_start, vector<Eigen::Vector3d>& ray_end){
    
    double d_fov = h_fov_depth*ang2rad_rate/dt_num;
    vector<double> yaws;
    double yaw_start = yaw - h_fov_depth*ang2rad_rate/2;
    
    double temp_x, temp_y, temp_yaw;
    Eigen::Matrix<double,2,1> start_temp,end_temp;
    for (int i = 0; i <= dt_num; i++){
        temp_yaw = yaw_start+i*d_fov;

        temp_x = sin(temp_yaw);
        temp_y = cos(temp_yaw);

        start_temp(0,0) = temp_x*max_range_depth + pos[0];
        start_temp(1,0) = temp_y*max_range_depth + pos[1];
        end_temp(0,0) = temp_x*max_range_lidar + pos[0];
        end_temp(1,0) = temp_y*max_range_lidar + pos[1];

        ray_start.push_back(Eigen::Vector3d(start_temp(0,0),start_temp(1,0),0));
        ray_end.push_back(Eigen::Vector3d(end_temp(0,0),end_temp(1,0),0));
    }
    // std::cout << "***** Update Ray_start&Ray_end *****" << std::endl; 
    
}

int Lidar_gain::Count_Lidar_Gains(Vector3d pos, double yaw){
    
    vector<Eigen::Vector3d> ray_0,ray_1;
    Get_Raycast_Pos(pos,yaw,ray_0,ray_1);

    int count = 0, d_count = 0;
    // std::cout << "Ray_size " << ray_0.size();
    while (ray_0.size())
    {
        Vector3d ray0 = ray_0.back();
        Vector3d ray1 = ray_1.back();
        ray_0.pop_back();
        ray_1.pop_back();

        raycaster->input(ray0,ray1);
        // int 
        Vector3i idx;
        d_count = 0;
        while (raycaster->nextId(idx)){
            if(getOccupancy(idx)==OCCUPIED || getOccupancy(idx)==-1){
                // std::cout << "Break " << std::endl;
                break;
            }
            d_count ++;
            
        }
        count = count + d_count;
        // std::cout << "dCounts " << d_count << std::endl;
        // std::cout << "raycast_gains " << raycast_gains.size() << std::endl;
    }
    std::cout << " All Counts " << count << std::endl;
    return count;
}

