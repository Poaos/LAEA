#include <path_searching/astar2.h>
#include <sstream>
#include <plan_env/sdf_map.h>

#include <cmath>

using namespace std;
using namespace Eigen;

namespace fast_planner
{
  Astar::Astar()
  {
  }

  Astar::~Astar()
  {
    for (int i = 0; i < allocate_num_; i++)
      delete path_node_pool_[i];
  }

  void Astar::init(ros::NodeHandle &nh, const EDTEnvironment::Ptr &env)
  {
    nh.param("astar/resolution_astar", resolution_, -1.0);
    nh.param("astar/lambda_heu", lambda_heu_, -1.0);
    nh.param("astar/max_search_time", max_search_time_, -1.0);
    nh.param("astar/allocate_num", allocate_num_, -1);

    tie_breaker_ = 1.0 + 1.0 / 1000;

    this->edt_env_ = env;

    /* ---------- map params ---------- */
    this->inv_resolution_ = 1.0 / resolution_;
    edt_env_->sdf_map_->getRegion(origin_, map_size_3d_);
    cout << "origin_: " << origin_.transpose() << endl;
    cout << "map size: " << map_size_3d_.transpose() << endl;

    path_node_pool_.resize(allocate_num_);
    for (int i = 0; i < allocate_num_; i++)
    {
      path_node_pool_[i] = new Node;
    }
    use_node_num_ = 0;
    iter_num_ = 0;
    early_terminate_cost_ = 0.0;
  }

  void Astar::setResolution(const double &res)
  {
    resolution_ = res;
    this->inv_resolution_ = 1.0 / resolution_;
  }

  bool Astar::judgePointsCollision(const Eigen::Vector3d &first_point, const Eigen::Vector3d &sec_point)
  {
    Eigen::Vector3d cur_dir = sec_point - first_point;
    int temp_i = floor((sec_point - first_point).norm() / resolution_);

    for (int i = 1; i < temp_i + 1; i++)
    {
      Eigen::Vector3d temp_first_point = first_point + i * cur_dir / (temp_i + 1);
      if (edt_env_->sdf_map_->getOccupancy(temp_first_point) == SDFMap::OCCUPIED)
        return true;
    }
    return false;
  }

  int Astar::search(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &end_pt, const bool &local_use)
  {

    if (local_use)
      lambda_heu_ = 100;
    else
      lambda_heu_ = 10000;

    if (local_use)
    {
      cout << "current resolution: " << resolution_ << endl;
      if (resolution_ == 0.4)
        setResolution(0.1);
    }

    NodePtr cur_node = path_node_pool_[0];
    cur_node->parent = NULL;
    cur_node->position = start_pt;
    posToIndex(start_pt, cur_node->index);
    cur_node->g_score = 0.0;
    cur_node->f_score = lambda_heu_ * getDiagHeu(cur_node->position, end_pt);

    Eigen::Vector3i end_index;
    posToIndex(end_pt, end_index);

    open_set_.push(cur_node);
    open_set_map_.insert(make_pair(cur_node->index, cur_node));
    use_node_num_ += 1;

    const auto t1 = ros::Time::now();

    /* ---------- search loop ---------- */
    while (!open_set_.empty())
    {
      cur_node = open_set_.top();
      bool reach_end = abs(cur_node->index(0) - end_index(0)) <= 1 &&
                       abs(cur_node->index(1) - end_index(1)) <= 1 && abs(cur_node->index(2) - end_index(2)) <= 1;
      if (reach_end)
      {
        backtrack(cur_node, end_pt);
        return REACH_END;
      }

      if ((cur_node->position - start_pt).norm() > 8 && local_use)
      {
        std::cout << "reach horizon" << endl;
        backtrack(cur_node, end_pt);
        return REACH_HORIZON;
      }

      // Early termination if time up
      if ((ros::Time::now() - t1).toSec() > max_search_time_ && !local_use)
      {
        std::cout << "early";
        early_terminate_cost_ = cur_node->g_score + getDiagHeu(cur_node->position, end_pt);
        return NO_PATH;
      }

      open_set_.pop();
      open_set_map_.erase(cur_node->index);
      close_set_map_.insert(make_pair(cur_node->index, 1));
      iter_num_ += 1;

      Eigen::Vector3d cur_pos = cur_node->position;
      Eigen::Vector3d nbr_pos;
      Eigen::Vector3d step;

      for (double dx = -resolution_; dx <= resolution_ + 1e-3; dx += resolution_)
        for (double dy = -resolution_; dy <= resolution_ + 1e-3; dy += resolution_)
          for (double dz = -resolution_; dz <= resolution_ + 1e-3; dz += resolution_)
          {
            step << dx, dy, dz;
            if (step.norm() < 1e-3)
              continue;
            nbr_pos = cur_pos + step;
            // Check safety
            if (!edt_env_->sdf_map_->isInBox(nbr_pos))
              continue;
            if (edt_env_->sdf_map_->getInflateOccupancy(nbr_pos) == 1 ||
                edt_env_->sdf_map_->getOccupancy(nbr_pos) == SDFMap::UNKNOWN)
              continue;

            bool safe = true;
            Vector3d dir = nbr_pos - cur_pos;
            double len = dir.norm();
            dir.normalize();
            for (double l = 0.1; l < len; l += 0.1)
            {
              Vector3d ckpt = cur_pos + l * dir;
              if (edt_env_->sdf_map_->getInflateOccupancy(ckpt) == 1 ||
                  edt_env_->sdf_map_->getOccupancy(ckpt) == SDFMap::UNKNOWN)
              {
                safe = false;
                break;
              }
            }

            if (local_use)
            {
              Eigen::Vector3i temp_index;

              edt_env_->sdf_map_->posToIndex(nbr_pos, temp_index);
              if (edt_env_->sdf_map_->getInflateOccupancy(temp_index) == 1 || edt_env_->sdf_map_->getOccupancy(temp_index) == SDFMap::UNKNOWN)
                      safe = false;

              // for (int i = -1; i <= 1; i++)
              //   for (int j = -1; j <= 1; j++)
              //     for (int k = -1; k <= 1; k++)
              //     {
              //       Eigen::Vector3i temp(temp_index[0] + i, temp_index[1] + j, temp_index[2] + k);
                    
              //     }
            }

            if (!safe)
              continue;

            // Check not in close set
            Eigen::Vector3i nbr_idx;
            posToIndex(nbr_pos, nbr_idx);
            if (close_set_map_.find(nbr_idx) != close_set_map_.end())
              continue;

            NodePtr neighbor;
            double tmp_g_score = step.norm() + cur_node->g_score;
            auto node_iter = open_set_map_.find(nbr_idx);
            if (node_iter == open_set_map_.end())
            {
              neighbor = path_node_pool_[use_node_num_];
              use_node_num_ += 1;
              if (use_node_num_ == allocate_num_)
              {
                cout << "run out of node pool." << endl;
                return NO_PATH;
              }
              neighbor->index = nbr_idx;
              neighbor->position = nbr_pos;
            }
            else if (tmp_g_score < node_iter->second->g_score)
            {
              neighbor = node_iter->second;
            }
            else
              continue;

            neighbor->parent = cur_node;
            neighbor->g_score = tmp_g_score;

            Eigen::Vector3d guideDir = (nbr_pos - cur_node->position).normalized();
            double angle = acos(guideDir.dot(cur_vel.normalized())) * 180 / 3.141593;
            if (angle > 180)
              angle = 360 - angle;
            if (cur_vel.norm() < 0.5)
              angle = 0;

            if (iter_num_ <= 20 && local_use)
            {
              neighbor->f_score = tmp_g_score + lambda_heu_ * getDiagHeu(nbr_pos, end_pt) + angle / iter_num_;
            }
            else
              neighbor->f_score = tmp_g_score + lambda_heu_ * getDiagHeu(nbr_pos, end_pt);

            open_set_.push(neighbor);
            open_set_map_[nbr_idx] = neighbor;
          }
    }
    // cout << "open set empty, no path!" << endl;
    // cout << "use node num: " << use_node_num_ << endl;
    // cout << "iter num: " << iter_num_ << endl;
    return NO_PATH;
  }

  double Astar::getEarlyTerminateCost()
  {
    return early_terminate_cost_;
  }

  void Astar::reset()
  {
    open_set_map_.clear();
    close_set_map_.clear();
    path_nodes_.clear();

    std::priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator0> empty_queue;
    open_set_.swap(empty_queue);
    for (int i = 0; i < use_node_num_; i++)
    {
      path_node_pool_[i]->parent = NULL;
    }
    use_node_num_ = 0;
    iter_num_ = 0;
  }

  double Astar::pathLength(const vector<Eigen::Vector3d> &path)
  {
    double length = 0.0;
    if (path.size() < 2)
      return length;
    for (int i = 0; i < path.size() - 1; ++i)
      length += (path[i + 1] - path[i]).norm();
    return length;
  }

  void Astar::backtrack(const NodePtr &end_node, const Eigen::Vector3d &end)
  {
    path_nodes_.push_back(end);
    path_nodes_.push_back(end_node->position);
    NodePtr cur_node = end_node;
    while (cur_node->parent != NULL)
    {
      cur_node = cur_node->parent;
      path_nodes_.push_back(cur_node->position);
    }
    reverse(path_nodes_.begin(), path_nodes_.end());
  }

  std::vector<Eigen::Vector3d> Astar::getPath()
  {
    //精简路径
    // std::vector<Eigen::Vector3d> prunePath;
    // prunePath.push_back(path_nodes_[0]);
    // for(int i = 1 ; i < path_nodes_.size() ; i++){
    //     Eigen::Vector3d curPos = prunePath.back();
    //     double dis = (path_nodes_[i] - curPos).norm();
    //     int gridNum = ceil(dis/resolution_);
    //     Eigen::Vector3d grad, dir, push_dir;
    //     double dist;
    //     for(int j = 1 ; j <= gridNum ; j++){
    //       Eigen::Vector3d tempPos = curPos + j*(path_nodes_[i] - curPos)/gridNum;
    //       //因为有sdf膨胀区域的存在  因此障碍物网格实际上比正常大一个分辨率的网格
    //       if(edt_env_->sdf_map_->getOccupancy(tempPos) == 1){
    //           edt_env_->evaluateEDTWithGrad(tempPos, -1 , dist , grad);
    //           if (grad.norm() > 1e-3) {
    //               grad.normalize();
    //               dir = (tempPos - prunePath.back()).normalized();
    //               push_dir = grad - grad.dot(dir) * dir;
    //               push_dir.normalize();
    //               tempPos = tempPos + resolution_ * push_dir;
    //               prunePath.push_back(tempPos);
    //           }
    //           break;
    //       }
    //     }
    // }
    // prunePath.push_back(path_nodes_.back());

    return path_nodes_;
  }

  double Astar::getDiagHeu(const Eigen::Vector3d &x1, const Eigen::Vector3d &x2)
  {
    double dx = fabs(x1(0) - x2(0));
    double dy = fabs(x1(1) - x2(1));
    double dz = fabs(x1(2) - x2(2));
    double h;
    double diag = min(min(dx, dy), dz);
    dx -= diag;
    dy -= diag;
    dz -= diag;

    if (dx < 1e-4)
    {
      h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dy, dz) + 1.0 * abs(dy - dz);
    }
    if (dy < 1e-4)
    {
      h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dz) + 1.0 * abs(dx - dz);
    }
    if (dz < 1e-4)
    {
      h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dy) + 1.0 * abs(dx - dy);
    }
    return tie_breaker_ * h;
  }

  double Astar::getManhHeu(const Eigen::Vector3d &x1, const Eigen::Vector3d &x2)
  {
    double dx = fabs(x1(0) - x2(0));
    double dy = fabs(x1(1) - x2(1));
    double dz = fabs(x1(2) - x2(2));
    return tie_breaker_ * (dx + dy + dz);
  }

  double Astar::getEuclHeu(const Eigen::Vector3d &x1, const Eigen::Vector3d &x2)
  {
    return tie_breaker_ * (x2 - x1).norm();
  }

  std::vector<Eigen::Vector3d> Astar::getVisited()
  {
    vector<Eigen::Vector3d> visited;
    for (int i = 0; i < use_node_num_; ++i)
      visited.push_back(path_node_pool_[i]->position);
    return visited;
  }

  void Astar::posToIndex(const Eigen::Vector3d &pt, Eigen::Vector3i &idx)
  {
    idx = ((pt - origin_) * inv_resolution_).array().floor().cast<int>();
  }

} // namespace fast_planner
