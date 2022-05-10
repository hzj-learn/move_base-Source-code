/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Eitan Marder-Eppstein
*********************************************************************/
#include <clear_costmap_recovery/clear_costmap_recovery.h>
#include <pluginlib/class_list_macros.h>
#include <vector>

//register this planner as a RecoveryBehavior plugin
PLUGINLIB_EXPORT_CLASS(clear_costmap_recovery::ClearCostmapRecovery, nav_core::RecoveryBehavior)

using costmap_2d::NO_INFORMATION;

namespace clear_costmap_recovery {

ClearCostmapRecovery::ClearCostmapRecovery(): global_costmap_(NULL), local_costmap_(NULL),
  tf_(NULL), initialized_(false) {}

void ClearCostmapRecovery::initialize(std::string name, tf2_ros::Buffer* tf,
    costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap){
  if(!initialized_){
    name_ = name;
    tf_ = tf;
    global_costmap_ = global_costmap;
    local_costmap_ = local_costmap;

    //(1)从参数服务器获取一些参数
    ros::NodeHandle private_nh("~/" + name_);

    private_nh.param("reset_distance", reset_distance_, 3.0);
    private_nh.param("force_updating", force_updating_, false);
    private_nh.param("affected_maps", affected_maps_, std::string("both"));
    if (affected_maps_ != "local" && affected_maps_ != "global" && affected_maps_ != "both")
    {
      ROS_WARN("Wrong value for affected_maps parameter: '%s'; valid values are 'local', 'global' or 'both'; " \
               "defaulting to 'both'", affected_maps_.c_str());
      affected_maps_ = "both";
    }

    std::vector<std::string> clearable_layers_default, clearable_layers;
    clearable_layers_default.push_back( std::string("obstacles") );
    private_nh.param("layer_names", clearable_layers, clearable_layers_default);

    //(2)把每一层的代价地图进行清除
    for(unsigned i=0; i < clearable_layers.size(); i++) {
        ROS_INFO("Recovery behavior will clear layer '%s'", clearable_layers[i].c_str());
        clearable_layers_.insert(clearable_layers[i]);
    }

    initialized_ = true;
  }
  else{
    ROS_ERROR("You should not call initialize twice on this object, doing nothing");
  }
}


//运行ClearCostmapRecovery行为。将costmap还原为用户指定窗口外的静态映射，并清除robot周围的未知空间。
void ClearCostmapRecovery::runBehavior(){
  //若行为重恢复没有初始化，退出
  if(!initialized_)
  {
    ROS_ERROR("This object must be initialized before runBehavior is called");
    return;
  }

  //若全局比图和局部地图为空，退出
  if(global_costmap_ == NULL || local_costmap_ == NULL){
    ROS_ERROR("The costmaps passed to the ClearCostmapRecovery object cannot be NULL. Doing nothing.");
    return;
  }

  ROS_WARN("Clearing %s costmap%s to unstuck robot (%.2fm).", affected_maps_.c_str(),
           affected_maps_ == "both" ? "s" : "", reset_distance_);

  //(1)清除并更新当前全局代价地图
  ros::WallTime t0 = ros::WallTime::now();
  if (affected_maps_ == "global" || affected_maps_ == "both")
  {
    //1)清除当前全局代价地图
    clear(global_costmap_);

    //2）更新全局地图
    if (force_updating_)
      global_costmap_->updateMap();

    ROS_DEBUG("Global costmap cleared in %fs", (ros::WallTime::now() - t0).toSec());
  }

  //(2)清除并更新当前局部代价地图
  t0 = ros::WallTime::now();
  if (affected_maps_ == "local" || affected_maps_ == "both")
  {
    //1)清除当前局部代价地图
    clear(local_costmap_);

    //2)更新当前局部代价地图
    if (force_updating_)
      local_costmap_->updateMap();

    ROS_DEBUG("Local costmap cleared in %fs", (ros::WallTime::now() - t0).toSec());
  }
}

void ClearCostmapRecovery::clear(costmap_2d::Costmap2DROS* costmap){
  //（1）通过插件costmap获取二维代价地图
  std::vector<boost::shared_ptr<costmap_2d::Layer> >* plugins = costmap->getLayeredCostmap()->getPlugins();

  geometry_msgs::PoseStamped pose;

  //（2）现在还不能够清除代价地图
  if(!costmap->getRobotPose(pose)){
    ROS_ERROR("Cannot clear map because pose cannot be retrieved");
    return;
  }

  double x = pose.pose.position.x;
  double y = pose.pose.position.y;

  for (std::vector<boost::shared_ptr<costmap_2d::Layer> >::iterator pluginp = plugins->begin(); pluginp != plugins->end(); ++pluginp) {
    boost::shared_ptr<costmap_2d::Layer> plugin = *pluginp;
    std::string name = plugin->getName();
    int slash = name.rfind('/');
    if( slash != std::string::npos ){
        name = name.substr(slash+1);
    }

    if(clearable_layers_.count(name)!=0){
      boost::shared_ptr<costmap_2d::CostmapLayer> costmap;
      costmap = boost::static_pointer_cast<costmap_2d::CostmapLayer>(plugin);
      clearMap(costmap, x, y);
    }
  }
}


void ClearCostmapRecovery::clearMap(boost::shared_ptr<costmap_2d::CostmapLayer> costmap,
                                        double pose_x, double pose_y){
  boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));

  double start_point_x = pose_x - reset_distance_ / 2;
  double start_point_y = pose_y - reset_distance_ / 2;
  double end_point_x = start_point_x + reset_distance_;
  double end_point_y = start_point_y + reset_distance_;

  int start_x, start_y, end_x, end_y;
  costmap->worldToMapNoBounds(start_point_x, start_point_y, start_x, start_y);
  costmap->worldToMapNoBounds(end_point_x, end_point_y, end_x, end_y);

  costmap->clearArea(start_x, start_y, end_x, end_y);

  double ox = costmap->getOriginX(), oy = costmap->getOriginY();
  double width = costmap->getSizeInMetersX(), height = costmap->getSizeInMetersY();
  costmap->addExtraBounds(ox, oy, ox + width, oy + height);
  return;
}

};
