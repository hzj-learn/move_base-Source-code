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
#include <rotate_recovery/rotate_recovery.h>
#include <pluginlib/class_list_macros.h>
#include <nav_core/parameter_magic.h>
#include <tf2/utils.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <angles/angles.h>
#include <algorithm>
#include <string>


// register this planner as a RecoveryBehavior plugin
PLUGINLIB_EXPORT_CLASS(rotate_recovery::RotateRecovery, nav_core::RecoveryBehavior)

namespace rotate_recovery
{
RotateRecovery::RotateRecovery(): local_costmap_(NULL), initialized_(false), world_model_(NULL)
{
}

void RotateRecovery::initialize(std::string name, tf2_ros::Buffer*,
                                costmap_2d::Costmap2DROS*, costmap_2d::Costmap2DROS* local_costmap)
{
  if (!initialized_)
  {
    local_costmap_ = local_costmap;

    //(1)从参数服务器获取一些参数
    ros::NodeHandle private_nh("~/" + name);
    ros::NodeHandle blp_nh("~/TrajectoryPlannerROS");
    //默认情况下，我们将模拟每个参数
    private_nh.param("sim_granularity", sim_granularity_, 0.017);
    private_nh.param("frequency", frequency_, 20.0);

    //（2）设置最大的加速度、最大和最小的速度、转角偏差的容忍度
    acc_lim_th_ = nav_core::loadParameterWithDeprecation(blp_nh, "acc_lim_theta", "acc_lim_th", 3.2);
    max_rotational_vel_ = nav_core::loadParameterWithDeprecation(blp_nh, "max_vel_theta", "max_rotational_vel", 1.0);
    min_rotational_vel_ = nav_core::loadParameterWithDeprecation(blp_nh, "min_in_place_vel_theta", "min_in_place_rotational_vel", 0.4);
    blp_nh.param("yaw_goal_tolerance", tolerance_, 0.10);

    //（3）加载车模型
    world_model_ = new base_local_planner::CostmapModel(*local_costmap_->getCostmap());
    initialized_ = true;
  }
  else
  {
    ROS_ERROR("You should not call initialize twice on this object, doing nothing");
  }
}

RotateRecovery::~RotateRecovery()
{
  delete world_model_;
}

void RotateRecovery::runBehavior()
{
  if (!initialized_)
  {
    ROS_ERROR("This object must be initialized before runBehavior is called");
    return;
  }

  if (local_costmap_ == NULL)
  {
    ROS_ERROR("The costmap passed to the RotateRecovery object cannot be NULL. Doing nothing.");
    return;
  }
  ROS_WARN("Rotate recovery behavior started.");

//（1）创建一个发布器，发布Twist指令
  ros::Rate r(frequency_);
  ros::NodeHandle n;
  ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);

//（2）获取机器人的全局位置、航向角
  geometry_msgs::PoseStamped global_pose;
  local_costmap_->getRobotPose(global_pose);
  double current_angle = tf2::getYaw(global_pose.pose.orientation);
  double start_angle = current_angle;

  bool got_180 = false;

//（3）当还没有转到位，或者还没转到180度之前，死循环继续转，直到旋转完成
  while (n.ok() &&
         (!got_180 ||
          std::fabs(angles::shortest_angular_distance(current_angle, start_angle)) > tolerance_))
  {
    // 1）更新当前机器人的位置、角度
    local_costmap_->getRobotPose(global_pose);
    current_angle = tf2::getYaw(global_pose.pose.orientation);

    // 2）计算要向左旋转的距离dist_left，并进行旋转
    double dist_left;
    if (!got_180)//如果我们还没有达到180，我们需要旋转半圈加上到180点的距离
    {
      double distance_to_180 = std::fabs(angles::shortest_angular_distance(current_angle, start_angle + M_PI));
      dist_left = M_PI + distance_to_180;

      if (distance_to_180 < tolerance_)
      {
        got_180 = true;
      }
    }
    else//如果我们达到180度，我们就可以回到起点了
    {
      dist_left = std::fabs(angles::shortest_angular_distance(current_angle, start_angle));
    }

    // 3）通过正向模拟检查速度是否合法
    double x = global_pose.pose.position.x, y = global_pose.pose.position.y;
    double sim_angle = 0.0;
    while (sim_angle < dist_left)
    {
      double theta = current_angle + sim_angle;

      // make sure that the point is legal, if it isn't... we'll abort
      double footprint_cost = world_model_->footprintCost(x, y, theta, local_costmap_->getRobotFootprint(), 0.0, 0.0);
      if (footprint_cost < 0.0)
      {
        ROS_ERROR("Rotate recovery can't rotate in place because there is a potential collision. Cost: %.2f",
                  footprint_cost);
        return;
      }

      sim_angle += sim_granularity_;
    }

    //4）计算到达目标时让我们停下来的速度 
    double vel = sqrt(2 * acc_lim_th_ * dist_left);
    //确保此速度在指定的限制范围内 
    vel = std::min(std::max(vel, min_rotational_vel_), max_rotational_vel_);

    //5）发布速度指令
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = vel;

    vel_pub.publish(cmd_vel);

    r.sleep();
  }
}
};  // namespace rotate_recovery
