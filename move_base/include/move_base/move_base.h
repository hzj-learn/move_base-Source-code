/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
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
#ifndef NAV_MOVE_BASE_ACTION_H_
#define NAV_MOVE_BASE_ACTION_H_

#include <vector>
#include <string>

#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <nav_core/base_local_planner.h>
#include <nav_core/base_global_planner.h>
#include <nav_core/recovery_behavior.h>
#include <geometry_msgs/PoseStamped.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_msgs/GetPlan.h>

#include <pluginlib/class_loader.hpp>
#include <std_srvs/Empty.h>

#include <dynamic_reconfigure/server.h>
#include "move_base/MoveBaseConfig.h"

namespace move_base {
  //typedefs to help us out with the action server so that we don't hace to type so much
  typedef actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> MoveBaseActionServer;

//move base的状态
  enum MoveBaseState {
    PLANNING,     //规划状态，如果我们处于PLANNING状态，那么我们将尝试规划一条路线make a plan
    CONTROLLING,  //控制状态，如果处于控制状态，我们会试图找到有效的速度twist命令
    CLEARING      //代价地图清除状态，我们将尝试用任何用户提供的恢复行为来清除空间
  };

//重恢复的状态
  enum RecoveryTrigger
  {
    PLANNING_R,   //规划状态，如果我们处于PLANNING状态，那么我们将尝试规划一条路线make a plan
    CONTROLLING_R,//控制状态，如果处于控制状态，我们会试图找到有效的速度twist命令
    OSCILLATION_R //震荡状态
  };

  /**
   * @class MoveBase
   * @brief A class that uses the    actionlib::ActionServer interface     that moves the robot base to a goal location.
   */
  class MoveBase {
    public:
      /**
       * @brief  Constructor for the actions
       * @param name The name of the action
       * @param tf A reference to a TransformListener
       */
      MoveBase(tf2_ros::Buffer& tf);

      /**
       * @brief  Destructor - Cleans up
       */
      virtual ~MoveBase();

      /**
       * @brief  Performs a control cycle【执行闭环控制】
       * @param goal A reference to the goal to pursue
       * @param global_plan A reference to the global plan being used
       * @return True if processing of the goal is done, false otherwise
       */
      bool executeCycle(geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& global_plan);




    private:
      /**
       * @brief  A service call that clears the costmaps of obstacles【执行清除代价地图的障碍物的服务】
       * @param req The service request 
       * @param resp The service response
       * @return True if the service call succeeds, false otherwise
       */
      bool clearCostmapsService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);

      /**
       * @brief  A service call that can be made when the action is inactive that will return a plan
       * 【当操作处于非活动状态(action)时可以进行的服务调用，它将返回一个plan路径】
       * @param  req The goal request
       * @param  resp The plan request
       * @return True if planning succeeded, false otherwise
       */
      bool planService(nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &resp);

      /**
       * @brief  Make a new global plan【进行一个新的路径目标点的规划】
       * @param  goal The goal to plan to
       * @param  plan Will be filled in with the plan made by the planner【选择是全局规划器】
       * @return  True if planning succeeds, false otherwise
       */
      bool makePlan(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

      /**
       * @brief  Load the recovery behaviors for the navigation stack from the parameter server
       * 从参数服务器加载navigation stack的恢复行为
       * @param node The ros::NodeHandle to be used for loading parameters 
       * @return True if the recovery behaviors were loaded successfully, false otherwise
       */
      bool loadRecoveryBehaviors(ros::NodeHandle node);

      /**
       * @brief  Loads the default recovery behaviors for the navigation stack
       * 加载navigation stack的默认恢复行为
       */
      void loadDefaultRecoveryBehaviors();

      /**
       * @brief  Clears obstacles within a window around the robot
       * 清除机器人范围内代价地图的障碍物
       * @param size_x The x size of the window
       * @param size_y The y size of the window
       */
      void clearCostmapWindows(double size_x, double size_y);

      /**
       * @brief  Publishes a velocity command of zero to the base
       * 向机器人发布零速度命令
       */
      void publishZeroVelocity();

      /**
       * @brief  Reset the state of the move_base action and send a zero velocity command to the base
       * 重置移动基础动作的状态，并向机器人发送零速度命令
       */
      void resetState();

      void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal);

      void planThread();

      void executeCb(const move_base_msgs::MoveBaseGoalConstPtr& move_base_goal);

      bool isQuaternionValid(const geometry_msgs::Quaternion& q);

      bool getRobotPose(geometry_msgs::PoseStamped& global_pose, costmap_2d::Costmap2DROS* costmap);

      double distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2);

      geometry_msgs::PoseStamped goalToGlobalFrame(const geometry_msgs::PoseStamped& goal_pose_msg);

      /**
       * @brief This is used to wake the planner at periodic intervals.
       * 用于定期唤醒规划器
       */
      void wakePlanner(const ros::TimerEvent& event);

      tf2_ros::Buffer& tf_;

      MoveBaseActionServer* as_;

      boost::shared_ptr<nav_core::BaseLocalPlanner> tc_;
      costmap_2d::Costmap2DROS* planner_costmap_ros_, *controller_costmap_ros_;

      boost::shared_ptr<nav_core::BaseGlobalPlanner> planner_;
      std::string robot_base_frame_, global_frame_;

      std::vector<boost::shared_ptr<nav_core::RecoveryBehavior> > recovery_behaviors_;
      unsigned int recovery_index_;

      geometry_msgs::PoseStamped global_pose_;
      double planner_frequency_, controller_frequency_, inscribed_radius_, circumscribed_radius_;
      double planner_patience_, controller_patience_;
      int32_t max_planning_retries_;
      uint32_t planning_retries_;
      double conservative_reset_dist_, clearing_radius_;
      ros::Publisher current_goal_pub_, vel_pub_, action_goal_pub_;
      ros::Subscriber goal_sub_;
      ros::ServiceServer make_plan_srv_, clear_costmaps_srv_;
      bool shutdown_costmaps_, clearing_rotation_allowed_, recovery_behavior_enabled_;
      bool make_plan_clear_costmap_, make_plan_add_unreachable_goal_;
      double oscillation_timeout_, oscillation_distance_;

      MoveBaseState state_;
      RecoveryTrigger recovery_trigger_;

      ros::Time last_valid_plan_, last_valid_control_, last_oscillation_reset_;
      geometry_msgs::PoseStamped oscillation_pose_;
      pluginlib::ClassLoader<nav_core::BaseGlobalPlanner> bgp_loader_;
      pluginlib::ClassLoader<nav_core::BaseLocalPlanner> blp_loader_;
      pluginlib::ClassLoader<nav_core::RecoveryBehavior> recovery_loader_;

      //set up plan triple buffer
      std::vector<geometry_msgs::PoseStamped>* planner_plan_;
      std::vector<geometry_msgs::PoseStamped>* latest_plan_;
      std::vector<geometry_msgs::PoseStamped>* controller_plan_;

      //set up the planner's thread
      bool runPlanner_;
      boost::recursive_mutex planner_mutex_;        //循环的规划
      boost::condition_variable_any planner_cond_;  //规划的条件变量
      geometry_msgs::PoseStamped planner_goal_;
      boost::thread* planner_thread_;


      boost::recursive_mutex configuration_mutex_;
      dynamic_reconfigure::Server<move_base::MoveBaseConfig> *dsrv_;
      
      void reconfigureCB(move_base::MoveBaseConfig &config, uint32_t level);

      move_base::MoveBaseConfig last_config_;
      move_base::MoveBaseConfig default_config_;
      bool setup_, p_freq_change_, c_freq_change_;
      bool new_global_plan_;
  };
};
#endif

