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

#include <base_local_planner/trajectory_planner_ros.h>

#ifdef HAVE_SYS_TIME_H
#include <sys/time.h>
#endif

#include <boost/tokenizer.hpp>

#include <Eigen/Core>
#include <cmath>

#include <ros/console.h>

#include <pluginlib/class_list_macros.h>

#include <base_local_planner/goal_functions.h>
#include <nav_msgs/Path.h>

#include <nav_core/parameter_magic.h>
#include <tf2/utils.h>

//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(base_local_planner::TrajectoryPlannerROS, nav_core::BaseLocalPlanner)

namespace base_local_planner {

  void TrajectoryPlannerROS::reconfigureCB(BaseLocalPlannerConfig &config, uint32_t level) {
      if (setup_ && config.restore_defaults) {
        config = default_config_;
        //Avoid looping
        config.restore_defaults = false;
      }
      if ( ! setup_) {
        default_config_ = config;
        setup_ = true;
      }
      tc_->reconfigure(config);
      reached_goal_ = false;
  }

  TrajectoryPlannerROS::TrajectoryPlannerROS() :
      world_model_(NULL), tc_(NULL), costmap_ros_(NULL), tf_(NULL), setup_(false), initialized_(false), odom_helper_("odom") {}

  TrajectoryPlannerROS::TrajectoryPlannerROS(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros) :
      world_model_(NULL), tc_(NULL), costmap_ros_(NULL), tf_(NULL), setup_(false), initialized_(false), odom_helper_("odom") {

      //initialize the planner
      initialize(name, tf, costmap_ros);
  }


//初始化
  void TrajectoryPlannerROS::initialize(
      std::string name,
      tf2_ros::Buffer* tf,
      costmap_2d::Costmap2DROS* costmap_ros){
    if (! isInitialized()) 
    {
      ros::NodeHandle private_nh("~/" + name);
      //（1）创建一个发布器，发布全局路径
      g_1plan_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1);
      //（2）创建一个发布器，发布局部路径
      l_plan_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);
      //（3）获取代价地图costmap
      costmap_ = costmap_ros_->getCostmap();
      //（4）定义一些局部规划要用到的变量
      tf_ = tf;
      costmap_ros_ = costmap_ros;
      rot_stopped_velocity_ = 1e-2;
      trans_stopped_velocity_ = 1e-2;
      double sim_time, sim_granularity, angular_sim_granularity;
      int vx_samples, vtheta_samples;
      double path_distance_bias, goal_distance_bias, occdist_scale, heading_lookahead, oscillation_reset_dist, escape_reset_dist, escape_reset_theta;
      bool holonomic_robot, dwa, simple_attractor, heading_scoring;
      double heading_scoring_timestep;
      double max_vel_x, min_vel_x;
      double backup_vel;
      double stop_time_buffer;
      std::string world_model_type;
      rotating_to_goal_ = false;

      //（5）使用上述变量设置参数
      {
      global_frame_ = costmap_ros_->getGlobalFrameID();
      robot_base_frame_ = costmap_ros_->getBaseFrameID();
      private_nh.param("prune_plan", prune_plan_, true);

      private_nh.param("yaw_goal_tolerance", yaw_goal_tolerance_, 0.05);
      private_nh.param("xy_goal_tolerance", xy_goal_tolerance_, 0.10);
      private_nh.param("acc_lim_x", acc_lim_x_, 2.5);
      private_nh.param("acc_lim_y", acc_lim_y_, 2.5);
      private_nh.param("acc_lim_theta", acc_lim_theta_, 3.2);

      private_nh.param("stop_time_buffer", stop_time_buffer, 0.2);

      private_nh.param("latch_xy_goal_tolerance", latch_xy_goal_tolerance_, false);

      //如果用户设置了一个错误的参数，就通知用户
      if(private_nh.hasParam("acc_limit_x"))
        ROS_ERROR("You are using acc_limit_x where you should be using acc_lim_x. Please change your configuration files appropriately. The documentation used to be wrong on this, sorry for any confusion.");

      if(private_nh.hasParam("acc_limit_y"))
        ROS_ERROR("You are using acc_limit_y where you should be using acc_lim_y. Please change your configuration files appropriately. The documentation used to be wrong on this, sorry for any confusion.");

      if(private_nh.hasParam("acc_limit_th"))
        ROS_ERROR("You are using acc_limit_th where you should be using acc_lim_th. Please change your configuration files appropriately. The documentation used to be wrong on this, sorry for any confusion.");

      //假设这个计划器是在导航堆栈中运行的，我们可以向上搜索它运行的频率。这也允许本地覆盖频率。
      std::string controller_frequency_param_name;
      if(!private_nh.searchParam("controller_frequency", controller_frequency_param_name))
        sim_period_ = 0.05;
      else
      {
        double controller_frequency = 0;
        private_nh.param(controller_frequency_param_name, controller_frequency, 20.0);
        if(controller_frequency > 0)
          sim_period_ = 1.0 / controller_frequency;
        else
        {
          ROS_WARN("A controller_frequency less than 0 has been set. Ignoring the parameter, assuming a rate of 20Hz");
          sim_period_ = 0.05;
        }
      }
      ROS_INFO("Sim period is set to %.2f", sim_period_);

      private_nh.param("sim_time", sim_time, 1.0);
      private_nh.param("sim_granularity", sim_granularity, 0.025);
      private_nh.param("angular_sim_granularity", angular_sim_granularity, sim_granularity);
      private_nh.param("vx_samples", vx_samples, 3);
      private_nh.param("vtheta_samples", vtheta_samples, 20);

      path_distance_bias = nav_core::loadParameterWithDeprecation(private_nh,
                                                                  "path_distance_bias",
                                                                  "pdist_scale",
                                                                  0.6);
      goal_distance_bias = nav_core::loadParameterWithDeprecation(private_nh,
                                                                  "goal_distance_bias",
                                                                  "gdist_scale",
                                                                  0.6);
      //不推荐使用的参数的值需要应用于当前参数，否则为动态重新配置定义的默认值将覆盖它们。
      if (private_nh.hasParam("pdist_scale") & !private_nh.hasParam("path_distance_bias"))
      {
        private_nh.setParam("path_distance_bias", path_distance_bias);
      }
      if (private_nh.hasParam("gdist_scale") & !private_nh.hasParam("goal_distance_bias"))
      {
        private_nh.setParam("goal_distance_bias", goal_distance_bias);
      }

      private_nh.param("occdist_scale", occdist_scale, 0.01);

      bool meter_scoring;
      if ( ! private_nh.hasParam("meter_scoring")) {
        ROS_WARN("Trajectory Rollout planner initialized with param meter_scoring not set. Set it to true to make your settings robust against changes of costmap resolution.");
      } else {
        private_nh.param("meter_scoring", meter_scoring, false);

        if(meter_scoring) {
          //if we use meter scoring, then we want to multiply the biases by the resolution of the costmap
          double resolution = costmap_->getResolution();
          goal_distance_bias *= resolution;
          path_distance_bias *= resolution;
        } else {
          ROS_WARN("Trajectory Rollout planner initialized with param meter_scoring set to false. Set it to true to make your settings robust against changes of costmap resolution.");
        }
      }

      private_nh.param("heading_lookahead", heading_lookahead, 0.325);
      private_nh.param("oscillation_reset_dist", oscillation_reset_dist, 0.05);
      private_nh.param("escape_reset_dist", escape_reset_dist, 0.10);
      private_nh.param("escape_reset_theta", escape_reset_theta, M_PI_4);
      private_nh.param("holonomic_robot", holonomic_robot, true);
      private_nh.param("max_vel_x", max_vel_x, 0.5);
      private_nh.param("min_vel_x", min_vel_x, 0.1);

      double max_rotational_vel;
      private_nh.param("max_rotational_vel", max_rotational_vel, 1.0);
      max_vel_th_ = max_rotational_vel;
      min_vel_th_ = -1.0 * max_rotational_vel;

      min_in_place_vel_th_ = nav_core::loadParameterWithDeprecation(private_nh,
                                                                    "min_in_place_vel_theta",
                                                                    "min_in_place_rotational_vel", 0.4);
      reached_goal_ = false;
      backup_vel = -0.1;
      if(private_nh.getParam("backup_vel", backup_vel))
        ROS_WARN("The backup_vel parameter has been deprecated in favor of the escape_vel parameter. To switch, just change the parameter name in your configuration files.");

      //如果同时设置了backup_vel和escape_vel，我们用escape_vel
      private_nh.getParam("escape_vel", backup_vel);

      if(backup_vel >= 0.0)
        ROS_WARN("You've specified a positive escape velocity. This is probably not what you want and will cause the robot to move forward instead of backward. You should probably change your escape_vel parameter to be negative");

      private_nh.param("world_model", world_model_type, std::string("costmap"));
      private_nh.param("dwa", dwa, true);
      private_nh.param("heading_scoring", heading_scoring, false);
      private_nh.param("heading_scoring_timestep", heading_scoring_timestep, 0.8);

      simple_attractor = false;

      //使用自由空间控制器的参数
      double min_pt_separation, max_obstacle_height, grid_resolution;
      private_nh.param("point_grid/max_sensor_range", max_sensor_range_, 2.0);
      private_nh.param("point_grid/min_pt_separation", min_pt_separation, 0.01);
      private_nh.param("point_grid/max_obstacle_height", max_obstacle_height, 2.0);
      private_nh.param("point_grid/grid_resolution", grid_resolution, 0.2);

      ROS_ASSERT_MSG(world_model_type == "costmap", "At this time, only costmap world models are supported by this controller");
    }

      //（6）设置小车在代价地图的world_model
      world_model_ = new CostmapModel(*costmap_);
      std::vector<double> y_vels = loadYVels(private_nh);

      //（7）设置小车轨迹打印出来
      footprint_spec_ = costmap_ros_->getRobotFootprint();

      //（8）创建一个局部规划器TrajectoryPlanner
      tc_ = new TrajectoryPlanner(*world_model_, *costmap_, footprint_spec_,
          acc_lim_x_, acc_lim_y_, acc_lim_theta_, sim_time, sim_granularity, vx_samples, vtheta_samples, path_distance_bias,
          goal_distance_bias, occdist_scale, heading_lookahead, oscillation_reset_dist, escape_reset_dist, escape_reset_theta, holonomic_robot,
          max_vel_x, min_vel_x, max_vel_th_, min_vel_th_, min_in_place_vel_th_, backup_vel,
          dwa, heading_scoring, heading_scoring_timestep, meter_scoring, simple_attractor, y_vels, stop_time_buffer, sim_period_, angular_sim_granularity);

      //（9）初始化地图可视化在rviz上
      map_viz_.initialize(name, global_frame_, boost::bind(&TrajectoryPlanner::getCellCosts, tc_, _1, _2, _3, _4, _5, _6));
      initialized_ = true;

      //（10）初始化动态参数服务器dynamic_reconfigure
      dsrv_ = new dynamic_reconfigure::Server<BaseLocalPlannerConfig>(private_nh);
      dynamic_reconfigure::Server<BaseLocalPlannerConfig>::CallbackType cb = boost::bind(&TrajectoryPlannerROS::reconfigureCB, this, _1, _2);
      dsrv_->setCallback(cb);

    }
    else 
    {
      ROS_WARN("This planner has already been initialized, doing nothing");
    }
  }

  std::vector<double> TrajectoryPlannerROS::loadYVels(ros::NodeHandle node){
    std::vector<double> y_vels;

    std::string y_vel_list;
    if(node.getParam("y_vels", y_vel_list)){
      typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
      boost::char_separator<char> sep("[], ");
      tokenizer tokens(y_vel_list, sep);

      for(tokenizer::iterator i = tokens.begin(); i != tokens.end(); i++){
        y_vels.push_back(atof((*i).c_str()));
      }
    }
    else{
      //if no values are passed in, we'll provide defaults
      y_vels.push_back(-0.3);
      y_vels.push_back(-0.1);
      y_vels.push_back(0.1);
      y_vels.push_back(0.3);
    }

    return y_vels;
  }

  TrajectoryPlannerROS::~TrajectoryPlannerROS() {
    //make sure to clean things up
    delete dsrv_;

    if(tc_ != NULL)
      delete tc_;

    if(world_model_ != NULL)
      delete world_model_;
  }

  bool TrajectoryPlannerROS::stopWithAccLimits(const geometry_msgs::PoseStamped& global_pose, const geometry_msgs::PoseStamped& robot_vel, geometry_msgs::Twist& cmd_vel){
    //slow down with the maximum possible acceleration... we should really use the frequency that we're running at to determine what is feasible
    //but we'll use a tenth of a second to be consistent with the implementation of the local planner.
    double vx = sign(robot_vel.pose.position.x) * std::max(0.0, (fabs(robot_vel.pose.position.x) - acc_lim_x_ * sim_period_));
    double vy = sign(robot_vel.pose.position.y) * std::max(0.0, (fabs(robot_vel.pose.position.y) - acc_lim_y_ * sim_period_));

    double vel_yaw = tf2::getYaw(robot_vel.pose.orientation);
    double vth = sign(vel_yaw) * std::max(0.0, (fabs(vel_yaw) - acc_lim_theta_ * sim_period_));

    //we do want to check whether or not the command is valid
    double yaw = tf2::getYaw(global_pose.pose.orientation);
    bool valid_cmd = tc_->checkTrajectory(global_pose.pose.position.x, global_pose.pose.position.y, yaw,
        robot_vel.pose.position.x, robot_vel.pose.position.y, vel_yaw, vx, vy, vth);

    //if we have a valid command, we'll pass it on, otherwise we'll command all zeros
    if(valid_cmd){
      ROS_DEBUG("Slowing down... using vx, vy, vth: %.2f, %.2f, %.2f", vx, vy, vth);
      cmd_vel.linear.x = vx;
      cmd_vel.linear.y = vy;
      cmd_vel.angular.z = vth;
      return true;
    }

    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
    return false;
  }

  bool TrajectoryPlannerROS::rotateToGoal(const geometry_msgs::PoseStamped& global_pose, const geometry_msgs::PoseStamped& robot_vel, double goal_th, geometry_msgs::Twist& cmd_vel){
    double yaw = tf2::getYaw(global_pose.pose.orientation);
    double vel_yaw = tf2::getYaw(robot_vel.pose.orientation);
    cmd_vel.linear.x = 0;
    cmd_vel.linear.y = 0;
    double ang_diff = angles::shortest_angular_distance(yaw, goal_th);

    double v_theta_samp = ang_diff > 0.0 ? std::min(max_vel_th_,
        std::max(min_in_place_vel_th_, ang_diff)) : std::max(min_vel_th_,
        std::min(-1.0 * min_in_place_vel_th_, ang_diff));

    //take the acceleration limits of the robot into account
    double max_acc_vel = fabs(vel_yaw) + acc_lim_theta_ * sim_period_;
    double min_acc_vel = fabs(vel_yaw) - acc_lim_theta_ * sim_period_;

    v_theta_samp = sign(v_theta_samp) * std::min(std::max(fabs(v_theta_samp), min_acc_vel), max_acc_vel);

    //we also want to make sure to send a velocity that allows us to stop when we reach the goal given our acceleration limits
    double max_speed_to_stop = sqrt(2 * acc_lim_theta_ * fabs(ang_diff)); 

    v_theta_samp = sign(v_theta_samp) * std::min(max_speed_to_stop, fabs(v_theta_samp));

    // Re-enforce min_in_place_vel_th_.  It is more important than the acceleration limits.
    v_theta_samp = v_theta_samp > 0.0
      ? std::min( max_vel_th_, std::max( min_in_place_vel_th_, v_theta_samp ))
      : std::max( min_vel_th_, std::min( -1.0 * min_in_place_vel_th_, v_theta_samp ));

    //we still want to lay down the footprint of the robot and check if the action is legal
    bool valid_cmd = tc_->checkTrajectory(global_pose.pose.position.x, global_pose.pose.position.y, yaw,
        robot_vel.pose.position.x, robot_vel.pose.position.y, vel_yaw, 0.0, 0.0, v_theta_samp);

    ROS_DEBUG("Moving to desired goal orientation, th cmd: %.2f, valid_cmd: %d", v_theta_samp, valid_cmd);

    if(valid_cmd){
      cmd_vel.angular.z = v_theta_samp;
      return true;
    }

    cmd_vel.angular.z = 0.0;
    return false;

  }



  bool TrajectoryPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan){
    if (! isInitialized()) {
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }

    //reset the global plan
    global_plan_.clear();
    global_plan_ = orig_global_plan;
    
    //when we get a new plan, we also want to clear any latch we may have on goal tolerances
    xy_tolerance_latch_ = false;
    //reset the at goal flag
    reached_goal_ = false;
    return true;
  }

//这里是真正的核心
  bool TrajectoryPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
  {
    if (! isInitialized()) {
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }

    //（1）首先在代价地图costmap中获取全局目标点，再将机器人的姿态以及全局路径规划转换到global_frame_上
    std::vector<geometry_msgs::PoseStamped> local_plan;
    geometry_msgs::PoseStamped global_pose;
    if (!costmap_ros_->getRobotPose(global_pose)) {
      return false;
    }
    std::vector<geometry_msgs::PoseStamped> transformed_plan;
    if (!transformGlobalPlan(*tf_, global_plan_, global_pose, *costmap_, global_frame_, transformed_plan)) {
      ROS_WARN("Could not transform the global plan to the frame of the controller");
      return false;
    }

    //（2）根据机器人的位置来，修剪掉那些远离机器人的全局规划的目标
    if(prune_plan_)
      prunePlan(global_pose, transformed_plan, global_plan_);

    //（3）定义机器人的位置控制指令、速度控制指令
    geometry_msgs::PoseStamped drive_cmds;
    drive_cmds.header.frame_id = robot_base_frame_;
    geometry_msgs::PoseStamped robot_vel;
    odom_helper_.getRobotVel(robot_vel);

    //（4）如果传入的全局规划为空，直接退出程序
    if(transformed_plan.empty())
      return false;

    //（5）设定目标点的位姿（x,y，yaw），假设全局目标是全局计划的最后一点
    const geometry_msgs::PoseStamped& goal_point = transformed_plan.back();
    const double goal_x = goal_point.pose.position.x;
    const double goal_y = goal_point.pose.position.y;
    const double yaw = tf2::getYaw(goal_point.pose.orientation);
    double goal_th = yaw;

    //（6）【核心】检查我们是否到达了目标位置，并采取控制措施
    if (xy_tolerance_latch_ || (getGoalPositionDistance(global_pose, goal_x, goal_y) <= xy_goal_tolerance_)) 
    {
      //1）如果用户想要锁定目标公差，如果我们到达目标位置，我们只需原地旋转
      if (latch_xy_goal_tolerance_) {
        xy_tolerance_latch_ = true;
      }

      //2）计算车辆当前朝向和目标朝向的偏差角
      double angle = getGoalOrientationAngleDifference(global_pose, goal_th);
      //3）检查是否达到目标方向
      if (fabs(angle) <= yaw_goal_tolerance_) //若达到目标角度，将速度命令设置为零
      {
        cmd_vel.linear.x = 0.0;
        cmd_vel.linear.y = 0.0;
        cmd_vel.angular.z = 0.0;
        rotating_to_goal_ = false;
        xy_tolerance_latch_ = false;
        reached_goal_ = true;
      }
      else  //若没有达到目标角度，进行重规划
      {
        //调用下两行以确保轨迹规划器更新其路径距离和目标距离网格
        tc_->updatePlan(transformed_plan);
        Trajectory path = tc_->findBestPath(global_pose, robot_vel, drive_cmds);
        map_viz_.publishCostCloud(costmap_);

        //复制里程计信息
        nav_msgs::Odometry base_odom;
        odom_helper_.getOdom(base_odom);

        //如果我们还没有停止。。。我们要停止。。。考虑到机器人的加速度限制
        if ( ! rotating_to_goal_ && !base_local_planner::stopped(base_odom, rot_stopped_velocity_, trans_stopped_velocity_)) 
        {
          if ( ! stopWithAccLimits(global_pose, robot_vel, cmd_vel)) {
            return false;
          }
        }
        //如果我们停止了。。。然后我们要旋转到目标
        else{
          rotating_to_goal_ = true;
          if(!rotateToGoal(global_pose, robot_vel, goal_th, cmd_vel)) {
            return false;
          }
        }
      }

      //4）发布一个空的计划，因为我们已经达到了目标位置
      publishPlan(transformed_plan, g_plan_pub_);
      publishPlan(local_plan, l_plan_pub_);

      //当我们旋转到目标时，我们实际上不想运行控制器
      return true;
    }

    //（7）更新一条全局路径出来
    tc_->updatePlan(transformed_plan);

    //（8）【核心】根据机器人的当前状态，找到一个好的轨迹，计算行驶轨迹robot_vel、drive_cmds，并转换成驱动器命令cmd_vel
    Trajectory path = tc_->findBestPath(global_pose, robot_vel, drive_cmds);
    cmd_vel.linear.x = drive_cmds.pose.position.x;
    cmd_vel.linear.y = drive_cmds.pose.position.y;
    cmd_vel.angular.z = tf2::getYaw(drive_cmds.pose.orientation);

    //（9）如果我们不能得到有效的局部路径，打印警告出来
    if (path.cost_ < 0) {
      ROS_DEBUG_NAMED("trajectory_planner_ros",
          "The rollout planner failed to find a valid plan. This means that the footprint of the robot was in collision for all simulated trajectories.");
      local_plan.clear();
      publishPlan(transformed_plan, g_plan_pub_);
      publishPlan(local_plan, l_plan_pub_);
      return false;
    }

    ROS_DEBUG_NAMED("trajectory_planner_ros", "A valid velocity command of (%.2f, %.2f, %.2f) was found for this cycle.",
        cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);

    //（10）填充局部规划local plan
    for (unsigned int i = 0; i < path.getPointsSize(); ++i) 
    {
      double p_x, p_y, p_th;
      path.getPoint(i, p_x, p_y, p_th);
      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = global_frame_;
      pose.header.stamp = ros::Time::now();
      pose.pose.position.x = p_x;
      pose.pose.position.y = p_y;
      pose.pose.position.z = 0.0;
      tf2::Quaternion q;
      q.setRPY(0, 0, p_th);
      tf2::convert(q, pose.pose.orientation);
      local_plan.push_back(pose);
    }

    //（11）发布代价地图costmap可视化数据
    map_viz_.publishCostCloud(costmap_);

    //（12）将global_plan、local_plan信息发布到可视化工具
    publishPlan(transformed_plan, g_plan_pub_);
    publishPlan(local_plan, l_plan_pub_);
    return true;
  }

  bool TrajectoryPlannerROS::checkTrajectory(double vx_samp, double vy_samp, double vtheta_samp, bool update_map){
    geometry_msgs::PoseStamped global_pose;
    if(costmap_ros_->getRobotPose(global_pose)){
      if(update_map){
        //we need to give the planne some sort of global plan, since we're only checking for legality
        //we'll just give the robots current position
        std::vector<geometry_msgs::PoseStamped> plan;
        plan.push_back(global_pose);
        tc_->updatePlan(plan, true);
      }

      //copy over the odometry information
      nav_msgs::Odometry base_odom;
      {
        boost::recursive_mutex::scoped_lock lock(odom_lock_);
        base_odom = base_odom_;
      }

      return tc_->checkTrajectory(global_pose.pose.position.x, global_pose.pose.position.y, tf2::getYaw(global_pose.pose.orientation),
          base_odom.twist.twist.linear.x,
          base_odom.twist.twist.linear.y,
          base_odom.twist.twist.angular.z, vx_samp, vy_samp, vtheta_samp);

    }
    ROS_WARN("Failed to get the pose of the robot. No trajectories will pass as legal in this case.");
    return false;
  }


  double TrajectoryPlannerROS::scoreTrajectory(double vx_samp, double vy_samp, double vtheta_samp, bool update_map){
    // Copy of checkTrajectory that returns a score instead of True / False
    geometry_msgs::PoseStamped global_pose;
    if(costmap_ros_->getRobotPose(global_pose)){
      if(update_map){
        //we need to give the planne some sort of global plan, since we're only checking for legality
        //we'll just give the robots current position
        std::vector<geometry_msgs::PoseStamped> plan;
        plan.push_back(global_pose);
        tc_->updatePlan(plan, true);
      }

      //copy over the odometry information
      nav_msgs::Odometry base_odom;
      {
        boost::recursive_mutex::scoped_lock lock(odom_lock_);
        base_odom = base_odom_;
      }

      return tc_->scoreTrajectory(global_pose.pose.position.x, global_pose.pose.position.y, tf2::getYaw(global_pose.pose.orientation),
          base_odom.twist.twist.linear.x,
          base_odom.twist.twist.linear.y,
          base_odom.twist.twist.angular.z, vx_samp, vy_samp, vtheta_samp);

    }
    ROS_WARN("Failed to get the pose of the robot. No trajectories will pass as legal in this case.");
    return -1.0;
  }

  bool TrajectoryPlannerROS::isGoalReached() {
    if (! isInitialized()) {
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }
    //return flag set in controller
    return reached_goal_; 
  }
};
