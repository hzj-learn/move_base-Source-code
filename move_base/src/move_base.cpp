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
*         Mike Phillips (put the planner in its own thread)
*********************************************************************/
#include <move_base/move_base.h>
#include <cmath>

#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>

#include <geometry_msgs/Twist.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace move_base {

  MoveBase::MoveBase(tf2_ros::Buffer& tf) :
    tf_(tf),
    as_(NULL),
    planner_costmap_ros_(NULL), controller_costmap_ros_(NULL),
    bgp_loader_("nav_core", "nav_core::BaseGlobalPlanner"),
    blp_loader_("nav_core", "nav_core::BaseLocalPlanner"), 
    recovery_loader_("nav_core", "nav_core::RecoveryBehavior"),
    planner_plan_(NULL), latest_plan_(NULL), controller_plan_(NULL),
    runPlanner_(false), setup_(false), p_freq_change_(false), c_freq_change_(false), new_global_plan_(false) 
    {
      //（0）定义服务是用于加载导航目标并完成导航//////////////
    as_ = new MoveBaseActionServer(ros::NodeHandle(), "move_base", boost::bind(&MoveBase::executeCb, this, _1), false);

    ros::NodeHandle private_nh("~");
    ros::NodeHandle nh;

    recovery_trigger_ = PLANNING_R; //重恢复行为处于planning状态

    //获取一些对move base节点全局规划的参数
    std::string global_planner, local_planner;
    private_nh.param("base_global_planner", global_planner, std::string("navfn/NavfnROS"));                         //全局规划器选用NavfnROS
    private_nh.param("base_local_planner", local_planner, std::string("base_local_planner/TrajectoryPlannerROS"));  //局部规划器选用base_local_planner
    private_nh.param("global_costmap/robot_base_frame", robot_base_frame_, std::string("base_link"));
    private_nh.param("global_costmap/global_frame", global_frame_, std::string("map"));
    private_nh.param("planner_frequency", planner_frequency_, 0.0);                                                 //设置规划的运行频率
    private_nh.param("controller_frequency", controller_frequency_, 20.0);
    private_nh.param("planner_patience", planner_patience_, 5.0);
    private_nh.param("controller_patience", controller_patience_, 15.0);
    private_nh.param("max_planning_retries", max_planning_retries_, -1);  // disabled by default
    private_nh.param("oscillation_timeout", oscillation_timeout_, 0.0);
    private_nh.param("oscillation_distance", oscillation_distance_, 0.5);

    //设置make_plan服务的参数
    private_nh.param("make_plan_clear_costmap", make_plan_clear_costmap_, true);
    private_nh.param("make_plan_add_unreachable_goal", make_plan_add_unreachable_goal_, true);

    //设置 plan triple buffer
    planner_plan_ = new std::vector<geometry_msgs::PoseStamped>();    //全局规划的路径点
    latest_plan_ = new std::vector<geometry_msgs::PoseStamped>();     //
    controller_plan_ = new std::vector<geometry_msgs::PoseStamped>(); //局部规划的路径点

    //(1)开一个线程专门用于做局部规划
    planner_thread_ = new boost::thread(boost::bind(&MoveBase::planThread, this));///////////////////////////////////////////////

    //（2）发布话题，用于规划的输出
    vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    current_goal_pub_ = private_nh.advertise<geometry_msgs::PoseStamped>("current_goal", 0 );
    ros::NodeHandle action_nh("move_base");
    action_goal_pub_ = action_nh.advertise<move_base_msgs::MoveBaseActionGoal>("goal", 1);

    //（3）订阅者，订阅rviz中以posestamp消息的形式发送目标，并将posestamp包装到操作消息中并重新发送到服务器
    ros::NodeHandle simple_nh("move_base_simple");
    goal_sub_ = simple_nh.subscribe<geometry_msgs::PoseStamped>("goal", 1, boost::bind(&MoveBase::goalCB, this, _1));

    //我们假设机器人的半径与costmaps指定的半径一致
    private_nh.param("local_costmap/inscribed_radius", inscribed_radius_, 0.325);
    private_nh.param("local_costmap/circumscribed_radius", circumscribed_radius_, 0.46);
    private_nh.param("clearing_radius", clearing_radius_, circumscribed_radius_);
    private_nh.param("conservative_reset_dist", conservative_reset_dist_, 3.0);

    private_nh.param("shutdown_costmaps", shutdown_costmaps_, false);
    private_nh.param("clearing_rotation_allowed", clearing_rotation_allowed_, true);
    private_nh.param("recovery_behavior_enabled", recovery_behavior_enabled_, true);

    //为planner's costmap创建ros包装（ros wrapper），初始化一个指针，我们将使用它作为基础映射
    planner_costmap_ros_ = new costmap_2d::Costmap2DROS("global_costmap", tf_);
    planner_costmap_ros_->pause();
    //（3）初始化全局规划器
    try {
      planner_ = bgp_loader_.createInstance(global_planner);
      planner_->initialize(bgp_loader_.getName(global_planner), planner_costmap_ros_);
    } catch (const pluginlib::PluginlibException& ex) {
      ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", global_planner.c_str(), ex.what());
      exit(1);
    }

  
    //为controller's costmap创建ros包装（ros wrapper），初始化一个指针，我们将使用它作为基础映射
    controller_costmap_ros_ = new costmap_2d::Costmap2DROS("local_costmap", tf_);
    controller_costmap_ros_->pause();
    //（4）初始化局部规划器
    try {
      tc_ = blp_loader_.createInstance(local_planner);
      ROS_INFO("Created local_planner %s", local_planner.c_str());
      tc_->initialize(blp_loader_.getName(local_planner), &tf_, controller_costmap_ros_);
    } catch (const pluginlib::PluginlibException& ex) {
      ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", local_planner.c_str(), ex.what());
      exit(1);
    }

    // （5）启动全局规划器和局部规划器的代价地图costmap
    planner_costmap_ros_->start();
    controller_costmap_ros_->start();

    //（6）发布一个service 用于返回获取一个planning的路径
    make_plan_srv_ = private_nh.advertiseService("make_plan", &MoveBase::planService, this);

    //（7）发布一个 service用于清除代价地图clearing the costmaps
    clear_costmaps_srv_ = private_nh.advertiseService("clear_costmaps", &MoveBase::clearCostmapsService, this);

    //（8）关闭代价地图的更新，shutdown_costmaps_
    if(shutdown_costmaps_){
      ROS_DEBUG_NAMED("move_base","Stopping costmaps initially");
      planner_costmap_ros_->stop();
      controller_costmap_ros_->stop();
    }

    //（9）加载任何用户指定的恢复行为，如果失败，则加载默认值
    if(!loadRecoveryBehaviors(private_nh)){
      loadDefaultRecoveryBehaviors();
    }

    //（10）我们都准备好了，可以启动动作服务器(action server)
    //首先，我们需要制定一个plan
    state_ = PLANNING;
    //我们将从列表的开头开始执行恢复行为
    recovery_index_ = 0;
    as_->start();

    //（11）动态参数配置
    dsrv_ = new dynamic_reconfigure::Server<move_base::MoveBaseConfig>(ros::NodeHandle("~"));
    dynamic_reconfigure::Server<move_base::MoveBaseConfig>::CallbackType cb = boost::bind(&MoveBase::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);
  }

//使用动态参数服务器，进行参数的动态配置
  void MoveBase::reconfigureCB(move_base::MoveBaseConfig &config, uint32_t level){
    boost::recursive_mutex::scoped_lock l(configuration_mutex_);
    //判断参数是否要进行重新设置
    if(!setup_)
    {
      last_config_ = config;
      default_config_ = config;
      setup_ = true;
      return;
    }

    if(config.restore_defaults) {
      config = default_config_;
      //if someone sets restore defaults on the parameter server, prevent looping
      config.restore_defaults = false;
    }

    //（1）更改全局规划的频率
    if(planner_frequency_ != config.planner_frequency)
    {
      planner_frequency_ = config.planner_frequency;
      p_freq_change_ = true;
    }

    //（2）更改局部规划的频率
    if(controller_frequency_ != config.controller_frequency)
    {
      controller_frequency_ = config.controller_frequency;
      c_freq_change_ = true;
    }

    planner_patience_ = config.planner_patience;
    controller_patience_ = config.controller_patience;
    max_planning_retries_ = config.max_planning_retries;
    conservative_reset_dist_ = config.conservative_reset_dist;

    recovery_behavior_enabled_ = config.recovery_behavior_enabled;
    clearing_rotation_allowed_ = config.clearing_rotation_allowed;
    shutdown_costmaps_ = config.shutdown_costmaps;

    oscillation_timeout_ = config.oscillation_timeout;
    oscillation_distance_ = config.oscillation_distance;

    //（3）更改现在使用的全局规划器
    if(config.base_global_planner != last_config_.base_global_planner) {
      boost::shared_ptr<nav_core::BaseGlobalPlanner> old_planner = planner_;
      //initialize the global planner
      ROS_INFO("Loading global planner %s", config.base_global_planner.c_str());
      try {
        planner_ = bgp_loader_.createInstance(config.base_global_planner);

        // wait for the current planner to finish planning
        boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);

        // Clean up before initializing the new planner
        planner_plan_->clear();
        latest_plan_->clear();
        controller_plan_->clear();
        resetState();
        planner_->initialize(bgp_loader_.getName(config.base_global_planner), planner_costmap_ros_);

        lock.unlock();
      } catch (const pluginlib::PluginlibException& ex) {
        ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the \
                   containing library is built? Exception: %s", config.base_global_planner.c_str(), ex.what());
        planner_ = old_planner;
        config.base_global_planner = last_config_.base_global_planner;
      }
    }

    //（4）更改现在使用的局部规划器
    if(config.base_local_planner != last_config_.base_local_planner){
      boost::shared_ptr<nav_core::BaseLocalPlanner> old_planner = tc_;
      //create a local planner
      try {
        tc_ = blp_loader_.createInstance(config.base_local_planner);
        // Clean up before initializing the new planner
        planner_plan_->clear();
        latest_plan_->clear();
        controller_plan_->clear();
        resetState();
        tc_->initialize(blp_loader_.getName(config.base_local_planner), &tf_, controller_costmap_ros_);
      } catch (const pluginlib::PluginlibException& ex) {
        ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the \
                   containing library is built? Exception: %s", config.base_local_planner.c_str(), ex.what());
        tc_ = old_planner;
        config.base_local_planner = last_config_.base_local_planner;
      }
    }

    make_plan_clear_costmap_ = config.make_plan_clear_costmap;
    make_plan_add_unreachable_goal_ = config.make_plan_add_unreachable_goal;

    last_config_ = config;
  }


//接收目标位置，并将posestamp包装到操作消息中并重新发送到服务器
  void MoveBase::goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal){
    ROS_DEBUG_NAMED("move_base","In ROS goal callback, wrapping the PoseStamped in the action message and re-sending to the server.");
    move_base_msgs::MoveBaseActionGoal action_goal;
    action_goal.header.stamp = ros::Time::now();
    action_goal.goal.target_pose = *goal;

    action_goal_pub_.publish(action_goal);
  }

//清除机器人范围内代价地图的障碍物
  void MoveBase::clearCostmapWindows(double size_x, double size_y){
    geometry_msgs::PoseStamped global_pose;

    //clear the planner's costmap
    getRobotPose(global_pose, planner_costmap_ros_);

    std::vector<geometry_msgs::Point> clear_poly;
    double x = global_pose.pose.position.x;
    double y = global_pose.pose.position.y;
    geometry_msgs::Point pt;

    pt.x = x - size_x / 2;
    pt.y = y - size_y / 2;
    clear_poly.push_back(pt);

    pt.x = x + size_x / 2;
    pt.y = y - size_y / 2;
    clear_poly.push_back(pt);

    pt.x = x + size_x / 2;
    pt.y = y + size_y / 2;
    clear_poly.push_back(pt);

    pt.x = x - size_x / 2;
    pt.y = y + size_y / 2;
    clear_poly.push_back(pt);

    planner_costmap_ros_->getCostmap()->setConvexPolygonCost(clear_poly, costmap_2d::FREE_SPACE);

    //clear the controller's costmap
    getRobotPose(global_pose, controller_costmap_ros_);

    clear_poly.clear();
    x = global_pose.pose.position.x;
    y = global_pose.pose.position.y;

    pt.x = x - size_x / 2;
    pt.y = y - size_y / 2;
    clear_poly.push_back(pt);

    pt.x = x + size_x / 2;
    pt.y = y - size_y / 2;
    clear_poly.push_back(pt);

    pt.x = x + size_x / 2;
    pt.y = y + size_y / 2;
    clear_poly.push_back(pt);

    pt.x = x - size_x / 2;
    pt.y = y + size_y / 2;
    clear_poly.push_back(pt);

    controller_costmap_ros_->getCostmap()->setConvexPolygonCost(clear_poly, costmap_2d::FREE_SPACE);
  }

//【执行清除代价地图的障碍物的服务】
  bool MoveBase::clearCostmapsService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp){
    //（1）调用costmap功能包，清除局部代价地图
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock_controller(*(controller_costmap_ros_->getCostmap()->getMutex()));
    controller_costmap_ros_->resetLayers();

    //（2）调用costmap功能包，清除全局代价地图
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock_planner(*(planner_costmap_ros_->getCostmap()->getMutex()));
    planner_costmap_ros_->resetLayers();
    return true;
  }

//用于获取一个planning的路径
//【当操作处于非活动状态(action)时可以进行的服务调用，它将返回一个plan路径】
  bool MoveBase::planService(nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &resp){
    //确保是已经打开了move_base节点
    if(as_->isActive()){
      ROS_ERROR("move_base must be in an inactive state to make a plan for an external user");
      return false;
    }
    //确保我们有提供到代价地图
    if(planner_costmap_ros_ == NULL){
      ROS_ERROR("move_base cannot make a plan for you because it doesn't have a costmap");
      return false;
    }

    //（1）指定开始位姿（如果用户没有指定开始位姿，则使用机器人空帧id标识的开始位姿）
    geometry_msgs::PoseStamped start;
    if(req.start.header.frame_id.empty())
    {
        geometry_msgs::PoseStamped global_pose;
        if(!getRobotPose(global_pose, planner_costmap_ros_)){
          ROS_ERROR("move_base cannot make a plan for you because it could not get the start pose of the robot");
          return false;
        }
        start = global_pose;
    }
    else
    {
        start = req.start;
    }

    //（2）更新规划器使用的代价地图
    if (make_plan_clear_costmap_) 
    {
      clearCostmapWindows(2 * clearing_radius_, 2 * clearing_radius_);
    }

    //（3）试着为确切的目标制定一个全局规划器，获取一个全局路径
    std::vector<geometry_msgs::PoseStamped> global_plan;
    if(!planner_->makePlan(start, req.goal, global_plan) || global_plan.empty())
    {
      //第一次规划不成功的时候，才会扩大范围无搜索
      ROS_DEBUG_NAMED("move_base","Failed to find a plan to exact goal of (%.2f, %.2f), searching for a feasible goal within tolerance", 
          req.goal.pose.position.x, req.goal.pose.position.y);

      //向外搜索指定公差范围内的可行目标，并进行全局路径规划
      geometry_msgs::PoseStamped p;
      p = req.goal;
      bool found_legal = false;
      float resolution = planner_costmap_ros_->getCostmap()->getResolution();
      float search_increment = resolution*3.0;
      if(req.tolerance > 0.0 && req.tolerance < search_increment) search_increment = req.tolerance;
      for(float max_offset = search_increment; max_offset <= req.tolerance && !found_legal; max_offset += search_increment) {
        for(float y_offset = 0; y_offset <= max_offset && !found_legal; y_offset += search_increment) {
          for(float x_offset = 0; x_offset <= max_offset && !found_legal; x_offset += search_increment) {

            //don't search again inside the current outer layer
            if(x_offset < max_offset-1e-9 && y_offset < max_offset-1e-9) continue;

            //search to both sides of the desired goal
            for(float y_mult = -1.0; y_mult <= 1.0 + 1e-9 && !found_legal; y_mult += 2.0) {

              //if one of the offsets is 0, -1*0 is still 0 (so get rid of one of the two)
              if(y_offset < 1e-9 && y_mult < -1.0 + 1e-9) continue;

              for(float x_mult = -1.0; x_mult <= 1.0 + 1e-9 && !found_legal; x_mult += 2.0) {
                if(x_offset < 1e-9 && x_mult < -1.0 + 1e-9) continue;

                p.pose.position.y = req.goal.pose.position.y + y_offset * y_mult;
                p.pose.position.x = req.goal.pose.position.x + x_offset * x_mult;

                if(planner_->makePlan(start, p, global_plan)){
                  if(!global_plan.empty()){

                    if (make_plan_add_unreachable_goal_) {
                      //adding the (unreachable) original goal to the end of the global plan, in case the local planner can get you there
                      //(the reachable goal should have been added by the global planner)
                      global_plan.push_back(req.goal);
                    }

                    found_legal = true;
                    ROS_DEBUG_NAMED("move_base", "Found a plan to point (%.2f, %.2f)", p.pose.position.x, p.pose.position.y);
                    break;
                  }
                }
                else{
                  ROS_DEBUG_NAMED("move_base","Failed to find a plan to point (%.2f, %.2f)", p.pose.position.x, p.pose.position.y);
                }
              }
            }
          }
        }
      }
    }

    //（4）把路径复制成一条信息，然后发送出去
    resp.plan.poses.resize(global_plan.size());
    for(unsigned int i = 0; i < global_plan.size(); ++i){
      resp.plan.poses[i] = global_plan[i];
    }

    return true;
  }

  MoveBase::~MoveBase(){
    recovery_behaviors_.clear();

    delete dsrv_;

    if(as_ != NULL)
      delete as_;

    if(planner_costmap_ros_ != NULL)
      delete planner_costmap_ros_;

    if(controller_costmap_ros_ != NULL)
      delete controller_costmap_ros_;

    planner_thread_->interrupt();
    planner_thread_->join();

    delete planner_thread_;

    delete planner_plan_;
    delete latest_plan_;
    delete controller_plan_;

    planner_.reset();
    tc_.reset();
  }



//【一个套壳来的，，规划部分的流程核心-包括进行全局规划和局部规划】进行一个新的目标路径目标点的规划，得到一系列的路径点
  bool MoveBase::makePlan(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){

    //(1)获取二维的代价地图
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(planner_costmap_ros_->getCostmap()->getMutex()));

    //(2)确保最初将全局规划设置为空
    plan.clear();

    //(3)判断二维代价地图插件时候为空
    if(planner_costmap_ros_ == NULL) {
      ROS_ERROR("Planner costmap ROS is NULL, unable to create global plan");
      return false;
    }

    //(4)获取机器人的起始位姿
    geometry_msgs::PoseStamped global_pose;
    if(!getRobotPose(global_pose, planner_costmap_ros_)) {
      ROS_WARN("Unable to get starting pose of robot, unable to create global plan");
      return false;
    }

    const geometry_msgs::PoseStamped& start = global_pose;

    //(5)如果规划器失败或返回零长度规划结果，则表示规划失败
    //【核心在这里】
    if(!planner_->makePlan(start, goal, plan) || plan.empty()){
      ROS_DEBUG_NAMED("move_base","Failed to find a  plan to point (%.2f, %.2f)", goal.pose.position.x, goal.pose.position.y);
      return false;
    }

    return true;
  }



//向机器人发布零速度命令
  void MoveBase::publishZeroVelocity(){
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
    vel_pub_.publish(cmd_vel);
  }

//检查有没有收到目标值，并检查目标值是否合法
  bool MoveBase::isQuaternionValid(const geometry_msgs::Quaternion& q){

    //first we need to check if the quaternion has nan's or infs
    if(!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w)){
      ROS_ERROR("Quaternion has nans or infs... discarding as a navigation goal");
      return false;
    }

    tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);

    //next, we need to check if the length of the quaternion is close to zero
    if(tf_q.length2() < 1e-6){
      ROS_ERROR("Quaternion has length close to zero... discarding as navigation goal");
      return false;
    }

    //next, we'll normalize the quaternion and check that it transforms the vertical vector correctly
    tf_q.normalize();

    tf2::Vector3 up(0, 0, 1);

    double dot = up.dot(up.rotate(tf_q.getAxis(), tf_q.getAngle()));

    if(fabs(dot - 1) > 1e-3){
      ROS_ERROR("Quaternion is invalid... for navigation the z-axis of the quaternion must be close to vertical.");
      return false;
    }

    return true;
  }

//把目标饭店转成成全局坐标来表示
  geometry_msgs::PoseStamped MoveBase::goalToGlobalFrame(const geometry_msgs::PoseStamped& goal_pose_msg){
    std::string global_frame = planner_costmap_ros_->getGlobalFrameID();
    geometry_msgs::PoseStamped goal_pose, global_pose;
    goal_pose = goal_pose_msg;

    //just get the latest available transform... for accuracy they should send
    //goals in the frame of the planner
    goal_pose.header.stamp = ros::Time();

    try{
      tf_.transform(goal_pose_msg, global_pose, global_frame);
    }
    catch(tf2::TransformException& ex){
      ROS_WARN("Failed to transform the goal pose from %s into the %s frame: %s",
          goal_pose.header.frame_id.c_str(), global_frame.c_str(), ex.what());
      return goal_pose_msg;
    }

    return global_pose;
  }

  void MoveBase::wakePlanner(const ros::TimerEvent& event)
  {
    // we have slept long enough for rate
    planner_cond_.notify_one();
  }




//【规划部分流程核心】开了一个线程专门用于做规划
  void MoveBase::planThread(){

    ROS_DEBUG_NAMED("move_base_plan_thread","Starting planner thread...");
    ros::NodeHandle n;
    ros::Timer timer;
    bool wait_for_wake = false;
    boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);

    while(n.ok()){
      //（1）检查是否开启了规划器标志位，或者唤醒规划器的标志位，开启了就进行规划，否则就进行等待
      while(wait_for_wake || !runPlanner_){
                                           //如果我们不应该运行规划器，那么挂起这个线程
        ROS_DEBUG_NAMED("move_base_plan_thread","Planner thread is suspending");
        planner_cond_.wait(lock);
        wait_for_wake = false;
      }
      ros::Time start_time = ros::Time::now();

      //（2）获取目标点并解锁互斥锁
      geometry_msgs::PoseStamped temp_goal = planner_goal_;
      lock.unlock();
      ROS_DEBUG_NAMED("move_base_plan_thread","Planning...");

      //（3）运行规划器makePlan【局部规划，具体是转到插件取实现的】
      planner_plan_->clear();
      bool gotPlan = n.ok() && makePlan(temp_goal, *planner_plan_);

      //（4）若规划器进行规划成功，拿到规划出来的路径，得到一系列的规划出来的point点，启动控制器进行控制【标志位实现】
      if(gotPlan)
      {
        ROS_DEBUG_NAMED("move_base_plan_thread","Got Plan with %zu points!", planner_plan_->size());
        //pointer swap the plans under mutex (the controller will pull from latest_plan_)
        std::vector<geometry_msgs::PoseStamped>* temp_plan = planner_plan_;

        lock.lock();
        planner_plan_ = latest_plan_;
        latest_plan_ = temp_plan;
        last_valid_plan_ = ros::Time::now();
        planning_retries_ = 0;
        new_global_plan_ = true;

        ROS_DEBUG_NAMED("move_base_plan_thread","Generated a plan from the base_global_planner");

        //确保我们只有在还没有达到目标时才启动控制器
        if(runPlanner_)
          state_ = CONTROLLING;
        if(planner_frequency_ <= 0)
          runPlanner_ = false;
        lock.unlock();
      }

     //（5）若规划器进行规划不成功，而我们处于计划状态（机器人没有移动），继续进行规划，直到超过规划时间或者超过规划的次数，停止规划
      else if(state_==PLANNING){
        ROS_DEBUG_NAMED("move_base_plan_thread","No Plan...");
        ros::Time attempt_end = last_valid_plan_ + ros::Duration(planner_patience_);
        lock.lock();
        planning_retries_++;
        //检查是否超过时间限制或最大重试次数制定计划
        if(runPlanner_ &&
           (ros::Time::now() > attempt_end || planning_retries_ > uint32_t(max_planning_retries_))){
          //we'll move into our obstacle clearing mode
          state_ = CLEARING;
          runPlanner_ = false;  // proper solution for issue #523
          publishZeroVelocity();
          recovery_trigger_ = PLANNING_R;
        }

        lock.unlock();
      }

      //（6）在下一次迭代中使用互斥锁
      lock.lock();

      //（7）如果需要，设置睡眠界面
      if(planner_frequency_ > 0){
        ros::Duration sleep_time = (start_time + ros::Duration(1.0/planner_frequency_)) - ros::Time::now();
        if (sleep_time > ros::Duration(0.0)){
          wait_for_wake = true;
          timer = n.createTimer(sleep_time, &MoveBase::wakePlanner, this);
        }
      }
    }
  }


//【核心服务-主的】用于加载导航目标并完成导航
  void MoveBase::executeCb(const move_base_msgs::MoveBaseGoalConstPtr& move_base_goal)
  {
  //服务初始化部分
    //（0）【执行的前提条件】在收到全局目标点后，executeCb服务才会被激活
    if(!isQuaternionValid(move_base_goal->target_pose.pose.orientation)){
      as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on goal because it was sent with an invalid quaternion");
      return;
    }

    //（1）把目标点转成成全局坐标来表示
    geometry_msgs::PoseStamped goal = goalToGlobalFrame(move_base_goal->target_pose);
  
    //（2）先让车停下来做规划  
    publishZeroVelocity();

    //（3）当获取到目标点，开始做规划planning【标志位实现】
    boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
    planner_goal_ = goal;      //给规划器目标点
    runPlanner_ = true;        //启动规划器【标志位】
    planner_cond_.notify_one();
    lock.unlock();

    //（4）把当前的目标点发送给rviz
    current_goal_pub_.publish(goal);     
    std::vector<geometry_msgs::PoseStamped> global_plan;  //规划出来的目标点
    ros::Rate r(controller_frequency_);                   //设置局部规划的频率

    //（5）若全局和局部的代价地图都关闭了，就启动全局和局部的代价地图
    if(shutdown_costmaps_){
      ROS_DEBUG_NAMED("move_base","Starting up costmaps that were shut down previously");
      planner_costmap_ros_->start();
      controller_costmap_ros_->start();
    }

    //（6）重新设置上次我们有一个有效的计划和控制时间戳
    last_valid_control_ = ros::Time::now();
    last_valid_plan_ = ros::Time::now();
    last_oscillation_reset_ = ros::Time::now();
    planning_retries_ = 0;

//服务运行部分
    ros::NodeHandle n;
    while(n.ok())
    {
      //(1)判断是否要改变局部规划器的控制频率
      if(c_freq_change_)
      {
        ROS_INFO("Setting controller frequency to %.2f", controller_frequency_);
        r = ros::Rate(controller_frequency_);
        c_freq_change_ = false;
      }

      //（2）判断新的全局目标点是否有效的，若有效启动全局规划器planner ，若无效退出程序
      if(as_->isPreemptRequested()){    
      
        if(as_->isNewGoalAvailable()){  //新的全局目标点是有效的
          //获取全局目标点
          move_base_msgs::MoveBaseGoal new_goal = *as_->acceptNewGoal();
          if(!isQuaternionValid(new_goal.target_pose.pose.orientation))
          {
            as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on goal because it was sent with an invalid quaternion");
            return;
          }
          goal = goalToGlobalFrame(new_goal.target_pose);

          //我们将确保为下一个执行周期重置状态
          recovery_index_ = 0;
          state_ = PLANNING;

          //启动全局规划器planner 
          lock.lock();
          planner_goal_ = goal;
          runPlanner_ = true;
          planner_cond_.notify_one();
          lock.unlock();

          //将目标点发布到可视化工具rviz
          ROS_DEBUG_NAMED("move_base","move_base has received a goal of x: %.2f, y: %.2f", goal.pose.position.x, goal.pose.position.y);
          current_goal_pub_.publish(goal);

          //重置我们的超时和计数器
          last_valid_control_ = ros::Time::now();
          last_valid_plan_ = ros::Time::now();
          last_oscillation_reset_ = ros::Time::now();
          planning_retries_ = 0;
        }
        else {                          //新的目标值是无效的，
          //重置移动基础动作的状态，并向机器人发送零速度命令
          resetState();

          //notify the ActionServer that we've successfully preempted
          ROS_DEBUG_NAMED("move_base","Move base preempting the current goal");
          as_->setPreempted();

          //we'll actually return from execute after preempting
          return;
        }
      }

      //（3）判断是否已经更改了全局坐标系，若有是，重新加载全局目标点并启动全局规划器planner
      if(goal.header.frame_id != planner_costmap_ros_->getGlobalFrameID()){
        goal = goalToGlobalFrame(goal);

        //we want to go back to the planning state for the next execution cycle
        recovery_index_ = 0;
        state_ = PLANNING;

        //we have a new goal so make sure the planner is awake
        lock.lock();
        planner_goal_ = goal;
        runPlanner_ = true;
        planner_cond_.notify_one();
        lock.unlock();

        //publish the goal point to the visualizer
        ROS_DEBUG_NAMED("move_base","The global frame for move_base has changed, new frame: %s, new goal position x: %.2f, y: %.2f", goal.header.frame_id.c_str(), goal.pose.position.x, goal.pose.position.y);
        current_goal_pub_.publish(goal);

        //make sure to reset our timeouts and counters
        last_valid_control_ = ros::Time::now();
        last_valid_plan_ = ros::Time::now();
        last_oscillation_reset_ = ros::Time::now();
        planning_retries_ = 0;
      }

      //for timing that gives real time even in simulation
      ros::WallTime start = ros::WallTime::now();

      //（4）【执行规划追踪目标闭环控制，真正干活的地方】the real work on pursuing a goal is done here
      bool done = executeCycle(goal, global_plan);

      //（5）若正常执行完规划追踪目标的闭环控制任务，在这里退出程序
      if(done)
        return;

      //check if execution of the goal has completed in some way

      ros::WallDuration t_diff = ros::WallTime::now() - start;
      ROS_DEBUG_NAMED("move_base","Full control cycle time: %.9f\n", t_diff.toSec());

      r.sleep();
      //make sure to sleep for the remainder of our cycle time
      if(r.cycleTime() > ros::Duration(1 / controller_frequency_) && state_ == CONTROLLING)
        ROS_WARN("Control loop missed its desired rate of %.4fHz... the loop actually took %.4f seconds", controller_frequency_, r.cycleTime().toSec());
    }

    //唤醒planner线程，以便它可以干净地退出
    lock.lock();
    runPlanner_ = true;
    planner_cond_.notify_one();
    lock.unlock();

    //如果节点被杀死，我们将中止并返回
    as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on the goal because the node has been killed");
    return;
  }

  double MoveBase::distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2)
  {
    return hypot(p1.pose.position.x - p2.pose.position.x, p1.pose.position.y - p2.pose.position.y);
  }


//【核心控制部分】【执行规划追踪目标闭环控制】
  bool MoveBase::executeCycle(geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& global_plan)
  {
    boost::recursive_mutex::scoped_lock ecl(configuration_mutex_);
    //（1）定义控制的twist指令
    geometry_msgs::Twist cmd_vel;

    //（2、3）获取机器人在全局坐标系下的当前位置,并发布机器人的当前位置给规划器
    geometry_msgs::PoseStamped global_pose;
    getRobotPose(global_pose, planner_costmap_ros_);                  
    const geometry_msgs::PoseStamped& current_position = global_pose;

    move_base_msgs::MoveBaseFeedback feedback;
    feedback.base_position = current_position;
    as_->publishFeedback(feedback);

    //（4）看看我们是否移动到足够远的地方，重置振荡超时（避免车辆来回震荡的）
    if(distance(current_position, oscillation_pose_) >= oscillation_distance_)
    {
      last_oscillation_reset_ = ros::Time::now();
      oscillation_pose_ = current_position;

      //如果我们上一次的恢复是由振荡引起的，我们想重新设置恢复指数
      if(recovery_trigger_ == OSCILLATION_R)
        recovery_index_ = 0;
    }

    //（5）检查costmap是否是最新的
    if(!controller_costmap_ros_->isCurrent()){
      ROS_WARN("[%s]:Sensor data is out of date, we're not going to allow commanding of the base for safety",ros::this_node::getName().c_str());
      publishZeroVelocity();
      return false;
    }

    //（6）如果我们有一个新全局规划路径 ，那就把全局路径交给局部规划器，并设置重恢复行为索引
    if(new_global_plan_){
      
      new_global_plan_ = false; //重置新的标志位为0

      ROS_DEBUG_NAMED("move_base","Got a new plan...swap pointers");

      //在互斥下进行全局路径指针的交换
      std::vector<geometry_msgs::PoseStamped>* temp_plan = controller_plan_;
      boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
      controller_plan_ = latest_plan_;    //最终的plan
      latest_plan_ = temp_plan;           //当前暂时的plan
      lock.unlock();
      ROS_DEBUG_NAMED("move_base","pointers swapped!");

      //设置局部规划器遵循的全局路径
      if(!tc_->setPlan(*controller_plan_)){//若规划器没有进行初始化，报错停止
        //ABORT and SHUTDOWN COSTMAPS
        ROS_ERROR("Failed to pass global plan to the controller, aborting.");
        resetState();

        //disable the planner thread
        lock.lock();
        runPlanner_ = false;
        lock.unlock();

        as_->setAborted(move_base_msgs::MoveBaseResult(), "Failed to pass global plan to the controller.");
        return true;
      }

      //设置重恢复行为索引
      if(recovery_trigger_ == PLANNING_R)
        recovery_index_ = 0;
    }

    //（7）运行 move_base 的有限状态机, 用于处理导航的控制逻辑
    switch(state_){
 
      case PLANNING:  //1）全局规划PLANNING状态，我们将尝试规划一条全局路径
        {
          boost::recursive_mutex::scoped_lock lock(planner_mutex_);//在planner_mutex_进行规划的
          runPlanner_ = true;
          planner_cond_.notify_one();
        }
        ROS_DEBUG_NAMED("move_base","Waiting for plan, in the planning state.");
        break;

      case CONTROLLING: //2）局部规划控制状态CONTROLLING，因为局部规划是包含控制的，我们会试图找到有效的速度twist命令
        ROS_DEBUG_NAMED("move_base","In controlling state.");

        //检查我们是否达到了目标，停止所有的操作
        if(tc_->isGoalReached()){
          ROS_DEBUG_NAMED("move_base","Goal reached!");
          resetState();

          //disable the planner thread
          boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
          runPlanner_ = false;
          lock.unlock();

          as_->setSucceeded(move_base_msgs::MoveBaseResult(), "Goal reached.");
          return true;
        }

        //检查有无振荡情况，把车辆停止下来,启动地图清除状态
        if(oscillation_timeout_ > 0.0 &&
            last_oscillation_reset_ + ros::Duration(oscillation_timeout_) < ros::Time::now())
        {
          publishZeroVelocity();
          state_ = CLEARING;
          recovery_trigger_ = OSCILLATION_R;
        }
        
        {
         boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(controller_costmap_ros_->getCostmap()->getMutex()));
        
        //车子震荡的时候，通过局局路径规划得到控制指令
        if(tc_->computeVelocityCommands(cmd_vel)){//若能通过局部规划计算得到控制车辆的速度指令
          ROS_DEBUG_NAMED( "move_base", "Got a valid command from the local planner: %.3lf, %.3lf, %.3lf",
                           cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z );
          last_valid_control_ = ros::Time::now();
          //make sure that we send the velocity command to the base
          vel_pub_.publish(cmd_vel);
          if(recovery_trigger_ == CONTROLLING_R)
            recovery_index_ = 0;
        }
        else {                                    //若不能通过局部规划计算得到控制车辆的速度指令，开启局部规划线程继续寻找，知道规划超时
          ROS_DEBUG_NAMED("move_base", "The local planner could not find a valid plan.");
          ros::Time attempt_end = last_valid_control_ + ros::Duration(controller_patience_);

          //检查我们是否超过局部规划时间限制
          if(ros::Time::now() > attempt_end){
            //我们将进入我们的障碍清除模式
            publishZeroVelocity();
            state_ = CLEARING;
            recovery_trigger_ = CONTROLLING_R;
          }
          else{
            //我们将进入我们的重规划模式
            last_valid_plan_ = ros::Time::now();
            planning_retries_ = 0;
            state_ = PLANNING;
            publishZeroVelocity();

            //启用规划器线程，以防它不在时钟上运行
            boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
            runPlanner_ = true;
            planner_cond_.notify_one();
            lock.unlock();
          }
        }
        }

        break;

     
      case CLEARING://3)地图清除状态CLEARING，我们将尝试用任何用户提供的恢复行为来清除空间
        ROS_DEBUG_NAMED("move_base","In clearing/recovery state");

        //启用重恢复行为【重恢复行为是一个状态机流程】
        if(recovery_behavior_enabled_ && recovery_index_ < recovery_behaviors_.size()){//1）启动了重恢复行为，且重恢复状态没有执行完所有的操作，每个操作依次执行
          ROS_DEBUG_NAMED("move_base_recovery","Executing behavior %u of %zu", recovery_index_, recovery_behaviors_.size());
          
          //启动重恢复行为
          recovery_behaviors_[recovery_index_]->runBehavior();

          //我们至少要给机器人一点时间，让它在完成动作后停止摆动
          last_oscillation_reset_ = ros::Time::now();

          //检查恢复行为是否有效
          ROS_DEBUG_NAMED("move_base_recovery","Going back to planning state");
          last_valid_plan_ = ros::Time::now();
          planning_retries_ = 0;
          state_ = PLANNING;

          //更新我们将尝试的下一个恢复行为的索引
          recovery_index_++;
        }
        else{                                                                          //2）启动了重恢复行为，且重恢复状态已经执行完所有的操作，证明重恢复已经没有用了，退出导航
          ROS_DEBUG_NAMED("move_base_recovery","All recovery behaviors have failed, locking the planner and disabling it.");
          //把规划的线程也关了，因为已经无药可救了
          boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
          runPlanner_ = false;
          lock.unlock();

          ROS_DEBUG_NAMED("move_base_recovery","Something should abort after this.");

          if(recovery_trigger_ == CONTROLLING_R){
            ROS_ERROR("Aborting because a valid control could not be found. Even after executing all recovery behaviors");
            as_->setAborted(move_base_msgs::MoveBaseResult(), "Failed to find a valid control. Even after executing recovery behaviors.");
          }
          else if(recovery_trigger_ == PLANNING_R){
            ROS_ERROR("Aborting because a valid plan could not be found. Even after executing all recovery behaviors");
            as_->setAborted(move_base_msgs::MoveBaseResult(), "Failed to find a valid plan. Even after executing recovery behaviors.");
          }
          else if(recovery_trigger_ == OSCILLATION_R){
            ROS_ERROR("Aborting because the robot appears to be oscillating over and over. Even after executing all recovery behaviors");
            as_->setAborted(move_base_msgs::MoveBaseResult(), "Robot is oscillating. Even after executing recovery behaviors.");
          }
          resetState();
          return true;
        }
        break;
      default:
        ROS_ERROR("This case should never be reached, something is wrong, aborting");
        resetState();
        //disable the planner thread
        boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
        runPlanner_ = false;
        lock.unlock();
        as_->setAborted(move_base_msgs::MoveBaseResult(), "Reached a case that should not be hit in move_base. This is a bug, please report it.");
        return true;
    }

    //we aren't done yet
    return false;
  }




//从参数服务器加载导航堆栈的恢复行为
  bool MoveBase::loadRecoveryBehaviors(ros::NodeHandle node){
    XmlRpc::XmlRpcValue behavior_list;
    if(node.getParam("recovery_behaviors", behavior_list)){
      if(behavior_list.getType() == XmlRpc::XmlRpcValue::TypeArray){
        for(int i = 0; i < behavior_list.size(); ++i){
          if(behavior_list[i].getType() == XmlRpc::XmlRpcValue::TypeStruct){
            if(behavior_list[i].hasMember("name") && behavior_list[i].hasMember("type")){
              //check for recovery behaviors with the same name
              for(int j = i + 1; j < behavior_list.size(); j++){
                if(behavior_list[j].getType() == XmlRpc::XmlRpcValue::TypeStruct){
                  if(behavior_list[j].hasMember("name") && behavior_list[j].hasMember("type")){
                    std::string name_i = behavior_list[i]["name"];
                    std::string name_j = behavior_list[j]["name"];
                    if(name_i == name_j){
                      ROS_ERROR("A recovery behavior with the name %s already exists, this is not allowed. Using the default recovery behaviors instead.", 
                          name_i.c_str());
                      return false;
                    }
                  }
                }
              }
            }
            else{
              ROS_ERROR("Recovery behaviors must have a name and a type and this does not. Using the default recovery behaviors instead.");
              return false;
            }
          }
          else{
            ROS_ERROR("Recovery behaviors must be specified as maps, but they are XmlRpcType %d. We'll use the default recovery behaviors instead.",
                behavior_list[i].getType());
            return false;
          }
        }

        //if we've made it to this point, we know that the list is legal so we'll create all the recovery behaviors
        for(int i = 0; i < behavior_list.size(); ++i){
          try{
            //check if a non fully qualified name has potentially been passed in
            if(!recovery_loader_.isClassAvailable(behavior_list[i]["type"])){
              std::vector<std::string> classes = recovery_loader_.getDeclaredClasses();
              for(unsigned int i = 0; i < classes.size(); ++i){
                if(behavior_list[i]["type"] == recovery_loader_.getName(classes[i])){
                  //if we've found a match... we'll get the fully qualified name and break out of the loop
                  ROS_WARN("Recovery behavior specifications should now include the package name. You are using a deprecated API. Please switch from %s to %s in your yaml file.",
                      std::string(behavior_list[i]["type"]).c_str(), classes[i].c_str());
                  behavior_list[i]["type"] = classes[i];
                  break;
                }
              }
            }

            boost::shared_ptr<nav_core::RecoveryBehavior> behavior(recovery_loader_.createInstance(behavior_list[i]["type"]));

            //shouldn't be possible, but it won't hurt to check
            if(behavior.get() == NULL){
              ROS_ERROR("The ClassLoader returned a null pointer without throwing an exception. This should not happen");
              return false;
            }

            //initialize the recovery behavior with its name
            behavior->initialize(behavior_list[i]["name"], &tf_, planner_costmap_ros_, controller_costmap_ros_);
            recovery_behaviors_.push_back(behavior);
          }
          catch(pluginlib::PluginlibException& ex){
            ROS_ERROR("Failed to load a plugin. Using default recovery behaviors. Error: %s", ex.what());
            return false;
          }
        }
      }
      else{
        ROS_ERROR("The recovery behavior specification must be a list, but is of XmlRpcType %d. We'll use the default recovery behaviors instead.", 
            behavior_list.getType());
        return false;
      }
    }
    else{
      //if no recovery_behaviors are specified, we'll just load the defaults
      return false;
    }

    //if we've made it here... we've constructed a recovery behavior list successfully
    return true;
  }

  //we'll load our default recovery behaviors here
  //加载navigation stack的默认恢复行为
  void MoveBase::loadDefaultRecoveryBehaviors(){
    recovery_behaviors_.clear();
    try{
      //we need to set some parameters based on what's been passed in to us to maintain backwards compatibility
      ros::NodeHandle n("~");
      n.setParam("conservative_reset/reset_distance", conservative_reset_dist_);
      n.setParam("aggressive_reset/reset_distance", circumscribed_radius_ * 4);

      //first, we'll load a recovery behavior to clear the costmap
      boost::shared_ptr<nav_core::RecoveryBehavior> cons_clear(recovery_loader_.createInstance("clear_costmap_recovery/ClearCostmapRecovery"));
      cons_clear->initialize("conservative_reset", &tf_, planner_costmap_ros_, controller_costmap_ros_);
      recovery_behaviors_.push_back(cons_clear);

      //next, we'll load a recovery behavior to rotate in place
      boost::shared_ptr<nav_core::RecoveryBehavior> rotate(recovery_loader_.createInstance("rotate_recovery/RotateRecovery"));
      if(clearing_rotation_allowed_){
        rotate->initialize("rotate_recovery", &tf_, planner_costmap_ros_, controller_costmap_ros_);
        recovery_behaviors_.push_back(rotate);
      }

      //next, we'll load a recovery behavior that will do an aggressive reset of the costmap
      boost::shared_ptr<nav_core::RecoveryBehavior> ags_clear(recovery_loader_.createInstance("clear_costmap_recovery/ClearCostmapRecovery"));
      ags_clear->initialize("aggressive_reset", &tf_, planner_costmap_ros_, controller_costmap_ros_);
      recovery_behaviors_.push_back(ags_clear);

      //we'll rotate in-place one more time
      if(clearing_rotation_allowed_)
        recovery_behaviors_.push_back(rotate);
    }
    catch(pluginlib::PluginlibException& ex){
      ROS_FATAL("Failed to load a plugin. This should not happen on default recovery behaviors. Error: %s", ex.what());
    }

    return;
  }

//重置移动基础动作的状态，并向机器人发送零速度命令
  void MoveBase::resetState(){
    // Disable the planner thread
    boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
    runPlanner_ = false;
    lock.unlock();

    // Reset statemachine
    state_ = PLANNING;
    recovery_index_ = 0;
    recovery_trigger_ = PLANNING_R;
    publishZeroVelocity();

    //if we shutdown our costmaps when we're deactivated... we'll do that now
    if(shutdown_costmaps_){
      ROS_DEBUG_NAMED("move_base","Stopping costmaps");
      planner_costmap_ros_->stop();
      controller_costmap_ros_->stop();
    }
  }

  bool MoveBase::getRobotPose(geometry_msgs::PoseStamped& global_pose, costmap_2d::Costmap2DROS* costmap)
  {
    tf2::toMsg(tf2::Transform::getIdentity(), global_pose.pose);
    geometry_msgs::PoseStamped robot_pose;
    tf2::toMsg(tf2::Transform::getIdentity(), robot_pose.pose);
    robot_pose.header.frame_id = robot_base_frame_;
    robot_pose.header.stamp = ros::Time(); // latest available
    ros::Time current_time = ros::Time::now();  // save time for checking tf delay later

    // get robot pose on the given costmap frame
    try
    {
      tf_.transform(robot_pose, global_pose, costmap->getGlobalFrameID());
    }
    catch (tf2::LookupException& ex)
    {
      ROS_ERROR_THROTTLE(1.0, "No Transform available Error looking up robot pose: %s\n", ex.what());
      return false;
    }
    catch (tf2::ConnectivityException& ex)
    {
      ROS_ERROR_THROTTLE(1.0, "Connectivity Error looking up robot pose: %s\n", ex.what());
      return false;
    }
    catch (tf2::ExtrapolationException& ex)
    {
      ROS_ERROR_THROTTLE(1.0, "Extrapolation Error looking up robot pose: %s\n", ex.what());
      return false;
    }

    // check if global_pose time stamp is within costmap transform tolerance
    if (current_time.toSec() - global_pose.header.stamp.toSec() > costmap->getTransformTolerance())
    {
      ROS_WARN_THROTTLE(1.0, "Transform timeout for %s. " \
                        "Current time: %.4f, pose stamp: %.4f, tolerance: %.4f", costmap->getName().c_str(),
                        current_time.toSec(), global_pose.header.stamp.toSec(), costmap->getTransformTolerance());
      return false;
    }

    return true;
  }
};
