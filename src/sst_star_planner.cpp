/*
  Copyright 2021 - Rafael Barreto
*/

#include <pluginlib/class_list_macros.h>
#include <ros/node_handle.h>
#include "sst_star_planner.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

// register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(sst_star_local_planner::SSTStarPlanner, nav_core::BaseLocalPlanner)

namespace sst_star_local_planner {

SSTStarPlanner::SSTStarPlanner() : costmap_(nullptr), initialized_(false) ,is_goal_reached_(false)
{}

SSTStarPlanner::~SSTStarPlanner()
{}

void SSTStarPlanner::initialize(std::string name, tf2_ros::Buffer* tf,costmap_2d::Costmap2DROS* costmap_ros) {
  if (!initialized_) {
    ROS_INFO("initializing the SST planner");
    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros->getCostmap();
    global_frame_ = "map";

    ros::NodeHandle private_nh("~/" + name);
    ros::NodeHandle nh2;

    private_nh.param("goal_tolerance", goal_tolerance_, 0.3);
    private_nh.param("delta_s", delta_s_, 0.1);
    private_nh.param("delta_BN", delta_BN_, 0.2);
    private_nh.param("T_prop", T_prop_, 0.5);
    private_nh.param("N", N_, 2000);
    private_nh.param("min_num_nodes", min_num_nodes_, 3);
    private_nh.param("max_num_nodes", max_num_nodes_, 5000);
    private_nh.param("min_radius", min_radius_, 2.0);
    private_nh.param("max_length", max_length_, 0.5);
    /****************************************/
    plan_pub_ = nh2.advertise<nav_msgs::Path>("lcplan", 20);
    plan_pub_2 = nh2.advertise<nav_msgs::Path>("gbplan", 20);
	collision_point_pub = nh2.advertise<geometry_msgs::PoseStamped>("collision_goal",20);
    local_goal_dist = 300;
    current_pose_.pose.position.x = 0;
    current_pose_.pose.position.y = 0;
    current_pose_.pose.position.z = 0;
    current_pose_.pose.orientation.z=1;
    current_pose_global_sub_ = nh2.subscribe<nav_msgs::Odometry>("odom", 1, &SSTStarPlanner::poseCallback, this);
	vel_ins_sub_ = nh2.subscribe<geometry_msgs::Twist>("/smart/cmd_vel2", 1, &SSTStarPlanner::twistCallback, this);
    cx = costmap_->getOriginX();
    cy = costmap_->getOriginY();
    ROS_INFO("successfully init!");
    local_to_pub.header.frame_id="map";
    global_to_pub.header.frame_id="map";
    /***********************************/
    // TODO(Rafael) remove hard coding
    if (search_specific_area_) {
      map_width_ = 1.0;
      map_height_ = 1.0;
    } else {
      map_width_ = costmap_->getSizeInMetersX();
      map_height_ = costmap_->getSizeInMetersY();
    }

    ROS_INFO("SST* Local Planner initialized successfully.");
    initialized_ = true;
  } else {
    ROS_WARN("This planner has already been initialized... doing nothing.");
  }
}

void  SSTStarPlanner::computeFinalPlan(std::vector<geometry_msgs::PoseStamped>& plan,
                                       const std::list<geometry_msgs::PoseStamped> &path) {
  // clean plan
  plan.clear();
  ros::Time plan_time = ros::Time::now();

  // convert points to poses
  for (const auto &point : path) {
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = plan_time;
    pose.header.frame_id = global_frame_;
    pose.pose.position = point.pose.position;
    pose.pose.orientation = pose.pose.orientation;
    plan.push_back(pose);
  }
}

bool SSTStarPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
{
    if(!initialized_)
{
    ROS_ERROR("SST planner not initialized yet.");
    return false;
}
    global_plan_.clear();
    global_plan_ = orig_global_plan;
    is_goal_reached_ = false;
    ROS_INFO("successfully setPlan!,%ld",global_plan_.size());
    global_to_pub.poses = global_plan_;
    plan_pub_2.publish(global_to_pub);
    return true;

}
bool SSTStarPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
  {
    ROS_INFO("trying to compute velocity!");
	// if(isGoalReached())
	// {
	// 	local_plan_.clear();
	// 	cmd_vel.linear.x=0;
	// 	cmd_vel.linear.y=0;
	// 	cmd_vel.linear.z=0;
	// 	ROS_INFO("we have arrived the global goal!");
	// 	return true;
	// }
	// ROS_INFO("fishish check the global goal!");
	cmd_vel = vel_ins;	
    if(isLocalGoalReached())
	{
		ROS_INFO("we have arrived a local goal!");
    	local_plan_.clear();
	}
	cx = costmap_->getOriginX();
    cy = costmap_->getOriginY();

      // ROS_INFO("now we have pose: %f,%f",current_pose_.pose.position.x,current_pose_.pose.position.y);
      if(global_plan_.size() >= local_goal_dist)
      {
          local_goal_ = global_plan_[local_goal_dist];
          local_goal_.pose.position.x -= cx;
          local_goal_.pose.position.y -= cy;
      }
      else if(global_plan_.size() >= 10)
      {
			ROS_INFO("navigaating to the final local goal!");
          local_goal_ = global_plan_.back();
          local_goal_.pose.position.x -= cx;
          local_goal_.pose.position.y -= cy;
      }
	  else
	  {
		if(isLocalGoalReached())
			ROS_INFO("we have arrived");
		else
			ROS_INFO("stuck near the global goal, needs recovery");
		local_plan_.clear();
		cmd_vel.linear.x=0;
		cmd_vel.linear.y=0;
		cmd_vel.linear.z=0;
		return true;
	  }
	  geometry_msgs::PoseStamped local_goal_to_pub = local_goal_;
	  local_goal_to_pub.pose.position.x+=cx;
	  local_goal_to_pub.pose.position.y+=cy;
	  collision_point_pub.publish(local_goal_to_pub);
	//   geometry_msgs::PoseStamped current_pose_to_pub = current_pose_;
	//   current_pose_to_pub.pose.position.x+=cx;
	//   current_pose_to_pub.pose.position.y+=cy;
	//   collision_point_pub.publish(current_pose_to_pub);
      makelocalPlan(current_pose_,local_goal_);
	//   ROS_INFO("from position:(%f,%f)in local map, set local goal: (%f,%f) in local map, (%f,%f) in global maap.",current_pose_.pose.position.x,current_pose_.pose.position.y, local_goal_.pose.position.x,local_goal_.pose.position.y,local_goal_to_pub.pose.position.x,local_goal_to_pub.pose.position.y);
      local_to_pub.poses = local_plan_;
      for(int i=0;i<local_to_pub.poses.size();i++)
      {
          local_to_pub.poses[i].pose.position.x+=cx;
          local_to_pub.poses[i].pose.position.y+=cy;
      }
      // ROS_INFO("don't have a local plan, making a new one.");
      plan_pub_.publish(local_to_pub);
      return true;

      
  }
bool SSTStarPlanner::makelocalPlan(const geometry_msgs::PoseStamped & current_pose, const geometry_msgs::PoseStamped & local_goal_pose)
{
    // Expander* _planner;

    // ROS_INFO("the resolution of cost map: %f ",costmap->getResolution());
    // ROS_INFO("here we make plan!");

    //检查设定的目标点参数是否合规

    // ROS_INFO("check succ!");
    local_plan_.clear();
    //正式将参数传入规划器中
    // ROS_INFO("trying to calculate");
    // if(!_planner->calculatePath(current_pose, local_goal_pose , costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY(), local_plan_, path_vehicles_pub_, pathNodes)) {
    //     return false;
    // }
    // ROS_INFO("succ calculate");
    // return true;
    std::pair<float, float> start_point = {current_pose.pose.position.x, current_pose.pose.position.y};
    std::pair<float, float> goal_point = {local_goal_pose.pose.position.x, local_goal_pose.pose.position.y};
	tf::Quaternion start_orien_ ;
	start_orien_.setW(current_pose.pose.orientation.w);
	start_orien_.setX(current_pose.pose.orientation.x);
	start_orien_.setY(current_pose.pose.orientation.y);
	start_orien_.setZ(current_pose.pose.orientation.z);
    planner_ = std::shared_ptr<SSTStar>(new SSTStar(start_point,
                                                  goal_point,
												  start_orien_,
                                                  costmap_,
                                                  goal_tolerance_,
                                                  max_num_nodes_,
                                                  min_num_nodes_,
                                                  map_width_,
                                                  map_height_,
                                                  delta_s_,
                                                  delta_BN_,
                                                  T_prop_,
                                                  N_,
                                          min_radius_,
                                          max_length_
                                                  ));
//   ROS_INFO("doing path planning");
    std::list<geometry_msgs::PoseStamped> path;
  
  if (planner_->pathPlanning(path)) {
    ROS_INFO("SST* Global Planner: Path found!!!!");
    succ ++;
    computeFinalPlan(local_plan_, path);
    return true;
  } else {
    fai ++;
    ROS_WARN("The planner failed to find a path, choose other goal position");
    // ROS_INFO("The planner failed to find a path, choose other goal position");
    return false;
  }
//   if(succ+fai >= 20)
//   {
//     ROS_INFO("succ rate: %f",(succ/succ+fai));
//   }

}


bool SSTStarPlanner::isLocalGoalReached()
{
    tf::Stamped<tf::Pose> goal_pose;
    if(local_plan_.size() < 1) return false;
    tf::poseStampedMsgToTF(local_plan_.back(), goal_pose);
    double dx = goal_pose.getOrigin().getX() - current_pose_.pose.position.x;
    double dy = goal_pose.getOrigin().getY() - current_pose_.pose.position.y;
    // ROS_INFO("executr is local goal reached!");
    if(sqrt(dx * dx + dy * dy) < goal_tolerance_)
        return true;
    return false;

}

bool SSTStarPlanner::isGoalReached()
{
	tf::Stamped<tf::Pose> goal_pose;
    tf::poseStampedMsgToTF(global_plan_.back(), goal_pose);
    double dx = goal_pose.getOrigin().getX() - current_pose_.pose.position.x - cx;
    double dy = goal_pose.getOrigin().getY() - current_pose_.pose.position.y - cy;
    if(sqrt(dx * dx + dy * dy) < 0.5)
        return true;
    return false;

}
bool SSTStarPlanner::checkStartPose(const geometry_msgs::PoseStamped &start) {
	unsigned int startx,starty;
	// ROS_INFO("here we call check the start pose!");
	// ROS_INFO("start pose : %f,%f",start.pose.position.x, start.pose.position.y);
	if (costmap_->worldToMap(start.pose.position.x, start.pose.position.y, startx, starty)) {
		// ROS_INFO("here we checked the start pose!");
		return true;
	}
	// ROS_WARN("The Start pose is out of the map!");
	// ROS_INFO("here we checked the start pose!");
	return false;
}//end of checkStartPose

bool SSTStarPlanner::checkgoalPose(const geometry_msgs::PoseStamped &goal) {
	unsigned int goalx,goaly;
	// ROS_INFO("here we call check the goal pose!");
	// ROS_INFO("goal pose : %f,%f",goal.pose.position.x, goal.pose.position.y);
	if (costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, goalx, goaly)) {
		if (costmap_->getCost( goalx, goaly ) > 252) {
		// std::cout << costmap->getCost(goalx, goaly) << std::endl;
		
		ROS_WARN("The Goal pose is occupied , please reset the goal!");
		return false;
		}
		// ROS_INFO("here we checked the start pose!");
		return true;
	}
	ROS_WARN("The Goal pose is out of the map! %d",costmap_->getCost(goalx, goaly));
	// ROS_INFO("here we checked the start pose!");
	return false;
}//end of checkgoalPose  

void SSTStarPlanner::poseCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
	static tf2_ros::Buffer buf;
    static tf2_ros::TransformListener tl(buf);
	cx = costmap_->getOriginX();
    cy = costmap_->getOriginY();
    current_pose_global_.header = msg->header;
    current_pose_global_.pose = msg->pose.pose;
	try
	{
		current_pose_ = buf.transform(current_pose_global_,"map");
	}
    catch(const std::exception& e)
    {
        ROS_WARN("error %s",e.what());
    }
    current_pose_.pose.position.x -= cx;
    current_pose_.pose.position.y -= cy;
    // ROS_WARN("Get Pose");
    // ROS_INFO("ODOM POS:%f,%f",msg->pose.pose.position.x,msg->pose.pose.position.y);
}

void SSTStarPlanner::twistCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
  vel_ins.angular = msg->angular;
  vel_ins.linear = msg->linear;
}
}  // namespace rrt_star_global_planner
