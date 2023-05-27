/*
  Copyright 2021 - Rafael Barreto
*/

#ifndef SST_STAR_LOCAL_PLANNER_SST_STAR_PLANNER_HPP_  // NOLINT
#define SST_STAR_LOCAL_PLANNER_SST_STAR_PLANNER_HPP_

#include <ros/ros.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_local_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/GetPlan.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>

#include <cmath>
#include <string>
#include <vector>
#include <list>
#include <utility>
#include <memory>


#include "sstnode.hpp"
#include "sst_star.hpp"
#include "random_double_generator.hpp"

namespace sst_star_local_planner {

/**
 * @class SSTStarPlanner
 * @brief Provides a ROS sst* global planner plugin
 */
class SSTStarPlanner : public nav_core::BaseLocalPlanner {
public:
	SSTStarPlanner();
	~SSTStarPlanner();
	void initialize(std::string name, tf2_ros::Buffer* tf,costmap_2d::Costmap2DROS* costmap_ros);

	void computeFinalPlan(std::vector<geometry_msgs::PoseStamped>& plan,  // NOLINT
		const std::list<geometry_msgs::PoseStamped> &path);

	bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

	bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan); 

	bool isGoalReached();

	bool makelocalPlan(const geometry_msgs::PoseStamped & current_pos, const geometry_msgs::PoseStamped & local_goal_pos);

	bool checkgoalPose(const geometry_msgs::PoseStamped &goal);

	bool checkStartPose(const geometry_msgs::PoseStamped &start);

	bool isLocalGoalReached();

	void poseCallback(const nav_msgs::Odometry::ConstPtr &msg);

	void twistCallback(const geometry_msgs::Twist::ConstPtr &msg);
private:
	costmap_2d::Costmap2D* costmap_{nullptr};
	costmap_2d::Costmap2DROS *costmap_ros_;
	bool initialized_{false};
	int max_num_nodes_;
	int min_num_nodes_;
	float map_width_;
	float map_height_;
	double goal_tolerance_;
	bool search_specific_area_{false};
	std::string global_frame_;
	std::shared_ptr<SSTStar> planner_;
	double delta_s_;
	double delta_BN_;
	double T_prop_;
	int N_;
	double min_radius_;
    double max_length_;

	bool is_goal_reached_;
	int local_goal_dist;
	ros::Publisher plan_pub_;
	ros::Publisher plan_pub_2;
	std::vector<geometry_msgs::PoseStamped> global_plan_;
	std::vector<geometry_msgs::PoseStamped> local_plan_;
	geometry_msgs::PoseStamped current_pose_;
	geometry_msgs::PoseStamped current_pose_global_;
	geometry_msgs::PoseStamped local_goal_;
	geometry_msgs::Twist vel_ins; 
	ros::Publisher path_vehicles_pub_;
	ros::Subscriber current_pose_global_sub_;
	ros::Subscriber vel_ins_sub_;
	nav_msgs::Path local_to_pub;
	nav_msgs::Path global_to_pub;
	double cx,cy;
	int succ=0;
  	int fai = 0;

	//for debug
	ros::Publisher collision_point_pub;
};

}  // namespace rrt_star_global_planner

#endif  // RRT_STAR_GLOBAL_PLANNER_RRT_STAR_PLANNER_HPP_  // NOLINT
