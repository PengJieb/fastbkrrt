/*
  Copyright 2021 - Rafael Barreto
*/

#ifndef SST_STAR_LOCAL_PLANNER_NODE_HPP_  // NOLINT
#define SST_STAR_LOCAL_PLANNER_NODE_HPP_

#include <cmath>
#include<tf/tf.h>
namespace sst_star_local_planner {

inline float euclideanDistance2D(float x1, float y1, float x2, float y2) {
  return std::hypot((x1 - x2), (y1 - y2));
}
struct Simple_node{
  float x;
  float y;
  tf::Quaternion orientation;
  Simple_node(){}
  Simple_node(float x_,float y_,tf::Quaternion orientation_)
  {
    x = x_;
    y = y_;
    orientation = orientation_;
  }
};
struct Node {
  float x;
  float y;
  int node_id;
  int parent_id;
  float cost{0.0};
  //Node *rep;
  /*******************added for SST*******************/
  bool is_active = false;
  
  int  childnum = 0;
  int   failtime = 0;
  tf::Quaternion orientation;
  std::list<Simple_node> waypoints;
  // int rep_id =-1 ;
  /***************************************************/
  Node() {}

  Node(float px, float py,tf::Quaternion orien, int node_index, int parent_index,bool active,int child) : x(px),
		y(py),
		orientation(orien),
		node_id(node_index),
		parent_id(parent_index),
		is_active(active),
		childnum(child) 
	{
		waypoints.clear();
	}

  bool operator ==(const Node& node) { return node_id == node.node_id; }

  bool operator !=(const Node& node) { return !(node_id == node.node_id); }
  bool is_leaf()
  {
    if (childnum > 0)
      return false;
    else
      return true;
  }
};

struct witness
{
    int rep_id;
    int s_id;
    float x;
    float y;
    witness(){}
    witness(float x_,float y_ ,int rep_id_,int s_id_):x(x_),y(y_),rep_id(rep_id_),s_id(s_id_)
    {
		
    }
    // ~witness();
    bool operator ==(const witness& w) { return s_id == w.s_id; }

    bool operator !=(const witness& w) { return !(s_id == w.s_id); }
};
// witness::witness(int node_id_,int rep_id_,int s_id_)


// witness::~witness()
// {
// }
}  // namespace rrt_star_global_planner

#endif  // RRT_STAR_GLOBAL_PLANNER_NODE_HPP_  NOLINT