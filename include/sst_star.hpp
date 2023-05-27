#ifndef SST_STAR_LOCAL_PLANNER_SST_STAR_HPP_  // NOLINT
#define SST_STAR_LOCAL_PLANNER_SST_STAR_HPP_

#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/PoseStamped.h>
#include <cmath>
#include <string>
#include <vector>
#include <list>
#include <utility>

// #include "sstnode.hpp"
#include "collision_detector.hpp"
#include "random_double_generator.hpp"
#include "param_trajectory_generator.hpp"

namespace sst_star_local_planner {
    class SSTStar
    {
        private:
        /******************auxiliary class vari******************************************/
            std::vector<Node> V;
            std::vector<witness> S;
            RandomDoubleGenerator random_double_;
            CollisionDetector cd_;
            PTG   ptg_;
        /******************normal parameters*****************************/
            std::pair<float, float> start_point_;
            std::pair<float, float> goal_point_;
            costmap_2d::Costmap2D* costmap_{nullptr};
            double goal_tolerance_;
            unsigned int max_num_nodes_;
            unsigned int min_num_nodes_;
            float map_width_;
            float map_height_;
            Node goal_node_;
            tf::Quaternion start_orien_;
        /*****************parameters for sst algorithm********************/
            double delta_s_;
            double delta_BN_;
            double T_prop_;
            int N_;
            double delta1=2,delta2=4;
            double MaxStepSize=0.5;
            int K=100;
            std::pair<float,float> xr;

        public:
            SSTStar(const std::pair<float, float> &start_point,
                const std::pair<float, float> &goal_point,
                tf::Quaternion start_orien,
                costmap_2d::Costmap2D* costmap,
                double goal_tolerance,
                unsigned int max_num_nodes,
                unsigned int min_num_nodes,
                float map_width,
                float map_height,
                double delta_s,
                double delta_BN,
                double T_prop,
                int N,
                double min_radius,
                double max_length);
            bool pathPlanning(std::list<geometry_msgs::PoseStamped> &path);
            std::pair<float, float> sampleFree();
            void computeFinalPath(std::list<geometry_msgs::PoseStamped> &path);
            bool isGoalReached_algo(const std::pair<float, float> &p_new);
            int Nearest(Node x);
            int Best_First_Selection(float delta);
            Node MonteCarlo_Prop(Node xn,std::pair<float,float> x_rand);
            bool Is_Node_Locally_the_Best(Node xn);
            void Prune_Dominated_Nodes_SST(Node xn);
            void del_node(int id);
            std::vector<int> Search_Near_Neighbours(Node x_rand,int n);
            Node Gen_Best_Node(std::vector<int>x_nears,Node x_rand,int m);
            Node Adaptive_Expand(int x_ibest,Node x_rand);
            Node Obs_Expand(std::vector<int>x_nears,Node x_rand);
            static bool cmp_by_failtime(Node x_node,Node y_node);
            int Nearest_to_state(std::vector<int> x_nears,Node x_rand);
            Node RandomConfig();
    };       

}
#endif 