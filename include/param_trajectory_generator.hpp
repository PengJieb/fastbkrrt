#include "collision_detector.hpp"

namespace sst_star_local_planner
{
    class PTG
    {
        public:
            PTG(costmap_2d::Costmap2D* costmap,float min_radius,float max_length);
            Node c_PTG_gen(float alpha,float length,Node node);
            std::pair<float,float> c_PTG_reverse(Node node,Node target);
            std::pair<float,float> coordinate_change(std::pair<float,float> carpos,tf::Quaternion orien,float x_pose,float y_pose);
            std::pair<float,float> coordinate_change_reverse(std::pair<float,float> carpos,tf::Quaternion orien,float x_pose,float y_pose);
            float get_max_alpha();
        private:
            CollisionDetector cd_;
            float min_radius_{1};
            float max_length_{3};
            float step_len_{0.05};
            float max_alpha;
            float v0,K,w0;
            

    };
}