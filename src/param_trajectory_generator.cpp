#include "param_trajectory_generator.hpp"

namespace sst_star_local_planner{
    PTG::PTG(costmap_2d::Costmap2D* costmap,float min_radius,float max_length):cd_(costmap),min_radius_(min_radius),max_length_(max_length)
    {
        // int i = 0;
        v0=1;
        K=1;
        w0=1;       
        max_alpha = 2 * std::atan(10 * 1 / min_radius);
    }
    Node PTG::c_PTG_gen(float alpha,float length,Node node)
    {
        /****************inputs********************/
        std::pair<float,float> pos;
        tf::Quaternion orien = node.orientation;
        pos.first = node.x;
        pos.second = node.y;
        float yaw = orien.getAngle();
        float V = v0*K;
        float W = w0*tan(alpha/2);
        float total_length = std::fmax(length,max_length_);
        float current_length = 0;
        int steps = int(total_length/step_len_);
        /****************tmps**********************/
        bool collision_flag = false;        
        float x_pose,y_pose,theta;        
        std::list<Simple_node> tmp_waypoints;
        tmp_waypoints.clear();
        /******************outputs**********************/
        std::pair<float,float> target_pose;
        tf::Quaternion target_orien;
        /***********************************************/
        for(int i=0;i<steps;i++)
        {
            // ROS_INFO("PTG itr %d, total %d",i,steps);
            current_length += step_len_;
            if(alpha != 0)
            {
                x_pose = (V/W)*std::sin(current_length*W);
                y_pose = (V/W) * (1 - std::cos(current_length * W));
                theta  = current_length * W + yaw;  
            }
            else
            {
                x_pose = V*(current_length);
                y_pose = 0;
                theta  = 0 + yaw;
            }
            target_pose = coordinate_change(pos, orien, x_pose,y_pose);
            target_orien.setRPY(0,0,theta);
            if(i > 2 && i < steps-2)
            {
                tmp_waypoints.push_front(Simple_node(target_pose.first,target_pose.second,target_orien));
            }
            if(cd_.isThisPointCollides(target_pose.first,target_pose.second))
            {
                ROS_INFO("collision occur in %f,%f!",target_pose.first,target_pose.second);
                collision_flag = true;
                break;
            }

        }
        if(!collision_flag)
        {
            if(alpha != 0)
            {
                x_pose = (V/W)*std::sin(total_length*W);
                y_pose = (V/W) * (1 - std::cos(total_length * W));
                theta  = total_length * W + yaw;  
            }
            else
            {
                x_pose = V*(total_length);
                y_pose = 0;
                theta  = 0 + yaw;
            }
            target_pose = coordinate_change(pos, orien, x_pose,y_pose);
            if(cd_.isThisPointCollides(target_pose.first,target_pose.second))
            {
                // ROS_INFO("collision occur in %f,%f!",target_pose.first,target_pose.second);
                collision_flag = true;
            }

        }
        
        Node ret_node; 
        if(collision_flag)
        {
            ret_node.node_id=-1;
            return ret_node;
        }
        else
        {
            
            target_orien.setRPY(0,0,theta);
            // ROS_INFO("no collision occur, ret %f,%f!",target_pose.first,target_pose.second);
            ret_node = Node(target_pose.first,target_pose.second,target_orien,0,node.node_id,true,0);//node_id not set yet
            ret_node.cost = node.cost + length;
            ret_node.waypoints = tmp_waypoints;
            return ret_node;
        }
        
    }

    std::pair<float,float> PTG::c_PTG_reverse(Node node,Node target)
    {
        std::pair<float,float> pos,target_pose;
        pos.first = node.x;
        pos.second = node.y;
        // ROS_INFO("car pos:%f,%f",pos.first,pos.second);
        // ROS_INFO("target pos:%f,%f",target.x,target.y);
        
        std::pair<float,float> target_in_car = coordinate_change_reverse(pos,node.orientation,target.x,target.y);
        
        float car_x = target_in_car.first   == 0 ? 0.001:target_in_car.first;
        float car_y = target_in_car.second  == 0 ? 0.001:target_in_car.second;
        // ROS_INFO("target pos after trans:%f,%f",car_x,car_y);
        float theta = std::atan2(car_y,car_x);
        // ROS_INFO("got theta: %f",theta);
        // ROS_INFO("PTG Hyper params:K-%f,v0-%f,w0-%f",K,v0,w0);
        // ROS_INFO("in alpha atan: %f",K*v0*sin(2*theta)/(car_x*w0));
        float alpha = 2*std::atan(K*v0*sin(2*theta)/(car_x*w0));
        float t     = 2*theta/(K*v0*sin(2*theta)/(car_x*w0));
        if(alpha > max_alpha)
            alpha = max_alpha;
        if(t > max_length_)
            t = max_length_;
        return std::make_pair(alpha,t);

    }
    float PTG::get_max_alpha()
    {
        return max_alpha;
    }
    
    //将PTG生成的taget从车身坐标系转换到地图坐标系
    //translate target generated by PTG form car coordinate to map coordinate
    std::pair<float,float> PTG::coordinate_change(std::pair<float,float> carpos,tf::Quaternion orien,float x_pose,float y_pose)
    {
        tf::Quaternion q = orien.inverse();
        tf::Quaternion p1;
        p1.setW(0);
        p1.setX(x_pose);
        p1.setY(y_pose);
        p1.setZ(0);
        tf::Quaternion t;
        t.setW(0);
        t.setX(carpos.first);
        t.setY(carpos.second);
        t.setZ(0);
        tf::Quaternion re = q.inverse()*(p1)*q + t;
        return std::make_pair(re.getX(),re.getY());
    }
    //将PTG生成的taget从地图坐标系转换到车身坐标系
    std::pair<float,float> PTG::coordinate_change_reverse(std::pair<float,float> carpos,tf::Quaternion car_orien,float x_pose,float y_pose)
    {
        tf::Quaternion q = car_orien.inverse();
        tf::Quaternion p,t;
        p.setW(0);
        p.setX(x_pose);
        p.setY(y_pose);
        p.setZ(0);
        t.setW(0);
        t.setX(carpos.first);
        t.setY(carpos.second);
        t.setZ(0);
        tf::Quaternion pos = q*(p-t)*q.inverse();
        return std::make_pair(pos.getX(),pos.getY());
    }
}