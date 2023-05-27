#include "sst_star.hpp"

namespace sst_star_local_planner
{
    SSTStar::SSTStar(const std::pair<float, float> &start_point,
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
                    double max_length
                    ): start_point_(start_point),
                                        goal_point_(goal_point),
                                        start_orien_(start_orien),
                                        costmap_(costmap),
                                        goal_tolerance_(goal_tolerance),
                                        max_num_nodes_(max_num_nodes),
                                        min_num_nodes_(min_num_nodes),
                                        map_width_(map_width),
                                        map_height_(map_height),
                                        cd_(costmap),
                                        delta_s_(delta_s),
                                        delta_BN_(delta_BN),
                                        T_prop_(T_prop),
                                        N_(N),
                                        ptg_(costmap,min_radius,max_length)
    {
        V.reserve(max_num_nodes_);
        S.reserve(max_num_nodes_);
        V.clear();
        S.clear();
        random_double_.setRange(-map_width_, map_width_);
        ROS_INFO("FBK class init!");
    }

    std::pair<float, float> SSTStar::sampleFree()
    {
        random_double_.setRange(0, map_width_);
        std::pair<float, float> random_point;
        random_point.first = random_double_.generate();
        random_double_.setRange(0, map_height_);
        random_point.second = random_double_.generate();
        return random_point;
    }

    int SSTStar::Nearest(Node x)
    {
        // ROS_INFO("enter nearest!");
        // ROS_INFO("node id: %d",x.node_id);
        float min_dist = __FLT_MAX__;
        int min_id = -1;
        float dist;
        // ROS_INFO("S Size: %d",S.size());
        witness s;
        for(int is=0;is<S.size();is++)
        {
            s = S[is];
            // Node sx = V[s.node_id];
            // ROS_INFO("s_id: %d,node_id: %d",s.s_id,s.node_id);
            dist = euclideanDistance2D(x.x,x.y,s.x,s.y);
            // ROS_INFO("ed: %f",dist);
            if (dist < min_dist)
            {
                // ROS_INFO("enter dist<min");
                min_id = s.s_id;
                min_dist = dist;
            }
            // ROS_INFO("out dist<min");
        }
        // ROS_INFO("min id : %d",min_id);
        return min_id;
    }

    int SSTStar::Best_First_Selection(float delta)
    {
        std::pair<float, float> x_rand;
        std::pair<Node, float> tmp;
        std::vector<std::pair<Node, float>> X_near;
        int dis;
        int close_id;
        float close_dist = __FLT_MAX__;
        int min_id;
        float min_cost = __FLT_MAX__;
        x_rand = sampleFree();
        xr = x_rand;
        // ROS_INFO("rand: %f,%f",x_rand.first,x_rand.second);
        // ROS_INFO("node_num: %d",V.size());
        for(auto node : V) // 需要修改 只管active的点 done
        {
            if(!node.is_active)
                continue;
            dis = euclideanDistance2D(x_rand.first,x_rand.second,node.x,node.y);
            if(dis < close_dist)
            {
                close_dist = dis;
                close_id = node.node_id;
            }
            if(dis<=delta)
            {
                tmp.first = node;
                tmp.second = dis + node.cost;
                X_near.push_back(tmp);
                if (tmp.second < min_cost)
                {
                    min_cost = tmp.second;
                    min_id = node.node_id;
                }
            }
        }
        if(X_near.empty())
        {
            return close_id;
        }
        return min_id;
    }

    Node SSTStar::MonteCarlo_Prop(Node xn,std::pair<float,float> xrand)
    {
        // ROS_INFO("inter MP");
        random_double_.setRange(0, T_prop_);
        // ROS_INFO("set range");
        // Node xp = V[xn.parent_id];
        Node xt;
        float x1,x2,y1,y2;
        y2 = xrand.second;
        y1 = xn.y;
        x2 = xrand.first;
        x1 = xn.x;
        float t = random_double_.generate();
        // ROS_INFO("t: %f",t);
        float theta = atan2(y2 - y1, x2 - x1);
        xt.x = x1 + t*cos(theta);
        xt.y = y1 + t*sin(theta);
        xt.cost = xn.cost + t;
        xt.parent_id = xn.node_id;
        xt.node_id = V.size();
        xt.childnum = 0;
        xt.is_active = true;
        //暂时不加入
        // V.push_back(xt);
        // ROS_INFO("finish");
        // ROS_INFO("x_sel: (%f,%f)",xn.x,xn.y);
        // ROS_INFO("x_new: (%f,%f)",xt.x,xt.y);
        // 重写添加算法 done
        return xt;
    }

    bool SSTStar::Is_Node_Locally_the_Best(Node xn)
    {
        // ROS_INFO("enter nearest");
        int snew_id = Nearest(xn);
        // ROS_INFO("snew_id: %d",snew_id);
        // ROS_INFO("out nearest");
        witness s_near = S[snew_id];
        if(euclideanDistance2D(s_near.x,s_near.y,xn.x,xn.y) > delta_s_)
        {
            // ROS_INFO("too far");
            witness sn = witness(xn.x,xn.y,-1,S.size());
            // ROS_INFO("snid: %d",sn.s_id);
            S.push_back(sn);
            // ROS_INFO("snid: check :%d",S[sn.s_id].s_id);
            snew_id = sn.s_id;

        }
        int xpeer_id = S[snew_id].rep_id;
        if(xpeer_id == -1 || xn.cost < V[xpeer_id].cost)
        {
            return true;
        }
        return false;
    }

    void SSTStar::Prune_Dominated_Nodes_SST(Node xn)
    {
        // ROS_INFO("start pruning");
        int pn = 0;
        int snew_id = Nearest(xn);
        int xpeer_id = S[snew_id].rep_id;
        // ROS_INFO("snew_id: %d, xpeer_id: %d",snew_id,xpeer_id);
        if(xpeer_id != -1)
        {
            V[xpeer_id].is_active = false;
        }
        S[snew_id].rep_id = xn.node_id;
        while(xpeer_id!=-1 && V[xpeer_id].is_leaf() && !V[xpeer_id].is_active)
        {
            int xparent_id = V[xpeer_id].parent_id;
            del_node(xpeer_id);
            pn ++;
            xpeer_id = xparent_id;
        }
        if(pn>0)
        {
            // ROS_INFO("pruned %d nodes",pn);
        }
        return;
    }

    void SSTStar::del_node(int id)
    {
        // ROS_INFO("delete occur, we del node with id:%d",id);
        // ROS_INFO("size of V: %d",V.size());
        V[V[id].parent_id].childnum--;
        std::vector<int> fx;
        std::vector<Node>::iterator it;
        int ii;
        bool wrong_flag = false;
        for( ii=0;ii<id;ii++)
        {
            fx.push_back(ii);
        }
        fx.push_back(-1);
        for(ii+=1;ii<V.size();ii++)
        {
            fx.push_back(ii-1);
        }
        it = V.begin()+id;
        // ROS_INFO("start erasing");
        V.erase(it);
        // ROS_INFO("finish erasing");
        for(auto &v:V)
        {
            v.node_id = fx[v.node_id];
            if(v.parent_id != -1)
                v.parent_id = fx[v.parent_id];
        }
        for(auto &s:S)
        {
            s.rep_id = fx[s.rep_id];
        }
        // for(int j=0;j<V.size();j++)
        // {
        //     if(j!=V[j].node_id)
        //     {
        //         ROS_INFO("id wrong after del, wrong j=%d node_id=%d",j,V[j].node_id);
        //         wrong_flag = true;
        //     }
        // }
        // if(wrong_flag)
        // {
        //     ROS_INFO("something wrong, start check.");
        //     for(auto im:fx)
        //     {
        //         ROS_INFO("%d",im);
        //     }
        // }
    }

    bool SSTStar::isGoalReached_algo(const std::pair<float, float> &p_new)
    {
        float dis = euclideanDistance2D(p_new.first,
                                    p_new.second,
                                    goal_point_.first,
                                    goal_point_.second);
        // ROS_INFO("this distance: %f",dis);
        return (dis < goal_tolerance_) ? true : false;
    }

    void SSTStar::computeFinalPath(std::list<geometry_msgs::PoseStamped> &path)
    {
        // ROS_INFO("start conputing final");
        path.clear();

        // Compute the path from the goal to the start
        Node current_node = goal_node_;

        // Final Path
        geometry_msgs::PoseStamped point;

        do {
            
            point.pose.position.x = current_node.x;
            point.pose.position.y = current_node.y;
            point.pose.position.z = 0.0;
            point.pose.orientation.w = current_node.orientation.getW();
            point.pose.orientation.x = current_node.orientation.getX();
            point.pose.orientation.y = current_node.orientation.getY();
            point.pose.orientation.z = current_node.orientation.getZ();
            // ROS_INFO("Tracing to %f,%f, with point id:%d parent id: %d",point.first,point.second,current_node.node_id,current_node.parent_id);
            path.push_front(point);
            for(auto snd:current_node.waypoints)
            {
                point.pose.position.x = snd.x;
                point.pose.position.y = snd.y;
                point.pose.position.z = 0.0;
                point.pose.orientation.w = snd.orientation.getW();
                point.pose.orientation.x = snd.orientation.getX();
                point.pose.orientation.y = snd.orientation.getY();
                point.pose.orientation.z = snd.orientation.getZ();
                path.push_front(point);
            }

            // update the current node
            current_node = V[current_node.parent_id];
        } while (current_node.parent_id != -1);
        ROS_INFO("end of loop id: %d",current_node.node_id);
    }

    bool SSTStar::pathPlanning(std::list<geometry_msgs::PoseStamped> &path)
    {
        bool goal_reached_ = false;
        // ROS_INFO("enter planning ,got start point:(%f, %f), target point:(%f,%f)",start_point_.first, start_point_.second,goal_point_.first,goal_point_.second);
            if (cd_.isThisPointCollides(goal_point_.first, goal_point_.second)) 
            {
                ROS_ERROR("Goal point chosen (%f, %f)is NOT in the FREE SPACE! Choose other goal!",goal_point_.first,goal_point_.second);
                return false;
            }
        // Start Node
        Node x0 = Node(start_point_.first, start_point_.second,start_orien_,0,-1,true,0);
        x0.cost = 0;
        // ROS_INFO("node_num: %d",V.size());
        V.push_back(x0);
        // ROS_INFO("node_num: %d",V.size());
        witness s = witness(x0.x,x0.y,0,0);
        S.push_back(s);
        // ROS_INFO("FBK panning start!");
        // ROS_INFO("start: %f,%f",start_point_.first,start_point_.second);
        // ROS_INFO("goal: %f,%f",goal_point_.first,goal_point_.second);
        std::pair<float,float> xnew;
        for(int its=0;its<N_;its++)
        {
            // ROS_INFO("enter loop");
            Node q_rand = RandomConfig();
            // ROS_INFO("finish random config");
            std::vector<int> q_near_id = Search_Near_Neighbours(q_rand,10);
            // ROS_INFO("finish search near neighbours");
            Node q_new  = Gen_Best_Node(q_near_id,q_rand,3);
            // ROS_INFO("finish gen best node");
            // ROS_INFO("got node id: %d",q_new.node_id);
            if (q_new.node_id==-1)
            {
                    // ROS_INFO("node_id is -1");
                    q_new = Obs_Expand(q_near_id,q_rand);
                    // ROS_INFO("finish obs expand");
            }
            if (q_new.node_id!=-1)
            {
                    if(Is_Node_Locally_the_Best(q_new))
                    {
                        // ROS_INFO("is best");
                        // ROS_INFO("We push a point with id: %d parent id: %d",q_new.node_id,q_new.parent_id);
                        V.push_back(q_new);
                        V[q_new.parent_id].childnum ++;
                        Prune_Dominated_Nodes_SST(q_new);
                    }
                    else
                    {
                        // ROS_INFO("not best");
                        continue;
                    }
                        
            }
            else
                continue;
            if (!goal_reached_) 
            {
                // ROS_INFO("goal not reached yet!");
                xnew.first = q_new.x;
                xnew.second = q_new.y;
                if (isGoalReached_algo(xnew)) 
                {
                    ROS_INFO("goal reached now!");
                    ROS_INFO("Tree size: %d points",V.size());
                    goal_reached_ = true;
                    goal_node_ = V.back();
                }
            }

            if (goal_reached_ && V.size() > min_num_nodes_) 
            {
            computeFinalPath(path);
            // ROS_INFO("finish conputing final");
            return true;
            }
        }
        // ROS_INFO("seems failed, trace back: ");
        // ROS_INFO("node num: %d",V.size());
        // ROS_INFO("nodes:");
        // for(auto v:V)
        // {
        //     ROS_INFO("position(%f,%f)",v.x,v.y);
        // }
        return false;
    }
    bool sort_by_distance(const std::pair<int,float> &p1,const std::pair<int,float> &p2)
    {
        return (p1.second < p2.second);
    }
    std::vector<int> SSTStar::Search_Near_Neighbours(Node q_rand,int n)
    {
        std::vector<std::pair<int,float>> tmp;
        std::vector<int> ret;
        float distance;
        for(auto v:V)
        {
            if(!v.is_active)
                continue;
            distance = euclideanDistance2D(q_rand.x,q_rand.y,v.x,v.y);
            tmp.push_back(std::make_pair(v.node_id,distance));
        }
        std::sort(tmp.begin(),tmp.end(),sort_by_distance);
        for(int i=0;i<n;i++)
        {
            if(i >= tmp.size())
                break;
            ret.push_back(tmp[i].first);
        }
        return ret;
    }
    Node SSTStar::Gen_Best_Node(std::vector<int>x_nears,Node x_rand,int m)
    {
        float best_cost = FLT_MAX;
        Node x_best;
        x_best.node_id = -1;
        std::sort(x_nears.begin(),x_nears.end(),[&](const int x_node_id,const int y_node_id){return V[x_node_id].failtime < V[y_node_id].failtime;});
        float back_punish = 0.0;
        float costt =0.0;
        // ROS_INFO("finish sort, got %d points",x_nears.size());
        for(int i=0;i<m;i++)
        {
            if(i>=x_nears.size())
                break;
            Node x_new = Adaptive_Expand(x_nears[i],x_rand);
            // ROS_INFO("finish adaptive expand, itr %d,total %d",i,x_nears.size());
            // ROS_INFO("node id returned by ae is %d",x_new.node_id);
            // ROS_INFO("neighbour: %f,%f",V[x_nears[i]].x,V[x_nears[i]].y);
            // ROS_INFO("this rand: %f,%f",x_rand.x,x_rand.y);
            // std::pair<float,float> pos_in_car_ = ptg_.coordinate_change_reverse(std::make_pair(V[x_nears[i]].x,V[x_nears[i]].y),V[x_nears[i]].orientation,x_rand.x,x_rand.y);
            // if(pos_in_car_.first < 0)
            // {
            //     back_punish = 1.0;
            // }
            // else
            // {
            //     back_punish = 0.0;
            // }
            costt = V[x_nears[i]].cost + euclideanDistance2D(V[x_nears[i]].x,V[x_nears[i]].y,x_rand.x,x_rand.y) + back_punish;
            // ROS_INFO("COST: %f",costt);
            if(x_new.node_id!=-1&&costt < best_cost)
            {
                x_best = x_new;
                best_cost = costt;
            }
        }
        // ROS_INFO("finally choose best node: %f,%f",x_best.x,x_best.y);
        return x_best;

    }
    Node SSTStar::Adaptive_Expand(int x_ibest,Node x_rand)
    {
        // ROS_INFO("enter adaptive expand");
        int count = V[x_ibest].failtime;
        float alpha,t;
        float r1 = 1/(1+count/delta1),r2 = 1/(1+count/delta2);
        r2 = r2<0.1?0.1:r2;
        // ROS_INFO("Got r1 = %f,r2 = %f",r1,r2);
        float stepsize;
        random_double_.setRange(r1*MaxStepSize,r2*MaxStepSize);
        stepsize = random_double_.generate();
        // ROS_INFO("finish generate rd, got step size: %f",stepsize);
        std::pair<float,float> PTG_param = ptg_.c_PTG_reverse(V[x_ibest],x_rand);
        // ROS_INFO("finish ptg reverse,got param alpha = %f,t = %f",PTG_param.first,PTG_param.second);
        alpha = PTG_param.first;
        t = PTG_param.second;
        float d = std::fmin(t,stepsize);
        
        Node x_new = ptg_.c_PTG_gen(alpha,d,V[x_ibest]);
        // ROS_INFO("finish ptg");
        if(x_new.node_id==-1)
        {
            ROS_INFO("Adaptive expand fail to get a new node!");
            V[x_ibest].failtime ++;
        }
        else
        {
            // ROS_INFO("got x_new: %f,%f,node id: %d",x_new.x,x_new.y,V.size());
            x_new.node_id = V.size();
        }
        return x_new;


        
    }
    Node SSTStar::Obs_Expand(std::vector<int>x_nears,Node x_rand)
    {
        // ROS_INFO("execl obs expand");
        float best_length = FLT_MAX;
        int x_best_id = Nearest_to_state(x_nears,x_rand);
        // ROS_INFO("find nearest node id : %d",x_best_id);
        Node x_new;
        x_new.node_id = -1;
        for(int i=0;i<K;i++)
        {
            std::pair<float,float> PTG_param;
            float alpha,t;
            random_double_.setRange(0,euclideanDistance2D(V[x_best_id].x,V[x_best_id].y,goal_point_.first,goal_point_.second));
            t = random_double_.generate();
            random_double_.setRange(-ptg_.get_max_alpha(),ptg_.get_max_alpha());
            alpha = random_double_.generate();
            Node x_tmp = ptg_.c_PTG_gen(alpha,t,V[x_best_id]);
            if(x_tmp.node_id!=-1)
            {
                if(euclideanDistance2D(x_tmp.x,x_tmp.y,x_rand.x,x_rand.y)<best_length)
                {
                    best_length = euclideanDistance2D(x_tmp.x,x_tmp.y,x_rand.x,x_rand.y);
                    x_new = x_tmp;
                    x_new.node_id = V.size();
                }
            }
        }
        return x_new;
    }
    bool SSTStar::cmp_by_failtime(Node x_node,Node y_node)
    {
        return x_node.failtime < y_node.failtime;
    }

    int SSTStar::Nearest_to_state(std::vector<int> x_nears,Node x_rand)
    {
        float min_distance = FLT_MAX;
        int best_id = -1;
        for (auto id:x_nears)
        {
            if(euclideanDistance2D(V[id].x,V[id].y,x_rand.x,x_rand.y)<min_distance)
            {
                min_distance = euclideanDistance2D(V[id].x,V[id].y,x_rand.x,x_rand.y);
                best_id = id;
            }
        }
        if(best_id==-1)
        {
            ROS_ERROR("invalid id in Nearest_to_state");
        }
        return best_id;
    }
    Node SSTStar::RandomConfig()
    {
        random_double_.setRange(0,1);       
        double db = random_double_.generate();
        std::pair<float,float> point;
        if(db<=0.05)
        {
            point= goal_point_;
        }
        else
            point= sampleFree();
        Node n;
        n.x=point.first;
        n.y=point.second;
        return n;
    }
}