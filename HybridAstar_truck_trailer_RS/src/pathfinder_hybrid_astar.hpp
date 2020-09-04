/* Copyright 2018 Weilun Peng */
//map class with all information about obstacles
#ifndef HYBRIDASTAR_H
#define HYBRIDASTAR_H

#include <vector>
#include <unordered_map>
#include <queue>
#include <iostream>
#include <math.h>
#include <time.h>

#include "node.hpp"
#include "map.hpp"
#include "def_all.hpp"
#include "trailerlib.hpp"
#include "rs_path.hpp"
#include "grid_a_star.hpp"

using namespace std;

class Path_Final;
Path_Final calc_hybrid_astar_path(double sx, double sy, double syaw, double syaw1, double gx, double gy, double gyaw, double goal_point_trailer_yaw, Map map);
bool is_same_grid(Node *node1, Node *node2);
Path_Final get_final_path(unordered_map<int, Node> *closed, Node *goal_point, Node *start_point, Map *map);
bool verify_index(Node *node, Map *map, double init_yaw_trailer);
Node calc_next_node(Node *current, int c_id, double steering_angle_interval, double directions_interval, Map *map);
vector<Node> update_node_with_RS_expansion(Node *current, Node *goal_point, Map *map, double goal_point_trailer_yaw);
vector<Path *> RS_expansion(Node *n, Node *goal_point, Map *map);
pair<vector<double>, vector<double>> calc_motion_inputs();

struct CompareByFirst_path
{
    constexpr bool operator()(pair<double, Path *> const &a,
                              pair<double, Path *> const &b) const noexcept
    {
        return a.first > b.first;
    }
};

// Path class
class Path_Final
{
public:
    Path_Final(){};
    Path_Final(vector<double> x, vector<double> y, vector<double> yaw, vector<double> yaw_trailer, vector<double> steer, vector<bool> direction, double cost)
        : x(x), y(y), yaw(yaw), yaw_trailer(yaw_trailer), steer(steer), direction(direction), cost(cost)
    {
    }

public:
    vector<double> x;           // x position [m]
    vector<double> y;           // y position [m]
    vector<double> yaw;         // yaw angle of front [rad]
    vector<double> yaw_trailer; // trailer angle [rad]
    vector<double> steer;       // steering angle of the wheel [rad]
    vector<bool> direction;     // direction forward: true, back false
    double cost;                // cost
};

Path_Final calc_hybrid_astar_path(double sx, double sy, double syaw, double syaw1, double gx, double gy, double gyaw, double goal_point_trailer_yaw, Map map)
{
    /*
    sx: start x position [m]
    sy: start y position [m]
    gx: goal x position [m]
    gx: goal x position [m]
    ox: x position list of Obstacles [m]
    oy: y position list of Obstacles [m]
    xyreso: grid resolution [m]
    yawreso: yaw angle resolution [rad]
    */
    cout << "Start Find Path!" << endl;
    syaw = PI_to_PI(syaw);
    gyaw = PI_to_PI(gyaw);

    map.calc_config();

    // 起点和终点转化为节点 Node
    Node start_point(int(round(sx / XY_GRID_RESOLUTION)), int(round(sy / XY_GRID_RESOLUTION)),
                     int(round(syaw / YAW_GRID_RESOLUTION)), true, {sx}, {sy}, {syaw}, {syaw1}, {true}, 0.0, 0.0, -1);
    Node goal_point(int(round(gx / XY_GRID_RESOLUTION)), int(round(gy / XY_GRID_RESOLUTION)),
                    int(round(gyaw / YAW_GRID_RESOLUTION)), true, {gx}, {gy}, {gyaw}, {goal_point_trailer_yaw}, {true}, 0.0, 0.0, -1);

    // To get a map with the h cost (distance to goal node) of each grid (without obstacle)
    map.calc_holonomic_with_obstacle_heuristic(&goal_point);

    // initilize the open set, closed set, priority queue
    unordered_map<int, Node> open_set;
    unordered_map<int, Node> closed_set;
    // fnode = None

    open_set[map.calc_index(&start_point)] = start_point;

    priority_queue<pair<double, int>, std::vector<pair<double, int>>, CompareByFirst_node> pq;

    pq.push(make_pair(map.calc_cost(&start_point), map.calc_index(&start_point)));

    // get motion primitives
    // 计算输入
    pair<vector<double>, vector<double>> u_and_d = calc_motion_inputs();
    vector<double> steering_angle_interval = u_and_d.first; // 转向角度
    vector<double> directions_interval = u_and_d.second;    // 方向:前进或者后退
    size_t steering_angle_interval_nums = steering_angle_interval.size();

    int c_id;
    Node current, fnode;
    vector<Node> finalnode;
    Path_Final path;

    // main iteration
    time_t startime = time(NULL);
    while (1)
    {
        if (open_set.empty())
        {
            cout << "Error: Cannot find path, No open set" << endl;
            exit(1);
        }

        // pq中查找,取pq最后加入的node
        c_id = pq.top().second;
        pq.pop();
        current = open_set[c_id];

        // move current node from open to closed
        open_set.erase(c_id);
        closed_set[c_id] = current;

        // use Reed-Shepp model to find a path between current node and goal node without obstacles,
        // which will not run every time for computational reasons
        // goal_point_trailer_yaw: 终点的trailer的yaw
        finalnode = update_node_with_RS_expansion(&current, &goal_point, &map, goal_point_trailer_yaw);

        // 如果能从当前的Node到目标Node直接给出一条RS曲线,那么就不用继续搜索,直接给出这条可以行驶的RS曲线
        if (!finalnode.empty())
        { // found
            fnode = finalnode[0];
            cout<<"RS found path"<<endl;
            break;
        }

        // cout<<"current.yaw_trailer.size()  "<< current.yaw_trailer.size() <<endl;
        // 4
        double init_yaw_trailer = current.yaw_trailer[0];

        Node node;
        int node_ind;
        for (size_t i = 0; i < steering_angle_interval_nums; i++)
        {
            // For each node, it will have multiple possible points but with one x,y index
            // 通过输入的转向角度和行驶方向,计算下一个Node
            node = calc_next_node(&current, c_id, steering_angle_interval[i], directions_interval[i], &map);

            // 判断node是否有碰撞,超出边界等
            if (!verify_index(&node, &map, init_yaw_trailer))
            {
                continue;
            }

            // 计算node的索引
            node_ind = map.calc_index(&node);

            // If it is already in the closed set, skip it
            if (closed_set.find(node_ind) != closed_set.end())
            {
                continue;
            }

            if (open_set.find(node_ind) == open_set.end())
            {
                open_set[node_ind] = node;
                // get the f cost of node n  // f=g+h  // node中只有g
                // f(n) 是从初始状态经由状态n到目标状态的代价估计
                // g(n) 是在状态空间中从初始状态到状态n的实际代价
                // h(n) 是从状态n到目标状态的最佳路径的估计代价
                pq.push(make_pair(map.calc_cost(&node), node_ind));
            }
            else
            { // 选择cost小的node,加入open_set
                if (open_set[node_ind].cost > node.cost)
                    // If so, update the node to have a new parent
                    open_set[node_ind] = node;
            }
        }
    }

    time_t finaltime = time(NULL);
    cout << "Calculated Time: " << finaltime - startime << "[s]" << endl;

    // 最终的轨迹点由close_set中的node和RS曲线组成
    path = get_final_path(&closed_set, &fnode, &start_point, &map);

    return path;
}

vector<Node> update_node_with_RS_expansion(Node *current, Node *goal_point, Map *map, double goal_point_trailer_yaw)
{
    // use Reed-Shepp model to find a path between current node and goal node without obstacles,
    // which will not run every time for computational reasons.
    vector<Path *> finalpath = RS_expansion(current, goal_point, map);
    vector<Node> finalnode;

    // finalpath不为空
    if (!finalpath.empty())
    {
        Path *apath = finalpath[0];
        // RS曲线的路径点和yaw
        vector<double> fx((apath->x).begin() + 1, (apath->x).end());
        vector<double> fy((apath->y).begin() + 1, (apath->y).end());
        vector<double> fyaw((apath->yaw).begin() + 1, (apath->yaw).end());

        // 从当前node到goal_point不能生成RS曲线
        if (abs(PI_to_PI(apath->yaw_trailer.back() - goal_point_trailer_yaw)) >= GOAL_TYAW_TH)
        {
            // finalnode 为空
            return finalnode; 
        }

        // f = g + h
        double fcost = current->cost + apath->pathcost;

        vector<double> fyaw1(apath->yaw_trailer.begin() + 1, apath->yaw_trailer.end());
        int fpind = map->calc_index(current);

        vector<bool> fd;
        // reserver函数用来给vector预分配存储区大小，即capacity的值 ，但是没有给这段内存进行初始化
        fd.reserve(apath->directions.size());

        double directions_interval;
        for (size_t i = 1; i < apath->directions.size(); i++)
        {
            directions_interval = apath->directions[i];
            if (directions_interval >= 0)
                fd.emplace_back(1);
            else
                fd.emplace_back(0);
        }

        double fsteer = 0.0;

        Node fpath(current->xind, current->yind, current->yawind, current->direction,
                   fx, fy, fyaw, fyaw1, fd, fsteer, fcost, fpind);
        delete apath;
        finalnode.emplace_back(fpath);
        return finalnode;
    }

    return finalnode; // no update
}

vector<Path *> RS_expansion(Node *n, Node *goal_point, Map *map)
{
    vector<Path *> finalpath;
    double sx = n->x.back();
    double sy = n->y.back();
    double syaw = n->yaw.back();

    double max_curvature = tan(MAX_STEER) / WB;
    vector<Path *> paths = calc_paths(sx, sy, syaw,
                                      goal_point->x.back(), goal_point->y.back(),
                                      goal_point->yaw.back(), max_curvature, MOTION_RESOLUTION);
    // 
    if (paths.empty())
    {
        return finalpath;
    }

    priority_queue<pair<double, Path *>,
                   std::vector<pair<double, Path *>>,
                   CompareByFirst_path>
        pathqueue;

    vector<double> steps;

    for (size_t i = 0; i < paths.size(); i++)
    {
        steps.resize(paths[i]->directions.size());
        for (size_t j = 0; j < paths[i]->directions.size(); j++)
        {
            steps[j] = MOTION_RESOLUTION * paths[i]->directions[j];
        }

        paths[i]->yaw_trailer =
            calc_trailer_yaw_from_xyyaw(&(paths[i]->yaw), n->yaw_trailer.back(), &steps);

        pathqueue.push(make_pair(paths[i]->calc_rs_path_cost(), paths[i]));
    }

    Path *path;
    vector<double> newpathx, newpathy, newpathyaw, newpathyaw1;
    while (!pathqueue.empty())
    {
        path = pathqueue.top().second;
        pathqueue.pop();
        newpathx.clear();
        newpathy.clear();
        newpathyaw.clear();
        newpathyaw1.clear();
        for (size_t i = 0; i < path->x.size(); i += SKIP_COLLISION_CHECK)
        {
            newpathx.emplace_back(path->x[i]);
            newpathy.emplace_back(path->y[i]);
            newpathyaw.emplace_back(path->yaw[i]);
            newpathyaw1.emplace_back(path->yaw_trailer[i]);
        }
        if (check_trailer_collision(map, &newpathx, &newpathy, &newpathyaw, &newpathyaw1))
        {
            finalpath.emplace_back(path);
            return finalpath; // path is ok
        }
    }
    return finalpath;
}

bool is_same_grid(Node *node1, Node *node2)
{
    if (node1->xind != node2->xind || node1->yind != node2->yind || node1->yawind != node2->yawind)
    {
        return false;
    }
    return true;
}

Path_Final get_final_path(unordered_map<int, Node> *closed, Node *RS_node, Node *start_point, Map *map)
{
    // goal_point is the result from Analytic Expansions
    // and the final path is the combination of points in RS_node and point in closed set
    vector<double> rx, ry, ryaw, ryaw1;
    vector<bool> direction;
    for (int i = RS_node->x.size() - 1; i >= 0; i--)
    {
        rx.emplace_back(RS_node->x[i]);
        ry.emplace_back(RS_node->y[i]);
        ryaw.emplace_back(RS_node->yaw[i]);
        ryaw1.emplace_back(RS_node->yaw_trailer[i]);
        direction.emplace_back(RS_node->directions[i]);
    }
    int nid = RS_node->pind;
    double finalcost = RS_node->cost;

    Node n;
    while (1)
    {
        n = (*closed)[nid];
        for (int i = n.x.size() - 1; i >= 0; i--)
        {
            rx.emplace_back(n.x[i]);
            ry.emplace_back(n.y[i]);
            ryaw.emplace_back(n.yaw[i]);
            ryaw1.emplace_back(n.yaw_trailer[i]);
            direction.emplace_back(n.directions[i]);
        }
        nid = n.pind;
        if (is_same_grid(&n, start_point))
        {
            break;
        }
    }

    vector<double> rx_new, ry_new, ryaw_new, ryaw1_new;
    vector<bool> direction_new;
    for (int i = rx.size() - 1; i >= 0; i--)
    {
        rx_new.emplace_back(rx[i]);
        ry_new.emplace_back(ry[i]);
        ryaw_new.emplace_back(ryaw[i]);
        ryaw1_new.emplace_back(ryaw1[i]);
        direction_new.emplace_back(direction[i]);
    }

    // adjuct first direction
    direction_new[0] = direction_new[1];
    double tem_len;
    vector<double> steer;
    steer.reserve(rx_new.size());
    for (size_t i = 0; i < rx_new.size(); i++)
    {
        if (i < rx_new.size() - 1)
        {
            tem_len = (ryaw_new[i + 1] - ryaw_new[i]) / MOTION_RESOLUTION;
            if (!direction[i])
                tem_len *= -1;
            steer.emplace_back(atan2(WB * tem_len, 1.0));
        }
        else
            steer.emplace_back(0.0);
    }
    // cout << "x: " << rx_new.size() << endl;
    // for (auto each_x: rx_new) {
    //     cout << each_x << " ";
    // }
    // cout << endl;
    // cout << "y: " << ry_new.size()  << endl;
    // for (auto each_y: ry_new) {
    //     cout << each_y << " ";
    // }
    // cout << endl;
    // cout << "yaw: " << ryaw_new.size()  << endl;
    // for (auto each_yaw: ryaw_new) {
    //     cout << each_yaw << " ";
    // }
    // cout << endl;
    // cout << "direction: " << direction_new.size()  << endl;
    // for (auto each_direction: direction_new) {
    //     cout << each_direction << " ";
    // }
    cout << endl;
    Path_Final path(rx_new, ry_new, ryaw_new, ryaw1_new, steer, direction_new, finalcost);
    return path;
}

bool verify_index(Node *node, Map *map, double init_yaw_trailer)
{
    // overflow map
    if ((node->xind - map->minx) >= map->xw)
        return false;
    else if ((node->xind - map->minx) <= 0)
        return false;

    if ((node->yind - map->miny) >= map->yw)
        return false;
    else if ((node->yind - map->miny) <= 0)
        return false;

    // check collisiton
    vector<double> steps;
    for (auto each_direction : node->directions)
    {
        steps.emplace_back(MOTION_RESOLUTION * each_direction);
    }
    vector<double> yaw_trailer = calc_trailer_yaw_from_xyyaw(&(node->yaw), init_yaw_trailer, &steps);
    vector<double> newnodex, newnodey, newnodeyaw, newnodeyaw1;
    for (size_t i = 0; i < node->x.size(); i += SKIP_COLLISION_CHECK)
    {
        newnodex.emplace_back(node->x[i]);
        newnodey.emplace_back(node->y[i]);
        newnodeyaw.emplace_back(node->yaw[i]);
        newnodeyaw1.emplace_back(yaw_trailer[i]);
    }
    if (!check_trailer_collision(map, &newnodex, &newnodey, &newnodeyaw, &newnodeyaw1))
        return false;

    return true; //index is ok"
}

// 通过输入的u,计算下一个Node
Node calc_next_node(Node *current, int c_id, double steering_angle_interval, double directions_interval, Map *map)
{ // 规划的距离的长度
    double arc_l = XY_GRID_RESOLUTION * 1.5;
    // 4
    int path_point_num = int(floor(arc_l / MOTION_RESOLUTION)) + 1;
    // cout<<"path_point_num   "<< path_point_num<<endl;
    vector<double> xlist(path_point_num, 0.0);
    vector<double> ylist(path_point_num, 0.0);
    vector<double> yawlist(path_point_num, 0.0);
    vector<double> yaw_trailer_list(path_point_num, 0.0);

    xlist[0] = current->x.back() + directions_interval * MOTION_RESOLUTION * cos(current->yaw.back());
    ylist[0] = current->y.back() + directions_interval * MOTION_RESOLUTION * sin(current->yaw.back());
    yawlist[0] = PI_to_PI(current->yaw.back() + directions_interval * MOTION_RESOLUTION / WB * tan(steering_angle_interval));
    // 拖车的yaw
    // https://zhuanlan.zhihu.com/p/130502383
    yaw_trailer_list[0] = PI_to_PI(current->yaw_trailer.back() + directions_interval * MOTION_RESOLUTION / LT * sin(current->yaw.back() - current->yaw_trailer.back()));

    // 循环计算4个轨迹点的位置和yaw
    for (size_t i = 0; i < path_point_num - 1; i++)
    {
        xlist[i + 1] = xlist[i] + directions_interval * MOTION_RESOLUTION * cos(yawlist[i]);
        ylist[i + 1] = ylist[i] + directions_interval * MOTION_RESOLUTION * sin(yawlist[i]);
        yawlist[i + 1] = PI_to_PI(yawlist[i] + directions_interval * MOTION_RESOLUTION / WB * tan(steering_angle_interval));
        yaw_trailer_list[i + 1] = PI_to_PI(yaw_trailer_list[i] + directions_interval * MOTION_RESOLUTION / LT * sin(yawlist[i] - yaw_trailer_list[i]));
    }

    int xind = int(round(xlist.back() / XY_GRID_RESOLUTION));
    int yind = int(round(ylist.back() / XY_GRID_RESOLUTION));
    int yawind = int(round(yawlist.back() / YAW_GRID_RESOLUTION));

    double addedcost = 0.0;
    bool direction;
    // 前进方向
    if (directions_interval > 0)
    {
        direction = true;
        addedcost += abs(arc_l);
    }
    else
    { // 后退  //惩罚后退更多
        direction = 0;
        addedcost += abs(arc_l) * BACK_COST;
    }

    // switch back penalty
    // 和当前node的行驶方向不同
    if (direction != current->direction) // switch back penalty
        addedcost += SB_COST;

    // steer penalty
    addedcost += STEER_COST * abs(steering_angle_interval);

    // steer change penalty
    // 和当前node的steering_angle不同
    addedcost += STEER_CHANGE_COST * abs(current->steer - steering_angle_interval);

    // jacknif cost
    double jack_cost = 0.0;
    for (size_t i = 0; i < yawlist.size(); i++)
    {
        jack_cost += abs(PI_to_PI(yawlist[i] - yaw_trailer_list[i]));
    }
    addedcost += JACKKNIF_COST * jack_cost;

    double cost = current->cost + addedcost;

    vector<bool> directions(xlist.size(), direction);

    Node node(xind, yind, yawind, direction, xlist, ylist, yawlist, yaw_trailer_list, directions, steering_angle_interval, cost, c_id);

    return node;
}

//
pair<vector<double>, vector<double>> calc_motion_inputs()
{
    // get the motion around current node with length 1 and different heading degree
    vector<double> up;
    vector<double> steering_angle_interval;
    // 转向的最大角度:MAX_STEER   // 转向的间隔数目: N_STEER
    // 左为正
    for (double i = MAX_STEER / N_STEER; i <= MAX_STEER; i += MAX_STEER / N_STEER)
    {
        up.emplace_back(i);
    }

    steering_angle_interval.emplace_back(0.0);
    steering_angle_interval.insert(steering_angle_interval.end(), up.begin(), up.end());

    // 右为负
    for (vector<double>::iterator it = up.begin(); it != up.end(); ++it)
    {
        steering_angle_interval.emplace_back(-(*it));
    }

    vector<double> directions_interval(steering_angle_interval.size(), 1.0);
    vector<double> uminus1(steering_angle_interval.size(), -1.0);

    // 在d.end()后边,插入元素
    directions_interval.insert(directions_interval.end(), uminus1.begin(), uminus1.end()); // direction: 1 或-1
    steering_angle_interval.insert(steering_angle_interval.end(), steering_angle_interval.begin(), steering_angle_interval.end());

    // cout << "steering_angle_interval" << endl;
    // for (int i = 0; i < steering_angle_interval.size(); i++)
    // {
    //     cout << steering_angle_interval[i] << "  ";
    // }
    // cout << "" << endl;

    // cout << "directions_interval" << endl;
    // for (int i = 0; i < directions_interval.size(); i++)
    // {
    //     cout << directions_interval[i] << "  ";
    // }
    // cout << "" << endl;

    return make_pair(steering_angle_interval, directions_interval);
}

#endif