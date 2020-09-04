/* Copyright 2018 Weilun Peng */
//map class with all information about obstacles
#ifndef MAP_H
#define MAP_H

#include <iostream>
#include <vector>
#include <math.h>
#include "kdtree.h"

#include "node.hpp"
#include "grid_a_star.hpp"
#include "def_all.hpp"

using namespace std;

class Map;

class Map {
 public:
    vector< double > ox;
    vector< double > oy;
    // a map with the h cost (distance to goal node) of each grid (without obstacle)
    vector <vector< double > > h_dp;
    kdt::KDTree<vector<double>> kdtree;

    int minx;
    int miny;
    int minyaw;
    int minyawt;
    int maxx;
    int maxy;
    int maxyaw;;
    int maxyawt;
    int xw;
    int yw;
    int yaww;
    int yawtw;

 public:
    Map() {};
    void build_own_map() {
        cout << "Start Build Own Map!" << endl;
        for (double i = -25; i <= 25; i++) {
            ox.emplace_back(i);
            oy.emplace_back(47.0);
        }
        for (double i = -25; i <= -3; i++) {
            ox.emplace_back(i);
            oy.emplace_back(3.0);
        }
        for (double i = -15; i <= 3; i++) {
            ox.emplace_back(-3.0);
            oy.emplace_back(i);
        }
        for (double i = -15; i <= 3; i++) {
            ox.emplace_back(3.0);
            oy.emplace_back(i);
        }
        for (double i = 3; i <= 25; i++) {
            ox.emplace_back(i);
            oy.emplace_back(3.0);
        }
        for (double i = -3; i <= 3; i++) {
            ox.emplace_back(i);
            oy.emplace_back(-15.0);
        }
    }

    void calc_config() {
        //make kdtree
        vector<vector<double> > samples;
        size_t N = ox.size();
        cout<<"N  "<<N<<endl;
        samples.resize(N);
        for (size_t i = 0; i < N; i++) {
            samples[i].emplace_back(ox[i]);
            samples[i].emplace_back(oy[i]);
        }

        kdtree.MakeKDTree(samples);

        // x方向的最大值,最小值
        auto ox_min_max = minmax_element(ox.begin(), ox.end());
        // y方向的最大值,最小值
        auto oy_min_max = minmax_element(oy.begin(), oy.end());
        double min_x_m = *(ox_min_max.first) - EXTEND_AREA;
		double min_y_m = *(oy_min_max.first) - EXTEND_AREA;
		double max_x_m = *(ox_min_max.second) + EXTEND_AREA;
		double max_y_m = *(oy_min_max.second) + EXTEND_AREA;

        ox.emplace_back(min_x_m);
        oy.emplace_back(min_y_m);
        ox.emplace_back(max_x_m);
        oy.emplace_back(max_y_m);

        // 转换为 grid_index
        minx = int(round(min_x_m/XY_GRID_RESOLUTION));
        miny = int(round(min_y_m/XY_GRID_RESOLUTION));
        maxx = int(round(max_x_m/XY_GRID_RESOLUTION));
        maxy = int(round(max_y_m/XY_GRID_RESOLUTION));

        // 
        xw = int(round(maxx - minx));
        yw = int(round(maxy - miny));

        minyaw = int(round(-PI/YAW_GRID_RESOLUTION)) - 1;
        maxyaw = int(round(PI/YAW_GRID_RESOLUTION));
        // yaww = int(round(maxyaw - minyaw))
        yaww = maxyaw - minyaw;

        minyawt = minyaw;
        maxyawt = maxyaw;
        yawtw = yaww;
       
    }

    int calc_index(Node* node){
        // calculate the index of node which count x, y, yaw and yaw_trailer to code index (4D)
        int ind = (node->yawind - minyaw)*xw*yw+(node->yind - miny)*xw + (node->xind - minx);

        // 4D grid
        int yaw1ind = int(round(node->yaw_trailer.back()/YAW_GRID_RESOLUTION));
        ind += (yaw1ind - minyawt) *xw*yw*yaww;

        if (ind <= 0) cout << "Error(calc_index): " << ind <<endl;
        return ind;
    } 

    void calc_holonomic_with_obstacle_heuristic(Node* gnode) {
        // To get a map with the h cost (distance to goal node) of each grid (without obstacle)
		grid_a_star gridastar;
        h_dp = gridastar.calc_dist_policy(gnode->x.back(), gnode->y.back(), &ox, &oy, XY_GRID_RESOLUTION, VR);
        // for (size_t i = 0; i < h_dp.size(); i++) {
        //     for (size_t j = 0; j < h_dp[0].size(); j++) {
        //         cout << h_dp[i][j] << " ";
        //         if (h_dp[i][j] == 0) {
        //             cout << "i: " << i << " j: " << j << endl;
        //         }
        //     }
        //     cout << endl;
        // }
    }

    double calc_cost(Node* n){
        // get the f cost of node n
        return (n->cost + H_COST*h_dp[n->xind - minx-1][n->yind - miny-1]);
    }
    
};

#endif