
// https://zhuanlan.zhihu.com/p/130502383
// http://planning.cs.uiuc.edu/node661.html#77556
// https://github.com/WillenPeng/Path_Planning_Algorithm

#include <iostream>
#include <fstream>
#include <vector>

#include "map.hpp"
#include "pathfinder_hybrid_astar.hpp"
#include "def_all.hpp"

using namespace std;

int main(int argc, char **argv)
{
    // set start and goal configuration
    double sx = 15.0; // [m]
    double sy = 10.0; // [m]
    double syaw0 = 0.0 * Degree_to_Radian;
    double syaw1 = 0.0 * Degree_to_Radian;

    double gx = 0.0; // [m]
    double gy = 0.0; // [m]
    double gyaw0 = 90.0 * Degree_to_Radian;
    double gyaw1 = 90.0 * Degree_to_Radian;

    cout << "Start Configuration: (" << sx << ", " << sy << ", " << syaw0 * Radian_to_Degree << ", " << syaw1 * Radian_to_Degree << ")" << endl;
    cout << "Goal Configuration: (" << gx << ", " << gy << ", " << gyaw0 * Radian_to_Degree << ", " << gyaw1 * Radian_to_Degree << ")" << endl;

    // 构建停车的搜索地图
    Map map;
    map.build_own_map();
    //find the final path
    Path_Final path = calc_hybrid_astar_path(sx, sy, syaw0, syaw1, gx, gy, gyaw0, gyaw1, map);

    cout << "Program Done!!" << endl;
    cout << "path size  " << path.x.size() << std::endl;

    // save data to a dat
    ofstream savefile;
    savefile.open("../simulation/path.dat");
    for (size_t i = 0; i < path.x.size(); i++)
    {
        savefile << path.x[i] << " " << path.y[i] << " " << path.yaw[i] << " " << path.yaw_trailer[i] << " " << path.direction[i] << endl;
    }
    savefile.close();

    savefile.open("../simulation/startandgoal.dat");
    savefile << sx << " " << sy << " " << syaw0 << " " << syaw1 << " " << gx << " " << gy << " " << gyaw0 << " " << gyaw1 << endl;
    savefile.close();

    // 地图的坐标
    savefile.open("simulation/map.dat");
    for (size_t i = 0; i < map.ox.size(); i++)
    {
        savefile << map.ox[i] << " " << map.oy[i] << endl;
    }
    savefile.close();

    return 0;
}