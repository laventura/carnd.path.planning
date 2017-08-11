#ifndef WAYPOINTS_H
#define WAYPOINTS_H

#include <vector>

#include "common.h"
#include "spline.h"

using std::vector;

class Waypoints {
  /** Stores Map Waypoints
  */

  public: 
    Waypoints(vector<double> x, vector<double> y,
              vector<double> s, vector<double>d_x, vector<double>d_y);

    ~Waypoints() {};

    vector<double> map_x_;
    vector<double> map_y_;
    vector<double> map_s_;
    vector<double> map_d_x_;
    vector<double> map_d_y_;

    // Fetch the Next Waypoint, given x, y, angle theta
    // Internally uses map_x, map_y
    int GetNextWaypoint(double x, double y, double theta);

};


#endif // WAYPOINTS_H