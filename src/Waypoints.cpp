#include "Waypoints.h"

Waypoints::Waypoints(vector<double> x, vector<double> y,
                  vector<double> s, vector<double>d_x, vector<double>d_y):
                  map_x_(x), map_y_(y), map_s_(s), map_d_x_(d_x), map_d_y_(d_y) 
{
  // empty
}

// Fetch the Next Waypoint, given x, y, angle theta
// Internally uses map_x_, map_y_ from the Maps Waypoints
int Waypoints::GetNextWaypoint(double x, double y, double theta)
{ 
    // use func in common.h
    return NextWaypoint(x, y, theta, this->map_x_, this->map_y_);
    // NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y);
}