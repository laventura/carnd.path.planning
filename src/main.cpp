#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <cmath>
#include <iomanip>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"

#include "common.h"
#include "Waypoints.h"

using namespace std;

// for convenience
using json = nlohmann::json;


// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Return Lane number (0 - Left, 1 - Center, 2 - Right)
double getCarLane(double car_d) {
  double lane = 1; // default CENTER lane
  if (car_d > 0 && car_d < 4) {
    lane = 0;    // LEFT
  } else if (car_d >=4 && car_d <= 8) {
    lane = 1;   // CENTER
  } else if (car_d > 8 && car_d <= 12) {
    lane = 2;    // RIGHT
  }
  return lane;
}

// print Lane name, given its index
string getLaneInfo(double lane_num) {
  if (lane_num == 0)      { return "LEFT"; }
  else if (lane_num == 1) { return "CENTER"; }
  else if (lane_num == 2) { return "RIGHT"; }
  else { return "NONE"; }
}

inline string yes_no(bool value) {
  if (value) { return "YES"; }
  else        { return "NO"; }
}


// Path Planner --  Return TRUE if safe to change into given lane
bool is_lane_safe(const int num_points,     // num of points to project speed for
                  const double ego_car_s,  // Ego Car's s
                  const double ref_vel,      // Ego Car's reference velocity
                  const double check_lane, // Lane to look for
                  const vector<vector<double> >& sensor_fusion_data)
{
  bool ok_to_change = false;      // should we move into the check_lane?

  double SHORTEST_FRONT = 100000; // Really big
  double SHORTEST_BACK  = -100000;

  cout << "   Front buffer (m): " << LANE_CHANGE_BUFFER_FRONT
       << ",  Back buffer: " << LANE_CHANGE_BUFFER_BACK << endl;

  // Calculate the closest Front and Back gaps
  for (int i = 0; i < sensor_fusion_data.size(); i++)
  {
    float d = sensor_fusion_data[i][6];    // d for a Traffic Car
    double other_car_lane = getCarLane(d); // lane of the Traffic Car
    // if a Traffic Car is in the lane to check
    if (other_car_lane == check_lane) {
      // get it's speed
      double vx = sensor_fusion_data[i][3];
      double vy = sensor_fusion_data[i][4];
      double check_speed = sqrt(vx*vx + vy*vy);
      // get it's s displacement
      double check_car_s = sensor_fusion_data[i][5];

      // see how far Other Car will go in TIME_INTERVAL seconds
      // i.e. project its future s
      check_car_s += ((double)num_points * TIME_INTERVAL * check_speed);

      // double future_ego_s = ego_car_s + ((double)num_points * TIME_INTERVAL * ref_vel);

      // see the gap from our Ego Car
      double dist_s = check_car_s - ego_car_s;  // WAS: ego_car_s
      // remove -ve sign
      double dist_pos = sqrt(dist_s * dist_s);  

      // store the shortest gap
      // SHORTEST_S = min(dist_pos, SHORTEST_S); 

      if (dist_s > 0) {                  // FRONT gap
        SHORTEST_FRONT = min(dist_s, SHORTEST_FRONT);
      } else if (dist_s <= 0) {          // BACK gap
        SHORTEST_BACK  = max(dist_s, SHORTEST_BACK);
      }

      cout << "   gap (m): " 
          << setprecision(5)
          << dist_s 
          << ", closest front: "
          << setprecision(5)
          << SHORTEST_FRONT
          << ", closest back: "
          << setprecision(5)
          << SHORTEST_BACK
          << endl;
    }
  }
        cout << "   gap (m): " 
          << " >>> Closest Front: "
          << setprecision(5)
          << SHORTEST_FRONT
          << ", closest Back: "
          << setprecision(5)
          << SHORTEST_BACK
          << " <<< "
          << endl;

  
  // Only if enough space in that lane, move to that lane
  if ( (SHORTEST_FRONT > LANE_CHANGE_BUFFER_FRONT)  &&
      (-1 * SHORTEST_BACK > LANE_CHANGE_BUFFER_BACK)) 
  {
    ok_to_change = true;
  }

  // if enough space to move into that lane
  // if (fabs(SHORTEST_S) < LANE_CHANGE_BUFFER_BACK) {
  //   ok_to_change = false;
  // }
  cout << " CHECK Lane  : " << getLaneInfo(check_lane) << ", OK_to_Change? " << yes_no(ok_to_change) << endl;
  return ok_to_change;

} // end func


// ------- MAIN --------
int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  
  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  // Store the Map Waypoints
  Waypoints waypoints(map_waypoints_x, map_waypoints_y, map_waypoints_s,
                      map_waypoints_dx, map_waypoints_dy );

  int   LANE     = 1;              // Start in CENTER lane
  int   lane_change_waypoint = 0;

  h.onMessage([&waypoints,
              &LANE, &lane_change_waypoint]
              (uWS::WebSocket<uWS::SERVER> ws, 
                                  char *data, size_t length,
                                   uWS::OpCode opCode) 
  {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
            double car_x = j[1]["x"];
            double car_y = j[1]["y"];
            double car_s = j[1]["s"];
            double car_d = j[1]["d"];
            double car_yaw = j[1]["yaw"];
            double car_speed = j[1]["speed"];

            // Previous path data given to the Planner
            auto previous_path_x = j[1]["previous_path_x"];
            auto previous_path_y = j[1]["previous_path_y"];
            // Previous path's end s and d values 
            double end_path_s = j[1]["end_path_s"];
            double end_path_d = j[1]["end_path_d"];

            // Sensor Fusion Data, a list of all other cars on the same side of the road.
            auto sensor_fusion = j[1]["sensor_fusion"];

            double MAX_SPEED = 50.0;            // mph
            double REF_V      = MAX_SPEED - 1;   // mph

            json msgJson;

            // Values for Next Path to send to simulator
            vector<double> next_x_vals;
            vector<double> next_y_vals;        

            // REFERENCE X, Y, Yaw states - intially set to previous path's values
            double ref_x = car_x;
            double ref_y = car_y;
            double ref_yaw = deg2rad(car_yaw);
                        
            int path_size = previous_path_x.size();
            int next_waypoint = -1;

            if (path_size < 2) { // Little or No Previous Path
                // fetch next waypoint, using given reference and map x/y
                next_waypoint = waypoints.GetNextWaypoint(ref_x, ref_y, ref_yaw);

            } else {
                // Else use previous path's end point as starting reference

                // Redefine reference
                ref_x  = previous_path_x[path_size-1];
                ref_y = previous_path_y[path_size-1];

                double ref_x_prev = previous_path_x[path_size-2];
                double ref_y_prev = previous_path_y[path_size-2];

                ref_yaw = atan2(ref_y - ref_y_prev,  ref_x - ref_x_prev);

                next_waypoint = waypoints.GetNextWaypoint(ref_x, ref_y, ref_yaw);

                // Get previous s and speed
                car_s = end_path_s;
                double dist = sqrt((ref_x - ref_x_prev)*(ref_x - ref_x_prev) + 
                                   (ref_y - ref_y_prev)*(ref_y - ref_y_prev));
                car_speed = (dist / TIME_INTERVAL) * MS_TO_MPH;
            }

            // ---- SENSOR FUSION PROCESSING ----
            // Determine Reference Velocity based on nearby traffic
            double   CLOSEST_DISTANCE_S = 100000;
            bool     change_lane = false;
            bool     too_close   = false;

            for (int i = 0; i < sensor_fusion.size(); i++)
            {
              double other_d     = sensor_fusion[i][6];    // other car's d
              double other_lane = getCarLane(other_d);    // other car's lane#
              double   other_vx  = sensor_fusion[i][3];
              double   other_vy   = sensor_fusion[i][4];
              double  other_car_s = sensor_fusion[i][5];

              // if Other Car in Ego car's lane..
              if (other_lane == LANE)
              {
                double   other_speed = sqrt(other_vx * other_vx + other_vy*other_vy);
                // project where Other Car will be in the next few steps 
                other_car_s += (path_size * TIME_INTERVAL * other_speed);
                // find front gap
                double front_gap = other_car_s - car_s; 
                // check if Other Car is ahead and by how much
                if (front_gap > 0  && front_gap < FRONT_BUFFER && front_gap < CLOSEST_DISTANCE_S)
                {
                  CLOSEST_DISTANCE_S = front_gap;

                  if (front_gap > FRONT_TOO_CLOSE) {
                    // follow the front car
                    REF_V = other_speed * MS_TO_MPH;
              
                    // Yes - try a lane change
                    change_lane = true;
                    cout << "Front gap: "  << front_gap 
                         << "\tReference velocity (mph): " 
                         << setprecision(4)
                         << REF_V
                         << ", current speed: "
                         << car_speed
                         << endl;
                  } else {  // FRONT TOO CLOSE!
                    // go slower than front car
                    REF_V = other_speed * MS_TO_MPH - 5.0; 
                    too_close = true;
                    // Definitely do a lane change!
                    change_lane = true;
                    cout << "FRONT TOO CLOSE! "  << front_gap 
                         << "\tReference velocity (mph): " 
                         << setprecision(4)   
                         << REF_V 
                         << ", current speed: "
                         << car_speed
                         << endl;
                  }
                cout << "   Maybe Change Lane? " << yes_no(change_lane) << endl;
                }
              } // if in my lane
            } // sensor-fusion

            // --- LANE CHANGE LOGIC: Determine Target Lane (if needed) ---
            int delta_wp = next_waypoint - lane_change_waypoint;
            int remain_wp = delta_wp % waypoints.map_x_.size();
            // cout << " delta wp   : " << delta_wp << endl;
            // cout << " map wp size: " << map_waypoints_x.size() << endl;
            // cout << " remain wp: " << remain_wp << endl;

            if (change_lane && remain_wp > 2)
            {
              cout << "..Checking Lane Change from: "
                   << getLaneInfo(LANE)
                   << ", at s: " << car_s << endl;
              bool did_change_lane = false;
              // First - check LEFT lane
              if (LANE != LANE_LEFT && !did_change_lane) {
                bool lane_safe = true;

                // Check if OK to go LEFT?
                lane_safe = is_lane_safe(path_size, 
                                        car_s, 
                                        REF_V,
                                        LANE - 1, // To the Left of Current
                                        sensor_fusion );
                
                if (lane_safe) { // OK to go LEFT
                  did_change_lane = true;
                  LANE -= 1;  // go Left by one lane
                  lane_change_waypoint = next_waypoint;
                }
              }
              // NEXT - Try Right Lane?
              if (LANE != LANE_RIGHT && !did_change_lane) {
                bool lane_safe = true;

                // Check if OK to go RIGHT
                lane_safe = is_lane_safe(path_size, 
                                         car_s,
                                         REF_V, 
                                         LANE + 1,  // to the Right of Current
                                         sensor_fusion);
                
                if (lane_safe) { // OK to go RIGHT
                  did_change_lane = true;
                  LANE += 1;  // go Right by one
                  lane_change_waypoint = next_waypoint;
                }
              }
              cout << " Current Lane: " 
                   << getLaneInfo(LANE) 
                   << ",  changed_lane: " 
                   << yes_no(did_change_lane) 
                   << ", s: " << car_s
                   << endl;
            } // if change lane
            // --- END LANE CHANGE ---

            // --- Now we know Target Lane and Reference Velocity. Now create a Smooth Path ---
            // Create list of widely spaced XY Anchor Points, evenly spaced at 30meters (SPACING)
            // We will interpolate these waypoints with a spline and fill with more points that control speed
            vector<double> anchor_pts_x;
            vector<double> anchor_pts_y;

            // If previous path size almost empty, use car as starting ref
            if (path_size < 2) {

              // Use two points that make the path tangent to the car
              double prev_car_x = car_x - cos(car_yaw);
              double prev_car_y = car_y - sin(car_yaw);

              // Add to Anchor Points
              anchor_pts_x.push_back(prev_car_x);
              anchor_pts_x.push_back(car_x);

              anchor_pts_y.push_back(prev_car_y);
              anchor_pts_y.push_back(car_y);
            } 
            else {  // Use couple of points from previous path

              // Add to Anchor Points
              anchor_pts_x.push_back(previous_path_x[path_size-2]);
              anchor_pts_x.push_back(previous_path_x[path_size-1]);

              anchor_pts_y.push_back(previous_path_y[path_size-2]);
              anchor_pts_y.push_back(previous_path_y[path_size-1]);
            }

            // In Frenet coordinates, add 30-meters evenly spaced points ahead of starting reference
          
            double TARGET_D = 2 + LANE * 4;  // d coord for target lane
            // DEBUG ONLY -
            // cout << " Target Lane #: " << getLaneInfo(LANE) 
            //      << ", Target D: " << TARGET_D 
            //      << ", s: " << car_s
            //      << endl;
            vector<double> next_wp0 = getXY((car_s + SPACING),   TARGET_D, waypoints.map_s_, waypoints.map_x_, waypoints.map_y_);
            vector<double> next_wp1 = getXY((car_s + SPACING*2), TARGET_D, waypoints.map_s_, waypoints.map_x_, waypoints.map_y_);
            vector<double> next_wp2 = getXY((car_s + SPACING*3), TARGET_D, waypoints.map_s_, waypoints.map_x_, waypoints.map_y_);

            // Add these next waypoints to Anchor Points
            anchor_pts_x.push_back(next_wp0[0]);
            anchor_pts_x.push_back(next_wp1[0]);
            anchor_pts_x.push_back(next_wp2[0]);

            anchor_pts_y.push_back(next_wp0[1]);
            anchor_pts_y.push_back(next_wp1[1]);
            anchor_pts_y.push_back(next_wp2[1]);

            // Transform to Local coordinates
            for (int i = 0; i < anchor_pts_x.size(); i++)
            {
              // SHIFT car reference angle to 0 degree
              double shift_x = anchor_pts_x[i] - ref_x;
              double shift_y = anchor_pts_y[i] - ref_y;

              // ROTATION
              anchor_pts_x[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
              anchor_pts_y[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
            }

            // Create a Spline
            tk::spline s_spline;

            // set Anchor points on the Spline
            s_spline.set_points(anchor_pts_x, anchor_pts_y);

            // ADD points from Previous Path - for continuity
            for(int i = 0; i < path_size; i++)
            {
                next_x_vals.push_back(previous_path_x[i]);
                next_y_vals.push_back(previous_path_y[i]);
            }

            // Target X and Y - Calculate how to break up spline points to travel at REF_VELOCITY
            double   target_x         = SPACING;  // HORIZON: going out to SPACING meters ahead
            double   target_y         = s_spline(target_x);
            double   target_distance = sqrt((target_x * target_x) + (target_y * target_y));

            double   x_add_on = 0;

            // double dist_inc = 0.4;    // Distance to increment, in meters
            // Fill up the rest of the path 
            
            // DEBUG -- 

            for(int i = 1; i < 50-path_size; i++)
            {
                // if too slow, speed up by a small amount
                if (car_speed < REF_V) {
                  car_speed += (MS_TO_MPH / 10);     // 0.224;
                } // else speed down by a small amount
                else if (car_speed > REF_V) {
                  car_speed -= (MS_TO_MPH / 10);    // 0.224;
                }
                // Calculate spacing of number of points based on desired Car Speed    
                double N         = (target_distance / (TIME_INTERVAL * car_speed/MS_TO_MPH)); // num of points
                double x_point   = x_add_on + (target_x) / N;
                double y_point   = s_spline(x_point);            // y on the spline

                x_add_on       = x_point;

                double x_ref   = x_point;
                double y_ref  = y_point;

                // Transform coordinates back to normal
                x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
                y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));
                // Add to our Reference x, y
                x_point += ref_x;
                y_point += ref_y;

                // FINALLY -- add to our Next Path vectors
                next_x_vals.push_back(x_point);
                next_y_vals.push_back(y_point);
          
            }
            // ------- END 
            //  define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
            msgJson["next_x"] = next_x_vals;
            msgJson["next_y"] = next_y_vals;

            auto msg = "42[\"control\","+ msgJson.dump()+"]";

            //this_thread::sleep_for(chrono::milliseconds(1000));
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
} // -- End Main -- 

