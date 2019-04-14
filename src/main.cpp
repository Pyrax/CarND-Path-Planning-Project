#include <uWS/uWS.h>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "config.h"
#include "BehaviorPlanner.h"
#include "Coords.h"
#include "MotionPlanner.h"
#include "TrajectoryGenerator.h"
#include "Vehicle.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;
using std::min;

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

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
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

  // Initialize ego car
  Vehicle ego_car{};

  Map map{map_waypoints_x,
          map_waypoints_y,
          map_waypoints_s,
          map_waypoints_dx,
          map_waypoints_dy};
  MotionPlanner motion{map};

  FrenetPath prev_path_frenet{};
  bool initialize = true;

  h.onMessage([&ego_car, &motion, &prev_path_frenet, &initialize,
                  &map_waypoints_x, &map_waypoints_y, &map_waypoints_s,
                  &map_waypoints_dx, &map_waypoints_dy]
                  (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                   uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
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

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          Coords prev_path_xy{previous_path_x, previous_path_y};

          // Update ego vehicle and vehicles in other lanes
          ego_car.set_position(car_s, car_d);
          ego_car.set_velocity(car_speed);

          auto number_lane_cars = sensor_fusion.size();
          vector<Vehicle> lane_cars{number_lane_cars};
          for (int i = 0; i < number_lane_cars; ++i) {
            // double other_car_id = sensor_fusion[i][0];
            // double other_car_x = sensor_fusion[i][1];
            // double other_car_y = sensor_fusion[i][2];
            double other_car_vx = sensor_fusion[i][3];
            double other_car_vy = sensor_fusion[i][4];
            double other_car_s = sensor_fusion[i][5];
            double other_car_d = sensor_fusion[i][6];

            double vel = sqrt(other_car_vx * other_car_vx + other_car_vy * other_car_vy);

            lane_cars[i] = Vehicle{other_car_s, other_car_d, vel};
          }

          int prev_size = previous_path_x.size();
          int prev_reuse = min(REUSE_POINTS, prev_size); // how many points to reuse from last generated trajectory

          BehaviorState new_state = STATE_START;

          if (initialize) {
            VehicleState init_s{car_s, car_speed, 0.0};
            VehicleState init_d{car_d, 0.0, 0.0};

            ego_car.set_state_s(init_s);
            ego_car.set_state_d(init_d);

            new_state = STATE_START;
            initialize = false;
          } else {
            // int cur = NUM_POINTS - prev_size + prev_reuse - 1;
            int cur = NUM_POINTS - prev_size + prev_reuse;
            ego_car.set_state_s(prev_path_frenet.s[cur]);
            ego_car.set_state_d(prev_path_frenet.d[cur]);
          }

          // Update state using the behavior planner to get new state
          /*BehaviorPlanner planner{ego_car, lane_cars};
          BehaviorState new_state = planner.get_next_state();*/

          TrajectoryGenerator generator{ego_car, new_state};
          Trajectory traj = generator.get_trajectory();

          Path p = motion.generate_path(
              generator.get_jmt_s(),
              generator.get_jmt_d(),
              prev_path_frenet,
              prev_path_xy,
              prev_reuse);

          prev_path_frenet = p.frenet_coords;

          msgJson["next_x"] = p.xy_coords.get_x();
          msgJson["next_y"] = p.xy_coords.get_y();

          auto msg = "42[\"control\"," + msgJson.dump() + "]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

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
}