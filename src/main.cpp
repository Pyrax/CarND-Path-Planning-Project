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
using std::cout;
using std::endl;

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

  // Add first waypoint again to close the loop:
  map_waypoints_x.push_back(map_waypoints_x[0]);
  map_waypoints_y.push_back(map_waypoints_y[0]);
  map_waypoints_s.push_back(TRACK_SIZE);
  map_waypoints_dx.push_back(map_waypoints_dx[0]);
  map_waypoints_dy.push_back(map_waypoints_dy[1]);

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

          // Update ego vehicle and other vehicles
          ego_car.set_position(car_s, car_d);
          ego_car.set_velocity(car_speed / MS_TO_MPH);

          auto number_cars = sensor_fusion.size();
          vector<Vehicle> cars{number_cars};
          for (int i = 0; i < number_cars; ++i) {
            // double other_car_id = sensor_fusion[i][0];
            // double other_car_x = sensor_fusion[i][1];
            // double other_car_y = sensor_fusion[i][2];
            double other_car_vx = sensor_fusion[i][3];
            double other_car_vy = sensor_fusion[i][4];
            double other_car_s = sensor_fusion[i][5];
            double other_car_d = sensor_fusion[i][6];

            double vel = sqrt(other_car_vx * other_car_vx + other_car_vy * other_car_vy);

            cars[i] = Vehicle{other_car_s, other_car_d, vel};
          }

          int prev_size = previous_path_x.size();

          if (initialize) {
            VehicleState init_s{car_s, car_speed, 0.0};
            VehicleState init_d{car_d, 0.0, 0.0};

            ego_car.set_state_s(init_s);
            ego_car.set_state_d(init_d);

            initialize = false;
          } else {
            int cur = NUM_POINTS - prev_size;
            ego_car.set_state_s(prev_path_frenet.s[cur]);
            ego_car.set_state_d(prev_path_frenet.d[cur]);

            /*auto cs = ego_car.get_state_s();
            auto cd = ego_car.get_state_d();
            cout  << "-----" << endl
                  << "STATE_S=" << "{" <<  cs.pos << "," << cs.vel << "," << cs.acc << "}" << endl
                  << "STATE_D=" << "{" <<  cd.pos << "," << cd.vel << "," << cd.acc << "}" << endl;*/
          }

          // Get best possible trajectory through our behavior planner
          BehaviorPlanner planner{ego_car, cars};
          Trajectory traj = planner.get_best_trajectory();
          ego_car.set_behavior(planner.get_next_state());

          Path p = motion.generate_path(
              traj.jmt.s,
              traj.jmt.d,
              prev_path_frenet,
              prev_path_xy);

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