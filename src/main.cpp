#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

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

  // Ego car's lane (start at the middle lane)
  int lane = 1;

  // Reference velocity
  double ref_vel = .0; // mph

  h.onMessage([&ref_vel, &lane, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
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
          
          // Previous path data given to the Planner
          // ["previous_path_x"] The previous list of x points previously given to the simulator
          // ["previous_path_y"] The previous list of y points previously given to the simulator
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];

          // Previous path's end s and d values 
          // ["end_path_s"] The previous list's last **point**'s frenet s value
          // ["end_path_d"] The previous list's last **point**'s frenet d value
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];


          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */  

          // Provided previous path point size.
           int prev_size = previous_path_x.size();

           // Preventing collisions
           if (prev_size > 0){
              car_s = end_path_s;
           }

           // Prediction: Predict other vehicle positions
           bool car_ahead = false;
           bool car_left = false;
           bool car_right = false;

           for (int i=0; i<sensor_fusion.size(); i++){
              float d = sensor_fusion[i][6];
              int car_lane = -1;
              // whether this car is on the same lane that ego vehicle drives on
              if (d > 0 && d < 4){
                  car_lane = 0;
              } else if (d > 4 && d < 8){
                  car_lane = 1;
              } else if (d > 8 && d < 12){
                  car_lane = 2;
              } 

              if (car_lane < 0){
                continue;
              }

              // grab car's speed
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_speed = sqrt(vx*vx + vy*vy);
              double check_car_s = sensor_fusion[i][5]; //car's s position in frenet coordinates
              // Estimate car's position after executing previous trajectory.
              // 查看车辆的未来位置
              check_car_s += ((double)prev_size * 0.02 * check_speed);

              if ( car_lane == lane ) {
                  // Car on the same lane.
                  car_ahead |= ((check_car_s > car_s) && (check_car_s - car_s < 30));
                } else if ( (car_lane - lane) == -1 ) {
                  // Car on the left lane.
                  car_left |= ((car_s - 30 < check_car_s) && (car_s + 30 > check_car_s));
                } else if ( (car_lane - lane) == 1 ) {
                  // Car on the right lane.
                  car_right |= ((car_s - 30 < check_car_s) && (car_s + 30 > check_car_s));
                }
           }

          // Behavior: take action (change lane/speed up/slow down/) approaching the car ahead.
          double speed_diff = 0;
          const double MAX_SPEED = 49.5;
          const double MAX_ACC = .224;

          if (car_ahead){
              // if there is a car ahead
              if (!car_left && lane > 0){
                  // if there is no car left and there is a left lane
                  lane--; // change to left lane
              } else if (!car_right && lane !=2){
                  // if there is no car right and there is right lane
                  lane++; // change to right lane
              } else {
                  speed_diff -= MAX_ACC;
              }
          } else {
              // if there is no car ahead
              if (lane != 1){
                // if the ego car is not on the middle lane
                if ((lane==0 && !car_right) || (lane==2 && !car_left)){
                  // 1.if the ego car is on the left lane and there is no car right
                  // 2.if the ego car is on the right lane and there is no car left
                  lane =1; // change back to the middle lane
                }
              }
              if (ref_vel < MAX_SPEED){
                  speed_diff += MAX_ACC;
              }
          }

          // Trajectory: take advantages of speed and lane output from behavior, car coordinates, and previous path points.
          // Create a list of widely space(x, y) waypoints, evenly spaced at 30m
          // Later we will interpolate these waypoints with a spline
          // 存储5个waypoints：2个来自previous path；3个来自接下来Frenet下的的30m, 60m, 90m
          // 利用这5个waypoints构建spline曲线
          vector<double> ptsx;  // store points in global coordinates
          vector<double> ptsy;

          // reference x, y, yaw
          // either we will reference the starting point as where the car is  
          // or at the previous points end point
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          // if previous size is almost empty, use the car as starting reference
          // 如果previous points不够，那么则用当前的位置通过ref_yaw来逆推出prev_car_x, prev_car_y
          // 并与car_x, car_y一同加入ptsx和ptsy 作为previous points
          if (prev_size < 2){
              // use two points that make the path tangent（相切） to the car
              double prev_car_x = car_x - cos(car_yaw);
              double prev_car_y = car_y - sin(car_yaw);

              ptsx.push_back(prev_car_x);
              ptsx.push_back(car_x);

              ptsy.push_back(prev_car_y);
              ptsy.push_back(car_y);

          } else {
              // redifine reference states as previous path end point
              // 如果previous points >= 2，那么将最近的2个points加入进来
              // 同时更新ref_yaw角度，并以此作为new planning path的starting positions
              ref_x = previous_path_x[prev_size-1];
              ref_y = previous_path_y[prev_size-1];

              double ref_x_prev = previous_path_x[prev_size - 2];
              double ref_y_prev = previous_path_y[prev_size - 2];
              ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

              // use two points that make the path tangent（相切） to the car
              ptsx.push_back(ref_x_prev);
              ptsx.push_back(ref_x);

              ptsy.push_back(ref_y_prev);
              ptsy.push_back(ref_y);
          }

          // In Frenet add evenly(均匀地) 30m spaced points ahead of the starting reference
          // transform to global coordinate
          // 将前方的30m，60m，90m的waypoints以Frenet坐标加入进来，同时转换成gloabl coordinates
          vector<double> next_wp0 = getXY(car_s + 30, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s + 60, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s + 90, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);

          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);

          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);

          // Transform to local car coordinates.
          // 转换为自车坐标
          for ( int i = 0; i < ptsx.size(); i++ ) {
              // shift car reference angle to 0 degree
              double shift_x = ptsx[i] - ref_x;
              double shift_y = ptsy[i] - ref_y;

              ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
              ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
          }

          // Create a spline
          tk::spline s;

          // set (x,y) points to the spline
          // 利用pts里的5个waypoints构建一条spline样条曲线
          s.set_points(ptsx, ptsy);

          // Define the actual(x, y) points in global coordinates used for the planner
          // from previous path for continuity
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // Start with all of the previous path points from last time
          // 将全部的previous points（<=50个）加入到next_vals中
          for (int i=0; i<previous_path_x.size(); i++){
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
          }

          // Calculate how to break up spline points so that travelling at the desired reference velocity
          // Calculate distance y position 30m ahead
          // spline曲线由相隔较远的5个waypoints构成，在此利用该spline计算local坐标系下接下来30m的轨迹
          // 每0.02秒为一个时间间隔计算出30m内每一个小间隔的x值以及对应的spline(x)值
          // 最后转换为global coordinates
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt(target_x*target_x + target_y*target_y);

          double x_add_on = 0;

          // Fill up the rest of the path planner after filling it with previous points
          for (int i=1; i<50-prev_size; i++){
              ref_vel += speed_diff;
              if (ref_vel > MAX_SPEED){
                ref_vel = MAX_SPEED;
              } else if (ref_vel < MAX_ACC){
                ref_vel = MAX_ACC;
              }

              // Calculate N, x_point, y_point
              double N = target_dist/(0.02*ref_vel/2.24);
              double x_point = x_add_on + target_x/N;
              double y_point = s(x_point);

              x_add_on = x_point;

              double x_ref = x_point;
              double y_ref = y_point;

              // Transform back to global coordinates
              x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
              y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

              x_point += ref_x;
              y_point += ref_y;

              next_x_vals.push_back(x_point);
              next_y_vals.push_back(y_point);
          }


          json msgJson;


          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

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