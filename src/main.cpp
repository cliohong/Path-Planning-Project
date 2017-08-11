#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <math.h>

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"

#include "polyTrajectoryGenerator.h"
#include "vehicle.h"
#include "jmt.h"
#include "spline.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;
using std::map;
using json = nlohmann::json;

constexpr double pi()    { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }
double logistic(double x){return (2.0 / (1 + exp(-x)) - 1.0);}

#define MAP_FILE                "../data/highway_map.csv"



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


//return (x,y)coordinates from frenet format
vector<double>convert_sd_2_xy(const double s, const double d,tk::spline const&spline_s_x, tk::spline const&spline_s_y, tk::spline const&spline_s_dx, tk::spline const&spline_s_dy){
  const double mod_s = fmod(s, MAX_TRACK_S);
  const double x_edges = spline_s_x(mod_s);
  const double y_edges = spline_s_y(mod_s);
  const double dx      = spline_s_dx(mod_s);
  const double dy      = spline_s_dy(mod_s);
  
  const double sx = x_edges + dx*d;
  const double sy = y_edges + dy*d;
  return  {sx,sy};
}


//create jmt waypoints from (s,d)coordinates
XYPoints jmtPath(JMT jmt_s, JMT jmt_d, const double delta_t, const int n, tk::spline const&spline_s_x, tk::spline const&spline_s_y, tk::spline const&spline_s_dx, tk::spline const&spline_s_dy){
  vector<double>sx;
  vector<double>sy;
  
  for(int i=0;i<n;i++){
    double s = jmt_s.jmt(delta_t*i);
    double d = jmt_d.jmt(delta_t*i);
    vector<double>pos = convert_sd_2_xy(s, d, spline_s_x, spline_s_y, spline_s_dx, spline_s_dy);
    sx.emplace_back(pos[0]);
    sy.emplace_back(pos[1]);
  }
  XYPoints path = {sx,sy,n};
  return path;
}

XYPoints get_start(Vehicle&car, tk::spline const&spline_s_x, tk::spline const&spline_s_y, tk::spline const&spline_s_dx, tk::spline const&spline_s_dy){
  const int n =200;
  const double t = n*PATH_PLAN_INCREMENT;
  const double target_v=20.0;
  const double target_s=car.s + 40;
  State state_start_s = {car.s, car.v,0.0};
  State state_start_d = {car.d, 0.0,0.0};
  
  State state_end_s = {target_s,target_v,0.0};
  State state_end_d = {car.convert_lane_to_d(), 0.0, 0.0};
  
  JMT jmt_s(state_start_s,state_end_s, t);
  JMT jmt_d(state_start_d,state_end_d, t);
  //update car's current state
  car.update_save_states(state_end_s, state_end_d);
  
  return jmtPath(jmt_s, jmt_d, PATH_PLAN_INCREMENT, n,spline_s_x , spline_s_y, spline_s_dx, spline_s_dy);
}


int main() {

  uWS::Hub h;
  // Load waypoints
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;
  bool first =true;
  double x1,y1,s1,dx1,dy1;
  string line;
  ifstream in_map_(MAP_FILE, ifstream::in);
  while (getline(in_map_, line))
  {
    istringstream iss(line);
    double x;
    double y;
    double s;
    double d_x;
    double d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.emplace_back(x);
    map_waypoints_y.emplace_back(y);
    map_waypoints_s.emplace_back(s);
    map_waypoints_dx.emplace_back(d_x);
    map_waypoints_dy.emplace_back(d_y);
    if(first){
      x1=x;
      y1=y;
      dx1=d_x;
      dy1=d_y;
      first=false;
    }
  }
  if(in_map_.is_open()){
    in_map_.close();
  }
  map_waypoints_x.emplace_back(x1);
  map_waypoints_y.emplace_back(y1);
  map_waypoints_dx.emplace_back(dx1);
  map_waypoints_dy.emplace_back(dy1);
  map_waypoints_s.emplace_back(MAX_TRACK_S);
  
  
  cout << "Program Started." << endl;
  bool start = true;

  cout << "Map loaded..." << endl;


  h.onMessage([&start,&map_waypoints_s,&map_waypoints_x,&map_waypoints_y,&map_waypoints_dx,&map_waypoints_dy](

    uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {

    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {

          //*********************************
          //* Get relevant data from simulator
          //*********************************

          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];

          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          vector<double> previous_path_x = j[1]["previous_path_x"];
          vector<double> previous_path_y = j[1]["previous_path_y"];
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          auto sensor_fusion = j[1]["sensor_fusion"];
          
          // Uses the loaded information to fit cubic polynomial curves
          tk::spline s_x_spline;
          tk::spline s_y_spline;
          tk::spline s_dx_spline;
          tk::spline s_dy_spline;
          
          s_x_spline.set_points (map_waypoints_s, map_waypoints_x);
          s_y_spline.set_points (map_waypoints_s, map_waypoints_y);
          s_dx_spline.set_points(map_waypoints_s, map_waypoints_dx);
          s_dy_spline.set_points(map_waypoints_s, map_waypoints_dy);
          
          

          //*********************************
          //* Update car object
          //*********************************

          Vehicle mycar;
          mycar.update_position(car_s, car_d);
          mycar.update_speed(car_speed);
          mycar.determine_lanes();

          //*********************************
          //* Generate the XY_points which will be sent to the simulator
          //*********************************

          int path_size = previous_path_x.size();
          XYPoints XY_points = {previous_path_x, previous_path_y, path_size};

          if (start) {
            
            XY_points = get_start(mycar, s_x_spline, s_y_spline, s_dx_spline, s_dy_spline);
            start = false;
            cout << "Engine started..." << endl;

          } else if (path_size < PATH_SIZE_CUTOFF) {

            vector<Vehicle> othercars;

            for (int i = 0; i < sensor_fusion.size(); i++) {

              int id = sensor_fusion[i][0];
              double s = sensor_fusion[i][5];
              double d = sensor_fusion[i][6];
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];

              Vehicle car;
              car.update_position(s, d);
              car.update_speed(sqrt(vx * vx + vy * vy));
              othercars.emplace_back(car);
            }

            // Print for debugging
            cout << "---------------------------------" << endl;
            cout << "STATE: s----d | x , y || v:" << endl;
            cout << car_s << " --- "
                 << car_d << " | "
                 << car_x << " , "
                 << car_y  << " || "
                 << car_speed << ":" << endl;

            cout << "---------------------------------" << endl;
            cout << "our left:  our lane:   our right:" << endl;
            LaneType mylane_l = mycar.lane_at_left;
            LaneType mylane_r = mycar.lane_at_right;
            LaneType mylane   = mycar.lane;
            vector<LaneType>lanes={mylane_l,mylane,mylane_r};
            for(int i=0;i<lanes.size();i++){
              if(lanes[i]==LaneType::LEFT){
                cout << "LEFT       ";
              }else if(lanes[i]==LaneType::MID){
                cout << "MID       ";
              }else if (lanes[i]==LaneType::RIGHT){
                cout << "RIGHT       ";
              }else{
                cout<<"UNKNOWN  ...";
              }
            }
            cout << endl;

            cout << "---------------------------------" << endl;

            // Decide whether to turn left, turn right or keeplane based on given data
           
            polyTrajectoryGenerator polyTrajectoryGenerator;
            PathType pathtype = polyTrajectoryGenerator.update(mycar, othercars);
            
            polyTrajectoryGenerator.trajectory(mycar, pathtype);
            
            mycar.update_save_states(polyTrajectoryGenerator.target_state_s, polyTrajectoryGenerator.target_state_d);
            
            XYPoints newXYpoints = jmtPath(polyTrajectoryGenerator.get_jmt_s(), polyTrajectoryGenerator.get_jmt_d(), PATH_PLAN_INCREMENT, NUMBER_OF_POINTS, s_x_spline, s_y_spline, s_dx_spline, s_dy_spline);
            
            newXYpoints.n = NUMBER_OF_POINTS;
            

            // append generated points to previous
            XY_points.xs.insert(
              XY_points.xs.end(), newXYpoints.xs.begin(), newXYpoints.xs.end());

            XY_points.ys.insert(
              XY_points.ys.end(), newXYpoints.ys.begin(), newXYpoints.ys.end());

            XY_points.n = XY_points.xs.size();
          }

          //*********************************
          //* Send updated path plan to simulator
          //*********************************

          json msgJson;

          msgJson["next_x"] = XY_points.xs;
          msgJson["next_y"] = XY_points.ys;

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

  // We don't need this since we're not using HTTP but if it's removed the program doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {

    const std::string s = "<h1>Hello world!</h1>";

    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      res->end(nullptr, 0); // i guess this should be done more gracefully?
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
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
