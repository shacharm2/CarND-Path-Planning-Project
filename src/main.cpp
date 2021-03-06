/*
	cost function - to deal with how to change lanes, start @ quiz
								- what's the cost of being in every lane, predict into the future location of cars
	https://github.com/ckyrkou/naive_bayes_classifier_cpp
*/
#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include "JMT.h"
// #include "trajectory.h"
#include "vehicle.h"
// #include "costs.h"
#include "utils.h"

using namespace std;

using Eigen::MatrixXd;
using Eigen::VectorXd;
// using std::vector;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
// constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

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


/*
double calc_velocity(double dv, a_max
				//double dv = -5;
				double a = abs(dv/(T - t_prev));
				while (a > 0.5 * a_max && dv < 0){
					dv += 0.01;
					a = abs(dv/(T - t_prev));
				}
				ref_vel += dv;
				ref_vel = max(neighbor_speed, ref_vel);
*/

float inefficiency_cost(int target_speed, int intended_lane, int final_lane, vector<int> lane_speeds) {
    /*
    Cost becomes higher for trajectories with intended lane and final lane that have traffic slower than target_speed.
    */
    
    //TODO: Replace cost = 0 with an appropriate cost function.
    float cost = 1 - exp(-(2 * target_speed - (float)lane_speeds[intended_lane] - (float)lane_speeds[final_lane]) / target_speed);

    return cost;
}


vector<Vehicle> generateNeighbors(vector<vector<double>> sensor_fusion)
{
	vector<Vehicle> neighbors;
	const int nID = 0, nX = 1, nY = 2, nVX = 3, nVY = 4, nS = 5, nD = 6;
	for (int i = 0; i < sensor_fusion.size(); i++)
	{
		// assumptions
		// vehicle is moving straight on S, no acceleration
		// d velocity mean is zero, such that d_dot[0] is ignore
		double neighbor_speed = sqrt(pow(sensor_fusion[i][nVX], 2) + pow(sensor_fusion[i][nVY], 2));		
		vector<double> S({sensor_fusion[i][nS], neighbor_speed, 0});
		vector<double> d({sensor_fusion[i][nD], 0, 0});
		neighbors.push_back(Vehicle(S, d));
	}
	return neighbors;
}

int getFrontVehicle(vector<vector<double>> sensor_fusion, int target_lane, double car_s, double car_speed, double t_bias, double& dist)
{
	const int nID = 0, nX = 1, nY = 2, nVX = 3, nVY = 4, nS = 5, nD = 6;
	double closest_dist = 1000;
	int front_vehicle = -1;
	for (int i = 0; i < sensor_fusion.size(); i++)
	{
		double neighbor_s = sensor_fusion[i][nS];
		double neighbor_d = sensor_fusion[i][nD];
		
		double vx_n = sensor_fusion[i][nVX];
		double vy_n = sensor_fusion[i][nVY];
		double neighbor_speed = sqrt(pow(vx_n, 2) + pow(vy_n, 2));

		int neighbor_lane = get_lane(sensor_fusion[i][nD]);
		// double dist = neighbor_s + (dt + t_prev) * neighbor_speed - car_s;
		//double front_dist = neighbor_s + t_bias * neighbor_speed - car_s;
		//double front_dist = neighbor_s + t_bias * neighbor_speed - car_s - t_bias * car_speed;
		double front_dist = neighbor_s + t_bias * neighbor_speed - car_s;
		if ((neighbor_lane == target_lane) && (front_dist > 0) && (front_dist < closest_dist))
		{
			closest_dist = front_dist;
			front_vehicle = i;
		}
	}
	dist = closest_dist;
	// cout << "closest_dist = " << dist << endl;
	return front_vehicle;
}



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

  int target_lane = 1;
//   double ref_vel = 0;
  Vehicle car(target_lane);
  string method = "JMT";
//   string method = "spline";
  

  h.onMessage([&car, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy, &target_lane](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
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

          	json msgJson;
			vector<double> next_x_vals, next_y_vals;

			/*
				set parameters	
			*/
			double T = 1; // [sec]
			double dt = T / 50;  // [sec] cycle time, step time
			double v_max = 21.5; // m/sec ~ 48 mph 
			double D = 30; // [m]
			double a_max = 10; // m/s^2
			double max_jerk = 10; // m/s^3
			double prev_pos_x, prev_pos_y, pos_x = car_x, pos_y = car_y, pos_s = car_s, pos_d = car_d, angle = deg2rad(car_yaw);
			// double safe_dist = D / 2;

			car_speed /= 2.24;
			double safe_dist = D;//+ T * car_speed;

			int prev_path_size = previous_path_x.size();
			double t_prev = dt * prev_path_size;

			// initial location
			car.set_sdt(car_s, car_d, car_speed, 0);
			car.set_safe_distance(safe_dist);

			vector<Vehicle> neighbors = generateNeighbors(sensor_fusion);
			car.set_neighbors(neighbors);
			double velocity = car_speed;

			if(prev_path_size < 2)
			{
				prev_pos_x = pos_x - cos(angle);
				prev_pos_y = pos_y - sin(angle);
			}
			else
			{
				pos_x = previous_path_x[prev_path_size-1];
				pos_y = previous_path_y[prev_path_size-1];

				prev_pos_x = previous_path_x[prev_path_size-2];
				prev_pos_y = previous_path_y[prev_path_size-2];

				angle = atan2(pos_y-prev_pos_y, pos_x-prev_pos_x);
				velocity = sqrt(pow((pos_x - prev_pos_x) / dt, 2) + pow((pos_y - prev_pos_y) / dt, 2));
			}
			
			if (prev_path_size > 0) {
				pos_s = end_path_s;
				pos_d = end_path_d;
			}

			// initial location
			car.set_sdt(pos_s, pos_d, velocity, t_prev);

			// target velocity
			car.set_reference_velocity(t_prev, T, safe_dist);

			// cost based lane selection
			target_lane = car.select_lane();

			// generat appropriate trajectory
			// how about to generate all trajectories prior to lane selection and then calculate total trajectory cost?
			car.generate_trajectory(previous_path_x, previous_path_y, t_prev, T, D, car_speed, angle, prev_pos_x, pos_x, prev_pos_y, pos_y,
				pos_s, pos_d, map_waypoints_x, map_waypoints_y, map_waypoints_s, next_x_vals, next_y_vals);

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
}
