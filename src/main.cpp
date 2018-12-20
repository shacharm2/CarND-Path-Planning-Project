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

float goal_distance_cost(int goal_lane, int intended_lane, int final_lane, float distance_to_goal) {
    /*
    The cost increases with both the distance of intended lane from the goal
    and the distance of the final lane from the goal. The cost of being out of the 
    goal lane also becomes larger as vehicle approaches the goal.
    */
    float cost = 1 - exp(-2*abs(2 * goal_lane - intended_lane - final_lane) / distance_to_goal);

    return cost;
}



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

  int car_lane = 1;
  double ref_vel = 0;
  Vehicle car(car_lane);
  string method = "JMT";
//   string method = "spline";
  

  h.onMessage([&car, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy, &car_lane, &ref_vel](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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
			double safe_dist = D + T * car_speed;

			int prev_path_size = previous_path_x.size();
			double t_prev = dt * prev_path_size;

			for(int i = 0; i < prev_path_size; i++)
			{
				next_x_vals.push_back(previous_path_x[i]);
				next_y_vals.push_back(previous_path_y[i]);
			}

			if (prev_path_size > 0) {
				pos_s = end_path_s;
				pos_d = end_path_d;
			}

			// initial location
			car.set_sdt(car_s, car_d, 0);

			/* temp hack */
			int prev_car_lane = car_lane;
			if (car_speed > 20)	{
				car_lane = 0;
			}
			/* temp hack */

			// sensor fusion
			const int nID = 0, nX = 1, nY = 2, nVX = 3, nVY = 4, nS = 5, nD = 6;
			double dampening = 0.5;
			double closest_dist = 1e99;
			double closest_neighbor = -1;
		
			vector<int> front_vehicles;
			vector<double> front_distances;
			vector<Vehicle> neighbors = generateNeighbors(sensor_fusion);
			// car.neighbors = vector<Vehicle>(neighbors);
			car.set_neighbors(neighbors);
			for (int i_lane = 0; i_lane < 3; i_lane++)
			{
				double distance = 1000;
				int front_curr_vehicle = car.get_leading(i_lane, distance);
				front_vehicles.push_back(front_curr_vehicle);
				front_distances.push_back(distance);
				// assert(n_leading == front_curr_vehicle);
			}
			cout << "front_distances[" << car_lane << "]=" << front_distances[car_lane] << " with id " << front_vehicles[car_lane] << endl ;


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
			
			vector<double> sd_init = getFrenet(pos_x, pos_y, angle, map_waypoints_x, map_waypoints_y);



			//
			int front_car_id = front_vehicles[car_lane];
			if (front_car_id == -1 || front_distances[car_lane] > 2 * safe_dist) {
				double dv = 5;
				double a = dv/(T - t_prev);
				while (a > 0.5 * a_max && dv > 0){
					dv -= 0.01;
					a = dv/(T - t_prev);
				}
				ref_vel += dv;
				ref_vel = min(v_max, ref_vel);
				cout << "accelerate = " << dv << " T-t_prev=" << T-t_prev << endl;
			}
			else if (front_distances[car_lane] > safe_dist) {

				cout << "match speed" << endl;
				double neighbor_speed = neighbors[front_car_id].s_state[1];
				if (neighbor_speed > ref_vel)
				{
					double dv = neighbor_speed - ref_vel;
					double dvs = (dv < 0 ? -1 : 1);
					double a = abs(dv/(T - t_prev));
					cout << a << " " << 0.5 * a_max << endl;
					while (a > 0.5 * a_max && dv != 0){
						dv = dvs * (abs(dv) - 0.05);
						a = abs(dv/(T - t_prev));
					}
					ref_vel += dv;
					ref_vel = min(v_max, ref_vel);
				}
			}			
			else if (front_distances[car_lane] < safe_dist) {
				double neighbor_speed = neighbors[front_car_id].s_state[1];

				double dv = -5;
				double a = abs(dv/(T - t_prev));
				while (a > 0.5 * a_max && dv < 0){
					dv += 0.01;
					a = abs(dv/(T - t_prev));
				}
				ref_vel += dv;
				ref_vel = max(neighbor_speed - 1, ref_vel);
				cout << "decelerate = " << dv << endl;
			}

/*
			else if (front_distances[car_lane] > safe_dist) {
				cout << "match speed" << endl;
				double vx = sensor_fusion[front_car_id][nVX];
				double vy = sensor_fusion[front_car_id][nVY];
				double neighbor_speed = sqrt(pow(vx, 2) + pow(vy, 2));
				
				ref_vel = min(neighbor_speed, ref_vel + 1);

			}
			else {
				double vx = sensor_fusion[front_car_id][nVX];
				double vy = sensor_fusion[front_car_id][nVY];
				double neighbor_speed = sqrt(pow(vx, 2) + pow(vy, 2));
				double closest_dist = front_distances[car_lane];
				double neighbor_car_s = sensor_fusion[front_car_id][nS];
				neighbor_car_s += prev_path_size * neighbor_speed * dt;

				if ((neighbor_car_s > car_s) && ((neighbor_car_s - car_s) < safe_dist / 2) && (ref_vel > neighbor_speed)) { //&& car_speed > neighbor_speed) {
					cout << "emergency break" << endl;
					ref_vel -= a_max * (T - t_prev) / 10;
				}
				
				else if ((neighbor_car_s > car_s) && ((neighbor_car_s - car_s) < safe_dist))
				{
					cout << "match speed" << endl;
					ref_vel = max(neighbor_speed - 0.1, ref_vel - 0.1);
				}

				// if (car_speed > neighbor_speed) {
				// 	//ref_vel = max(0.98 * neighbor_speed, ref_vel * exp(-dampening * (safe_dist - closest_dist) / safe_dist));
				// 	//ref_vel = ref_vel * exp(-dampening * (safe_dist - closest_dist) / safe_dist);
				// 	ref_vel = neighbor_speed; //ref_vel * exp(-dampening * (safe_dist - closest_dist) / safe_dist);
				// }
				// else {
				// 	ref_vel = neighbor_speed * exp(-dampening * (safe_dist - closest_dist) / safe_dist);
				// }
			}

			if (ref_vel == -1)	{
				ref_vel = min(v_max, car_speed + (T - t_prev) * a_max);
			}
*/
			// generate trajectories


			/* create a smooth trajetory  */

			// cout << "ref_vel=" << ref_vel << endl;
			// car.set_target(pos_x, pos_y, velocity, ref_vel, t_prev, T, angle, car_lane);

			// pos_s = sd_init[0];


			// did not work
			// vector<vector<double> > sd_full = car.get_frenet(map_waypoints_x, map_waypoints_y);
			// vector<double> s_full = sd_full[0], d_full = sd_full[1];
			// cout << "sfull = [" << s_full[0] << " " << s_full[1] << " " << s_full[2] << "]" << endl;

			// cout << car.init_loc.acc << ", " << sqrt(pow(s_full[2],2) + pow(d_full[2],2)) << endl;
			// double a0 = car.init_loc.acc, a1 = sqrt(pow(s_full[2],2) + pow(d_full[2],2));
			// assert( abs(a0 - a1) / (abs(a1) + 1e-6) < 1e-3 );

			// trajectory_planner
			trajectory trajectory_planner("spline");

			// previous anchors
			vector<double> anchor_x, anchor_y;
			anchor_x.push_back(prev_pos_x);
			anchor_y.push_back(prev_pos_y);

			anchor_x.push_back(pos_x);
			anchor_y.push_back(pos_y);
			// trajectory_planner.create_trajectory(vector<double> x, vector<double>y, const double start_d, const double end_d)

			// next anchors
			// double pos_d = get_pose(car_lane);
			for (int id = 0; id < 3; id++)
			{
				//vector<double> wp = getXY(sd_init[0] + (id+1) * D, sd_init[1], map_waypoints_s, map_waypoints_x, map_waypoints_y);
				//vector<double> wp = getXY(sd_init[0] + (id+1) * D, pos_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
				if (id == 0){
					pos_d = get_pose(prev_car_lane);
				}
				else if (id == 1) {
					pos_d = 0.5 * (get_pose(prev_car_lane) + get_pose(car_lane));
				}
				else {
					pos_d = get_pose(car_lane);
				}
				vector<double> wp = getXY(pos_s + (id+1) * D, pos_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
				anchor_x.push_back(wp[0]);
				anchor_y.push_back(wp[1]);
			}

			// transform to car coordiantes
			for (int ia = 0; ia < anchor_x.size(); ia++)
			{
				double _x = anchor_x[ia] - pos_x, _y = anchor_y[ia] - pos_y;
				anchor_x[ia] = _x * cos(angle) + _y * sin(angle);
				anchor_y[ia] = _y * cos(angle) - _x * sin(angle);
			}


			// create a spline
			trajectory estimator("spline");	// tk::spline spline_estimator;
			

			// spline_estimator.set_points(anchor_x, anchor_y);
			estimator.set_points(anchor_x, anchor_y);
			// start_s, start_d
			//pos_x, pos_y, angle
			D = pos_x + car_speed * (T - t_prev) +  0.5 * a_max * pow(T - t_prev, 2);

			double target_x = D, target_y = estimator(D); //spline_estimator(D);
			double distance = sqrt(pow(target_x, 2) + pow(target_y, 2));
			int N = (int)(distance / (dt * ref_vel)); // /2.24
			// D^2 + spline_estimator(D) ^ 2 = distance

			double next_d = 6, next_s = pos_s;
			double dist_inc = 0.5;
			vector<double> X, Y;
			double vx, vy;

			for(int i = 0; i < 50-prev_path_size; i++)
			{    
				if (false)
				{
					/*
					next_s = pos_s + (i+1) * dist_inc;
					vector<double> xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
					next_x_vals.push_back(xy[0]);
					next_y_vals.push_back(xy[1]);
					*/
				}
				else if (true)
				{
					double xc = (i+1) * target_x / N;
					double yc = estimator(xc); //spline_estimator(xc);
					// cout << "xc,yc=" << xc << "," << yc << endl;

					double x = xc * cos(angle) - yc * sin(angle) + pos_x;
					double y = xc * sin(angle) + yc * cos(angle) + pos_y;
					// vector<double> xy = getXY(xf_s, yf_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
					next_x_vals.push_back(x);
					next_y_vals.push_back(y);
					// car.add(x, y);
					X.push_back(x);
					Y.push_back(y);
					if (X.size() > 3)
					{
						X.erase(X.begin());
						Y.erase(Y.begin());
						double ax = (X[0] - 2 * X[1] + X[2]) / (dt * dt);
						double ay = (Y[0] - 2 * Y[1] + Y[2]) / (dt * dt);
					}
				}
				else
				{
					// cout << "est"<<endl;
					vector<double> s_full = car.jmt_estimator_s.eval(i * dt);
					vector<double> d_full = car.jmt_estimator_d.eval(i * dt);
					// will set them for next round
					// car.s_init = s_full;
					// car.d_init = d_full;
					cout << s_full[0] << ",";

					//vector<double> wp = getXY(sd_init[0] + (id+1) * D, pos_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
					//jmt_estimator_s.set_points(sd_targets_init[0], sd_targets_final[0]);
					//jmt_estimator_d.set_points(sd_targets_init[1], sd_targets_final[1]);
					// cout << "dfull = [" << d_full[0] << " " << d_full[1] << " " << d_full[2] << "]" << endl;
					vector<double> xy = getXY(s_full[0], d_full[0], map_waypoints_s, map_waypoints_x, map_waypoints_y);
					
					// cout << "next s/d, x/y :" << s_full[0] << "/" << d_full[0] << " | " << xy[0] << "/" << xy[1] << endl;
					next_x_vals.push_back(xy[0]);
					next_y_vals.push_back(xy[1]);
				}

			}


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
