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
#include "queue"

using namespace std;

using Eigen::MatrixXd;
using Eigen::VectorXd;
// using std::vector;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
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

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y-y),(map_x-x));

	double angle = fabs(theta-heading);
  angle = min(2*pi() - angle, angle);

  if(angle > pi()/4)
  {
    closestWaypoint++;
  if (closestWaypoint == maps_x.size())
  {
    closestWaypoint = 0;
  }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

int get_lane(double d)
{
	int lane = round((d - 2) / 4);
	lane = max(0, lane);
	return lane;
}

double get_pose(int lane)
{
	double d = 2 + 4 * lane;
	return d;
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
  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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

          	vector<double> next_x_vals;
          	vector<double> next_y_vals;

          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
			/*
				set parameters	
			*/
			double T = 1;
			double dt = 0.02;  // [sec] cycle time, step time
			double v_max = 21.5; // m/sec ~ 48 mph 
			double D = 30; // [m]

			double a_max = 0.8 * 9.8; // m/s^2
			double max_jerk = 10; // m/s^3
			int car_lane = get_lane(car_d);
			// car_lane = 0;
			int path_size = previous_path_x.size();
			double t_prev = dt * path_size;
			double prev_pos_x, prev_pos_y, pos_x = car_x, pos_y = car_y, pos_s = car_s, angle = deg2rad(car_yaw);


			car_speed /= 2.24;
			double safe_dist = 0.66 * D  + T * car_speed;

			// sensor fusion
			const int nID = 0, nX = 1, nY = 2, nVX = 3, nVY = 4, nS = 5, nD = 6;
			double dampening = 0.8;
			double closest_dist = -1e99;
			double closest_neighbor = -1;
			cout << "---------------" << endl;
			//double ref_vel = min(v_max, car_speed + (T - t_prev) * a_max);
			double ref_vel = -1;
			for (int i = 0; i < sensor_fusion.size(); i++)
			{
				double neighbor_s = sensor_fusion[i][nS];
				double neighbor_d = sensor_fusion[i][nD];
				
				double vx_n = sensor_fusion[i][nVX];
				double vy_n = sensor_fusion[i][nVY];
				double neighbor_speed = sqrt(pow(vx_n, 2) + pow(vy_n, 2));

				int neighbor_lane = get_lane(sensor_fusion[i][nD]);
				double dist = neighbor_s + (dt + t_prev) * neighbor_speed - car_s;

				// cout << "distances = ";
				// will do closest on each lane
				double dspeed = a_max * dt;
				cout << "dspeed=" << dspeed << endl;
				if ((neighbor_lane == car_lane) && (dist > 0))
				//if (neighbor_d > 4 * car_lane && neighbor_d < 4 * (car_lane + 1) && dist > 0) // && dist < closest_s)
				{
					// cout << "neighbor " << i << " speed " << neighbor_speed << " vs car speed " << car_speed << endl;
					// cout << "distances " << dist << ", N=" << neighbor_s + (dt + t_prev) * neighbor_speed << ", car=" << car_s << endl;;
					// cout << "dist " << dist << endl;
					closest_dist = dist;
					// cout << closest_dist << " " << car_speed  << " " << neighbor_speed << endl;
					if (closest_dist < 0 || closest_dist > safe_dist) {  // free road!
						cout << "accelerate" << endl;
						ref_vel = min(v_max, car_speed + dspeed);				//ref_vel = min(v_max, car_speed + (T - t_prev) * a_max);
					}
					else if (car_speed > neighbor_speed) {
						cout << "2 fast 2 close" << endl;
						ref_vel = max(neighbor_speed, car_speed * exp(-dampening * (safe_dist - closest_dist) / safe_dist)); 	//ref_vel = (car_speed - 2 < 0 ? 0 : 1);// * exp(-dampening * (safe_dist - closest_dist) / safe_dist);
					}
					else {
						cout << "close" << endl;
						ref_vel = neighbor_speed * exp(-dampening * (safe_dist - closest_dist) / safe_dist);
					}

					// cout << dist << ", " << (dist > 0) << ", ";
					// if (state == "accelerate" && (dist > 2 * safe_dist || dist < 0)) {
					// if (dist > 0 && dist < safe_dist / 10) {
					// 	//ref_vel = neighbor_speed * exp(-dampening * (safe_dist - dist) / safe_dist);
					// 	ref_vel = min(v_max, car_speed - 4);
					// 	cout << "fast break " << ref_vel << endl;
					// 	state = "slowdown";
					// }
					// else (dist > 2 * safe_dist) {
					// 	//ref_vel = min(v_max, car_speed + (T - t_prev) * a_max);
					// 	ref_vel = min(v_max, car_speed + 1);
					// 	cout << "max pedal " << ref_vel << ", v_max " <<  v_max << endl;
					// }

					// cout << state << endl;	
				}
				// cout << endl;
				// cout << "closest_dist = " << closest_s << endl;
			}

			if (ref_vel == -1)	{
				ref_vel = min(v_max, car_speed + (T - t_prev) * a_max);
			}


			/* create a smooth trajetory 
			*/
			vector<double> anchor_x, anchor_y;


			for(int i = 0; i < path_size; i++)
			{
				next_x_vals.push_back(previous_path_x[i]);
				next_y_vals.push_back(previous_path_y[i]);
			}



			if(path_size < 2)
			{
				// pos_x = car_x;// pos_y = car_y;// pos_s = car_s;// angle = car_yaw;
				prev_pos_x = pos_x - dt * ref_vel * cos(car_yaw);
				prev_pos_y = pos_y - dt * ref_vel * sin(car_yaw);
			}
			else
			{
				pos_x = previous_path_x[path_size-1];
				pos_y = previous_path_y[path_size-1];

				prev_pos_x = previous_path_x[path_size-2];
				prev_pos_y = previous_path_y[path_size-2];
				angle = atan2(pos_y-prev_pos_y, pos_x-prev_pos_x);
				vector<double> sd = getFrenet(pos_x, pos_y, angle, map_waypoints_x, map_waypoints_y);
				pos_s = sd[0];
			}

			// previous anchors
			anchor_x.push_back(prev_pos_x);
			anchor_y.push_back(prev_pos_y);

			anchor_x.push_back(pos_x);
			anchor_y.push_back(pos_y);

			// next anchors
			
			// set vend
			

			// int lane = 1;
			//double pos_d = 2 + 4 * lane;
			double pos_d = get_pose(car_lane);
			//pos_d = 2 + 4*lane
			// int lane = (int)((pos_d - 2) / 4);

			// cout <<"D="<<D<<endl;
			for (int id = 0; id < 3; id++)
			{
				// cout << "pos_s + (" << (id+1) << ") * D = " << pos_s + (id+1) * D << endl;
				vector<double> wp = getXY(pos_s + (id+1) * D, pos_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
				anchor_x.push_back(wp[0]);
				anchor_y.push_back(wp[1]);
			}

			// transform to car coordiantes
			// cout << "anchor_x.size()=" <<anchor_x.size()<<endl;
			// cout << "anchor_y.size()=" <<anchor_y.size()<<endl;
			// cout << "anchors" << endl;
			for (int ia = 0; ia < anchor_x.size(); ia++)
			{
				double _x = anchor_x[ia] - pos_x, _y = anchor_y[ia] - pos_y;
				anchor_x[ia] = _x * cos(angle) + _y * sin(angle);
				anchor_y[ia] = _y * cos(angle) - _x * sin(angle);
				// cout << anchor_x[ia] << "," << anchor_y[ia] << endl;
			}
			// cout << endl;

			// create a spline
			tk::spline spline_estimator;

			spline_estimator.set_points(anchor_x, anchor_y);

			// double vend = min(max_vel, (car_speed / 2.24) + 0.5 * T);
			// cout << "vend="<<vend<<endl;
			// // cout << "vend=" << vend << " " << "car_speed=" << (car_speed / 2.24) <<  endl;
			// D = min(D, T * (vend + car_speed / 2.24));

			// cout << "D=" << D << endl;

			double target_x = D, target_y = spline_estimator(D);
			double distance = sqrt(pow(target_x, 2) + pow(target_y, 2));
			int N = (int)(distance / (dt * ref_vel)); // /2.24

			// D^2 + spline_estimator(D) ^ 2 = distance
			

			// cout << "N=" << N << endl;
			// cout << "distance=" << distance << endl;
			// cout << "dt=" << dt << endl;car_speed / 2.24
			double next_d = 6, next_s = pos_s;
			double dist_inc = 0.5;
			vector<double> X, Y;
			double vx, vy;

			for(int i = 0; i < 50-path_size; i++)
			{    
				if (false)
				{
					next_s = pos_s + (i+1) * dist_inc;
					vector<double> xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
					next_x_vals.push_back(xy[0]);
					next_y_vals.push_back(xy[1]);
				}
				else
				{
					double xc = (i+1) * target_x / N;
					double yc = spline_estimator(xc);
					// cout << "xc,yc=" << xc << "," << yc << endl;

					double x = xc * cos(angle) - yc * sin(angle) + pos_x;
					double y = xc * sin(angle) + yc * cos(angle) + pos_y;
					// vector<double> xy = getXY(xf_s, yf_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
					next_x_vals.push_back(x);
					next_y_vals.push_back(y);
					X.push_back(x);
					Y.push_back(y);
					if (X.size() > 3)
					{
						X.erase(X.begin());
						Y.erase(Y.begin());
						// cout << "X vec " << X[0] << " "  << X[1] << " "  << X[2] << endl;
						double ax = (X[0] - 2 * X[1] + X[2]) / (dt * dt);
						double ay = (Y[0] - 2 * Y[1] + Y[2]) / (dt * dt);
						// cout << "total acc = " << sqrt(ax*ax + ay*ay) << endl;
					}

					// if (x_prev != -1)
					// {
 					// 	vx = (x - x_prev) / dt;
					// 	vy = (y - y_prev) / dt;
					// }


					// if (x_prev == -1) {
					// 	cout << "i, pos_x,pos_y, xc,yc, x,y=" << i << ", " << pos_x << " " << pos_y << ", " << xc << " " << yc << ", " << x << " " << y << endl;
					// }
					// else {
					// 	double vx = (x - x_prev) / dt, vy = (y - y_prev) / dt;
					// 	cout << "i, pos_x,pos_y, xc,yc, x,y, vx, vy=" << i << ", " << pos_x << " " << pos_y << ", " << xc << " " << yc << ", " << x << " " << y << "," << vx << " " << vy << endl;
					// }

				}
			}

			// for (int i = 0; i < next_x_vals.size(); i++) {
			// 	cout << next_x_vals[i] << " ";
			// }
			// cout << endl;
			// cout << "---------------------------------------------------------------" << endl;


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
