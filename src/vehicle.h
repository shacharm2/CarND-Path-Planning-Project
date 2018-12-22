#pragma once
#include <string>
#include <vector>
#include <cassert>
#include <math.h>
#include "utils.h"
#include "trajectory.h"

using namespace std;


enum States
{ 
	LANE_KEEP,
	LCL,
	PLCL,
	LCR,
	PLCR
};

struct PhysicalState
{
	double x;
	double y;
	double s;
	double d;
	double yaw;
	double speed;
	double acc;
};


vector< vector<double> > get_frenet_full_state(double x, double y, double angle, double speed, double acc, double sampling_time, const vector<double> &maps_x, const vector<double> &maps_y);


class Vehicle
{
public:
	// C'tors
	Vehicle(vector<double> S, vector<double> d);
	Vehicle(const int lane);//, const double sampling_time);
    Vehicle(const Vehicle &vehicle);  // copy constructor

    // Vehicle(const Vehicle &obj);  // copy constructor

	//Vehicle::Vehicle(const Vehicle &obj) 

	~Vehicle() {}

	int get_leading(int target_lane, double& distance);
	int get_trailing(int target_lane, double& distance);
	double predict(const double t);
	void set_sdt(const double s, const double d, const double sdot, const double t);
	void generate_trajectories();
	vector<Vehicle> neighbors;
	int select_lane();
	vector<double> get_distances(int lane);

	bool is_infront_of(Vehicle& car, double& dist);

	void cycle(PhysicalState physical_state, const vector<double> x, const vector<double> y);
	vector< vector<double> > get_frenet() const;
	void set_neighbors(vector<Vehicle>& neighbors);
	vector<double> get_loc() const;
	vector<double> get_vel() const;
	vector<double> get_acc() const;

	void set_reference_velocity(const double tstart, const double tend, const double safe_dist);

	//double operator() (double x) const;
	void add(const double x, const double y);
	void add(const vector<double> x, const vector<double> y);
	void clear_trajectory();
	void set_location(double x, double y, double s, double d, double yaw, double speed, double acc = 0.);
	void set_target_velocity(const double v);
	void set_target(const double pos_x, const double pos_y, const double v_start, const double v_end, const double t_start, 
		const double t_end, const double angle, const int target_lane);

	vector<vector<double> > get_trajectory();
	vector< vector<double> > get_prev_target();
	vector< vector<double> > get_target(const double x, const double y, const double angle, const double v, const double T, const int target_lane);
	vector< vector<double> > get_init_target(const double x, const double y, const double angle, const double v, const double T, const int target_lane);

	PhysicalState init_loc, curr_loc;

	double t = 0;

	trajectory jmt_estimator_s = trajectory("JMT");
	trajectory jmt_estimator_d = trajectory("JMT");
	double sampling_time;
	double ref_vel;
	int lane;
private:
	
	const double v_max = 21.5; // m/sec ~ 48 mph 
	const double a_max = 10; // m/s^2
	const double max_jerk = 10; // m/s^3

	vector<double> s_state, d_state;
	const double frame_time = 1;
	void update(const vector<double> x, const vector<double> y);	
	
	double target_vel;
	double target_acc;

	vector<double> maps_x, maps_y;

	vector<double> trajectory_x;
	vector<double> trajectory_y;

	vector<double> prev_trajectory_x;
	vector<double> prev_trajectory_y;

	vector< vector<double> > target;
	vector< vector<double> > prev_target;

	string state;
};



