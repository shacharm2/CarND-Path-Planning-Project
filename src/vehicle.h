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
	Vehicle(const int lane);
	~Vehicle() {}
	void set_map(const vector<double> &maps_x, const vector<double> &maps_y);
	void cycle(PhysicalState physical_state, const vector<double> x, const vector<double> y);
	vector< vector<double> > get_frenet() const;

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
	vector<double> s_init, d_init;

	trajectory jmt_estimator_s = trajectory("JMT");
	trajectory jmt_estimator_d = trajectory("JMT");
private:

	const double sampling_time = 0.02;
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
	int lane;
};



