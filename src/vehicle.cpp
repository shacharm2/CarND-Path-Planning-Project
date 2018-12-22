#include "vehicle.h"


vector< vector<double> > get_frenet_full_state(double x, double y, double angle, double speed, double acc, double sampling_time, const vector<double> &maps_x, const vector<double> &maps_y)
{
	double vx = speed * cos(angle);
	double vy = speed * sin(angle);
	double ax = acc * cos(angle);
	double ay = acc * sin(angle);
	// cout << "get_frenet_full_state inputs" << endl;
	// cout << x << "," << y << "," << speed << "," << acc << "," << angle << endl;
	// cout << "vx,vy,ax,ax=" << vx << "," << vy << "," << ax << "," << ay << endl;	

	double dt = sampling_time;
	double x_prev = x - vx * dt - 0.5 * ax * dt * dt;
	double y_prev = y - vy * dt - 0.5 * ay * dt * dt;

	vector<double> sd = getFrenet(x, y, angle, maps_x, maps_y);
	vector<double> sd_prev = getFrenet(x_prev, y_prev, angle, maps_x, maps_y);

	// (x,y) : (d,s)
	double sd_angle = atan2(sd[0] - sd_prev[0], sd[1] - sd_prev[1]);
	vector<double> s_full({sd[0], speed * sin(sd_angle), acc * sin(sd_angle)});
	vector<double> d_full({sd[1], speed * cos(sd_angle), acc * cos(sd_angle)});

	
	// double sd_angle = atan2(dx, dy);

	// double vs = speed * sin(sd_angle);
	// double vs = speed * sin(sd_angle);


	// // calc s, s_dt, s_dt2, d, d_dt, d_dt2
	// s_full.push_back(s[0]);
	// s_full.push_back((s[0] - s[1]) / sampling_time);
	// s_full.push_back((s[0] - 2 * s[1] + s[2]) / pow(sampling_time, 2));
	// cout << "s_full = [" << s_full[0] << "," << s_full[1] << "," << s_full[2] << "]" << endl;
	
	// d_full.push_back(d[0]);
	// d_full.push_back((d[0] - d[1]) / sampling_time);
	// d_full.push_back((d[0] - 2 * d[1] + d[2]) / pow(sampling_time, 2));
	// cout << "d_full = [" << d_full[0] << "," << d_full[1] << "," << d_full[2] << "]" << endl<< endl;
	

	vector< vector<double> > output = {s_full, d_full};
	return output;
}



Vehicle::Vehicle(vector<double> S, vector<double> d)
{
	this->s_state = vector<double>(S);
	this->d_state = vector<double>(d);
	this->lane = get_lane(d[0]);
	this->state = LANE_KEEP;
}


Vehicle::Vehicle(const int lane)//, const double sampling_time)
{
	this->lane = lane;
	// this->sampling_time = sampling_time;
	this->s_state = vector<double>({0, 0, 0});
	this->d_state = vector<double>({get_pose(lane), 0, 0});

	this->state = LANE_KEEP;
}

Vehicle::Vehicle(const Vehicle &vehicle)
{
	this->s_state = vehicle.s_state;
	this->d_state = vehicle.d_state;
	this->state = vehicle.state;
	this->lane = vehicle.lane;
}

void Vehicle::set_sdt(const double s, const double d, const double t)
{
	this->s_state[0] = s;
	this->d_state[0] = d;
	this->t = t;
	this->lane = get_lane(d);
}

void Vehicle::set_neighbors(vector<Vehicle>& neighbors)
{
	this->neighbors = vector<Vehicle>(neighbors);
}

vector<double> Vehicle::get_loc() const
{
	return vector<double>({this->s_state[0], this->d_state[0]});
}

vector<double> Vehicle::get_vel() const
{
	return vector<double>({this->s_state[1], this->d_state[1]});
}

vector<double> Vehicle::get_acc() const
{
	return vector<double>({this->s_state[2], this->d_state[2]});
}

vector<double> Vehicle::get_distances(int lane)
{
	vector<double> dists;
	for (int in = 0; in < this->neighbors.size(); in++)
	{
		if (this->neighbors[in].lane != lane) {
			continue;
		}
		dists.push_back(this->neighbors[in].predict(this->t) - this->predict(this->t));
		// cout << "(" << in << ", " << this->neighbors[in].predict(this->t) - this->predict(this->t) << ") ";
	}
	// cout << endl;
	return dists;
}

void Vehicle::generate_trajectories()
{
	vector<int> candidates;
	if (this->lane == 1) {
		candidates = vector<int>({0, 1, 2});
	}
	else if (this->lane == 0) {
		candidates = vector<int>({0, 1});
	}
	else { // 2
		candidates = vector<int>({1, 2});
	}
}

int Vehicle::select_lane()
{

}

void Vehicle::set_location(double x, double y, double s, double d, double yaw, double speed, double acc)
{
	this->init_loc = PhysicalState({x, y, s, d, yaw, speed, acc});

	// this->curr_loc.x = this->trajectory_x[last];
	// this->curr_loc.y = this->trajectory_y[last];

}


int Vehicle::get_leading(int target_lane, double& distance)
{
	if (target_lane == -1) {
		target_lane = this->lane;
	}

	double closest_dist = 1000;
	int n_leading = -1;
	
	for (int in = 0; in < neighbors.size(); in++)
	{
		// double front_dist = (neighbors[in].s_state[0] + neighbors[in].s_state[1] * this->sampling_time) - (this->s_state[0] + this->s_state[1] * this->sampling_time);

		//double front_dist = neighbors[in].predict(this->t + 0.02) - this->s_state[0];
		double front_dist = neighbors[in].predict(this->t + 0.02) - this->get_loc()[0]; //s_state[0];
		if ((target_lane == neighbors[in].lane) && (front_dist > 0) && (front_dist < closest_dist))
		{
			// cout << in << " : lane " << neighbors[in].lane << ", s=" << neighbors[in].s_state[0] - this->s_state[0] << endl;
			closest_dist = front_dist;
			n_leading = in;
		}
	}
	// cout << in << " : " << front_dist << endl;
	// cout << "closest_dist[" << this->lane <<  "] = " << closest_dist << "@ " << n_leading << endl;
	// if (this->front_vehicle) {
	// 	cout << closest_dist << ", " << this->front_vehicle->s_state[0] << ", " << this->s_state[0] << endl;
	// 	cout << "this->front_vehicle=" << this->front_vehicle << endl;
	// }
	distance = closest_dist;
	return n_leading;
}

int Vehicle::get_trailing(int target_lane, double& distance)
{
	if (target_lane == -1) {
		target_lane = this->lane;
	}

	double closest_dist = 1000;
	int n_traling = -1;
	
	for (int in = 0; in < neighbors.size(); in++)
	{
		//double back_dist = this->s_state[0] - neighbors[in].predict(this->t + 0.02);
		double back_dist = this->get_loc()[0] - neighbors[in].predict(this->t + 0.02);
		if ((target_lane == neighbors[in].lane) && (back_dist >= 0) && (back_dist < closest_dist))
		{
			// cout << in << " : lane " << neighbors[in].lane << ", s=" << neighbors[in].s_state[0] - this->s_state[0] << endl;
			closest_dist = back_dist;
			n_traling = in;
		}
	}
	distance = -closest_dist;
	return n_traling;
}


double Vehicle::predict(const double t)
{
	double pos_s = this->get_loc()[0];
	double speed_s = this->get_vel()[0];
	double acc_s = this->get_acc()[0];
	return pos_s + speed_s * t + 0.5 * acc_s * t * t;
}


bool Vehicle::is_infront_of(Vehicle& car, double& dist)
{
	// s_state, d_state
	// cout <<this->lane <<"," <<car.lane<<endl;
	if (this->lane != car.lane) {
		return false;
	}
	// cout << "this->s_state[0]" 
	dist = this->s_state[0] - car.s_state[0];
	return true;
}


void Vehicle::set_target(const double pos_x, const double pos_y, const double v_start, const double v_end, const double t_start, 
	const double t_end, const double angle, const int target_lane)
{
	// assuming zero acceleration?
	double dt = t_end - t_start;
	// cout << "a = " << a << endl;
	// double x_target = pos_x + dt * v_start * cos(angle) + 0.5 * a * cos(angle) * dt * dt;
	// double y_target = pos_y + dt * v_start * sin(angle) + 0.5 * a * sin(angle) * dt * dt;

	// if (jmt_estimator_s.init)
	// {
	// 	s_state = this->jmt_estimator_s.eval(t_start);
	// 	d_state = this->jmt_estimator_d.eval(t_start);
	// }
	// else
	//if (d_state.empty())
	double a = (v_end - v_start) / dt;
	if (!this->jmt_estimator_d.init)
	{
		vector<double> sd_target = getFrenet(pos_x, pos_y, angle, maps_x, maps_y);
		d_state = vector<double>({get_pose(lane), 0, 0});
		s_state = vector<double> ({sd_target[0], v_start, a});
	}
	else
	{
		d_state = this->jmt_estimator_d.eval(t_start);
		s_state = this->jmt_estimator_s.eval(t_start);
	}

	// double x_target = pos_x + dt * v_end * cos(angle);
	// double y_target = pos_y + dt * v_end * sin(angle);
	double x_target = pos_x + dt * v_start * cos(angle) + 0.5 * a * cos(angle) * dt * dt;
	double y_target = pos_y + dt * v_start * sin(angle) + 0.5 * a * sin(angle) * dt * dt;

	vector<double> sd_target = getFrenet(x_target, y_target, angle, maps_x, maps_y);
	vector<double> d_final = vector<double>({get_pose(target_lane), 0, 0});
	vector<double> s_final;
	cout << "v_start = " << v_start << endl;
	cout << "s_state +10 vs st_target " << s_state[0] << ", " << sd_target[0] << endl;
	if (v_start < 10){
		cout << "low" << endl;
		s_final = vector<double> ({s_state[0] + 10, v_end, 0});
	}
	else {
		cout << "high" << endl;
		s_final = vector<double> ({sd_target[0], v_end, 0});
	}

	this->jmt_estimator_s.set_points(s_state, s_final, t_end - t_start + 0.02);
	this->jmt_estimator_d.set_points(d_state, d_final, t_end - t_start + 0.02);	
	// cout << "d_state = [" << d_state[0] << " " << d_state[1] << " " << d_state[2] << "]" << endl;
	// cout << "d_final = [" << d_final[0] << " " << d_final[1] << " " << d_final[2] << "]" << endl;
	// cout << "s_state = [" << s_state[0] << " " << s_state[1] << " " << s_state[2] << "]" << endl;
	// cout << "s_final = [" << s_final[0] << " " << s_final[1] << " " << s_final[2] << "]" << endl;

	// vector<double> s_full = car.jmt_estimator_s.eval(i * dt);
	// vector<double> d_full = car.jmt_estimator_d.eval(i * dt);
	int n = (int)((t_end - t_start) / 0.02);
	for (int i = 0; i < n; i++)
	{
		vector<double> S = this->jmt_estimator_s.eval(i * 0.02);
		cout << S[0] << ",";
	}
	cout << endl;

	this->lane = target_lane;
}


vector< vector<double> > Vehicle::get_init_target(const double x, const double y, const double angle, const double v, const double T, const int target_lane)
{
	double x_target = x + T * v * cos(angle);
	double y_target = y + T * v * sin(angle);
	// cout << "targets : " << x_target << "," << y_target << endl;
	
	vector<double> sd_target = getFrenet(x_target, y_target, angle, maps_x, maps_y);
	if (s_state.empty())
	{
		d_state = vector<double>({get_pose(target_lane), 0, 0});
		s_state = vector<double> ({sd_target[0], v, 0});
	}
	return vector< vector<double> >({s_state, d_state});	
}

vector< vector<double> > Vehicle::get_target(const double x, const double y, const double angle, const double v, const double T, const int target_lane)
{
	double x_target = x + T * v * cos(angle);
	double y_target = y + T * v * sin(angle);
	cout << "targets : " << x_target << "," << y_target << endl;
	vector<double> d_full, s_full;
	
	vector<double> sd_target = getFrenet(x_target, y_target, angle, maps_x, maps_y);
	//if (!init_target)
	if (s_state.empty())
	{
		d_state = vector<double>({get_pose(target_lane), 0, 0});
		s_state = vector<double> ({sd_target[0], v, 0});
	}
	return vector< vector<double> >({s_state, d_state});
}

vector< vector<double> > Vehicle::get_frenet() const
{
	
	vector< vector<double> > output;
	output = get_frenet_full_state(this->init_loc.x,
		this->init_loc.y,
		this->init_loc.yaw,
		this->init_loc.speed,
		this->init_loc.acc,
		this->sampling_time,
		maps_x,
		maps_y);

	// double vx = this->init_loc.speed * cos(this->init_loc.yaw);
	// double vy = this->init_loc.speed * sin(this->init_loc.yaw);
	// double ax = this->init_loc.acc * cos(this->init_loc.yaw);
	// double ay = this->init_loc.acc * sin(this->init_loc.yaw);
	// vector<double> s, d;
	// for (int i = 0; i < 3; i++)
	// {
	// 	double dt = i * this->sampling_time;
	// 	double x = this->init_loc.x - vx * dt - 0.5 * ax * dt * dt;
	// 	double y = this->init_loc.y - vy * dt - 0.5 * ay * dt * dt;
	// 	vector<double> sd = getFrenet(x, y, this->init_loc.yaw, maps_x, maps_y);
	// 	s.push_back(sd[0]);
	// 	d.push_back(sd[1]);
	// }

	// vector<double> s_full, d_full;
	// // calc s, s_dt, s_dt2, d, d_dt, d_dt2
	// s_full.push_back(s[0]);
	// s_full.push_back((s[0] - s[1]) / this->sampling_time);
	// s_full.push_back((s[0] - 2 * s[1] + s[2]) / pow(this->sampling_time, 2));
	
	// d_full.push_back(d[0]);
	// d_full.push_back((d[0] - d[1]) / this->sampling_time);
	// d_full.push_back((d[0] - 2 * d[1] + d[2]) / pow(this->sampling_time, 2));
	
	// vector< vector<double> > output = {}
	return output;
}


void Vehicle::cycle(PhysicalState physical_state, const vector<double> x, const vector<double> y)
{

	int i = 0;
	this->init_loc = physical_state;

	// no update and previous exists
	// const vector<double> * x_tr = &this->trajectory_x, * y_tr = &this->trajectory_y;
	if (x.size() < 4 && this->trajectory_x.size() > 3)
	{
		int size = this->trajectory_x.size();
		double vx = (this->trajectory_x[size-1] - this->trajectory_x[size-2]) / this->sampling_time;
		double vy = (this->trajectory_y[size-1] - this->trajectory_y[size-2]) / this->sampling_time;
		double prev_vx = (this->trajectory_x[size-2] - this->trajectory_x[size-3]) / this->sampling_time;
		double prev_vy = (this->trajectory_y[size-2] - this->trajectory_y[size-3]) / this->sampling_time;
		double ax = (vx - prev_vx) / this->sampling_time;
		double ay = (vy - prev_vy) / this->sampling_time;
		this->init_loc.acc = sqrt(ax*ax + ay*ay);

		// next - s & d
	}
	else if (x.size() > 3)
	{
		int size = x.size();

		double vx = (x[size-1] - x[size-2]) / this->sampling_time;
		double vy = (y[size-1] - y[size-2]) / this->sampling_time;
		double prev_vx = (x[size-2] - x[size-3]) / this->sampling_time;
		double prev_vy = (y[size-2] - y[size-3]) / this->sampling_time;
		double ax = (vx - prev_vx) / this->sampling_time;
		double ay = (vy - prev_vy) / this->sampling_time;
		this->init_loc.speed = sqrt(vx*vx + vy*vy);
		this->init_loc.acc = sqrt(ax*ax + ay*ay);
	}

	prev_trajectory_x = trajectory_x;
	prev_trajectory_y = trajectory_y;
	this->clear_trajectory();
	this->add(x, y);

	// cout << "acceleration init: " << this->init_loc.acc << endl;
	// dot([ax,ay], s_unit)
}

void Vehicle::add(const double x, const double y)
{
	this->trajectory_x.push_back(x);
	this->trajectory_y.push_back(y);
}

void Vehicle::add(const vector<double> x, const vector<double> y)
{
	assert (x.size() == y.size());
	this->trajectory_x.insert(this->trajectory_x.end(), x.begin(), x.end());
	this->trajectory_y.insert(this->trajectory_y.end(), y.begin(), y.end());

}

void Vehicle::update(const vector<double> x, const vector<double> y)
{
	int last = this->trajectory_x.size() - 1;
	this->curr_loc = PhysicalState(this->init_loc);
	this->curr_loc.x = this->trajectory_x[last];
	this->curr_loc.y = this->trajectory_y[last];

	double vx, vy, ax, ay;

	if (this->trajectory_x.size() > 1)
	{
		vx = (this->trajectory_x[last] - this->trajectory_x[last-1]) / this->sampling_time;
		vy = (this->trajectory_y[last] - this->trajectory_y[last-1]) / this->sampling_time;
		this->curr_loc.speed = sqrt(vx*vx + vy*vy);
	}

	if (this->trajectory_x.size() > 2)
	{
		double vx_prev = (this->trajectory_x[last-1] - this->trajectory_x[last-2]) / this->sampling_time;
		double vy_prev = (this->trajectory_y[last-1] - this->trajectory_y[last-2]) / this->sampling_time;
		ax = (vx - vx_prev) / this->sampling_time;
		ay = (vy - vy_prev) / this->sampling_time;
		// cout << "vx_prev, vx=" << vx_prev << ", " << vx << endl;
		// cout << "vy_prev, vy=" << vy_prev << ", " << vy << endl;
		this->curr_loc.acc = sqrt(ax*ax + ay*ay);
	}
}

void Vehicle::clear_trajectory()
{
	this->trajectory_x.clear();
	this->trajectory_y.clear();
	// this->init_loc = this->curr_loc;
}

vector<vector<double> > Vehicle::get_trajectory()
{
	vector<vector<double> > trajectory;
	trajectory.push_back(this->trajectory_x);
	trajectory.push_back(this->trajectory_y);
}