#pragma once
#include "vehicle.h"
#include "math.h"

double eval_is_leading_vehicle(Vehicle& car, int lane, double ref_dist)
{
	double cost = 1, dist = 10000;
	car.get_leading(lane, dist);
	// cost = 1 / (1 + exp(-dist / ref_dist)); 
	//cost = 1 / (1 + exp(- ref_dist / dist));
	cost = max((double)0, 1 - ref_dist / dist);
	return cost;
}

// double eval_leading_velocity_cost(Vehicle& car, int lane, double ref_dist)
// {
// 	vector<double> dists = car.get_distances(lane);
// 	double lane_leading_distance = 10000, lane_trailing_distance = -10000;
// 	int leading = car.get_leading(lane, lane_leading_distance);
// 	assert(lane_leading_distance > 0);

// 	// calculate costs
// 	double cost = exp(1 - lane_leading_distance / ref_dist); // passes 1 for small distance
// 	return cost;
// }

double eval_leading_distance_cost(Vehicle& car, int lane, double ref_dist)
{
	double lane_leading_distance = 10000;
	int leading = car.get_leading(lane, lane_leading_distance);
	assert(lane_leading_distance > 0);
	// cout << "leading " << lane << ", " << ref_dist << ", " << lane_leading_distance << endl;

	// calculate costs
	double cost = exp(1 - lane_leading_distance / ref_dist); // passes 1 for small distance
	// double cost = max((double)0, 1 - lane_leading_distance / ref_dist); // passes 1 for small distance	

	return cost;
}

double eval_leading_safety_cost(Vehicle& car, int lane, double ref_dist)
{
	double lane_leading_distance = 10000;
	int leading = car.get_leading(lane, lane_leading_distance);
	double cost = 0;
	if (leading != -1)
	{
		if (lane_leading_distance < ref_dist && lane_leading_distance >= 0) {
			cost = 1;
		}
	}
	return cost;
}

double eval_trailing_safety_cost(Vehicle& car, int lane, double ref_dist, double ref_vel)
{
	double lane_trailing_distance = -10000;
	int trailing = car.get_trailing(lane, lane_trailing_distance);
	assert(lane_trailing_distance < 0);
	double cost = 0;
	if (trailing != -1)
	{
		double trailing_vel = car.neighbors[trailing].get_vel()[0]; //s_state[1];
		if (abs(lane_trailing_distance) < ref_dist / 3 && lane != car.lane) { 
			cost = 1;
		}
		else if (-lane_trailing_distance < ref_dist && lane != car.lane && trailing_vel >= ref_vel) {
			cost = 1;
		}
	}					
	return cost;
}

double eval_change_lane_cost(Vehicle& car, int lane)
{
	//double cost = pow(car.lane - lane, 2);
	double cost = abs(car.lane - lane) > 1;
	return cost;
}