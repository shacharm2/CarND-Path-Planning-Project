#include "trajectory.h"

trajectory::trajectory(string type)
{
	this->type = type;
	this->init = false;
}

void trajectory::set_points(const vector<double>& x, const vector<double>& y)
{
	if (this->type == "spline")
	{
		spline_estimator.set_points(x, y);
		this->init = true;
	}
}

void trajectory::set_points(const vector<double>& start, const vector<double>& end, const double T)
{
	vector<double> jmt_params;
	if (this->type == "JMT")
	{
		// here x & y are start and end
		// for (int i = 0; i < start.size(); i++) {
		// 	cout << start[i] << " ";
		// }
		// cout << endl;

		this->jmt_params = JMT(start, end, T);
		this->init = true;
	}
}

double trajectory::operator() (double x) const
{
	double interpolate;
	if (this->type == "spline")
	{	
		interpolate = spline_estimator(x);
	}
	return interpolate;		
}

vector<double> trajectory::eval(double t) const
{
	vector<double> xxdtxdt2;
	if (this->type == "JMT")
	{	
		double x = 0, xdt = 0, xdt2 = 0;
		for (int i = 0; i < jmt_params.size(); i++) {
			x += jmt_params[i] * pow(t, i);
		}
		xxdtxdt2.push_back(x);

		xdt = jmt_params[1] + jmt_params[2] * t + 3 * jmt_params[3] * pow(t, 2) + 4 * jmt_params[4] * pow(t, 3) + 5 * jmt_params[5] * pow(t, 4); 
		xxdtxdt2.push_back(xdt);

		xdt2 = jmt_params[2] * t + 6 * jmt_params[3] * t + 12 * jmt_params[4] * pow(t, 2) + 20 * jmt_params[5] * pow(t, 3);
		xxdtxdt2.push_back(xdt2);

	}
	return xxdtxdt2;		
}
vector<double> trajectory::operator() (vector<double> x) const
{
	vector<double> interpolate;
	if (this->type == "spline")
	{	
		for (vector<double>::iterator _x = x.begin(); _x != x.end(); _x++) {
			interpolate.push_back(spline_estimator(*_x));
		}
	}

	return interpolate;
}


void trajectory::create_trajectory(vector<double> x, vector<double>y, const double start_d, const double end_d)
{
	int start_lane = get_lane(start_d);
	int end_lane = get_lane(end_d);
	
}