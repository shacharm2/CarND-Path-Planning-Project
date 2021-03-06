#include <string>
#include <vector>
#include "utils.h"
#include "spline.h"
#include "JMT.h"

using namespace std;
using std::string;
using std::vector;


class trajectory 
{
public:
	trajectory(string type);
	~trajectory() {};
	void set_points(const vector<double>& x, const vector<double>& y);
	void set_points(const vector<double>& start, const vector<double>& end, const double T);
	void create_trajectory(vector<double> x, vector<double>y, const double start_d, const double end_d);
	vector<double> operator() (vector<double> x) const;
	double operator() (double x) const;
	vector<double> eval(double t) const;
	bool init;

private:
	string type;
	tk::spline spline_estimator;
	vector<double> jmt_params;
};

