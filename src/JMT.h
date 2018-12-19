#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>

#include "Eigen-3.3/Eigen/Dense"
using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

// TODO - complete this function
vector<double> JMT(vector< double> start, vector <double> end, double T);

bool close_enough(vector< double > poly, vector<double> target_poly);
