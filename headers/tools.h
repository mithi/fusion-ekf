#ifndef TOOLS_H_
#define TOOLS_H_

#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <stdlib.h>

#include <vector>
#include "../src/Eigen/Dense"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

VectorXd convert_cartesian_to_polar(const VectorXd v);
VectorXd convert_polar_to_cartesian(const VectorXd v);
MatrixXd calculate_jacobian(const VectorXd &v);
VectorXd calculate_RMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth);

#endif /* TOOLS_H_ */
