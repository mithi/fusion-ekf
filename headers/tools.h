#ifndef TOOLS_H_
#define TOOLS_H_

#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <stdlib.h>

#include <vector>
#include "../src/Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

VectorXd convert_cartesian_to_polar(const VectorXd& v);
VectorXd convert_polar_to_cartesian(const VectorXd& v);
MatrixXd calculate_jacobian(const VectorXd &v);
VectorXd calculate_RMSE(const std::vector<VectorXd> &estimations, const std::vector<VectorXd> &ground_truths);

#endif /* TOOLS_H_ */
