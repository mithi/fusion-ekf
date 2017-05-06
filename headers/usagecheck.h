#ifndef USAGECHECK_H_
#define USAGECHECK_H_

#include <fstream>
#include <iostream>
#include <stdlib.h>
#include <iomanip>
#include <vector>
#include "datapoint.h"
#include "../src/Eigen/Dense"

void check_arguments(int argc, char* argv[]);
void check_files(ifstream& in_file, string& in_nams, ofstream& out_file, string& out_name);
void print_EKF_data(const VectorXd &RMSE, const vector<VectorXd> &estimations,
    const std::vector<VectorXd> &ground_truths, const std::vector<DataPoint> &all_sensor_data);

#endif /* USAGECHECK_H_ */
