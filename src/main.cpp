#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <stdlib.h>
#include "Eigen/Dense"
#include "usagecheck.h"
#include "datapoint.h"
#include "tools.h"
#include "fusionekf.h"

using namespace std;
using std::vector;
using Eigen::MatrixXd;
using Eigen::VectorXd;

int main(int argc, char* argv[]) {

  /*******************************************************************
   * CHECK IF CORRECTLY EXECUTED BY USER
   *******************************************************************/
  check_arguments(argc, argv);

  string in_filename = argv[1];
  string out_filename = argv[2];

  ifstream in_file(in_filename.c_str(), ifstream::in);
  ofstream out_file(out_filename.c_str(), ofstream::out);

  check_files(in_file, in_filename, out_file, out_filename);

  /*******************************************************************
   * READ DATA FROM FILE AND STORE IN MEMORY
   *******************************************************************/
  vector<DataPoint> all_sensor_data;
  vector<DataPoint> all_truth_data;

  double val1, val2, val3;
  double x, y, vx, vy;
  long long timestamp;
  string sensor_id;

  string line;

  while(getline(in_file, line)){

    istringstream iss(line);
    DataPoint sensor_data;
    DataPoint truth_data;

    iss >> sensor_id;

    if (sensor_id.compare("L") == 0){

      iss >> val1;
      iss >> val2;
      iss >> timestamp;

      VectorXd lidar_vec(2);
      lidar_vec << val1, val2;
      sensor_data.set(timestamp, DataPointType::LIDAR, lidar_vec);

    } else if (sensor_id.compare("R") == 0){

      iss >> val1;
      iss >> val2;
      iss >> val3;
      iss >> timestamp;

      VectorXd radar_vec(3);
      radar_vec << val1, val2, val3;
      sensor_data.set(timestamp, DataPointType::RADAR, radar_vec);
    }

    iss >> x;
    iss >> y;
    iss >> vx;
    iss >> vy;

    VectorXd truth_vec(4);
    truth_vec << x, y, vx, vy;
    truth_data.set(timestamp, DataPointType::STATE, truth_vec);

    all_sensor_data.push_back(sensor_data);
    all_truth_data.push_back(truth_data);
  }

  /*******************************************************************
   * USE DATA AND FUSIONEKF FOR STATE ESTIMATIONS
   *******************************************************************/
   FusionEKF fusionEKF;

   vector<VectorXd> estimations;
   vector<VectorXd> ground_truths;

  for (int k = 0; k < all_sensor_data.size(); ++k){

    fusionEKF.process(all_sensor_data[k]);

    VectorXd prediction = fusionEKF.get();
    VectorXd measurement = all_sensor_data[k].get_state();
    VectorXd truth =  all_truth_data[k].get();

    out_file << prediction(0) << "\t";
    out_file << prediction(1) << "\t";
    out_file << prediction(2) << "\t";
    out_file << prediction(3) << "\t";

    out_file << measurement(0) << "\t";
    out_file << measurement(1) << "\t";

    out_file << truth(0) << "\t";
    out_file << truth(1) << "\t";
    out_file << truth(2) << "\t";
    out_file << truth(3) << "\n";

    estimations.push_back(prediction);
    ground_truths.push_back(truth);
  }

  /*******************************************************************
   * CALCULATE ROOT MEAN SQUARE ERROR
   *******************************************************************/
   VectorXd RMSE = calculate_RMSE(estimations, ground_truths);
   cout << "Accuracy - RMSE:" << endl;
   cout << RMSE << endl;

  /*******************************************************************
   * PRINT TO CONSOLE IN A NICE FORMAT FOR DEBUGGING
   *******************************************************************/
   //print_EKF_data(RMSE, estimations, ground_truths, all_sensor_data);

  /*******************************************************************
   * CLOSE FILES
   *******************************************************************/
  if (out_file.is_open()) { out_file.close(); }
  if (in_file.is_open()) { in_file.close(); }

  return 0;
}
