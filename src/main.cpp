#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <stdlib.h>
#include "Eigen/Dense"
#include "usagecheck.h"
#include "datapoint.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;
using std::vector;

void test_datapoints();
void print_data_points(vector<DataPoint> all_sensor_data, vector<DataPoint> all_truth_data);

int main(int argc, char* argv[]) {

  /*******************************************************************
   * CHECK IF CORRECTLY EXECUTED
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

  float val1, val2, val3;
  float x, y, vx, vy;  
  long timestamp;
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
   * PRINT ALL DATA POINTS
   *******************************************************************/
  print_data_points(all_sensor_data, all_truth_data);


   * DATAPOINT SAMPLE USAGE
   *******************************************************************/
  //test_datapoints()

  /*******************************************************************
   * END
   *******************************************************************/  
  return 0;
}

void print_data_points(vector<DataPoint> all_sensor_data, vector<DataPoint> all_truth_data) {

  for (long int k = 0; k < all_sensor_data.size(); k++){
      
    cout << "======================================" << endl;

    cout << "SENSOR DATA:" << k << endl;
    all_sensor_data[k].print();

    cout << "TRUTH DATA:" << k << endl;
    all_truth_data[k].print();
  }
}


void test_datapoints() {

  VectorXd lidar_v(2);
  lidar_v << 1.2, 2.5; 
  DataPoint lidar_data(1234, DataPointType::LIDAR, lidar_v);

  VectorXd radar_v(3);
  radar_v << 1.2, 2.5, 4.3; 
  DataPoint radar_data;
  radar_data.set(5678, DataPointType::RADAR, radar_v);

  VectorXd state_v(4);
  state_v << 1.2, 2.5, 7.0, 1.0; 
  DataPoint state_data(1234, DataPointType::STATE, state_v);

  cout << "lidar data" << endl;
  lidar_data.print();

  cout << "radar data" << endl;
  radar_data.print();

  cout << "state data" << endl;
  state_data.print();
  
  cout << "lidar data" << endl;
  cout << lidar_data.get() << endl;

  cout << "radar data" << endl;
  cout << radar_data.get() << endl;

  cout << "state data" << endl;
  cout << state_data.get() << endl;
}