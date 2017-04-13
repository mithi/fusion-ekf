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
   * DATAPOINT SAMPLE USAGE
   *******************************************************************/
  test_datapoints();

  /*******************************************************************
   * END
   *******************************************************************/  
  return 0;
}

void test_datapoints(){

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