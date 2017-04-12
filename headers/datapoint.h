#ifndef DATAPOINT_H_
#define DATAPOINT_H_

#include "../src/Eigen/Dense"
#include <stdlib.h>
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std; 

enum class DataPointType{
  LIDAR, RADAR, STATE
};

class DataPoint{

  private:
    long timestamp;
    bool is_initialized;
    DataPointType data_type; 
    VectorXd raw;
  
  public: 
    DataPoint();
    DataPoint(const long timestamp, const DataPointType data_type, const VectorXd raw);
    void set(long timestamp, const DataPointType data_type, const VectorXd raw);
    void print() const;
    VectorXd get() const;
  
};

#endif /* DATAPOINT_H_*/
