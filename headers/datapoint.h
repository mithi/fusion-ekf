#ifndef DATAPOINT_H_
#define DATAPOINT_H_

#include "../src/Eigen/Dense"
#include <stdlib.h>
#include <iostream>
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

enum class DataPointType{
  LIDAR, RADAR, STATE
};

class DataPoint{

  private:
    long long timestamp;
    bool initialized;
    DataPointType data_type;
    VectorXd raw;

  public:
    DataPoint();
    DataPoint(const long long timestamp, const DataPointType data_type, const VectorXd raw);
    void set(long timestamp, const DataPointType data_type, const VectorXd raw);
    VectorXd get() const;
    VectorXd get_state() const;
    DataPointType get_type() const;
    long long get_timestamp() const;
    void print() const;
};

#endif /* DATAPOINT_H_*/
