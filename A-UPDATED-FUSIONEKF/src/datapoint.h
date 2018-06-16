#ifndef DATAPOINT_H_
#define DATAPOINT_H_

#include "../src/Eigen/Dense"
#include <stdlib.h>
#include <iostream>
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

enum class DataPointType{
  LIDAR, RADAR
};

class DataPoint{

  private:
    long long _timestamp;
    bool _initialized;
    DataPointType _data_type;
    VectorXd _raw;

  public:
    DataPoint();
    DataPoint(const long long timestamp, const DataPointType data_type, const VectorXd raw);
    void set(long timestamp, const DataPointType data_type, const VectorXd raw);
    VectorXd get_raw_data() const;
    VectorXd get_state() const;
    DataPointType get_type() const;
    long long get_timestamp() const;
    void print() const;
};

#endif /* DATAPOINT_H_*/
