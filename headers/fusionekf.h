#ifndef FUSIONEKF_H_
#define FUSIONEKF_H_

#include "../src/Eigen/Dense"
#include "datapoint.h"
#include "kalmanfilter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class FusionEKF{

  private:
    const int n = 4;
    const int lidar_n = 2;
    const int radar_n = 3;
    const double ax = 9.0; //5.0
    const double ay = 9.0; //5.0
    bool initialized;
    long long timestamp;
    MatrixXd P;
    MatrixXd F;
    MatrixXd Q;
    MatrixXd radar_R;
    MatrixXd lidar_R;
    MatrixXd lidar_H;
    KalmanFilter KF;

  public:
    FusionEKF();
    void updateQ(const double dt);
    void compute(const DataPoint& data);
    void start(const DataPoint& data);
    void process(const DataPoint& data);
    VectorXd get() const;
};

#endif /* FUSIONEKF_H_*/
