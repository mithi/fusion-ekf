#ifndef FUSIONEKF_H_
#define FUSIONEKF_H_

#include "../src/Eigen/Dense"
#include "datapoint.h"
#include "kalmanfilter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class FusionEKF{

  private:
    const int _n = 4;
    const int _lidar_n = 2;
    const int _radar_n = 3;
    const double _ax = 9.0;
    const double _ay = 9.0;
    bool _initialized;
    long long _timestamp;
    MatrixXd _P;
    MatrixXd _F;
    MatrixXd _Q;
    MatrixXd _radar_R;
    MatrixXd _lidar_R;
    MatrixXd _lidar_H;
    KalmanFilter _KF;

  public:
    FusionEKF();
    void updateQ(const double dt);
    void compute(const DataPoint& data);
    void start(const DataPoint& data);
    void process(const DataPoint& data);
    VectorXd get_resulting_state() const;
};

#endif /* FUSIONEKF_H_*/
