#ifndef KALMANFILTER_H_
#define KALMANFILTER_H_

#include "../src/Eigen/Dense"
#include "datapoint.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class KalmanFilter{

  private:
    int n;
    VectorXd x;
    MatrixXd P;
    MatrixXd F;
    MatrixXd Q;
    MatrixXd I;

  public:
    KalmanFilter(){};
    void start(int n, VectorXd x, MatrixXd P, MatrixXd F, MatrixXd Q);
    void setQ(MatrixXd &Q);
    void updateF(const double dt);
    VectorXd get() const;
    void predict();
    void update(const VectorXd &z, const MatrixXd &H, const VectorXd &Hx, const MatrixXd &R);
};

#endif /* KALMANFILTER_H_ */
