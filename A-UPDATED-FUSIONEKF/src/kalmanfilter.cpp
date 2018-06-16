#include "kalmanfilter.h"

void KalmanFilter::start(
  const int nin, const VectorXd& xin, const MatrixXd& Pin, const MatrixXd& Fin, const MatrixXd& Qin){

  _n = nin;
  _I = MatrixXd::Identity(_n, _n);
  _x = xin;
  _P = Pin;
  _F = Fin;
  _Q = Qin;
}

void KalmanFilter::setQ(const MatrixXd& Qin){
  _Q = Qin;
}

void KalmanFilter::updateF(const double dt){
  _F(0, 2) = dt;
  _F(1, 3) = dt;
}

VectorXd KalmanFilter::get_resulting_state() const{
  return _x;
}

void KalmanFilter::predict(){
  _x = _F * _x;
  _P = _F * _P * _F.transpose() + _Q;
}

void KalmanFilter::update(const VectorXd& z, const MatrixXd& H, const VectorXd& Hx, const MatrixXd& R){

  const MatrixXd PHt = _P * H.transpose();
  const MatrixXd S = H * PHt + R;
  const MatrixXd K = PHt * S.inverse();
  VectorXd y = z - Hx;

  // Assume this is radar measurement
  // y(1) is an angle (phi), it shoulde be normalized
  // refer to the comment at the bottom of this file
  if (y.size() == 3) y(1) = atan2(sin(y(1)), cos(y(1)));

  _x = _x + K * y;
  _P = (_I - K * H) * _P;
}

/*
Normalizing Angles
In C++, atan2() returns values between -pi and pi.
When calculating phi in y = z - h(x) for radar measurements,
the resulting angle phi in the y vector should be adjusted
so that it is between -pi and pi. The Kalman filter is expecting
small angle values between the range -pi and pi.
*/
