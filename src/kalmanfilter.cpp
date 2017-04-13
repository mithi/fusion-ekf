#include "kalmanfilter.h"

void KalmanFilter::start(int n, VectorXd x, MatrixXd P, MatrixXd F, MatrixXd Q){

  this->n = n;
  this->I = MatrixXd::Identity(n, n);
  this->x = x;
  this->P = P;
  this->F = F;
  this->Q = Q;
}

void KalmanFilter::setQ(MatrixXd &Q){
  this->Q = Q;
}

void KalmanFilter::updateF(double dt){
  this->F(0, 2) = dt;
  this->F(1, 3) = dt;
}

VectorXd KalmanFilter::get() const{
  return this->x;
}

void KalmanFilter::predict(){
  this->x = this->F * this->x;
  this->P = this->F * this->P * this->F.transpose() + this->Q;
}

void KalmanFilter::update(const VectorXd &z, const MatrixXd &H, const VectorXd &Hx, const MatrixXd &R){

  MatrixXd PHt = this->P * H.transpose();
  VectorXd y = z - Hx;
  MatrixXd S = H * PHt + R;
  MatrixXd K = PHt * S.inverse();

  this->x = this->x + K * y;
  this->P = (this->I - K * H) * this->P;
}
