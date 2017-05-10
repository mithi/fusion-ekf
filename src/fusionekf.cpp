#include "fusionekf.h"

FusionEKF::FusionEKF(){

  this->initialized = false;

  this->lidar_R = MatrixXd(this->lidar_n, this->lidar_n);
  this->radar_R = MatrixXd(this->radar_n, this->radar_n);
  this->lidar_H = MatrixXd(this->lidar_n, this->n);

  this->P = MatrixXd(this->n, this->n);
  this->F = MatrixXd::Identity(this->n, this->n);
  this->Q = MatrixXd::Zero(this->n, this->n);

  this->lidar_R << 0.0225, 0.0,
                   0.0, 0.0225;

  this->radar_R  << 0.09, 0.0, 0.0,
                    0.0, 0.0009, 0,
                    0.0, 0.0, 0.09;

  this->lidar_H << 1.0, 0.0, 0.0, 0.0,
                   0.0, 1.0, 0.0, 0.0;

  this->P << 1.0, 0.0, 0.0, 0.0,
             0.0, 1.0, 0.0, 0.0,
             0.0, 0.0, 1000.0, 0.0,
             0.0, 0.0, 0.0, 1000;
}

void FusionEKF::updateQ(const double dt){

  const double dt2 = dt * dt;
  const double dt3 = dt * dt2;
  const double dt4 = dt * dt3;

  const double r11 = dt4 * this->ax / 4;
  const double r13 = dt3 * this->ax / 2;
  const double r22 = dt4 * this->ay / 4;
  const double r24 = dt3 * this->ay / 2;
  const double r31 = dt3 * this->ax / 2;
  const double r33 = dt2 * this->ax;
  const double r42 = dt3 * this->ay / 2;
  const double r44 = dt2 * this->ay;

  this->Q << r11, 0.0, r13, 0.0,
             0.0, r22, 0.0, r24,
             r31, 0.0, r33, 0.0,
             0.0, r42, 0.0, r44;

  this->KF.setQ(Q);
}

void FusionEKF::start(const DataPoint& data){

  this->timestamp = data.get_timestamp();
  VectorXd x = data.get_state();
  this->KF.start(this->n, x, this->P, this->F, this->Q);
  this->initialized = true;
}

void FusionEKF::compute(const DataPoint& data){

  /**************************************************************************
   * PREDICTION STEP
   **************************************************************************/
  const double dt = (data.get_timestamp() - this->timestamp) / 1.e6;
  this->timestamp = data.get_timestamp();

  this->updateQ(dt);
  this->KF.updateF(dt);
  this->KF.predict();

  /**************************************************************************
   * UPDATE STEP
   **************************************************************************/
  const VectorXd z = data.get();
  const VectorXd x = this->KF.get();

  VectorXd Hx;
  MatrixXd R;
  MatrixXd H;

  if (data.get_type() == DataPointType::RADAR){

    VectorXd s = data.get_state();
    H = calculate_jacobian(s);
    Hx = convert_cartesian_to_polar(x);
    R =  this->radar_R;

  } else if (data.get_type() == DataPointType::LIDAR){

    H = this->lidar_H;
    Hx = this->lidar_H * x;
    R = this->lidar_R;
  }

  this->KF.update(z, H, Hx, R);
}

void FusionEKF::process(const DataPoint& data){
  this->initialized ? this->compute(data) : this->start(data);
}

VectorXd FusionEKF::get() const{
  return this->KF.get();
}
