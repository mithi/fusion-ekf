#include "fusionekf.h"

FusionEKF::FusionEKF(){

  _initialized = false;

  _lidar_R = MatrixXd(_lidar_n, _lidar_n);
  _radar_R = MatrixXd(_radar_n, _radar_n);
  _lidar_H = MatrixXd(_lidar_n, _n);

  _P = MatrixXd(_n, _n);
  _F = MatrixXd::Identity(_n, _n);
  _Q = MatrixXd::Zero(_n, _n);

  _lidar_R << 0.0225, 0.0,
              0.0, 0.0225;

  _radar_R  << 0.09, 0.0, 0.0,
               0.0, 0.0009, 0,
               0.0, 0.0, 0.09;

  _lidar_H << 1.0, 0.0, 0.0, 0.0,
              0.0, 1.0, 0.0, 0.0;

  _P << 1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1000.0, 0.0,
        0.0, 0.0, 0.0, 1000;
}

void FusionEKF::updateQ(const double dt){

  // ax, ay are acceleration covariances treated as noise

  const double dt2 = dt * dt;
  const double dt3 = dt * dt2;
  const double dt4 = dt * dt3;

  const double r11 = dt4 * _ax / 4;
  const double r13 = dt3 * _ax / 2;
  const double r22 = dt4 * _ay / 4;
  const double r24 = dt3 * _ay / 2;
  const double r31 = dt3 * _ax / 2;
  const double r33 = dt2 * _ax;
  const double r42 = dt3 * _ay / 2;
  const double r44 = dt2 * _ay;

  _Q << r11, 0.0, r13, 0.0,
        0.0, r22, 0.0, r24,
        r31, 0.0, r33, 0.0,
        0.0, r42, 0.0, r44;

  _KF.setQ(_Q);
}

void FusionEKF::start(const DataPoint& data){

  _timestamp = data.get_timestamp();
  VectorXd x = data.get_state();
  _KF.start(_n, x, _P, _F, _Q);
  _initialized = true;
}

void FusionEKF::compute(const DataPoint& data){

  /**************************************************************************
   * PREDICTION STEP
   - Assumes current velocity is the same for this time period
   **************************************************************************/
  const double dt = (data.get_timestamp() - _timestamp) / 1.e6;
  _timestamp = data.get_timestamp();

  this->updateQ(dt);
  _KF.updateF(dt); 
  _KF.predict();

  /**************************************************************************
   * UPDATE STEP
   - Updates appropriate matrices given on measurement
   - Assumes measurement received is either from radar or lidar
   **************************************************************************/
  const VectorXd z = data.get_raw_data(); // measurement from sensor
  const VectorXd x = _KF.get_resulting_state(); // predicted state

  // Hx is the sensor measurement if the predicted state is true
  // and the sensors are perfect (no noise)
  VectorXd Hx;  
  MatrixXd R; 
  MatrixXd H;

  if (data.get_type() == DataPointType::RADAR){

    VectorXd s = data.get_state();
    H = calculate_jacobian(s);
    Hx = convert_cartesian_to_polar(x);
    R = _radar_R;

  } else if (data.get_type() == DataPointType::LIDAR){

    H = _lidar_H;
    Hx = _lidar_H * x;
    R = _lidar_R;
  }

  _KF.update(z, H, Hx, R);
}

void FusionEKF::process(const DataPoint& data){
  _initialized ? this->compute(data) : this->start(data);
}

VectorXd FusionEKF::get_resulting_state() const{
  return _KF.get_resulting_state();
}
