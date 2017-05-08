#include "tools.h"

using namespace std;

VectorXd convert_cartesian_to_polar(const VectorXd v){

  double THRESH = 0.0001;
  VectorXd polar_vec(3);
  polar_vec << 0.0, 0.0, 0.0;

  double x = v(0);
  double y = v(1);
  double vx = v(2);
  double vy = v(3);

  double rho = sqrt( x * x + y * y);
  double phi = atan2(y, x);
  double drho;

  if (rho > THRESH){
    drho = ( x * vx + y * vy ) / rho;
  }

  polar_vec << rho, phi, drho;
  return polar_vec;
}

VectorXd convert_polar_to_cartesian(const VectorXd v){

  VectorXd cartesian_vec(4);

  double rho = v(0);
  double phi = v(1);
  double drho = v(2);

  double x = rho * cos(phi);
  double y = rho * sin(phi);
  double vx = drho * cos(phi);
  double vy = drho * sin(phi);
  cartesian_vec << x, y, vx, vy;

  return cartesian_vec;
}

MatrixXd calculate_jacobian(const VectorXd &v){

  double THRESH = 0.0001;
  MatrixXd H = MatrixXd::Zero(3, 4);

  double x = v(0);
  double y = v(1);
  double vx = v(2);
  double vy = v(3);

  double d_squared = x * x + y * y;
  double d = sqrt(d_squared);
  double d_cubed = d_squared * d;

  if (d >= THRESH){

    double r11 = x / d;
    double r12 = y / d;
    double r21 = -y / d_squared;
    double r22 = x / d_squared;
    double r31 = y * (vx * y - vy * x) / d_cubed;
    double r32 = x * (vy * x - vx * y) / d_cubed;

    H << r11, r12, 0.0, 0.0,
         r21, r22, 0.0, 0.0,
         r31, r31, r11, r12;
  }

  return H;
}

VectorXd calculate_RMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truths){

  VectorXd rmse(4);
  rmse << 0.0, 0.0, 0.0, 0.0;

  for (int k = 0; k < estimations.size(); ++k){

    VectorXd diff = estimations[k] - ground_truths[k];
    diff = diff.array() * diff.array();
    rmse += diff;
  }

  rmse /= (double)estimations.size();
  rmse = rmse.array().sqrt();

  return rmse;
}
