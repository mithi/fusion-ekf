#include "tools.h"

using namespace std;

VectorXd convert_cartesian_to_polar(const VectorXd& v){

  const double THRESH = 0.0001;
  VectorXd polar_vec(3);

  const double px = v(0);
  const double py = v(1);
  const double vx = v(2);
  const double vy = v(3);

  const double rho = sqrt( px * px + py * py);
  const double phi = atan2(py, px); //accounts for atan2(0, 0)
  const double drho = (rho > THRESH) ? ( px * vx + py * vy ) / rho : 0.0;

  polar_vec << rho, phi, drho;
  return polar_vec;
}

VectorXd convert_polar_to_cartesian(const VectorXd& v){

  VectorXd cartesian_vec(4);

  const double rho = v(0);
  const double phi = v(1);
  const double drho = v(2);

  const double px = rho * cos(phi);
  const double py = rho * sin(phi);
  const double vx = drho * cos(phi);
  const double vy = drho * sin(phi);

  cartesian_vec << px, py, vx, vy;
  return cartesian_vec;
}

MatrixXd calculate_jacobian(const VectorXd &v){

  const double THRESH = 0.0001;
  MatrixXd H = MatrixXd::Zero(3, 4);

  const double px = v(0);
  const double py = v(1);
  const double vx = v(2);
  const double vy = v(3);

  const double d_squared = px * px + py * py;
  const double d = sqrt(d_squared);
  const double d_cubed = d_squared * d;

  if (d >= THRESH){

    const double r11 = px / d;
    const double r12 = py / d;
    const double r21 = -py / d_squared;
    const double r22 = px / d_squared;
    const double r31 = py * (vx * py - vy * px) / d_cubed;
    const double r32 = px * (vy * px - vx * py) / d_cubed;

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
