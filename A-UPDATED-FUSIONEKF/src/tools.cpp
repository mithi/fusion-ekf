#include "tools.h"

using namespace std;

VectorXd convert_cartesian_to_polar(const VectorXd& v){

  const double THRESH = 0.0001;
  VectorXd polar_vector(3);

  const double px = v(0);
  const double py = v(1);
  const double vx = v(2);
  const double vy = v(3);

  const double rho = sqrt( px * px + py * py);

  const bool degenerate = (py == 0.0 && px == 0.0);
  const double phi = degenerate ? 0.0 : atan2(py, px);
  // Avoids degenerate case, see error handling in
  // http://en.cppreference.com/w/cpp/numeric/math/atan2

  const double drho = (rho > THRESH) ? ( px * vx + py * vy ) / rho : 0.0;
  // Above line avoids dividing by zero

  polar_vector << rho, phi, drho;
  return polar_vector;
}

VectorXd convert_polar_to_cartesian(const VectorXd& v){

  VectorXd cartesian_vector(4);

  // Assumes that the the (vx, vy) is
  // at the direction of drho which is
  // not necessarily the case for for radar measurements
  const double rho = v(0);
  const double phi = v(1);
  const double drho = v(2);

  const double px = rho * cos(phi);
  const double py = rho * sin(phi);
  const double vx = drho * cos(phi);
  const double vy = drho * sin(phi);

  cartesian_vector << px, py, vx, vy;
  return cartesian_vector;
}

MatrixXd calculate_jacobian(const VectorXd &s){

  MatrixXd H = MatrixXd::Zero(3, 4);
  // Assumes Jacobian matrix is 3 x 4

  // Assumes size of passed vector s is 4
  const double px = s(0);
  const double py = s(1);
  const double vx = s(2);
  const double vy = s(3);

  const double d_squared = px * px + py * py;
  const double d = sqrt(d_squared);
  const double d_cubed = d_squared * d;

  const double THRESH = 0.0001;

  if (d >= THRESH){
  // Avoid dividing by zero

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
