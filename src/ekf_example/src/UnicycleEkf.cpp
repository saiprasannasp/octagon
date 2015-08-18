#include <ekf_example/UnicycleEkf.h>

using namespace Eigen;

namespace ekf_example
{

UnicycleEkf::UnicycleEkf(const std::vector<double>& qvals, double sample_time)
{
  // Initialize state and error covariance
  Xk.setZero(5, 1);
  Pk.setIdentity(5,5);

  // Initialize Q matrix
  Q.setZero(5,5);
  for (int i = 0; i < 5; i++) {
    Q(i, i) = qvals[i] * qvals[i];
  }

  ROS_INFO_STREAM(Xk);
  ROS_INFO_STREAM(Pk);
  ROS_INFO_STREAM(Q);

  // Set sample time
  Ts = sample_time;
}

void UnicycleEkf::stepFilter()
{
  // Update state Jacobian
  stateJacobian();

  /* Prediction step */
  // Xkkm1 = f(Xk)
  Matrix<double, 5, 1> Xkkm1;
  stateModel(Xkkm1);

  // Pkkm1 = A*Pkm1km1*A' + Q
  Matrix<double, 5, 5> Pkkm1;
  Pkkm1 = Ak * Pk * Ak.transpose() + Q;

  /* Update step */
  // Skip if no measurements are available
  if (measurements.rows() > 0) {
    // S = C*Pkkm1*C' + R
    MatrixXd S;
    S = Ck * Pkkm1 * Ck.transpose() + MatrixXd(rvals.asDiagonal());

    // K = (Pkkm1*C')*inv(S)
    MatrixXd K;
    K = Pkkm1 * Ck.transpose() * S.inverse();

    // Xkk = Xkkm1 + K*(z-h(Xkkm1))
    Xk = Xkkm1 + K * (measurements - h_function);

    // Pkk = (eye - K*C)*Pkkm1
    Pk = (MatrixXd::Identity(K.rows(), Ck.cols()) - K * Ck) * Pkkm1;

    // Reset measurements
    measurements.resize(0,NoChange);
    rvals.resize(0,NoChange);
    h_function.resize(0,NoChange);
    Ck.resize(0,0);
  } else {
    // Update state and covariance with predicted values
    Xk = Xkkm1;
    Pk = Pkkm1;
  }

}

#define XX    Xk(0)
#define YY    Xk(1)
#define PSI   Xk(2)
#define VV    Xk(3)
#define PDOT  Xk(4)

#define CPSI  cos((double)PSI)
#define SPSI  sin((double)PSI)

void UnicycleEkf::addGpsMeas(double x, double y, double r)
{
  // Add GPS x measurement
  pushVector(x, measurements); // Actual measurement
  pushVector(r * r, rvals); // Measurement covariance

  // Add measurement function output
  double x_meas = XX;
  pushVector(x_meas, h_function);

  // Add measurement Jacobian row
  MatrixXd x_row(1, 5);
  x_row << 1, 0, 0, 0, 0;
  pushRow(x_row, Ck);

  // Add GPS y measurement
  pushVector(y, measurements); // Actual measurement
  pushVector(r * r, rvals); // Measurement covariance

  // Add measurement function output
  double y_meas = YY;
  pushVector(y_meas, h_function);

  // Add measurement Jacobian row
  MatrixXd y_row(1, 5);
  y_row << 0, 1, 0, 0, 0;
  pushRow(y_row, Ck);
}

void UnicycleEkf::addVelMeas(double v, double r)
{
  pushVector(v, measurements); // Actual measurement
  pushVector(r * r, rvals); // Measurement covariance

  // Add measurement function output
  double vel_meas = VV;
  pushVector(vel_meas, h_function);

  // Add measurement Jacobian row
  MatrixXd v_row(1, 5);
  v_row << 0, 0, 0, 1, 0;
  pushRow(v_row, Ck);
}

void UnicycleEkf::addYawRateMeas(double yaw_rate, double r)
{
  pushVector(yaw_rate, measurements); // Actual measurement
  pushVector(r * r, rvals); // Measurement covariance

  // Add measurement function output
  double yaw_rate_meas = PDOT;
  pushVector(yaw_rate_meas, h_function);

  // Add measurement Jacobian row
  MatrixXd yaw_row(1, 5);
  yaw_row << 0, 0, 0, 0, 1;
  pushRow(yaw_row, Ck);
}

void UnicycleEkf::stateModel(Eigen::Matrix<double, 5, 1>& f)
{
  // Discrete, non-linear state equation
  f(0) = XX + Ts * VV * CPSI;
  f(1) = YY + Ts * VV * SPSI;
  f(2) = PSI + Ts * PDOT;
  f(3) = VV;
  f(4) = PDOT;
}

void UnicycleEkf::stateJacobian()
{
  // Jacobian of continuous function
  Matrix<double, 5, 5> dfdx;
  dfdx.row(0) << 0, 0, -VV * SPSI, CPSI, 0;
  dfdx.row(1) << 0, 0, VV * CPSI, SPSI, 0;
  dfdx.row(2) << 0, 0, 0, 0, 1;
  dfdx.row(3) << 0, 0, 0, 0, 0;
  dfdx.row(4) << 0, 0, 0, 0, 0;

  // Discretize
  Ak = MatrixXd::Identity(5, 5) + Ts * dfdx;
}

void UnicycleEkf::pushRow(const Eigen::MatrixXd& new_row, Eigen::MatrixXd& mat)
{
  // Add a row by increasing row size by 1 and inserting the new row
  mat.conservativeResize(mat.rows() + 1, new_row.cols());
  mat.row(mat.rows() - 1) = new_row;
}

void UnicycleEkf::pushVector(double val, Eigen::VectorXd& vect)
{
  // Add a vector element to the end of an Eigen::VectorXd
  vect.conservativeResize(vect.rows() + 1);
  vect(vect.rows() - 1) = val;
}

const UnicycleState& UnicycleEkf::getState()
{
  // Populate a UnicycleState structure and return it
  state.x = XX;
  state.y = YY;
  state.psi = PSI;
  state.v = VV;
  state.pdot = PDOT;
  return state;
}

UnicycleEkf::~UnicycleEkf()
{
}

}
