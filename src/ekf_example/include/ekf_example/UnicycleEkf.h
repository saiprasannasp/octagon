#ifndef UNICYCLEEKF_H_
#define UNICYCLEEKF_H_

#include <ros/ros.h>
#include <vector>
#include <eigen3/Eigen/Dense>

namespace ekf_example
{

typedef struct
{
  double x;
  double y;
  double psi;
  double v;
  double pdot;
} UnicycleState;

class UnicycleEkf
{
public:
  UnicycleEkf(const std::vector<double>& qvals, double sample_time);
  ~UnicycleEkf();

  void stepFilter();

  void addGpsMeas(double x, double y, double r);
  void addVelMeas(double v, double r);
  void addYawRateMeas(double yaw_rate, double r);

  const UnicycleState& getState();

private:

  void stateModel(Eigen::Matrix<double, 5, 1>& f);
  void stateJacobian();

  void pushVector(double val, Eigen::VectorXd& vect);
  void pushRow(const Eigen::MatrixXd& new_row, Eigen::MatrixXd& mat);

  Eigen::Matrix<double, 5, 1> Xk; // Current state estimate
  Eigen::Matrix<double, 5, 5> Pk; // Current estimate covariance matrix

  Eigen::Matrix<double, 5, 5> Ak; // Current state Jacobian
  Eigen::MatrixXd Ck; // Current measurement Jacobian

  Eigen::Matrix<double, 5, 5> Q; // State covariance matrix

  Eigen::VectorXd measurements; // Vector of current measurements
  Eigen::VectorXd h_function; // Vector of measurement function outputs
  Eigen::VectorXd rvals; // Vector of measurement covariances

  double Ts; // Sample time of the filter

  UnicycleState state;
};

}

#endif /* UNICYCLEEKF_H_ */
