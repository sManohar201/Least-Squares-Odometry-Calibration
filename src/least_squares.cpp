#include "least_squares.h"

namespace LeastSquares
{
void least_squares_calibrate_odometry(const Eigen::MatrixXd &grond_truth, const Eigen::MatrixXd &measurements,
                                      Matrix3X3 &calib_parameters)
{
  calib_parameters = Matrix3X3::Identity(3, 3);
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(9, 9);
  Eigen::RowVectorXd b = Eigen::RowVectorXd::Zero(9);
  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(3, 9);
  Eigen::VectorXd error;
  for (size_t i = 0; i < grond_truth.rows(); i++)
  {
    Vector1X3 truth = grond_truth.row(i);
    Vector1X3 measure = measurements.row(i);
    calculate_error(truth, measure, calib_parameters, error);
    calculate_jacobian(measure, J);
    H = H + J.transpose() * J;
    b = b + error.transpose() * J;
  }
  Eigen::VectorXd delta = -H.inverse() * b.transpose();
  Eigen::Map<Eigen::MatrixXd> dX(delta.data(), 3, 3);
  calib_parameters = calib_parameters + dX.transpose();
}

void calculate_error(const Vector1X3 &truthX, const Vector1X3 &measureX, Matrix3X3 &calib_parameters,
                     Eigen::VectorXd &error)
{
  error = truthX.transpose() - calib_parameters * measureX.transpose();
}

void calculate_jacobian(const Vector1X3 &measureX, Eigen::MatrixXd &jacobian)
{
  jacobian(0, 0) = -measureX(0);
  jacobian(0, 1) = -measureX(1);
  jacobian(0, 2) = -measureX(2);
  jacobian(1, 3) = -measureX(0);
  jacobian(1, 4) = -measureX(1);
  jacobian(1, 5) = -measureX(2);
  jacobian(2, 6) = -measureX(0);
  jacobian(2, 7) = -measureX(1);
  jacobian(2, 8) = -measureX(2);
}
}  // namespace LeastSquares