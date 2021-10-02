#ifndef LEAST_SQUARES_H
#define LEAST_SQUARES_H

#include <iostream>
#include <eigen3/Eigen/Dense>

namespace LeastSquares
{

  typedef Eigen::Matrix3d Matrix3X3;
  typedef Eigen::RowVector3d Vector1X3;

  /**
   * @brief - computes the calibration parameters through least squares method.
   * 
   * @param[in] ground_truth - ground truth data.  
   * @param[in] measurements - scanmatched measurements
   * @param[out] calib_parameters - output calibration parameters.
   */
  void least_squares_calibrate_odometry(const Eigen::MatrixXd &ground_truth, 
                                        const Eigen::MatrixXd &measurements, 
                                        Matrix3X3 &calib_parameters);

  /**
   * @brief - computes the error of the ith measurement 
   * 
   * @param[in] truthX - ground truth data.
   * @param[in] measureX - measurement data.
   * @param[in] calib_parameters - calibration matrix
   * @param[out] error - error between the ground truth data and measurement data.
   */
  void calculate_error(const Vector1X3 &truthX, const Vector1X3 &measureX, 
                      Matrix3X3 &calib_parameters, Eigen::VectorXd &error);

  /**
   * @brief - derivative of the error function for the ith measurement.
   * 
   * @param[in] measureX - measurement data.
   * @param[out] jacobian - jacobian of the ith measurement.
   */
  void calculate_jacobian(const Vector1X3 &measureX, Eigen::MatrixXd &jacobian);


} // namespace LeastSquares

#endif // LEAST_SQUARES_H