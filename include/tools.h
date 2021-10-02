#ifndef LEAST_SQUARES_TOOLS_H
#define LEAST_SQUARES_TOOLS_H

#include <eigen3/Eigen/Dense>

namespace LeastSquares
{

  typedef Eigen::Matrix3d Matrix3X3;
  typedef Eigen::RowVector3d Vector1X3;
  /**
   * @brief - converts pose to transformation matrix
   * 
   * @param[in] pose - pose vector [x, y, theta]
   * @return Matrix3X3 
   */
  Matrix3X3 poseToTransform(const Vector1X3 &pose)
  {
    Matrix3X3 matrix = Matrix3X3::Zero();
    double cos_theta = cos(pose(2));
    double sin_theta = sin(pose(2));
    matrix(0,0) = cos_theta;
    matrix(0,1) = -sin_theta;
    matrix(1,0) = sin_theta;
    matrix(1,1) = cos_theta;
    matrix(0,2) = pose(0);
    matrix(1,2) = pose(1);
    matrix(2,2) = 1;
    return matrix;
  }

  /**
   * @brief - converts transform matrix to pose vector
   * 
   * @param[in] transform - transform matrix [T]
   * @return Vector1X3 
   */
  Vector1X3 transformToPose(const Matrix3X3 &transform)
  {
    Vector1X3 vector = Vector1X3::Zero();
    vector(0) = transform(0, 2);
    vector(1) = transform(1, 2);
    vector(2) = atan2(transform(1,0), transform(0,0));
    return vector;
  }

  /**
   * @brief - compute the trajectory of the robot by chaining up the odom information.
   * 
   * @param[in] odom - odometry measurements either the ground truth or scaned measurements
   * @param[out] trajectory = final trajectory computed from the odom measurements 
   */
  void compute_trajectory(const Eigen::MatrixXd &odom, Eigen::MatrixXd &trajectory)
  {
    // final trajectory matrix to return
    trajectory.resize(odom.rows()+1, odom.cols());
    // this vector holds  the current pose and pose no 1.
    Vector1X3 currentPose = Vector1X3::Zero();
    trajectory.row(0) = currentPose;

    Matrix3X3 transform_chain = poseToTransform(currentPose);

    for (size_t i=0; i<odom.rows(); i++)
    {
      transform_chain = transform_chain * poseToTransform(odom.row(i));
      trajectory.row(i+1) = transformToPose(transform_chain);
    }
  }

  /**
   * @brief - returns the corrected motions.
   * 
   * @param[in] parameters - calibration parameter
   * @param[in] odom - odometry measurement to be calibrated
   * @param[out] corrected_odom - corrected odometry measurement.
   */
  void apply_odometry_correction(const Matrix3X3 &parameters, const Eigen::MatrixXd &odom, Eigen::MatrixXd &corrected_odom)
  {
    corrected_odom.resize(odom.rows(), odom.cols());
    for (size_t i=0; i<odom.rows(); i++)
    {
      corrected_odom.row(i) = (parameters*odom.row(i).transpose()).transpose();
    }
  }

} //namespace LeastSquares

#endif