#ifndef LEAST_SQUARES_TOOLS_H
#define LEAST_SQUARES_TOOLS_H

#include <eigen3/Eigen/Dense>
#include <math.h>

typedef Eigen::Matrix3d Matrix3X3;
typedef Eigen::RowVector3d Vector1X3;

namespace LeastSquares
{
  /**
   * @brief - converts pose to transformation matrix
   * 
   * @param[out] pose - 
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
    return matrix;
  }

  /**
   * @brief 
   * 
   * @param transform 
   * @return Vector1X3 
   */
  Vector1X3 transformToPose(const Matrix3X3 &transform)
  {
    Vector1X3 vector = Vector1X3::Zero();
    vector(0) = transform(0, 2);
    vector(1) = transform(1, 2);
    vector(3) = atan2(transform(1,0), transform(0,0));
    return vector;
  }
} //namespace LeastSquares

#endif