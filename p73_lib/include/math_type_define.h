#ifndef MATH_TYPE_DEFINE_H
#define MATH_TYPE_DEFINE_H

#define DEG2RAD (0.01745329251994329576923690768489)

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/Geometry>
#include <unsupported/Eigen/MatrixFunctions>
#include <Eigen/SVD>
#include <iostream>
#include <chrono>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <vector>
#include <geometry_msgs/msg/pose.h>

// REDUCED MODEL
#define MODEL_DOF 13
#define MODEL_DOF_VIRTUAL 19
#define MODEL_DOF_QVIRTUAL 20

#define ENDEFFECTOR_NUMBER 2
#define LINK_NUMBER 14

#define GRAVITY 9.80665
#define MAX_DOF 50U
#define RAD2DEG 1 / DEG2RAD

#define INERITA_SIZE 198

namespace Eigen
{
// Eigen default type definition
#define EIGEN_MAKE_TYPEDEFS(Type, TypeSuffix, Size, SizeSuffix)    \
  typedef Matrix<Type, Size, Size> Matrix##SizeSuffix##TypeSuffix; \
  typedef Matrix<Type, Size, 1> Vector##SizeSuffix##TypeSuffix;    \
  typedef Matrix<Type, 1, Size> RowVector##SizeSuffix##TypeSuffix;

  typedef double rScalar;
  typedef float lScalar;

  typedef Matrix<lScalar, MODEL_DOF_QVIRTUAL, 1> VectorQVQf;
  typedef Matrix<lScalar, MODEL_DOF_VIRTUAL, 1> VectorVQf;
  typedef Matrix<lScalar, MODEL_DOF, 1> VectorQf;


  EIGEN_MAKE_TYPEDEFS(rScalar, d, 5, 5)
  EIGEN_MAKE_TYPEDEFS(rScalar, d, 6, 6)
  EIGEN_MAKE_TYPEDEFS(rScalar, d, 7, 7)
  EIGEN_MAKE_TYPEDEFS(rScalar, d, 8, 8)

  typedef Matrix<rScalar, 1, 3> Matrix1x3d;
  typedef Matrix<rScalar, 1, 4> Matrix1x4d;
  typedef Matrix<rScalar, 4, 3> Matrix4x3d;
  typedef Matrix<rScalar, 6, 3> Matrix6x3d;
  typedef Matrix<rScalar, 6, 7> Matrix6x7d;
  typedef Matrix<rScalar, 8, 4> Matrix8x4d;
  typedef Matrix<rScalar, -1, 1, 0, MAX_DOF, 1> VectorJXd;
  typedef Matrix<rScalar, -1, 1, 0, 12, 1> VectorLXd; // Leg IK
  typedef Matrix<rScalar, -1, -1, 0, MAX_DOF, MAX_DOF> MatrixJXd;

  typedef Matrix<rScalar, MODEL_DOF_VIRTUAL, MODEL_DOF_VIRTUAL> MatrixVVd;
  typedef Matrix<lScalar, MODEL_DOF_VIRTUAL, MODEL_DOF_VIRTUAL> MatrixVVf;
  typedef Matrix<lScalar, MODEL_DOF, MODEL_DOF> MatrixQQf;
  typedef Matrix<lScalar, MODEL_DOF, MODEL_DOF_VIRTUAL> MatrixQVf;
  typedef Matrix<lScalar, MODEL_DOF_VIRTUAL, MODEL_DOF> MatrixVQf;

  typedef Matrix<rScalar, 12, 1> Vector12d;

  typedef Matrix<rScalar, MODEL_DOF, 1> VectorQd;
  typedef Matrix<rScalar, MODEL_DOF_VIRTUAL, 1> VectorVQd;
  typedef Matrix<rScalar, MODEL_DOF_QVIRTUAL, 1> VectorQVQd;

  typedef Matrix<rScalar, 6, MODEL_DOF_VIRTUAL> Matrix6Vd;
  typedef Matrix<rScalar, 3, MODEL_DOF_VIRTUAL> Matrix3Vd;

  typedef Matrix<lScalar, 6, MODEL_DOF_VIRTUAL> Matrix6Vf;
  typedef Matrix<lScalar, 3, MODEL_DOF_VIRTUAL> Matrix3Vf;

  typedef Matrix<rScalar, 6, 12> Matrix6x12d;
  typedef Matrix<rScalar, 6, 8> Matrix6x8d;
  typedef Matrix<rScalar, 6, 3> Matrix6x3d;
  typedef Matrix<rScalar, 3, 12> Matrix3x12d;
  typedef Matrix<rScalar, 3, 8> Matrix3x8d;
  typedef Matrix<rScalar, 3, 3> Matrix3x3d;

  typedef Matrix<rScalar, 6, MODEL_DOF> Matrix6Qd;
  typedef Matrix<rScalar, MODEL_DOF, MODEL_DOF> MatrixQQd;
  typedef Matrix<rScalar, MODEL_DOF_VIRTUAL, MODEL_DOF_VIRTUAL> MatrixVQVQd;
  typedef Matrix<rScalar, 3, MODEL_DOF> Matrix3Qd;

  // Complex
  typedef Matrix<std::complex<double>, 8, 4> Matrix8x4cd;

} // namespace Eigen

namespace DyrosMath
{


  static Eigen::Matrix3d skm(Eigen::Vector3d x)
  {
    Eigen::Matrix3d Skew_temp1(3, 3);
    Skew_temp1.setZero();
    Skew_temp1(0, 1) = -x(2);
    Skew_temp1(0, 2) = x(1);
    Skew_temp1(1, 0) = x(2);
    Skew_temp1(1, 2) = -x(0);
    Skew_temp1(2, 0) = -x(1);
    Skew_temp1(2, 1) = x(0);
    return Skew_temp1;
  }

  static double cubic(double time,    ///< Current time
                      double time_0,  ///< Start time
                      double time_f,  ///< End time
                      double x_0,     ///< Start state
                      double x_f,     ///< End state
                      double x_dot_0, ///< Start state dot
                      double x_dot_f  ///< End state dot
  )
  {
    double x_t;

    if (time < time_0)
    {
      x_t = x_0;
    }
    else if (time > time_f)
    {
      x_t = x_f;
    }
    else
    {
      double elapsed_time = time - time_0;
      double total_time = time_f - time_0;
      double total_time2 = total_time * total_time;  // pow(t,2)
      double total_time3 = total_time2 * total_time; // pow(t,3)
      double total_x = x_f - x_0;

      x_t = x_0 + x_dot_0 * elapsed_time

            + (3 * total_x / total_time2 - 2 * x_dot_0 / total_time - x_dot_f / total_time) * elapsed_time * elapsed_time

            + (-2 * total_x / total_time3 +
               (x_dot_0 + x_dot_f) / total_time2) *
                  elapsed_time * elapsed_time * elapsed_time;
    }

    return x_t;
  }

  static double cubicDot(double time,    ///< Current time
                         double time_0,  ///< Start time
                         double time_f,  ///< End time
                         double x_0,     ///< Start state
                         double x_f,     ///< End state
                         double x_dot_0, ///< Start state dot
                         double x_dot_f, ///< End state dot
                         int hz = 0)
  {
    double x_t;

    if (time < time_0)
    {
      x_t = x_dot_0;
    }
    else if (time > time_f)
    {
      x_t = x_dot_f;
    }
    else
    {
      double elapsed_time = time - time_0;
      double total_time = time_f - time_0;
      double total_time2 = total_time * total_time;  // pow(t,2)
      double total_time3 = total_time2 * total_time; // pow(t,3)
      double total_x = x_f - x_0;

      x_t = x_dot_0

            + 2 * (3 * total_x / total_time2 - 2 * x_dot_0 / total_time - x_dot_f / total_time) * elapsed_time

            + 3 * (-2 * total_x / total_time3 + (x_dot_0 + x_dot_f) / total_time2) * elapsed_time * elapsed_time;
    }

    return x_t;
  }
  static double cubicDdot(double time,    ///< Current time
                          double time_0,  ///< Start time
                          double time_f,  ///< End time
                          double x_0,     ///< Start state
                          double x_f,     ///< End state
                          double x_dot_0, ///< Start state dot
                          double x_dot_f  ///< End state dot
  )
  {
    double x_t;

    if (time < time_0)
    {
      x_t = x_dot_0;
    }
    else if (time > time_f)
    {
      x_t = x_dot_f;
    }
    else
    {
      double elapsed_time = time - time_0;
      double total_time = time_f - time_0;
      double total_time2 = total_time * total_time;  // pow(t,2)
      double total_time3 = total_time2 * total_time; // pow(t,3)
      double total_x = x_f - x_0;

      x_t = 2 * (3 * total_x / total_time2 - 2 * x_dot_0 / total_time - x_dot_f / total_time)

            + 6 * (-2 * total_x / total_time3 + (x_dot_0 + x_dot_f) / total_time2) * elapsed_time;
    }

    return x_t;
  }

  template <int N>
  static Eigen::Matrix<double, N, 1> cubicVector(double time,                         ///< Current time
                                                 double time_0,                       ///< Start time
                                                 double time_f,                       ///< End time
                                                 Eigen::Matrix<double, N, 1> x_0,     ///< Start state
                                                 Eigen::Matrix<double, N, 1> x_f,     ///< End state
                                                 Eigen::Matrix<double, N, 1> x_dot_0, ///< Start state dot
                                                 Eigen::Matrix<double, N, 1> x_dot_f  ///< End state dot
  )
  {

    Eigen::Matrix<double, N, 1> res;
    for (unsigned int i = 0; i < N; i++)
    {
      res(i) = cubic(time, time_0, time_f, x_0(i), x_f(i), x_dot_0(i), x_dot_f(i));
    }
    return res;
  }

  static Eigen::Vector3d getPhi(Eigen::Matrix3d current_rotation,
                                Eigen::Matrix3d desired_rotation)
  {
    Eigen::Vector3d phi;
    Eigen::Vector3d s[3], v[3], w[3];

    for (int i = 0; i < 3; i++)
    {
      v[i] = current_rotation.block<3, 1>(0, i);
      w[i] = desired_rotation.block<3, 1>(0, i);
      s[i] = v[i].cross(w[i]);
    }
    phi = s[0] + s[1] + s[2];
    phi = -0.5 * phi;

    return phi;
  }

  static Eigen::Isometry3d multiplyIsometry3d(Eigen::Isometry3d A,
                                              Eigen::Isometry3d B)
  {
    Eigen::Isometry3d AB;

    AB.linear() = A.linear() * B.linear();
    AB.translation() = A.linear() * B.translation() + A.translation();
    return AB;
  }

  static Eigen::Vector3d multiplyIsometry3dVector3d(Eigen::Isometry3d A,
                                                    Eigen::Vector3d B)
  {
    Eigen::Vector3d AB;
    AB = A.linear() * B + A.translation();
    return AB;
  }

  static Eigen::Isometry3d inverseIsometry3d(Eigen::Isometry3d A)
  {
    Eigen::Isometry3d A_inv;

    A_inv.linear() = A.linear().transpose();
    A_inv.translation() = -A.linear().transpose() * A.translation();
    return A_inv;
  }

  static Eigen::Matrix3d rotateWithZ(double yaw_angle)
  {
    Eigen::Matrix3d rotate_wth_z(3, 3);

    rotate_wth_z(0, 0) = cos(yaw_angle);
    rotate_wth_z(1, 0) = sin(yaw_angle);
    rotate_wth_z(2, 0) = 0.0;

    rotate_wth_z(0, 1) = -sin(yaw_angle);
    rotate_wth_z(1, 1) = cos(yaw_angle);
    rotate_wth_z(2, 1) = 0.0;

    rotate_wth_z(0, 2) = 0.0;
    rotate_wth_z(1, 2) = 0.0;
    rotate_wth_z(2, 2) = 1.0;

    return rotate_wth_z;
  }

  static Eigen::Matrix3d rotateWithY(double pitch_angle)
  {
    Eigen::Matrix3d rotate_wth_y(3, 3);

    rotate_wth_y(0, 0) = cos(pitch_angle);
    rotate_wth_y(1, 0) = 0.0;
    rotate_wth_y(2, 0) = -sin(pitch_angle);

    rotate_wth_y(0, 1) = 0.0;
    rotate_wth_y(1, 1) = 1.0;
    rotate_wth_y(2, 1) = 0.0;

    rotate_wth_y(0, 2) = sin(pitch_angle);
    rotate_wth_y(1, 2) = 0.0;
    rotate_wth_y(2, 2) = cos(pitch_angle);

    return rotate_wth_y;
  }

  static Eigen::Matrix3d rotateWithX(double roll_angle)
  {
    Eigen::Matrix3d rotate_wth_x(3, 3);

    rotate_wth_x(0, 0) = 1.0;
    rotate_wth_x(1, 0) = 0.0;
    rotate_wth_x(2, 0) = 0.0;

    rotate_wth_x(0, 1) = 0.0;
    rotate_wth_x(1, 1) = cos(roll_angle);
    rotate_wth_x(2, 1) = sin(roll_angle);

    rotate_wth_x(0, 2) = 0.0;
    rotate_wth_x(1, 2) = -sin(roll_angle);
    rotate_wth_x(2, 2) = cos(roll_angle);

    return rotate_wth_x;
  }

  static Eigen::Vector3d rot2Euler(Eigen::Matrix3d Rot)
  {
    double beta;
    Eigen::Vector3d angle;
    beta = -asin(Rot(2, 0));

    if (abs(beta) < 90 * DEG2RAD)
      beta = beta;
    else
      beta = 180 * DEG2RAD - beta;

    angle(0) = atan2(Rot(2, 1), Rot(2, 2) + 1E-37); // roll
    angle(2) = atan2(Rot(1, 0), Rot(0, 0) + 1E-37); // pitch
    angle(1) = beta;                                // yaw

    return angle;
  }

  static Eigen::Vector3d rot2Euler_tf(Eigen::Matrix3d Rot)
  {
    Eigen::Vector3d angle;
    tf2::Matrix3x3 m(Rot(0, 0), Rot(0, 1), Rot(0, 2), Rot(1, 0), Rot(1, 1), Rot(1, 2), Rot(2, 0), Rot(2, 1), Rot(2, 2));
    m.getRPY(angle(0), angle(1), angle(2));
    return angle;
  }

  static void rot2Euler_tf2(Eigen::Matrix3d Rot, double &roll, double &pitch, double &yaw)
  {
    tf2::Matrix3x3 m(Rot(0, 0), Rot(0, 1), Rot(0, 2), Rot(1, 0), Rot(1, 1), Rot(1, 2), Rot(2, 0), Rot(2, 1), Rot(2, 2));
    m.getRPY(roll, pitch, yaw);
  }
  static Eigen::Matrix3d Euler2rot(double roll, double pitch, double yaw)
  {
    Eigen::Matrix3d m_;

    m_ = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());

    return m_;
  }

  static Eigen::Matrix3d Euler2rot_tf(Eigen::Vector3d eulr)
  {
    // tf2::Quaternion q(eulr(2), eulr(1), eulr(0));
    // tf2::Matrix3x3 m(q);

    Eigen::Matrix3d Mat_temp;

    // tf2::
    return Mat_temp;
  }

  static Eigen::Vector4d rot2Axis(Eigen::Matrix3d Rot)
  {
    double theta;
    Eigen::Vector4d result;
    Eigen::Vector3d omega;

    theta = acos((Rot(0, 0) + Rot(1, 1) + Rot(2, 2) - 1) / 2);
    if (theta == 0)
    {
      omega.setZero();
    }
    else
    {
      omega(0) = (Rot(2, 1) - Rot(1, 2)) / (2 * sin(theta));
      omega(1) = (Rot(0, 2) - Rot(2, 0)) / (2 * sin(theta));
      omega(2) = (Rot(1, 0) - Rot(0, 1)) / (2 * sin(theta));
    }
    result.segment<3>(0) = omega;
    result(3) = theta;

    return result;
  }

  static Eigen::Vector3d quatRotateInverse(const Eigen::Quaterniond& q, const Eigen::Vector3d& v) {

    Eigen::Vector3d q_vec = q.vec();
    double q_w = q.w();

    Eigen::Vector3d a = v * (2.0 * q_w * q_w - 1.0);
    Eigen::Vector3d b = 2.0 * q_w * q_vec.cross(v);
    Eigen::Vector3d c = 2.0 * q_vec * q_vec.dot(v);

    return a - b + c;
  }

  static Eigen::Matrix3d Add_vel_to_Rotm(Eigen::Matrix3d Rotm, Eigen::Vector3d Rot_Vel, double d_time_)
  {
    Eigen::Quaterniond qtemp(Rotm);
    Eigen::Quaterniond res;
    Eigen::Quaterniond angvel_(1, Rot_Vel(0) * d_time_ * 0.5, Rot_Vel(1) * d_time_ * 0.5, Rot_Vel(2) * d_time_ * 0.5);
    res = angvel_ * qtemp;
    Rotm = res.normalized().toRotationMatrix();
    return Rotm;
  }
 
  static Eigen::MatrixXd pinv_SVD(const Eigen::MatrixXd &A, double epsilon = std::numeric_limits<double>::epsilon())
  {
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);

    double tolerance = epsilon * std::max(A.cols(), A.rows()) * svd.singularValues().array().abs()(0);
    return svd.matrixV() * (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
  }

  template <int N, int M>
  static Eigen::Matrix<double, M, N> pinv_QRs(Eigen::Matrix<double, N, M> &A)
  {
    Eigen::ColPivHouseholderQR<Eigen::Matrix<double, N, M>> qr(A);
    qr.setThreshold(10e-6);

    int rank = qr.rank();
    int cols, rows;
    cols = A.cols();
    rows = A.rows();
    // std::cout << "Rank : " << rank << std::endl;
    if (rank == 0)
    {
      std::cout << "WARN: pinvQRs rank 0" << std::endl;
      return Eigen::MatrixXd::Identity(rows, cols);
    }
    else
    {
      if (cols > rows)
      {
        Eigen::Matrix<double, M, N> Rpsinv2;
        Rpsinv2.setZero(rows, cols);
        Rpsinv2.topLeftCorner(rank, rank) = qr.matrixQR().topLeftCorner(rank, rank).template triangularView<Eigen::Upper>().solve(Eigen::MatrixXd::Identity(rank, rank));
        return (qr.colsPermutation() * Rpsinv2 * qr.householderQ().transpose()).transpose();
      }
      else
      {
        Eigen::Matrix<double, N, M> Rpsinv2;
        Rpsinv2.setZero(cols, rows);
        Rpsinv2.topLeftCorner(rank, rank) = qr.matrixQR().topLeftCorner(rank, rank).template triangularView<Eigen::Upper>().solve(Eigen::MatrixXd::Identity(rank, rank));
        return qr.colsPermutation() * Rpsinv2 * qr.householderQ().transpose();
      }
    }
  }

  static Eigen::MatrixXd pinv_COD(const Eigen::MatrixXd &A)
  {
    Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd> cod(A.rows(), A.cols());

    cod.setThreshold(1.0e-6);

    cod.compute(A);

    return cod.pseudoInverse();
  }

  static Eigen::MatrixXd pinv_QR(const Eigen::MatrixXd &A) // faster than pinv_SVD,
  {
    Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(A);
    qr.setThreshold(1.0e-6);
    int rank = qr.rank();

    int cols, rows;

    cols = A.cols();
    rows = A.rows();

    if (rank == 0)
    {
      std::cout << "WARN::Input Matrix seems to be zero matrix" << std::endl;
      return A;
    }
    else
    {
      if (cols > rows)
      {
        Eigen::MatrixXd Rpsinv2(rows, cols);
        Rpsinv2.setZero();
        Rpsinv2.topLeftCorner(rank, rank) = qr.matrixQR().topLeftCorner(rank, rank).template triangularView<Eigen::Upper>().solve(Eigen::MatrixXd::Identity(rank, rank));
        return (qr.colsPermutation() * Rpsinv2 * qr.householderQ().transpose()).transpose();
      }
      else
      {
        Eigen::MatrixXd Rpsinv2(cols, rows);
        Rpsinv2.setZero();
        Rpsinv2.topLeftCorner(rank, rank) = qr.matrixQR().topLeftCorner(rank, rank).template triangularView<Eigen::Upper>().solve(Eigen::MatrixXd::Identity(rank, rank));
        return qr.colsPermutation() * Rpsinv2 * qr.householderQ().transpose();
      }
    }
  }

  static Eigen::MatrixXd pinv_QR_prev(const Eigen::MatrixXd &A) // faster than pinv_SVD,
  {
    Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(A);
    qr.setThreshold(10e-6);
    int rank = qr.rank();

    int cols, rows;

    cols = A.cols();
    rows = A.rows();

    if (rank == 0)
    {
      std::cout << "WARN::Input Matrix seems to be zero matrix" << std::endl;
      return A;
    }
    else
    {
      if (cols > rows)
      {
        Eigen::MatrixXd Rpsinv2(rows, cols);
        Eigen::MatrixXd R(rank, rank);
        R = qr.matrixQR().topLeftCorner(rank, rank).template triangularView<Eigen::Upper>();
        Rpsinv2.setZero();
        Rpsinv2.topLeftCorner(rank, rank) = R.inverse();
        return (qr.colsPermutation() * Rpsinv2 * qr.householderQ().transpose()).transpose();
      }
      else
      {
        Eigen::MatrixXd Rpsinv2(cols, rows);
        Eigen::MatrixXd R(rank, rank);
        R = qr.matrixQR().topLeftCorner(rank, rank).template triangularView<Eigen::Upper>();
        Rpsinv2.setZero();
        Rpsinv2.topLeftCorner(rank, rank) = R.inverse();
        return qr.colsPermutation() * Rpsinv2 * qr.householderQ().transpose();
      }
    }
  }

  static Eigen::Matrix4d pose2Tmat(const geometry_msgs::msg::Pose &p_)
  {
    Eigen::Quaterniond q_(p_.orientation.w, p_.orientation.x, p_.orientation.y, p_.orientation.z);

    Eigen::Matrix4d ret;
    ret.setIdentity();

    ret.block(0, 0, 3, 3) = q_.normalized().toRotationMatrix();

    ret(0, 3) = p_.position.x;
    ret(1, 3) = p_.position.y;
    ret(2, 3) = p_.position.z;

    return ret;
  }

  static Eigen::MatrixXd dc_inv_QR(const Eigen::MatrixXd &A, const Eigen::MatrixXd &W)
  {
    Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(A);
    qr.setThreshold(10e-8);

    int rank = qr.rank();
    int cols, rows;
    cols = A.cols();
    rows = A.rows();

    Eigen::MatrixXd V2;

    V2 = qr.colsPermutation();

    int rank_def = cols - rank;
    V2.rightCols(rank_def);

    Eigen::MatrixXd pinv;

    if (rank == 0)
    {
      std::cout << "dynamically consistent inverse error" << std::endl;
    }
    else
    {
      Eigen::MatrixXd R = qr.matrixQR().topLeftCorner(rank, rank).template triangularView<Eigen::Upper>();
      Eigen::MatrixXd Rpsinv2(rows, cols);

      Rpsinv2.setZero();
      Rpsinv2.topLeftCorner(rank, rank) = R.inverse();
      pinv = (qr.colsPermutation() * Rpsinv2 * qr.householderQ().transpose()).transpose();
    }

    Eigen::MatrixXd Ix;

    Ix.setIdentity(rows, rows);

    return (Ix - V2 * pinv_QR(V2.transpose() * W * V2) * V2.transpose() * W) * pinv;
  }
  static std::pair<Eigen::MatrixXd, Eigen::MatrixXd> pinv_QR_pair(Eigen::MatrixXd &A)
  {
    Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(A);
    // qr.setThreshold(10e-10);
    int rank = qr.rank();

    int cols, rows;

    cols = A.cols();
    rows = A.rows();

    if (rank == 0)
    {
      std::cout << "WARN::Input Matrix seems to be zero matrix" << std::endl;
      std::pair<Eigen::MatrixXd, Eigen::MatrixXd> ret(A, A);
      return ret;
    }
    else
    {
      if (cols > rows)
      {
        Eigen::MatrixXd Rpsinv2(rows, cols);

        Rpsinv2.setZero();
        Rpsinv2.topLeftCorner(rank, rank) = qr.matrixQR().topLeftCorner(rank, rank).template triangularView<Eigen::Upper>().solve(Eigen::MatrixXd::Identity(rank, rank));

        Eigen::MatrixXd P;
        P = qr.householderQ().transpose();

        std::pair<Eigen::MatrixXd, Eigen::MatrixXd> ret((qr.colsPermutation() * Rpsinv2 * qr.householderQ().transpose()).transpose(), P.block(rank, 0, P.rows() - rank, P.cols()));
        return ret;
      }
      else
      {
        Eigen::MatrixXd Rpsinv2(cols, rows);
        Rpsinv2.setZero();
        Rpsinv2.topLeftCorner(rank, rank) = qr.matrixQR().topLeftCorner(rank, rank).template triangularView<Eigen::Upper>().solve(Eigen::MatrixXd::Identity(rank, rank));

        Eigen::MatrixXd P;
        P = qr.householderQ().transpose();

        std::pair<Eigen::MatrixXd, Eigen::MatrixXd> ret((qr.colsPermutation() * Rpsinv2 * qr.householderQ().transpose()), P.block(rank, 0, P.rows() - rank, P.cols()));
        return ret;
      }
    }
  }

  static Eigen::MatrixQQd WinvCalc(const Eigen::MatrixQQd &W, Eigen::MatrixXd &V2)
  {
    Eigen::ColPivHouseholderQR<Eigen::MatrixQQd> qr(W);
    qr.setThreshold(1.0e-6);
    int cols = W.cols();
    int rows = W.rows();

    int rank = qr.rank();
    if ((rank == MODEL_DOF - 18) || (rank == MODEL_DOF - 12) || (rank == MODEL_DOF - 6) || (rank == MODEL_DOF))
    {
      Eigen::MatrixQQd Rpsinv;
      Rpsinv.setZero();
      Rpsinv.topLeftCorner(rank, rank) = qr.matrixQR().topLeftCorner(rank, rank).template triangularView<Eigen::Upper>().solve(Eigen::MatrixXd::Identity(rank, rank));
      Eigen::MatrixQQd P;
      P = qr.householderQ().transpose();
      V2 = P.block(rank, 0, P.rows() - rank, P.cols());

      return qr.colsPermutation() * Rpsinv * P;
    }
    else
    {
      std::cout << "Winv Calc Error : rank = " << rank << std::endl;
      return W;
    }
  }

  static Eigen::MatrixXd pinv_COD(const Eigen::MatrixXd &A, Eigen::MatrixXd &V2)
  {
    Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd> cod(A.rows(), A.cols());

    cod.setThreshold(1.0e-7);

    cod.compute(A);

    int rank = cod.rank();
    
    if ((rank == MODEL_DOF - 18) || (rank == MODEL_DOF - 12) || (rank == MODEL_DOF - 6) || (rank == MODEL_DOF))
    {
    }
    else
    {
      std::cout << "pinv_COD Calc Error : rank = " << rank << std::endl;
      // return A.transpose()*(A*A.transpose() + Eigen::MatrixQQd::Identity()*1.0e-6 ).inverse();
    }
    Eigen::MatrixXd vtemp = cod.householderQ().transpose();
    V2 = vtemp.block(rank, 0, A.rows() - rank, A.cols());
    Eigen::MatrixXd output_temp = cod.pseudoInverse();
    return output_temp;
  }

  static Eigen::MatrixXd pinv_QR(const Eigen::MatrixXd &A, Eigen::MatrixXd &V2) // faster than pinv_SVD,
  {
    // FullPivHouseholderQR<MatrixXd> qr(A);
    // qr.compute(A);
    // qr.setThreshold(10e-10);
    // return qr.inverse();

    Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(A);

    qr.setThreshold(1.0e-9);

    int rank = qr.rank();

    int cols, rows;

    cols = A.cols();
    rows = A.rows();

    if (rank == 0)
    {
      std::cout << "WARN::Input Matrix seems to be zero matrix" << std::endl;
      return A;
    }
    else
    {
      if (cols > rows)
      {
        Eigen::MatrixXd Rpsinv2(rows, cols);
        Rpsinv2.setZero();
        Rpsinv2.topLeftCorner(rank, rank) = qr.matrixQR().topLeftCorner(rank, rank).template triangularView<Eigen::Upper>().solve(Eigen::MatrixXd::Identity(rank, rank));

        std::cout << "ERrrroooorrrrrr in pinv QR pair" << std::endl;
        return (qr.colsPermutation() * Rpsinv2 * qr.householderQ().transpose()).transpose();
      }
      else
      {
        Eigen::MatrixXd Rpsinv2(cols, rows);
        Rpsinv2.setZero();
        Rpsinv2.topLeftCorner(rank, rank) = qr.matrixQR().topLeftCorner(rank, rank).template triangularView<Eigen::Upper>().solve(Eigen::MatrixXd::Identity(rank, rank));

        Eigen::MatrixXd P;
        P = qr.householderQ().transpose();

        V2 = P.block(rank, 0, P.rows() - rank, P.cols());


        return qr.colsPermutation() * Rpsinv2 * P;
      }
    }
  }

  static void floatGyroframe(Eigen::Isometry3d trunk, Eigen::Isometry3d reference, Eigen::Isometry3d new_trunk)
  {
    Eigen::Vector3d rpy_ang;
    rpy_ang = DyrosMath::rot2Euler(reference.linear());

    Eigen::Matrix3d temp;
    temp = DyrosMath::rotateWithZ(-rpy_ang(2));

    new_trunk.linear() = temp * trunk.linear();
    new_trunk.translation() = temp * (trunk.translation() - reference.translation());
  }


  const static Eigen::Matrix3d rotationCubic(double time,
                                             double time_0,
                                             double time_f,
                                             const Eigen::Matrix3d &rotation_0,
                                             const Eigen::Matrix3d &rotation_f)
  {
    if (time >= time_f)
    {
      return rotation_f;
    }
    else if (time < time_0)
    {
      return rotation_0;
    }
    double tau = cubic(time, time_0, time_f, 0, 1, 0, 0);
    Eigen::AngleAxisd ang_diff(rotation_f * rotation_0.transpose());
    Eigen::Matrix3d diff_m;
    diff_m = Eigen::AngleAxisd(ang_diff.angle() * tau, ang_diff.axis());
    Eigen::Matrix3d result = diff_m * rotation_0;

    return result;
  }

  static Eigen::Vector3d QuinticSpline(
      double time,     ///< Current time
      double time_0,   ///< Start time
      double time_f,   ///< End time
      double x_0,      ///< Start state
      double x_dot_0,  ///< Start state dot
      double x_ddot_0, ///< Start state ddot
      double x_f,      ///< End state
      double x_dot_f,  ///< End state
      double x_ddot_f) ///< End state ddot
  {
    double a1, a2, a3, a4, a5, a6;
    double time_s;

    Eigen::Vector3d result;

    if (time < time_0)
    {
      result << x_0, x_dot_0, x_ddot_0;
      return result;
    }
    else if (time > time_f)
    {
      result << x_f, x_dot_f, x_ddot_f;
      return result;
    }

    time_s = time_f - time_0;
    a1 = x_0;
    a2 = x_dot_0;
    a3 = x_ddot_0 / 2.0;

    Eigen::Matrix3d Temp;
    Temp << pow(time_s, 3), pow(time_s, 4), pow(time_s, 5),
        3.0 * pow(time_s, 2), 4.0 * pow(time_s, 3), 5.0 * pow(time_s, 4),
        6.0 * time_s, 12.0 * pow(time_s, 2), 20.0 * pow(time_s, 3);

    Eigen::Vector3d R_temp;
    R_temp << x_f - x_0 - x_dot_0 * time_s - x_ddot_0 * pow(time_s, 2) / 2.0,
        x_dot_f - x_dot_0 - x_ddot_0 * time_s,
        x_ddot_f - x_ddot_0;

    Eigen::Vector3d RES;

    RES = Temp.inverse() * R_temp;

    a4 = RES(0);
    a5 = RES(1);
    a6 = RES(2);

    double time_fs = time - time_0;

    double position = a1 + a2 * pow(time_fs, 1) + a3 * pow(time_fs, 2) + a4 * pow(time_fs, 3) + a5 * pow(time_fs, 4) + a6 * pow(time_fs, 5);
    double velocity = a2 + 2.0 * a3 * pow(time_fs, 1) + 3.0 * a4 * pow(time_fs, 2) + 4.0 * a5 * pow(time_fs, 3) + 5.0 * a6 * pow(time_fs, 4);
    double acceleration = 2.0 * a3 + 6.0 * a4 * pow(time_fs, 1) + 12.0 * a5 * pow(time_fs, 2) + 20.0 * a6 * pow(time_fs, 3);

    result << position, velocity, acceleration;

    return result;
  }

  static double minmax_cut(double val, double min_, double max_)
  {
    if (val < min_)
      return min_;
    else if (val > max_)
      return max_;
    else
      return val;
  }

  static int minmax_cut(int val, int min_, int max_)
  {
    if (val < min_)
      return min_;
    else if (val > max_)
      return max_;
    else
      return val;
  }

  static double check_border(double x, double y, double x0, double x1, double y0, double y1, double sign)
  {
    return -sign * ((y1 - y0) * (x - x0) + (x1 - x0) * (y0 - y));
  }

  static inline double lpf(double input, double prev_res, double samping_freq, double cutoff_freq)
  {
    double rc = 1.0 / (cutoff_freq * 2 * M_PI);
    double dt = 1.0 / samping_freq;
    double a = dt / (rc + dt);

    return (prev_res + a * (input - prev_res));
  }

  static inline double lowPassFilter(double input, double prev, double ts, double tau)
  {
    return (tau * prev + ts * input) / (tau + ts);
  }

  static double secondOrderLowPassFilter(
      double x_k,   ///< input x[k]
      double x_k_1, ///< x[k-1]
      double x_k_2, ///< x[k-2]
      double y_k_1, ///< y[k-1]
      double y_k_2, ///< y[k-2]
      double fc,    ///< cut off frequency
      double d,     ///< damping ratio
      double hz)    ///< sampling freqeuncy
  {
    double y_k;
    double omega = 2 * M_PI * fc / hz;
    double D = 4 + 4 * d * omega + omega * omega;

    y_k = (8 - 2 * omega * omega) / D * y_k_1 - (4 - 4 * d * omega + omega * omega) / D * y_k_2 + omega * omega / D * (x_k + 2 * x_k_1 + x_k_2);

    return y_k;
  }

  template <int N>
  static Eigen::Matrix<double, N, 1> secondOrderLowPassFilter(

      Eigen::Matrix<double, N, 1> x_k,   ///< input x[k]
      Eigen::Matrix<double, N, 1> x_k_1, ///< x[k-1]
      Eigen::Matrix<double, N, 1> x_k_2, ///< x[k-2]
      Eigen::Matrix<double, N, 1> y_k_1, ///< y[k-1]
      Eigen::Matrix<double, N, 1> y_k_2, ///< y[k-2]
      double fc,                         ///< cut off frequency
      double d,                          ///< damping ratio
      double hz)                         ///< sampling freqeuncy
  {
    Eigen::Matrix<double, N, 1> res;
    for (int i = 0; i < N; i++)
    {
      res(i) = secondOrderLowPassFilter(x_k(i), x_k_1(i), x_k_2(i), y_k_1(i), y_k_2(i), fc, d, hz);
    }
    return res;
  }

  static inline double hpf(double input, double prev_input, double prev_res, double samping_freq, double cutoff_freq)
  {
    double rc = 1.0 / (cutoff_freq * 2 * M_PI);
    double dt = 1.0 / samping_freq;
    double a = rc / (rc + dt);

    return a*(prev_res + input - prev_input);
  }

  static double getOrientation2d(Eigen::Vector2d p1, Eigen::Vector2d p2)
  {
    return atan2(p1(0) * p2(1) - p1(1) * p2(0), p1(0) * p2(0) + p1(1) * p2(1));
  }

  static bool checkIntersect(Eigen::Vector2d p1, Eigen::Vector2d p2, Eigen::Vector2d q1, Eigen::Vector2d q2)
  {
    if ((getOrientation2d(p2 - p1, q1 - p1) * getOrientation2d(p2 - p1, q2 - p1)) < 0 && (getOrientation2d(q2 - q1, p1 - q1) * getOrientation2d(q2 - q1, p2 - q1)) < 0)
    {
      return true;
    }
    else
    {
      return false;
    }
  }

  static int findMinAdr(std::vector<double> ar)
  {
    int idx = 0;
    for (int i = 0; i < ar.size(); i++)
    {
      if (ar[idx] > ar[i])
      {
        idx = i;
      }
    }
    return idx;
  }

  static int findMaxAdr(std::vector<double> ar)
  {
    int idx = 0;
    for (int i = 0; i < ar.size(); i++)
    {
      if (ar[idx] < ar[i])
      {
        idx = i;
      }
    }
    return idx;
  }

  static bool isInPolygon(Eigen::Vector2d point, Eigen::MatrixXd polygon)
  {
    if (polygon.rows() == 2)
    {
      int line_number = polygon.cols() - 1;

      if (polygon.block(0, 0, 2, 1) != polygon.block(0, line_number, 2, 1))
      {
        std::cout << "polygon is not closed" << std::endl;
        return false;
      }
      else
      {
        bool check = true;
        for (int i = 0; i < line_number; i++)
        {
          check = check && (0 < getOrientation2d(polygon.block(0, i + 1, 2, 1) - polygon.block(0, i, 2, 1), point - polygon.block(0, i, 2, 1)));
        }
        if (check)
        {
          return true;
        }
      }
    }
    return false;
  }

  static Eigen::Vector2d getIntersectPoint(Eigen::Vector2d p1, Eigen::Vector2d p2, Eigen::Vector2d q1, Eigen::Vector2d q2)
  {
    double a1, a2, b1, b2;
    Eigen::Vector2d v1;
    v1.setZero();
    if (checkIntersect(p1, p2, q1, q2))
    {
      a1 = (p2(1) - p1(1)) / (p2(0) - p1(0));
      b1 = p1(1) - a1 * p1(0);

      a2 = (q2(1) - q1(1)) / (q2(0) - q1(0));
      b2 = q1(1) - a2 * q1(0);

      if (abs(a1) > 1000)
      {
        // std::cout << "a1 over " << a1 << std::endl;
        v1(0) = p1(0);

        // std::cout << "a2 : " << a2 << "  b2 : " << b2 << std::endl;
        v1(1) = a2 * v1(0) + b2;
      }
      else if (abs(a2) > 1000)
      {
        // std::cout << "a2 over " << a2 << std::endl;
        v1(0) = q1(0);
        // std::cout << "a1 : " << a1 << "  b1 : " << b1 << std::endl;
        v1(1) = a1 * v1(0) + b1;
      }
      else
      {
        v1(0) = (b2 - b1) / (a1 - a2);
        v1(1) = a1 * v1(0) + b1;
      }
    }
    else
    {
      std::cout << "given points are not intersect !";
    }
    return v1;
  }

  template <int N>
  static Eigen::Matrix<double, N, 1> lowPassFilter(Eigen::Matrix<double, N, 1> input, Eigen::Matrix<double, N, 1> prev, double ts, double tau)
  {
    Eigen::Matrix<double, N, 1> res;
    for (int i = 0; i < N; i++)
    {
      res(i) = lowPassFilter(input(i), prev(i), ts, tau);
    }
    return res;
  }

  template <int N>
  static Eigen::Matrix<double, N, 1> lpf(Eigen::Matrix<double, N, 1> input, Eigen::Matrix<double, N, 1> prev, double samping_freq, double cutoff_freq)
  {
    Eigen::Matrix<double, N, 1> res;

    for (int i = 0; i < N; i++)
    {
      res(i) = lpf(input(i), prev(i), samping_freq, cutoff_freq);
    }
    return res;
  }

  template <int N>
  static Eigen::Matrix<double, N, 1> hpf(Eigen::Matrix<double, N, 1> input, Eigen::Matrix<double, N, 1> prev_input, Eigen::Matrix<double, N, 1> prev, double samping_freq, double cutoff_freq)
  {
    Eigen::Matrix<double, N, 1> res;

    for (int i = 0; i < N; i++)
    {
      res(i) = hpf(input(i), prev_input(i), prev(i), samping_freq, cutoff_freq);
    }
    return res;
  }
  static const Eigen::Matrix3d skew(const Eigen::Vector3d &src)
  {
    Eigen::Matrix3d skew;
    skew.setZero();
    skew(0, 1) = -src[2];
    skew(0, 2) = src[1];
    skew(1, 0) = src[2];
    skew(1, 2) = -src[0];
    skew(2, 0) = -src[1];
    skew(2, 1) = src[0];

    return skew;
  }

  static int sign(double value)
  {
    if (value > 0.0)
    {
      return 1;
    }
    else if (value < 0.0)
    {
      return -1;
    }
    else
    {
      return 0;
    }
  }
} // namespace DyrosMath
#endif
