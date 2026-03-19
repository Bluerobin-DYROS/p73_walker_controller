#ifndef LINK_H
#define LINK_H

#include <pinocchio/fwd.hpp>

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "pinocchio/parsers/urdf.hpp"
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/spatial/inertia.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp> 
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/crba.hpp>      
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/contact-jacobian.hpp>

#include "p73_lib/p73.h"

using namespace std;
using namespace Eigen;

class LinkData
{

public:
  /*
   * @brief
   *  Core kinematic and dynamic data structure for a single robot link.
   *  Provides initialization, and kinematic update utilities.
   */
  void Initialize(pinocchio::Model& model_, pinocchio::FrameIndex id_);
  void UpdatePosition(pinocchio::Model &model_, pinocchio::Data &data_);
  void UpdateVW(pinocchio::Model &model_, pinocchio::Data &data_);
  void UpdateJacobian(pinocchio::Model &model_, pinocchio::Data &data_);

  /*
   * @brief
   *  Trajectory generation functions.
   *  Each overload defines polynomial boundary conditions for position, velocity, and acceleration.
   */
  // set realtime trajectory of link from quintic spline.
  void SetTrajectoryQuintic(double current_time, double start_time, double end_time);
  void SetTrajectoryQuintic(double current_time, double start_time, double end_time, Eigen::Vector3d pos_desired);
  void SetTrajectoryQuintic(double current_time, double start_time, double end_time, Eigen::Vector3d pos_init, Eigen::Vector3d pos_desired);

  // set realtime trajectory of rotation of link
  void SetTrajectoryRotation(double current_time, double start_time, double end_time, Eigen::Matrix3d rot_desired);
  void SetTrajectoryRotation(double current_time, double start_time, double end_time);

  // set link initial position and rotation. initial position for task control.
  void SetInitialWithPosition();

  /*
   * @brief
   *  Kinematic and dynamic properties of the link.
   */
  Eigen::Matrix6Vd jac;
  pinocchio::FrameIndex id;
  double mass;

  // local COM position of body
  Eigen::Vector3d com_position;

  // inertial matrix
  Eigen::Matrix3d inertia;

  // rotation matrix
  Eigen::Matrix3d rotm;

  // global position of body
  Eigen::Vector3d xpos;

  // cartesian velocity of body
  Eigen::Vector3d v;

  // rotational velocity of body
  Eigen::Vector3d w;

  double roll, pitch, yaw;
  double roll_traj, pitch_traj, yaw_traj;
  double roll_init, pitch_init, yaw_init;

  // Local sensor point
  Eigen::Vector3d contact_point;

  /*
   * @brief
   *  realtime traj of cartesian & orientation.
   *  x : cartesian coordinate traj (3x1)
   *  v : cartesian velocity (3x1)
   *  r : rotational matrix of current orientation (3x3)
   *  w : rotational speed of current orientation (3x1)
   */
  Eigen::Vector3d x_traj;
  Eigen::Vector3d v_traj;
  Eigen::Vector3d a_traj;

  Eigen::Matrix3d r_traj;
  Eigen::Vector3d w_traj;
  Eigen::Vector3d ra_traj;

  Eigen::Vector3d x_init;
  Eigen::Vector3d v_init;
  Eigen::Matrix3d rot_init;
  Eigen::Vector3d w_init;

  Eigen::Vector3d x_desired;
  Eigen::Matrix3d rot_desired;

  Eigen::MatrixXd j_temp;
};

class EndEffector : public LinkData
{
public:
  void InitializeEE(LinkData &lk_, double cs_x_length_, double cs_y_length_, double contact_force_minimum_, double friction_coeff_);
  
  void UpdateLinkData(LinkData &lk_);
  
  void SetContact(pinocchio::Model &model_, pinocchio::Data &data_, Eigen::VectorQVQd &q_virtual_);

  Eigen::MatrixXd GetFrictionConeConstrainMatrix();

  Eigen::Vector3d xpos_contact;
  Eigen::Matrix6Vd jac_contact;

  double friction_coeff;

  double cs_x_length;
  double cs_y_length;

  double contact_force_minimum;

  bool contact;
};

#endif