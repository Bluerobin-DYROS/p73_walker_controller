#ifndef ROBOT_EIGEN_DATA_H
#define ROBOT_EIGEN_DATA_H

#include "p73_lib/link.h"

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <vector>
#include <atomic>
#include <sys/types.h>

#include "br_driver.hpp"
#include "rclcpp/rclcpp.hpp"
#include "p73_msgs/msg/task_cmd.hpp"
#include "p73_lib/link.h"
#include "math_type_define.h"
#include <limits>

using namespace std;
using namespace Eigen;

struct RobotEigenData
{
    ~RobotEigenData() { cout << cred << "RobotEigenData terminate" << creset << endl; }
    std::atomic<float> control_time_;
    std::atomic<float> control_time_us_;

    pinocchio::Model model_;
    pinocchio::Data data_;
    pinocchio::Model model_clik_;
    pinocchio::Data data_clik_;
    LinkData link_[LINK_NUMBER + 1];
    LinkData link_local_[LINK_NUMBER + 1];
    EndEffector ee_[ENDEFFECTOR_NUMBER];

    /////////////////////////////////////
    // Dynamics Data
    /////////////////////////////////////
    MatrixVVd A_, A_inv_, C_;
    VectorVQd G_;

    Eigen::Vector6d centroidal_momentum_;
    Eigen::Vector3d centroidal_angular_momentum_;
    Eigen::Vector3d centroidal_linear_momentum_;

    // CONTACT DYNAMICS
    MatrixXd J_C, J_C_INV_T;

    MatrixVVd N_C; 
    MatrixXd I_C, P_C;

    MatrixXd NwJw;
    MatrixQQd W, W_inv;
    MatrixXd Lambda_c;
    MatrixXd qr_V2;

    Vector12d fc_redist_;

    VectorQd torque_grav, torque_limit, torque_contact;
    VectorQd torque_fric;    
    Vector3d grav_ref;

    MatrixXd J_task;
    VectorXd e_task;
    unsigned int task_dof = 0;

    double total_mass_ = 0;

    int contact_index = 0;
    int contact_part[4] = {-1, -1, -1, -1};
    int ee_idx[4] = {-1, -1, -1, -1};
    volatile bool firstCalc = false;

    // joint position and velocity
    VectorQd q_, q_dot_, q_torque_; 
    VectorQd q_dot_lpf_; 
    VectorQd q_motor_, q_dot_motor_, q_torque_motor_;

    VectorQVQd q_virtual_;
    VectorVQd q_dot_virtual_;

    // JOINT POSITION AND VELOCITY LIMITS
    Eigen::VectorQd q_min;
    Eigen::VectorQd q_max;
    Eigen::VectorQd q_dot_max;

    // imu data
    Vector3d imu_ang_vel, imu_lin_acc;
    double roll, pitch, yaw;
    double yaw_init;
    Matrix3d imu_euler_cov, imu_ang_vel_cov, imu_lin_acc_cov;

    std::vector<double> Kp_m, Kd_m;
    std::vector<double> Kp_j, Kd_j;
    std::vector<double> tau_coulomb, tau_viscous;
    
    VectorQd q_desired, q_dot_desired, torque_desired;
    VectorQd q_motor_desired, torque_motor_desired;
    MatrixQQd four_bar_Jaco_, four_bar_Jaco_inv_;
};

struct DataContainer
{
    DataContainer(rclcpp::Node::SharedPtr node) : node_(node) {}
    ~DataContainer() { cout << cred << "DC terminate" << creset << endl; }
    RobotEigenData rd_;
 
    std::atomic<int> control_command_count{0};
    std::atomic<bool> torque_control_running{false};
    std::atomic<bool> triggerThread1;
    double command_[MODEL_DOF]; // it can be torque command or position command

    bool simMode = false;
    bool useMjcVirtual = false; // using mujoco data for state estimation

    rclcpp::Node::SharedPtr node_;

    // Switches
    bool torqOnSwitch = false;
    bool torqOffSwitch = false;
    bool safetyResetSwitch = false;
    bool inityawSwitch = false;
    bool stateEstimateModeSwitch = false;
    bool pauseSwitch = false;
    bool positionHoldSwitch = false;
    bool avatarModeSwitch = false;
    bool calibrationModeSwitch = false;
    bool readyPoseModeSwitch = false;
    
    //Task Command
    p73_msgs::msg::TaskCmd task_cmd_;
    bool tc_mode = false;

    // Ik Task Mode
    bool ik_mode_start = false;
    bool ik_mode = false;
    double ik_time_start = 0.0;
    double ik_traj_t_ = 0.0;
    int ik_target_link = -1;
    Vector3d ik_target_pos;

    // Joint Position Control
    bool pc_mode = false;
    bool pc_grav = false;
    VectorQd pos_ctrl_q_des, pos_ctrl_q_init, pos_ctrl_q_vel_init;
    double pos_ctrl_t_, pos_ctrl_traj_t_;

    // State Estimation
    bool se_mode = false;

    // Elmo Torque Activate -> used for real robot
    bool torqOn = false;
    bool torqRisingSeq = false;
    bool torqDecreaseSeq = false;
};


#endif
