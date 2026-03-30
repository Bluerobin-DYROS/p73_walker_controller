#ifndef STATE_ESTIMATOR_H
#define STATE_ESTIMATOR_H

#include "p73_lib/robot_data.h"
#include "p73_lib/4bar_jac_func.h"
#include "br_driver.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int8_multi_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

using namespace std;
using namespace Eigen;

class StateEstimator
{
public:
    StateEstimator(DataContainer &dc);
    ~StateEstimator();

    void *StateEstimatorThread();
    static void *ThreadStarter(void *context) { return ((StateEstimator *)context)->StateEstimatorThread(); }

    void GetRobotData();
    void InitYaw();
    void StateEstimate();
    void UpdateKinematics();
    void UpdateKinematicsLocal();
    void UpdateDynamics();
    void StoreState(RobotEigenData &rd_global_);
    void SendCommand();

    FourBarKinematics four_bar_kinematics_;
    
    bool is_4bar_model = false;

    // datas for controller
    DataContainer &dc_;
    RobotEigenData &rd_global_;
    RobotEigenData rd_;

    // data from the robot (mjc or real robot)
    Robot robot;
    RobotData robot_data;

    timespec tv_us1;
    int rcv_cnt = -1;
    float control_time_;

    // joint position and velocity
    VectorQd q_, q_dot_, q_torque_;
    VectorQd q_dot_lpf_;
    // motor position and velocity from elmo
    VectorQd q_motor_, q_dot_motor_, q_torque_motor_;
    // joint + motor position and velocity
    Matrix<rScalar, MODEL_DOF+8, 1> q_4bar_, q_dot_4bar_;
    MatrixQQd four_bar_Jaco_, four_bar_Jaco_inv_;
    std::vector<double> motor_armature, joint_armature, wn, wnwn;
    std::vector<double> torque_limit;
    Eigen::VectorQd Kp_j, Kd_j;
    tf2::Quaternion imu_quat_;

    // motor pos command to elmo
    VectorQd q_pos_desired_motor_, motor_torque_command;
    // VectorQd q_torque_elmo_;
    
    VectorQVQd q_virtual_, q_virtual_local_, q_virtual_local_yaw_initialized;
    VectorVQd q_dot_virtual_, q_dot_virtual_local_;

    Vector3d q_virtual_mjc_, q_dot_virtual_mjc_;

    //--- Pinocchio
    LinkData link_[LINK_NUMBER + 1];
    LinkData link_local_[LINK_NUMBER + 1];
    pinocchio::FrameIndex link_id_[LINK_NUMBER + 1];

    pinocchio::Model model_, model_local_;
    pinocchio::Data data_, data_local_;

    double total_mass_ = 0.0;

    MatrixVVd A_, A_inv_, C_;
    VectorVQd G_;

    Eigen::Vector6d centroidal_momentum_;
    Eigen::Vector3d centroidal_angular_momentum_;
    Eigen::Vector3d centroidal_linear_momentum_;

    double torqOnTime = 0.0;
    double torqOffTime = 0.0;
    double torqRatio = 0.0;

    /////////////////////////////////////
    // Logging and Publishing Data
    /////////////////////////////////////
    void *LoggingThread();
    static void *ThreadStarterLogging(void *context) { return ((StateEstimator *)context)->LoggingThread(); }
    void PublishData();
    void StatusPub(const char *str, ...);


    // for logging and publishing data
    int8_t elmo_state_prev[MODEL_DOF], elmo_ctrl_state_prev[MODEL_DOF];
    int sys_state[2];
    int elmo_state[MODEL_DOF];
    int getElmoState(uint16_t state_bit);
    // Elmo state constants (copied from ecat_util.h to avoid dependencies)
    enum ElmoStates {
        ELMO_FAULT = 0,
        ELMO_NOTFAULT = 2,
        ELMO_READY_TO_SWITCH_ON = 3,
        ELMO_SWITCHED_ON = 4,
        ELMO_OPERATION_ENABLE = 1,
    };
    // CANopen status word bit definitions
    static constexpr int READY_TO_SWITCH_ON_BIT = 0;
    static constexpr int SWITCHED_ON_BIT = 1;
    static constexpr int OPERATION_ENABLE_BIT = 2;
    static constexpr int FAULT_BIT = 3;

    sensor_msgs::msg::JointState joint_state_msg_;
    std_msgs::msg::Int8MultiArray elmo_state_msg_;
    std_msgs::msg::Int8MultiArray sys_state_msg_;

    void virtualStateCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void GuiCmdCallback(const std_msgs::msg::String::SharedPtr msg);

private:
    rclcpp::CallbackGroup::SharedPtr cbg_se_;
    rclcpp::executors::SingleThreadedExecutor exec_se_;

    // for publishing data
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr ctrl_time_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_log_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pelv_state_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr imu_state_pub_;
    rclcpp::Publisher<std_msgs::msg::Int8MultiArray>::SharedPtr elmo_state_pub_;
    rclcpp::Publisher<std_msgs::msg::Int8MultiArray>::SharedPtr sys_state_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr imu_pose_pub_;
    // for subscribing data
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr mjc_virtual_state_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr gui_cmd_sub_;
};

#endif