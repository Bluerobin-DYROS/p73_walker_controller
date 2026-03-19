#ifndef P73_LIB_4BAR_JAC_FUNC_H
#define P73_LIB_4BAR_JAC_FUNC_H

#include "math_type_define.h"

class FourBarKinematics
{
public:
    FourBarKinematics();
    ~FourBarKinematics();

    void Motor2JointPosVel(const Eigen::VectorQd &motor_pos, Eigen::VectorQd &joint_pos, const Eigen::VectorQd &motor_vel, Eigen::VectorQd &joint_vel);
        Eigen::MatrixQQd getFourBarJaco() {return four_bar_Jaco;};
    void Joint2MotorDesiredPos(const Eigen::VectorQd &joint_desired_pos, Eigen::VectorQd &motor_desired_pos);
    void JointTorq2ElmoTorq(const Eigen::VectorQd &joint_torque, Eigen::VectorQd &elmo_torque, const Eigen::VectorQd &motor_pos);

private:
void CalculateJacobians(const Eigen::VectorQd &q_joint_calc,
                        const Eigen::VectorQd &q_motor_calc,
                        Eigen::MatrixXd &Lankle_Jaco,
                        Eigen::MatrixXd &Rankle_Jaco,
                        Eigen::MatrixXd &Lknee_Jaco,
                        Eigen::MatrixXd &Rknee_Jaco,
                        Eigen::MatrixXd &Lankle_Jaco_inv,
                        Eigen::MatrixXd &Rankle_Jaco_inv,
                        Eigen::MatrixXd &Lknee_Jaco_inv,
                        Eigen::MatrixXd &Rknee_Jaco_inv);;

    Eigen::MatrixQQd four_bar_Jaco;

    // State variables for Motor2JointPosVel
    bool first_run_pos_;
    Eigen::VectorQd q_iter_calc_;
    Eigen::VectorQd q_motor_iter_calc_;

    // State variables for Joint2MotorDesiredPos
    bool first_run_des_;
    Eigen::VectorQd q_desired_motor_iter_calc_;
    Eigen::VectorQd q_desired_iter_calc_;

    // State variables for JointTorq2ElmoTorq
    bool first_run_torque_;
    Eigen::VectorQd q_torque_iter_calc;
    Eigen::VectorQd q_torque_motor_iter_calc_;
};

#endif

