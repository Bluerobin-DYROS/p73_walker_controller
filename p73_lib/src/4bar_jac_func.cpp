#include "p73_lib/4bar_jac_func.h"
#include <iostream>

FourBarKinematics::FourBarKinematics()
    : first_run_pos_(true), first_run_des_(true), first_run_torque_(true)
{
    q_iter_calc_.setZero();
    q_motor_iter_calc_.setZero();
    q_desired_motor_iter_calc_.setZero();
    q_desired_iter_calc_.setZero();
    q_torque_iter_calc.setZero();
    q_torque_motor_iter_calc_.setZero();
}

FourBarKinematics::~FourBarKinematics()
{
}
void FourBarKinematics::Motor2JointPosVel(const Eigen::VectorQd &motor_pos,
                                          Eigen::VectorQd &joint_pos,
                                          const Eigen::VectorQd &motor_vel,
                                          Eigen::VectorQd &joint_vel)
{
    // Convert motor-side position/velocity to joint-side position/velocity
    // for the knee and ankle four-bar mechanisms only.

    Eigen::MatrixXd Lankle_Jaco, Rankle_Jaco;
    Eigen::MatrixXd Lankle_Jaco_inv, Rankle_Jaco_inv;
    Eigen::MatrixXd Lknee_Jaco, Rknee_Jaco;
    Eigen::MatrixXd Lknee_Jaco_inv, Rknee_Jaco_inv;

    Lankle_Jaco.setZero(2, 2);
    Rankle_Jaco.setZero(2, 2);
    Lankle_Jaco_inv.setZero(2, 2);
    Rankle_Jaco_inv.setZero(2, 2);
    Lknee_Jaco.setZero(1, 1);
    Rknee_Jaco.setZero(1, 1);
    Lknee_Jaco_inv.setZero(1, 1);
    Rknee_Jaco_inv.setZero(1, 1);

    Eigen::VectorQd q_dot_error;
    Eigen::VectorQd q_dot_calc;
    Eigen::VectorQd q_motor_dot_error;

    int update_iteration_num = 1;
    double update_iteration_dt = 1.0 / 10.0;
    double update_iteration_kp = 8.0;

    if (first_run_pos_)
    {
        first_run_pos_ = false;

        q_iter_calc_ = motor_pos;
        q_iter_calc_.segment(3, 3).setZero();   // Left knee + ankle
        q_iter_calc_.segment(9, 3).setZero();   // Right knee + ankle

        q_motor_iter_calc_ = motor_pos;
        q_motor_iter_calc_.segment(3, 3).setZero();
        q_motor_iter_calc_.segment(9, 3).setZero();

        update_iteration_num = 200;
        update_iteration_dt = 1.0 / 100.0;
        update_iteration_kp = 20.0;
    }

    for (int i = 0; i < update_iteration_num; i++)
    {
        q_motor_dot_error.setZero();
        q_motor_dot_error = update_iteration_kp * RAD2DEG * (motor_pos - q_motor_iter_calc_);

        CalculateJacobians(q_iter_calc_, q_motor_iter_calc_,
                           Lankle_Jaco, Rankle_Jaco,
                           Lknee_Jaco, Rknee_Jaco,
                           Lankle_Jaco_inv, Rankle_Jaco_inv,
                           Lknee_Jaco_inv, Rknee_Jaco_inv);

        q_dot_error.setZero();

        q_dot_error.segment(3, 1) = Lknee_Jaco * q_motor_dot_error.segment(3, 1);
        q_dot_error.segment(4, 2) = Lankle_Jaco * q_motor_dot_error.segment(4, 2);

        q_dot_error.segment(9, 1) = Rknee_Jaco * q_motor_dot_error.segment(9, 1);
        q_dot_error.segment(10, 2) = Rankle_Jaco * q_motor_dot_error.segment(10, 2);

        q_iter_calc_ = q_iter_calc_ + q_dot_error * update_iteration_dt * DEG2RAD;

        q_dot_calc.setZero();

        q_dot_calc.segment(3, 1) = Lknee_Jaco_inv * q_dot_error.segment(3, 1);
        q_dot_calc.segment(4, 2) = Lankle_Jaco_inv * q_dot_error.segment(4, 2);

        q_dot_calc.segment(9, 1) = Rknee_Jaco_inv * q_dot_error.segment(9, 1);
        q_dot_calc.segment(10, 2) = Rankle_Jaco_inv * q_dot_error.segment(10, 2);

        q_motor_iter_calc_ = q_motor_iter_calc_ + q_dot_calc * update_iteration_dt * DEG2RAD;
    }

    CalculateJacobians(q_iter_calc_, q_motor_iter_calc_,
                       Lankle_Jaco, Rankle_Jaco,
                       Lknee_Jaco, Rknee_Jaco,
                       Lankle_Jaco_inv, Rankle_Jaco_inv,
                       Lknee_Jaco_inv, Rknee_Jaco_inv);

    joint_pos = motor_pos;
    joint_pos.segment(3, 1) = q_iter_calc_.segment(3, 1);
    joint_pos.segment(4, 2) = q_iter_calc_.segment(4, 2);

    joint_pos.segment(9, 1) = q_iter_calc_.segment(9, 1);
    joint_pos.segment(10, 2) = q_iter_calc_.segment(10, 2);

    joint_vel = motor_vel;
    joint_vel.segment(3, 1) = Lknee_Jaco * motor_vel.segment(3, 1);
    joint_vel.segment(4, 2) = Lankle_Jaco * motor_vel.segment(4, 2);

    joint_vel.segment(9, 1) = Rknee_Jaco * motor_vel.segment(9, 1);
    joint_vel.segment(10, 2) = Rankle_Jaco * motor_vel.segment(10, 2);

    four_bar_Jaco = Eigen::MatrixQQd::Identity();
    four_bar_Jaco.block(3, 3, 1, 1) = Lknee_Jaco;
    four_bar_Jaco.block(4, 4, 2, 2) = Lankle_Jaco;
    four_bar_Jaco.block(9, 9, 1, 1) = Rknee_Jaco;
    four_bar_Jaco.block(10, 10, 2, 2) = Rankle_Jaco;
}

void FourBarKinematics::Joint2MotorDesiredPos(const Eigen::VectorQd &joint_desired_pos,
                                              Eigen::VectorQd &motor_desired_pos)
{
    // Convert joint-side desired positions to motor-side desired positions
    // for the knee and ankle four-bar mechanisms only.

    Eigen::MatrixXd Lankle_Jaco, Rankle_Jaco;
    Eigen::MatrixXd Lankle_Jaco_inv, Rankle_Jaco_inv;
    Eigen::MatrixXd Lknee_Jaco, Rknee_Jaco;
    Eigen::MatrixXd Lknee_Jaco_inv, Rknee_Jaco_inv;

    Lankle_Jaco.setZero(2, 2);
    Rankle_Jaco.setZero(2, 2);
    Lankle_Jaco_inv.setZero(2, 2);
    Rankle_Jaco_inv.setZero(2, 2);
    Lknee_Jaco.setZero(1, 1);
    Rknee_Jaco.setZero(1, 1);
    Lknee_Jaco_inv.setZero(1, 1);
    Rknee_Jaco_inv.setZero(1, 1);

    Eigen::VectorQd q_dot_error;
    Eigen::VectorQd q_dot_calc;
    Eigen::VectorQd q_motor_dot_error;

    int update_iteration_num = 1;
    double update_iteration_dt = 1.0 / 10.0;
    double update_iteration_kp = 8.0;

    if (first_run_des_)
    {
        first_run_des_ = false;

        q_desired_motor_iter_calc_ = joint_desired_pos;
        q_desired_motor_iter_calc_.segment(3, 3).setZero();
        q_desired_motor_iter_calc_.segment(9, 3).setZero();

        q_desired_iter_calc_ = joint_desired_pos;
        q_desired_iter_calc_.segment(3, 3).setZero();
        q_desired_iter_calc_.segment(9, 3).setZero();

        update_iteration_num = 2000;
        update_iteration_dt = 1.0 / 1000.0;
        update_iteration_kp = 20.0;
    }

    for (int i = 0; i < update_iteration_num; i++)
    {
        q_dot_error.setZero();
        q_dot_error = update_iteration_kp * RAD2DEG * (joint_desired_pos - q_desired_iter_calc_);

        CalculateJacobians(q_desired_iter_calc_, q_desired_motor_iter_calc_,
                           Lankle_Jaco, Rankle_Jaco,
                           Lknee_Jaco, Rknee_Jaco,
                           Lankle_Jaco_inv, Rankle_Jaco_inv,
                           Lknee_Jaco_inv, Rknee_Jaco_inv);

        q_motor_dot_error.setZero();
        q_motor_dot_error.segment(3, 1) = Lknee_Jaco_inv * q_dot_error.segment(3, 1);
        q_motor_dot_error.segment(4, 2) = Lankle_Jaco_inv * q_dot_error.segment(4, 2);

        q_motor_dot_error.segment(9, 1) = Rknee_Jaco_inv * q_dot_error.segment(9, 1);
        q_motor_dot_error.segment(10, 2) = Rankle_Jaco_inv * q_dot_error.segment(10, 2);

        q_desired_motor_iter_calc_ =
            q_desired_motor_iter_calc_ + q_motor_dot_error * update_iteration_dt * DEG2RAD;

        q_dot_calc.setZero();
        q_dot_calc.segment(3, 1) = Lknee_Jaco * q_motor_dot_error.segment(3, 1);
        q_dot_calc.segment(4, 2) = Lankle_Jaco * q_motor_dot_error.segment(4, 2);

        q_dot_calc.segment(9, 1) = Rknee_Jaco * q_motor_dot_error.segment(9, 1);
        q_dot_calc.segment(10, 2) = Rankle_Jaco * q_motor_dot_error.segment(10, 2);

        q_desired_iter_calc_ =
            q_desired_iter_calc_ + q_dot_calc * update_iteration_dt * DEG2RAD;
    }

    motor_desired_pos = joint_desired_pos;
    motor_desired_pos.segment(3, 1) = q_desired_motor_iter_calc_.segment(3, 1);
    motor_desired_pos.segment(4, 2) = q_desired_motor_iter_calc_.segment(4, 2);

    motor_desired_pos.segment(9, 1) = q_desired_motor_iter_calc_.segment(9, 1);
    motor_desired_pos.segment(10, 2) = q_desired_motor_iter_calc_.segment(10, 2);
}
void FourBarKinematics::JointTorq2ElmoTorq(const Eigen::VectorQd &joint_torque,
                                           Eigen::VectorQd &elmo_torque,
                                           const Eigen::VectorQd &motor_pos)
{
    // Map joint-side torques to motor-side torques
    // for the knee and ankle four-bar mechanisms only.

    Eigen::MatrixXd Lankle_Jaco, Rankle_Jaco;
    Eigen::MatrixXd Lankle_Jaco_inv, Rankle_Jaco_inv;
    Eigen::MatrixXd Lknee_Jaco, Rknee_Jaco;
    Eigen::MatrixXd Lknee_Jaco_inv, Rknee_Jaco_inv;

    Lankle_Jaco.setZero(2, 2);
    Rankle_Jaco.setZero(2, 2);
    Lankle_Jaco_inv.setZero(2, 2);
    Rankle_Jaco_inv.setZero(2, 2);
    Lknee_Jaco.setZero(1, 1);
    Rknee_Jaco.setZero(1, 1);
    Lknee_Jaco_inv.setZero(1, 1);
    Rknee_Jaco_inv.setZero(1, 1);

    Eigen::VectorQd q_dot_error;
    Eigen::VectorQd q_dot_calc;
    Eigen::VectorQd q_motor_dot_error;

    int update_iteration_num = 1;
    double update_iteration_dt = 1.0 / 10.0;
    double update_iteration_kp = 8.0;

    if (first_run_torque_)
    {
        first_run_torque_ = false;

        q_torque_iter_calc = motor_pos;
        q_torque_iter_calc.segment(3, 3).setZero();
        q_torque_iter_calc.segment(9, 3).setZero();

        q_torque_motor_iter_calc_ = motor_pos;
        q_torque_motor_iter_calc_.segment(3, 3).setZero();
        q_torque_motor_iter_calc_.segment(9, 3).setZero();

        update_iteration_num = 2000;
        update_iteration_dt = 1.0 / 1000.0;
        update_iteration_kp = 20.0;
    }

    for (int i = 0; i < update_iteration_num; i++)
    {
        q_dot_error.setZero();
        q_dot_error = update_iteration_kp * RAD2DEG * (motor_pos - q_torque_iter_calc);

        CalculateJacobians(q_torque_iter_calc, q_torque_motor_iter_calc_,
                           Lankle_Jaco, Rankle_Jaco,
                           Lknee_Jaco, Rknee_Jaco,
                           Lankle_Jaco_inv, Rankle_Jaco_inv,
                           Lknee_Jaco_inv, Rknee_Jaco_inv);

        q_motor_dot_error.setZero();
        q_motor_dot_error.segment(3, 1) = Lknee_Jaco_inv * q_dot_error.segment(3, 1);
        q_motor_dot_error.segment(4, 2) = Lankle_Jaco_inv * q_dot_error.segment(4, 2);

        q_motor_dot_error.segment(9, 1) = Rknee_Jaco_inv * q_dot_error.segment(9, 1);
        q_motor_dot_error.segment(10, 2) = Rankle_Jaco_inv * q_dot_error.segment(10, 2);

        q_torque_motor_iter_calc_ =
            q_torque_motor_iter_calc_ + q_motor_dot_error * update_iteration_dt * DEG2RAD;

        q_dot_calc.setZero();
        q_dot_calc.segment(3, 1) = Lknee_Jaco * q_motor_dot_error.segment(3, 1);
        q_dot_calc.segment(4, 2) = Lankle_Jaco * q_motor_dot_error.segment(4, 2);

        q_dot_calc.segment(9, 1) = Rknee_Jaco * q_motor_dot_error.segment(9, 1);
        q_dot_calc.segment(10, 2) = Rankle_Jaco * q_motor_dot_error.segment(10, 2);

        q_torque_iter_calc =
            q_torque_iter_calc + q_dot_calc * update_iteration_dt * DEG2RAD;
    }

    elmo_torque = joint_torque;
    elmo_torque.segment(3, 1) = Lknee_Jaco.transpose() * joint_torque.segment(3, 1);
    elmo_torque.segment(4, 2) = Lankle_Jaco.transpose() * joint_torque.segment(4, 2);

    elmo_torque.segment(9, 1) = Rknee_Jaco.transpose() * joint_torque.segment(9, 1);
    elmo_torque.segment(10, 2) = Rankle_Jaco.transpose() * joint_torque.segment(10, 2);
}
void FourBarKinematics::CalculateJacobians(const Eigen::VectorQd &q_joint_calc,
                                           const Eigen::VectorQd &q_motor_calc,
                                           Eigen::MatrixXd &Lankle_Jaco,
                                           Eigen::MatrixXd &Rankle_Jaco,
                                           Eigen::MatrixXd &Lknee_Jaco,
                                           Eigen::MatrixXd &Rknee_Jaco,
                                           Eigen::MatrixXd &Lankle_Jaco_inv,
                                           Eigen::MatrixXd &Rankle_Jaco_inv,
                                           Eigen::MatrixXd &Lknee_Jaco_inv,
                                           Eigen::MatrixXd &Rknee_Jaco_inv)
{
    Eigen::Vector3d Lankle_m1_pos, Lankle_m2_pos, Rankle_m1_pos, Rankle_m2_pos;
    Eigen::Vector3d Lankle_rp_pos1, Lankle_rp_pos2, Rankle_rp_pos1, Rankle_rp_pos2;
    Eigen::Vector3d Lknee_m_pos, Lknee_p_pos, Rknee_m_pos, Rknee_p_pos;

    double Lankle_m1_m_dot_coef, Lankle_m1_p_dot_coef, Lankle_m1_r_dot_coef;
    double Lankle_m2_m_dot_coef, Lankle_m2_p_dot_coef, Lankle_m2_r_dot_coef;
    double Rankle_m1_m_dot_coef, Rankle_m1_p_dot_coef, Rankle_m1_r_dot_coef;
    double Rankle_m2_m_dot_coef, Rankle_m2_p_dot_coef, Rankle_m2_r_dot_coef;
    double Lknee_m_dot_coef, Lknee_p_dot_coef;
    double Rknee_m_dot_coef, Rknee_p_dot_coef;

    // =============================
    // Left ankle Jacobian
    // =============================
    Lankle_m1_pos(0) = 12.007 - 70 * cos(q_motor_calc(4));
    Lankle_m1_pos(1) = 40.700;
    Lankle_m1_pos(2) = 338.738 + 70 * sin(q_motor_calc(4));

    Lankle_rp_pos1(0) = -55 * cos(q_joint_calc(4)) + 27.375 * sin(q_joint_calc(5)) * sin(q_joint_calc(4));
    Lankle_rp_pos1(1) = 27.375 * cos(q_joint_calc(5));
    Lankle_rp_pos1(2) = 55 * sin(q_joint_calc(4)) + 27.375 * sin(q_joint_calc(5)) * cos(q_joint_calc(4));

    Lankle_m1_m_dot_coef = (Lankle_m1_pos(0) - Lankle_rp_pos1(0)) * (70.000 * sin(q_motor_calc(4)))
                         + (Lankle_m1_pos(2) - Lankle_rp_pos1(2)) * (70.000 * cos(q_motor_calc(4)));

    Lankle_m1_p_dot_coef = (Lankle_m1_pos(0) - Lankle_rp_pos1(0)) * (-55.000 * sin(q_joint_calc(4)) - 27.375 * sin(q_joint_calc(5)) * cos(q_joint_calc(4)))
                         + (Lankle_m1_pos(2) - Lankle_rp_pos1(2)) * (-55.000 * cos(q_joint_calc(4)) + 27.375 * sin(q_joint_calc(5)) * sin(q_joint_calc(4)));

    Lankle_m1_r_dot_coef = (Lankle_m1_pos(0) - Lankle_rp_pos1(0)) * (-27.375 * cos(q_joint_calc(5)) * sin(q_joint_calc(4)))
                         + (Lankle_m1_pos(1) - Lankle_rp_pos1(1)) * (27.375 * sin(q_joint_calc(5)))
                         + (Lankle_m1_pos(2) - Lankle_rp_pos1(2)) * (-27.375 * cos(q_joint_calc(5)) * cos(q_joint_calc(4)));

    Lankle_Jaco_inv(0, 0) = -Lankle_m1_p_dot_coef / Lankle_m1_m_dot_coef;
    Lankle_Jaco_inv(0, 1) = -Lankle_m1_r_dot_coef / Lankle_m1_m_dot_coef;

    Lankle_m2_pos(0) = 12.007 - 70 * cos(q_motor_calc(5));
    Lankle_m2_pos(1) = -40.700;
    Lankle_m2_pos(2) = 233.034 - 70 * sin(q_motor_calc(5));

    Lankle_rp_pos2(0) = -55 * cos(q_joint_calc(4)) - 27.375 * sin(q_joint_calc(5)) * sin(q_joint_calc(4));
    Lankle_rp_pos2(1) = -27.375 * cos(q_joint_calc(5));
    Lankle_rp_pos2(2) = 55 * sin(q_joint_calc(4)) - 27.375 * sin(q_joint_calc(5)) * cos(q_joint_calc(4));

    Lankle_m2_m_dot_coef = (Lankle_m2_pos(0) - Lankle_rp_pos2(0)) * (70.000 * sin(q_motor_calc(5)))
                         + (Lankle_m2_pos(2) - Lankle_rp_pos2(2)) * (-70.000 * cos(q_motor_calc(5)));

    Lankle_m2_p_dot_coef = (Lankle_m2_pos(0) - Lankle_rp_pos2(0)) * (-55.000 * sin(q_joint_calc(4)) + 27.375 * sin(q_joint_calc(5)) * cos(q_joint_calc(4)))
                         + (Lankle_m2_pos(2) - Lankle_rp_pos2(2)) * (-55.000 * cos(q_joint_calc(4)) - 27.375 * sin(q_joint_calc(5)) * sin(q_joint_calc(4)));

    Lankle_m2_r_dot_coef = (Lankle_m2_pos(0) - Lankle_rp_pos2(0)) * (27.375 * cos(q_joint_calc(5)) * sin(q_joint_calc(4)))
                         + (Lankle_m2_pos(1) - Lankle_rp_pos2(1)) * (-27.375 * sin(q_joint_calc(5)))
                         + (Lankle_m2_pos(2) - Lankle_rp_pos2(2)) * (27.375 * cos(q_joint_calc(5)) * cos(q_joint_calc(4)));

    Lankle_Jaco_inv(1, 0) = -Lankle_m2_p_dot_coef / Lankle_m2_m_dot_coef;
    Lankle_Jaco_inv(1, 1) = -Lankle_m2_r_dot_coef / Lankle_m2_m_dot_coef;

    Lankle_Jaco = Lankle_Jaco_inv.inverse();

    // =============================
    // Right ankle Jacobian
    // =============================
    Rankle_m1_pos(0) = 12.007 - 70 * cos(q_motor_calc(10));
    Rankle_m1_pos(1) = -40.700;
    Rankle_m1_pos(2) = 338.738 - 70 * sin(q_motor_calc(10));

    Rankle_rp_pos1(0) = -55 * cos(-q_joint_calc(10)) - 27.375 * sin(q_joint_calc(11)) * sin(-q_joint_calc(10));
    Rankle_rp_pos1(1) = -27.375 * cos(q_joint_calc(11));
    Rankle_rp_pos1(2) = 55 * sin(-q_joint_calc(10)) - 27.375 * sin(q_joint_calc(11)) * cos(-q_joint_calc(10));

    Rankle_m1_m_dot_coef = (Rankle_m1_pos(0) - Rankle_rp_pos1(0)) * (70.000 * sin(q_motor_calc(10)))
                         + (Rankle_m1_pos(2) - Rankle_rp_pos1(2)) * (-70.000 * cos(q_motor_calc(10)));

    Rankle_m1_p_dot_coef = (Rankle_m1_pos(0) - Rankle_rp_pos1(0)) * (55.000 * sin(-q_joint_calc(10)) - 27.375 * sin(q_joint_calc(11)) * cos(-q_joint_calc(10)))
                         + (Rankle_m1_pos(2) - Rankle_rp_pos1(2)) * (55.000 * cos(-q_joint_calc(10)) + 27.375 * sin(q_joint_calc(11)) * sin(-q_joint_calc(10)));

    Rankle_m1_r_dot_coef = (Rankle_m1_pos(0) - Rankle_rp_pos1(0)) * (27.375 * cos(q_joint_calc(11)) * sin(-q_joint_calc(10)))
                         + (Rankle_m1_pos(1) - Rankle_rp_pos1(1)) * (-27.375 * sin(q_joint_calc(11)))
                         + (Rankle_m1_pos(2) - Rankle_rp_pos1(2)) * (27.375 * cos(q_joint_calc(11)) * cos(-q_joint_calc(10)));

    Rankle_Jaco_inv(0, 0) = -Rankle_m1_p_dot_coef / Rankle_m1_m_dot_coef;
    Rankle_Jaco_inv(0, 1) = -Rankle_m1_r_dot_coef / Rankle_m1_m_dot_coef;

    Rankle_m2_pos(0) = 12.007 - 70 * cos(q_motor_calc(11));
    Rankle_m2_pos(1) = 40.700;
    Rankle_m2_pos(2) = 233.034 + 70 * sin(q_motor_calc(11));

    Rankle_rp_pos2(0) = -55 * cos(-q_joint_calc(10)) + 27.375 * sin(q_joint_calc(11)) * sin(-q_joint_calc(10));
    Rankle_rp_pos2(1) = 27.375 * cos(q_joint_calc(11));
    Rankle_rp_pos2(2) = 55 * sin(-q_joint_calc(10)) + 27.375 * sin(q_joint_calc(11)) * cos(-q_joint_calc(10));

    Rankle_m2_m_dot_coef = (Rankle_m2_pos(0) - Rankle_rp_pos2(0)) * (70.000 * sin(q_motor_calc(11)))
                         + (Rankle_m2_pos(2) - Rankle_rp_pos2(2)) * (70.000 * cos(q_motor_calc(11)));

    Rankle_m2_p_dot_coef = (Rankle_m2_pos(0) - Rankle_rp_pos2(0)) * (55.000 * sin(-q_joint_calc(10)) + 27.375 * sin(q_joint_calc(11)) * cos(-q_joint_calc(10)))
                         + (Rankle_m2_pos(2) - Rankle_rp_pos2(2)) * (55.000 * cos(-q_joint_calc(10)) - 27.375 * sin(q_joint_calc(11)) * sin(-q_joint_calc(10)));

    Rankle_m2_r_dot_coef = (Rankle_m2_pos(0) - Rankle_rp_pos2(0)) * (-27.375 * cos(q_joint_calc(11)) * sin(-q_joint_calc(10)))
                         + (Rankle_m2_pos(1) - Rankle_rp_pos2(1)) * (27.375 * sin(q_joint_calc(11)))
                         + (Rankle_m2_pos(2) - Rankle_rp_pos2(2)) * (-27.375 * cos(q_joint_calc(11)) * cos(-q_joint_calc(10)));

    Rankle_Jaco_inv(1, 0) = -Rankle_m2_p_dot_coef / Rankle_m2_m_dot_coef;
    Rankle_Jaco_inv(1, 1) = -Rankle_m2_r_dot_coef / Rankle_m2_m_dot_coef;

    Rankle_Jaco = Rankle_Jaco_inv.inverse();

    // =============================
    // Left knee Jacobian
    // =============================
    Lknee_m_pos(0) = 100 * cos(40 * DEG2RAD - q_motor_calc(3));
    Lknee_m_pos(2) = 100 * sin(40 * DEG2RAD - q_motor_calc(3));

    Lknee_p_pos(0) = -125 * cos(70 * DEG2RAD) + 85 * cos(14.296883 * DEG2RAD - q_joint_calc(3));
    Lknee_p_pos(2) = -125 * sin(70 * DEG2RAD) + 85 * sin(14.296883 * DEG2RAD - q_joint_calc(3));

    Lknee_m_dot_coef = (Lknee_m_pos(0) - Lknee_p_pos(0)) * (100.000 * sin(40 * DEG2RAD - q_motor_calc(3)))
                     + (Lknee_m_pos(2) - Lknee_p_pos(2)) * (-100.000 * cos(40 * DEG2RAD - q_motor_calc(3)));

    Lknee_p_dot_coef = (Lknee_m_pos(0) - Lknee_p_pos(0)) * (-85.000 * sin(14.296883 * DEG2RAD - q_joint_calc(3)))
                     + (Lknee_m_pos(2) - Lknee_p_pos(2)) * (85.000 * cos(14.296883 * DEG2RAD - q_joint_calc(3)));

    Lknee_Jaco_inv(0, 0) = -Lknee_p_dot_coef / Lknee_m_dot_coef;
    Lknee_Jaco = Lknee_Jaco_inv.inverse();

    // =============================
    // Right knee Jacobian
    // =============================
    Rknee_m_pos(0) = 100 * cos(40 * DEG2RAD + q_motor_calc(9));
    Rknee_m_pos(2) = 100 * sin(40 * DEG2RAD + q_motor_calc(9));

    Rknee_p_pos(0) = -125 * cos(70 * DEG2RAD) + 85 * cos(14.296883 * DEG2RAD + q_joint_calc(9));
    Rknee_p_pos(2) = -125 * sin(70 * DEG2RAD) + 85 * sin(14.296883 * DEG2RAD + q_joint_calc(9));

    Rknee_m_dot_coef = (Rknee_m_pos(0) - Rknee_p_pos(0)) * (100.000 * sin(40 * DEG2RAD + q_motor_calc(9)))
                     + (Rknee_m_pos(2) - Rknee_p_pos(2)) * (-100.000 * cos(40 * DEG2RAD + q_motor_calc(9)));

    Rknee_p_dot_coef = (Rknee_m_pos(0) - Rknee_p_pos(0)) * (-85.000 * sin(14.296883 * DEG2RAD + q_joint_calc(9)))
                     + (Rknee_m_pos(2) - Rknee_p_pos(2)) * (85.000 * cos(14.296883 * DEG2RAD + q_joint_calc(9)));

    Rknee_Jaco_inv(0, 0) = -Rknee_p_dot_coef / Rknee_m_dot_coef;
    Rknee_Jaco = Rknee_Jaco_inv.inverse();
}