#include "p73_controller/p73_controller.h"
using namespace std;

std::filesystem::path data_dir = "/home/bluerobin/ros2_ws/src/data/";

ofstream joint_desired_log(data_dir / "joint_desired_log.txt");
ofstream joint_position_log(data_dir / "joint_position_log.txt");
ofstream joint_velocity_log(data_dir / "joint_velocity_log.txt");
ofstream torque_sum_log(data_dir / "torque_sum_log.txt");

P73Controller::P73Controller(StateEstimator &stm, rclcpp::Node::SharedPtr node)
    : stm_(stm), dc_(stm.dc_), rd_(stm.dc_.rd_), node_(node)
    #ifdef COMPILE_P73_CC
    , cc_(*new CustomController(dc_, rd_))
    #endif
{
    // Create callback group for p73 controller
    cbg_p73_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions opts;
    opts.callback_group = cbg_p73_;

    ctrl_mode_sub_ = node_->create_subscription<std_msgs::msg::UInt32>("p73/ctrlMode", 10, std::bind(&P73Controller::ctrlModeCallback, this, std::placeholders::_1), opts);
    task_cmd_sub_ = node_->create_subscription<p73_msgs::msg::TaskCmd>("p73/taskCommand", 10, std::bind(&P73Controller::taskCmdCallback, this, std::placeholders::_1), opts);
    pos_cmd_sub_ = node_->create_subscription<p73_msgs::msg::PosCmd>("p73/posCommand", 10, std::bind(&P73Controller::PosCmdCallback, this, std::placeholders::_1), opts);
    iktask_mode_sub_ = node_->create_subscription<p73_msgs::msg::IKTaskCmd>("p73/ikTaskmode", 10, std::bind(&P73Controller::IkTaskmodeCallback, this, std::placeholders::_1), opts);

    exec_p73_.add_callback_group(cbg_p73_, node_->get_node_base_interface());
}

P73Controller::~P73Controller()
{
    cout << "P73Controller terminate" << creset << endl;
}

void *P73Controller::TaskCtrlThread()
{

    cout << cblue << "THREAD1 : P73Controller TaskCtrlThread start" << creset << endl;
    signalThread1 = true;

    // --- Waiting...
    while (!rd_.firstCalc)
    {
        std::this_thread::sleep_for(std::chrono::microseconds(1));
        if (!stm_.robot.is_initialized() || !rclcpp::ok())
            break;
    }
    WBC::SetContactInit(rd_);
    WBC::SetTrajectoryInit(rd_);

    while (stm_.robot.is_initialized() && rclcpp::ok()) {
        if (dc_.triggerThread1) {

            exec_p73_.spin_once(std::chrono::microseconds(1));
            dc_.triggerThread1 = false;

            static VectorQd zero_m = VectorQd::Zero();
            WBC::ContactCalcDefault(rd_);
            // WBC::FrictionCompensationTorques(rd_);

            // pause switch
            if (dc_.pauseSwitch)
            {
                dc_.pauseSwitch = false;

                rd_.q_desired = rd_.q_;
                dc_.positionHoldSwitch = true;
                dc_.pc_mode = true;
                dc_.tc_mode = false;
                dc_.ik_mode = false;

                std::cout << " CNTRL : Position Hold!" << rd_.control_time_ << std::endl;
            }

            if (dc_.pc_mode)
            {
                if (!dc_.positionHoldSwitch)
                    rd_.q_desired = DyrosMath::cubicVector(rd_.control_time_, dc_.pos_ctrl_t_, dc_.pos_ctrl_t_ + dc_.pos_ctrl_traj_t_, dc_.pos_ctrl_q_init, dc_.pos_ctrl_q_des, dc_.pos_ctrl_q_vel_init, zero_m);
                
                if(!dc_.simMode){
                    rd_.torque_desired = WBC::JointPositionToMotorTorque(rd_);
                }
                else{
                    for(int i = 0; i < MODEL_DOF; i++)
                    {
                        rd_.torque_desired(i) = rd_.Kp_j[i] * (rd_.q_desired(i) - rd_.q_(i)) + rd_.Kd_j[i] * (0.0 - rd_.q_dot_(i));
                    }
                }
            }
            else if (dc_.tc_mode)
            {
                if (dc_.task_cmd_.task_mode == 0)   // JOINT TUNE MODE
                {
                    rd_.torque_desired.setZero();

                    static bool is_pd_tune_init = true;
                    static double start_time = 0.0;
                    static double phase = 0.0;

                    double current_time = rd_.control_time_;

                    static Eigen::VectorQd q_init_ = Eigen::VectorQd::Zero();
                    static Eigen::VectorQd q_init_motor_ = Eigen::VectorQd::Zero();

                    const int sinusoid_joint_target_ = 12;
                    const double sinusoid_joint_min_ = 0.1;
                    const double sinusoid_joint_max_ = 0.1;
                    const double sinusoid_period_ = 1.5;

                    if (is_pd_tune_init == true)
                    {
                        q_init_ = rd_.q_;
                        q_init_motor_ = rd_.q_motor_;
                        start_time = current_time;

                        const double A = 0.1;
                        const double B = 0.1;

                        const double q0 = q_init_(sinusoid_joint_target_);

                        const double c = q0 + 0.5 * (A - B);
                        const double a = 0.5 * (A + B);

                        double sin_phi = (q0 - c) / a;
                        sin_phi = std::min(1.0, std::max(-1.0, sin_phi));

                        phase = std::asin(sin_phi);

                        std::cout << "==================================" << std::endl;
                        std::cout << "========== PD TUNE Mode ==========" << std::endl;
                        std::cout << "TARGET JOINT : " << sinusoid_joint_target_ << std::endl;
                        std::cout << "JOINT MIN    : " << sinusoid_joint_min_    << std::endl;
                        std::cout << "JOINT MAX    : " << sinusoid_joint_max_    << std::endl;
                        std::cout << "PERIOD [s]   : " << sinusoid_period_       << std::endl;
                        std::cout << "PHASE [rad]   : " << phase                  << std::endl;
                        std::cout << "==================================" << std::endl;
                        is_pd_tune_init = false;
                    }

                    rd_.q_desired = q_init_;

                    //--- Sinusoidal Joint Trajectory
                    const double t = current_time - start_time;
                    const double w = 2.0 * M_PI / sinusoid_period_;

                    const double A = sinusoid_joint_max_;
                    const double B = sinusoid_joint_min_;

                    const double q0 = q_init_(sinusoid_joint_target_);
                    const double c = q0 + 0.5 * (A - B);
                    const double a = 0.5 * (A + B);

                    rd_.q_desired(sinusoid_joint_target_) = c + a * std::sin(w * t + phase);

                    if(!dc_.simMode){
                        for(int i = 0; i < MODEL_DOF; i++)
                        {
                            rd_.torque_desired(i) = (rd_.Kp_m[i]) * (q_init_motor_[i] - rd_.q_motor_[i]) + rd_.Kd_m[i] * (0.0 - rd_.q_dot_motor_(i));
                        }
                    }
                    else{
                        for(int i = 0; i < MODEL_DOF; i++)
                        {
                            rd_.torque_desired(i) = (rd_.Kp_j[i]) * (q_init_[i] - rd_.q_[i]) + rd_.Kd_j[i] * (0.0 - rd_.q_dot_(i));
                        }
                    }

                    Eigen::VectorQd torque_joint; torque_joint.setZero();
                    Eigen::VectorQd torque_motor; torque_motor.setZero();

                    // ================================
                    if(sinusoid_joint_target_ == 0 || sinusoid_joint_target_ == 1 || sinusoid_joint_target_ == 2 || sinusoid_joint_target_ == 3 ||
                       sinusoid_joint_target_ == 6 || sinusoid_joint_target_ == 7 || sinusoid_joint_target_ == 8 || sinusoid_joint_target_ == 9 ||
                       sinusoid_joint_target_ == 12)
                    {
                        for (int i = 0; i < MODEL_DOF; i++) {
                            torque_joint(i) = rd_.Kp_j[i] * (rd_.q_desired(i) - rd_.q_(i)) + rd_.Kd_j[i] * (0.0 - rd_.q_dot_(i));
                        }

                        if(!dc_.simMode)
                        {
                            torque_motor = rd_.four_bar_Jaco_.transpose() * torque_joint;
                        }
                        else
                        {
                            torque_motor = torque_joint;
                        }

                        rd_.torque_desired(sinusoid_joint_target_) = torque_motor(sinusoid_joint_target_);
                    }
                    else if(sinusoid_joint_target_ == 4 || sinusoid_joint_target_ == 5)
                    {
                        for (int i = 0; i < MODEL_DOF; i++) {
                            torque_joint(i) = rd_.Kp_j[i] * (rd_.q_desired(i) - rd_.q_(i)) + rd_.Kd_j[i] * (0.0 - rd_.q_dot_(i));
                        }

                        if(!dc_.simMode)
                        {
                            torque_motor = rd_.four_bar_Jaco_.transpose() * torque_joint;

                            rd_.torque_desired(4) = torque_motor(4);
                            rd_.torque_desired(5) = torque_motor(5);
                        }
                        else
                        {
                            torque_motor = torque_joint;
                            rd_.torque_desired(sinusoid_joint_target_) = torque_motor(sinusoid_joint_target_);
                        }
                    }
                    else if(sinusoid_joint_target_ == 10 || sinusoid_joint_target_ == 11)
                    {
                        for (int i = 0; i < MODEL_DOF; i++) {
                            torque_joint(i) = rd_.Kp_j[i] * (rd_.q_desired(i) - rd_.q_(i)) + rd_.Kd_j[i] * (0.0 - rd_.q_dot_(i));
                        }

                        if(!dc_.simMode)
                        {
                            torque_motor = rd_.four_bar_Jaco_.transpose() * torque_joint;

                            rd_.torque_desired(10) = torque_motor(10);
                            rd_.torque_desired(11) = torque_motor(11);
                        }
                        else
                        {
                            torque_motor = torque_joint;
                            rd_.torque_desired(sinusoid_joint_target_) = torque_motor(sinusoid_joint_target_);
                        }
                    }

                    joint_desired_log << rd_.q_desired(sinusoid_joint_target_) << std::endl;
                    joint_position_log << rd_.q_(sinusoid_joint_target_) << std::endl;
                    joint_velocity_log << rd_.q_dot_(sinusoid_joint_target_) << std::endl;
                    torque_sum_log << rd_.torque_desired.head(12).transpose() << std::endl;
                }
                else if (dc_.task_cmd_.task_mode == 1)  // FRICTION COMPENSATION MODE
                {
                    rd_.torque_desired.setZero();

                    static bool is_pd_tune_init = true;

                    static Eigen::VectorQd q_init_ = Eigen::VectorQd::Zero();
                    static Eigen::VectorQd q_init_motor_ = Eigen::VectorQd::Zero();

                    const int friction_joint_target_ = 12;

                    if (is_pd_tune_init == true)
                    {
                        q_init_ = rd_.q_;
                        q_init_motor_ = rd_.q_motor_;


                        std::cout << "==================================" << std::endl;
                        std::cout << "========== PD TUNE Mode ==========" << std::endl;
                        std::cout << "TARGET JOINT : " << friction_joint_target_ << std::endl;
                        std::cout << "==================================" << std::endl;
                        is_pd_tune_init = false;
                    }

                    rd_.q_desired = q_init_;

                    //--- Sinusoidal Joint Trajectory
                    if(!dc_.simMode){
                        for(int i = 0; i < MODEL_DOF; i++)
                        {
                            rd_.torque_desired(i) = (rd_.Kp_m[i]) * (q_init_motor_[i] - rd_.q_motor_[i]) + rd_.Kd_m[i] * (0.0 - rd_.q_dot_motor_(i));
                        }
                    }
                    else{
                        for(int i = 0; i < MODEL_DOF; i++)
                        {
                            rd_.torque_desired(i) = (rd_.Kp_j[i]) * (q_init_[i] - rd_.q_[i]) + rd_.Kd_j[i] * (0.0 - rd_.q_dot_(i));
                        }
                    }

                    Eigen::VectorQd torque_joint; torque_joint.setZero();
                    Eigen::VectorQd torque_motor; torque_motor.setZero();

                    WBC::FrictionCompensationTorques(rd_, rd_.q_dot_);
                    torque_joint = rd_.torque_fric;

                    // ================================
                    if(friction_joint_target_ == 0 || friction_joint_target_ == 1 || friction_joint_target_ == 2 || friction_joint_target_ == 3 ||
                       friction_joint_target_ == 6 || friction_joint_target_ == 7 || friction_joint_target_ == 8 || friction_joint_target_ == 9 ||
                       friction_joint_target_ == 12)
                    {
                        if(!dc_.simMode)
                        {
                            torque_motor = rd_.four_bar_Jaco_.transpose() * torque_joint;
                        }
                        else
                        {
                            torque_motor = torque_joint;
                        }

                        rd_.torque_desired(friction_joint_target_) = torque_motor(friction_joint_target_);
                    }
                    else if(friction_joint_target_ == 4 || friction_joint_target_ == 5)
                    {
                        if(!dc_.simMode)
                        {
                            torque_motor = rd_.four_bar_Jaco_.transpose() * torque_joint;

                            rd_.torque_desired(4) = torque_motor(4);
                            rd_.torque_desired(5) = torque_motor(5);
                        }
                        else
                        {
                            torque_motor = torque_joint;
                            rd_.torque_desired(friction_joint_target_) = torque_motor(friction_joint_target_);
                        }
                    }
                    else if(friction_joint_target_ == 10 || friction_joint_target_ == 11)
                    {
                        if(!dc_.simMode)
                        {
                            torque_motor = rd_.four_bar_Jaco_.transpose() * torque_joint;

                            rd_.torque_desired(10) = torque_motor(10);
                            rd_.torque_desired(11) = torque_motor(11);
                        }
                        else
                        {
                            torque_motor = torque_joint;
                            rd_.torque_desired(friction_joint_target_) = torque_motor(friction_joint_target_);
                        }
                    }

                }
                else if (dc_.task_cmd_.task_mode == 2)  // SIMPLE JOINT MOTION MODE
                {

                }
                else if (dc_.task_cmd_.task_mode == 3)  // IK MODE (FLOAT)
                {
                    static bool is_ik_init = true;
                    static double time_init = 0.0;

                    constexpr double circle_period = 3.0;
                    constexpr double circle_radius = 0.02;

                    if (is_ik_init){
                        time_init = rd_.control_time_;
                        rd_.link_local_[Left_Foot].x_init = rd_.link_local_[Left_Foot].xpos;
                        rd_.link_local_[Right_Foot].x_init = rd_.link_local_[Right_Foot].xpos;

                        std::cout << "===================================" << std::endl;
                        std::cout << "========== IK FLOAT Mode ==========" << std::endl;
                        std::cout << "===================================" << std::endl;

                        is_ik_init = false;
                    }

                    const double elapsed = rd_.control_time_ - time_init;
                    const double omega = 2.0 * M_PI / circle_period;
                    const double phase_left = omega * elapsed;
                    const double phase_right = phase_left + M_PI;
                    const double phase_left_0 = 0.0;
                    const double phase_right_0 = M_PI;

                    rd_.link_local_[Left_Foot].x_traj = rd_.link_local_[Left_Foot].x_init;

                    rd_.link_local_[Right_Foot].x_traj = rd_.link_local_[Right_Foot].x_init;

                    rd_.link_local_[Left_Foot].x_traj(0)  += circle_radius * (std::cos(phase_left) - std::cos(phase_left_0));
                    rd_.link_local_[Left_Foot].x_traj(2)  += circle_radius * (std::sin(phase_left) - std::sin(phase_left_0));
                    rd_.link_local_[Right_Foot].x_traj(0) += circle_radius * (std::cos(phase_right) - std::cos(phase_right_0));
                    rd_.link_local_[Right_Foot].x_traj(2) += circle_radius * (std::sin(phase_right) - std::sin(phase_right_0));

                    // rd_.J_task.setZero(12, MODEL_DOF);
                    // rd_.e_task.setZero(12);
                    // rd_.J_task.block(0, 0, 6, MODEL_DOF) = rd_.link_local_[Left_Foot].jac.rightCols(MODEL_DOF);
                    // rd_.J_task.block(6, 0, 6, MODEL_DOF) = rd_.link_local_[Right_Foot].jac.rightCols(MODEL_DOF);
                    // rd_.e_task.segment<3>(0) = rd_.link_local_[Left_Foot].x_traj - rd_.link_local_[Left_Foot].xpos;
                    // rd_.e_task.segment<3>(3) = -DyrosMath::getPhi(rd_.link_local_[Left_Foot].rotm, Eigen::Matrix3d::Identity());
                    // rd_.e_task.segment<3>(6) = rd_.link_local_[Right_Foot].x_traj - rd_.link_local_[Right_Foot].xpos;
                    // rd_.e_task.segment<3>(9) = -DyrosMath::getPhi(rd_.link_local_[Right_Foot].rotm, Eigen::Matrix3d::Identity());

                    rd_.J_task.setZero(6, MODEL_DOF);
                    rd_.e_task.setZero(6);
                    rd_.J_task.block(0, 0, 3, MODEL_DOF) = rd_.link_local_[Left_Foot].jac.rightCols(MODEL_DOF).topRows(3);
                    rd_.J_task.block(3, 0, 3, MODEL_DOF) = rd_.link_local_[Right_Foot].jac.rightCols(MODEL_DOF).topRows(3);
                    rd_.e_task.segment<3>(0) = 10.0 * (rd_.link_local_[Left_Foot].x_traj - rd_.link_local_[Left_Foot].xpos);
                    rd_.e_task.segment<3>(3) = 10.0 * (rd_.link_local_[Right_Foot].x_traj - rd_.link_local_[Right_Foot].xpos);

                    Eigen::MatrixXd J_task_joint = rd_.J_task;
                    Eigen::VectorXd q_delta_joint = DyrosMath::pinv_SVD(J_task_joint) * rd_.e_task;
                    rd_.q_desired += q_delta_joint / 1000.0;

                    for (int i = 0; i < MODEL_DOF; i++) {
                        rd_.torque_desired(i) = rd_.Kp_j[i] * (rd_.q_desired(i) - rd_.q_(i)) + rd_.Kd_j[i] * (0.0 - rd_.q_dot_(i));
                    }

                    torque_sum_log << rd_.torque_desired.transpose() << " " << rd_.q_torque_.transpose() << std::endl;

                    if(!dc_.simMode){
                        rd_.torque_desired = WBC::JointTorqueToMotorTorque(rd_, rd_.torque_desired);
                    }
                }
                else if (dc_.task_cmd_.task_mode == 4)  // IK MODE (CONTACT)
                {
                    WBC::SetContact(rd_, true, true);
                    static bool is_ik_init = true;
                    static double time_init = 0.0;
                    if (is_ik_init){
                        time_init = rd_.control_time_;
                        rd_.link_[Pelvis].x_init = rd_.link_[Pelvis].xpos;

                        std::cout << "==================================" << std::endl;
                        std::cout << "========== IK CONTACT Mode ==========" << std::endl;
                        std::cout << "==================================" << std::endl;

                        is_ik_init = false;
                    }

                    rd_.link_[Pelvis].x_desired = rd_.link_[Pelvis].x_init;
                    rd_.link_[Pelvis].x_desired(2) += -0.1;
                    rd_.link_[Pelvis].SetTrajectoryQuintic(rd_.control_time_, time_init, time_init + 3.0, rd_.link_[Pelvis].x_init, rd_.link_[Pelvis].x_desired);

                    rd_.J_task.setZero(6, MODEL_DOF_VIRTUAL);
                    rd_.e_task.setZero(6);
                    rd_.J_task = rd_.link_[Pelvis].jac;
                    rd_.e_task.head(3) = rd_.link_[Pelvis].x_traj - rd_.link_[Pelvis].xpos;
                    rd_.e_task.tail(3) = -DyrosMath::getPhi(rd_.link_[Pelvis].rotm, Eigen::Matrix3d::Identity());
                    WBC::NullspaceInverseKinematics(rd_);

                    for (int i = 0; i < MODEL_DOF; i++) {
                        rd_.torque_desired(i) = rd_.Kp_j[i] * (rd_.q_desired(i) - rd_.q_(i)) + rd_.Kd_j[i] * (0.0 - rd_.q_dot_(i));
                    }
                     
                    if(!dc_.simMode){
                        rd_.torque_desired = WBC::JointTorqueToMotorTorque(rd_, rd_.torque_desired);
                    }
                }
#ifdef COMPILE_P73_CC
                if (dc_.task_cmd_.task_mode >= 5 && dc_.task_cmd_.task_mode < 10)
                {
                    try {
                        cc_.computeFast();
                    }
                    catch (const std::exception &e)
                    {
                        std::cout << "Error occured at RL-BASED CONTROL THREAD1" << std::endl;
                        std::cerr << e.what() << '\n';
                        dc_.positionHoldSwitch = true;
                    }
                    
                }
#endif
            }
            else
            {
                if(dc_.simMode){
                    static Eigen::VectorQd q_init = rd_.q_;

                    for(int i = 0; i < MODEL_DOF; i++)
                    {
                        rd_.torque_desired(i) = rd_.Kp_j[i] * (q_init(i) - rd_.q_(i)) - rd_.Kd_j[i] * rd_.q_dot_(i);
                    }
                }
                else{
                    rd_.torque_desired.setZero();
                }
            }

            SendCommand(rd_.torque_desired);
        }
        else {
            std::this_thread::sleep_for(std::chrono::microseconds(10));
        }
    }
    cout << cyellow << "THREAD1 : TaskCtrlThread end" << creset << endl;
    return NULL;
}

void *P73Controller::ComputeSlowThread()
{
    /*
    This thread is used to compute whole body QP control
    */
    while (rclcpp::ok())
    {
        if (signalThread1 || stm_.robot.is_initialized())
            break;
        std::this_thread::sleep_for(std::chrono::microseconds(10));
    }
    cout << cblue << "THREAD2 : ComputeSlowThread start" << creset << endl;

    while (stm_.robot.is_initialized() && rclcpp::ok())
    {
            if (triggerThread2)
            {
                triggerThread2 = false;
                if (dc_.tc_mode) 
                {

                }
                std::this_thread::sleep_for(std::chrono::microseconds(10));
            }
            else
                std::this_thread::sleep_for(std::chrono::microseconds(10));
    }
    cout << cyellow << "THREAD2 : ComputeSlowThread end" << creset << endl;
    return NULL;
}

void *P73Controller::ComputeMpcThread()
{
    /*
    This thread is used to compute the MPC.
    */
    while (rclcpp::ok())
    {
        if (signalThread1 || stm_.robot.is_initialized())
            break;
        std::this_thread::sleep_for(std::chrono::microseconds(1));
    }
    cout << cblue << "THREAD3 : ComputeMPCThread start" << creset << endl;

    while (stm_.robot.is_initialized() && rclcpp::ok())
    {
        if (triggerThread3)
        {
            triggerThread3 = false;
            if (dc_.tc_mode) {

            }
            std::this_thread::sleep_for(std::chrono::microseconds(10));
        }
        else
            std::this_thread::sleep_for(std::chrono::microseconds(10));
    }
    cout << cyellow << "THREAD3 : ComputeMPCThread end" << creset << endl;
    return NULL;
}

void P73Controller::RequestThread2()
{
    triggerThread2 = true;
}
void P73Controller::RequestThread3()
{
    triggerThread3 = true;
}

void P73Controller::SendCommand(VectorQd torque_command)
{
    while (dc_.torque_control_running && rclcpp::ok())
    {
        std::this_thread::sleep_for(std::chrono::microseconds(1));
    }
    dc_.torque_control_running = true;
    dc_.control_command_count++;
    std::copy(torque_command.data(), torque_command.data() + MODEL_DOF, dc_.command_);
    dc_.torque_control_running = false;
}

void P73Controller::taskCmdCallback(const p73_msgs::msg::TaskCmd::SharedPtr msg)
{
    dc_.pc_mode = false;
    dc_.tc_mode = true;
    dc_.ik_mode = false;
    dc_.task_cmd_ = *msg;
    cout << "CNTRL : task signal received mode :" << dc_.task_cmd_.task_mode << endl;
    stm_.StatusPub("CNTRL : task Control mode : %d", dc_.task_cmd_.task_mode);
}

void P73Controller::ctrlModeCallback(const std_msgs::msg::UInt32::SharedPtr msg)
{
    if (msg->data == 1) // pos ctrl mode
    {
        stm_.StatusPub("%f Position Control Activate", rd_.control_time_.load());
        dc_.pauseSwitch = true;
    }
    else if (msg->data == 2) // grav ctrl mode
    {
        stm_.StatusPub("%f Gravity Compensation Activate", rd_.control_time_.load());
        dc_.pc_mode = false;
        dc_.tc_mode = false;
        dc_.ik_mode = false;
    }
}

void P73Controller::PosCmdCallback(const p73_msgs::msg::PosCmd::SharedPtr msg)
{
    dc_.pos_ctrl_traj_t_ = msg->traj_time;

    dc_.pos_ctrl_t_ = rd_.control_time_;
    dc_.pos_ctrl_q_init = rd_.q_;
    dc_.pos_ctrl_q_vel_init = rd_.q_dot_;

    for (int i = 0; i < MODEL_DOF; i++)
        dc_.pos_ctrl_q_des(i) = msg->position[i];

    dc_.pc_mode = true;
    dc_.tc_mode = false;
    dc_.ik_mode = false;

    dc_.pc_grav = msg->gravity;
    dc_.positionHoldSwitch = false;

    stm_.StatusPub("%f Position Control", (float)rd_.control_time_.load());
    cout << " CNTRL : Position command received" << endl;
}

void P73Controller::IkTaskmodeCallback(const p73_msgs::msg::IKTaskCmd::SharedPtr msg)
{
    dc_.pc_mode = false;
    dc_.tc_mode = false;
    dc_.ik_mode = msg->ik_mode;

    WBC::SetContactInit(rd_);
    WBC::SetTrajectoryInit(rd_);
    rd_.q_desired = rd_.q_;
    dc_.ik_target_link = msg->target_link;
    dc_.ik_target_pos << Vector3d(msg->target_pos[0], msg->target_pos[1], msg->target_pos[2]);
    dc_.ik_time_start = rd_.control_time_;
    dc_.ik_traj_t_ = msg->traj_time;

    cout << "CNTRL : Ik Task Mode signal received" << " target_link: " << dc_.ik_target_link << endl;
    stm_.StatusPub("CNTRL : Ik Task Mode signal received", " target_link: %d", dc_.ik_target_link);
}
