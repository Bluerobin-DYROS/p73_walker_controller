#include "p73_controller/p73_controller.h"
using namespace std;

std::filesystem::path data_dir = "/home/kwan/ros2_ws/src/p73_walker_controller/logging/data/";

ofstream joint_desired_log(data_dir / "joint_desired_log.txt");
ofstream joint_position_log(data_dir / "joint_position_log.txt");
ofstream joint_velocity_log(data_dir / "joint_velocity_log.txt");
ofstream foot_traj_log(data_dir / "foot_traj_log.txt");
ofstream torque_joint_log(data_dir / "torque_joint_log.txt");
ofstream torque_motor_log(data_dir / "torque_motor_log.txt");

P73Controller::P73Controller(StateEstimator &stm, rclcpp::Node::SharedPtr node)
    : stm_(stm), dc_(stm.dc_), rd_(stm.dc_.rd_), node_(node)
    #ifdef COMPILE_CC
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

                    const int sinusoid_joint_target_ = 5;
                    const double sinusoid_joint_min_ = 0.2;
                    const double sinusoid_joint_max_ = 0.2;
                    const double sinusoid_period_ = 0.5;

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
                else if (dc_.task_cmd_.task_mode == 2)  // CHIRP DATA COLLECTION
                {
                    rd_.torque_desired.setZero();

                    static bool is_chirp_init = true;
                    static bool chirp_finished = false;
                    static bool chirp_finish_printed = false;
                    static double chirp_start_time = 0.0;

                    static Eigen::VectorQd q_init_ = Eigen::VectorQd::Zero();
                    static Eigen::VectorQd q_last_ = Eigen::VectorQd::Zero();

                    // Chirp configuration
                    const double chirp_amplitude_ = 0.05;  // [rad]
                    const double chirp_f0_ = 0.10;         // [Hz]
                    const double chirp_f1_ = 3.00;         // [Hz]
                    const double chirp_duration_ = 30.0;   // [s]

                    const double current_time = rd_.control_time_;

                    if (is_chirp_init)
                    {
                        q_init_ = rd_.q_;
                        chirp_start_time = current_time;
                        chirp_finished = false;
                        chirp_finish_printed = false;

                        std::cout << "==========================================" << std::endl;
                        std::cout << "======== CHIRP DATA COLLECTION MODE ======" << std::endl;
                        std::cout << "EXCITED JOINTS : ALL" << std::endl;
                        std::cout << "AMPLITUDE [rad]: " << chirp_amplitude_ << std::endl;
                        std::cout << "FREQ START [Hz]: " << chirp_f0_ << std::endl;
                        std::cout << "FREQ END   [Hz]: " << chirp_f1_ << std::endl;
                        std::cout << "DURATION   [s] : " << chirp_duration_ << std::endl;
                        std::cout << "==========================================" << std::endl;

                        is_chirp_init = false;
                    }

                    rd_.q_desired = q_init_;

                    // https://en.wikipedia.org/wiki/Chirp
                    double t = current_time - chirp_start_time;
                    const double k = (chirp_f1_ - chirp_f0_) / chirp_duration_;
                    const double phase = 2.0 * M_PI * (chirp_f0_ * t + 0.5 * k * t * t);

                    if (chirp_finished && !chirp_finish_printed)
                    {
                        std::cout << "CNTRL : Chirp data collection finished. Holding final pose." << std::endl;
                        q_last_ = rd_.q_;
                        chirp_finish_printed = true;
                    }

                    if (!chirp_finished)
                    {
                        if (t <= chirp_duration_)
                        {
                            const double chirp_pos = chirp_amplitude_ * std::sin(phase);
                            for (int i = 0; i < MODEL_DOF; i++)
                            {
                                if(i == 0) 
                                {
                                    rd_.q_desired(i) = q_init_(i) + chirp_pos;
                                }
                                else if(i == 6)  
                                {
                                    rd_.q_desired(i) = q_init_(i) - chirp_pos;
                                }
                                else
                                {
                                    rd_.q_desired(i) = q_init_(i) + chirp_pos;
                                }
                            }
                        }
                        else
                        {
                            chirp_finished = true;
                            rd_.q_desired = q_last_;
                        }
                    }

                    // Compute chirp torques for all joints in joint space.
                    for (int i = 0; i < MODEL_DOF; i++)
                    {
                        rd_.torque_desired(i) = rd_.Kp_j[i] * (rd_.q_desired(i) - rd_.q_(i)) + rd_.Kd_j[i] * (0.0 - rd_.q_dot_(i));
                    }

                    // Log full vectors for all-joint identification.
                    joint_desired_log << rd_.q_desired.transpose() << std::endl;
                    joint_position_log << rd_.q_.transpose() << std::endl;
                    joint_velocity_log << rd_.q_dot_.transpose() << std::endl;
                    torque_joint_log << rd_.torque_desired.transpose() << " " << rd_.q_torque_.transpose() << std::endl;

                    if (!dc_.simMode)
                    {
                        rd_.torque_desired = rd_.four_bar_Jaco_.transpose() * rd_.torque_desired;
                        torque_motor_log << rd_.torque_desired.transpose() << " " << rd_.q_torque_motor_.transpose() << std::endl;
                    }
                }
                else if (dc_.task_cmd_.task_mode == 3)  // IK MODE (FLOAT)
                {
                    static bool is_ik_init = true;
                    static double time_init = 0.0;

                    constexpr double circle_period = 3.0;
                    // constexpr double circle_radius = 0.02;
                    // constexpr double circle_period = 1.0;
                    constexpr double circle_radius = 0.05;


                    static std::string urdf_path;
                    static pinocchio::Model model_clik;
                    static pinocchio::Data data_clik;
                    static pinocchio::FrameIndex left_foot_frame_id;
                    static pinocchio::FrameIndex right_foot_frame_id;

                    if (is_ik_init){
                        time_init = rd_.control_time_;
                        rd_.link_local_[Left_Foot].x_init = rd_.link_local_[Left_Foot].xpos;
                        rd_.link_local_[Right_Foot].x_init = rd_.link_local_[Right_Foot].xpos;
                        rd_.link_local_[Left_Foot].rot_init = rd_.link_local_[Left_Foot].rotm;
                        rd_.link_local_[Right_Foot].rot_init = rd_.link_local_[Right_Foot].rotm;

                        node_->get_parameter("urdf_path", urdf_path);
                        pinocchio::urdf::buildModel(urdf_path, model_clik);
                        data_clik = pinocchio::Data(model_clik);

                        left_foot_frame_id = model_clik.getFrameId("L_Foot_Link");
                        right_foot_frame_id = model_clik.getFrameId("R_Foot_Link");

                        std::cout << "left_foot_frame_id: " << left_foot_frame_id << std::endl;
                        std::cout << "right_foot_frame_id: " << right_foot_frame_id << std::endl;

                        rd_.q_desired = rd_.q_;

                        std::cout << "===================================" << std::endl;
                        std::cout << "========== IK FLOAT Mode ==========" << std::endl;
                        std::cout << "===================================" << std::endl;
                        std::cout << "CLIK MODEL PATH : " << urdf_path << std::endl;
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

                    rd_.link_local_[Left_Foot].r_traj = rd_.link_local_[Left_Foot].rot_init;
                    rd_.link_local_[Right_Foot].r_traj = rd_.link_local_[Right_Foot].rot_init;

                    constexpr int clik_max_iter = 50;
                    constexpr double clik_eps = 1e-4;
                    constexpr double clik_step = 0.001;
                    constexpr double clik_damp = 1e-6;

                    Eigen::VectorQd q_clik = rd_.q_desired;

                    rd_.J_task.setZero(12, MODEL_DOF);
                    rd_.e_task.setZero(12);

                    static Eigen::Vector3d left_x = Eigen::Vector3d::Zero();
                    static Eigen::Vector3d right_x = Eigen::Vector3d::Zero();
                    static Eigen::Matrix3d left_rot = Eigen::Matrix3d::Identity();
                    static Eigen::Matrix3d right_rot = Eigen::Matrix3d::Identity();

                    for (int clik_iter = 0; clik_iter < clik_max_iter; clik_iter++)
                    {
                        pinocchio::forwardKinematics(model_clik, data_clik, q_clik);
                        pinocchio::updateFramePlacements(model_clik, data_clik);
                        pinocchio::computeJointJacobians(model_clik, data_clik, q_clik);

                        Eigen::MatrixXd J_left(6, MODEL_DOF);
                        Eigen::MatrixXd J_right(6, MODEL_DOF);
                        pinocchio::getFrameJacobian(model_clik, data_clik, left_foot_frame_id, pinocchio::LOCAL_WORLD_ALIGNED, J_left);
                        pinocchio::getFrameJacobian(model_clik, data_clik, right_foot_frame_id, pinocchio::LOCAL_WORLD_ALIGNED, J_right);

                        left_x = data_clik.oMf[left_foot_frame_id].translation();
                        right_x = data_clik.oMf[right_foot_frame_id].translation();
                        left_rot = data_clik.oMf[left_foot_frame_id].rotation();
                        right_rot = data_clik.oMf[right_foot_frame_id].rotation();

                        const double kp_pos = 100.0;
                        const double kp_rot = 1.0;
                        rd_.J_task.topRows(6) = J_left;
                        rd_.J_task.bottomRows(6) = J_right;
                        rd_.e_task.segment<3>(0) = kp_pos * (rd_.link_local_[Left_Foot].x_traj - left_x);
                        const Eigen::Matrix3d left_R_err = left_rot.transpose() * rd_.link_local_[Left_Foot].r_traj;
                        rd_.e_task.segment<3>(3) = kp_rot * pinocchio::log3(left_R_err);
                        rd_.e_task.segment<3>(6) = kp_pos * (rd_.link_local_[Right_Foot].x_traj - right_x);
                        const Eigen::Matrix3d right_R_err = right_rot.transpose() * rd_.link_local_[Right_Foot].r_traj;
                        rd_.e_task.segment<3>(9) = kp_rot * pinocchio::log3(right_R_err);

                        if (rd_.e_task.norm() < clik_eps)
                        {
                            break;
                        }

                        Eigen::MatrixXd JJt = rd_.J_task * rd_.J_task.transpose();
                        Eigen::VectorQd q_delta_joint = rd_.J_task.transpose() * (JJt + clik_damp * Eigen::MatrixXd::Identity(12, 12)).ldlt().solve(rd_.e_task);
                        Eigen::VectorQd v_clik = clik_step * q_delta_joint;
                        q_clik = pinocchio::integrate(model_clik, q_clik, v_clik);
                    }

                    static int clik_print_count = 0;
                    if ((clik_print_count++ % 1000) == 0)
                    {
                        std::cout << "========== CLIK LOG ==========" << std::endl;
                        std::cout << "Target Joint Position " << q_clik.transpose() << std::endl;
                        std::cout << "LeftFoot Position " << left_x.transpose() << std::endl;
                        std::cout << "RightFoot Position " << right_x.transpose() << std::endl;
                        std::cout << "LeftFoot PosTraj " <<  rd_.link_local_[Left_Foot].x_traj.transpose() << std::endl;
                        std::cout << "RightFoot PosTraj " << rd_.link_local_[Right_Foot].x_traj.transpose() << std::endl;
                        std::cout << "LeftFoot Rotation "  << std::endl << left_rot << std::endl;
                        std::cout << "RightFoot Rotation " << std::endl << right_rot << std::endl;
                        std::cout << "LeftFoot RotTraj "  << std::endl << rd_.link_local_[Left_Foot].r_traj << std::endl;
                        std::cout << "RightFoot RotTraj " << std::endl << rd_.link_local_[Right_Foot].r_traj << std::endl;
                        std::cout << "CLIK final error: " << rd_.e_task.transpose() << std::endl;
                        std::cout << "=============================" << std::endl;
                    }

                    if(q_clik.allFinite())
                    {
                        rd_.q_desired = q_clik;
                    }
                    else
                    {
                        static bool nan_warning_printed = false;
                        static Eigen::VectorQd q_last = Eigen::VectorQd::Zero();
                        if (!nan_warning_printed)                        {
                            std::cout << "CNTRL WARNING: CLIK solution contains NaN. Holding current joint position." << std::endl;
                            q_last = rd_.q_;
                            nan_warning_printed = true;
                        }
                        rd_.q_desired = q_last;
                    }

                    for (int i = 0; i < MODEL_DOF; i++) {
                        rd_.torque_desired(i) = rd_.Kp_j[i] * (rd_.q_desired(i) - rd_.q_(i)) + rd_.Kd_j[i] * (0.0 - rd_.q_dot_(i));
                    }

                    joint_desired_log << rd_.q_desired.transpose() << std::endl;
                    joint_position_log << rd_.q_.transpose() << std::endl;
                    joint_velocity_log << rd_.q_dot_.transpose() << std::endl;
                    foot_traj_log << rd_.link_local_[Left_Foot].x_traj.transpose() << " " << rd_.link_local_[Right_Foot].x_traj.transpose()  << " " 
                                  << rd_.link_local_[Left_Foot].xpos.transpose() << " " << rd_.link_local_[Right_Foot].xpos.transpose() << std::endl;
                    torque_joint_log << rd_.torque_desired.transpose() << " " << rd_.q_torque_.transpose() << std::endl;

                    if(!dc_.simMode){
                        rd_.torque_desired = WBC::JointTorqueToMotorTorque(rd_, rd_.torque_desired);
                        torque_motor_log << rd_.torque_desired.transpose() << " " << rd_.q_torque_motor_.transpose() << std::endl;
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
#ifdef COMPILE_CC
                else if (dc_.task_cmd_.task_mode >= 5 && dc_.task_cmd_.task_mode < 10)
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
