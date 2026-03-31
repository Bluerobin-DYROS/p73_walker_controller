#include "p73_controller/state_estimator.h"
#include <iomanip>

StateEstimator::StateEstimator(DataContainer &dc)
    : dc_(dc), rd_global_(dc.rd_)
{
    // =============================
    // Load robot model from URDF
    // =============================
    string urdf_path = dc_.node_->declare_parameter<string>("urdf_path", "");
    string xml_path = dc_.node_->declare_parameter<string>("xml_path", "");

    // Pinocchio Model Update
    pinocchio::JointModelFreeFlyer floating_base;
    pinocchio::urdf::buildModel(urdf_path, floating_base, model_);
    data_ = pinocchio::Data(model_);

    model_local_ = model_;
    data_local_ = data_;  

    bool pinocchio_model_checker = false;

    if (model_.nv == MODEL_DOF_VIRTUAL)
    {
        for (int i = 0; i < LINK_NUMBER; i++)
        {
            link_id_[i] = model_.getFrameId(P73::LINK_NAME[i]);
            link_[i].Initialize(model_, link_id_[i]);
            link_local_[i].Initialize(model_local_, link_id_[i]);

            if(!model_.existBodyName(P73::LINK_NAME[i]))
                RCLCPP_INFO(dc_.node_->get_logger(), "Failed to get body id at link %d : %s", i, P73::LINK_NAME[i]);
        }

        total_mass_ = pinocchio::computeTotalMass(model_);
        rd_global_.total_mass_ = total_mass_;
        cout << "Total mass: " << total_mass_ << endl;
    }

    // Checker
    if(pinocchio_model_checker)
    {
        cout << "model.nq      : "      << model_.nq << endl;
        cout << "model.nv      : "      << model_.nv << endl;
        cout << "model.njoints : " << model_.njoints << endl;
        cout << "model.nbodies : " << model_.nbodies << endl;
        cout << "model.nframes : " << model_.nframes << endl;

        for (int i = 0; i < LINK_NUMBER; i++)
        {
            pinocchio::FrameIndex fid = link_id_[i];
            cout << "Frame idx           : " << fid << endl;
            cout << "Frame name          : " << model_.frames[fid].name << endl;

            pinocchio::JointIndex jid = model_.frames[fid].parent;
            cout << "Parent Joint idx    : "  << jid << endl;
            cout << "Parent Joint name   : "  << model_.names[jid] << endl;
            cout << "Parent Joint mass   : "   << model_.inertias[jid].mass();
            cout << "Parent Joint com    : "    << model_.inertias[jid].lever();
            cout << "Parent Joint inertia: "<< model_.inertias[jid].inertia();
        }
    }

    pinocchio::urdf::buildModel(urdf_path, rd_global_.model_clik_);
    rd_global_.data_clik_ = pinocchio::Data(rd_global_.model_clik_);



    // =============================
    // Initialize robot interface
    // =============================
    cout << "Library version: " << robot.version() << endl;
    cout << "\nInitializing robot..." << endl;
    if (!robot.init())
    {
        cerr << "Failed to initialize robot!" << endl;
        return;
    }
    cout << "Robot initialized successfully!" << endl;

    // Wait for system to become operational
    cout << "\nWaiting for system to become operational..." << endl;
    while (true && rclcpp::ok())
    {
        robot_data = robot.get_data();
        if (robot_data.joint.valid && robot_data.joint.system_status == ECAT_OPERATIONAL && robot_data.imu.valid)
        {
            cout << "System is operational!" << endl;
            imu_quat_ = tf2::Quaternion(robot_data.imu.orientation[1], 
                                         robot_data.imu.orientation[2], 
                                         robot_data.imu.orientation[3], 
                                         robot_data.imu.orientation[0]); // x, y, z, w

            rd_.roll = robot_data.imu.orientation_euler[0];
            rd_.pitch = robot_data.imu.orientation_euler[1];
            rd_.yaw = robot_data.imu.orientation_euler[2];

            q_dot_virtual_local_.segment(3, 3) = Map<Vector3d>(robot_data.imu.angular_velocity).cast<double>();
            rd_.imu_ang_vel = Map<Vector3d>(robot_data.imu.angular_velocity).cast<double>();
            rd_.imu_lin_acc = Vector3d(robot_data.imu.linear_acceleration[0], robot_data.imu.linear_acceleration[1], robot_data.imu.linear_acceleration[2]);
            
            if (!dc_.simMode){
                rd_.imu_euler_cov.diagonal() = Vector3d(robot_data.imu.orientation_covariance[0], 
                                                        robot_data.imu.orientation_covariance[4], 
                                                        robot_data.imu.orientation_covariance[8]);
                rd_.imu_ang_vel_cov.diagonal() = Vector3d(robot_data.imu.angular_velocity_covariance[0], 
                                                        robot_data.imu.angular_velocity_covariance[4], 
                                                        robot_data.imu.angular_velocity_covariance[8]);
                rd_.imu_lin_acc_cov.diagonal() = Vector3d(robot_data.imu.linear_acceleration_covariance[0], 
                                                        robot_data.imu.linear_acceleration_covariance[4], 
                                                        robot_data.imu.linear_acceleration_covariance[8]);
        }
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }





    // =============================
    // Create callback group
    // =============================
    cbg_se_ = dc_.node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions opts;
    opts.callback_group = cbg_se_;

    gui_cmd_sub_ = dc_.node_->create_subscription<std_msgs::msg::String>("p73/guiCommand", 10, std::bind(&StateEstimator::GuiCmdCallback, this, std::placeholders::_1), opts);
    mjc_virtual_state_sub_ = dc_.node_->create_subscription<std_msgs::msg::Float64MultiArray>("p73/mjcVirtualState", 10, std::bind(&StateEstimator::virtualStateCallback, this, std::placeholders::_1), opts);
    exec_se_.add_callback_group(cbg_se_, dc_.node_->get_node_base_interface());

    ctrl_time_pub_ = dc_.node_->create_publisher<std_msgs::msg::Float32>("p73/ctrlTime", 10);
    joint_state_pub_ = dc_.node_->create_publisher<sensor_msgs::msg::JointState>("p73/jointState", 10);
    joint_state_msg_.name.resize(MODEL_DOF);
    joint_state_msg_.position.resize(MODEL_DOF);
    joint_state_msg_.velocity.resize(MODEL_DOF);
    joint_state_msg_.effort.resize(MODEL_DOF);
    for(int i = 0; i < MODEL_DOF; i++) 
        joint_state_msg_.name[i] = P73::JOINT_NAME[i];

    status_log_pub_ = dc_.node_->create_publisher<std_msgs::msg::String>("p73/statusLog", 10);
    pelv_state_pub_ = dc_.node_->create_publisher<std_msgs::msg::Float64MultiArray>("p73/pelvState", 10);
    imu_state_pub_ = dc_.node_->create_publisher<std_msgs::msg::Float64MultiArray>("p73/imuState", 10);
    
    elmo_state_pub_ = dc_.node_->create_publisher<std_msgs::msg::Int8MultiArray>("p73/elmoState", 10);
    elmo_state_msg_.data.resize(MODEL_DOF);

    sys_state_pub_ = dc_.node_->create_publisher<std_msgs::msg::Int8MultiArray>("p73/sysState", 10);
    sys_state_msg_.data.resize(4);

    imu_pose_pub_ = dc_.node_->create_publisher<geometry_msgs::msg::Pose>("p73/imuPose", 10);




    
    // =============================
    // Parameter Loader
    // =============================
    //--- Motor inertia
    dc_.node_->declare_parameter<std::vector<double>>("motor_armature", std::vector<double>(MODEL_DOF, 0.0));
    dc_.node_->get_parameter("motor_armature", motor_armature);
    std::cout << "motor_armature: " << " (size=" << motor_armature.size() << "): ";
    for (const auto &param : motor_armature)
        std::cout << std::fixed << std::setprecision(3) << param << " ";
    std::cout << std::endl;

    //--- Joint inertia
    dc_.node_->declare_parameter<std::vector<double>>("joint_armature", std::vector<double>(MODEL_DOF, 0.0));
    dc_.node_->get_parameter("joint_armature", joint_armature);
    std::cout << "joint_armature: " << " (size=" << joint_armature.size() << "): ";
    for (const auto &param : joint_armature)
        std::cout << std::fixed << std::setprecision(3) << param << " ";
    std::cout << std::endl;

    //--- Nautral frequency
    dc_.node_->declare_parameter<std::vector<double>>("wn", std::vector<double>(MODEL_DOF, 0.0));
    dc_.node_->get_parameter("wn", wn);
    wnwn.resize(wn.size());
    for(int i = 0; i < MODEL_DOF; i++){
        wnwn[i] = wn[i] * wn[i];    
    }
    std::cout << "natural freq: " << " (size=" << wn.size() << "): ";
    for (const auto &param : wn)
        std::cout << std::fixed << std::setprecision(3) << param << " ";
    std::cout << std::endl;

    //--- Motor PD Gain
    dc_.node_->declare_parameter<std::vector<double>>("Kp_m", std::vector<double>(MODEL_DOF, 0.0));
    dc_.node_->declare_parameter<std::vector<double>>("Kd_m", std::vector<double>(MODEL_DOF, 0.0));
    dc_.node_->get_parameter("Kp_m", rd_global_.Kp_m);
    dc_.node_->get_parameter("Kd_m", rd_global_.Kd_m);

    std::cout << "Kp_m: " << " (size=" << rd_global_.Kp_m.size() << "): ";
    for (const auto &param : rd_global_.Kp_m)
        std::cout << std::fixed << std::setprecision(3) << param << " ";
    std::cout << std::endl;
    std::cout << "Kd_m: " << " (size=" << rd_global_.Kd_m.size() << "): ";
    for (const auto &param : rd_global_.Kd_m)
        std::cout << std::fixed << std::setprecision(3) << param << " ";
    std::cout << std::endl;

    //--- Joint PD Gain
    rd_global_.Kp_j.resize(rd_global_.Kp_m.size());
    rd_global_.Kd_j.resize(rd_global_.Kd_m.size());

    for(int i = 0; i < MODEL_DOF; i++){
        rd_global_.Kp_j[i] =     joint_armature[i] * wnwn[i];
        rd_global_.Kd_j[i] = 2 * joint_armature[i] * wn[i];
    }

    std::cout << "Kp_j: " << " (size=" << rd_global_.Kp_j.size() << "): ";
    for (const auto &param : rd_global_.Kp_j)
        std::cout << std::fixed << std::setprecision(3) << param << " ";
    std::cout << std::endl;
    std::cout << "Kd_j: " << " (size=" << rd_global_.Kd_j.size() << "): ";
    for (const auto &param : rd_global_.Kd_j)
        std::cout << std::fixed << std::setprecision(3) << param << " ";
    std::cout << std::endl;

    //--- Friction Parameter
    dc_.node_->declare_parameter<std::vector<double>>("tau_coulomb", std::vector<double>(MODEL_DOF, 0.0));
    dc_.node_->declare_parameter<std::vector<double>>("tau_viscous", std::vector<double>(MODEL_DOF, 0.0));
    dc_.node_->get_parameter("tau_coulomb", rd_global_.tau_coulomb);
    dc_.node_->get_parameter("tau_viscous", rd_global_.tau_viscous);

    std::cout << "tau_coulomb: " << " (size=" << rd_global_.tau_coulomb.size() << "): ";
    for (const auto &param : rd_global_.tau_coulomb)
        std::cout << std::fixed << std::setprecision(3) << param << " ";
    std::cout << std::endl;
    std::cout << "tau_viscous: " << " (size=" << rd_global_.tau_viscous.size() << "): ";
    for (const auto &param : rd_global_.tau_viscous)
        std::cout << std::fixed << std::setprecision(3) << param << " ";
    std::cout << std::endl;

    //--- Joint limits
    const Eigen::VectorXd q_lower = model_.lowerPositionLimit;
    const Eigen::VectorXd q_upper = model_.upperPositionLimit;
    const Eigen::VectorXd v_limit = model_.velocityLimit;
    rd_global_.q_min.resize(MODEL_DOF);
    rd_global_.q_max.resize(MODEL_DOF);
    rd_global_.q_dot_max.resize(MODEL_DOF);
    for (int i = 0; i < MODEL_DOF; ++i) {
        rd_global_.q_min[i] = q_lower(7 + i);
        rd_global_.q_max[i] = q_upper(7 + i);
        rd_global_.q_dot_max[i] = v_limit(6 + i);
    }

    std::cout << "q_min: " << " (size=" << rd_global_.q_min.size() << "): ";
    for (const auto &param : rd_global_.q_min)
        std::cout << std::fixed << std::setprecision(3) << param << " ";
    std::cout << std::endl;
    std::cout << "q_max: " << " (size=" << rd_global_.q_max.size() << "): ";
    for (const auto &param : rd_global_.q_max)
        std::cout << std::fixed << std::setprecision(3) << param << " ";
    std::cout << std::endl;
    std::cout << "q_dot_max: " << " (size=" << rd_global_.q_dot_max.size() << "): ";
    for (const auto &param : rd_global_.q_dot_max)
        std::cout << std::fixed << std::setprecision(3) << param << " ";
    std::cout << std::endl;

    //--- Torque limit
    dc_.node_->declare_parameter<std::vector<double>>("torque_limit", std::vector<double>(MODEL_DOF, 0.0));
    dc_.node_->get_parameter("torque_limit", torque_limit);
    std::cout << "torque limit: " << " (size=" << torque_limit.size() << "): ";
    for (const auto &param : torque_limit)
        std::cout << std::fixed << std::setprecision(3) << param << " ";
    std::cout << std::endl;

    //--- Pose handling option
    dc_.node_->declare_parameter<bool>("zero_rpy_for_circular_traj", true);
    dc_.node_->get_parameter("zero_rpy_for_circular_traj", zero_rpy_for_circular_traj_);
    std::cout << "zero_rpy_for_circular_traj: " << (zero_rpy_for_circular_traj_ ? "true" : "false") << std::endl;
}

StateEstimator::~StateEstimator()
{
    robot.cleanup();
    cout << "StateEstimator Terminate" << creset << endl;
}

void *StateEstimator::StateEstimatorThread()
{
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    cout << cblue << "THREAD0 : StateEstimatorThread Start" << creset << endl;

    // rcv_cnt = robot.get_data().joint.ecat_cnt;

    auto time_start = std::chrono::steady_clock::now();
    rcv_cnt = 0;
    
    tv_us1.tv_sec = 0;
    tv_us1.tv_nsec = 10000;
    while (rclcpp::ok())
    {
        auto loop_start = std::chrono::steady_clock::now();
        auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(loop_start - time_start).count();
        if (!dc_.simMode)
            control_time_ = elapsed_ms / 1000.0;

        auto t0 = chrono::steady_clock::now();
        exec_se_.spin_once(std::chrono::microseconds(10));
        if (!robot.is_initialized())
            break;
        
        GetRobotData();
        auto d0 = chrono::duration_cast<chrono::microseconds>(chrono::steady_clock::now() - t0).count();
        auto t1 = chrono::steady_clock::now();
        // auto dur_start_ = chrono::duration_cast<chrono::microseconds>(chrono::steady_clock::now() - time_start).count(); 
        InitYaw();
        UpdateKinematicsLocal();
        auto d1 = chrono::duration_cast<chrono::microseconds>(chrono::steady_clock::now() - t1).count();
        auto t2 = chrono::steady_clock::now();
        StateEstimate();
        UpdateDynamics();
        UpdateKinematics();
        auto d2 = chrono::duration_cast<chrono::microseconds>(chrono::steady_clock::now() - t2).count();
        auto t3 = chrono::steady_clock::now();
        StoreState(rd_global_);
        SendCommand();
        auto d3 = chrono::duration_cast<chrono::microseconds>(chrono::steady_clock::now() - t3).count();

        if (!dc_.simMode)
        {

            if ((d0 + d1 + d2 + d3) > 1000)
            {
                if (control_time_ > 0.1)
                    printf(" STATE : %7.1f stm over 1000, d0 : %ld, d1 : %ld, d2 : %ld, d3 : %ld\n", control_time_, d0, d1, d2, d3);
            }
        }
        // Sleep to maintain loop rate
        auto loop_end = std::chrono::steady_clock::now();
        auto loop_duration = std::chrono::duration_cast<std::chrono::microseconds>(loop_end - loop_start).count();
        auto sleep_time = static_cast<long>(1000.0) - loop_duration;
        
        if (sleep_time > 0)
        {
            std::this_thread::sleep_for(std::chrono::microseconds(sleep_time));
        }
    }
    // Set to IDLE mode
    cout << cgreen << "\n=== Setting to IDLE mode ===" << creset << endl;
    JointCommand idle_cmd;
    for (int i = 0; i < MODEL_DOF; i++)
    {
        robot.send_command(idle_cmd);
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    cout << cyellow << "THREAD0 : StateEstimatorThread end" << creset << endl;
    return NULL;
}

void StateEstimator::GetRobotData()
{
    robot_data = robot.get_data();
    while (!(robot_data.joint.valid) && rclcpp::ok())
    {
        clock_nanosleep(CLOCK_MONOTONIC, 0, &tv_us1, NULL);
        robot_data = robot.get_data();
        __asm__("pause" ::
            : "memory");
    }

    // =============================
    // Encoder Data
    // =============================
    if (robot_data.joint.valid){
        sys_state[1] = robot_data.joint.system_status;

        //--- Simulation Mode
        if (dc_.simMode) {
            rcv_cnt = robot_data.joint.ecat_cnt;
            control_time_ = rcv_cnt / 1000.0;
            
            q_ = Map<VectorQd>(robot_data.joint.position_external).cast<double>();
            q_dot_ = Map<VectorQd>(robot_data.joint.velocity).cast<double>();
            q_torque_ = Map<VectorQd>(robot_data.joint.torque).cast<double>();   
        }
        //--- Realrobot Mode
        else 
        {
            rcv_cnt++;
            for(int i = 0; i < MODEL_DOF; i++)
            {
                const int actuator_idx = P73::ELMO_2_JOINT[i];
                q_motor_(i) = robot_data.joint.position_external[actuator_idx];
                q_dot_motor_(i) = robot_data.joint.velocity[actuator_idx];
                q_torque_motor_(i) = robot_data.joint.torque[actuator_idx];
                elmo_state[i] = getElmoState(robot_data.joint.status_word[actuator_idx]);
            }

            four_bar_kinematics_.Motor2JointPosVel(q_motor_, q_, q_dot_motor_, q_dot_);
            four_bar_Jaco_ = four_bar_kinematics_.getFourBarJaco();
            four_bar_Jaco_inv_ = four_bar_Jaco_.inverse();

            q_torque_ = four_bar_Jaco_.transpose().inverse() * q_torque_motor_;
        }                  

        //---Joint armature
        // Eigen::MatrixQQd motor_armature_matrix; motor_armature_matrix.setZero();
        // Eigen::MatrixQQd joint_armature_matrix; joint_armature_matrix.setZero();
        // motor_armature_matrix.diagonal() = Eigen::VectorXd::Map(motor_armature.data(), motor_armature.size());
        // joint_armature_matrix = four_bar_Jaco_.transpose() * motor_armature_matrix * four_bar_Jaco_; 
        // for(int i = 0; i < MODEL_DOF; i++){
        //     joint_armature[i] = joint_armature_matrix(i, i);
        // }

        // std::cout << "===== INFO =====" << std::endl;
        // std::cout << "joint_armature: " << std::endl;
        // std::cout << std::fixed << std::setprecision(5);
        // for(int i = 0; i < MODEL_DOF; i++){
        //     std::cout << joint_armature[i] << std::endl;
        // }
        // std::cout << std::defaultfloat;
        // std::cout << " " << std::endl;

        q_virtual_local_.segment(7, MODEL_DOF) = q_;
        q_dot_virtual_local_.segment(6, MODEL_DOF) = q_dot_;  

        if (dc_.useMjcVirtual)
        {
            q_virtual_local_.segment(0, 3) = q_virtual_mjc_;
            q_dot_virtual_local_.segment(0, 3) = q_dot_virtual_mjc_;
        }
        else
        {
            q_virtual_local_.segment(0, 3) = Vector3d::Zero();
            q_dot_virtual_local_.segment(0, 3) = Vector3d::Zero();
        }
    }

    // =============================
    // IMU Data
    // =============================
    if (robot_data.imu.valid){
        sys_state[0] = robot_data.imu.status_flag;
        imu_quat_ = tf2::Quaternion(robot_data.imu.orientation[1], 
                                    robot_data.imu.orientation[2], 
                                    robot_data.imu.orientation[3], 
                                    robot_data.imu.orientation[0]); // x, y, z, w


        q_dot_virtual_local_.segment(3, 3) = Map<Vector3d>(robot_data.imu.angular_velocity).cast<double>();
        rd_.imu_ang_vel = Map<Vector3d>(robot_data.imu.angular_velocity).cast<double>();
        rd_.imu_lin_acc = Vector3d(robot_data.imu.linear_acceleration[0], robot_data.imu.linear_acceleration[1], robot_data.imu.linear_acceleration[2]);
        
        if (!dc_.simMode){
            rd_.imu_euler_cov.diagonal() = Vector3d(robot_data.imu.orientation_covariance[0], 
                                                    robot_data.imu.orientation_covariance[4], 
                                                    robot_data.imu.orientation_covariance[8]);
            rd_.imu_ang_vel_cov.diagonal() = Vector3d(robot_data.imu.angular_velocity_covariance[0], 
                                                      robot_data.imu.angular_velocity_covariance[4], 
                                                      robot_data.imu.angular_velocity_covariance[8]);
            rd_.imu_lin_acc_cov.diagonal() = Vector3d(robot_data.imu.linear_acceleration_covariance[0], 
                                                      robot_data.imu.linear_acceleration_covariance[4], 
                                                      robot_data.imu.linear_acceleration_covariance[8]);
        }
    }
}

void StateEstimator::UpdateKinematicsLocal()
{
    pinocchio::forwardKinematics(model_local_, data_local_, q_virtual_local_, q_dot_virtual_local_);
    pinocchio::updateFramePlacements(model_local_, data_local_);
    for (int i = 0; i < LINK_NUMBER; i++)
    {
        link_local_[i].UpdatePosition(model_local_, data_local_);
        link_local_[i].UpdateVW(model_local_, data_local_);
    }
}

void StateEstimator::InitYaw()
{
    tf2::Matrix3x3 m(imu_quat_);
    m.getRPY(rd_.roll, rd_.pitch, rd_.yaw);

    if (dc_.inityawSwitch)
    {
        cout << " STATE : Yaw Initialized : " << rd_.yaw << endl;
        StatusPub("STATE : Yaw Initialized : %f", rd_.yaw);
        rd_.yaw_init = rd_.yaw;
        dc_.inityawSwitch = false;
    }

    tf2::Quaternion q_mod;
    rd_.yaw = rd_.yaw - rd_.yaw_init;

    if (zero_rpy_for_circular_traj_)
    {
        // Keep pelvis orientation fixed to zero for circular trajectory mode.
        rd_.roll = 0.0;
        rd_.pitch = 0.0;
        rd_.yaw = 0.0;
    }

    q_mod.setRPY(rd_.roll, rd_.pitch, rd_.yaw);

    q_virtual_local_(3) = q_mod.getX();
    q_virtual_local_(4) = q_mod.getY();
    q_virtual_local_(5) = q_mod.getZ();
    q_virtual_local_(6) = q_mod.getW();

    // if i get nan, print error
    if (std::isnan(q_virtual_local_(3)) || std::isnan(q_virtual_local_(4)) || std::isnan(q_virtual_local_(5)) || std::isnan(q_virtual_local_(6))) {
        cout << " ERROR : yaw is nan" << endl;
        StatusPub("ERROR : yaw is nan");
    }
}


void StateEstimator::StateEstimate()
{
    if (dc_.se_mode)
    {
        // state estimate on
        static Eigen::Vector3d imu_init;
        const double dt = 1.0 / 1000.0;
        //--- Dimension Info
        const int num_contacts = 2; // Left Foot + Right Foot
        const int dim_contacts = 3 * num_contacts;
        const int num_state = 6 + dim_contacts; // Base Pos + Base Vel + Contact Pos
        const int num_observe  = 2 * dim_contacts + num_contacts; // Relative Pos, Vel between Base and Contact Pos / Foot Height
        //--- LKF State
        static Eigen::VectorXd x_hat_; // Estimated State
        static Eigen::MatrixXd P_;  // Estimated Covariance

        static Eigen::VectorXd ps_, vs_;  // Relative Pos, Vel between Base and Contact Pos 

        static Eigen::MatrixXd F_;  // State Matrix
        static Eigen::MatrixXd B_;  // Input Matrix
        static Eigen::MatrixXd H_;  // Output Matrix
        static Eigen::MatrixXd C1_; 
        static Eigen::MatrixXd C2_; 

        if (dc_.stateEstimateModeSwitch)
        {
            //--- Initial State
            x_hat_.setZero(num_state);
            //--- Initial Estimated Covariance
            P_ = 100. * Eigen::MatrixXd::Identity(num_state, num_state);
            //--- State Matrix
            F_.setIdentity(num_state, num_state);
            F_.block(0, 3, 3, 3) = dt * Eigen::Matrix3d::Identity();
            //--- Input Matrix
            B_.setZero(num_state, 3);
            B_.block(0, 0, 3, 3) = 0.5 * dt * dt * Eigen::Matrix3d::Identity();
            B_.block(3, 0, 3, 3) = dt * Eigen::Matrix3d::Identity();
            //--- Output Matrix
            H_.setZero(num_observe, num_state); 
            C1_.setZero(3, 6); C1_.leftCols(3).setIdentity();
            C2_.setZero(3, 6); C2_.rightCols(3).setIdentity();
            for (int i = 0; i < num_contacts; ++i) {
                H_.block(3 * i,                  0, 3, 6) = C1_;
                H_.block(3 * (num_contacts + i), 0, 3, 6) = C2_;
                H_(2 * dim_contacts + i, 6 + 3 * i + 2) = 1.0;
            }
            H_.block(0, 6, dim_contacts, dim_contacts) = -MatrixXd::Identity(dim_contacts, dim_contacts);

            //--- etc
            imu_init = link_local_[P73::Pelvis].rotm * rd_.imu_lin_acc;
            ps_.setZero(dim_contacts);
            vs_.setZero(dim_contacts);

            dc_.stateEstimateModeSwitch = false;
        }
        //--- LKF Covariance
        static Eigen::MatrixXd Q_;  Q_.setIdentity(num_state, num_state); 
        static Eigen::MatrixXd R_;  R_.setIdentity(num_observe, num_observe); 
        
        double imuProcessNoisePosition_ = 0.02;
        double imuProcessNoiseVelocity_ = 0.02;
        double footProcessNoisePosition_ = 0.002;
        
        double footSensorNoisePosition_ = 0.5;
        double footSensorNoiseVelocity_ = 0.1;
        double footHeightSensorNoise_ = 0.01;

        Q_.block(0, 0, 3, 3) *= imuProcessNoisePosition_;
        Q_.block(3, 3, 3, 3) *= imuProcessNoiseVelocity_;
        Q_.block(6, 6, dim_contacts, dim_contacts) *= footProcessNoisePosition_;

        R_.block(0, 0, dim_contacts, dim_contacts) *= footSensorNoisePosition_;
        R_.block(dim_contacts, dim_contacts, dim_contacts, dim_contacts) *= footSensorNoiseVelocity_;
        R_.block(2 * dim_contacts, 2 * dim_contacts, num_contacts, num_contacts) *= footHeightSensorNoise_;

        std::vector<Eigen::Vector3d> eePos = { link_local_[P73::Left_Foot].xpos, link_local_[P73::Right_Foot].xpos };
        std::vector<Eigen::Vector3d> eeVel = { link_local_[P73::Left_Foot].v, link_local_[P73::Right_Foot].v };
        double contact_height = 0.0715;

        for (int i = 0; i < num_contacts; i++) {
            int i1 = 3 * i;

            int qIndex = 6 + i1;
            int rIndex1 = i1;
            int rIndex2 = dim_contacts + i1;
            int rIndex3 = 2 * dim_contacts + i;
            // bool isContact = rd_global_.ee_[i].contact;
            bool isContact = rd_global_.ee_[i].contact;

            double high_suspect_number = 100.0;
            Q_.block(qIndex, qIndex, 3, 3)   *= (isContact ? 1. : high_suspect_number);
            R_.block(rIndex1, rIndex1, 3, 3) *= (isContact ? 1. : high_suspect_number);
            R_.block(rIndex2, rIndex2, 3, 3) *= (isContact ? 1. : high_suspect_number);
            R_(rIndex3, rIndex3) = (isContact ? 1. : high_suspect_number) * R_(rIndex3, rIndex3);
            ps_.segment(3 * i, 3) = -eePos[i];
            ps_.segment(3 * i, 3)[2] += contact_height;
            vs_.segment(3 * i, 3) = -eeVel[i];
        }

        Eigen::Vector3d imu_acc_dat;
        imu_acc_dat = link_local_[P73::Pelvis].rotm * rd_.imu_lin_acc + rd_global_.grav_ref;
        // imu_acc_dat = imu_acc_dat - imu_init;    

        Eigen::VectorXd feetHeights = Eigen::VectorXd::Zero(num_contacts);

        Eigen::VectorXd y(num_observe);
        y << ps_, vs_, feetHeights;

        //--- Predicted State Estimate
        x_hat_ = F_ * x_hat_ + B_ * imu_acc_dat;
        //--- Predicted Covariance Estimate
        P_ = F_ * P_ * F_.transpose() + Q_;
        //--- Residual
        Eigen::VectorXd ey = y - H_ * x_hat_;
        //--- Residual Covariance
        Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_;
        //--- Updated State Estimate
        x_hat_ += P_ * H_.transpose() * S.llt().solve(ey);
        //--- Updated Covariance ESTIMATE
        P_ = (Eigen::MatrixXd::Identity(num_state, num_state) - P_ * H_.transpose() * S.llt().solve(H_)) * P_;
        P_ = (P_ + P_.transpose()) * 0.5;

        q_virtual_ = q_virtual_local_;
        q_dot_virtual_ = q_dot_virtual_local_;

        if (!dc_.useMjcVirtual) {
            q_virtual_.segment(0, 3)     = x_hat_.segment(0, 3);
            q_dot_virtual_.segment(0, 3) = x_hat_.segment(3, 3);
        }

        static ofstream log_file("/home/kwan/ros2_ws/src/p73_walker_controller/logging/data/state_estimate_log.txt");
        log_file << x_hat_.segment(0, 6).transpose() << " " << q_virtual_mjc_.segment(0, 3).transpose() << " " << q_dot_virtual_mjc_.segment(0,3).transpose() << std::endl; 
    }
    else
    {
        // state estimate off
        q_virtual_ = q_virtual_local_;
        q_dot_virtual_ = q_dot_virtual_local_;
    }
}

void StateEstimator::UpdateKinematics()
{
    pinocchio::forwardKinematics(model_, data_, q_virtual_, q_dot_virtual_);
    pinocchio::updateFramePlacements(model_, data_);
    pinocchio::computeJointJacobians(model_, data_, q_virtual_);
    for (int i = 0; i < LINK_NUMBER; i++)
    {
        link_[i].UpdatePosition(model_, data_);
        link_[i].UpdateJacobian(model_, data_);
    }
    // --- center of mass calculation
    pinocchio::centerOfMass(model_, data_, q_virtual_, q_dot_virtual_);  
    pinocchio::jacobianCenterOfMass(model_, data_, q_virtual_);  

    link_[P73::COM_id].mass = total_mass_;
    link_[P73::COM_id].xpos = data_.com[0];
    link_[P73::COM_id].v    = data_.vcom[0];
    link_[P73::COM_id].w    = link_[P73::Pelvis].w;
    link_[P73::COM_id].rotm = link_[P73::Pelvis].rotm;

    link_[P73::COM_id].jac.setZero(6, MODEL_DOF_VIRTUAL);
    link_[P73::COM_id].jac.block(0, 0, 3, MODEL_DOF_VIRTUAL) = data_.Jcom;
    link_[P73::COM_id].jac.block(3, 0, 3, MODEL_DOF_VIRTUAL) = link_[P73::Pelvis].jac.block(3, 0, 3, MODEL_DOF_VIRTUAL);

    pinocchio::ccrba(model_, data_, q_virtual_, q_dot_virtual_);
    Eigen::Matrix3d i_com_ = data_.Ig.inertia();
    link_[P73::COM_id].inertia = i_com_;
    // -------------------- Debug print --------------------
    // std::cout << "\n[CoM DEBUG INFO]" << std::endl;
    // std::cout << "------------------------------------------" << std::endl;
    // std::cout << "Total mass       : " << link_[P73::COM_id].mass << std::endl;
    // std::cout << "CoM position [m] : " << link_[P73::COM_id].xpos.transpose() << std::endl;
    // std::cout << "CoM velocity [m/s]: " << link_[P73::COM_id].v.transpose() << std::endl;

    // std::cout << "\nCombined CoM Jacobian (6 x " << MODEL_DOF_VIRTUAL << "):" << std::endl;
    // std::cout << link_[P73::COM_id].jac << std::endl;

    // std::cout << "\nCoM inertia matrix [kg·m²]:" << std::endl;
    // std::cout << link_[P73::COM_id].inertia << std::endl;
    // std::cout << "------------------------------------------\n" << std::endl;
}

void StateEstimator::UpdateDynamics()
{
    pinocchio::normalize(model_, q_virtual_);
    pinocchio::crba(model_, data_, q_virtual_);

    data_.M.triangularView<Eigen::StrictlyLower>() = data_.M.transpose().triangularView<Eigen::StrictlyLower>();
    
    A_ = data_.M;   
    A_inv_ = A_.inverse();
    C_ = pinocchio::computeCoriolisMatrix(model_, data_, q_virtual_, q_dot_virtual_); 
    G_ = pinocchio::computeGeneralizedGravity(model_, data_, q_virtual_);

    pinocchio::computeCentroidalMomentum(model_, data_, q_virtual_, q_dot_virtual_);
    centroidal_momentum_         = data_.hg.toVector();   // 6x1 = [angular; linear]
    centroidal_angular_momentum_ = data_.hg.angular();    // 3x1
    centroidal_linear_momentum_  = data_.hg.linear();     // 3x1
}

void StateEstimator::StoreState(RobotEigenData &rd_global_)
{
    memcpy(&rd_global_.model_, &model_, sizeof(pinocchio::Model));
    memcpy(&rd_global_.data_,  &data_,  sizeof(pinocchio::Data));

    for (int i = 0; i < (LINK_NUMBER + 1); i++)
    {
        memcpy(&rd_global_.link_[i].jac,  &link_[i].jac, sizeof(Matrix6Vd));
        memcpy(&rd_global_.link_[i].xpos, &link_[i].xpos, sizeof(Vector3d));
        memcpy(&rd_global_.link_[i].rotm, &link_[i].rotm, sizeof(Matrix3d));
        memcpy(&rd_global_.link_[i].v,    &link_[i].v, sizeof(Vector3d));
        memcpy(&rd_global_.link_[i].w,    &link_[i].w, sizeof(Vector3d));
    }

    memcpy(&rd_global_.A_,     &A_,     sizeof(MatrixVVd));
    memcpy(&rd_global_.A_inv_, &A_inv_, sizeof(MatrixVVd));
    memcpy(&rd_global_.C_,     &C_,     sizeof(MatrixVVd));
    memcpy(&rd_global_.G_,     &G_,     sizeof(VectorVQd));  

    memcpy(&rd_global_.centroidal_momentum_, &centroidal_momentum_, sizeof(Eigen::Vector6d));
    memcpy(&rd_global_.centroidal_angular_momentum_, &centroidal_angular_momentum_, sizeof(Eigen::Vector3d));
    memcpy(&rd_global_.centroidal_linear_momentum_, &centroidal_linear_momentum_, sizeof(Eigen::Vector3d));
    
    memcpy(&rd_global_.four_bar_Jaco_, &four_bar_Jaco_, sizeof(MatrixQQd));  
    memcpy(&rd_global_.four_bar_Jaco_inv_, &four_bar_Jaco_inv_, sizeof(MatrixQQd));  

    for (int i = 0; i < (LINK_NUMBER + 1); i++)
    {
        memcpy(&rd_global_.link_local_[i].jac,  &link_local_[i].jac, sizeof(Matrix6Vd));
        memcpy(&rd_global_.link_local_[i].xpos, &link_local_[i].xpos, sizeof(Vector3d));
        memcpy(&rd_global_.link_local_[i].rotm, &link_local_[i].rotm, sizeof(Matrix3d));
        memcpy(&rd_global_.link_local_[i].v,    &link_local_[i].v, sizeof(Vector3d));
        memcpy(&rd_global_.link_local_[i].w,    &link_local_[i].w, sizeof(Vector3d));
    }

    memcpy(&rd_global_.q_, &q_, sizeof(VectorQd));
    memcpy(&rd_global_.q_dot_, &q_dot_, sizeof(VectorQd));
    memcpy(&rd_global_.q_motor_, &q_motor_, sizeof(VectorQd));
    memcpy(&rd_global_.q_dot_motor_, &q_dot_motor_, sizeof(VectorQd));
    memcpy(&rd_global_.q_torque_, &q_torque_, sizeof(VectorQd));
    memcpy(&rd_global_.q_torque_motor_, &q_torque_motor_, sizeof(VectorQd));
    memcpy(&rd_global_.q_virtual_, &q_virtual_, sizeof(VectorQVQd));
    memcpy(&rd_global_.q_dot_virtual_, &q_dot_virtual_, sizeof(VectorVQd));

    rd_global_.roll = rd_.roll;
    rd_global_.pitch = rd_.pitch;
    rd_global_.yaw = rd_.yaw;

    rd_global_.control_time_ = control_time_;
    rd_global_.control_time_us_ = static_cast<float>(control_time_ * 1.0e6);

    if (!rd_global_.firstCalc)
    {
        memcpy(&rd_global_.link_, link_, (LINK_NUMBER + 1) * sizeof(LinkData));
        rd_global_.firstCalc = true;
    }
    dc_.triggerThread1 = true;
}

void StateEstimator::SendCommand()
{    
    timespec t_u10;
    t_u10.tv_nsec = 10000;
    t_u10.tv_sec = 0;

    static double command[MODEL_DOF];
    VectorQd torque_command;
    // static double joint_desired_pos_[MODEL_DOF];
    // wait for torque or position command to be sent from p73 controller
    while (dc_.torque_control_running && rclcpp::ok())
        clock_nanosleep(CLOCK_MONOTONIC, 0, &t_u10, NULL);
    dc_.torque_control_running = true;
    std::copy(dc_.command_, dc_.command_ + MODEL_DOF, command);
    // std::copy(dc_.command_, dc_.command_ + MODEL_DOF, joint_desired_pos_);
    dc_.torque_control_running = false;

    //--- Realrobot mode
    if (!dc_.simMode)
    {
        float control_time_at_ = rd_global_.control_time_;
        if (dc_.torqOnSwitch) {
            dc_.pauseSwitch = true;
            dc_.torqOnSwitch = false;
            if (dc_.torqOn)
                cout << " STATE : Torque is already on " << endl;
            else {
                cout << " STATE : Turning on ... " << endl;
                torqOnTime = control_time_;
                dc_.torqOn = true;
                dc_.torqRisingSeq = true;
            }
        }
        else if (dc_.torqOffSwitch) {
            dc_.torqOffSwitch = false;
            if (dc_.torqOn) {
                cout << " STATE : Turning off ... " << endl;
                torqOffTime = control_time_;
                dc_.torqDecreaseSeq = true;
            }
            else 
                cout << " STATE : Torque is already off " << endl;
        }
        const double rTime1 = 4.0;
        const double rTime2 = 1.0;
        const double rat1 = 0.3;
        const double rat2 = 0.7;   
        const double maxTorque = _MAXTORQUE; // SYSTEM MAX TORQUE
        int maxTorqueCommand;
        if (dc_.torqOn)
        {
            if (dc_.torqRisingSeq)
            {
                if (control_time_at_ <= torqOnTime + rTime1)
                    torqRatio = rat1 * DyrosMath::minmax_cut((control_time_at_ - torqOnTime) / rTime1, 0.0, 1.0);

                if (control_time_at_ > torqOnTime + rTime1 && control_time_at_ <= torqOnTime + rTime1 + rTime2)
                    torqRatio = rat1 + rat2 * DyrosMath::minmax_cut((control_time_at_ - torqOnTime - rTime1) / rTime2, 0.0, 1.0);
                else if (control_time_at_ > torqOnTime + rTime1 + rTime2)
                {
                    cout << " STATE : Torque 100% ! " << endl;
                    StatusPub("%f Torque 100%", control_time_);
                    torqRatio = 1.0;
                    dc_.torqRisingSeq = false;
                }
                maxTorqueCommand = maxTorque * torqRatio;
            }
            else if (dc_.torqDecreaseSeq)
            {
                if (control_time_at_ <= torqOffTime + rTime2)
                    torqRatio = (1 - rat2 * DyrosMath::minmax_cut((control_time_at_ - torqOffTime) / rTime2, 0.0, 1.0));

                if (control_time_at_ > torqOffTime + rTime2 && control_time_at_ <= torqOffTime + rTime2 + rTime1)
                    torqRatio = (1 - rat2 - rat1 * DyrosMath::minmax_cut((control_time_at_ - torqOffTime - rTime2) / rTime1, 0.0, 1.0));
                else if (control_time_at_ > torqOffTime + rTime2 + rTime1)
                {
                    dc_.torqDecreaseSeq = false;
                    // rd_global_.tc_run = false;
                    cout << " STATE : Torque 0% .. torque Off " << endl;
                    StatusPub("%f Torque 0%", control_time_);
                    torqRatio = 0.0;
                    dc_.torqOn = false;
                }
                maxTorqueCommand = maxTorque * torqRatio;
            }
            else
            {
                torqRatio = 1.0;
                maxTorqueCommand = (int)maxTorque;
            }
        }
        else
        {
            torqRatio = 0.0;
            maxTorqueCommand = 0;
        }

        //--- Send Torque Command to Robot
        torque_command = Map<VectorQd>(command);

        JointCommand cmd;
        if (robot_data.joint.system_status == ECAT_OPERATIONAL || robot_data.joint.system_status == ECAT_CONTROL){
            cmd.system_command  = CONTROL_COMMAND;
            for (int i = 0; i < MODEL_DOF; i++) {
                if (torque_command(i) > torque_limit[i]) torque_command(i) = torque_limit[i];
                if (torque_command(i) < -torque_limit[i]) torque_command(i) = -torque_limit[i];

                const int actuator_idx = P73::JOINT_2_ELMO[i];
                cmd.target_torque[actuator_idx] = torque_command(i);
                cmd.control_mode[actuator_idx] = TORQUE_CONTROL;
                cmd.max_torque[actuator_idx] = maxTorqueCommand;
            }
        }
        else if (robot_data.joint.system_status == ECAT_SAFETY) {
            if (dc_.safetyResetSwitch) {
                cout << " STATE : Safety reset" << endl;
                StatusPub("STATE : Safety reset");
                cmd.system_command  = SAFETY_CLEAR_COMMAND;
                dc_.safetyResetSwitch = false;
            }
        }
        else {
            cout << " STATE : ECAT_ERROR" << creset << endl;
        }
        robot.send_command(cmd);
    }
    else // mjc sim mode 
    {
        torque_command = Map<VectorQd>(command);

        JointCommand cmd;
        if (robot_data.joint.system_status == ECAT_OPERATIONAL || robot_data.joint.system_status == ECAT_CONTROL){
            cmd.system_command  = CONTROL_COMMAND;
            for (int i = 0; i < MODEL_DOF; i++) {
                cmd.target_torque[i] = torque_command[i];
            }
        }
        robot.send_command(cmd);
    }
}

void StateEstimator::virtualStateCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    q_virtual_mjc_ = Vector3d(msg->data[0], msg->data[1], msg->data[2]);
    q_dot_virtual_mjc_ = Vector3d(msg->data[3], msg->data[4], msg->data[5]);
}

void StateEstimator::GuiCmdCallback(const std_msgs::msg::String::SharedPtr msg)
{
    if (msg->data == "initYaw")
        dc_.inityawSwitch = true;
    else if (msg->data == "stateEstimate") {
        if (dc_.se_mode){
            cout << " STATE : stateestimation off" << endl;
            StatusPub("STATE : stateestimation off");
            dc_.se_mode = false;
        }
        else{
            cout << " STATE : stateestimation on" << endl;
            StatusPub("STATE : stateestimation on");
            // dc_.inityawSwitch = true;
            dc_.stateEstimateModeSwitch = true;
            dc_.se_mode = true;
            dc_.useMjcVirtual = false;
        }
    }
    else if (msg->data == "safetyReset") {
        dc_.safetyResetSwitch = true;
    }
    else if (msg->data == "torqOn") {
        dc_.torqOnSwitch = true;
    }
    else if (msg->data == "torqOff") {
        dc_.torqOffSwitch = true;
    }
    else if (msg->data == "avatarMode") {
        dc_.avatarModeSwitch = true;
    }
    else if (msg->data == "calibrationOn") {
        dc_.calibrationModeSwitch = true;
    }
    else if (msg->data == "readyPose") {
        dc_.readyPoseModeSwitch = true;
    }
}

void *StateEstimator::LoggingThread()
{
    cout << cblue << "THREAD4 : LoggingThread Start" << creset << endl;
    int pub_data_cnt = 0;
    // make all elmo_state_prev to zero
    for (int i = 0; i < MODEL_DOF; i++)
        elmo_state_prev[i] = 0;
    while (rclcpp::ok())
    {
        auto loop_start = chrono::steady_clock::now();
        pub_data_cnt++;
        if (pub_data_cnt % 20 == 0)
            PublishData();
        // rclcpp::spin_some(dc_.node_);
        auto loop_duration = chrono::duration_cast<chrono::microseconds>(chrono::steady_clock::now() - loop_start).count();
        auto sleep_time = static_cast<long>(500.0) - loop_duration;
        if (sleep_time > 0)
            this_thread::sleep_for(chrono::microseconds(sleep_time));
    }
    cout << cyellow << "THREAD4 : LoggingThread End" << creset << endl;
    return NULL;
}

void StateEstimator::PublishData()
{
    std_msgs::msg::Float32 ctrl_time_msg_;
    ctrl_time_msg_.data = control_time_;
    ctrl_time_pub_->publish(ctrl_time_msg_);

    // joint state pub
    joint_state_msg_.header.stamp = dc_.node_->now();
    for (int i = 0; i < MODEL_DOF; i++)
    {
        joint_state_msg_.position[i] = q_virtual_local_[i+7];
        joint_state_msg_.velocity[i] = q_dot_virtual_local_[i+6];
        joint_state_msg_.effort[i] = q_torque_[i];
    }
    joint_state_pub_->publish(joint_state_msg_);
    
    // pelv state pub
    std_msgs::msg::Float64MultiArray pelv_state_msg_;
    pelv_state_msg_.data.resize(13);
    for (int i = 0; i < 3; i++) // se virtual state
    {
        pelv_state_msg_.data[i] = q_virtual_(i);
        pelv_state_msg_.data[i+3] = q_dot_virtual_(i);
    }
    for (int i = 0; i < 3; i++) // mjc virtual state
    {
        pelv_state_msg_.data[i+6] = q_virtual_mjc_(i);
        pelv_state_msg_.data[i+9] = q_dot_virtual_mjc_(i);
    }
    pelv_state_msg_.data[12] = torqRatio;
    pelv_state_pub_->publish(pelv_state_msg_);

    // imu state pub
    std_msgs::msg::Float64MultiArray imu_state_msg_;
    imu_state_msg_.data.resize(10);
    // for (int i = 0; i < 4; i++)
        // imu_state_msg_.data[i] = q_virtual_local_(i+3);
    imu_state_msg_.data[0] = rd_.roll;
    imu_state_msg_.data[1] = rd_.pitch;
    imu_state_msg_.data[2] = rd_.yaw;
    for (int i = 0; i < 3; i++)
        imu_state_msg_.data[i+4] = rd_.imu_ang_vel(i);
    for (int i = 0; i < 3; i++)
        imu_state_msg_.data[i+7] = rd_.imu_lin_acc(i);
    imu_state_pub_->publish(imu_state_msg_);

    // elmo state pub
    bool elmo_publish_flag = false;
    for (int i = 0; i < MODEL_DOF; i++)
    {
        elmo_state_msg_.data[i] = elmo_state[i];
        if (dc_.simMode)
            elmo_state_msg_.data[i] = 1;
        // elmo_state_msg_.data[i+MODEL_DOF] = robot_data.joint.control_status[i];
        // no need
        // elmo_state_msg_.data[i+2*MODEL_DOF] = zp_state(i);
        if (elmo_state_msg_.data[i] != elmo_state_prev[i]) 
            elmo_publish_flag = true;
        elmo_state_prev[i] = elmo_state_msg_.data[i];
        // elmo_ctrl_state_prev[i] = elmo_state_msg_.data[i+MODEL_DOF];
    }
    if (elmo_publish_flag) 
        elmo_state_pub_->publish(elmo_state_msg_);

    // system status pub
    sys_state_msg_.data[0] = dc_.se_mode;
    sys_state_msg_.data[1] = dc_.tc_mode;
    sys_state_msg_.data[2] = sys_state[0]; // IMU status
    sys_state_msg_.data[3] = sys_state[1]; // ECAT status
    // sys_state_msg_.data[4] = sys_state[2];
    if (dc_.simMode) {
        sys_state_msg_.data[2] = 2;
        sys_state_msg_.data[3] = 2;
        // sys_state_msg_.data[4] = 2;
    }
    sys_state_pub_->publish(sys_state_msg_);

    // imu data pub
    geometry_msgs::msg::Pose imu_pose_msg_;
    imu_pose_msg_.position.x = q_virtual_(0);
    imu_pose_msg_.position.y = q_virtual_(1);
    imu_pose_msg_.position.z = q_virtual_(2);
    imu_pose_msg_.orientation.x = q_virtual_(3);
    imu_pose_msg_.orientation.y = q_virtual_(4);
    imu_pose_msg_.orientation.z = q_virtual_(5);
    imu_pose_msg_.orientation.w = q_virtual_(6);
    imu_pose_pub_->publish(imu_pose_msg_);
}

void StateEstimator::StatusPub(const char *str, ...)
{
    va_list lst;
    va_start(lst, str);
    char text_[256];
    vsnprintf(text_, 255, str, lst);
    string str_(text_);
    std_msgs::msg::String str_msg_;
    str_msg_.data = str_;

    status_log_pub_->publish(str_msg_);
    va_end(lst);
}

// Implementation of getElmoState function (copied from ecat_util.c to avoid dependencies)
int StateEstimator::getElmoState(uint16_t state_bit) {
    if (!(state_bit & (1 << OPERATION_ENABLE_BIT))) {
        if (!(state_bit & (1 << SWITCHED_ON_BIT))) {
            if (!(state_bit & (1 << READY_TO_SWITCH_ON_BIT))) {
                if (state_bit & (1 << FAULT_BIT)) {
                    return ELMO_FAULT;
                } else {
                    return ELMO_NOTFAULT;
                }
            } else {
                return ELMO_READY_TO_SWITCH_ON;
            }
        } else {
            return ELMO_SWITCHED_ON;
        }
    } else {
        return ELMO_OPERATION_ENABLE;
    }
}
