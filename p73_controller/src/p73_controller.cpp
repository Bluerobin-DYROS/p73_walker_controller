#include "p73_controller/p73_controller.h"
using namespace std;

P73Controller::P73Controller(StateEstimator &stm, rclcpp::Node::SharedPtr node)
    : stm_(stm), dc_(stm.dc_), rd_(stm.dc_.rd_), node_(node)
    #ifdef COMPILE_P73_CC
    , cc_(*new CustomController(dc_, rd_))
    #endif
{
    node_->declare_parameter<std::vector<double>>("Kp", std::vector<double>(MODEL_DOF, 0.0));
    node_->declare_parameter<std::vector<double>>("Kd", std::vector<double>(MODEL_DOF, 0.0));
    node_->get_parameter("Kp", rd_.Kp);
    node_->get_parameter("Kd", rd_.Kd);

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
            WBC::FrictionCompensationTorques(rd_);

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
                rd_.torque_desired = WBC::JointPositionToMotorTorque(rd_);
            }
            else if (dc_.tc_mode)
            {
                if (dc_.task_cmd_.task_mode == 0)   // JOINT TUNE MODE
                {

                }
                else if (dc_.task_cmd_.task_mode == 1)  // FRICTION COMPENSATION MODE
                {

                }
                else if (dc_.task_cmd.task_mode == 2)   // SIMPLE JOINT MOTION MODE
                {

                }
                else if (dc_.task_cmd_.task_mode == 3)  // IK MODE (FLOAT)
                {

                }
                else if (dc_.task_cmd_.task_mode == 4)  // IK MODE (CONTACT)
                {

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
                if(!dc_.simMode){
                    rd_.torque_desired = WBC::JointTorqueToMotorTorque(rd_, rd_.torque_desired);
                }

            }
            else
            {
                if(dc_.simMode){
                    WBC::SetContact(rd_, 1, 1);
                    rd_.torque_desired = WBC::ContactForceFrictionConeConstraintTorque(rd_, WBC::GravityCompensationTorque(rd_));
                }
                else{
                    rd_.torque_desired.setZero();
                }

                if(!dc_.simMode){
                    rd_.torque_desired = WBC::JointTorqueToMotorTorque(rd_, rd_.torque_desired);
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
