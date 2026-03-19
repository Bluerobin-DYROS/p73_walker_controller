#ifndef P73_CONTROLLER_H
#define P73_CONTROLLER_H

#include "p73_controller/state_estimator.h"
#include "wholebody_functions.h"

#include "rclcpp/rclcpp.hpp"
#include "p73_msgs/msg/task_cmd.hpp"
#include "p73_msgs/msg/pos_cmd.hpp"
#include "p73_msgs/msg/ik_task_cmd.hpp"
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <iomanip>   
#include <ncurses.h>

#ifdef COMPILE_CC
#include "cc.h"
#endif

class P73Controller
{
public:
    P73Controller(StateEstimator &stm, rclcpp::Node::SharedPtr node);
    ~P73Controller();

    void *TaskCtrlThread();
    void *ComputeSlowThread();
    void *ComputeMpcThread();

    StateEstimator &stm_;
    DataContainer &dc_;
    RobotEigenData &rd_;

#ifdef COMPILE_CC
    CustomController &cc_;
#endif
    FourBarKinematics four_bar_kinematics_;

    static void *Thread1Starter(void *context) { return ((P73Controller *)context)->TaskCtrlThread(); }
    static void *Thread2Starter(void *context) { return ((P73Controller *)context)->ComputeSlowThread(); }
    static void *Thread3Starter(void *context) { return ((P73Controller *)context)->ComputeMpcThread(); }

    void RequestThread2();
    void RequestThread3();

    std::atomic<bool> signalThread1;
    std::atomic<bool> triggerThread2;
    std::atomic<bool> triggerThread3;

    void SendCommand(VectorQd torque_command);

    void taskCmdCallback(const p73_msgs::msg::TaskCmd::SharedPtr msg);
    void ctrlModeCallback(const std_msgs::msg::UInt32::SharedPtr msg);
    void PosCmdCallback(const p73_msgs::msg::PosCmd::SharedPtr msg);
    void IkTaskmodeCallback(const p73_msgs::msg::IKTaskCmd::SharedPtr msg);

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::CallbackGroup::SharedPtr cbg_p73_;
    rclcpp::executors::SingleThreadedExecutor exec_p73_;

    rclcpp::Subscription<p73_msgs::msg::TaskCmd>::SharedPtr task_cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr ctrl_mode_sub_;
    rclcpp::Subscription<p73_msgs::msg::PosCmd>::SharedPtr pos_cmd_sub_;
    rclcpp::Subscription<p73_msgs::msg::IKTaskCmd>::SharedPtr iktask_mode_sub_;
};

#endif