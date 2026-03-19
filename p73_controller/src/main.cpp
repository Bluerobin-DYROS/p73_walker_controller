#include "br_driver.hpp"
#include "p73_controller/p73_controller.h"
#include <iostream>
#include <thread>
#include <chrono>

using namespace std;


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("p73_controller");
    node->declare_parameter<bool>("sim_mode", true);

    cout << endl
         << cblue << "=====================================" << creset << endl;
    cout << cblue << " CNTRL : Starting P73 CONTROLLER  !  " << creset << endl;
    cout << cblue << "=====================================" << creset << endl;

    // Create robot instance
    DataContainer dc_(node);
    StateEstimator stm(dc_);
    P73Controller p73_(stm, node);

    const int thread_number = 5;

    struct sched_param param;
    pthread_attr_t attrs[thread_number];
    pthread_t threads[thread_number];
    param.sched_priority = 42 + 20;
    cpu_set_t cpusets[thread_number];

    dc_.simMode = node->get_parameter("sim_mode").as_bool();
    if (dc_.simMode)
        cout << cgreen << "Simulation Mode is enabled" << creset << endl;
    else
        cout << cgreen << "Real Robot Mode is enabled" << creset << endl;

    for (int i = 0; i < thread_number; i++)
    {
        if (pthread_attr_init(&attrs[i]))
            printf("attr %d init failed ", i);
        if (!dc_.simMode)
        {
            if (pthread_attr_setschedpolicy(&attrs[i], SCHED_FIFO))
                printf("attr %d setschedpolicy failed ", i);
            if (pthread_attr_setschedparam(&attrs[i], &param))
                printf("attr %d setschedparam failed ", i);
            if (pthread_attr_setinheritsched(&attrs[i], PTHREAD_EXPLICIT_SCHED))
                printf("attr %d setinheritsched failed ", i);
        }
    }

    if (!dc_.simMode)
    {
        // for real robot, set the priority of the state estimator and lowlevel controller
        struct sched_param param_state_estimator;
        struct sched_param param_lowlevel_controller;
        param_state_estimator.sched_priority = 45 + 20;
        param_lowlevel_controller.sched_priority = 45 + 20;
        if (pthread_attr_setschedparam(&attrs[0], &param_state_estimator))
            printf("attr %d setschedparam failed ", 0);
        if (pthread_attr_setschedparam(&attrs[1], &param_lowlevel_controller))
            printf("attr %d setschedparam failed ", 1);

        // for state estimator, 5th core is used
        CPU_ZERO(&cpusets[0]);
        CPU_SET(5, &cpusets[0]);
        if (pthread_attr_setaffinity_np(&attrs[0], sizeof(cpu_set_t), &cpusets[0]))
            printf("attr %d setaffinity failed ", 0);
    }

    if (pthread_create(&threads[0], &attrs[0], &StateEstimator::ThreadStarter, &stm))
        printf("threads[0] create failed\n");
    if (pthread_create(&threads[1], &attrs[1], &P73Controller::Thread1Starter, &p73_))
        printf("threads[1] create failed\n");
    if (pthread_create(&threads[2], &attrs[2], &P73Controller::Thread2Starter, &p73_))
        printf("threads[2] create failed\n");
    if (pthread_create(&threads[3], &attrs[3], &P73Controller::Thread3Starter, &p73_))
        printf("threads[3] create failed\n");
    if (pthread_create(&threads[4], &attrs[4], &StateEstimator::ThreadStarterLogging, &stm))
        printf("threads[4] create failed\n");

    for (int i = 0; i < thread_number; i++)
        pthread_attr_destroy(&attrs[i]);

    for (int i = 0; i < thread_number; i++)
        pthread_join(threads[i], NULL);

    rclcpp::shutdown();
    return 0;
}