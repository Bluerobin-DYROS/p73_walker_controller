#ifndef TOCABI_H
#define TOCABI_H

#include <iostream>
#include "math_type_define.h"

#define _MAXTORQUE 1000 // 1000 is the max number

namespace P73
{
    const std::string JOINT_NAME[MODEL_DOF] = {
        "L_HipYaw_Joint", "L_HipRoll_Joint", "L_HipPitch_Joint", "L_Knee_Joint", "L_AnklePitch_Joint", "L_AnkleRoll_Joint",
        "R_HipYaw_Joint", "R_HipRoll_Joint", "R_HipPitch_Joint", "R_Knee_Joint", "R_AnklePitch_Joint", "R_AnkleRoll_Joint",
        "WaistYaw_Joint"};

    static constexpr const char *LINK_NAME[LINK_NUMBER] = {
        "base_link",

        "L_HipRoll_Link", "L_HipPitch_Link", "L_Thigh_Link", "L_Knee_Link", "L_AnkleRoll_Link", "L_Foot_Link",
        "R_HipRoll_Link", "R_HipPitch_Link", "R_Thigh_Link", "R_Knee_Link", "R_AnkleRoll_Link", "R_Foot_Link",

        "WaistYaw_Link"};

    const int Pelvis     = 0;
    const int Left_Foot  = 6;
    const int Right_Foot = 12;
    const int Waist = 13;
    const int COM_id = 14;  

    const int JOINT_2_ELMO[MODEL_DOF] = {
        0, 1, 2, 3, 4, 5,
        6, 7, 8, 9, 10, 11, 
        12};

    const int ELMO_2_JOINT[MODEL_DOF] = {
        0, 1, 2, 3, 4, 5,
        6, 7, 8, 9, 10, 11,
        12};

} // namespace P73

#endif