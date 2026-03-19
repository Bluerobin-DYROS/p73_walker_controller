#ifndef WBF_H
#define WBF_H

#include "p73_lib/robot_data.h"
#include "p73_lib/4bar_jac_func.h"
#include "qp.h"

using namespace P73;

namespace WBC
{
    //--- Contact
    void SetContactInit(RobotEigenData &rd_);
    void SetTrajectoryInit(RobotEigenData &rd_);
    void SetContact(RobotEigenData &Robot, bool left_foot, bool right_foot, bool left_hand = 0, bool right_hand = 0);
    void ContactCalcDefault(RobotEigenData &Robot);

    void CalcContact(RobotEigenData &rd_);

    //---Whole-body Dynamics Controller
    VectorQd GravityCompensationTorque(RobotEigenData &rd_);
    VectorQd ContactForceFrictionConeConstraintTorque(RobotEigenData &rd_, Eigen::VectorQd command_torque);
    void FrictionCompensationTorques(RobotEigenData& rd_);

    //---Whole-body Inverse Kinematics Controller
    VectorQd NullspaceInverseKinematics(RobotEigenData &rd_);

    //--- Mapping
    VectorQd JointTorqueToMotorTorque(RobotEigenData& rd_, const Eigen::VectorQd& torque_joint);
    VectorQd JointPositionToMotorTorque(RobotEigenData& rd_);

    void CheckTorqueLimit(RobotEigenData &rd_, VectorQd command_torque);
}

#endif
