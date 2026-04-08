#include "wholebody_functions.h"
#include <onnxruntime/onnxruntime_cxx_api.h>
#include <vector>

using namespace P73;

namespace {
    Ort::Env g_ort_env{ORT_LOGGING_LEVEL_WARNING, "actuatornet"};
    Ort::SessionOptions g_session_options;
    std::vector<Ort::Session> g_sessions;
    bool g_models_ready = false;
}

namespace WBC
{
    void SetContactInit(RobotEigenData &rd_)
    {
        rd_.grav_ref << 0, 0, -GRAVITY;

        rd_.ee_[0].contact_point << 0.05, 0.0, -0.0715;
        rd_.ee_[1].contact_point << 0.05, 0.0, -0.0715;

        rd_.ee_[0].InitializeEE(rd_.link_[P73::Left_Foot],  0.2, 0.06, 50, 0.8);
        rd_.ee_[1].InitializeEE(rd_.link_[P73::Right_Foot], 0.2, 0.06, 50, 0.8);
    }

    void SetTrajectoryInit(RobotEigenData &rd_)
    {
        rd_.link_[Right_Foot].SetInitialWithPosition();
        rd_.link_[Left_Foot].SetInitialWithPosition();
        rd_.link_[Waist].SetInitialWithPosition();
        rd_.link_[Pelvis].SetInitialWithPosition();
        rd_.link_[COM_id].SetInitialWithPosition();
    }

    void SetContact(RobotEigenData &rd_, bool left_foot, bool right_foot)
    {
        rd_.ee_[0].contact = left_foot;
        rd_.ee_[1].contact = right_foot;

        rd_.contact_index = 0;
        if (left_foot)
        {
            rd_.contact_part[rd_.contact_index] = P73::Left_Foot;
            rd_.ee_idx[rd_.contact_index] = 0;
            rd_.contact_index++;
        }
        if (right_foot)
        {
            rd_.contact_part[rd_.contact_index] = P73::Right_Foot;
            rd_.ee_idx[rd_.contact_index] = 1;
            rd_.contact_index++;
        }

        rd_.J_C.setZero(rd_.contact_index * 6, MODEL_DOF_VIRTUAL);

        for (int i = 0; i < rd_.contact_index; i++)
        {
            rd_.J_C.block(i * 6, 0, 6, MODEL_DOF_VIRTUAL) = rd_.ee_[rd_.ee_idx[i]].jac_contact;
        }

        CalcContact(rd_);
    }

    void ContactCalcDefault(RobotEigenData &rd_)
    {
        rd_.ee_[0].SetContact(rd_.model_, rd_.data_, rd_.q_virtual_);
        rd_.ee_[1].SetContact(rd_.model_, rd_.data_, rd_.q_virtual_);
    }

    void CalcContact(RobotEigenData &rd_)
    {
        rd_.I_C.setIdentity(rd_.contact_index * 6, rd_.contact_index * 6);
        rd_.Lambda_c = (rd_.J_C * rd_.A_inv_ * rd_.J_C.transpose()).inverse();
        rd_.J_C_INV_T = rd_.Lambda_c * rd_.J_C * rd_.A_inv_;
        rd_.N_C = MatrixVVd::Identity() - rd_.J_C.transpose() * rd_.J_C_INV_T;
        rd_.W = rd_.A_inv_.bottomRows(MODEL_DOF) * rd_.N_C.rightCols(MODEL_DOF);

        rd_.W_inv = DyrosMath::pinv_COD(rd_.W, rd_.qr_V2);

        if ((rd_.qr_V2.cols() == MODEL_DOF) && (rd_.qr_V2.rows() == (rd_.contact_index * 6 - 6)) && (rd_.contact_index > 1))
        {
            rd_.NwJw = rd_.qr_V2.transpose() * (rd_.J_C_INV_T.rightCols(MODEL_DOF).topRows(6) * rd_.qr_V2.transpose()).inverse();
        }
        else
        {
            if (rd_.contact_index != 1)
                std::cout << "Contact Calculation Error ! " << std::endl;
        }
    }

    VectorQd GravityCompensationTorque(RobotEigenData &rd_)
    {
        rd_.torque_grav = rd_.W_inv * (rd_.A_inv_.bottomRows(MODEL_DOF) * (rd_.N_C * rd_.G_));
        rd_.P_C = rd_.J_C_INV_T * rd_.G_;

        return rd_.torque_grav;
    }

    VectorQd ContactForceFrictionConeConstraintTorque(RobotEigenData &rd_, Eigen::VectorQd command_torque)
    {
        //--- Cost
        Eigen::MatrixQQd H;
        Eigen::VectorQd g;

        H.setIdentity();
        g.setZero();
        g = (-1.0) * command_torque;

        //--- Constraints
        Eigen::MatrixXd A;
        Eigen::VectorXd lbA, ubA;

        unsigned int friction_cone_constraint_size = 17;
        unsigned int total_variable_size = MODEL_DOF;
        unsigned int total_constraint_size =  rd_.contact_index * friction_cone_constraint_size;
        unsigned int contact_dof = rd_.contact_index * 6;

        A.setZero(total_constraint_size, total_variable_size);
        lbA.setZero(total_constraint_size);
        ubA.setZero(total_constraint_size);

        Eigen::MatrixXd total_force_const_matrix;
        total_force_const_matrix.setZero(total_constraint_size, contact_dof);

        for (int i = 0; i < rd_.contact_index; i++)
        {
            total_force_const_matrix.block(i * friction_cone_constraint_size, i * 6, friction_cone_constraint_size, 6) = rd_.ee_[rd_.ee_idx[i]].GetFrictionConeConstrainMatrix();
        }

        A   = total_force_const_matrix * rd_.J_C_INV_T.rightCols(total_variable_size);
        lbA = total_force_const_matrix * rd_.P_C;
        ubA.segment(0, total_constraint_size).setConstant(OsqpEigen::INFTY);

        //--- Quadratic Programming
        static CQuadraticProgram qp_torque_contact_;
        static bool firstCalcQp = false;
        if(!firstCalcQp)
        {
            qp_torque_contact_.InitializeProblemSize(total_variable_size, total_constraint_size);
            qp_torque_contact_.setWarmStartOption();
        
            // qp_torque_contact_.PrintHessGrad();
            // qp_torque_contact_.PrintSubjectToAx();
            // qp_torque_contact_.PrintSubjectTox();

            firstCalcQp = true;
        }

        qp_torque_contact_.UpdateMinProblem(H, g);
        qp_torque_contact_.UpdateSubjectToAx(A, lbA, ubA);

        Eigen::VectorXd torque_qp;
        Eigen::VectorQd torque_safety;

        if (qp_torque_contact_.solveQP(1000, torque_qp))
        {
            torque_safety = torque_qp.segment(0, MODEL_DOF);
            return torque_safety;
        }
        else
        {
            std::cout << "===========QP SOLVE FAILED===========" << std::endl;
            return command_torque;
        }
    }

    void NullspaceInverseKinematics(RobotEigenData& rd_)
    {
        Eigen::MatrixVVd Ni = Eigen::MatrixVVd::Identity() - DyrosMath::pinv_SVD(rd_.J_C) * rd_.J_C;
        Eigen::MatrixXd J_pre = rd_.J_task * Ni;
        Eigen::MatrixXd J_pinv = DyrosMath::pinv_SVD(J_pre);
        Eigen::VectorVQd q_delta = J_pinv * rd_.e_task;

        rd_.q_desired = rd_.q_ + q_delta.tail(MODEL_DOF);
    }

    void FrictionCompensationTorques(RobotEigenData& rd_, const Eigen::VectorQd &q_dot)
    {
        double alpha_fric = 0.95;
        Eigen::VectorQd torque_coulomb;
        Eigen::VectorQd torque_viscous;

        for(int i = 0; i < MODEL_DOF; i++){
            torque_coulomb(i) = rd_.tau_coulomb[i] * tanh(alpha_fric * q_dot(i)); 
            torque_viscous(i) = rd_.tau_viscous[i] * tanh(alpha_fric * q_dot(i)) * sqrt(abs(q_dot(i))); 
        }

        rd_.torque_fric = torque_coulomb + torque_viscous;
    }

    VectorQd JointTorqueToMotorTorque(RobotEigenData& rd_, const Eigen::VectorQd& torque_joint)
    {
        VectorQd torque_motor = rd_.four_bar_Jaco_.transpose() * torque_joint;

        return torque_motor;
    }

    VectorQd JointPositionToMotorTorque(RobotEigenData& rd_)
    {
        VectorQd q_desired_motor; q_desired_motor.setZero(); 
        VectorQd torque_motor; torque_motor.setZero();

        static FourBarKinematics four_bar_kinematics_;
        four_bar_kinematics_.Joint2MotorDesiredPos(rd_.q_desired, q_desired_motor);
        for (int i = 0; i < MODEL_DOF; i++) {
            torque_motor[i] = rd_.Kp_m[i] * (q_desired_motor[i] - rd_.q_motor_[i]) + rd_.Kd_m[i] * (0.0 - rd_.q_dot_motor_[i]);
        }

        return torque_motor;
    }

    void loadActuatorNetModels()
    {
        if (g_models_ready) return;
        try {
            std::string p = "/home/dyros/ros2_ws/src/p73_walker_controller/p73_lib/src/actuatornet_models/";
            std::vector<std::string> model_names = {
                "p73_lstm_left_hip_roll",   "p73_lstm_left_hip_pitch",  "p73_lstm_left_hip_yaw",
                "p73_lstm_left_knee_pitch", "p73_lstm_left_ankle_pitch","p73_lstm_left_ankle_roll",
                "p73_lstm_right_hip_roll",  "p73_lstm_right_hip_pitch", "p73_lstm_right_hip_yaw",
                "p73_lstm_right_knee_pitch","p73_lstm_right_ankle_pitch","p73_lstm_right_ankle_roll"
            };
            g_session_options.SetIntraOpNumThreads(1);
            g_session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
            for (auto& name : model_names) {
                g_sessions.emplace_back(g_ort_env, (p + name + ".onnx").c_str(), g_session_options);
            }
            g_models_ready = true;
            std::cout << "All ActuatorNet Models Loaded Successfully!" << std::endl;
        } catch (const Ort::Exception& e) {
            std::cerr << "Error loading ActuatorNet models: " << e.what() << std::endl;
        }
    }

    Vector12d inferActuatorTorqueFromNet(RobotEigenData& rd_, double elapsed_time)
    {
        if (!g_models_ready)
        {
            std::cerr << "ActuatorNet: models not loaded. Call loadActuatorNetModels() before inference." << std::endl;
            return Eigen::Vector12d::Zero();
        }

        // LSTM was trained with h=0, c=0 reset for every independent sample
        // (shuffled DataLoader, seq_len=1, state=None each forward pass).
        // Carrying state between steps would be distribution shift → reset each call.
        const int HIDDEN_SIZE = 32;
        const int NUM_LAYERS  = 3;
        const int HC_SIZE     = NUM_LAYERS * HIDDEN_SIZE; // 96 floats per joint

        std::array<float, HC_SIZE> h_zero; h_zero.fill(0.0f);
        std::array<float, HC_SIZE> c_zero; c_zero.fill(0.0f);

        Eigen::Vector12d inferred_torque = Eigen::Vector12d::Zero();

        Ort::MemoryInfo mem_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
        std::array<int64_t, 3> input_shape{1, 1, 2};
        std::array<int64_t, 3> hc_shape{NUM_LAYERS, 1, HIDDEN_SIZE};

        const char* input_names[]  = {"input", "h_in", "c_in"};
        const char* output_names[] = {"output", "h_out", "c_out"};

        for (int i = 0; i < 12; ++i) {
            std::array<float, 2> input_data = {
                (float)(rd_.q_desired(i) - rd_.q_(i)),
                (float)rd_.q_dot_(i)
            };

            std::vector<Ort::Value> input_tensors;
            input_tensors.reserve(3);
            input_tensors.push_back(Ort::Value::CreateTensor<float>(
                mem_info, input_data.data(), 2, input_shape.data(), 3));
            input_tensors.push_back(Ort::Value::CreateTensor<float>(
                mem_info, h_zero.data(), HC_SIZE, hc_shape.data(), 3));
            input_tensors.push_back(Ort::Value::CreateTensor<float>(
                mem_info, c_zero.data(), HC_SIZE, hc_shape.data(), 3));

            try {
                auto output_tensors = g_sessions[i].Run(
                    Ort::RunOptions{nullptr}, input_names, input_tensors.data(), 3, output_names, 3);

                float* out_data = output_tensors[0].GetTensorMutableData<float>();
                inferred_torque(i) = out_data[0] / 0.01f;
            } catch (const Ort::Exception& e) {
                std::cerr << "ActuatorNet LSTM inference error joint " << i << ": " << e.what() << std::endl;
            }
        }
        rd_.torque_actuatornet_ = inferred_torque;

        return rd_.torque_actuatornet_;
    }

}
