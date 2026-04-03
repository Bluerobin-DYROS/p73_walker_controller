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
            std::string p = "/home/bluerobin/ros2_ws/src/p73_walker_controller/p73_lib/src/actuatornet_models/";
            std::vector<std::string> model_names = {
                "p73_left_hip_roll",  "p73_left_hip_pitch", "p73_left_hip_yaw",
                "p73_left_knee_pitch","p73_left_ankle",
                "p73_right_hip_roll", "p73_right_hip_pitch","p73_right_hip_yaw",
                "p73_right_knee_pitch","p73_right_ankle"
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
        //--- Description:
        // rd_.q_ : joint position
        // rd_.q_desired : joint position desired
        // rd_.q_dot_ : joint velocity
        // rd_.q_torque_ : joint torque
        
        //--- ToDo:
        // 1. load 10 Networks (pt or onnx)
        // 2. load history vector (joint position, velocity, torque)
        // 2. infer torque from network.
        // 3. return inferred torque (rd_.torque_actuatornet_).
        // 

        if (!g_models_ready)
        {
            std::cerr << "ActuatorNet: models not loaded. Call loadActuatorNetModels() before inference." << std::endl;
            return Eigen::Vector12d::Zero();
        }

        const int HISTORY_SIZE = 21;
        static Eigen::MatrixXd e_hist      = Eigen::MatrixXd::Zero(12, HISTORY_SIZE);
        static Eigen::MatrixXd v_hist      = Eigen::MatrixXd::Zero(12, HISTORY_SIZE);
        static Eigen::MatrixXd e_hist_prev = Eigen::MatrixXd::Zero(12, HISTORY_SIZE);
        static Eigen::MatrixXd v_hist_prev = Eigen::MatrixXd::Zero(12, HISTORY_SIZE);

        e_hist_prev = e_hist;
        v_hist_prev = v_hist;

        e_hist.leftCols(HISTORY_SIZE - 1) = e_hist_prev.rightCols(HISTORY_SIZE - 1);
        v_hist.leftCols(HISTORY_SIZE - 1) = v_hist_prev.rightCols(HISTORY_SIZE - 1);
        e_hist.col(HISTORY_SIZE - 1) = (rd_.q_desired - rd_.q_).head(12);
        v_hist.col(HISTORY_SIZE - 1) = rd_.q_dot_.head(12);

        int idx_t0 = HISTORY_SIZE - 1;   // t      = col 20
        int idx_t1 = HISTORY_SIZE - 11;  // t-10   = col 10
        int idx_t2 = 0;                  // t-20   = col 0

        std::vector<std::vector<int>> joint_groups = {
            {0}, {1}, {2}, {3}, {4,5},
            {6}, {7}, {8}, {9}, {10,11}
        };
        Eigen::Vector12d inferred_torque = Eigen::Vector12d::Zero();

        Ort::AllocatorWithDefaultOptions allocator;
        for (int i = 0; i < 10; ++i) {
            int num_joints  = joint_groups[i].size();
            int feature_size = num_joints * 6;

            std::vector<float> features(feature_size);
            int f_idx = 0;
            for (int j : joint_groups[i]) {
                features[f_idx++] = e_hist(j, idx_t0);
                features[f_idx++] = e_hist(j, idx_t1);
                features[f_idx++] = e_hist(j, idx_t2);
                features[f_idx++] = v_hist(j, idx_t0);
                features[f_idx++] = v_hist(j, idx_t1);
                features[f_idx++] = v_hist(j, idx_t2);
            }

            std::array<int64_t, 2> input_shape{1, feature_size};
            Ort::MemoryInfo mem_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
            Ort::Value input_tensor  = Ort::Value::CreateTensor<float>(
                mem_info, features.data(), feature_size, input_shape.data(), input_shape.size());

            auto input_name  = g_sessions[i].GetInputNameAllocated(0, allocator);
            auto output_name = g_sessions[i].GetOutputNameAllocated(0, allocator);
            const char* input_names[]  = {input_name.get()};
            const char* output_names[] = {output_name.get()};

            try {
                auto output_tensors = g_sessions[i].Run(
                    Ort::RunOptions{nullptr}, input_names, &input_tensor, 1, output_names, 1);
                float* out_data = output_tensors[0].GetTensorMutableData<float>();
                for (int j = 0; j < num_joints; ++j) {
                    inferred_torque(joint_groups[i][j]) = out_data[j] / 0.01f;
                }
            } catch (const Ort::Exception& e) {
                std::cerr << "Error during inference " << i << ": " << e.what() << std::endl;
            }
        }
        rd_.torque_actuatornet_ = inferred_torque;

        return rd_.torque_actuatornet_;

    }

}
