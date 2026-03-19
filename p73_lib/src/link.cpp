#include <p73_lib/link.h>

//--- LinkData Class ---///
void LinkData::Initialize(pinocchio::Model& model_, pinocchio::FrameIndex id_)
{
    id = id_;

    pinocchio::JointIndex jid = model_.frames[id].parent;

    mass = model_.inertias[jid].mass();
    com_position = model_.inertias[jid].lever();
    inertia = model_.inertias[jid].inertia();
}

void LinkData::UpdatePosition(pinocchio::Model &model_, pinocchio::Data &data_)
{
    const pinocchio::SE3 &oMf = data_.oMf[id]; 

    xpos = oMf.translation();
    rotm = oMf.rotation();

    DyrosMath::rot2Euler_tf2(rotm, roll, pitch, yaw);
}

void LinkData::UpdateVW(pinocchio::Model &model_, pinocchio::Data &data_)
{
    const pinocchio::Motion v_frame_world = pinocchio::getFrameVelocity(model_, data_, id, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED);

    v = v_frame_world.linear();    
    w = v_frame_world.angular();   
}

void LinkData::UpdateJacobian(pinocchio::Model &model_, pinocchio::Data &data_)
{
    j_temp.setZero(6, MODEL_DOF_VIRTUAL);

    pinocchio::getFrameJacobian(model_, data_, id, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, j_temp);

    jac = j_temp.block(0, 0, 6, MODEL_DOF_VIRTUAL);

    const pinocchio::Motion v_frame_world = pinocchio::getFrameVelocity(model_, data_, id, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED);

    v = v_frame_world.linear();
    w = v_frame_world.angular();   
}

void LinkData::SetTrajectoryQuintic(double current_time, double start_time, double end_time)
{
    for (int j = 0; j < 3; j++)
    {
        Eigen::Vector3d quintic = DyrosMath::QuinticSpline(current_time, start_time, end_time, x_init(j), v_init(j), 0, x_desired(j), 0, 0);
        x_traj(j) = quintic(0);
        v_traj(j) = quintic(1);
        a_traj(j) = quintic(2);
    }

    r_traj = rot_init;
    w_traj = Eigen::Vector3d::Zero();
}

void LinkData::SetTrajectoryQuintic(double current_time, double start_time, double end_time, Eigen::Vector3d pos_desired)
{
    for (int j = 0; j < 3; j++)
    {
        Eigen::Vector3d quintic = DyrosMath::QuinticSpline(current_time, start_time, end_time, x_init(j), v_init(j), 0, pos_desired(j), 0, 0);
        x_traj(j) = quintic(0);
        v_traj(j) = quintic(1);
        a_traj(j) = quintic(2);
    }

    r_traj = rot_init;
    w_traj = Eigen::Vector3d::Zero();
}

void LinkData::SetTrajectoryRotation(double current_time, double start_time, double end_time)
{
    Eigen::Quaterniond q0(rot_init);
    Eigen::Quaterniond q1(rot_desired);

    Eigen::Vector3d qs_ = DyrosMath::QuinticSpline(current_time, start_time, end_time, 0, 0, 0, 1, 0, 0);

    Eigen::Quaterniond q_traj = q0.slerp(qs_(0), q1);

    r_traj = q_traj.toRotationMatrix();

    Eigen::AngleAxisd axd(q1 * q0.inverse());

    w_traj = axd.angle() * qs_(1) * axd.axis();
    ra_traj = axd.angle() * qs_(2) * axd.axis();
}


void LinkData::SetTrajectoryRotation(double current_time, double start_time, double end_time, Eigen::Matrix3d rot_desired_)
{
    Eigen::Vector3d axis;
    double angle;

    Eigen::AngleAxisd aa(rot_init.transpose() * rot_desired_);
    axis = aa.axis();
    angle = aa.angle();

    double c_a = DyrosMath::cubic(current_time, start_time, end_time, 0.0, angle, 0.0, 0.0);
    Eigen::Matrix3d rmat;
    rmat = Eigen::AngleAxisd(c_a, axis);

    r_traj = rot_init * rmat;

    double dtime = 0.0001;
    double c_a_dtime = DyrosMath::cubic(current_time + dtime, start_time, end_time, 0.0, angle, 0.0, 0.0);

    Eigen::Vector3d ea = r_traj.eulerAngles(0, 1, 2);

    Eigen::Vector3d ea_dtime = (rot_init * Eigen::AngleAxisd(c_a_dtime, axis)).eulerAngles(0, 1, 2);

    w_traj = (ea_dtime - ea) / dtime;
}


void LinkData::SetTrajectoryQuintic(double current_time, double start_time, double end_time, Eigen::Vector3d pos_init, Eigen::Vector3d pos_desired)
{
    for (int j = 0; j < 3; j++)
    {
        Eigen::Vector3d quintic = DyrosMath::QuinticSpline(current_time, start_time, end_time, pos_init(j), 0, 0, pos_desired(j), 0, 0);
        x_traj(j) = quintic(0);
        v_traj(j) = quintic(1);
        a_traj(j) = quintic(2);
    }

    r_traj = rot_init;
    w_traj = Eigen::Vector3d::Zero();
}

void LinkData::SetInitialWithPosition()
{
    x_init = xpos;
    v_init = v;
    rot_init = rotm;
    DyrosMath::rot2Euler_tf2(rotm, roll_init, pitch_init, yaw_init);
}


//--- EndEffector Class ---///
void EndEffector::InitializeEE(LinkData &lk_, double cs_x_length_, double cs_y_length_, double contact_force_minimum_, double friction_coeff_)
{
    id = lk_.id;
    mass = lk_.mass;
    com_position = lk_.com_position;
    rotm.setZero();
    inertia.setZero();

    inertia = lk_.inertia;
    jac.setZero();

    cs_x_length = cs_x_length_;
    cs_y_length = cs_y_length_;
    contact_force_minimum = contact_force_minimum_;
    friction_coeff = friction_coeff_;
}

void EndEffector::SetContact(pinocchio::Model &model_, pinocchio::Data &data_, Eigen::VectorQVQd &q_virtual_)
{
    // pinocchio::forwardKinematics(model_, data_, q_virtual_);
    // pinocchio::computeJointJacobians(model_, data_, q_virtual_);
    // pinocchio::updateFramePlacements(model_, data_);

    const pinocchio::SE3 &oMf = data_.oMf[id];
    xpos_contact = oMf.translation() + oMf.rotation() * contact_point;

    const pinocchio::JointIndex joint_id = model_.frames[id].parent;
    const pinocchio::SE3 frame_on_joint = model_.frames[id].placement;
    const pinocchio::SE3 joint_contact_placement = frame_on_joint * pinocchio::SE3(Eigen::Matrix3d::Identity(), contact_point);

    pinocchio::RigidConstraintModel cmodel(pinocchio::CONTACT_6D,
                                           model_,
                                           joint_id,
                                           joint_contact_placement,
                                           pinocchio::ReferenceFrame::LOCAL);
    pinocchio::RigidConstraintData cdata(cmodel);

    j_temp.setZero(6, MODEL_DOF_VIRTUAL);
    pinocchio::getConstraintJacobian(model_, data_, cmodel, cdata, j_temp);

    jac_contact = j_temp;
}


void EndEffector::UpdateLinkData(LinkData &lk_)
{
    xpos = lk_.xpos;

    v = lk_.v;
    w = lk_.w;

    rotm = lk_.rotm;

    roll = lk_.roll;
    pitch = lk_.pitch;
    yaw = lk_.yaw;

    memcpy(&jac, &lk_.jac, sizeof(Matrix6Vf));
}

Eigen::MatrixXd EndEffector::GetFrictionConeConstrainMatrix()
{
    Eigen::MatrixXd force_const_matrix; 
    force_const_matrix.setZero(17, 6);

    double mu = friction_coeff;
    double X = cs_x_length / 2.0; 
    double Y = cs_y_length / 2.0; 
    force_const_matrix <<  0,  0,            -1,   0,   0,  0,
                          -1,  0,           -mu,   0,   0,  0,
                          +1,  0,           -mu,   0,   0,  0,
                           0, -1,           -mu,   0,   0,  0,
                           0, +1,           -mu,   0,   0,  0,
                           0,  0,            -Y,  -1,   0,  0,
                           0,  0,            -Y,  +1,   0,  0,
                           0,  0,            -X,   0,  -1,  0,
                           0,  0,            -X,   0,  +1,  0,
                          -Y, -X, -(X + Y) * mu, -mu, +mu, -1,
                          +Y, +X, -(X + Y) * mu, +mu, -mu, -1,
                          +Y, -X, -(X + Y) * mu, +mu, +mu, -1,
                          +Y, +X, -(X + Y) * mu, +mu, +mu, -1,
                          +Y, -X, -(X + Y) * mu, +mu, +mu, +1,
                          +Y, +X, -(X + Y) * mu, +mu, -mu, +1,
                          -Y, -X, -(X + Y) * mu, -mu, -mu, +1,
                          -Y, +X, -(X + Y) * mu, -mu, +mu, +1;

    return force_const_matrix;
}