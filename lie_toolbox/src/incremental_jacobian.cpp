#include "lie_toolbox/incremental_jacobian.hpp"
#include <cstdlib>

using namespace liegroup;

IncrementalJacobian::IncrementalJacobian(int numjoint, int delta_time)
{
    numjoint_ = numjoint;
    delta_time_ = delta_time;
    bodies.resize(numjoint_ + 1);
    // FloatingBase : J = (6,6+n)
    // JointGroundedBase : J = (6,n_0+n) (mobile : x,y,theta,+n)
    // FixedGroundedBase : J = (6,6+n)
    bodies[0].jacobian_matrix = Eigen::MatrixXd::Zero(6, numjoint_);
    bodies[0].derivative_jacobian_matrix = Eigen::MatrixXd::Zero(6, numjoint_);
    bodies[0].current_body_velocity = Eigen::MatrixXd::Zero(6, 1);
    bodies[0].current_body_acceleration = Eigen::MatrixXd::Zero(6, 1);
    bodies[0].select_joint_number_matrix = Eigen::MatrixXd::Zero(numjoint_, 1);
    bodies[0].theta = 0;
    bodies[0].theta_dot = 0;
    bodies[0].prev_theta_dot = 0;

    for (int i = 1; i <= numjoint_; ++i)
    {

        bodies[i].jacobian_matrix = Eigen::MatrixXd::Zero(6, numjoint_); // Adjust size as necessary
        bodies[i].derivative_jacobian_matrix = Eigen::MatrixXd::Zero(6, numjoint_);
        bodies[i].select_joint_number_matrix = Eigen::MatrixXd::Zero(numjoint_, 1);

        bodies[i].theta = 0;
        bodies[i].theta_dot = 0;
        bodies[i].prev_theta_dot = 0;
        bodies[i].between_twist_prev_body = Eigen::MatrixXd::Zero(6, 1);
        bodies[i].joint_matrix_e = Eigen::MatrixXd::Zero(6, 1); // Adjust based on joint type
        // bodies[i].derivative_joint_matrix_e = Eigen::MatrixXd::Zero(6, 1);

        bodies[i].current_body_velocity = Eigen::MatrixXd::Zero(6, 1);
        bodies[i].current_body_acceleration = Eigen::MatrixXd::Zero(6, 1);
    }
}
IncrementalJacobian::~IncrementalJacobian()
{
}

void IncrementalJacobian::PropagateKinematics()
{
    Eigen::VectorXd twist_adjoint;
    Eigen::VectorXd twist_adjoint_dot;
    twist_adjoint.resize(numjoint_);
    twist_adjoint.setZero();
    twist_adjoint_dot.resize(numjoint_);
    twist_adjoint_dot.setZero();
    static double dt = 1 / double(delta_time_);
    // (numjoint_,1) 크기를 가지고 각 관절의 가속도를 저장
    for (int j = 1; j <= numjoint_; ++j)
    {
        bodies[j].theta_ddot = (bodies[j].theta_dot - bodies[j].prev_theta_dot) / dt;
        twist_adjoint_dot += bodies[j].select_joint_number_matrix * bodies[j].theta_ddot;
    }
    // (numjoint_,1) 크기를 가지고 각 관절의 속도를 저장
    for (int j = 1; j <= numjoint_; ++j)
    {
        bodies[j].prev_theta_dot = bodies[j].theta_dot;
        twist_adjoint += bodies[j].select_joint_number_matrix * bodies[j].theta_dot;
    }
    for (int i = 1; i <= numjoint_; ++i)
    {
        // Update Jacobian and its derivative for the current body
        bodies[i].jacobian_matrix = LieAlgebra::InverseAdjoint(bodies[i].adjacent_transfom_matrix) * bodies[i - 1].jacobian_matrix + (bodies[i].joint_matrix_e * bodies[i].select_joint_number_matrix.transpose());
        bodies[i].between_twist_prev_body = bodies[i].joint_matrix_e * bodies[i].theta_dot;
        bodies[i].derivative_jacobian_matrix = (LieAlgebra::InverseAdjoint(bodies[i].adjacent_transfom_matrix) * bodies[i - 1].derivative_jacobian_matrix) -
                                               (LieAlgebra::AdOperator(bodies[i].between_twist_prev_body) * LieAlgebra::InverseAdjoint(bodies[i].adjacent_transfom_matrix) *
                                                bodies[i - 1].jacobian_matrix); // + (bodies[i].derivative_joint_matrix_e * bodies[i].select_joint_number_matrix.transpose());

        bodies[i].current_body_velocity = bodies[i].jacobian_matrix * twist_adjoint;
        bodies[i].current_body_acceleration = bodies[i].jacobian_matrix * twist_adjoint_dot + bodies[i].derivative_jacobian_matrix * twist_adjoint;

        // std::cout << std::fixed << std::setprecision(3) << "bodies[i].jacobian_matrix " << i << ":\n"
        //           << bodies[i].jacobian_matrix << "\n";
        // std::cout << std::fixed << std::setprecision(3) << "bodies[i].derivative_jacobian_matrix " << i << ":\n"
        //           << bodies[i].derivative_jacobian_matrix << "\n";

        // std::cout << std::fixed << std::setprecision(3) << "body v " << i << ":\n"
        //           << bodies[i].current_body_velocity.transpose() << "\n";
        // std::cout << std::fixed << std::setprecision(3) << "body v dot " << i << ":\n"
        //           << bodies[i].current_body_acceleration.transpose() << "\n";
    }
}
