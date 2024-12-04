#ifndef INCREMENTAL_JACOBIAN_HPP_
#define INCREMENTAL_JACOBIAN_HPP_

#include "lie_toolbox/lie_algebra.hpp"
#include <vector>
#include <iomanip>

namespace liegroup
{
    class IncrementalJacobian
    {
    private:
        /* data */
    public:
        struct Body
        {
            // Jacobian matrix
            Eigen::MatrixXd jacobian_matrix;
            // Derivative of the Jacobian         
            Eigen::MatrixXd derivative_jacobian_matrix;
            // Twist with prev body      
            Eigen::MatrixXd between_twist_prev_body; 
            // model joint transfom matrix
            Eigen::Matrix4d adjacent_transfom_matrix;
            // Joint Matrix, revolute or prismatic
            Eigen::MatrixXd joint_matrix_e;
            // Selection matrix for the joint number     
            Eigen::MatrixXd select_joint_number_matrix;
            // Selection Derivative revolute or prismatic
            // Eigen::MatrixXd derivative_joint_matrix_e;
            // velocity body
            Eigen::MatrixXd current_body_velocity;
            // acceleration body
            Eigen::MatrixXd current_body_acceleration;
            // Joint angle
            double theta;     
            // Joint velocity
            double theta_dot;     
            // Joint acceleration
            double theta_ddot;     
            // prev Joint velocity
            double prev_theta_dot; 
        };

        std::vector<Body> bodies;
        int numjoint_;
        int delta_time_;
        IncrementalJacobian(int numjoint, int delta_time);
        ~IncrementalJacobian();
        void PropagateKinematics();
    };

}

#endif
