#include "lie_toolbox/incremental_jacobian.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include "std_msgs/msg/float64_multi_array.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include "lie_toolbox/incremental_jacobian.hpp"
#include <iostream>
#include <vector>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
using std::placeholders::_1;
using JointStates = sensor_msgs::msg::JointState;
double delta_time = 8;
std::vector<double> body_masses = {0.62810, 2.07542, 0.83596, 0.92391, 0.32657, 0.41516, 0.19201, 0.50000};

class LeftArm : public rclcpp::Node
{
public:
    std::vector<std::string> joint_names = {
        "l_sh_p", "l_sh_r", "l_sh_y", "l_el_p", "l_wr_y", "l_wr_p", "l_wr_r"};
    // 각 관절 이름에 대한 인덱스를 매핑하는 맵
    std::unordered_map<std::string, int> joint_name_to_number;
    liegroup::IncrementalJacobian jacobianCalculator;

    // 관절의 위치와 속도를 저장하는 배열
    std::vector<double> joint_positions;
    std::vector<double> joint_velocities;

    LeftArm()
        : Node("LeftArm"), jacobianCalculator(8, delta_time)
    {
        rclcpp::QoS qos_profile(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
        qos_profile.keep_last(10);
        qos_profile.reliability(rclcpp::ReliabilityPolicy::Reliable);
        qos_profile.durability(rclcpp::DurabilityPolicy::Volatile);
        joint_subscriber_ = this->create_subscription<JointStates>(
            "/robot/joint_states",
            qos_profile,
            std::bind(&LeftArm::JointMessageCallback, this, _1));

        // 엔드포인트 속도 퍼블리셔
        velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/end_effector_velocity", qos_profile);

        // 엔드포인트 가속도 퍼블리셔
        accelerate_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/end_effector_accelerate", qos_profile);

        // 엔드포인트 힘 퍼블리셔
        force_publisher_ = this->create_publisher<geometry_msgs::msg::Wrench>("/end_effector_force", qos_profile);

        joint_publisher_ = this->create_publisher<JointStates>("/joint_states", qos_profile);

        // timer_ = this->create_wall_timer(
        //     std::chrono::milliseconds(int(1000 / loop_rate)), std::bind(&LeftArm::Calc, this));

        auto node_clock = this->get_clock();
        timer_ = rclcpp::create_timer(
            this,
            node_clock,
            std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double>(4 / 1000)),
            std::bind(&LeftArm::Calc, this));

        joint_positions.resize(joint_names.size());
        joint_velocities.resize(joint_names.size());

        for (size_t i = 0; i < joint_names.size(); ++i)
        {
            joint_positions[i] = 0;
            joint_velocities[i] = 0;
        }

        // // 초기 상태 설정 (위치, 속도, 가속도)

        endpoint_pos = Eigen::MatrixXd::Identity(4, 4);
        endpoint_vel = Eigen::MatrixXd::Zero(6, 1);
        endpoint_acc = Eigen::MatrixXd::Zero(6, 1);
    }

private:
    void JointMessageCallback(const JointStates::SharedPtr msg)
    {
        // 관절 이름 인덱스 초기화
        for (const std::string &joint_name : joint_names)
        {
            auto it = std::find(msg->name.begin(), msg->name.end(), joint_name);
            if (it != msg->name.end())
            {
                int index = std::distance(msg->name.begin(), it);
                joint_name_to_number[joint_name] = index;
            }
        }

        // 관절의 위치와 속도를 배열에 저장
        for (size_t i = 0; i < joint_names.size(); ++i)
        {
            const std::string &joint_name = joint_names[i];
            int index = joint_name_to_number[joint_name];
            joint_positions[i] = msg->position[index];
            joint_velocities[i] = msg->velocity[index];
        }
    }
    Eigen::VectorXd VectorToEigen(const std::vector<double> &vec)
    {
        Eigen::VectorXd eigen_vec(vec.size());
        for (size_t i = 0; i < vec.size(); ++i)
        {
            eigen_vec(i) = vec[i];
        }
        return eigen_vec;
    }
    void Calc()
    {
        Eigen::VectorXd body_e_force(6);
        // double k_spring = 20;
        // double k_damping = 0;
        // Eigen::VectorXd k_spring(6);
        // Eigen::VectorXd k_damping(6);
        // k_spring << 20 , 20 , 20 , 20 , 20, 20;
        // k_damping << 0 , 0 , 0 , 0 , 0, 0;
        static double current_time = 0;
        double time_step = delta_time / 1000;
        std::vector<double_t> position = {0.3, 0, -0.461, 0, 0, 0};
        std::vector<double_t> velocity = {0, 0, 0, 0, 0, 0};
        std::vector<double_t> acceleration = {0, 0, 0, 0, 0, 0};

        double x = position[0];
        double y = position[1];
        double z = position[2];
        double roll = position[3];
        double pitch = position[4];
        double yaw = position[5];

        // 4x4 변환 행렬 생성
        Eigen::Matrix4d positiontransformationMatrix = liegroup::GetTransformationMatrix(x, y, z, roll, pitch, yaw);

        // Eigen::Matrix4d error_matrix = positiontransformationMatrix * endpoint_pos.inverse();

        Eigen::Vector3d e_position;
        e_position = positiontransformationMatrix.block<3, 1>(0, 3) - endpoint_pos.block<3, 1>(0, 3);
        // std::cout << "eeee_position: ";
        // for (double ep : e_position)
        // {
        //     std::cout << std::fixed << std::setprecision(3) << ep << " ";
        // }
        // std::cout << std::endl;
        // // 회전 행렬 오차 계산: R_error = R_target * R_current^(-1)
        // Eigen::Matrix3d e_rotationMatrix = positiontransformationMatrix.block<3, 3>(0, 0) * endpoint_pos.block<3, 3>(0, 0).inverse();
        // 회전 행렬을 쿼터니언으로 변환
        Eigen::Quaterniond q_target(positiontransformationMatrix.block<3, 3>(0, 0)); // 목표 회전 쿼터니언
        Eigen::Quaterniond q_current(endpoint_pos.block<3, 3>(0, 0));                // 현재 회전 쿼터니언

        // 쿼터니언 차이 계산 (q_error = q_target * q_current^(-1))
        Eigen::Quaterniond q_error = q_target * q_current.inverse();

        // 쿼터니언을 회전 행렬로 변환
        Eigen::Matrix3d e_rotationMatrix = q_error.toRotationMatrix();
        Eigen::Vector3d rpy = liegroup::RotationMatrixToRPY(e_rotationMatrix);

        // 위치와 회전 오차를 하나의 벡터로 결합
        Eigen::VectorXd result(6);
        result << e_position, rpy;

        // 현재 시간을 출력
        // std::cout << "Time: " << current_time << "s" << std::endl;

        // // 속도 출력
        // std::cout << "Velocity: ";
        // for (double v : velocity)
        // {
        //     std::cout << v << " ";
        // }
        // std::cout << std::endl;

        // // 가속도 출력
        // std::cout << "Acceleration: ";
        // for (double a : acceleration)
        // {
        //     std::cout << std::fixed << std::setprecision(3) << a << " ";
        // }
        // std::cout << std::endl;

        // 다음 시간으로 넘어감
        Eigen::MatrixXd body_jacobian = CalcJacobian();
        Eigen::MatrixXd body_inertial_matrix = CalcBodyInertialMatrix();
        Eigen::MatrixXd reference_force = Eigen::MatrixXd::Zero(6, 1);
        Eigen::VectorXd gravity_matrix(6);
        Eigen::MatrixXd damping_matrix = Eigen::MatrixXd::Identity(6, 6);
        Eigen::MatrixXd spring_matrix = Eigen::MatrixXd::Identity(6, 6);

        spring_matrix << 1000, 0, 0, 0, 0, 0,
            0, 1000, 0, 0, 0, 0,
            0, 0, 1000, 0, 0, 0,
            0, 0, 0, 20, 0, 0,
            0, 0, 0, 0, 20, 0,
            0, 0, 0, 0, 0, 20;

        damping_matrix << 50, 0, 0, 0, 0, 0,
            0, 50, 0, 0, 0, 0,
            0, 0, 50, 0, 0, 0,
            0, 0, 0, 5, 0, 0,
            0, 0, 0, 0, 5, 0,
            0, 0, 0, 0, 0, 5;
        gravity_matrix << 0, 0, 9.81, 0, 0, 0;
        // std::vector<double>를 Eigen::VectorXd로 변환
        Eigen::VectorXd eigen_position = VectorToEigen(position);
        Eigen::VectorXd eigen_velocity = VectorToEigen(velocity);
        Eigen::VectorXd eigen_acceleration = VectorToEigen(acceleration);
        // body_e_force = body_inertial_matrix * (eigen_acceleration - endpoint_acc) +
        //                k_damping * damping_matrix * (eigen_velocity - endpoint_vel) +
        //                k_spring * spring_matrix * (result);

        // std::cout << "eeee_position: ";
        // for (double ep : e_position)
        // {
        //     std::cout << std::fixed << std::setprecision(3) << ep << " ";
        // }

        // std::cout << std::fixed << std::setprecision(3) << body_inertial_matrix << std::endl;
        // std::cout << std::endl;
        body_e_force = body_inertial_matrix * (eigen_acceleration - endpoint_acc) +
                       damping_matrix * (eigen_velocity - endpoint_vel) +
                       spring_matrix * (result) +
                       (body_inertial_matrix * liegroup::LieAlgebra::AdOperator(endpoint_vel) + liegroup::LieAlgebra::CoAdOperator(endpoint_vel) * body_inertial_matrix) * endpoint_vel;
        // 위치 출력
        // std::cout << "Position: ";
        // for (double p : position)
        // {
        //     std::cout << std::fixed << std::setprecision(3) << p << " ";
        // }
        // std::cout << std::endl;

        Eigen::Vector3d curr_position;
        curr_position << endpoint_pos(0, 3), endpoint_pos(1, 3), endpoint_pos(2, 3);

        // 회전 행렬 추출 (상위 3x3 부분)
        Eigen::Matrix3d curr_rotationMatrix = endpoint_pos.block<3, 3>(0, 0);

        // roll, pitch, yaw 추출
        Eigen::Vector3d curr_rpy = liegroup::RotationMatrixToRPY(curr_rotationMatrix);
        std::vector<double_t> curr_position_v = {curr_position[0], curr_position[1], curr_position[2], curr_rpy[0], curr_rpy[1], curr_rpy[2]};
        // 위치 출력
        // std::cout << "curr Position: ";
        // for (double ep : curr_position_v)
        // {
        //     std::cout << std::fixed << std::setprecision(3) << ep << " ";
        // }
        // std::cout << std::endl;

        std::vector<double> jointp = {joint_positions[0], joint_positions[1], joint_positions[2], joint_positions[3],
                                      joint_positions[4], joint_positions[5], joint_positions[6]};


        Eigen::MatrixXd jacobian_transpose = body_jacobian.transpose();
        Eigen::VectorXd joint_torques = jacobian_transpose * body_e_force;

        Eigen::VectorXd l_gravity_torque = Eigen::VectorXd::Zero(body_jacobian.cols());
        for (size_t i = 0; i < body_masses.size(); ++i)
        {
            l_gravity_torque += body_masses[i] * body_jacobian.transpose() * gravity_matrix;
        }
        joint_torques += l_gravity_torque;
        // std::cout << "joint_torques: ";
        // for (double tq : joint_torques)
        // {
        //     std::cout << std::fixed << std::setprecision(3) << tq << " ";
        // }
        // std::cout << std::endl;

        current_time += time_step;
        // 한 줄 띄기
        // std::cout << "--------------------------------" << std::endl;
        auto joint_state_msg = sensor_msgs::msg::JointState();
        joint_state_msg.header.stamp = this->now();
        int q = 0;
        for (const auto &joint_name : joint_names)
        {
            joint_state_msg.name.push_back(joint_name);
            double tq = joint_torques(q);
            joint_state_msg.position.push_back(tq);
            q++;
        }
        joint_publisher_->publish(joint_state_msg);

        // Eigen::VectorXd body_e_force(6);

        // body_e_force = body_inertial_matrix_all * jacobianCalculator.bodies[8].curr_V_dot + (body_inertial_matrix_all * heroehs::LieAlgebra::ad(jacobianCalculator.bodies[8].curr_V) + heroehs::LieAlgebra::Co_ad(jacobianCalculator.bodies[8].curr_V) *
        //                                                                                                                                                                                    body_inertial_matrix_all) *
        //                                                                                         jacobianCalculator.bodies[8].curr_V;

        // std::cout << std::fixed << std::setprecision(3) << "body 8 force : \n"
        //           << body_8_force << "\n";

        // 엔드포인트 속도 퍼블리시
        // geometry_msgs::msg::Twist velocity_msg;
        // velocity_msg.linear.x = jacobianCalculator.bodies[8].curr_V(0);
        // velocity_msg.linear.y = jacobianCalculator.bodies[8].curr_V(1);
        // velocity_msg.linear.z = jacobianCalculator.bodies[8].curr_V(2);
        // velocity_msg.angular.x = jacobianCalculator.bodies[8].curr_V(3);
        // velocity_msg.angular.y = jacobianCalculator.bodies[8].curr_V(4);
        // velocity_msg.angular.z = jacobianCalculator.bodies[8].curr_V(5);
        // velocity_publisher_->publish(velocity_msg);

        // geometry_msgs::msg::Twist accelerate_msg;
        // accelerate_msg.linear.x = jacobianCalculator.bodies[8].curr_V_dot(0);
        // accelerate_msg.linear.y = jacobianCalculator.bodies[8].curr_V_dot(1);
        // accelerate_msg.linear.z = jacobianCalculator.bodies[8].curr_V_dot(2);
        // accelerate_msg.angular.x = jacobianCalculator.bodies[8].curr_V_dot(3);
        // accelerate_msg.angular.y = jacobianCalculator.bodies[8].curr_V_dot(4);
        // accelerate_msg.angular.z = jacobianCalculator.bodies[8].curr_V_dot(5);
        // accelerate_publisher_->publish(accelerate_msg);

        // // 엔드포인트 힘 퍼블리시
        // geometry_msgs::msg::Wrench force_msg;
        // force_msg.force.x = body_e_force(0);
        // force_msg.force.y = body_e_force(1);
        // force_msg.force.z = body_e_force(2);
        // force_msg.torque.x = body_e_force(3);
        // force_msg.torque.y = body_e_force(4);
        // force_msg.torque.z = body_e_force(5);
        // force_publisher_->publish(force_msg);
    }

    Eigen::MatrixXd CalcJacobian()
    {
        Eigen::Matrix4d T_a = Eigen::MatrixXd::Identity(4, 4);
        H.clear();
        // 0.291, 0.238, 0.265, 0.245, 0.165
        Eigen::Matrix4d offset_joint_0_T_1;
        Eigen::Matrix4d offset_joint_1_T_2;
        Eigen::Matrix4d offset_joint_2_T_3;
        Eigen::Matrix4d offset_joint_3_T_4;
        Eigen::Matrix4d offset_joint_4_T_5;
        Eigen::Matrix4d offset_joint_5_T_6;
        Eigen::Matrix4d offset_joint_6_T_7;
        Eigen::Matrix4d offset_joint_7_T_8;

        offset_joint_0_T_1 << 1, 0, 0, 0,
            0, 0, 1, 0,
            0, -1, 0, 0,
            0, 0, 0, 1;

        offset_joint_1_T_2 << 0, 0, 1, 0,
            0, -1, 0, 0,
            1, 0, 0, 0,
            0, 0, 0, 1;

        offset_joint_2_T_3 << 0, 1, 0, 0,
            0, 0, 1, 0,
            1, 0, 0, 0,
            0, 0, 0, 1;

        offset_joint_3_T_4 << 1, 0, 0, 0,
            0, 0, 1, 0,
            0, -1, 0, -0.265,
            0, 0, 0, 1;

        offset_joint_4_T_5 << 1, 0, 0, 0,
            0, 0, -1, 0,
            0, 1, 0, 0,
            0, 0, 0, 1;

        offset_joint_5_T_6 << 1, 0, 0, 0,
            0, 0, 1, 0,
            0, -1, 0, -0.245,
            0, 0, 0, 1;

        offset_joint_6_T_7 << 0, 0, 1, 0,
            0, -1, 0, 0,
            1, 0, 0, 0,
            0, 0, 0, 1;

        offset_joint_7_T_8 << 0, 1, 0, 0,
            0, 0, 1, -0.165,
            1, 0, 0, 0,
            0, 0, 0, 1;

        Eigen::Matrix4d joint_T_1;
        Eigen::Matrix4d joint_T_2;
        Eigen::Matrix4d joint_T_3;
        Eigen::Matrix4d joint_T_4;
        Eigen::Matrix4d joint_T_5;
        Eigen::Matrix4d joint_T_6;
        Eigen::Matrix4d joint_T_7;
        Eigen::Matrix4d joint_T_8;

        joint_T_1 << cos(joint_positions[0]), -sin(joint_positions[0]), 0, 0,
            sin(joint_positions[0]), cos(joint_positions[0]), 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1; // sh_p

        joint_T_2 << cos(joint_positions[1]), -sin(joint_positions[1]), 0, 0,
            sin(joint_positions[1]), cos(joint_positions[1]), 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1; // sh_r
        joint_T_3 << cos(joint_positions[2]), -sin(joint_positions[2]), 0, 0,
            sin(joint_positions[2]), cos(joint_positions[2]), 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1; // sh_y
        joint_T_4 << cos(joint_positions[3]), -sin(joint_positions[3]), 0, 0,
            sin(joint_positions[3]), cos(joint_positions[3]), 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1; // el_p

        joint_T_5 << cos(joint_positions[4]), -sin(joint_positions[4]), 0, 0,
            sin(joint_positions[4]), cos(joint_positions[4]), 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1; // wr_y

        joint_T_6 << cos(joint_positions[5]), -sin(joint_positions[5]), 0, 0,
            sin(joint_positions[5]), cos(joint_positions[5]), 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1; // wr_p

        joint_T_7 << cos(joint_positions[6]), -sin(joint_positions[6]), 0, 0,
            sin(joint_positions[6]), cos(joint_positions[6]), 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1; // wr_r

        // joint_T_8 = Eigen::MatrixXd::Identity(4, 4);

        Eigen::Matrix4d adjacent_T_1 = offset_joint_0_T_1 * joint_T_1;
        Eigen::Matrix4d adjacent_T_2 = offset_joint_1_T_2 * joint_T_2;
        Eigen::Matrix4d adjacent_T_3 = offset_joint_2_T_3 * joint_T_3;
        Eigen::Matrix4d adjacent_T_4 = offset_joint_3_T_4 * joint_T_4;
        Eigen::Matrix4d adjacent_T_5 = offset_joint_4_T_5 * joint_T_5;
        Eigen::Matrix4d adjacent_T_6 = offset_joint_5_T_6 * joint_T_6;
        Eigen::Matrix4d adjacent_T_7 = offset_joint_6_T_7 * joint_T_7;
        Eigen::Matrix4d adjacent_T_8 = offset_joint_7_T_8;

        H.push_back(adjacent_T_1);
        H.push_back(adjacent_T_2);
        H.push_back(adjacent_T_3);
        H.push_back(adjacent_T_4);
        H.push_back(adjacent_T_5);
        H.push_back(adjacent_T_6);
        H.push_back(adjacent_T_7);
        H.push_back(adjacent_T_8);

        T_a = T_a * adjacent_T_1 * adjacent_T_2 * adjacent_T_3 * adjacent_T_4 * adjacent_T_5 * adjacent_T_6 * adjacent_T_7 * adjacent_T_8;

        int num_joint = jacobianCalculator.numjoint_; // Example number of bodies
        for (int i = 1; i <= num_joint; i++)
        {
            jacobianCalculator.bodies[i].adjacent_transfom_matrix = H[i - 1];
            jacobianCalculator.bodies[i].joint_matrix_e << 0, 0, 0, 0, 0, 1; // revolute joint
                                                                             // jacobianCalculator.bodies[i].E << 0, 0, 1, 0, 0, 0; //transepose

            // if (i == 1)
            // {
            //     jacobianCalculator.bodies[i].theta = -1 * joint_positions[i - 1];
            //     jacobianCalculator.bodies[i].theta_dot = -1 * joint_velocities[i - 1];
            // }
            // else
            // {
            jacobianCalculator.bodies[i].theta = joint_positions[i - 1];
            jacobianCalculator.bodies[i].theta_dot = joint_velocities[i - 1];
            // }
        }
        for (int i = 1; i <= num_joint; ++i)
        {
            jacobianCalculator.bodies[i].select_joint_number_matrix(i - 1, 0) = 1;
        }
        jacobianCalculator.PropagateKinematics();

        // Eigen::Matrix3d R = T_a.block<3, 3>(0, 0);
        // Eigen::Quaterniond q(R);

        // // 쿼터니언으로부터 오일러 각 추출 (XYZ 순서)
        // Eigen::Vector3d euler_angles = q.toRotationMatrix().eulerAngles(2, 1, 0); // Roll, Pitch, Yaw
        //                                                                           // 각도 정규화 (각각의 각도에 대해 [−π, π] 범위로 조정)
        // for (int i = 0; i < 3; ++i)
        // {
        //     if (std::abs(euler_angles[i] - M_PI) < 1e-6 || std::abs(euler_angles[i] + M_PI) < 1e-6)
        //     {
        //         euler_angles[i] = 0.0; // π 또는 -π일 경우 0으로 간주
        //     }
        // }

        // 위치 출력
        // std::cout << "euler_angles: ";
        // for (double e : euler_angles)
        // {
        //     std::cout << e << " ";
        // }
        // std::cout << std::endl;

        // 위치 벡터 추출
        // Eigen::Vector3d translation = T_a.block<3, 1>(0, 3);
        // std::cout << "tttttttt: ";
        // for (double e : translation)
        // {
        //     std::cout << std::fixed << std::setprecision(3)<< e << " ";
        // }
        // std::cout << std::endl;

        // 6x1 포즈 벡터 생성
        // Eigen::VectorXd end_pose(6);
        // end_pose.head<3>() = translation;
        // end_pose.tail<3>() = euler_angles;
        endpoint_pos = T_a;
        endpoint_vel = jacobianCalculator.bodies[jacobianCalculator.numjoint_].current_body_velocity;
        endpoint_acc = jacobianCalculator.bodies[jacobianCalculator.numjoint_].current_body_acceleration;
        return jacobianCalculator.bodies[jacobianCalculator.numjoint_].jacobian_matrix;
    }

    Eigen::MatrixXd CalcBodyInertialMatrix()
    {
        Eigen::Matrix4d inertial_offset_1 = liegroup::GetTransformationMatrix(0.00781, -0.02032, 0.00003, 0, 0, 0);
        Eigen::Matrix4d inertial_offset_2 = liegroup::GetTransformationMatrix(0.00012, -0.00000, -0.07647, 0, 0, 0);
        Eigen::Matrix4d inertial_offset_3 = liegroup::GetTransformationMatrix(0.00015, 0.00119, -0.25929, 0, 0, 0);
        Eigen::Matrix4d inertial_offset_4 = liegroup::GetTransformationMatrix(-0.00005, 0.00525, -0.05652, 0, 0, 0);
        Eigen::Matrix4d inertial_offset_5 = liegroup::GetTransformationMatrix(0.00194, 0.00154, -0.19197, 0, 0, 0);
        Eigen::Matrix4d inertial_offset_6 = liegroup::GetTransformationMatrix(-0.01662, -0.00020, 0.01447, 0, 0, 0);
        Eigen::Matrix4d inertial_offset_7 = liegroup::GetTransformationMatrix(-0.01054, 0.00000, -0.02898, 0, 0, 0);
        Eigen::Matrix4d inertial_offset_8 = liegroup::GetTransformationMatrix(0, 0, -0.05300, 0, 0, 0);

        Eigen::MatrixXd body_inertial_matrix_1 = Eigen::MatrixXd::Zero(6, 6);
        Eigen::MatrixXd body_inertial_matrix_2 = Eigen::MatrixXd::Zero(6, 6);
        Eigen::MatrixXd body_inertial_matrix_3 = Eigen::MatrixXd::Zero(6, 6);
        Eigen::MatrixXd body_inertial_matrix_4 = Eigen::MatrixXd::Zero(6, 6);
        Eigen::MatrixXd body_inertial_matrix_5 = Eigen::MatrixXd::Zero(6, 6);
        Eigen::MatrixXd body_inertial_matrix_6 = Eigen::MatrixXd::Zero(6, 6);
        Eigen::MatrixXd body_inertial_matrix_7 = Eigen::MatrixXd::Zero(6, 6);
        Eigen::MatrixXd body_inertial_matrix_8 = Eigen::MatrixXd::Zero(6, 6);
        Eigen::MatrixXd body_inertial_matrix_all = Eigen::MatrixXd::Zero(6, 6);

        Eigen::MatrixXd moment_matrix_1 = Eigen::MatrixXd::Identity(3, 3);
        moment_matrix_1 << 0.00104, -0.00010, 0.00000,
            -0.00010, 0.00127, 0.00000,
            0.00000, 0.00000, 0.00187;

        Eigen::MatrixXd moment_matrix_2 = Eigen::MatrixXd::Identity(3, 3);
        moment_matrix_2 << 0.01155, 0.00000, -0.00005,
            0.00000, 0.01169, -0.00000,
            -0.00005, -0.00000, 0.00165;

        Eigen::MatrixXd moment_matrix_3 = Eigen::MatrixXd::Identity(3, 3);
        moment_matrix_3 << 0.00122, 0.00000, -0.00000,
            0.00000, 0.00127, 0.00000,
            -0.00000, 0.00000, 0.00066;

        Eigen::MatrixXd moment_matrix_4 = Eigen::MatrixXd::Identity(3, 3);
        moment_matrix_4 << 0.00356, -0.00000, 0.00000,
            -0.00000, 0.00310, -0.00029,
            0.00000, -0.00029, 0.00101;

        Eigen::MatrixXd moment_matrix_5 = Eigen::MatrixXd::Identity(3, 3);
        moment_matrix_5 << 0.00059, 0.00000, -0.00000,
            0.00000, 0.00046, 0.00004,
            -0.00000, 0.00004, 0.00030;

        Eigen::MatrixXd moment_matrix_6 = Eigen::MatrixXd::Identity(3, 3);
        moment_matrix_6 << 0.00021, 0.00001, 0.00001,
            0.00001, 0.00038, 0.00000,
            0.00001, 0.00000, 0.00031;

        Eigen::MatrixXd moment_matrix_7 = Eigen::MatrixXd::Identity(3, 3);
        moment_matrix_7 << 0.00012, -0.00000, -0.00002,
            -0.00000, 0.00036, 0.00000,
            -0.00002, 0.00000, 0.00031;

        Eigen::MatrixXd moment_matrix_8 = Eigen::MatrixXd::Identity(3, 3);
        moment_matrix_8 << 0.00051, 0.0, 0.0,
            0.0, 0.00051, 0.0,
            0.0, 0.0, 0.00010;

        double body_mass_1 = 0.62810;
        double body_mass_2 = 2.07542;
        double body_mass_3 = 0.83596;
        double body_mass_4 = 0.92391;
        double body_mass_5 = 0.32657;
        double body_mass_6 = 0.41516;
        double body_mass_7 = 0.19201;
        double body_mass_8 = 0.50000;

        body_inertial_matrix_1.block<3, 3>(0, 0) = body_mass_1 * Eigen::MatrixXd::Identity(3, 3);
        body_inertial_matrix_1.block<3, 3>(3, 3) = moment_matrix_1;
        body_inertial_matrix_2.block<3, 3>(0, 0) = body_mass_2 * Eigen::MatrixXd::Identity(3, 3);
        body_inertial_matrix_2.block<3, 3>(3, 3) = moment_matrix_2;
        body_inertial_matrix_3.block<3, 3>(0, 0) = body_mass_3 * Eigen::MatrixXd::Identity(3, 3);
        body_inertial_matrix_3.block<3, 3>(3, 3) = moment_matrix_3;
        body_inertial_matrix_4.block<3, 3>(0, 0) = body_mass_4 * Eigen::MatrixXd::Identity(3, 3);
        body_inertial_matrix_4.block<3, 3>(3, 3) = moment_matrix_4;
        body_inertial_matrix_5.block<3, 3>(0, 0) = body_mass_5 * Eigen::MatrixXd::Identity(3, 3);
        body_inertial_matrix_5.block<3, 3>(3, 3) = moment_matrix_5;
        body_inertial_matrix_6.block<3, 3>(0, 0) = body_mass_6 * Eigen::MatrixXd::Identity(3, 3);
        body_inertial_matrix_6.block<3, 3>(3, 3) = moment_matrix_6;
        body_inertial_matrix_7.block<3, 3>(0, 0) = body_mass_7 * Eigen::MatrixXd::Identity(3, 3);
        body_inertial_matrix_7.block<3, 3>(3, 3) = moment_matrix_7;
        body_inertial_matrix_8.block<3, 3>(0, 0) = body_mass_8 * Eigen::MatrixXd::Identity(3, 3);
        body_inertial_matrix_8.block<3, 3>(3, 3) = moment_matrix_8;

        body_inertial_matrix_1 = liegroup::LieAlgebra::AdjointTransformInverseTranspose(inertial_offset_1) * body_inertial_matrix_1 * liegroup::LieAlgebra::InverseAdjoint(inertial_offset_1);
        body_inertial_matrix_2 = liegroup::LieAlgebra::AdjointTransformInverseTranspose(inertial_offset_2) * body_inertial_matrix_2 * liegroup::LieAlgebra::InverseAdjoint(inertial_offset_2);
        body_inertial_matrix_3 = liegroup::LieAlgebra::AdjointTransformInverseTranspose(inertial_offset_3) * body_inertial_matrix_3 * liegroup::LieAlgebra::InverseAdjoint(inertial_offset_3);
        body_inertial_matrix_4 = liegroup::LieAlgebra::AdjointTransformInverseTranspose(inertial_offset_4) * body_inertial_matrix_4 * liegroup::LieAlgebra::InverseAdjoint(inertial_offset_4);
        body_inertial_matrix_5 = liegroup::LieAlgebra::AdjointTransformInverseTranspose(inertial_offset_5) * body_inertial_matrix_5 * liegroup::LieAlgebra::InverseAdjoint(inertial_offset_5);
        body_inertial_matrix_6 = liegroup::LieAlgebra::AdjointTransformInverseTranspose(inertial_offset_6) * body_inertial_matrix_6 * liegroup::LieAlgebra::InverseAdjoint(inertial_offset_6);
        body_inertial_matrix_7 = liegroup::LieAlgebra::AdjointTransformInverseTranspose(inertial_offset_7) * body_inertial_matrix_7 * liegroup::LieAlgebra::InverseAdjoint(inertial_offset_7);
        body_inertial_matrix_8 = liegroup::LieAlgebra::AdjointTransformInverseTranspose(inertial_offset_8) * body_inertial_matrix_8 * liegroup::LieAlgebra::InverseAdjoint(inertial_offset_8);

        body_inertial_matrix_1 = liegroup::LieAlgebra::AdjointTransformInverseTranspose(H[0]) *
                                 body_inertial_matrix_1 *
                                 liegroup::LieAlgebra::InverseAdjoint(H[0]);
        body_inertial_matrix_2 = liegroup::LieAlgebra::AdjointTransformInverseTranspose(H[0] * H[1]) *
                                 body_inertial_matrix_2 *
                                 liegroup::LieAlgebra::InverseAdjoint(H[0] * H[1]);
        body_inertial_matrix_3 = liegroup::LieAlgebra::AdjointTransformInverseTranspose(H[0] * H[1] * H[2]) *
                                 body_inertial_matrix_3 *
                                 liegroup::LieAlgebra::InverseAdjoint(H[0] * H[1] * H[2]);
        body_inertial_matrix_4 = liegroup::LieAlgebra::AdjointTransformInverseTranspose(H[0] * H[1] * H[2] * H[3]) *
                                 body_inertial_matrix_4 *
                                 liegroup::LieAlgebra::InverseAdjoint(H[0] * H[1] * H[2] * H[3]);
        body_inertial_matrix_5 = liegroup::LieAlgebra::AdjointTransformInverseTranspose(H[0] * H[1] * H[2] * H[3] * H[4]) *
                                 body_inertial_matrix_5 *
                                 liegroup::LieAlgebra::InverseAdjoint(H[0] * H[1] * H[2] * H[3] * H[4]);
        body_inertial_matrix_6 = liegroup::LieAlgebra::AdjointTransformInverseTranspose(H[0] * H[1] * H[2] * H[3] * H[4] * H[5]) *
                                 body_inertial_matrix_6 *
                                 liegroup::LieAlgebra::InverseAdjoint(H[0] * H[1] * H[2] * H[3] * H[4] * H[5]);
        body_inertial_matrix_7 = liegroup::LieAlgebra::AdjointTransformInverseTranspose(H[0] * H[1] * H[2] * H[3] * H[4] * H[5] * H[6]) *
                                 body_inertial_matrix_7 *
                                 liegroup::LieAlgebra::InverseAdjoint(H[0] * H[1] * H[2] * H[3] * H[4] * H[5] * H[6]);
        body_inertial_matrix_8 = liegroup::LieAlgebra::AdjointTransformInverseTranspose(H[0] * H[1] * H[2] * H[3] * H[4] * H[5] * H[6] * H[7]) *
                                 body_inertial_matrix_8 *
                                 liegroup::LieAlgebra::InverseAdjoint(H[0] * H[1] * H[2] * H[3] * H[4] * H[5] * H[6] * H[7]);

        body_inertial_matrix_all = body_inertial_matrix_1 +
                                   body_inertial_matrix_2 +
                                   body_inertial_matrix_3 +
                                   body_inertial_matrix_4 +
                                   body_inertial_matrix_5 +
                                   body_inertial_matrix_6 +
                                   body_inertial_matrix_7 +
                                   body_inertial_matrix_8;

        // 질량 및 각 링크의 변환 행렬을 사용해 초기화
        double total_mass = body_mass_1 + body_mass_2 + body_mass_3 + body_mass_4 +
                            body_mass_5 + body_mass_6 + body_mass_7 + body_mass_8;

        Eigen::Vector3d com_total = Eigen::Vector3d::Zero();
        // 각 링크의 질량중심 위치 계산
        Eigen::Vector3d com_link_1 = (H[0] * inertial_offset_1).block<3, 1>(0, 3);
        Eigen::Vector3d com_link_2 = (H[0] * H[1] * inertial_offset_2).block<3, 1>(0, 3);
        Eigen::Vector3d com_link_3 = (H[0] * H[1] * H[2] * inertial_offset_3).block<3, 1>(0, 3);
        Eigen::Vector3d com_link_4 = (H[0] * H[1] * H[2] * H[3] * inertial_offset_4).block<3, 1>(0, 3);
        Eigen::Vector3d com_link_5 = (H[0] * H[1] * H[2] * H[3] * H[4] * inertial_offset_5).block<3, 1>(0, 3);
        Eigen::Vector3d com_link_6 = (H[0] * H[1] * H[2] * H[3] * H[4] * H[5] * inertial_offset_6).block<3, 1>(0, 3);
        Eigen::Vector3d com_link_7 = (H[0] * H[1] * H[2] * H[3] * H[4] * H[5] * H[6] * inertial_offset_7).block<3, 1>(0, 3);
        Eigen::Vector3d com_link_8 = (H[0] * H[1] * H[2] * H[3] * H[4] * H[5] * H[6] * H[7] * inertial_offset_8).block<3, 1>(0, 3);
        // 질량 가중치 합산
        com_total = (body_mass_1 * com_link_1 +
                    body_mass_2 * com_link_2 +
                    body_mass_3 * com_link_3 +
                    body_mass_4 * com_link_4 +
                    body_mass_5 * com_link_5 +
                    body_mass_6 * com_link_6 +
                    body_mass_7 * com_link_7 +
                    body_mass_8 * com_link_8) / total_mass;
        // std::cout << std::fixed << std::setprecision(3) << com_total << std::endl;
        return body_inertial_matrix_all;
    }
    rclcpp::Subscription<JointStates>::SharedPtr joint_subscriber_;
    rclcpp::Publisher<JointStates>::SharedPtr joint_publisher_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr accelerate_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr force_publisher_;

    std::vector<Eigen::Matrix4d> H;
    Eigen::Matrix4d endpoint_pos;
    Eigen::VectorXd endpoint_vel;
    Eigen::VectorXd endpoint_acc;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LeftArm>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}