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

class LEG : public rclcpp::Node
{
public:
    std::vector<std::string> joint_names = {
        "l_hip_p", "l_hip_r", "l_hip_y", "l_knee_p", "l_ankle_p", "l_ankle_r"};
    // 각 관절 이름에 대한 인덱스를 매핑하는 맵
    std::unordered_map<std::string, int> joint_name_to_number;

    // 관절의 위치와 속도를 저장하는 배열
    std::vector<double> joint_positions;
    std::vector<double> cmd_joint_positions;
    std::vector<double> joint_velocities;

    LEG()
        : Node("LEG")
    {
        rclcpp::QoS qos_profile(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
        qos_profile.keep_last(10);
        qos_profile.reliability(rclcpp::ReliabilityPolicy::Reliable);
        qos_profile.durability(rclcpp::DurabilityPolicy::Volatile);
        joint_subscriber_ = this->create_subscription<JointStates>(
            "/robot/joint_states",
            qos_profile,
            std::bind(&LEG::JointMessageCallback, this, _1));

        // 엔드포인트 속도 퍼블리셔
        position_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/end_effector_position", qos_profile);

        velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/end_effector_velocity", qos_profile);

        // 엔드포인트 가속도 퍼블리셔
        accelerate_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/end_effector_accelerate", qos_profile);

        // 엔드포인트 힘 퍼블리셔
        force_publisher_ = this->create_publisher<geometry_msgs::msg::Wrench>("/end_effector_force", qos_profile);

        joint_publisher_ = this->create_publisher<JointStates>("/joint_states", qos_profile);

        auto node_clock = this->get_clock();
        timer_ = rclcpp::create_timer(
            this,
            node_clock,
            std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double>(4 / 1000)),
            std::bind(&LEG::Calc, this));

        joint_positions.resize(joint_names.size());
        cmd_joint_positions.resize(joint_names.size());
        joint_velocities.resize(joint_names.size());

        for (size_t i = 0; i < joint_names.size(); ++i)
        {
            joint_positions[i] = 0;
            joint_velocities[i] = 0;
        }

        // 초기 상태 설정 (위치, 속도, 가속도)
        endpoint_pos = Eigen::MatrixXd::Identity(4, 4);
        endpoint_vel = Eigen::MatrixXd::Zero(6, 1);
        endpoint_acc = Eigen::MatrixXd::Zero(6, 1);

        // 시작 시간 기록
        start_time_ = this->now();
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
        joint_states_received = true;
    }

    void Calc()
    {
        if (!joint_states_received)
        {
            // 관절 상태를 수신할 때까지 계산을 건너뜁니다
            return;
        }

        // 현재 각도와 속도를 사용하여 엔드 이펙터 위치 계산
        Eigen::VectorXd theta(joint_positions.size());
        for (size_t i = 0; i < joint_positions.size(); ++i)
        {
            theta(i) = joint_positions[i];
        }

        const double tolerance = 1e-4; // 원하는 위치 오차 임계값 설정
        const double time_step = 0.1;  // 학습률 설정

        // 시간에 따른 사인파 목표 z 위치 생성
        double elapsed_time = (this->now() - start_time_).seconds();
        double amplitude = 0.1; // 사인파의 진폭 (미터 단위)
        double frequency = 0.5; // 사인파의 주파수 (Hz)
        double omega = 2 * M_PI * frequency;
        double base_z = -0.5; // 기본 z 위치
        double z = amplitude * sin(omega * elapsed_time) + base_z;

        // 목표 위치와 회전을 SE(3) 변환 행렬로 설정
        Eigen::VectorXd p_des(6);
        p_des << 0.1, 0.0, -0.5, liegroup::DegToRad(5.0), liegroup::DegToRad(20.0), liegroup::DegToRad(40.0);
        Eigen::Matrix4d T_desired = liegroup::GetTransformationMatrix(p_des(0), p_des(1), p_des(2), p_des(3), p_des(4), p_des(5));

        // 현재 위치를 SE(3) 변환 행렬로 계산
        Eigen::Matrix4d T_current = ForwardKinematicsSE3(theta);

        // 상대 변환 계산
        Eigen::Matrix4d T_relative = T_current.inverse() * T_desired;

        // 반복 계산 시작
        auto start_time_calc = std::chrono::high_resolution_clock::now();
        int iteration = 0;

        while (true)
        {
            // 현재 위치와 목표 위치의 상대 변환 재계산
            T_current = ForwardKinematicsSE3(theta);
            T_relative = T_current.inverse() * T_desired;
            // std::cout << "T_relative: " << std::endl
            //           << T_relative << std::endl;
            // 상대 변환에서 오차 추출
            Eigen::VectorXd lambda = liegroup::LieAlgebra::LogarithmMapSE3(T_relative);

            // 반복 종료 조건: 오차가 임계값 이하
            if (lambda.norm() < tolerance)
            {
                break;
            }

            // 상대 자코비안 계산
            double h = 1e-3;
            std::vector<Eigen::Matrix4d> dT_dtheta(theta.size());
            {
                for (int i = 0; i < (int)theta.size(); i++)
                {
                    Eigen::VectorXd theta_plus = theta;
                    Eigen::VectorXd theta_minus = theta;

                    theta_plus(i) += h;
                    theta_minus(i) -= h;

                    Eigen::Matrix4d T_plus = ForwardKinematicsSE3(theta_plus);
                    Eigen::Matrix4d T_minus = ForwardKinematicsSE3(theta_minus);

                    dT_dtheta[i] = (T_plus - T_minus) / (2.0 * h);
                }
            }

            Eigen::MatrixXd J = ComputeRelativeJacobian(T_relative, dT_dtheta);
            // std::cout << "J: " << std::endl
            //           << J << std::endl;

            // 유사 역행렬을 사용한 업데이트
            Eigen::MatrixXd J_pseudo_inverse = DampedLeastSquaresInverse(J, 1e-3);
            Eigen::VectorXd delta_theta = -time_step * J_pseudo_inverse * lambda;
            // Eigen::MatrixXd J_pseudo_inverse = DampedLeastSquaresInverse(J, 1e-3);
            // Eigen::VectorXd delta_theta = -time_step * J.transpose() * lambda;
            // Eigen::VectorXd delta_theta = -time_step * J.transpose() * lambda;
            theta += delta_theta;
            iteration++;
        }

        auto end_time_calc = std::chrono::high_resolution_clock::now();
        auto duration_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time_calc - start_time_calc).count();
        double duration_ms = duration_ns / 1e6; // 나노초를 밀리초로 변환

        // 결과 출력
        RCLCPP_INFO(this->get_logger(), "While 문이 반복을 완료하는데 걸린 시간: %.3f ms | 횟수: %d", duration_ms, iteration);

        // 메시지 퍼블리싱
        auto joint_state_msg = sensor_msgs::msg::JointState();
        joint_state_msg.header.stamp = this->now();
        for (const auto &joint_name : joint_names)
        {
            joint_state_msg.name.push_back(joint_name);
            joint_state_msg.position.push_back(theta[joint_name_to_number[joint_name]]);
        }
        joint_publisher_->publish(joint_state_msg);
    }

    Eigen::Matrix4d ForwardKinematicsSE3(const Eigen::VectorXd &theta)
    {
        Eigen::Matrix4d offset_joint_0_T_1;
        Eigen::Matrix4d offset_joint_1_T_2;
        Eigen::Matrix4d offset_joint_2_T_3;
        Eigen::Matrix4d offset_joint_3_T_4;
        Eigen::Matrix4d offset_joint_4_T_5;
        Eigen::Matrix4d offset_joint_5_T_6;
        Eigen::Matrix4d offset_joint_6_T_7;

        offset_joint_0_T_1 << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;
        offset_joint_0_T_1 = offset_joint_0_T_1 * liegroup::GetTransformationMatrix(0, 0, 0, liegroup::DegToRad(5.0), 0, 0);

        offset_joint_1_T_2 << 0, 0, 1, 0,
            1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 0, 1;

        offset_joint_2_T_3 << 0, 0, 1, 0,
            0, -1, 0, -0.055,
            1, 0, 0, 0,
            0, 0, 0, 1;

        offset_joint_3_T_4 << 1, 0, 0, 0,
            0, 1, 0, 0.355,
            0, 0, 1, 0,
            0, 0, 0, 1;

        offset_joint_4_T_5 << 0, 0, 1, 0,
            0, -1, 0, 0.355,
            1, 0, 0, 0,
            0, 0, 0, 1;

        offset_joint_5_T_6 << 0, 0, 1, 0,
            0, -1, 0, -0.015,
            1, 0, 0, 0,
            0, 0, 0, 1;

        offset_joint_6_T_7 << 1, 0, 0, 0,
            0, 0, -1, 0.034,
            0, 1, 0, 0,
            0, 0, 0, 1;

        Eigen::Matrix4d T_goal = Eigen::MatrixXd::Identity(4, 4);
        Eigen::Matrix4d joint_T_1;
        Eigen::Matrix4d joint_T_2;
        Eigen::Matrix4d joint_T_3;
        Eigen::Matrix4d joint_T_4;
        Eigen::Matrix4d joint_T_5;
        Eigen::Matrix4d joint_T_6;

        //  "l_hip_p", "l_hip_r", "l_hip_y", "l_knee_p", "l_ankle_p", "l_ankle_r"
        joint_T_1 << cos(theta(2)), -sin(theta(2)), 0, 0,
            sin(theta(2)), cos(theta(2)), 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1; // l_hip_y

        joint_T_2 << cos(theta(1)), -sin(theta(1)), 0, 0,
            sin(theta(1)), cos(theta(1)), 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1; // l_hip_r
        joint_T_3 << cos(theta(0)), -sin(theta(0)), 0, 0,
            sin(theta(0)), cos(theta(0)), 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1; // l_hip_p
        joint_T_4 << cos(theta(3)), -sin(theta(3)), 0, 0,
            sin(theta(3)), cos(theta(3)), 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1; // l_knee_p

        joint_T_5 << cos(theta(5)), -sin(theta(5)), 0, 0,
            sin(theta(5)), cos(theta(5)), 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1; // l_ankle_r

        joint_T_6 << cos(theta(4)), -sin(theta(4)), 0, 0,
            sin(theta(4)), cos(theta(4)), 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1; // l_ankle_p

        Eigen::Matrix4d adjacent_T_1 = offset_joint_0_T_1 * joint_T_1;
        Eigen::Matrix4d adjacent_T_2 = offset_joint_1_T_2 * joint_T_2;
        Eigen::Matrix4d adjacent_T_3 = offset_joint_2_T_3 * joint_T_3;
        Eigen::Matrix4d adjacent_T_4 = offset_joint_3_T_4 * joint_T_4;
        Eigen::Matrix4d adjacent_T_5 = offset_joint_4_T_5 * joint_T_5;
        Eigen::Matrix4d adjacent_T_6 = offset_joint_5_T_6 * joint_T_6;
        Eigen::Matrix4d adjacent_T_7 = offset_joint_6_T_7;
        return T_goal = T_goal * adjacent_T_1 * adjacent_T_2 * adjacent_T_3 * adjacent_T_4 * adjacent_T_5 * adjacent_T_6 * adjacent_T_7;
        // Eigen::VectorXd result(6);

        // // Position (x, y, z)
        // result[0] = T_goal(0, 3);
        // result[1] = T_goal(1, 3);
        // result[2] = T_goal(2, 3);

        // // Orientation (roll, pitch, yaw) using intrinsic Tait-Bryan angles (XYZ convention)
        // double roll = atan2(T_goal(2, 1), T_goal(2, 2));
        // double pitch = atan2(-T_goal(2, 0), sqrt(T_goal(2, 1) * T_goal(2, 1) + T_goal(2, 2) * T_goal(2, 2)));
        // double yaw = atan2(T_goal(1, 0), T_goal(0, 0));

        // result[3] = roll;
        // result[4] = pitch;
        // result[5] = yaw;

        // return result;
    }

    Eigen::MatrixXd ComputeRelativeJacobian(
        const Eigen::Matrix4d &T_relative,
        const std::vector<Eigen::Matrix4d> &dT_dtheta)
    {
        int n = dT_dtheta.size(); // Number of joints
        Eigen::MatrixXd J(6, n);  // Relative Jacobian (6 x n)

        // Step 1: Compute lambda using logarithm map
        Eigen::VectorXd lambda = liegroup::LieAlgebra::LogarithmMapSE3(T_relative);

        // Step 2: Compute dexp(-lambda)
        // Eigen::MatrixXd dexp_neg_lambda = liegroup::LieAlgebra::InverseDifferentialExponentialMapSE3(lambda);
        Eigen::MatrixXd dexp_neg_lambda = liegroup::LieAlgebra::DifferentialExponentialMapSE3(lambda);

        // Step 3: Compute J_k for each joint
        for (int k = 0; k < n; ++k)
        {
            Eigen::Matrix4d dT_k = dT_dtheta[k];
            Eigen::Matrix4d T_inv_dT = T_relative.inverse() * dT_k;

            // Convert T_inv_dT to se(3) vector form
            Eigen::VectorXd se3_vector = liegroup::LieAlgebra::Floor6DVectorOperator(T_inv_dT);
            // J.col(k) = dexp_neg_lambda * se3_vector;
            J.col(k) = se3_vector;
        }

        return J;
    }

    Eigen::MatrixXd DampedLeastSquaresInverse(const Eigen::MatrixXd &J, double epsilon = 1e-6)
    {
        int m = J.rows();
        int n = J.cols();
        if (m >= n)
        {
            // 왼쪽 역행렬 (m >= n 인 경우)
            return (J.transpose() * J + epsilon * Eigen::MatrixXd::Identity(n, n)).inverse() * J.transpose();
        }
        else
        {
            // 오른쪽 역행렬 (m < n 인 경우)
            return J.transpose() * (J * J.transpose() + epsilon * Eigen::MatrixXd::Identity(m, m)).inverse();
        }
    }

    rclcpp::Subscription<JointStates>::SharedPtr joint_subscriber_;
    rclcpp::Publisher<JointStates>::SharedPtr joint_publisher_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_, position_publisher_, accelerate_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr force_publisher_;

    std::vector<Eigen::Matrix4d> H;
    Eigen::Matrix4d endpoint_pos;
    Eigen::VectorXd endpoint_vel;
    Eigen::VectorXd endpoint_acc;
    rclcpp::TimerBase::SharedPtr timer_;
    bool joint_states_received = false;
    rclcpp::Time start_time_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LEG>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
