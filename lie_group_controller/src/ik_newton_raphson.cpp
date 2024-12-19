#include "lie_toolbox/polynomial_interpolation.hpp"
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
        "l_hip_y", "l_hip_r", "l_hip_p", "l_knee_p", "l_ankle_r", "l_ankle_p"};
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

        trajectory_ = liegroup::FifthOrderPolynomialTrajectory(
            0.0, -0.6, 0.0, 0.0, // 초기 시간, 위치, 속도, 가속도
            1.0, -0.3, 0.0, 0.0  // 최종 시간, 위치, 속도, 가속도
        );
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

        const double tolerance = 1e-6; // 원하는 위치 오차 임계값 설정

        Eigen::VectorXd current_pose = ForwardKinematics(theta);

        double current_time = (this->now() - start_time_).seconds();

        double z_position = trajectory_.GetPosition(current_time);
        // double z_velocity = trajectory_.GetVelocity(current_time);
        // double z_acceleration = trajectory_.GetAcceleration(current_time);

        // 목표 위치 벡터 설정
        Eigen::VectorXd p_des(6);
        p_des << 0.0, 0.0, z_position, liegroup::DegToRad(0.0), liegroup::DegToRad(0.0), liegroup::DegToRad(0.0);

        int iteration = 0;

        Eigen::VectorXd delta_p = p_des - ForwardKinematics(theta);

        // 반복 종료 조건
        if (delta_p.norm() < tolerance)
        {
            return;
        }
        auto start_time_calc = std::chrono::steady_clock::now();
        while (true)
        {
            // 순방향 운동학 계산

            Eigen::VectorXd current_pose = ForwardKinematics(theta);
            // for (double e : current_pose)
            // {
            //     std::cout << std::fixed << std::setprecision(3) << e << " ";
            // }
            // std::cout << std::endl;
            Eigen::VectorXd delta_p = p_des - current_pose;
        
            // 만약 편차가 임계값 이하로 작아지면 반복을 종료
            if (delta_p.norm() < tolerance)
            {
                break;
            }
            // 자코비안 계산
            Eigen::MatrixXd J = ComputeJacobian([this](const Eigen::VectorXd &theta)
                                                { return ForwardKinematics(theta); }, theta);
            // 뉴턴-랩슨 방법으로 각도 업데이트 (뎀프드 최소자승 역행렬 사용)
            Eigen::VectorXd delta_theta = DampedLeastSquaresInverse(J, 1e-4) * delta_p;
            theta += delta_theta;
            iteration++;
        }
        auto end_time_calc = std::chrono::steady_clock::now();
        auto duration_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time_calc - start_time_calc).count();
        double duration_ms = static_cast<double>(duration_ns) / 1e6; // 나노초를 밀리초로 변환

        // 걸린 시간 출력
        std::cout << "While 문이 반복을 완료하는데 걸린 시간: " << duration_ms << " ms" << std::endl;

        // 메시지 퍼블리싱
        auto joint_state_msg = sensor_msgs::msg::JointState();
        joint_state_msg.header.stamp = this->now();
        for (const auto &joint_name : joint_names)
        {
            joint_state_msg.name.push_back(joint_name);
            joint_state_msg.position.push_back(theta[joint_name_to_number[joint_name]]);
        }
        joint_publisher_->publish(joint_state_msg);

        // geometry_msgs::msg::Twist position_msg;
        // position_msg.linear.x = current_pose(0);
        // position_msg.linear.y = current_pose(1);
        // position_msg.linear.z = current_pose(2);
        // position_msg.angular.x = current_pose(3);
        // position_msg.angular.y = current_pose(4);
        // position_msg.angular.z = current_pose(5);
        // position_publisher_->publish(position_msg);
        // 궤적이 끝에 도달했는지 확인하고 반전
        if (current_time >= trajectory_.final_time_)
        {
            reverse_ = !reverse_;
            // double initial_time = reverse_ ? 5.0 : 0.0;
            // double final_time = reverse_ ? 0.0 : 5.0;
            double initial_pos = reverse_ ? -0.3 : -0.6;
            double final_pos = reverse_ ? -0.6 : -0.3;
            double initial_time = 0.0;
            double final_time = 1.0;

            trajectory_.ChangeTrajectory(
                initial_time, initial_pos, 0.0, 0.0,
                final_time, final_pos, 0.0, 0.0);
            start_time_ = this->now(); // 시작 시간 초기화
        }
    }

    Eigen::VectorXd ForwardKinematics(const Eigen::VectorXd &theta)
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
        joint_T_1 << cos(theta(0)), -sin(theta(0)), 0, 0,
            sin(theta(0)), cos(theta(0)), 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1; // l_hip_y

        joint_T_2 << cos(theta(1)), -sin(theta(1)), 0, 0,
            sin(theta(1)), cos(theta(1)), 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1; // l_hip_r
        joint_T_3 << cos(theta(2)), -sin(theta(2)), 0, 0,
            sin(theta(2)), cos(theta(2)), 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1; // l_hip_p
        joint_T_4 << cos(theta(3)), -sin(theta(3)), 0, 0,
            sin(theta(3)), cos(theta(3)), 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1; // l_knee_p

        joint_T_5 << cos(theta(4)), -sin(theta(4)), 0, 0,
            sin(theta(4)), cos(theta(4)), 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1; // l_ankle_r

        joint_T_6 << cos(theta(5)), -sin(theta(5)), 0, 0,
            sin(theta(5)), cos(theta(5)), 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1; // l_ankle_p

        Eigen::Matrix4d adjacent_T_1 = offset_joint_0_T_1 * joint_T_1;
        Eigen::Matrix4d adjacent_T_2 = offset_joint_1_T_2 * joint_T_2;
        Eigen::Matrix4d adjacent_T_3 = offset_joint_2_T_3 * joint_T_3;
        Eigen::Matrix4d adjacent_T_4 = offset_joint_3_T_4 * joint_T_4;
        Eigen::Matrix4d adjacent_T_5 = offset_joint_4_T_5 * joint_T_5;
        Eigen::Matrix4d adjacent_T_6 = offset_joint_5_T_6 * joint_T_6;
        Eigen::Matrix4d adjacent_T_7 = offset_joint_6_T_7;
        T_goal = T_goal * adjacent_T_1 * adjacent_T_2 * adjacent_T_3 * adjacent_T_4 * adjacent_T_5 * adjacent_T_6 * adjacent_T_7;
        Eigen::VectorXd result(6);

        // Position (x, y, z)
        result[0] = T_goal(0, 3);
        result[1] = T_goal(1, 3);
        result[2] = T_goal(2, 3);

        // Orientation (roll, pitch, yaw) using intrinsic Tait-Bryan angles (XYZ convention)
        double roll = atan2(T_goal(2, 1), T_goal(2, 2));
        double pitch = atan2(-T_goal(2, 0), sqrt(T_goal(2, 1) * T_goal(2, 1) + T_goal(2, 2) * T_goal(2, 2)));
        double yaw = atan2(T_goal(1, 0), T_goal(0, 0));

        result[3] = roll;
        result[4] = pitch;
        result[5] = yaw;

        return result;
    }

    Eigen::MatrixXd ComputeJacobian(std::function<Eigen::VectorXd(Eigen::VectorXd)> func, const Eigen::VectorXd &theta)
    {
        int m = func(theta).size(); // 출력 벡터의 크기 (6)
        int n = theta.size();       // 입력 벡터의 크기 (6)
        Eigen::MatrixXd J(m, n);    // 자코비안 행렬 생성

        double h = 1e-6; // 작은 변화량 (유한 차분)
        Eigen::VectorXd theta_perturbed = theta;

        for (int i = 0; i < n; ++i)
        {
            theta_perturbed = theta;
            theta_perturbed(i) += h;

            Eigen::VectorXd f_plus = func(theta_perturbed); // f(theta + h)
            Eigen::VectorXd f_minus = func(theta);          // f(theta)

            J.col(i) = (f_plus - f_minus) / h; // 유한 차분 계산
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
    liegroup::FifthOrderPolynomialTrajectory trajectory_;

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
    bool reverse_ = false;
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
