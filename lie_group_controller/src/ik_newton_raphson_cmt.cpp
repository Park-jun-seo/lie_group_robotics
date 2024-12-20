#include "lie_toolbox/polynomial_interpolation.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include "std_msgs/msg/float64_multi_array.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include "lie_toolbox/incremental_jacobian.hpp"
#include <osqp.h>
#include <iostream>
#include <vector>
#include <chrono>
#include <Eigen/Sparse>
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
    std::vector<double> prev_joint_velocities;
    std::vector<double> joint_acceleration;

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
        joint_acceleration.resize(joint_names.size());
        prev_joint_velocities.resize(joint_names.size());
        for (size_t i = 0; i < joint_names.size(); ++i)
        {
            joint_positions[i] = 0;
            joint_velocities[i] = 0;
            joint_acceleration[i] = 0;
            prev_joint_velocities[i] = 0;
        }

        // 초기 상태 설정 (위치, 속도, 가속도)
        endpoint_pos = Eigen::MatrixXd::Identity(4, 4);
        endpoint_vel = Eigen::MatrixXd::Zero(6, 1);
        endpoint_acc = Eigen::MatrixXd::Zero(6, 1);

        // 시작 시간 기록
        start_time_ = this->now();
        moment_matrix_1.resize(3, 3);
        moment_matrix_2.resize(3, 3);
        moment_matrix_3.resize(3, 3);
        moment_matrix_4.resize(3, 3);
        moment_matrix_5.resize(3, 3);
        moment_matrix_6.resize(3, 3);

        moment_matrix_1 << 0.0002, 0, 0,
            0, 0.0002, 0,
            0, 0, 0.0002;
        moment_matrix_2 << 0.0002, 0, 0,
            0, 0.0002, 0,
            0, 0, 0.0002;
        moment_matrix_3 << 0.0002, 0, 0,
            0, 0.0002, 0,
            0, 0, 0.0002;
        moment_matrix_4 << 0.0002, 0, 0,
            0, 0.0002, 0,
            0, 0, 0.0002;
        moment_matrix_5 << 0.0002, 0, 0,
            0, 0.0002, 0,
            0, 0, 0.0002;
        moment_matrix_6 << 0.00185933, 0, 0,
            0, 0.00185933, 0,
            0, 0, 0.00333333;

        moment_matrix.push_back(moment_matrix_1);
        moment_matrix.push_back(moment_matrix_2);
        moment_matrix.push_back(moment_matrix_3);
        moment_matrix.push_back(moment_matrix_4);
        moment_matrix.push_back(moment_matrix_5);
        moment_matrix.push_back(moment_matrix_6);

        inertia_offset_6 = inertia_offset_5 =
            inertia_offset_4 = inertia_offset_3 =
                inertia_offset_2 = inertia_offset_1 = Eigen::MatrixXd::Identity(4, 4);

        body_inertia_matrix_1.resize(6, 6);
        body_inertia_matrix_2.resize(6, 6);
        body_inertia_matrix_3.resize(6, 6);
        body_inertia_matrix_4.resize(6, 6);
        body_inertia_matrix_5.resize(6, 6);
        body_inertia_matrix_6.resize(6, 6);
        body_inertia_matrix_1.block<3, 3>(0, 0) = body_masses[0] * Eigen::MatrixXd::Identity(3, 3);
        body_inertia_matrix_1.block<3, 3>(3, 3) = moment_matrix[0];
        body_inertia_matrix_2.block<3, 3>(0, 0) = body_masses[1] * Eigen::MatrixXd::Identity(3, 3);
        body_inertia_matrix_2.block<3, 3>(3, 3) = moment_matrix[1];
        body_inertia_matrix_3.block<3, 3>(0, 0) = body_masses[2] * Eigen::MatrixXd::Identity(3, 3);
        body_inertia_matrix_3.block<3, 3>(3, 3) = moment_matrix[2];
        body_inertia_matrix_4.block<3, 3>(0, 0) = body_masses[3] * Eigen::MatrixXd::Identity(3, 3);
        body_inertia_matrix_4.block<3, 3>(3, 3) = moment_matrix[3];
        body_inertia_matrix_5.block<3, 3>(0, 0) = body_masses[4] * Eigen::MatrixXd::Identity(3, 3);
        body_inertia_matrix_5.block<3, 3>(3, 3) = moment_matrix[4];
        body_inertia_matrix_6.block<3, 3>(0, 0) = body_masses[5] * Eigen::MatrixXd::Identity(3, 3);
        body_inertia_matrix_6.block<3, 3>(3, 3) = moment_matrix[5];
        body_inertia_matrix = {body_inertia_matrix_1, body_inertia_matrix_2, body_inertia_matrix_3, body_inertia_matrix_4, body_inertia_matrix_5, body_inertia_matrix_6};

        Eigen::VectorXd e_offset_1(6);
        e_offset_1 << inertia_offset_1(0, 3), inertia_offset_1(1, 3), inertia_offset_1(2, 3), 0, 0, 1;
        Eigen::VectorXd e_offset_2(6);
        e_offset_2 << inertia_offset_2(0, 3), inertia_offset_2(1, 3), inertia_offset_2(2, 3), 0, 0, 1;
        Eigen::VectorXd e_offset_3(6);
        e_offset_3 << inertia_offset_3(0, 3), inertia_offset_3(1, 3), inertia_offset_3(2, 3), 0, 0, 1;
        Eigen::VectorXd e_offset_4(6);
        e_offset_4 << inertia_offset_4(0, 3), inertia_offset_4(1, 3), inertia_offset_4(2, 3), 0, 0, 1;
        Eigen::VectorXd e_offset_5(6);
        e_offset_5 << inertia_offset_5(0, 3), inertia_offset_5(1, 3), inertia_offset_5(2, 3), 0, 0, 1;
        Eigen::VectorXd e_offset_6(6);
        e_offset_6 << inertia_offset_6(0, 3), inertia_offset_6(1, 3), inertia_offset_6(2, 3), 0, 0, 1;
        e_offset = {e_offset_1, e_offset_2, e_offset_3, e_offset_4, e_offset_5, e_offset_6};
        for (int i = 0; i < 7; ++i)
        {
            Eigen::VectorXd twist = Eigen::VectorXd::Zero(6); // 6x1 영 벡터
            body_twist.push_back(twist);
        }

        for (int i = 0; i < 7; ++i)
        {
            Eigen::VectorXd twist = Eigen::VectorXd::Zero(6); // 6x1 영 벡터
            body_twist_dot.push_back(twist);
        }

        for (int i = 0; i < 6; ++i)
        {
            Eigen::MatrixXd bais_m;
            bais_m.resize(6, 6);
            bais_m.setZero(6, 6);
            bais_matrix.push_back(bais_m);
        }

        trajectory_ = liegroup::FifthOrderPolynomialTrajectory(
            0.0, -0.81, 0.0, 0.0, // 초기 시간, 위치, 속도, 가속도
            2.0, -0.4, 0.0, 0.0   // 최종 시간, 위치, 속도, 가속도
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
            joint_acceleration[i] = (msg->velocity[index] - prev_joint_velocities[i]) / 0.004;
            prev_joint_velocities[i] = msg->velocity[index];
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
        Eigen::VectorXd curr_theta(joint_positions.size());
        Eigen::VectorXd des_theta(joint_positions.size());
        Eigen::VectorXd curr_theta_dot(joint_positions.size());
        Eigen::VectorXd delta_theta_dot(joint_positions.size());
        Eigen::VectorXd torque(joint_positions.size());
        for (size_t i = 0; i < joint_names.size(); ++i)
        {
            const std::string &joint_name = joint_names[i];
            curr_theta[i] = joint_positions[joint_name_to_number[joint_name]];
            des_theta[i] = joint_positions[joint_name_to_number[joint_name]];
            curr_theta_dot[i] = joint_velocities[joint_name_to_number[joint_name]];
            delta_theta_dot(i) = 0;
            torque(i) = 0;
        }

        const double tolerance = 1e-4; // 원하는 위치 오차 임계값 설정

        Eigen::VectorXd theta_min(6); // 최소 각도 제한
        Eigen::VectorXd theta_max(6); // 최대 각도 제한
        theta_min << -1.57, -3.14, -3.14, 0, -3.14, -3.14;
        theta_max << 1.57, 3.14, 3.14, 3.14, 3.14, 3.14;


        double current_time = (this->now() - start_time_).seconds();

        double z_position = trajectory_.GetPosition(current_time);
        double z_velocity = trajectory_.GetVelocity(current_time);
        double z_acceleration = trajectory_.GetAcceleration(current_time);

        // 목표 위치 벡터 설정
        Eigen::VectorXd p_des(6);
        p_des << 0.0, 0.0, z_position, liegroup::DegToRad(0.0), liegroup::DegToRad(0.0), liegroup::DegToRad(0.0);

        // int iteration = 0;

        Eigen::VectorXd delta_p = p_des - ForwardKinematics(curr_theta);

        // 반복 종료 조건
        if (delta_p.norm() < tolerance)
        {
            return;
        }
        // auto start_time_calc = std::chrono::steady_clock::now();

        Eigen::MatrixXd J = ComputeJacobian([this](const Eigen::VectorXd &theta)
                                            { return ForwardKinematics(theta); }, curr_theta);
        SolveOptimization(J, delta_p, delta_theta_dot, curr_theta, theta_min, theta_max);
        des_theta += delta_theta_dot;

        UpdateForwardKinematics();
        Eigen::MatrixXd mass_matrix = ComputeMassMatrix();
        Eigen::MatrixXd corioli_matrix = ComputeCorioliMatrix();
        Eigen::VectorXd gravity_matrix = ComputeGravityMatrix();
        // std::cout << "-----------------" << std::endl;
        // std::cout << gravity_matrix << std::endl;

        double dt = 0.004; // dt 설정
        Eigen::VectorXd theta_t_dt = curr_theta + delta_theta_dot * dt;
        Eigen::MatrixXd J_dot = ComputeJacobianDot(
            [this](const Eigen::VectorXd &theta)
            {
                return ForwardKinematics(theta);
            },
            curr_theta, // 현재 theta (t 시점)
            theta_t_dt, // dt 후의 theta (t+dt 시점)
            dt          // 시간 스텝
        );

        Eigen::VectorXd task_des_acc(6);
        task_des_acc << 0.0, 0.0, z_acceleration, 0, 0, 0;

        Eigen::VectorXd task_des_vel(6);
        task_des_vel << 0.0, 0.0, z_velocity, 0, 0, 0;

        Eigen::VectorXd des_theta_dot(6);
        des_theta_dot = J.inverse() * task_des_vel;

        Eigen::VectorXd des_theta_ddot(6);

        des_theta_ddot = J.inverse() * (task_des_acc - J_dot * des_theta_dot);

        Eigen::VectorXd Kp(6), Kv(6);
        Kp << 1000, 3000, 5000, 1000, 200000, 200000;
        Kv << 100, 100, 100, 300, 500, 1000;

        Eigen::VectorXd e = des_theta - curr_theta;
        Eigen::VectorXd dot_e = des_theta_dot - curr_theta_dot;

        Eigen::VectorXd ref_theta_ddot(6);
        ref_theta_ddot = des_theta_ddot + Kv.asDiagonal() * dot_e + Kp.asDiagonal() * e;
        torque = mass_matrix * ref_theta_ddot + corioli_matrix * curr_theta_dot + gravity_matrix;
        // torque = gravity_matrix;

        // auto end_time_calc = std::chrono::steady_clock::now();
        // auto duration_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time_calc - start_time_calc).count();
        // double duration_ms = static_cast<double>(duration_ns) / 1e6; // 나노초를 밀리초로 변환

        // // 걸린 시간 출력
        // std::cout << "While 문이 반복을 완료하는데 걸린 시간: " << duration_ms << " ms" << std::endl;

        // 메시지 퍼블리싱
        auto joint_state_msg = sensor_msgs::msg::JointState();
        joint_state_msg.header.stamp = this->now();
        for (const auto &joint_name : joint_names)
        {
            joint_state_msg.name.push_back(joint_name);
            joint_state_msg.position.push_back(torque[joint_name_to_number[joint_name]]);
        }
        joint_publisher_->publish(joint_state_msg);

        // auto joint_state_msg = sensor_msgs::msg::JointState();
        // joint_state_msg.header.stamp = this->now();
        // for (const auto &joint_name : joint_names)
        // {
        //     joint_state_msg.name.push_back(joint_name);
        //     joint_state_msg.position.push_back(des_theta_dot[joint_name_to_number[joint_name]] + curr_theta[joint_name_to_number[joint_name]]);
        // }
        // joint_publisher_->publish(joint_state_msg);

        // 궤적이 끝에 도달했는지 확인하고 반전
        if (current_time >= trajectory_.final_time_)
        {
            reverse_ = !reverse_;
            // double initial_time = reverse_ ? 5.0 : 0.0;
            // double final_time = reverse_ ? 0.0 : 5.0;
            double initial_pos = reverse_ ? -0.4 : -0.81;
            double final_pos = reverse_ ? -0.81 : -0.4;
            double initial_time = 0.0;
            double final_time = 2.0;

            trajectory_.ChangeTrajectory(
                initial_time, initial_pos, 0.0, 0.0,
                final_time, final_pos, 0.0, 0.0);
            start_time_ = this->now(); // 시작 시간 초기화
        }
    }

    void SolveOptimization(const Eigen::MatrixXd &J, const Eigen::VectorXd &p_dot, Eigen::VectorXd &theta_dot,
                           const Eigen::VectorXd &theta_current,
                           const Eigen::VectorXd &theta_min, const Eigen::VectorXd &theta_max,
                           double epsilon = 1e-4)
    {
        int n = J.cols(); // 관절 개수
        // int m = J.rows(); // 출력 벡터의 수 (엔드 이펙터 속도)

        // P 행렬 (상삼각)
        Eigen::MatrixXd P_dense = J.transpose() * J + epsilon * Eigen::MatrixXd::Identity(n, n);
        for (int i = 0; i < P_dense.rows(); ++i)
        {
            for (int j = 0; j < i; ++j)
            {
                P_dense(i, j) = 0.0; // 대각선 아래 0으로 설정
            }
        }
        Eigen::SparseMatrix<double> P_sparse = P_dense.sparseView();

        // q 벡터 (목표 속도)
        Eigen::VectorXd q = -J.transpose() * p_dot;

        // 제약 조건 행렬 (단위 행렬 사용)
        Eigen::SparseMatrix<double> A_sparse(n, n);
        A_sparse.setIdentity();

        // 각도 제한 계산
        std::vector<c_float> l(n); // 하한
        std::vector<c_float> u(n); // 상한

        for (int i = 0; i < n; ++i)
        {
            l[i] = liegroup::Clamp(theta_min[i] - theta_current[i], -0.5, 0.5); // 각도 제한 하한
            u[i] = liegroup::Clamp(theta_max[i] - theta_current[i], -0.5, 0.5); // 각도 제한 상한
        }

        // CSC 포맷으로 변환
        std::vector<c_float> P_data(P_sparse.valuePtr(), P_sparse.valuePtr() + P_sparse.nonZeros());
        std::vector<c_int> P_indices(P_sparse.innerIndexPtr(), P_sparse.innerIndexPtr() + P_sparse.nonZeros());
        std::vector<c_int> P_pointers(P_sparse.outerIndexPtr(), P_sparse.outerIndexPtr() + P_sparse.outerSize() + 1);

        std::vector<c_float> q_data(q.data(), q.data() + q.size());

        std::vector<c_float> A_data(A_sparse.valuePtr(), A_sparse.valuePtr() + A_sparse.nonZeros());
        std::vector<c_int> A_indices(A_sparse.innerIndexPtr(), A_sparse.innerIndexPtr() + A_sparse.nonZeros());
        std::vector<c_int> A_pointers(A_sparse.outerIndexPtr(), A_sparse.outerIndexPtr() + A_sparse.outerSize() + 1);

        // OSQPData 초기화
        OSQPData data;
        data.n = n; // 변수 개수
        data.m = n; // 제약 조건 개수
        data.P = csc_matrix(n, n, P_sparse.nonZeros(), P_data.data(), P_indices.data(), P_pointers.data());
        data.q = q_data.data();
        data.A = csc_matrix(n, n, n, A_data.data(), A_indices.data(), A_pointers.data());
        data.l = l.data();
        data.u = u.data();

        // OSQPSettings 초기화
        OSQPSettings settings;
        osqp_set_default_settings(&settings);
        settings.verbose = false; // 출력 비활성화

        // OSQPWorkspace 설정
        OSQPWorkspace *work = nullptr;
        c_int status = osqp_setup(&work, &data, &settings);

        if (status != 0)
        {
            std::cerr << "OSQP setup failed with error code: " << status << std::endl;
            return;
        }

        // 문제 풀이
        osqp_solve(work);

        if (work->info->status_val != OSQP_SOLVED)
        {
            std::cerr << "OSQP failed to solve the problem with status: " << work->info->status_val << std::endl;
            osqp_cleanup(work);
            return;
        }

        // 결과 저장
        theta_dot.resize(n);
        for (int i = 0; i < n; ++i)
        {
            theta_dot(i) = work->solution->x[i];
        }

        // 메모리 해제
        osqp_cleanup(work);
    }
    void UpdateForwardKinematics()
    {
        model_frame_matrix.clear();
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

        Eigen::Matrix4d joint_T_1;
        Eigen::Matrix4d joint_T_2;
        Eigen::Matrix4d joint_T_3;
        Eigen::Matrix4d joint_T_4;
        Eigen::Matrix4d joint_T_5;
        Eigen::Matrix4d joint_T_6;

        //  "l_hip_p", "l_hip_r", "l_hip_y", "l_knee_p", "l_ankle_p", "l_ankle_r"
        joint_T_1 << cos(joint_positions[joint_name_to_number["l_hip_y"]]), -sin(joint_positions[joint_name_to_number["l_hip_y"]]), 0, 0,
            sin(joint_positions[joint_name_to_number["l_hip_y"]]), cos(joint_positions[joint_name_to_number["l_hip_y"]]), 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1; // l_hip_y

        joint_T_2 << cos(joint_positions[joint_name_to_number["l_hip_r"]]), -sin(joint_positions[joint_name_to_number["l_hip_r"]]), 0, 0,
            sin(joint_positions[joint_name_to_number["l_hip_r"]]), cos(joint_positions[joint_name_to_number["l_hip_r"]]), 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1; // l_hip_r
        joint_T_3 << cos(joint_positions[joint_name_to_number["l_hip_p"]]), -sin(joint_positions[joint_name_to_number["l_hip_p"]]), 0, 0,
            sin(joint_positions[joint_name_to_number["l_hip_p"]]), cos(joint_positions[joint_name_to_number["l_hip_p"]]), 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1; // l_hip_p
        joint_T_4 << cos(joint_positions[joint_name_to_number["l_knee_p"]]), -sin(joint_positions[joint_name_to_number["l_knee_p"]]), 0, 0,
            sin(joint_positions[joint_name_to_number["l_knee_p"]]), cos(joint_positions[joint_name_to_number["l_knee_p"]]), 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1; // l_knee_p

        joint_T_5 << cos(joint_positions[joint_name_to_number["l_ankle_r"]]), -sin(joint_positions[joint_name_to_number["l_ankle_r"]]), 0, 0,
            sin(joint_positions[joint_name_to_number["l_ankle_r"]]), cos(joint_positions[joint_name_to_number["l_ankle_r"]]), 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1; // l_ankle_r

        joint_T_6 << cos(joint_positions[joint_name_to_number["l_ankle_p"]]), -sin(joint_positions[joint_name_to_number["l_ankle_p"]]), 0, 0,
            sin(joint_positions[joint_name_to_number["l_ankle_p"]]), cos(joint_positions[joint_name_to_number["l_ankle_p"]]), 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1; // l_ankle_p

        Eigen::Matrix4d adjacent_T_1 = offset_joint_0_T_1 * joint_T_1;
        Eigen::Matrix4d adjacent_T_2 = offset_joint_1_T_2 * joint_T_2;
        Eigen::Matrix4d adjacent_T_3 = offset_joint_2_T_3 * joint_T_3;
        Eigen::Matrix4d adjacent_T_4 = offset_joint_3_T_4 * joint_T_4;
        Eigen::Matrix4d adjacent_T_5 = offset_joint_4_T_5 * joint_T_5;
        Eigen::Matrix4d adjacent_T_6 = offset_joint_5_T_6 * joint_T_6;
        Eigen::Matrix4d adjacent_T_7 = offset_joint_6_T_7;

        model_frame_matrix.push_back(adjacent_T_1);
        model_frame_matrix.push_back(adjacent_T_2);
        model_frame_matrix.push_back(adjacent_T_3);
        model_frame_matrix.push_back(adjacent_T_4);
        model_frame_matrix.push_back(adjacent_T_5);
        model_frame_matrix.push_back(adjacent_T_6);
        model_frame_matrix.push_back(adjacent_T_7);
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

    Eigen::MatrixXd ComputeJacobianDot(
        std::function<Eigen::VectorXd(Eigen::VectorXd)> func,
        const Eigen::VectorXd &theta_t,
        const Eigen::VectorXd &theta_t_dt,
        double dt)
    {
        // 시간 t에서의 자코비안
        Eigen::MatrixXd J_t = ComputeJacobian(func, theta_t);

        // 시간 t + dt에서의 자코비안
        Eigen::MatrixXd J_t_dt = ComputeJacobian(func, theta_t_dt);

        // 시간미분 근사
        Eigen::MatrixXd J_dot = (J_t_dt - J_t) / dt;

        return J_dot;
    }
    Eigen::MatrixXd ComputeIntertiaOffset(const Eigen::Matrix4d &transform, const Eigen::MatrixXd &body_inertia)
    {
        Eigen::MatrixXd adjoint_inv_transpose;
        Eigen::MatrixXd inverse_adjoint;

        adjoint_inv_transpose.resize(6, 6);
        inverse_adjoint.resize(6, 6);
        adjoint_inv_transpose = liegroup::LieAlgebra::AdjointTransformInverseTranspose(transform);
        inverse_adjoint = liegroup::LieAlgebra::InverseAdjoint(transform);

        return adjoint_inv_transpose * body_inertia * inverse_adjoint;
    }

    void UpdateBodyTwistData()
    {
        Eigen::VectorXd select_z_joint(6);
        select_z_joint << 0, 0, 0, 0, 0, 1;
        body_twist[0] = liegroup::LieAlgebra::InverseAdjoint(model_frame_matrix[0]) * select_z_joint * joint_velocities[joint_name_to_number["l_hip_y"]];
        body_twist[1] = liegroup::LieAlgebra::InverseAdjoint(model_frame_matrix[1]) * body_twist[0] + select_z_joint * joint_velocities[joint_name_to_number["l_hip_r"]];
        body_twist[2] = liegroup::LieAlgebra::InverseAdjoint(model_frame_matrix[2]) * body_twist[1] + select_z_joint * joint_velocities[joint_name_to_number["l_hip_p"]];
        body_twist[3] = liegroup::LieAlgebra::InverseAdjoint(model_frame_matrix[3]) * body_twist[2] + select_z_joint * joint_velocities[joint_name_to_number["l_knee_p"]];
        body_twist[4] = liegroup::LieAlgebra::InverseAdjoint(model_frame_matrix[4]) * body_twist[3] + select_z_joint * joint_velocities[joint_name_to_number["l_ankle_r"]];
        body_twist[5] = liegroup::LieAlgebra::InverseAdjoint(model_frame_matrix[5]) * body_twist[4] + select_z_joint * joint_velocities[joint_name_to_number["l_ankle_p"]];
        body_twist[6] = liegroup::LieAlgebra::InverseAdjoint(model_frame_matrix[6]) * body_twist[5];

        body_twist_dot[0] = liegroup::LieAlgebra::InverseAdjoint(model_frame_matrix[0]) * select_z_joint * joint_acceleration[joint_name_to_number["l_hip_y"]];
        body_twist_dot[1] = liegroup::LieAlgebra::InverseAdjoint(model_frame_matrix[1]) * body_twist_dot[0] + select_z_joint * joint_acceleration[joint_name_to_number["l_hip_r"]] -
                            liegroup::LieAlgebra::AdOperator(select_z_joint * joint_velocities[joint_name_to_number["l_hip_y"]]) * liegroup::LieAlgebra::InverseAdjoint(model_frame_matrix[1]) * body_twist[0];

        body_twist_dot[2] = liegroup::LieAlgebra::InverseAdjoint(model_frame_matrix[2]) * body_twist_dot[1] + select_z_joint * joint_acceleration[joint_name_to_number["l_hip_p"]] -
                            liegroup::LieAlgebra::AdOperator(select_z_joint * joint_velocities[joint_name_to_number["l_hip_r"]]) * liegroup::LieAlgebra::InverseAdjoint(model_frame_matrix[2]) * body_twist[1];

        body_twist_dot[3] = liegroup::LieAlgebra::InverseAdjoint(model_frame_matrix[3]) * body_twist_dot[2] + select_z_joint * joint_acceleration[joint_name_to_number["l_knee_p"]] -
                            liegroup::LieAlgebra::AdOperator(select_z_joint * joint_velocities[joint_name_to_number["l_hip_p"]]) * liegroup::LieAlgebra::InverseAdjoint(model_frame_matrix[3]) * body_twist[2];

        body_twist_dot[4] = liegroup::LieAlgebra::InverseAdjoint(model_frame_matrix[4]) * body_twist_dot[3] + select_z_joint * joint_acceleration[joint_name_to_number["l_ankle_p"]] -
                            liegroup::LieAlgebra::AdOperator(select_z_joint * joint_velocities[joint_name_to_number["l_knee_p"]]) * liegroup::LieAlgebra::InverseAdjoint(model_frame_matrix[4]) * body_twist[3];

        body_twist_dot[5] = liegroup::LieAlgebra::InverseAdjoint(model_frame_matrix[5]) * body_twist_dot[4] + select_z_joint * joint_acceleration[joint_name_to_number["l_ankle_r"]] -
                            liegroup::LieAlgebra::AdOperator(select_z_joint * joint_velocities[joint_name_to_number["l_ankle_p"]]) * liegroup::LieAlgebra::InverseAdjoint(model_frame_matrix[5]) * body_twist[4];

        body_twist_dot[6] = liegroup::LieAlgebra::InverseAdjoint(model_frame_matrix[6]) * body_twist_dot[5];
    }

    void UpdateBaisMatrix()
    {
        UpdateBodyTwistData();
        bais_matrix[0].block<3, 3>(0, 0) = body_masses[0] * liegroup::LieAlgebra::Ceil3DVectorOperator(body_twist[0].tail(3));
        bais_matrix[0].block<3, 3>(3, 3) = -1 * liegroup::LieAlgebra::Ceil3DVectorOperator(moment_matrix[0] * body_twist[0].tail(3));

        bais_matrix[1].block<3, 3>(0, 0) = body_masses[1] * liegroup::LieAlgebra::Ceil3DVectorOperator(body_twist[1].tail(3));
        bais_matrix[1].block<3, 3>(3, 3) = -1 * liegroup::LieAlgebra::Ceil3DVectorOperator(moment_matrix[1] * body_twist[1].tail(3));

        bais_matrix[2].block<3, 3>(0, 0) = body_masses[2] * liegroup::LieAlgebra::Ceil3DVectorOperator(body_twist[2].tail(3));
        bais_matrix[2].block<3, 3>(3, 3) = -1 * liegroup::LieAlgebra::Ceil3DVectorOperator(moment_matrix[2] * body_twist[2].tail(3));

        bais_matrix[3].block<3, 3>(0, 0) = body_masses[3] * liegroup::LieAlgebra::Ceil3DVectorOperator(body_twist[3].tail(3));
        bais_matrix[3].block<3, 3>(3, 3) = -1 * liegroup::LieAlgebra::Ceil3DVectorOperator(moment_matrix[3] * body_twist[3].tail(3));

        bais_matrix[4].block<3, 3>(0, 0) = body_masses[4] * liegroup::LieAlgebra::Ceil3DVectorOperator(body_twist[4].tail(3));
        bais_matrix[4].block<3, 3>(3, 3) = -1 * liegroup::LieAlgebra::Ceil3DVectorOperator(moment_matrix[4] * body_twist[4].tail(3));

        bais_matrix[5].block<3, 3>(0, 0) = body_masses[5] * liegroup::LieAlgebra::Ceil3DVectorOperator(body_twist[5].tail(3));
        bais_matrix[5].block<3, 3>(3, 3) = -1 * liegroup::LieAlgebra::Ceil3DVectorOperator(moment_matrix[5] * body_twist[5].tail(3));
    }
    Eigen::VectorXd ComputeGravityMatrix()
    {
        Eigen::VectorXd gravity_x(6);
        Eigen::VectorXd gravity_y(6);
        Eigen::VectorXd gravity_z(6);
        gravity_x << 9.81, 0, 0, 0, 0, 0; // 각 원소에 값 설정
        gravity_y << 0, 9.81, 0, 0, 0, 0; // 각 원소에 값 설정
        gravity_z << 0, 0, 9.81, 0, 0, 0; // 각 원소에 값 설정

        Eigen::VectorXd f_1(6);
        Eigen::VectorXd f_2(6);
        Eigen::VectorXd f_3(6);
        Eigen::VectorXd f_4(6);
        Eigen::VectorXd f_5(6);
        Eigen::VectorXd f_6(6);

        f_1 = body_inertia_matrix[0] * (gravity_y);
        f_2 = body_inertia_matrix[1] * (gravity_y * -1.0);
        f_3 = body_inertia_matrix[2] * (gravity_y * -1.0);
        f_4 = body_inertia_matrix[3] * (gravity_y);
        f_5 = body_inertia_matrix[4] * (gravity_y * -1.0);
        f_6 = body_inertia_matrix[5] * (gravity_z);

        Eigen::VectorXd gravity_matrix(6);

        gravity_matrix(0) = e_offset[0].transpose() * (liegroup::LieAlgebra::AdjointTransformInverseTranspose(model_frame_matrix[1]) * f_1 +
                                                       liegroup::LieAlgebra::AdjointTransformInverseTranspose(model_frame_matrix[1] * model_frame_matrix[2]) * f_2 +
                                                       liegroup::LieAlgebra::AdjointTransformInverseTranspose(model_frame_matrix[1] * model_frame_matrix[2] * model_frame_matrix[3]) * f_3 +
                                                       liegroup::LieAlgebra::AdjointTransformInverseTranspose(model_frame_matrix[1] * model_frame_matrix[2] * model_frame_matrix[3] * model_frame_matrix[4]) * f_4 +
                                                       liegroup::LieAlgebra::AdjointTransformInverseTranspose(model_frame_matrix[1] * model_frame_matrix[2] * model_frame_matrix[3] * model_frame_matrix[4] * model_frame_matrix[5]) * f_5 +
                                                       liegroup::LieAlgebra::AdjointTransformInverseTranspose(model_frame_matrix[1] * model_frame_matrix[2] * model_frame_matrix[3] * model_frame_matrix[4] * model_frame_matrix[5] * model_frame_matrix[6]) * f_6);

        gravity_matrix(1) = e_offset[1].transpose() * (liegroup::LieAlgebra::AdjointTransformInverseTranspose(model_frame_matrix[2]) * f_2 +
                                                       liegroup::LieAlgebra::AdjointTransformInverseTranspose(model_frame_matrix[2] * model_frame_matrix[3]) * f_3 +
                                                       liegroup::LieAlgebra::AdjointTransformInverseTranspose(model_frame_matrix[2] * model_frame_matrix[3] * model_frame_matrix[4]) * f_4 +
                                                       liegroup::LieAlgebra::AdjointTransformInverseTranspose(model_frame_matrix[2] * model_frame_matrix[3] * model_frame_matrix[4] * model_frame_matrix[5]) * f_5 +
                                                       liegroup::LieAlgebra::AdjointTransformInverseTranspose(model_frame_matrix[2] * model_frame_matrix[3] * model_frame_matrix[4] * model_frame_matrix[5] * model_frame_matrix[6]) * f_6);

        gravity_matrix(2) = e_offset[2].transpose() * (liegroup::LieAlgebra::AdjointTransformInverseTranspose(model_frame_matrix[3]) * f_3 +
                                                       liegroup::LieAlgebra::AdjointTransformInverseTranspose(model_frame_matrix[3] * model_frame_matrix[4]) * f_4 +
                                                       liegroup::LieAlgebra::AdjointTransformInverseTranspose(model_frame_matrix[3] * model_frame_matrix[4] * model_frame_matrix[5]) * f_5 +
                                                       liegroup::LieAlgebra::AdjointTransformInverseTranspose(model_frame_matrix[3] * model_frame_matrix[4] * model_frame_matrix[5] * model_frame_matrix[6]) * f_6);

        gravity_matrix(3) = e_offset[3].transpose() * (liegroup::LieAlgebra::AdjointTransformInverseTranspose(model_frame_matrix[4]) * f_4 +
                                                       liegroup::LieAlgebra::AdjointTransformInverseTranspose(model_frame_matrix[4] * model_frame_matrix[5]) * f_5 +
                                                       liegroup::LieAlgebra::AdjointTransformInverseTranspose(model_frame_matrix[4] * model_frame_matrix[5] * model_frame_matrix[6]) * f_6);

        gravity_matrix(4) = e_offset[4].transpose() * (liegroup::LieAlgebra::AdjointTransformInverseTranspose(model_frame_matrix[5]) * f_5 +
                                                       liegroup::LieAlgebra::AdjointTransformInverseTranspose(model_frame_matrix[5] * model_frame_matrix[6]) * f_6);

        gravity_matrix(5) = e_offset[5].transpose() * (liegroup::LieAlgebra::AdjointTransformInverseTranspose(model_frame_matrix[6]) * f_6);

        return gravity_matrix;
    }

    Eigen::MatrixXd ComputeCorioliMatrix()
    {
        UpdateBaisMatrix();
        Eigen::MatrixXd corioli_matrix;
        Eigen::MatrixXd c_11;
        c_11.resize(6, 6);
        c_11.setZero();
        Eigen::Matrix4d current_transform = Eigen::Matrix4d::Identity();
        c_11 += bais_matrix[0];
        for (int i = 1; i <= 5; ++i)
        {
            current_transform *= model_frame_matrix[i]; // Accumulate the transforms
            c_11 += ComputeIntertiaOffset(current_transform, bais_matrix[i] - body_inertia_matrix[i - 1] * liegroup::LieAlgebra::AdOperator(body_twist[i - 1].tail(3)));
        }
        Eigen::MatrixXd c_22;
        c_22.resize(6, 6);
        c_22.setZero();
        current_transform = Eigen::Matrix4d::Identity();
        c_22 += bais_matrix[1];
        for (int i = 2; i <= 5; ++i)
        {
            current_transform *= model_frame_matrix[i]; // Accumulate the transforms
            c_22 += ComputeIntertiaOffset(current_transform, bais_matrix[i] - body_inertia_matrix[i - 1] * liegroup::LieAlgebra::AdOperator(body_twist[i - 1].tail(3)));
        }
        Eigen::MatrixXd c_33;
        c_33.resize(6, 6);
        c_33.setZero();
        current_transform = Eigen::Matrix4d::Identity();
        c_33 += bais_matrix[2];
        for (int i = 3; i <= 5; ++i)
        {
            current_transform *= model_frame_matrix[i]; // Accumulate the transforms
            c_33 += ComputeIntertiaOffset(current_transform, bais_matrix[i] - body_inertia_matrix[i - 1] * liegroup::LieAlgebra::AdOperator(body_twist[i - 1].tail(3)));
        }
        Eigen::MatrixXd c_44;
        c_44.resize(6, 6);
        c_44.setZero();
        current_transform = Eigen::Matrix4d::Identity();
        c_44 += bais_matrix[3];
        for (int i = 4; i <= 5; ++i)
        {
            current_transform *= model_frame_matrix[i]; // Accumulate the transforms
            c_44 += ComputeIntertiaOffset(current_transform, bais_matrix[i] - body_inertia_matrix[i - 1] * liegroup::LieAlgebra::AdOperator(body_twist[i - 1].tail(3)));
        }

        Eigen::MatrixXd c_55;
        c_55.resize(6, 6);
        c_55.setZero();
        current_transform = Eigen::Matrix4d::Identity();
        c_55 += bais_matrix[4];
        for (int i = 4; i <= 5; ++i)
        {
            current_transform *= model_frame_matrix[i]; // Accumulate the transforms
            c_55 += ComputeIntertiaOffset(current_transform, bais_matrix[i] - body_inertia_matrix[i - 1] * liegroup::LieAlgebra::AdOperator(body_twist[i - 1].tail(3)));
        }
        Eigen::MatrixXd c_66;
        c_66 = bais_matrix[5];
        corioli_matrix.resize(6, 6);
        corioli_matrix.setZero();

        corioli_matrix(0, 0) = e_offset[0].transpose() * c_11 * e_offset[0];
        corioli_matrix(1, 1) = e_offset[1].transpose() * c_22 * e_offset[1];
        corioli_matrix(2, 2) = e_offset[2].transpose() * c_33 * e_offset[2];
        corioli_matrix(3, 3) = e_offset[3].transpose() * c_44 * e_offset[3];
        corioli_matrix(4, 4) = e_offset[4].transpose() * c_55 * e_offset[4];
        corioli_matrix(5, 5) = e_offset[5].transpose() * c_66 * e_offset[5];

        return corioli_matrix;
    }
    Eigen::MatrixXd ComputeMassMatrix()
    {
        Eigen::MatrixXd mass_matrix;

        Eigen::MatrixXd m_11;
        m_11.resize(6, 6);
        m_11.setZero();
        Eigen::Matrix4d current_transform = Eigen::Matrix4d::Identity(); // Initialize to identity matrix
        for (int i = 1; i <= 5; ++i)
        {
            current_transform *= model_frame_matrix[i]; // Accumulate the transforms
            m_11 += ComputeIntertiaOffset(current_transform, body_inertia_matrix[i]);
        }
        m_11 += body_inertia_matrix_1;

        Eigen::MatrixXd m_22;
        m_22.resize(6, 6);
        m_22.setZero();
        current_transform = Eigen::Matrix4d::Identity(); // Initialize to identity matrix
        for (int i = 2; i <= 5; ++i)
        {
            current_transform *= model_frame_matrix[i]; // Accumulate the transforms
            m_22 += ComputeIntertiaOffset(current_transform, body_inertia_matrix[i]);
        }
        m_22 += body_inertia_matrix_2;

        Eigen::MatrixXd m_33;
        m_33.resize(6, 6);
        m_33.setZero();
        current_transform = Eigen::Matrix4d::Identity(); // Initialize to identity matrix
        for (int i = 3; i <= 5; ++i)
        {
            current_transform *= model_frame_matrix[i]; // Accumulate the transforms
            m_33 += ComputeIntertiaOffset(current_transform, body_inertia_matrix[i]);
        }
        m_33 += body_inertia_matrix_3;

        Eigen::MatrixXd m_44;
        m_44.resize(6, 6);
        m_44.setZero();
        current_transform = Eigen::Matrix4d::Identity(); // Initialize to identity matrix
        for (int i = 4; i <= 5; ++i)
        {
            current_transform *= model_frame_matrix[i]; // Accumulate the transforms
            m_44 += ComputeIntertiaOffset(current_transform, body_inertia_matrix[i]);
        }
        m_44 += body_inertia_matrix_4;

        Eigen::MatrixXd m_55;
        m_55.resize(6, 6);
        m_55.setZero();
        current_transform = Eigen::Matrix4d::Identity(); // Initialize to identity matrix
        for (int i = 5; i <= 5; ++i)
        {
            current_transform *= model_frame_matrix[i]; // Accumulate the transforms
            m_55 += ComputeIntertiaOffset(current_transform, body_inertia_matrix[i]);
        }
        m_55 += body_inertia_matrix_5;
        Eigen::MatrixXd m_66 = body_inertia_matrix_6;

        mass_matrix.resize(6, 6);
        mass_matrix.setZero();

        mass_matrix(0, 0) = e_offset[0].transpose() * m_11 * e_offset[0];
        mass_matrix(1, 1) = e_offset[1].transpose() * m_22 * e_offset[1];
        mass_matrix(2, 2) = e_offset[2].transpose() * m_33 * e_offset[2];
        mass_matrix(3, 3) = e_offset[3].transpose() * m_44 * e_offset[3];
        mass_matrix(4, 4) = e_offset[4].transpose() * m_55 * e_offset[4];
        mass_matrix(5, 5) = e_offset[5].transpose() * m_66 * e_offset[5];

        return mass_matrix;
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

    std::vector<Eigen::VectorXd> body_twist;
    std::vector<Eigen::VectorXd> body_twist_dot;
    std::vector<Eigen::VectorXd> e_offset;
    std::vector<Eigen::Matrix4d> model_frame_matrix;
    Eigen::Matrix4d endpoint_pos;
    Eigen::VectorXd endpoint_vel;
    Eigen::VectorXd endpoint_acc;
    rclcpp::TimerBase::SharedPtr timer_;
    bool joint_states_received = false;
    Eigen::MatrixXd moment_matrix_1, moment_matrix_2, moment_matrix_3, moment_matrix_4, moment_matrix_5, moment_matrix_6;
    std::vector<Eigen::MatrixXd> moment_matrix;
    std::vector<Eigen::MatrixXd> bais_matrix;
    Eigen::MatrixXd body_inertia_matrix_1, body_inertia_matrix_2, body_inertia_matrix_3, body_inertia_matrix_4, body_inertia_matrix_5, body_inertia_matrix_6, body_inertia_matrix_7, body_inertia_matrix_8;
    std::vector<Eigen::MatrixXd> body_inertia_matrix;
    Eigen::Matrix4d inertia_offset_1, inertia_offset_2, inertia_offset_3, inertia_offset_4, inertia_offset_5, inertia_offset_6, inertia_offset_7, inertia_offset_8;
    std::vector<double> body_masses = {0.2, 0.2, 0.2, 0.2, 0.2, 0.5};
    rclcpp::Time start_time_;
    bool reverse_ = false;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LEG>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
