// FirstOrderPolynomialTrajectory.hpp

#ifndef POLYNOMIAL_TRAJECTORY_HPP
#define POLYNOMIAL_TRAJECTORY_HPP

#include <Eigen/Dense>

namespace liegroup
{
    /**
     * @brief 1차(선형) 다항식 궤적 클래스.
     */
    class FirstOrderPolynomialTrajectory
    {
    public:
        /**
         * @brief 기본 생성자.
         * 초기 및 최종 시간과 위치를 0으로 초기화합니다.
         */
        FirstOrderPolynomialTrajectory();

        /**
         * @brief 매개변수가 있는 생성자.
         *
         * @param initial_time 초기 시간.
         * @param initial_pos 초기 위치.
         * @param final_time 최종 시간.
         * @param final_pos 최종 위치.
         *
         * @throws std::invalid_argument final_time이 initial_time보다 작을 경우.
         */
        FirstOrderPolynomialTrajectory(double initial_time, double initial_pos,
                                       double final_time, double final_pos);

        /**
         * @brief 소멸자.
         */
        ~FirstOrderPolynomialTrajectory();

        /**
         * @brief 궤적을 변경합니다.
         *
         * @param final_time 새로운 최종 시간.
         * @param final_pos 새로운 최종 위치.
         *
         * @return true 변경 성공.
         * @return false final_time이 초기_time보다 작을 경우.
         */
        bool ChangeTrajectory(double final_time, double final_pos);

        /**
         * @brief 궤적을 변경합니다.
         *
         * @param initial_time 새로운 초기 시간.
         * @param initial_pos 새로운 초기 위치.
         * @param final_time 새로운 최종 시간.
         * @param final_pos 새로운 최종 위치.
         *
         * @return true 변경 성공.
         * @return false final_time이 initial_time보다 작을 경우.
         */
        bool ChangeTrajectory(double initial_time, double initial_pos,
                              double final_time, double final_pos);

        /**
         * @brief 주어진 시간에서의 위치를 반환합니다.
         *
         * @param time 쿼리 시간.
         * @return double 보간된 위치.
         */
        double GetPosition(double time);

        /**
         * @brief 현재 시간을 설정하고 위치를 업데이트합니다.
         *
         * @param time 설정할 시간.
         */
        void SetTime(double time);

        /**
         * @brief 현재 위치를 반환합니다.
         *
         * @return double 현재 위치.
         */
        double GetPosition() const;

        /**
         * @brief 현재 속도를 반환합니다.
         *
         * @return double 현재 속도.
         */
        double GetVelocity() const;

        /**
         * @brief 현재 가속도를 반환합니다.
         *
         * @return double 현재 가속도 (항상 0).
         */
        double GetAcceleration() const;

        double initial_time_; ///< 초기 시간.
        double initial_pos_;  ///< 초기 위치.

        double final_time_; ///< 최종 시간.
        double final_pos_;  ///< 최종 위치.

        double current_time_; ///< 현재 시간.
        double current_pos_;  ///< 현재 위치.
        double current_vel_;  ///< 현재 속도.
        double current_acc_;  ///< 현재 가속도 (항상 0).


    private:
        
        /**
         * @brief 선형 보간을 위한 속도 계산.
         */
        void CalculateVelocity();
    };

    class ThirdOrderPolynomialTrajectory
    {
    public:
        /**
         * @brief 기본 생성자.
         * 초기 및 최종 시간과 상태를 0으로 초기화합니다.
         */
        ThirdOrderPolynomialTrajectory();

        /**
         * @brief 매개변수가 있는 생성자.
         *
         * @param initial_time 초기 시간.
         * @param initial_pos 초기 위치.
         * @param initial_vel 초기 속도.
         * @param final_time 최종 시간.
         * @param final_pos 최종 위치.
         * @param final_vel 최종 속도.
         *
         * @throws std::invalid_argument final_time이 initial_time보다 작을 경우.
         */
        ThirdOrderPolynomialTrajectory(double initial_time, double initial_pos, double initial_vel,
                                       double final_time, double final_pos, double final_vel);

        /**
         * @brief 소멸자.
         */
        ~ThirdOrderPolynomialTrajectory();

        /**
         * @brief 궤적을 변경합니다.
         *
         * @param final_time 새로운 최종 시간.
         * @param final_pos 새로운 최종 위치.
         * @param final_vel 새로운 최종 속도.
         *
         * @return true 변경 성공.
         * @return false final_time이 초기_time보다 작을 경우.
         */
        bool ChangeTrajectory(double final_time, double final_pos, double final_vel);

        /**
         * @brief 궤적을 변경합니다.
         *
         * @param initial_time 새로운 초기 시간.
         * @param initial_pos 새로운 초기 위치.
         * @param initial_vel 새로운 초기 속도.
         * @param final_time 새로운 최종 시간.
         * @param final_pos 새로운 최종 위치.
         * @param final_vel 새로운 최종 속도.
         *
         * @return true 변경 성공.
         * @return false final_time이 initial_time보다 작을 경우.
         */
        bool ChangeTrajectory(double initial_time, double initial_pos, double initial_vel,
                              double final_time, double final_pos, double final_vel);

        /**
         * @brief 주어진 시간에서의 위치를 반환합니다.
         *
         * @param time 쿼리 시간.
         * @return double 보간된 위치.
         */
        double GetPosition(double time);

        /**
         * @brief 주어진 시간에서의 속도를 반환합니다.
         *
         * @param time 쿼리 시간.
         * @return double 보간된 속도.
         */
        double GetVelocity(double time);

        /**
         * @brief 주어진 시간에서의 가속도를 반환합니다.
         *
         * @param time 쿼리 시간.
         * @return double 보간된 가속도.
         */
        double GetAcceleration(double time);

        /**
         * @brief 현재 시간을 설정하고 위치, 속도, 가속도를 업데이트합니다.
         *
         * @param time 설정할 시간.
         */
        void SetTime(double time);

        /**
         * @brief 현재 위치를 반환합니다.
         *
         * @return double 현재 위치.
         */
        double GetPosition() const;

        /**
         * @brief 현재 속도를 반환합니다.
         *
         * @return double 현재 속도.
         */
        double GetVelocity() const;

        /**
         * @brief 현재 가속도를 반환합니다.
         *
         * @return double 현재 가속도.
         */
        double GetAcceleration() const;

        double initial_time_; ///< 초기 시간.
        double initial_pos_;  ///< 초기 위치.
        double initial_vel_;  ///< 초기 속도.

        double final_time_; ///< 최종 시간.
        double final_pos_;  ///< 최종 위치.
        double final_vel_;  ///< 최종 속도.

        double current_time_; ///< 현재 시간.
        double current_pos_;  ///< 현재 위치.
        double current_vel_;  ///< 현재 속도.
        double current_acc_;  ///< 현재 가속도.

    private:
    
        Eigen::Vector4d coeff_; ///< 3차 다항식 계수 [a3, a2, a1, a0]^T.

        /**
         * @brief 3차 다항식 계수를 계산합니다.
         */
        void CalculateCoefficients();
    };

    /**
     * @brief 5차 다항식 궤적 클래스.
     */
    class FifthOrderPolynomialTrajectory
    {
    public:
        /**
         * @brief 기본 생성자.
         * 초기 및 최종 시간과 상태를 0으로 초기화합니다.
         */
        FifthOrderPolynomialTrajectory();

        /**
         * @brief 매개변수가 있는 생성자.
         *
         * @param initial_time 초기 시간.
         * @param initial_pos 초기 위치.
         * @param initial_vel 초기 속도.
         * @param initial_acc 초기 가속도.
         * @param final_time 최종 시간.
         * @param final_pos 최종 위치.
         * @param final_vel 최종 속도.
         * @param final_acc 최종 가속도.
         *
         * @throws std::invalid_argument final_time이 initial_time보다 작을 경우.
         */
        FifthOrderPolynomialTrajectory(double initial_time, double initial_pos, double initial_vel, double initial_acc,
                                       double final_time, double final_pos, double final_vel, double final_acc);

        /**
         * @brief 소멸자.
         */
        ~FifthOrderPolynomialTrajectory();

        /**
         * @brief 궤적을 변경합니다.
         *
         * @param final_pos 새로운 최종 위치.
         * @param final_vel 새로운 최종 속도.
         * @param final_acc 새로운 최종 가속도.
         *
         * @return true 변경 성공.
         * @return false 변경 실패.
         */
        bool ChangeTrajectory(double final_pos, double final_vel, double final_acc);

        /**
         * @brief 궤적을 변경합니다.
         *
         * @param final_time 새로운 최종 시간.
         * @param final_pos 새로운 최종 위치.
         * @param final_vel 새로운 최종 속도.
         * @param final_acc 새로운 최종 가속도.
         *
         * @return true 변경 성공.
         * @return false final_time이 초기_time보다 작을 경우.
         */
        bool ChangeTrajectory(double final_time, double final_pos, double final_vel, double final_acc);

        /**
         * @brief 궤적을 변경합니다.
         *
         * @param initial_time 새로운 초기 시간.
         * @param initial_pos 새로운 초기 위치.
         * @param initial_vel 새로운 초기 속도.
         * @param initial_acc 새로운 초기 가속도.
         * @param final_time 새로운 최종 시간.
         * @param final_pos 새로운 최종 위치.
         * @param final_vel 새로운 최종 속도.
         * @param final_acc 새로운 최종 가속도.
         *
         * @return true 변경 성공.
         * @return false final_time이 initial_time보다 작을 경우.
         */
        bool ChangeTrajectory(double initial_time, double initial_pos, double initial_vel, double initial_acc,
                              double final_time, double final_pos, double final_vel, double final_acc);

        /**
         * @brief 주어진 시간에서의 위치를 반환합니다.
         *
         * @param time 쿼리 시간.
         * @return double 보간된 위치.
         */
        double GetPosition(double time);

        /**
         * @brief 주어진 시간에서의 속도를 반환합니다.
         *
         * @param time 쿼리 시간.
         * @return double 보간된 속도.
         */
        double GetVelocity(double time);

        /**
         * @brief 주어진 시간에서의 가속도를 반환합니다.
         *
         * @param time 쿼리 시간.
         * @return double 보간된 가속도.
         */
        double GetAcceleration(double time);

        /**
         * @brief 현재 시간을 설정하고 위치, 속도, 가속도를 업데이트합니다.
         *
         * @param time 설정할 시간.
         */
        void SetTime(double time);

        /**
         * @brief 현재 위치를 반환합니다.
         *
         * @return double 현재 위치.
         */
        double GetPosition() const;

        /**
         * @brief 현재 속도를 반환합니다.
         *
         * @return double 현재 속도.
         */
        double GetVelocity() const;

        /**
         * @brief 현재 가속도를 반환합니다.
         *
         * @return double 현재 가속도.
         */
        double GetAcceleration() const;

        double initial_time_; ///< 초기 시간.
        double initial_pos_;  ///< 초기 위치.
        double initial_vel_;  ///< 초기 속도.
        double initial_acc_;  ///< 초기 가속도.

        double final_time_; ///< 최종 시간.
        double final_pos_;  ///< 최종 위치.
        double final_vel_;  ///< 최종 속도.
        double final_acc_;  ///< 최종 가속도.

        double current_time_; ///< 현재 시간.
        double current_pos_;  ///< 현재 위치.
        double current_vel_;  ///< 현재 속도.
        double current_acc_;  ///< 현재 가속도.

    private:
        
        Eigen::VectorXd position_coeff_;     ///< 5차 다항식 위치 계수 [a5, a4, a3, a2, a1, a0]^T.
        Eigen::VectorXd velocity_coeff_;     ///< 4차 다항식 속도 계수 [b4, b3, b2, b1, b0]^T.
        Eigen::VectorXd acceleration_coeff_; ///< 3차 다항식 가속도 계수 [c3, c2, c1, c0]^T.

        Eigen::VectorXd time_variables_; ///< 시간 변수 벡터 [t^5, t^4, t^3, t^2, t, 1]^T.

        /**
         * @brief 5차 다항식 계수를 계산합니다.
         */
        void CalculateCoefficients();
    };

    /**
     * @brief 7차 다항식 궤적 클래스.
     */
    class SeventhOrderPolynomialTrajectory
    {
    public:
        /**
         * @brief 기본 생성자.
         * 초기 및 최종 시간과 상태를 0으로 초기화합니다.
         */
        SeventhOrderPolynomialTrajectory();

        /**
         * @brief 매개변수가 있는 생성자.
         *
         * @param initial_time 초기 시간.
         * @param initial_pos 초기 위치.
         * @param initial_vel 초기 속도.
         * @param initial_acc 초기 가속도.
         * @param initial_jerk 초기 젭.
         * @param final_time 최종 시간.
         * @param final_pos 최종 위치.
         * @param final_vel 최종 속도.
         * @param final_acc 최종 가속도.
         * @param final_jerk 최종 젭.
         *
         * @throws std::invalid_argument final_time이 initial_time보다 작을 경우.
         */
        SeventhOrderPolynomialTrajectory(double initial_time, double initial_pos, double initial_vel, double initial_acc, double initial_jerk,
                                         double final_time, double final_pos, double final_vel, double final_acc, double final_jerk);

        /**
         * @brief 소멸자.
         */
        ~SeventhOrderPolynomialTrajectory();

        /**
         * @brief 궤적을 변경합니다.
         *
         * @param final_pos 새로운 최종 위치.
         * @param final_vel 새로운 최종 속도.
         * @param final_acc 새로운 최종 가속도.
         * @param final_jerk 새로운 최종 젭.
         *
         * @return true 변경 성공.
         * @return false 변경 실패.
         */
        bool ChangeTrajectory(double final_pos, double final_vel, double final_acc, double final_jerk);

        /**
         * @brief 궤적을 변경합니다.
         *
         * @param final_time 새로운 최종 시간.
         * @param final_pos 새로운 최종 위치.
         * @param final_vel 새로운 최종 속도.
         * @param final_acc 새로운 최종 가속도.
         * @param final_jerk 새로운 최종 젭.
         *
         * @return true 변경 성공.
         * @return false final_time이 초기_time보다 작을 경우.
         */
        bool ChangeTrajectory(double final_time, double final_pos, double final_vel, double final_acc, double final_jerk);

        /**
         * @brief 궤적을 변경합니다.
         *
         * @param initial_time 새로운 초기 시간.
         * @param initial_pos 새로운 초기 위치.
         * @param initial_vel 새로운 초기 속도.
         * @param initial_acc 새로운 초기 가속도.
         * @param initial_jerk 새로운 초기 젭.
         * @param final_time 새로운 최종 시간.
         * @param final_pos 새로운 최종 위치.
         * @param final_vel 새로운 최종 속도.
         * @param final_acc 새로운 최종 가속도.
         * @param final_jerk 새로운 최종 젭.
         *
         * @return true 변경 성공.
         * @return false final_time이 initial_time보다 작을 경우.
         */
        bool ChangeTrajectory(double initial_time, double initial_pos, double initial_vel, double initial_acc, double initial_jerk,
                              double final_time, double final_pos, double final_vel, double final_acc, double final_jerk);

        /**
         * @brief 주어진 시간에서의 위치를 반환합니다.
         *
         * @param time 쿼리 시간.
         * @return double 보간된 위치.
         */
        double GetPosition(double time);

        /**
         * @brief 주어진 시간에서의 속도를 반환합니다.
         *
         * @param time 쿼리 시간.
         * @return double 보간된 속도.
         */
        double GetVelocity(double time);

        /**
         * @brief 주어진 시간에서의 가속도를 반환합니다.
         *
         * @param time 쿼리 시간.
         * @return double 보간된 가속도.
         */
        double GetAcceleration(double time);

        /**
         * @brief 주어진 시간에서의 젭을 반환합니다.
         *
         * @param time 쿼리 시간.
         * @return double 보간된 젭.
         */
        double GetJerk(double time);

        /**
         * @brief 현재 시간을 설정하고 위치, 속도, 가속도, 젭을 업데이트합니다.
         *
         * @param time 설정할 시간.
         */
        void SetTime(double time);

        /**
         * @brief 현재 위치를 반환합니다.
         *
         * @return double 현재 위치.
         */
        double GetPosition() const;

        /**
         * @brief 현재 속도를 반환합니다.
         *
         * @return double 현재 속도.
         */
        double GetVelocity() const;

        /**
         * @brief 현재 가속도를 반환합니다.
         *
         * @return double 현재 가속도.
         */
        double GetAcceleration() const;

        /**
         * @brief 현재 젭을 반환합니다.
         *
         * @return double 현재 젭.
         */
        double GetJerk() const;

        double initial_time_; ///< 초기 시간.
        double initial_pos_;  ///< 초기 위치.
        double initial_vel_;  ///< 초기 속도.
        double initial_acc_;  ///< 초기 가속도.
        double initial_jerk_; ///< 초기 젭.

        double final_time_; ///< 최종 시간.
        double final_pos_;  ///< 최종 위치.
        double final_vel_;  ///< 최종 속도.
        double final_acc_;  ///< 최종 가속도.
        double final_jerk_; ///< 최종 젭.

        double current_time_; ///< 현재 시간.
        double current_pos_;  ///< 현재 위치.
        double current_vel_;  ///< 현재 속도.
        double current_acc_;  ///< 현재 가속도.
        double current_jerk_; ///< 현재 젭.

    private:
        
        Eigen::VectorXd position_coeff_;     ///< 7차 다항식 위치 계수 [a7, a6, a5, a4, a3, a2, a1, a0]^T.
        Eigen::VectorXd velocity_coeff_;     ///< 6차 다항식 속도 계수 [b6, b5, b4, b3, b2, b1, b0]^T.
        Eigen::VectorXd acceleration_coeff_; ///< 5차 다항식 가속도 계수 [c5, c4, c3, c2, c1, c0]^T.
        Eigen::VectorXd jerk_coeff_;         ///< 4차 다항식 젭 계수 [d4, d3, d2, d1, d0]^T.

        Eigen::VectorXd time_variables_; ///< 시간 변수 벡터 [t^7, t^6, t^5, t^4, t^3, t^2, t, 1]^T.

        /**
         * @brief 7차 다항식 계수를 계산합니다.
         */
        void CalculateCoefficients();
    };

    /**
     * @brief 9차 다항식 궤적 클래스.
     */
    class NinthOrderPolynomialTrajectory
    {
    public:
        /**
         * @brief 기본 생성자.
         * 모든 멤버 변수를 0으로 초기화합니다.
         */
        NinthOrderPolynomialTrajectory();

        /**
         * @brief 매개변수가 있는 생성자.
         *
         * @param initial_time 초기 시간.
         * @param initial_pos 초기 위치.
         * @param initial_vel 초기 속도.
         * @param initial_acc 초기 가속도.
         * @param initial_jerk 초기 젭.
         * @param initial_snap 초기 스냅.
         * @param final_time 최종 시간.
         * @param final_pos 최종 위치.
         * @param final_vel 최종 속도.
         * @param final_acc 최종 가속도.
         * @param final_jerk 최종 젭.
         * @param final_snap 최종 스냅.
         *
         * @throws std::invalid_argument final_time이 initial_time보다 작거나 같을 경우.
         */
        NinthOrderPolynomialTrajectory(double initial_time, double initial_pos, double initial_vel, double initial_acc, double initial_jerk, double initial_snap,
                                       double final_time, double final_pos, double final_vel, double final_acc, double final_jerk, double final_snap);

        /**
         * @brief 소멸자.
         */
        ~NinthOrderPolynomialTrajectory();

        /**
         * @brief 궤적을 변경합니다.
         *
         * @param final_pos 새로운 최종 위치.
         * @param final_vel 새로운 최종 속도.
         * @param final_acc 새로운 최종 가속도.
         * @param final_jerk 새로운 최종 젭.
         * @param final_snap 새로운 최종 스냅.
         *
         * @return true 변경 성공.
         * @return false 변경 실패.
         */
        bool ChangeTrajectory(double final_pos, double final_vel, double final_acc, double final_jerk, double final_snap);

        /**
         * @brief 최종 시간과 최종 상태를 변경합니다.
         *
         * @param final_time 새로운 최종 시간.
         * @param final_pos 새로운 최종 위치.
         * @param final_vel 새로운 최종 속도.
         * @param final_acc 새로운 최종 가속도.
         * @param final_jerk 새로운 최종 젭.
         * @param final_snap 새로운 최종 스냅.
         *
         * @return true 변경 성공.
         * @return false final_time이 initial_time보다 작거나 같을 경우.
         */
        bool ChangeTrajectory(double final_time, double final_pos, double final_vel, double final_acc, double final_jerk, double final_snap);

        /**
         * @brief 초기 및 최종 상태를 모두 변경합니다.
         *
         * @param initial_time 새로운 초기 시간.
         * @param initial_pos 새로운 초기 위치.
         * @param initial_vel 새로운 초기 속도.
         * @param initial_acc 새로운 초기 가속도.
         * @param initial_jerk 새로운 초기 젭.
         * @param initial_snap 새로운 초기 스냅.
         * @param final_time 새로운 최종 시간.
         * @param final_pos 새로운 최종 위치.
         * @param final_vel 새로운 최종 속도.
         * @param final_acc 새로운 최종 가속도.
         * @param final_jerk 새로운 최종 젭.
         * @param final_snap 새로운 최종 스냅.
         *
         * @return true 변경 성공.
         * @return false final_time이 initial_time보다 작거나 같을 경우.
         */
        bool ChangeTrajectory(double initial_time, double initial_pos, double initial_vel, double initial_acc, double initial_jerk, double initial_snap,
                              double final_time, double final_pos, double final_vel, double final_acc, double final_jerk, double final_snap);

        /**
         * @brief 주어진 시간에서의 위치를 반환합니다.
         *
         * @param time 쿼리 시간.
         * @return double 보간된 위치.
         */
        double GetPosition(double time);

        /**
         * @brief 주어진 시간에서의 속도를 반환합니다.
         *
         * @param time 쿼리 시간.
         * @return double 보간된 속도.
         */
        double GetVelocity(double time);

        /**
         * @brief 주어진 시간에서의 가속도를 반환합니다.
         *
         * @param time 쿼리 시간.
         * @return double 보간된 가속도.
         */
        double GetAcceleration(double time);

        /**
         * @brief 주어진 시간에서의 젭을 반환합니다.
         *
         * @param time 쿼리 시간.
         * @return double 보간된 젭.
         */
        double GetJerk(double time);

        /**
         * @brief 주어진 시간에서의 스냅을 반환합니다.
         *
         * @param time 쿼리 시간.
         * @return double 보간된 스냅.
         */
        double GetSnap(double time);

        /**
         * @brief 현재 시간을 설정하고 위치, 속도, 가속도, 젭, 스냅을 업데이트합니다.
         *
         * @param time 설정할 시간.
         */
        void SetTime(double time);

        /**
         * @brief 현재 위치를 반환합니다.
         *
         * @return double 현재 위치.
         */
        double GetPosition() const;

        /**
         * @brief 현재 속도를 반환합니다.
         *
         * @return double 현재 속도.
         */
        double GetVelocity() const;

        /**
         * @brief 현재 가속도를 반환합니다.
         *
         * @return double 현재 가속도.
         */
        double GetAcceleration() const;

        /**
         * @brief 현재 젭을 반환합니다.
         *
         * @return double 현재 젭.
         */
        double GetJerk() const;

        /**
         * @brief 현재 스냅을 반환합니다.
         *
         * @return double 현재 스냅.
         */
        double GetSnap() const;

        double initial_time_, initial_pos_, initial_vel_, initial_acc_, initial_jerk_, initial_snap_;
        double final_time_, final_pos_, final_vel_, final_acc_, final_jerk_, final_snap_;
        double current_time_, current_pos_, current_vel_, current_acc_, current_jerk_, current_snap_;

    private:
        
        Eigen::VectorXd position_coeff_;     ///< 9차 다항식 위치 계수 [a9, a8, ..., a0]^T.
        Eigen::VectorXd velocity_coeff_;     ///< 8차 다항식 속도 계수 [b8, b7, ..., b0]^T.
        Eigen::VectorXd acceleration_coeff_; ///< 7차 다항식 가속도 계수 [c7, c6, ..., c0]^T.
        Eigen::VectorXd jerk_coeff_;         ///< 6차 다항식 젭 계수 [d6, d5, ..., d0]^T.
        Eigen::VectorXd snap_coeff_;         ///< 5차 다항식 스냅 계수 [e5, e4, ..., e0]^T.
        Eigen::VectorXd time_variables_;     ///< 시간 변수 벡터 [t^9, t^8, ..., t, 1]^T.

        /**
         * @brief 9차 다항식 계수를 계산합니다.
         */
        void CalculateCoefficients();
    };
}

#endif // FIRST_ORDER_POLYNOMIAL_TRAJECTORY_HPP
