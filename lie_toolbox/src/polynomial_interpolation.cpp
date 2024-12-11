#include "lie_toolbox/polynomial_interpolation.hpp"

using namespace liegroup;

// 기본 생성자 구현
FirstOrderPolynomialTrajectory::FirstOrderPolynomialTrajectory()
    : initial_time_(0.0),
      initial_pos_(0.0),
      final_time_(0.0),
      final_pos_(0.0),
      current_time_(0.0),
      current_pos_(0.0),
      current_vel_(0.0),
      current_acc_(0.0)
{
    // 선형 궤적에서는 가속도가 항상 0입니다.
}

// 매개변수가 있는 생성자 구현
FirstOrderPolynomialTrajectory::FirstOrderPolynomialTrajectory(double initial_time, double initial_pos,
                                                               double final_time, double final_pos)
    : initial_time_(initial_time),
      initial_pos_(initial_pos),
      final_time_(final_time),
      final_pos_(final_pos),
      current_time_(initial_time),
      current_pos_(initial_pos),
      current_vel_(0.0),
      current_acc_(0.0)
{
    if (final_time_ < initial_time_)
    {
        throw std::invalid_argument("final_time은 initial_time보다 작을 수 없습니다.");
    }
    CalculateVelocity();
}

// 소멸자 구현
FirstOrderPolynomialTrajectory::~FirstOrderPolynomialTrajectory() {}

// 궤적 변경: 최종 시간과 위치만 변경
bool FirstOrderPolynomialTrajectory::ChangeTrajectory(double final_time, double final_pos)
{
    if (final_time < initial_time_)
    {
        return false;
    }

    final_time_ = final_time;
    final_pos_ = final_pos;

    CalculateVelocity();

    return true;
}

// 궤적 변경: 초기 시간과 위치, 최종 시간과 위치를 모두 변경
bool FirstOrderPolynomialTrajectory::ChangeTrajectory(double initial_time, double initial_pos,
                                                      double final_time, double final_pos)
{
    if (final_time < initial_time)
    {
        return false;
    }

    initial_time_ = initial_time;
    initial_pos_ = initial_pos;
    final_time_ = final_time;
    final_pos_ = final_pos;

    current_time_ = initial_time_;
    current_pos_ = initial_pos_;

    CalculateVelocity();

    return true;
}

// 주어진 시간에서의 위치를 반환
double FirstOrderPolynomialTrajectory::GetPosition(double time)
{
    if (time >= final_time_)
    {
        current_time_ = final_time_;
        current_pos_ = final_pos_;
        current_vel_ = (final_pos_ - initial_pos_) / (final_time_ - initial_time_);
        current_acc_ = 0.0;
        return final_pos_;
    }
    else if (time <= initial_time_)
    {
        current_time_ = initial_time_;
        current_pos_ = initial_pos_;
        current_vel_ = 0.0;
        current_acc_ = 0.0;
        return initial_pos_;
    }
    else
    {
        double s = (time - initial_time_) / (final_time_ - initial_time_);
        current_time_ = time;
        current_pos_ = (1.0 - s) * initial_pos_ + s * final_pos_;
        current_vel_ = (final_pos_ - initial_pos_) / (final_time_ - initial_time_);
        current_acc_ = 0.0;
        return current_pos_;
    }
}

// 현재 시간을 설정하고 위치를 업데이트
void FirstOrderPolynomialTrajectory::SetTime(double time)
{
    GetPosition(time);
}

// 현재 위치를 반환
double FirstOrderPolynomialTrajectory::GetPosition() const
{
    return current_pos_;
}

// 현재 속도를 반환
double FirstOrderPolynomialTrajectory::GetVelocity() const
{
    return current_vel_;
}

// 현재 가속도를 반환 (항상 0)
double FirstOrderPolynomialTrajectory::GetAcceleration() const
{
    return current_acc_;
}

// 선형 보간을 위한 속도 계산
void FirstOrderPolynomialTrajectory::CalculateVelocity()
{
    if (final_time_ != initial_time_)
    {
        current_vel_ = (final_pos_ - initial_pos_) / (final_time_ - initial_time_);
    }
    else
    {
        current_vel_ = 0.0;
    }
}

// 기본 생성자 구현
ThirdOrderPolynomialTrajectory::ThirdOrderPolynomialTrajectory()
    : initial_time_(0.0),
      initial_pos_(0.0),
      initial_vel_(0.0),
      final_time_(0.0),
      final_pos_(0.0),
      final_vel_(0.0),
      current_time_(0.0),
      current_pos_(0.0),
      current_vel_(0.0),
      current_acc_(0.0),
      coeff_(Eigen::Vector4d::Zero())
{
    // 초기화 완료
}

// 매개변수가 있는 생성자 구현
ThirdOrderPolynomialTrajectory::ThirdOrderPolynomialTrajectory(double initial_time, double initial_pos, double initial_vel,
                                                               double final_time, double final_pos, double final_vel)
    : initial_time_(initial_time),
      initial_pos_(initial_pos),
      initial_vel_(initial_vel),
      final_time_(final_time),
      final_pos_(final_pos),
      final_vel_(final_vel),
      current_time_(initial_time),
      current_pos_(initial_pos),
      current_vel_(initial_vel),
      current_acc_(0.0),
      coeff_(Eigen::Vector4d::Zero())
{
    if (final_time_ < initial_time_)
    {
        throw std::invalid_argument("final_time은 initial_time보다 작을 수 없습니다.");
    }
    CalculateCoefficients();
}

// 소멸자 구현
ThirdOrderPolynomialTrajectory::~ThirdOrderPolynomialTrajectory() {}

// 궤적 변경: 최종 시간, 위치, 속도만 변경
bool ThirdOrderPolynomialTrajectory::ChangeTrajectory(double final_time, double final_pos, double final_vel)
{
    if (final_time < initial_time_)
    {
        return false;
    }

    final_time_ = final_time;
    final_pos_ = final_pos;
    final_vel_ = final_vel;

    CalculateCoefficients();

    return true;
}

// 궤적 변경: 초기 시간, 위치, 속도와 최종 시간, 위치, 속도 모두 변경
bool ThirdOrderPolynomialTrajectory::ChangeTrajectory(double initial_time, double initial_pos, double initial_vel,
                                                      double final_time, double final_pos, double final_vel)
{
    if (final_time < initial_time)
    {
        return false;
    }

    initial_time_ = initial_time;
    initial_pos_ = initial_pos;
    initial_vel_ = initial_vel;
    final_time_ = final_time;
    final_pos_ = final_pos;
    final_vel_ = final_vel;

    current_time_ = initial_time_;
    current_pos_ = initial_pos_;
    current_vel_ = initial_vel_;

    CalculateCoefficients();

    return true;
}

// 3차 다항식 계수를 계산하는 함수
void ThirdOrderPolynomialTrajectory::CalculateCoefficients()
{
    double t0 = initial_time_;
    double tf = final_time_;
    double p0 = initial_pos_;
    double pf = final_pos_;
    double v0 = initial_vel_;
    double vf = final_vel_;

    // 3차 다항식: p(t) = a3*t^3 + a2*t^2 + a1*t + a0
    // 경계 조건:
    // p(t0) = p0
    // p'(t0) = v0
    // p(tf) = pf
    // p'(tf) = vf

    Eigen::Matrix4d time_mat;
    time_mat << pow(t0, 3), pow(t0, 2), t0, 1.0,
        3 * pow(t0, 2), 2 * t0, 1.0, 0.0,
        pow(tf, 3), pow(tf, 2), tf, 1.0,
        3 * pow(tf, 2), 2 * tf, 1.0, 0.0;

    Eigen::Vector4d conditions;
    conditions << p0, v0, pf, vf;

    // Solve for coefficients: a = time_mat.inverse() * conditions
    coeff_ = time_mat.inverse() * conditions;
}

// 주어진 시간에서의 위치를 반환
double ThirdOrderPolynomialTrajectory::GetPosition(double time)
{
    if (time >= final_time_)
    {
        current_time_ = final_time_;
        current_pos_ = final_pos_;
        current_vel_ = final_vel_;
        current_acc_ = 0.0;
        return final_pos_;
    }
    else if (time <= initial_time_)
    {
        current_time_ = initial_time_;
        current_pos_ = initial_pos_;
        current_vel_ = initial_vel_;
        current_acc_ = 0.0;
        return initial_pos_;
    }
    else
    {
        double t = time;
        current_time_ = t;
        current_pos_ = coeff_(0) * pow(t, 3) + coeff_(1) * pow(t, 2) + coeff_(2) * t + coeff_(3);
        current_vel_ = 3 * coeff_(0) * pow(t, 2) + 2 * coeff_(1) * t + coeff_(2);
        current_acc_ = 6 * coeff_(0) * t + 2 * coeff_(1);
        return current_pos_;
    }
}

// 주어진 시간에서의 속도를 반환
double ThirdOrderPolynomialTrajectory::GetVelocity(double time)
{
    if (time >= final_time_)
    {
        current_time_ = final_time_;
        current_vel_ = final_vel_;
        return final_vel_;
    }
    else if (time <= initial_time_)
    {
        current_time_ = initial_time_;
        current_vel_ = initial_vel_;
        return initial_vel_;
    }
    else
    {
        double t = time;
        current_time_ = t;
        current_vel_ = 3 * coeff_(0) * pow(t, 2) + 2 * coeff_(1) * t + coeff_(2);
        current_acc_ = 6 * coeff_(0) * t + 2 * coeff_(1);
        current_pos_ = coeff_(0) * pow(t, 3) + coeff_(1) * pow(t, 2) + coeff_(2) * t + coeff_(3);
        return current_vel_;
    }
}

// 주어진 시간에서의 가속도를 반환
double ThirdOrderPolynomialTrajectory::GetAcceleration(double time)
{
    if (time >= final_time_)
    {
        current_time_ = final_time_;
        current_acc_ = 0.0;
        return 0.0;
    }
    else if (time <= initial_time_)
    {
        current_time_ = initial_time_;
        current_acc_ = 0.0;
        return 0.0;
    }
    else
    {
        double t = time;
        current_time_ = t;
        current_acc_ = 6 * coeff_(0) * t + 2 * coeff_(1);
        current_vel_ = 3 * coeff_(0) * pow(t, 2) + 2 * coeff_(1) * t + coeff_(2);
        current_pos_ = coeff_(0) * pow(t, 3) + coeff_(1) * pow(t, 2) + coeff_(2) * t + coeff_(3);
        return current_acc_;
    }
}

// 현재 시간을 설정하고 위치, 속도, 가속도를 업데이트
void ThirdOrderPolynomialTrajectory::SetTime(double time)
{
    GetPosition(time);
}

// 현재 위치를 반환
double ThirdOrderPolynomialTrajectory::GetPosition() const
{
    return current_pos_;
}

// 현재 속도를 반환
double ThirdOrderPolynomialTrajectory::GetVelocity() const
{
    return current_vel_;
}

// 현재 가속도를 반환
double ThirdOrderPolynomialTrajectory::GetAcceleration() const
{
    return current_acc_;
}

// 기본 생성자 구현
FifthOrderPolynomialTrajectory::FifthOrderPolynomialTrajectory()
    : initial_time_(0.0),
      initial_pos_(0.0),
      initial_vel_(0.0),
      initial_acc_(0.0),
      final_time_(0.0),
      final_pos_(0.0),
      final_vel_(0.0),
      final_acc_(0.0),
      current_time_(0.0),
      current_pos_(0.0),
      current_vel_(0.0),
      current_acc_(0.0),
      position_coeff_(Eigen::VectorXd::Zero(6)),
      velocity_coeff_(Eigen::VectorXd::Zero(5)),
      acceleration_coeff_(Eigen::VectorXd::Zero(4)),
      time_variables_(Eigen::VectorXd::Zero(6))
{
    // 초기화 완료
}

// 매개변수가 있는 생성자 구현
FifthOrderPolynomialTrajectory::FifthOrderPolynomialTrajectory(double initial_time, double initial_pos, double initial_vel, double initial_acc,
                                                               double final_time, double final_pos, double final_vel, double final_acc)
    : initial_time_(initial_time),
      initial_pos_(initial_pos),
      initial_vel_(initial_vel),
      initial_acc_(initial_acc),
      final_time_(final_time),
      final_pos_(final_pos),
      final_vel_(final_vel),
      final_acc_(final_acc),
      current_time_(initial_time),
      current_pos_(initial_pos),
      current_vel_(initial_vel),
      current_acc_(initial_acc),
      position_coeff_(Eigen::VectorXd::Zero(6)),
      velocity_coeff_(Eigen::VectorXd::Zero(5)),
      acceleration_coeff_(Eigen::VectorXd::Zero(4)),
      time_variables_(Eigen::VectorXd::Zero(6))
{
    if (final_time_ < initial_time_)
    {
        throw std::invalid_argument("final_time은 initial_time보다 작을 수 없습니다.");
    }
    CalculateCoefficients();
}

// 소멸자 구현
FifthOrderPolynomialTrajectory::~FifthOrderPolynomialTrajectory() {}

// 궤적 변경: 최종 위치, 속도, 가속도만 변경
bool FifthOrderPolynomialTrajectory::ChangeTrajectory(double final_pos, double final_vel, double final_acc)
{
    final_pos_ = final_pos;
    final_vel_ = final_vel;
    final_acc_ = final_acc;

    CalculateCoefficients();

    return true;
}

// 궤적 변경: 최종 시간, 위치, 속도, 가속도 변경
bool FifthOrderPolynomialTrajectory::ChangeTrajectory(double final_time, double final_pos, double final_vel, double final_acc)
{
    if (final_time < initial_time_)
    {
        return false;
    }

    final_time_ = final_time;
    final_pos_ = final_pos;
    final_vel_ = final_vel;
    final_acc_ = final_acc;

    CalculateCoefficients();

    return true;
}

// 궤적 변경: 초기 시간, 위치, 속도, 가속도와 최종 시간, 위치, 속도, 가속도 모두 변경
bool FifthOrderPolynomialTrajectory::ChangeTrajectory(double initial_time, double initial_pos, double initial_vel, double initial_acc,
                                                      double final_time, double final_pos, double final_vel, double final_acc)
{
    if (final_time < initial_time)
    {
        return false;
    }

    initial_time_ = initial_time;
    initial_pos_ = initial_pos;
    initial_vel_ = initial_vel;
    initial_acc_ = initial_acc;
    final_time_ = final_time;
    final_pos_ = final_pos;
    final_vel_ = final_vel;
    final_acc_ = final_acc;

    current_time_ = initial_time_;
    current_pos_ = initial_pos_;
    current_vel_ = initial_vel_;
    current_acc_ = initial_acc_;

    CalculateCoefficients();

    return true;
}

// 5차 다항식 계수를 계산하는 함수
void FifthOrderPolynomialTrajectory::CalculateCoefficients()
{
    double t0 = initial_time_;
    double tf = final_time_;
    double p0 = initial_pos_;
    double pf = final_pos_;
    double v0 = initial_vel_;
    double vf = final_vel_;
    double a0 = initial_acc_;
    double af = final_acc_;

    // 5차 다항식: p(t) = a5*t^5 + a4*t^4 + a3*t^3 + a2*t^2 + a1*t + a0
    // 경계 조건:
    // p(t0) = p0
    // p'(t0) = v0
    // p''(t0) = a0
    // p(tf) = pf
    // p'(tf) = vf
    // p''(tf) = af

    Eigen::MatrixXd time_mat(6, 6);
    time_mat << pow(t0, 5), pow(t0, 4), pow(t0, 3), pow(t0, 2), t0, 1.0,
        5 * pow(t0, 4), 4 * pow(t0, 3), 3 * pow(t0, 2), 2 * t0, 1.0, 0.0,
        20 * pow(t0, 3), 12 * pow(t0, 2), 6 * t0, 2.0, 0.0, 0.0,
        pow(tf, 5), pow(tf, 4), pow(tf, 3), pow(tf, 2), tf, 1.0,
        5 * pow(tf, 4), 4 * pow(tf, 3), 3 * pow(tf, 2), 2 * tf, 1.0, 0.0,
        20 * pow(tf, 3), 12 * pow(tf, 2), 6 * tf, 2.0, 0.0, 0.0;

    Eigen::VectorXd conditions(6);
    conditions << p0, v0, a0, pf, vf, af;

    // Solve for coefficients: a = time_mat.inverse() * conditions
    // Alternatively, use a linear solver for better numerical stability
    Eigen::VectorXd coeff = time_mat.colPivHouseholderQr().solve(conditions);

    position_coeff_ = coeff;

    // Compute velocity coefficients: derivative of position_coeff_
    // p'(t) = 5*a5*t^4 + 4*a4*t^3 + 3*a3*t^2 + 2*a2*t + a1
    // Thus, velocity_coeff_ = [5*a5, 4*a4, 3*a3, 2*a2, a1]^T
    velocity_coeff_.resize(5);
    velocity_coeff_ << 5 * position_coeff_(0), 4 * position_coeff_(1), 3 * position_coeff_(2), 2 * position_coeff_(3), position_coeff_(4);

    // Compute acceleration coefficients: derivative of velocity_coeff_
    // p''(t) = 20*a5*t^3 + 12*a4*t^2 + 6*a3*t + 2*a2
    // Thus, acceleration_coeff_ = [20*a5, 12*a4, 6*a3, 2*a2]^T
    acceleration_coeff_.resize(4);
    acceleration_coeff_ << 20 * position_coeff_(0), 12 * position_coeff_(1), 6 * position_coeff_(2), 2 * position_coeff_(3);

    // Reset current state to initial
    current_time_ = t0;
    current_pos_ = p0;
    current_vel_ = v0;
    current_acc_ = a0;
}

// 주어진 시간에서의 위치를 반환
double FifthOrderPolynomialTrajectory::GetPosition(double time)
{
    if (time >= final_time_)
    {
        current_time_ = final_time_;
        current_pos_ = final_pos_;
        current_vel_ = final_vel_;
        current_acc_ = final_acc_;
        return final_pos_;
    }
    else if (time <= initial_time_)
    {
        current_time_ = initial_time_;
        current_pos_ = initial_pos_;
        current_vel_ = initial_vel_;
        current_acc_ = initial_acc_;
        return initial_pos_;
    }
    else
    {
        double t = time;
        current_time_ = t;
        current_pos_ = position_coeff_(0) * pow(t, 5) + position_coeff_(1) * pow(t, 4) +
                       position_coeff_(2) * pow(t, 3) + position_coeff_(3) * pow(t, 2) +
                       position_coeff_(4) * t + position_coeff_(5);
        current_vel_ = velocity_coeff_(0) * pow(t, 4) + velocity_coeff_(1) * pow(t, 3) +
                       velocity_coeff_(2) * pow(t, 2) + velocity_coeff_(3) * t +
                       velocity_coeff_(4);
        current_acc_ = acceleration_coeff_(0) * pow(t, 3) + acceleration_coeff_(1) * pow(t, 2) +
                       acceleration_coeff_(2) * t + acceleration_coeff_(3);
        return current_pos_;
    }
}

// 주어진 시간에서의 속도를 반환
double FifthOrderPolynomialTrajectory::GetVelocity(double time)
{
    if (time >= final_time_)
    {
        current_time_ = final_time_;
        current_pos_ = final_pos_;
        current_vel_ = final_vel_;
        current_acc_ = final_acc_;
        return final_vel_;
    }
    else if (time <= initial_time_)
    {
        current_time_ = initial_time_;
        current_pos_ = initial_pos_;
        current_vel_ = initial_vel_;
        current_acc_ = initial_acc_;
        return initial_vel_;
    }
    else
    {
        double t = time;
        current_time_ = t;
        current_pos_ = position_coeff_(0) * pow(t, 5) + position_coeff_(1) * pow(t, 4) +
                       position_coeff_(2) * pow(t, 3) + position_coeff_(3) * pow(t, 2) +
                       position_coeff_(4) * t + position_coeff_(5);
        current_vel_ = velocity_coeff_(0) * pow(t, 4) + velocity_coeff_(1) * pow(t, 3) +
                       velocity_coeff_(2) * pow(t, 2) + velocity_coeff_(3) * t +
                       velocity_coeff_(4);
        current_acc_ = acceleration_coeff_(0) * pow(t, 3) + acceleration_coeff_(1) * pow(t, 2) +
                       acceleration_coeff_(2) * t + acceleration_coeff_(3);
        return current_vel_;
    }
}

// 주어진 시간에서의 가속도를 반환
double FifthOrderPolynomialTrajectory::GetAcceleration(double time)
{
    if (time >= final_time_)
    {
        current_time_ = final_time_;
        current_pos_ = final_pos_;
        current_vel_ = final_vel_;
        current_acc_ = final_acc_;
        return final_acc_;
    }
    else if (time <= initial_time_)
    {
        current_time_ = initial_time_;
        current_pos_ = initial_pos_;
        current_vel_ = initial_vel_;
        current_acc_ = initial_acc_;
        return initial_acc_;
    }
    else
    {
        double t = time;
        current_time_ = t;
        current_pos_ = position_coeff_(0) * pow(t, 5) + position_coeff_(1) * pow(t, 4) +
                       position_coeff_(2) * pow(t, 3) + position_coeff_(3) * pow(t, 2) +
                       position_coeff_(4) * t + position_coeff_(5);
        current_vel_ = velocity_coeff_(0) * pow(t, 4) + velocity_coeff_(1) * pow(t, 3) +
                       velocity_coeff_(2) * pow(t, 2) + velocity_coeff_(3) * t +
                       velocity_coeff_(4);
        current_acc_ = acceleration_coeff_(0) * pow(t, 3) + acceleration_coeff_(1) * pow(t, 2) +
                       acceleration_coeff_(2) * t + acceleration_coeff_(3);
        return current_acc_;
    }
}

// 현재 시간을 설정하고 위치, 속도, 가속도를 업데이트
void FifthOrderPolynomialTrajectory::SetTime(double time)
{
    GetPosition(time);
}

// 현재 위치를 반환
double FifthOrderPolynomialTrajectory::GetPosition() const
{
    return current_pos_;
}

// 현재 속도를 반환
double FifthOrderPolynomialTrajectory::GetVelocity() const
{
    return current_vel_;
}

// 현재 가속도를 반환
double FifthOrderPolynomialTrajectory::GetAcceleration() const
{
    return current_acc_;
}

// 기본 생성자 구현
SeventhOrderPolynomialTrajectory::SeventhOrderPolynomialTrajectory()
    : initial_time_(0.0),
      initial_pos_(0.0),
      initial_vel_(0.0),
      initial_acc_(0.0),
      initial_jerk_(0.0),
      final_time_(0.0),
      final_pos_(0.0),
      final_vel_(0.0),
      final_acc_(0.0),
      final_jerk_(0.0),
      current_time_(0.0),
      current_pos_(0.0),
      current_vel_(0.0),
      current_acc_(0.0),
      current_jerk_(0.0),
      position_coeff_(Eigen::VectorXd::Zero(8)),
      velocity_coeff_(Eigen::VectorXd::Zero(7)),
      acceleration_coeff_(Eigen::VectorXd::Zero(6)),
      jerk_coeff_(Eigen::VectorXd::Zero(5)),
      time_variables_(Eigen::VectorXd::Zero(7)) // t^7부터 t^1까지
{
    // 초기화 완료
}

// 매개변수가 있는 생성자 구현
SeventhOrderPolynomialTrajectory::SeventhOrderPolynomialTrajectory(double initial_time, double initial_pos, double initial_vel, double initial_acc, double initial_jerk,
                                                                   double final_time, double final_pos, double final_vel, double final_acc, double final_jerk)
    : initial_time_(initial_time),
      initial_pos_(initial_pos),
      initial_vel_(initial_vel),
      initial_acc_(initial_acc),
      initial_jerk_(initial_jerk),
      final_time_(final_time),
      final_pos_(final_pos),
      final_vel_(final_vel),
      final_acc_(final_acc),
      final_jerk_(final_jerk),
      current_time_(initial_time),
      current_pos_(initial_pos),
      current_vel_(initial_vel),
      current_acc_(initial_acc),
      current_jerk_(initial_jerk),
      position_coeff_(Eigen::VectorXd::Zero(8)),
      velocity_coeff_(Eigen::VectorXd::Zero(7)),
      acceleration_coeff_(Eigen::VectorXd::Zero(6)),
      jerk_coeff_(Eigen::VectorXd::Zero(5)),
      time_variables_(Eigen::VectorXd::Zero(7))
{
    if (final_time_ < initial_time_)
    {
        throw std::invalid_argument("final_time은 initial_time보다 작을 수 없습니다.");
    }
    CalculateCoefficients();
}

// 소멸자 구현
SeventhOrderPolynomialTrajectory::~SeventhOrderPolynomialTrajectory() {}

// 궤적 변경: 최종 상태만 변경
bool SeventhOrderPolynomialTrajectory::ChangeTrajectory(double final_pos, double final_vel, double final_acc, double final_jerk)
{
    final_pos_ = final_pos;
    final_vel_ = final_vel;
    final_acc_ = final_acc;
    final_jerk_ = final_jerk;

    CalculateCoefficients();

    return true;
}

// 궤적 변경: 최종 시간과 최종 상태 변경
bool SeventhOrderPolynomialTrajectory::ChangeTrajectory(double final_time, double final_pos, double final_vel, double final_acc, double final_jerk)
{
    if (final_time < initial_time_)
    {
        return false;
    }

    final_time_ = final_time;
    final_pos_ = final_pos;
    final_vel_ = final_vel;
    final_acc_ = final_acc;
    final_jerk_ = final_jerk;

    CalculateCoefficients();

    return true;
}

// 궤적 변경: 초기 및 최종 시간과 상태를 모두 변경
bool SeventhOrderPolynomialTrajectory::ChangeTrajectory(double initial_time, double initial_pos, double initial_vel, double initial_acc, double initial_jerk,
                                                        double final_time, double final_pos, double final_vel, double final_acc, double final_jerk)
{
    if (final_time < initial_time)
    {
        return false;
    }

    initial_time_ = initial_time;
    initial_pos_ = initial_pos;
    initial_vel_ = initial_vel;
    initial_acc_ = initial_acc;
    initial_jerk_ = initial_jerk;

    final_time_ = final_time;
    final_pos_ = final_pos;
    final_vel_ = final_vel;
    final_acc_ = final_acc;
    final_jerk_ = final_jerk;

    current_time_ = initial_time_;
    current_pos_ = initial_pos_;
    current_vel_ = initial_vel_;
    current_acc_ = initial_acc_;
    current_jerk_ = initial_jerk_;

    CalculateCoefficients();

    return true;
}

// 7차 다항식 계수를 계산하는 함수
void SeventhOrderPolynomialTrajectory::CalculateCoefficients()
{
    double t0 = initial_time_;
    double tf = final_time_;
    double p0 = initial_pos_;
    double pf = final_pos_;
    double v0 = initial_vel_;
    double vf = final_vel_;
    double a0 = initial_acc_;
    double af = final_acc_;
    double j0 = initial_jerk_;
    double jf = final_jerk_;

    // 7차 다항식: p(t) = a7*t^7 + a6*t^6 + a5*t^5 + a4*t^4 + a3*t^3 + a2*t^2 + a1*t + a0
    // 경계 조건:
    // p(t0) = p0
    // p'(t0) = v0
    // p''(t0) = a0
    // p'''(t0) = j0
    // p(tf) = pf
    // p'(tf) = vf
    // p''(tf) = af
    // p'''(tf) = jf

    Eigen::MatrixXd time_mat(8, 8);
    time_mat << pow(t0, 7), pow(t0, 6), pow(t0, 5), pow(t0, 4), pow(t0, 3), pow(t0, 2), t0, 1.0,
        7 * pow(t0, 6), 6 * pow(t0, 5), 5 * pow(t0, 4), 4 * pow(t0, 3), 3 * pow(t0, 2), 2 * t0, 1.0, 0.0,
        42 * pow(t0, 5), 30 * pow(t0, 4), 20 * pow(t0, 3), 12 * pow(t0, 2), 6 * t0, 2.0, 0.0, 0.0,
        210 * pow(t0, 4), 120 * pow(t0, 3), 60 * pow(t0, 2), 24 * pow(t0, 1), 6.0, 0.0, 0.0, 0.0,
        pow(tf, 7), pow(tf, 6), pow(tf, 5), pow(tf, 4), pow(tf, 3), pow(tf, 2), tf, 1.0,
        7 * pow(tf, 6), 6 * pow(tf, 5), 5 * pow(tf, 4), 4 * pow(tf, 3), 3 * pow(tf, 2), 2 * tf, 1.0, 0.0,
        42 * pow(tf, 5), 30 * pow(tf, 4), 20 * pow(tf, 3), 12 * pow(tf, 2), 6 * tf, 2.0, 0.0, 0.0,
        210 * pow(tf, 4), 120 * pow(tf, 3), 60 * pow(tf, 2), 24 * pow(tf, 1), 6.0, 0.0, 0.0, 0.0;

    Eigen::VectorXd conditions(8);
    conditions << p0, v0, a0, j0, pf, vf, af, jf;

    // Solve for coefficients: a = time_mat.inverse() * conditions
    // Alternatively, use a linear solver for better numerical stability
    Eigen::VectorXd coeff = time_mat.colPivHouseholderQr().solve(conditions);

    position_coeff_ = coeff;

    // Compute velocity coefficients: derivative of position_coeff_
    // p'(t) = 7*a7*t^6 + 6*a6*t^5 + 5*a5*t^4 + 4*a4*t^3 + 3*a3*t^2 + 2*a2*t + a1
    velocity_coeff_.resize(7);
    velocity_coeff_ << 7 * position_coeff_(0), 6 * position_coeff_(1), 5 * position_coeff_(2),
        4 * position_coeff_(3), 3 * position_coeff_(4), 2 * position_coeff_(5),
        position_coeff_(6);

    // Compute acceleration coefficients: derivative of velocity_coeff_
    // p''(t) = 42*a7*t^5 + 30*a6*t^4 + 20*a5*t^3 + 12*a4*t^2 + 6*a3*t + 2*a2
    acceleration_coeff_.resize(6);
    acceleration_coeff_ << 42 * position_coeff_(0), 30 * position_coeff_(1), 20 * position_coeff_(2),
        12 * position_coeff_(3), 6 * position_coeff_(4), 2 * position_coeff_(5);

    // Compute jerk coefficients: derivative of acceleration_coeff_
    // p'''(t) = 210*a7*t^4 + 120*a6*t^3 + 60*a5*t^2 + 24*a4*t + 6*a3
    jerk_coeff_.resize(5);
    jerk_coeff_ << 210 * position_coeff_(0), 120 * position_coeff_(1), 60 * position_coeff_(2),
        24 * position_coeff_(3), 6 * position_coeff_(4);

    // Reset current state to initial
    current_time_ = t0;
    current_pos_ = p0;
    current_vel_ = v0;
    current_acc_ = a0;
    current_jerk_ = j0;
}

// 주어진 시간에서의 위치를 반환
double SeventhOrderPolynomialTrajectory::GetPosition(double time)
{
    if (time >= final_time_)
    {
        current_time_ = final_time_;
        current_pos_ = final_pos_;
        current_vel_ = final_vel_;
        current_acc_ = final_acc_;
        current_jerk_ = final_jerk_;
        return final_pos_;
    }
    else if (time <= initial_time_)
    {
        current_time_ = initial_time_;
        current_pos_ = initial_pos_;
        current_vel_ = initial_vel_;
        current_acc_ = initial_acc_;
        current_jerk_ = initial_jerk_;
        return initial_pos_;
    }
    else
    {
        double t = time;
        current_time_ = t;
        current_pos_ = position_coeff_(0) * pow(t, 7) + position_coeff_(1) * pow(t, 6) +
                       position_coeff_(2) * pow(t, 5) + position_coeff_(3) * pow(t, 4) +
                       position_coeff_(4) * pow(t, 3) + position_coeff_(5) * pow(t, 2) +
                       position_coeff_(6) * t + position_coeff_(7);
        current_vel_ = velocity_coeff_(0) * pow(t, 6) + velocity_coeff_(1) * pow(t, 5) +
                       velocity_coeff_(2) * pow(t, 4) + velocity_coeff_(3) * pow(t, 3) +
                       velocity_coeff_(4) * pow(t, 2) + velocity_coeff_(5) * t +
                       velocity_coeff_(6);
        current_acc_ = acceleration_coeff_(0) * pow(t, 5) + acceleration_coeff_(1) * pow(t, 4) +
                       acceleration_coeff_(2) * pow(t, 3) + acceleration_coeff_(3) * pow(t, 2) +
                       acceleration_coeff_(4) * t + acceleration_coeff_(5);
        current_jerk_ = jerk_coeff_(0) * pow(t, 4) + jerk_coeff_(1) * pow(t, 3) +
                        jerk_coeff_(2) * pow(t, 2) + jerk_coeff_(3) * t +
                        jerk_coeff_(4);
        return current_pos_;
    }
}

// 주어진 시간에서의 속도를 반환
double SeventhOrderPolynomialTrajectory::GetVelocity(double time)
{
    if (time >= final_time_)
    {
        current_time_ = final_time_;
        current_pos_ = final_pos_;
        current_vel_ = final_vel_;
        current_acc_ = final_acc_;
        current_jerk_ = final_jerk_;
        return final_vel_;
    }
    else if (time <= initial_time_)
    {
        current_time_ = initial_time_;
        current_pos_ = initial_pos_;
        current_vel_ = initial_vel_;
        current_acc_ = initial_acc_;
        current_jerk_ = initial_jerk_;
        return initial_vel_;
    }
    else
    {
        double t = time;
        current_time_ = t;
        current_pos_ = position_coeff_(0) * pow(t, 7) + position_coeff_(1) * pow(t, 6) +
                       position_coeff_(2) * pow(t, 5) + position_coeff_(3) * pow(t, 4) +
                       position_coeff_(4) * pow(t, 3) + position_coeff_(5) * pow(t, 2) +
                       position_coeff_(6) * t + position_coeff_(7);
        current_vel_ = velocity_coeff_(0) * pow(t, 6) + velocity_coeff_(1) * pow(t, 5) +
                       velocity_coeff_(2) * pow(t, 4) + velocity_coeff_(3) * pow(t, 3) +
                       velocity_coeff_(4) * pow(t, 2) + velocity_coeff_(5) * t +
                       velocity_coeff_(6);
        current_acc_ = acceleration_coeff_(0) * pow(t, 5) + acceleration_coeff_(1) * pow(t, 4) +
                       acceleration_coeff_(2) * pow(t, 3) + acceleration_coeff_(3) * pow(t, 2) +
                       acceleration_coeff_(4) * t + acceleration_coeff_(5);
        current_jerk_ = jerk_coeff_(0) * pow(t, 4) + jerk_coeff_(1) * pow(t, 3) +
                        jerk_coeff_(2) * pow(t, 2) + jerk_coeff_(3) * t +
                        jerk_coeff_(4);
        return current_vel_;
    }
}

// 주어진 시간에서의 가속도를 반환
double SeventhOrderPolynomialTrajectory::GetAcceleration(double time)
{
    if (time >= final_time_)
    {
        current_time_ = final_time_;
        current_pos_ = final_pos_;
        current_vel_ = final_vel_;
        current_acc_ = final_acc_;
        current_jerk_ = final_jerk_;
        return final_acc_;
    }
    else if (time <= initial_time_)
    {
        current_time_ = initial_time_;
        current_pos_ = initial_pos_;
        current_vel_ = initial_vel_;
        current_acc_ = initial_acc_;
        current_jerk_ = initial_jerk_;
        return initial_acc_;
    }
    else
    {
        double t = time;
        current_time_ = t;
        current_pos_ = position_coeff_(0) * pow(t, 7) + position_coeff_(1) * pow(t, 6) +
                       position_coeff_(2) * pow(t, 5) + position_coeff_(3) * pow(t, 4) +
                       position_coeff_(4) * pow(t, 3) + position_coeff_(5) * pow(t, 2) +
                       position_coeff_(6) * t + position_coeff_(7);
        current_vel_ = velocity_coeff_(0) * pow(t, 6) + velocity_coeff_(1) * pow(t, 5) +
                       velocity_coeff_(2) * pow(t, 4) + velocity_coeff_(3) * pow(t, 3) +
                       velocity_coeff_(4) * pow(t, 2) + velocity_coeff_(5) * t +
                       velocity_coeff_(6);
        current_acc_ = acceleration_coeff_(0) * pow(t, 5) + acceleration_coeff_(1) * pow(t, 4) +
                       acceleration_coeff_(2) * pow(t, 3) + acceleration_coeff_(3) * pow(t, 2) +
                       acceleration_coeff_(4) * t + acceleration_coeff_(5);
        current_jerk_ = jerk_coeff_(0) * pow(t, 4) + jerk_coeff_(1) * pow(t, 3) +
                        jerk_coeff_(2) * pow(t, 2) + jerk_coeff_(3) * t +
                        jerk_coeff_(4);
        return current_acc_;
    }
}

// 주어진 시간에서의 젭을 반환
double SeventhOrderPolynomialTrajectory::GetJerk(double time)
{
    if (time >= final_time_)
    {
        current_time_ = final_time_;
        current_pos_ = final_pos_;
        current_vel_ = final_vel_;
        current_acc_ = final_acc_;
        current_jerk_ = final_jerk_;
        return final_jerk_;
    }
    else if (time <= initial_time_)
    {
        current_time_ = initial_time_;
        current_pos_ = initial_pos_;
        current_vel_ = initial_vel_;
        current_acc_ = initial_acc_;
        current_jerk_ = initial_jerk_;
        return initial_jerk_;
    }
    else
    {
        double t = time;
        current_time_ = t;
        current_pos_ = position_coeff_(0) * pow(t, 7) + position_coeff_(1) * pow(t, 6) +
                       position_coeff_(2) * pow(t, 5) + position_coeff_(3) * pow(t, 4) +
                       position_coeff_(4) * pow(t, 3) + position_coeff_(5) * pow(t, 2) +
                       position_coeff_(6) * t + position_coeff_(7);
        current_vel_ = velocity_coeff_(0) * pow(t, 6) + velocity_coeff_(1) * pow(t, 5) +
                       velocity_coeff_(2) * pow(t, 4) + velocity_coeff_(3) * pow(t, 3) +
                       velocity_coeff_(4) * pow(t, 2) + velocity_coeff_(5) * t +
                       velocity_coeff_(6);
        current_acc_ = acceleration_coeff_(0) * pow(t, 5) + acceleration_coeff_(1) * pow(t, 4) +
                       acceleration_coeff_(2) * pow(t, 3) + acceleration_coeff_(3) * pow(t, 2) +
                       acceleration_coeff_(4) * t + acceleration_coeff_(5);
        current_jerk_ = jerk_coeff_(0) * pow(t, 4) + jerk_coeff_(1) * pow(t, 3) +
                        jerk_coeff_(2) * pow(t, 2) + jerk_coeff_(3) * t +
                        jerk_coeff_(4);
        return current_jerk_;
    }
}

// 현재 시간을 설정하고 위치, 속도, 가속도, 젭을 업데이트
void SeventhOrderPolynomialTrajectory::SetTime(double time)
{
    GetPosition(time); // GetPosition을 호출하여 모든 상태를 업데이트
}

// 현재 위치를 반환
double SeventhOrderPolynomialTrajectory::GetPosition() const
{
    return current_pos_;
}

// 현재 속도를 반환
double SeventhOrderPolynomialTrajectory::GetVelocity() const
{
    return current_vel_;
}

// 현재 가속도를 반환
double SeventhOrderPolynomialTrajectory::GetAcceleration() const
{
    return current_acc_;
}

// 현재 젭을 반환
double SeventhOrderPolynomialTrajectory::GetJerk() const
{
    return current_jerk_;
}

NinthOrderPolynomialTrajectory::NinthOrderPolynomialTrajectory()
    : initial_time_(0.0),
      initial_pos_(0.0),
      initial_vel_(0.0),
      initial_acc_(0.0),
      initial_jerk_(0.0),
      initial_snap_(0.0),
      final_time_(0.0),
      final_pos_(0.0),
      final_vel_(0.0),
      final_acc_(0.0),
      final_jerk_(0.0),
      final_snap_(0.0),
      current_time_(0.0),
      current_pos_(0.0),
      current_vel_(0.0),
      current_acc_(0.0),
      current_jerk_(0.0),
      current_snap_(0.0),
      position_coeff_(Eigen::VectorXd::Zero(10)),
      velocity_coeff_(Eigen::VectorXd::Zero(10)),
      acceleration_coeff_(Eigen::VectorXd::Zero(10)),
      jerk_coeff_(Eigen::VectorXd::Zero(10)),
      snap_coeff_(Eigen::VectorXd::Zero(10)),
      time_variables_(Eigen::VectorXd::Zero(10))
{
    // 초기화 완료
}

// 매개변수가 있는 생성자 구현
NinthOrderPolynomialTrajectory::NinthOrderPolynomialTrajectory(
    double initial_time, double initial_pos, double initial_vel, double initial_acc, double initial_jerk, double initial_snap,
    double final_time, double final_pos, double final_vel, double final_acc, double final_jerk, double final_snap)
    : initial_time_(initial_time),
      initial_pos_(initial_pos),
      initial_vel_(initial_vel),
      initial_acc_(initial_acc),
      initial_jerk_(initial_jerk),   
      initial_snap_(initial_snap),  
      final_time_(final_time),
      final_pos_(final_pos),
      final_vel_(final_vel),
      final_acc_(final_acc),
      final_jerk_(final_jerk),      
      final_snap_(final_snap),      
      current_time_(initial_time),
      current_pos_(initial_pos),
      current_vel_(initial_vel),
      current_acc_(initial_acc),
      current_jerk_(initial_jerk),   
      current_snap_(initial_snap),  
      position_coeff_(Eigen::VectorXd::Zero(10)),
      velocity_coeff_(Eigen::VectorXd::Zero(10)),
      acceleration_coeff_(Eigen::VectorXd::Zero(10)),
      jerk_coeff_(Eigen::VectorXd::Zero(10)),
      snap_coeff_(Eigen::VectorXd::Zero(10)),
      time_variables_(Eigen::VectorXd::Zero(10))
{
    if (final_time_ <= initial_time_) {
        throw std::invalid_argument("final_time은 initial_time보다 커야 합니다.");
    }
    CalculateCoefficients();
}

// 소멸자 구현
NinthOrderPolynomialTrajectory::~NinthOrderPolynomialTrajectory()
{
    // 특별한 작업이 필요하지 않음
}

// 궤적을 변경하는 메서드들

// 최종 상태만 변경
bool NinthOrderPolynomialTrajectory::ChangeTrajectory(double final_pos, double final_vel, double final_acc, double final_jerk, double final_snap)
{
    final_pos_ = final_pos;
    final_vel_ = final_vel;
    final_acc_ = final_acc;
    final_jerk_ = final_jerk;
    final_snap_ = final_snap;

    CalculateCoefficients();

    return true;
}

// 최종 시간과 최종 상태 변경
bool NinthOrderPolynomialTrajectory::ChangeTrajectory(double final_time, double final_pos, double final_vel, double final_acc, double final_jerk, double final_snap)
{
    if (final_time <= initial_time_)
    {
        return false;
    }

    final_time_ = final_time;
    final_pos_ = final_pos;
    final_vel_ = final_vel;
    final_acc_ = final_acc;
    final_jerk_ = final_jerk;
    final_snap_ = final_snap;

    CalculateCoefficients();

    return true;
}

// 초기 및 최종 상태를 모두 변경
bool NinthOrderPolynomialTrajectory::ChangeTrajectory(double initial_time, double initial_pos, double initial_vel, double initial_acc, double initial_jerk, double initial_snap,
                                                      double final_time, double final_pos, double final_vel, double final_acc, double final_jerk, double final_snap)
{
    if (final_time <= initial_time)
    {
        return false;
    }

    initial_time_ = initial_time;
    initial_pos_ = initial_pos;
    initial_vel_ = initial_vel;
    initial_acc_ = initial_acc;
    initial_jerk_ = initial_jerk;
    initial_snap_ = initial_snap;

    final_time_ = final_time;
    final_pos_ = final_pos;
    final_vel_ = final_vel;
    final_acc_ = final_acc;
    final_jerk_ = final_jerk;
    final_snap_ = final_snap;

    current_time_ = initial_time_;
    current_pos_ = initial_pos_;
    current_vel_ = initial_vel_;
    current_acc_ = initial_acc_;
    current_jerk_ = initial_jerk_;
    current_snap_ = initial_snap_;

    CalculateCoefficients();

    return true;
}

// 계수를 계산하는 메서드
void NinthOrderPolynomialTrajectory::CalculateCoefficients()
{
    double t0 = initial_time_;
    double tf = final_time_;
    double p0 = initial_pos_;
    double pf = final_pos_;
    double v0 = initial_vel_;
    double vf = final_vel_;
    double a0 = initial_acc_;
    double af = final_acc_;
    double j0 = initial_jerk_;
    double jf = final_jerk_;
    double s0 = initial_snap_;
    double sf = final_snap_;

    // 9차 다항식: p(t) = a9*t^9 + a8*t^8 + ... + a1*t + a0
    // 경계 조건:
    // p(t0) = p0
    // p'(t0) = v0
    // p''(t0) = a0
    // p'''(t0) = j0
    // p''''(t0) = s0
    // p(tf) = pf
    // p'(tf) = vf
    // p''(tf) = af
    // p'''(tf) = jf
    // p''''(tf) = sf

    Eigen::MatrixXd time_mat(10, 10);
    Eigen::VectorXd conditions_mat(10);

    // 초기 시간 조건
    time_mat(0, 0) = std::pow(t0, 9);
    time_mat(0, 1) = std::pow(t0, 8);
    time_mat(0, 2) = std::pow(t0, 7);
    time_mat(0, 3) = std::pow(t0, 6);
    time_mat(0, 4) = std::pow(t0, 5);
    time_mat(0, 5) = std::pow(t0, 4);
    time_mat(0, 6) = std::pow(t0, 3);
    time_mat(0, 7) = std::pow(t0, 2);
    time_mat(0, 8) = t0;
    time_mat(0, 9) = 1.0;

    // 초기 속도 조건 (p'(t0) = v0)
    time_mat(1, 0) = 9.0 * std::pow(t0, 8);
    time_mat(1, 1) = 8.0 * std::pow(t0, 7);
    time_mat(1, 2) = 7.0 * std::pow(t0, 6);
    time_mat(1, 3) = 6.0 * std::pow(t0, 5);
    time_mat(1, 4) = 5.0 * std::pow(t0, 4);
    time_mat(1, 5) = 4.0 * std::pow(t0, 3);
    time_mat(1, 6) = 3.0 * std::pow(t0, 2);
    time_mat(1, 7) = 2.0 * t0;
    time_mat(1, 8) = 1.0;
    time_mat(1, 9) = 0.0;

    // 초기 가속도 조건 (p''(t0) = a0)
    time_mat(2, 0) = 72.0 * std::pow(t0, 7);
    time_mat(2, 1) = 56.0 * std::pow(t0, 6);
    time_mat(2, 2) = 42.0 * std::pow(t0, 5);
    time_mat(2, 3) = 24.0 * std::pow(t0, 4);
    time_mat(2, 4) = 20.0 * std::pow(t0, 3);
    time_mat(2, 5) = 12.0 * std::pow(t0, 2);
    time_mat(2, 6) = 6.0 * t0;
    time_mat(2, 7) = 2.0;
    time_mat(2, 8) = 0.0;
    time_mat(2, 9) = 0.0;

    // 초기 젭 조건 (p'''(t0) = j0)
    time_mat(3, 0) = 504.0 * std::pow(t0, 6);
    time_mat(3, 1) = 336.0 * std::pow(t0, 5);
    time_mat(3, 2) = 210.0 * std::pow(t0, 4);
    time_mat(3, 3) = 120.0 * std::pow(t0, 3);
    time_mat(3, 4) = 60.0 * std::pow(t0, 2);
    time_mat(3, 5) = 24.0 * t0;
    time_mat(3, 6) = 6.0;
    time_mat(3, 7) = 0.0;
    time_mat(3, 8) = 0.0;
    time_mat(3, 9) = 0.0;

    // 초기 스냅 조건 (p''''(t0) = s0)
    time_mat(4, 0) = 3024.0 * std::pow(t0, 5);
    time_mat(4, 1) = 1680.0 * std::pow(t0, 4);
    time_mat(4, 2) = 840.0 * std::pow(t0, 3);
    time_mat(4, 3) = 360.0 * std::pow(t0, 2);
    time_mat(4, 4) = 120.0 * t0;
    time_mat(4, 5) = 24.0;
    time_mat(4, 6) = 0.0;
    time_mat(4, 7) = 0.0;
    time_mat(4, 8) = 0.0;
    time_mat(4, 9) = 0.0;

    // 최종 시간 조건
    time_mat(5, 0) = std::pow(tf, 9);
    time_mat(5, 1) = std::pow(tf, 8);
    time_mat(5, 2) = std::pow(tf, 7);
    time_mat(5, 3) = std::pow(tf, 6);
    time_mat(5, 4) = std::pow(tf, 5);
    time_mat(5, 5) = std::pow(tf, 4);
    time_mat(5, 6) = std::pow(tf, 3);
    time_mat(5, 7) = std::pow(tf, 2);
    time_mat(5, 8) = tf;
    time_mat(5, 9) = 1.0;

    // 최종 속도 조건 (p'(tf) = vf)
    time_mat(6, 0) = 9.0 * std::pow(tf, 8);
    time_mat(6, 1) = 8.0 * std::pow(tf, 7);
    time_mat(6, 2) = 7.0 * std::pow(tf, 6);
    time_mat(6, 3) = 6.0 * std::pow(tf, 5);
    time_mat(6, 4) = 5.0 * std::pow(tf, 4);
    time_mat(6, 5) = 4.0 * std::pow(tf, 3);
    time_mat(6, 6) = 3.0 * std::pow(tf, 2);
    time_mat(6, 7) = 2.0 * tf;
    time_mat(6, 8) = 1.0;
    time_mat(6, 9) = 0.0;

    // 최종 가속도 조건 (p''(tf) = af)
    time_mat(7, 0) = 72.0 * std::pow(tf, 7);
    time_mat(7, 1) = 56.0 * std::pow(tf, 6);
    time_mat(7, 2) = 42.0 * std::pow(tf, 5);
    time_mat(7, 3) = 24.0 * std::pow(tf, 4);
    time_mat(7, 4) = 20.0 * std::pow(tf, 3);
    time_mat(7, 5) = 12.0 * std::pow(tf, 2);
    time_mat(7, 6) = 6.0 * tf;
    time_mat(7, 7) = 2.0;
    time_mat(7, 8) = 0.0;
    time_mat(7, 9) = 0.0;

    // 최종 젭 조건 (p'''(tf) = jf)
    time_mat(8, 0) = 504.0 * std::pow(tf, 6);
    time_mat(8, 1) = 336.0 * std::pow(tf, 5);
    time_mat(8, 2) = 210.0 * std::pow(tf, 4);
    time_mat(8, 3) = 120.0 * std::pow(tf, 3);
    time_mat(8, 4) = 60.0 * std::pow(tf, 2);
    time_mat(8, 5) = 24.0 * tf;
    time_mat(8, 6) = 6.0;
    time_mat(8, 7) = 0.0;
    time_mat(8, 8) = 0.0;
    time_mat(8, 9) = 0.0;

    // 최종 스냅 조건 (p''''(tf) = sf)
    time_mat(9, 0) = 3024.0 * std::pow(tf, 5);
    time_mat(9, 1) = 1680.0 * std::pow(tf, 4);
    time_mat(9, 2) = 840.0 * std::pow(tf, 3);
    time_mat(9, 3) = 360.0 * std::pow(tf, 2);
    time_mat(9, 4) = 120.0 * tf;
    time_mat(9, 5) = 24.0;
    time_mat(9, 6) = 0.0;
    time_mat(9, 7) = 0.0;
    time_mat(9, 8) = 0.0;
    time_mat(9, 9) = 0.0;

    // 조건 벡터 설정
    conditions_mat << p0, v0, a0, j0, s0, pf, vf, af, jf, sf;

    // 선형 방정식 풀기
    Eigen::VectorXd coeff = time_mat.colPivHouseholderQr().solve(conditions_mat);

    position_coeff_ = coeff;

    // 속도 계수 계산 (p'(t) = 9*a9*t^8 + 8*a8*t^7 + ... + a1)
    velocity_coeff_.resize(10);
    velocity_coeff_(0) = 0.0;
    for (int i = 0; i < 9; ++i)
    {
        velocity_coeff_(i + 1) = (i + 1) * position_coeff_(i);
    }

    // 가속도 계수 계산 (p''(t) = 72*a9*t^7 + 56*a8*t^6 + ... + 2*a2)
    acceleration_coeff_.resize(10);
    acceleration_coeff_.fill(0.0);
    for (int i = 0; i < 8; ++i)
    {
        acceleration_coeff_(i + 2) = (i + 2) * (i + 1) * position_coeff_(i);
    }

    // 젭 계수 계산 (p'''(t) = 504*a9*t^6 + 336*a8*t^5 + ... + 6*a3)
    jerk_coeff_.resize(10);
    jerk_coeff_.fill(0.0);
    for (int i = 0; i < 7; ++i)
    {
        jerk_coeff_(i + 3) = (i + 3) * (i + 2) * (i + 1) * position_coeff_(i);
    }

    // 스냅 계수 계산 (p''''(t) = 3024*a9*t^5 + 1680*a8*t^4 + ... + 24*a4)
    snap_coeff_.resize(10);
    snap_coeff_.fill(0.0);
    for (int i = 0; i < 6; ++i)
    {
        snap_coeff_(i + 4) = (i + 4) * (i + 3) * (i + 2) * (i + 1) * position_coeff_(i);
    }

    // 현재 상태를 초기 상태로 리셋
    current_time_ = t0;
    current_pos_ = p0;
    current_vel_ = v0;
    current_acc_ = a0;
    current_jerk_ = j0;
    current_snap_ = s0;
}

// 위치를 얻는 메서드
double NinthOrderPolynomialTrajectory::GetPosition(double time)
{
    if (time >= final_time_)
    {
        current_time_ = final_time_;
        current_pos_ = final_pos_;
        current_vel_ = final_vel_;
        current_acc_ = final_acc_;
        current_jerk_ = final_jerk_;
        current_snap_ = final_snap_;
        return final_pos_;
    }
    else if (time <= initial_time_)
    {
        current_time_ = initial_time_;
        current_pos_ = initial_pos_;
        current_vel_ = initial_vel_;
        current_acc_ = initial_acc_;
        current_jerk_ = initial_jerk_;
        current_snap_ = initial_snap_;
        return initial_pos_;
    }
    else
    {
        current_time_ = time;
        // 시간 변수 벡터 설정
        time_variables_ << std::pow(time, 9), std::pow(time, 8), std::pow(time, 7), std::pow(time, 6),
            std::pow(time, 5), std::pow(time, 4), std::pow(time, 3), std::pow(time, 2),
            time, 1.0;
        current_pos_ = (time_variables_.transpose() * position_coeff_).coeff(0, 0);
        current_vel_ = (time_variables_.transpose() * velocity_coeff_).coeff(0, 0);
        current_acc_ = (time_variables_.transpose() * acceleration_coeff_).coeff(0, 0);
        current_jerk_ = (time_variables_.transpose() * jerk_coeff_).coeff(0, 0);
        current_snap_ = (time_variables_.transpose() * snap_coeff_).coeff(0, 0);
        return current_pos_;
    }
}

// 속도를 얻는 메서드
double NinthOrderPolynomialTrajectory::GetVelocity(double time)
{
    if (time >= final_time_)
    {
        current_time_ = final_time_;
        current_vel_ = final_vel_;
        return final_vel_;
    }
    else if (time <= initial_time_)
    {
        current_time_ = initial_time_;
        current_vel_ = initial_vel_;
        return initial_vel_;
    }
    else
    {
        current_time_ = time;
        // 시간 변수 벡터 설정
        time_variables_ << std::pow(time, 9), std::pow(time, 8), std::pow(time, 7), std::pow(time, 6),
            std::pow(time, 5), std::pow(time, 4), std::pow(time, 3), std::pow(time, 2),
            time, 1.0;
        current_vel_ = (time_variables_.transpose() * velocity_coeff_).coeff(0, 0);
        current_acc_ = (time_variables_.transpose() * acceleration_coeff_).coeff(0, 0);
        current_jerk_ = (time_variables_.transpose() * jerk_coeff_).coeff(0, 0);
        current_snap_ = (time_variables_.transpose() * snap_coeff_).coeff(0, 0);
        return current_vel_;
    }
}

// 가속도를 얻는 메서드
double NinthOrderPolynomialTrajectory::GetAcceleration(double time)
{
    if (time >= final_time_)
    {
        current_time_ = final_time_;
        current_acc_ = final_acc_;
        return final_acc_;
    }
    else if (time <= initial_time_)
    {
        current_time_ = initial_time_;
        current_acc_ = initial_acc_;
        return initial_acc_;
    }
    else
    {
        current_time_ = time;
        // 시간 변수 벡터 설정
        time_variables_ << std::pow(time, 9), std::pow(time, 8), std::pow(time, 7), std::pow(time, 6),
            std::pow(time, 5), std::pow(time, 4), std::pow(time, 3), std::pow(time, 2),
            time, 1.0;
        current_acc_ = (time_variables_.transpose() * acceleration_coeff_).coeff(0, 0);
        current_jerk_ = (time_variables_.transpose() * jerk_coeff_).coeff(0, 0);
        current_snap_ = (time_variables_.transpose() * snap_coeff_).coeff(0, 0);
        return current_acc_;
    }
}

// 젭을 얻는 메서드
double NinthOrderPolynomialTrajectory::GetJerk(double time)
{
    if (time >= final_time_)
    {
        current_time_ = final_time_;
        current_jerk_ = final_jerk_;
        return final_jerk_;
    }
    else if (time <= initial_time_)
    {
        current_time_ = initial_time_;
        current_jerk_ = initial_jerk_;
        return initial_jerk_;
    }
    else
    {
        current_time_ = time;
        // 시간 변수 벡터 설정
        time_variables_ << std::pow(time, 9), std::pow(time, 8), std::pow(time, 7), std::pow(time, 6),
            std::pow(time, 5), std::pow(time, 4), std::pow(time, 3), std::pow(time, 2),
            time, 1.0;
        current_jerk_ = (time_variables_.transpose() * jerk_coeff_).coeff(0, 0);
        current_snap_ = (time_variables_.transpose() * snap_coeff_).coeff(0, 0);
        return current_jerk_;
    }
}

// 스냅을 얻는 메서드
double NinthOrderPolynomialTrajectory::GetSnap(double time)
{
    if (time >= final_time_)
    {
        current_time_ = final_time_;
        current_snap_ = final_snap_;
        return final_snap_;
    }
    else if (time <= initial_time_)
    {
        current_time_ = initial_time_;
        current_snap_ = initial_snap_;
        return initial_snap_;
    }
    else
    {
        current_time_ = time;
        // 시간 변수 벡터 설정
        time_variables_ << std::pow(time, 9), std::pow(time, 8), std::pow(time, 7), std::pow(time, 6),
            std::pow(time, 5), std::pow(time, 4), std::pow(time, 3), std::pow(time, 2),
            time, 1.0;
        current_snap_ = (time_variables_.transpose() * snap_coeff_).coeff(0, 0);
        return current_snap_;
    }
}

// 시간을 설정하고 모든 상태를 업데이트하는 메서드
void NinthOrderPolynomialTrajectory::SetTime(double time)
{
    if (time >= final_time_)
    {
        current_time_ = final_time_;
        current_pos_ = final_pos_;
        current_vel_ = final_vel_;
        current_acc_ = final_acc_;
        current_jerk_ = final_jerk_;
        current_snap_ = final_snap_;
    }
    else if (time <= initial_time_)
    {
        current_time_ = initial_time_;
        current_pos_ = initial_pos_;
        current_vel_ = initial_vel_;
        current_acc_ = initial_acc_;
        current_jerk_ = initial_jerk_;
        current_snap_ = initial_snap_;
    }
    else
    {
        current_time_ = time;
        // 시간 변수 벡터 설정
        time_variables_ << std::pow(time, 9), std::pow(time, 8), std::pow(time, 7), std::pow(time, 6),
            std::pow(time, 5), std::pow(time, 4), std::pow(time, 3), std::pow(time, 2),
            time, 1.0;
        current_pos_ = (time_variables_.transpose() * position_coeff_).coeff(0, 0);
        current_vel_ = (time_variables_.transpose() * velocity_coeff_).coeff(0, 0);
        current_acc_ = (time_variables_.transpose() * acceleration_coeff_).coeff(0, 0);
        current_jerk_ = (time_variables_.transpose() * jerk_coeff_).coeff(0, 0);
        current_snap_ = (time_variables_.transpose() * snap_coeff_).coeff(0, 0);
    }
}

// 현재 상태를 반환하는 메서드들

double NinthOrderPolynomialTrajectory::GetPosition() const
{
    return current_pos_;
}

double NinthOrderPolynomialTrajectory::GetVelocity() const
{
    return current_vel_;
}

double NinthOrderPolynomialTrajectory::GetAcceleration() const
{
    return current_acc_;
}

double NinthOrderPolynomialTrajectory::GetJerk() const
{
    return current_jerk_;
}

double NinthOrderPolynomialTrajectory::GetSnap() const
{
    return current_snap_;
}