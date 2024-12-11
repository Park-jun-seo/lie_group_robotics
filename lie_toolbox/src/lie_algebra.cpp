#include "lie_toolbox/lie_algebra.hpp"

using namespace liegroup;

Eigen::Matrix3d LieAlgebra::Ceil3DVectorOperator(const Eigen::Vector3d &omega)
{
    Eigen::Matrix3d W;
    W << 0, -omega.z(), omega.y(),
        omega.z(), 0, -omega.x(),
        -omega.y(), omega.x(), 0;
    return W;
}

Eigen::Vector3d LieAlgebra::Floor3DVectorOperator(const Eigen::Matrix3d &W)
{
    Eigen::Vector3d omega;
    omega.x() = W(2, 1); // W의 (3,2) 요소
    omega.y() = W(0, 2); // W의 (1,3) 요소
    omega.z() = W(1, 0); // W의 (2,1) 요소
    return omega;
}

Eigen::Matrix4d LieAlgebra::Ceil6DVectorOperator(const Eigen::VectorXd &V)
{
    Eigen::Matrix4d M;
    M << 0, -V(5), V(4), V(0),
        V(5), 0, -V(3), V(1),
        -V(4), V(3), 0, V(2),
        0, 0, 0, 0;
    return M;
}

Eigen::VectorXd LieAlgebra::Floor6DVectorOperator(const Eigen::Matrix4d &M)
{
    Eigen::VectorXd V(6);
    V << M(0, 3), M(1, 3), M(2, 3), M(1, 2), M(2, 0), M(0, 1);
    return V;
}

Eigen::MatrixXd LieAlgebra::AdjointTransform(const Eigen::Matrix4d &T)
{
    Eigen::Matrix3d R = T.block<3, 3>(0, 0);           // 3x3 회전 행렬
    Eigen::Vector3d r = T.block<3, 1>(0, 3);           // 위치 벡터
    Eigen::Matrix3d r_cross = Ceil3DVectorOperator(r); // 위치 벡터를 반대칭 행렬로 변환

    Eigen::MatrixXd AdT(6, 6);
    AdT.block<3, 3>(0, 0) = R;
    AdT.block<3, 3>(0, 3) = r_cross * R;
    AdT.block<3, 3>(3, 0) = Eigen::Matrix3d::Zero(3, 3);
    AdT.block<3, 3>(3, 3) = R;

    return AdT;
}

Eigen::MatrixXd LieAlgebra::AdjointTransformTranspose(const Eigen::Matrix4d &T)
{
    Eigen::Matrix3d R = T.block<3, 3>(0, 0);
    Eigen::Vector3d p = T.block<3, 1>(0, 3);
    Eigen::Matrix3d p_cross = Ceil3DVectorOperator(p);
    Eigen::MatrixXd Ad_inv(6, 6);
    Ad_inv.block<3, 3>(0, 0) = R.transpose();
    Ad_inv.block<3, 3>(0, 3) = Eigen::Matrix3d::Zero();
    Ad_inv.block<3, 3>(3, 0) = -R.transpose() * p_cross;
    Ad_inv.block<3, 3>(3, 3) = R.transpose();
    return Ad_inv;
}

Eigen::MatrixXd LieAlgebra::AdjointTransformInverseTranspose(const Eigen::Matrix4d &T)
{
    Eigen::Matrix3d R = T.block<3, 3>(0, 0);           // 3x3 회전 행렬
    Eigen::Vector3d r = T.block<3, 1>(0, 3);           // 위치 벡터
    Eigen::Matrix3d r_cross = Ceil3DVectorOperator(r); // 위치 벡터를 반대칭 행렬로 변환

    Eigen::MatrixXd AdT(6, 6);
    AdT.block<3, 3>(0, 0) = R;
    AdT.block<3, 3>(0, 3) = Eigen::Matrix3d::Zero(3, 3);
    AdT.block<3, 3>(3, 0) = r_cross * R;
    AdT.block<3, 3>(3, 3) = R;

    return AdT;
}

Eigen::MatrixXd LieAlgebra::AdOperator(const Eigen::VectorXd &twist)
{
    Eigen::Vector3d v = twist.head<3>();
    Eigen::Vector3d omega = twist.tail<3>();
    Eigen::Matrix3d omega_cross = Ceil3DVectorOperator(omega);
    Eigen::Matrix3d v_cross = Ceil3DVectorOperator(v);

    Eigen::MatrixXd ad_matrix(6, 6);
    ad_matrix.block<3, 3>(0, 0) = omega_cross;
    ad_matrix.block<3, 3>(0, 3) = v_cross;
    ad_matrix.block<3, 3>(3, 0) = Eigen::Matrix3d::Zero();
    ad_matrix.block<3, 3>(3, 3) = omega_cross;

    return ad_matrix;
}

Eigen::MatrixXd LieAlgebra::CoAdOperator(const Eigen::VectorXd &twist)
{
    Eigen::Vector3d v = twist.head<3>();
    Eigen::Vector3d omega = twist.tail<3>();
    Eigen::Matrix3d omega_cross = Ceil3DVectorOperator(omega);
    Eigen::Matrix3d v_cross = Ceil3DVectorOperator(v);

    Eigen::MatrixXd ad_matrix(6, 6);
    ad_matrix.block<3, 3>(0, 0) = omega_cross;
    ad_matrix.block<3, 3>(0, 3) = Eigen::Matrix3d::Zero();
    ad_matrix.block<3, 3>(3, 0) = v_cross;
    ad_matrix.block<3, 3>(3, 3) = omega_cross;

    return ad_matrix;
}

Eigen::MatrixXd LieAlgebra::InverseAdjoint(const Eigen::MatrixXd &T)
{
    Eigen::Matrix3d R = T.block<3, 3>(0, 0); // 3x3 회전 행렬
    Eigen::Vector3d p = T.block<3, 1>(0, 3); // 위치 벡터
    Eigen::Matrix3d p_cross = Ceil3DVectorOperator(p); // 위치 벡터를 반대칭 행렬로 변환
    Eigen::MatrixXd Ad_inv(6, 6);
    Ad_inv.block<3, 3>(0, 0) = R.transpose();
    Ad_inv.block<3, 3>(0, 3) = -R.transpose() * p_cross;
    Ad_inv.block<3, 3>(3, 0) = Eigen::Matrix3d::Zero();
    Ad_inv.block<3, 3>(3, 3) = R.transpose();
    return Ad_inv;
}

Eigen::MatrixXd LieAlgebra::CreateAdjointMatrix(const Eigen::VectorXd &V)
{
    assert(V.size() == 6);

    Eigen::Vector3d v = V.head(3);     // 상위 3개 성분 (선형 성분)
    Eigen::Vector3d omega = V.tail(3); // 하위 3개 성분 (각속도 성분)

    Eigen::Matrix3d v_matrix = Ceil3DVectorOperator(v);
    Eigen::Matrix3d omega_matrix = Ceil3DVectorOperator(omega);

    Eigen::MatrixXd adv(6, 6);
    adv.block<3, 3>(0, 0) = v_matrix;
    adv.block<3, 3>(0, 3) = omega_matrix;
    adv.block<3, 3>(3, 0) = Eigen::Matrix3d::Zero();
    adv.block<3, 3>(3, 3) = v_matrix;

    return adv;
}

Eigen::Matrix3d LieAlgebra::ExponentialMapSO3(const Eigen::Vector3d &omega)
{
    double theta = omega.norm();
    if (theta == 0)
        return Eigen::Matrix3d::Identity();

    Eigen::Matrix3d omega_hat = Ceil3DVectorOperator(omega / theta);
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity() + std::sin(theta) * omega_hat + (1 - std::cos(theta)) * (omega_hat * omega_hat);
    return R;
}

Eigen::Matrix4d LieAlgebra::ExponentialMapSE3(const Eigen::VectorXd &xi)
{
    assert(xi.size() == 6);
    Eigen::Vector3d omega = xi.tail<3>();
    Eigen::Vector3d v = xi.head<3>();

    Eigen::Matrix3d R = ExponentialMapSO3(omega);

    double theta = omega.norm();
    if (theta == 0)
    {
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T.block<3, 3>(0, 0) = R;
        T.block<3, 1>(0, 3) = v;
        return T;
    }

    Eigen::Matrix3d omega_hat = Ceil3DVectorOperator(omega);

    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();

    Eigen::Vector3d t = (1 / (theta * theta)) * ((I - R) * (omega_hat * v) + omega * (omega.transpose() * v));

    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = R;
    T.block<3, 1>(0, 3) = t;

    return T;
}

double LieAlgebra::ComputeThetaFromTrace(const Eigen::Matrix3d &R)
{
    double trace = R.trace();
    double value = (3 - trace) / 2;
    value = liegroup::Clamp(value, 0.0, 1.0); // Ensure numerical stability
    double theta = 2 * std::asin(std::sqrt(value));
    return theta;
}

Eigen::Vector3d LieAlgebra::LogarithmMapSO3(const Eigen::Matrix3d &R)
{
    double theta = ComputeThetaFromTrace(R);

    if (theta == 0)
    {
        return Eigen::Vector3d::Zero(); // Identity rotation
    }

    Eigen::Matrix3d skew_symmetric = (R - R.transpose()) / (2 * std::sin(theta));
    Eigen::Vector3d xi = Floor3DVectorOperator(skew_symmetric) * theta;

    return xi;
}

Eigen::VectorXd LieAlgebra::LogarithmMapSE3(const Eigen::Matrix4d &T)
{
    // Extract rotation matrix R and translation vector t
    Eigen::Matrix3d R = T.block<3, 3>(0, 0);
    Eigen::Vector3d t = T.block<3, 1>(0, 3);

    // Compute rotation vector xi_omega using LogarithmMapSO3
    Eigen::Vector3d xi_omega = LogarithmMapSO3(R);

    double theta = xi_omega.norm(); // Magnitude of rotation vector

    // Handle the special case of theta = 0 (no rotation)
    if (theta == 0)
    {
        return (Eigen::VectorXd(6) << t, Eigen::Vector3d::Zero()).finished();
    }

    // Compute translation part xi_v
    Eigen::Matrix3d omega_hat = Ceil3DVectorOperator(xi_omega / theta);
    Eigen::Matrix3d V_inv = Eigen::Matrix3d::Identity() - 0.5 * omega_hat + (1 - theta / (2 * std::tan(theta / 2))) / (theta * theta) * (omega_hat * omega_hat);
    Eigen::Vector3d xi_v = V_inv * t;

    // Combine xi_v and xi_omega into the full twist xi
    Eigen::VectorXd xi(6);
    xi.head<3>() = xi_v;
    xi.tail<3>() = xi_omega;

    return xi;
}

Eigen::Matrix3d LieAlgebra::DifferentialExponentialMapSO3(const Eigen::Vector3d &xi)
{
    double theta = xi.norm();
    if (theta == 0)
    {
        return Eigen::Matrix3d::Identity();
    }

    Eigen::Matrix3d xi_hat = Ceil3DVectorOperator(xi / theta);
    double alpha = std::sin(theta) / theta;
    double beta = (1 - std::cos(theta)) / (theta * theta);

    Eigen::Matrix3d dexp = Eigen::Matrix3d::Identity() + (beta / 2) * xi_hat + ((1 - alpha) / (theta * theta)) * (xi_hat * xi_hat);
    return dexp;
}

Eigen::Matrix3d LieAlgebra::InverseDifferentialExponentialMapSO3(const Eigen::Vector3d &xi)
{
    double theta = xi.norm();
    if (theta == 0)
    {
        return Eigen::Matrix3d::Identity();
    }

    Eigen::Matrix3d xi_hat = Ceil3DVectorOperator(xi / theta);
    double alpha = std::sin(theta) / theta;
    double beta = (1 - std::cos(theta)) / (theta * theta);
    double gamma = alpha / beta;

    Eigen::Matrix3d dexp_inv = Eigen::Matrix3d::Identity() - 0.5 * xi_hat + ((1 - gamma) / (theta * theta)) * (xi_hat * xi_hat);
    return dexp_inv;
}

// Differential of the exponential map for SE(3)
Eigen::MatrixXd LieAlgebra::DifferentialExponentialMapSE3(const Eigen::VectorXd &xi)
{
    assert(xi.size() == 6);

    Eigen::Vector3d xi_v = xi.head<3>();
    Eigen::Vector3d xi_omega = xi.tail<3>();

    double theta = xi_omega.norm();
    Eigen::Matrix3d dexp_omega = DifferentialExponentialMapSO3(xi_omega);

    Eigen::Matrix3d C_xi;
    if (theta == 0)
    {
        C_xi = Ceil3DVectorOperator(xi_v);
    }
    else
    {
        Eigen::Matrix3d omega_hat = Ceil3DVectorOperator(xi_omega / theta);
        double alpha = std::sin(theta) / theta;
        double beta = (1 - std::cos(theta)) / (theta * theta);

        C_xi = (beta / 2) * Ceil3DVectorOperator(xi_v) + ((1 - alpha) / (theta * theta)) * (Ceil3DVectorOperator(xi_v) * omega_hat + omega_hat * Ceil3DVectorOperator(xi_v)) +
               ((alpha - beta) / (theta * theta)) * (xi_omega.dot(xi_v) * omega_hat);
    }

    Eigen::MatrixXd dexp(6, 6);
    dexp.block<3, 3>(0, 0) = dexp_omega;
    dexp.block<3, 3>(0, 3) = C_xi;
    dexp.block<3, 3>(3, 0) = Eigen::Matrix3d::Zero();
    dexp.block<3, 3>(3, 3) = dexp_omega;

    return dexp;
}

// Inverse differential of the exponential map for SE(3)
Eigen::MatrixXd LieAlgebra::InverseDifferentialExponentialMapSE3(const Eigen::VectorXd &xi)
{
    assert(xi.size() == 6);

    Eigen::Vector3d xi_v = xi.head<3>();
    Eigen::Vector3d xi_omega = xi.tail<3>();

    double theta = xi_omega.norm();
    Eigen::Matrix3d dexp_inv_omega = InverseDifferentialExponentialMapSO3(xi_omega);

    Eigen::Matrix3d D_xi;
    if (theta == 0)
    {
        D_xi = -0.5 * Ceil3DVectorOperator(xi_v);
    }
    else
    {
        Eigen::Matrix3d omega_hat = Ceil3DVectorOperator(xi_omega / theta);
        double alpha = std::sin(theta) / theta;
        double beta = (1 - std::cos(theta)) / (theta * theta);
        double gamma = (alpha - beta) / (theta * theta);

        D_xi = -0.5 * Ceil3DVectorOperator(xi_v) + ((1 - gamma) / (theta * theta)) * (Ceil3DVectorOperator(xi_v) * omega_hat + omega_hat * Ceil3DVectorOperator(xi_v)) +
               ((1 / beta + gamma - 2) / (theta * theta)) * (xi_omega.dot(xi_v) * omega_hat);
    }

    Eigen::MatrixXd dexp_inv(6, 6);
    dexp_inv.block<3, 3>(0, 0) = dexp_inv_omega;
    dexp_inv.block<3, 3>(0, 3) = D_xi;
    dexp_inv.block<3, 3>(3, 0) = Eigen::Matrix3d::Zero();
    dexp_inv.block<3, 3>(3, 3) = dexp_inv_omega;

    return dexp_inv;
}