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
