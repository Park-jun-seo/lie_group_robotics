#ifndef LIE_ALGEBRA_HPP_
#define LIE_ALGEBRA_HPP_

#include <eigen3/Eigen/Dense>
#include <iostream>
#include <cmath>

namespace liegroup
{
    class LieAlgebra
    {
    public:
        // 3x1 벡터를 받아 3x3 반대칭 행렬로 변환하는 함수
        static Eigen::Matrix3d Ceil3DVectorOperator(const Eigen::Vector3d &omega);
        // 3x3 반대칭 행렬을 받아 3x1 벡터로 변환하는 함수
        static Eigen::Vector3d Floor3DVectorOperator(const Eigen::Matrix3d &W);
        // 6x1 벡터를 받아 4x4 반대칭 행렬로 변환하는 함수
        static Eigen::Matrix4d Ceil6DVectorOperator(const Eigen::VectorXd &V);
        // 4x4 반대칭 행렬을 받아 6x1 벡터로 변환하는 함수
        static Eigen::VectorXd Floor6DVectorOperator(const Eigen::Matrix4d &M);
        // 4x4 변환 행렬을 받아 6x6 Adjoint 행렬로 변환하는 함수
        static Eigen::MatrixXd AdjointTransform(const Eigen::Matrix4d &T);
        // 4x4 변환 행렬을 받아 6x6 Adjoint 역행렬의 전치행렬로 변환하는 함수
        static Eigen::MatrixXd AdjointTransformInverseTranspose(const Eigen::Matrix4d &T);
        // 4x4 변환 행렬을 받아 6x6 Adjoint 전치행렬로 변환하는 함수
        static Eigen::MatrixXd AdjointTransformTranspose(const Eigen::Matrix4d &T);
        // 6x1 벡터를 받아 6x6 adjoint 행렬로 변환하는 함수
        static Eigen::MatrixXd CreateAdjointMatrix(const Eigen::VectorXd &V);
        // 4x4 변환 행렬을 받아 6x6 Adjoint 역행렬로 변환하는 함수
        static Eigen::MatrixXd InverseAdjoint(const Eigen::MatrixXd &T);
        // adjoint operator 를 계산하는 함수, 여기서 twist는 [v; omega] 형태를 가집니다.
        static Eigen::MatrixXd AdOperator(const Eigen::VectorXd &twist);
        // co adjoint 를 계산하는 함수 -1 * adjoint operator의 transpose
        static Eigen::MatrixXd CoAdOperator(const Eigen::VectorXd &twist);
    };

    inline Eigen::Matrix4d GetTransformationMatrix(const double x, double y, double z, double roll, double pitch, double yaw)
    {
        // 위치 변환 행렬
        Eigen::Matrix4d translation = Eigen::Matrix4d::Identity();
        translation(0, 3) = x;
        translation(1, 3) = y;
        translation(2, 3) = z;

        // 회전 행렬 (Z-Y-X 순서로 Roll-Pitch-Yaw 적용)
        Eigen::Matrix3d rotation;

        double cosRoll = std::cos(roll);
        double sinRoll = std::sin(roll);
        double cosPitch = std::cos(pitch);
        double sinPitch = std::sin(pitch);
        double cosYaw = std::cos(yaw);
        double sinYaw = std::sin(yaw);

        rotation << cosYaw * cosPitch, cosYaw * sinPitch * sinRoll - sinYaw * cosRoll, cosYaw * sinPitch * cosRoll + sinYaw * sinRoll,
            sinYaw * cosPitch, sinYaw * sinPitch * sinRoll + cosYaw * cosRoll, sinYaw * sinPitch * cosRoll - cosYaw * sinRoll,
            -sinPitch, cosPitch * sinRoll, cosPitch * cosRoll;

        // 4x4 회전 변환 행렬
        Eigen::Matrix4d rotation4x4 = Eigen::Matrix4d::Identity();
        rotation4x4.block<3, 3>(0, 0) = rotation;

        // 최종 변환 행렬 (회전 + 평행 이동)
        Eigen::Matrix4d transformation = translation * rotation4x4;

        return transformation;
    }

    inline Eigen::Vector3d RotationMatrixToRPY(const Eigen::Matrix3d &rotationMatrix)
    {
        Eigen::Vector3d rpy;

        double sy = std::sqrt(rotationMatrix(0, 0) * rotationMatrix(0, 0) + rotationMatrix(1, 0) * rotationMatrix(1, 0));

        bool singular = sy < 1e-6; // 임계값으로 수평 상태 판정

        if (!singular)
        {
            rpy[0] = std::atan2(rotationMatrix(2, 1), rotationMatrix(2, 2)); // roll
            rpy[1] = std::atan2(-rotationMatrix(2, 0), sy);                  // pitch
            rpy[2] = std::atan2(rotationMatrix(1, 0), rotationMatrix(0, 0)); // yaw
        }
        else
        {
            rpy[0] = std::atan2(-rotationMatrix(1, 2), rotationMatrix(1, 1)); // roll
            rpy[1] = std::atan2(-rotationMatrix(2, 0), sy);                   // pitch
            rpy[2] = 0;                                                       // yaw (불명확)
        }

        return rpy;
    }

}

#endif
