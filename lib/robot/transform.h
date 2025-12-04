#pragma once
// Eigen 헤더
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <cmath>
#include <vector>
#include <unsupported/Eigen/src/AutoDiff/AutoDiffScalar.h>

// ------------------------------------------------------------------
// 매크로 및 상수 정의
// ------------------------------------------------------------------
#ifndef NORM_DEG_180
#define NORM_DEG_180(deg) (fmod(fmod((deg) + 180.0, 360.0) + 360.0, 360.0) - 180.0)
#endif
#ifndef NORM_RAD_180
#define NORM_RAD_180(rad) (fmod(fmod((rad) + M_PI, 2 * M_PI) + 2 * M_PI, 2 * M_PI) - M_PI)
#endif
#ifndef DEG_TO_RAD
#define DEG_TO_RAD(x) ((x) * 0.017453292519943295769236907684886)
#endif
#ifndef RAD_TO_DEG
#define RAD_TO_DEG(x) ((x) * 57.29577951308232087679815481410518)
#endif

// 기존 편의성 타입 (double 전용)
template<int Rows, int Cols>
using mat = Eigen::Matrix<double, Rows, Cols>;
template<int Rows>
using vec = Eigen::Matrix<double, Rows, 1>;

using mat4 = Eigen::Matrix4d;
using vec3 = Eigen::Vector3d;
using quat = Eigen::Quaterniond;
using mat3 = Eigen::Matrix3d;
using AngleAxis = Eigen::AngleAxisd;


namespace Robot
{



template <typename Scalar>
class TransformT {
public:
    // Eigen 내부 타입 정의
    using Isometry_t = Eigen::Transform<Scalar, 3, Eigen::Isometry>;
    using Matrix3_t  = Eigen::Matrix<Scalar, 3, 3>;
    using Vector3_t  = Eigen::Matrix<Scalar, 3, 1>;
    using Matrix4_t  = Eigen::Matrix<Scalar, 4, 4>;
    using AngleAxis_t= Eigen::AngleAxis<Scalar>;
    using Quat_t     = Eigen::Quaternion<Scalar>;
    using Twist_t    = Eigen::Matrix<Scalar, 6, 1>;

private:
    Isometry_t T_;

public:
    TransformT() : T_(Isometry_t::Identity()) {}
    explicit TransformT(const Isometry_t& iso) : T_(iso) {}
    explicit TransformT(const Matrix4_t& H) : T_(H) {}

    TransformT(const Quat_t& q, const Vector3_t& t) : T_(Isometry_t::Identity())
    {
        T_.rotate(q);
        T_.pretranslate(t);
    }

    TransformT(const AngleAxis_t& r, const Vector3_t& t) : T_(Isometry_t::Identity())
    {
        T_.rotate(r);
        T_.pretranslate(t);
    }



    /// Getter
    const Matrix4_t& matrix() const { return T_.matrix(); }
    Vector3_t translation()   const { return T_.translation(); }
    Matrix3_t rotation()      const { return T_.rotation(); }



    /// Setter
    void translate(const Vector3_t& t) { T_.translation() = t; }
    void rotate(const AngleAxis_t& r)  { T_.linear() = r.toRotationMatrix(); }




    /// 대입 연산자
    TransformT& operator=(const Matrix4_t& m)
    {
        this->T_ = m;
        return *this;
    }


    /// 곱셈 연산자
    TransformT operator*(const TransformT& other) const
    {
        return TransformT(this->T_ * other.T_);
    }


    /// 출력 연산자
    friend std::ostream& operator<<(std::ostream& os, const TransformT& rt)
    {
        os << rt.matrix();
        return os;
    }


    /// Twist (Error) 계산
    static Twist_t twist(const TransformT& tf_cur, const TransformT& tf_tar)
    {
        Twist_t error_twist;

        // 1. 위치 오차 (Linear Error): p_tar - p_cur
        error_twist.template head<3>() = tf_tar.translation() - tf_cur.translation();

        // 2. 회전 오차 (Angular Error): Log Map 근사
        // R_diff = R_tar * R_cur^T (Current에서 바라본 Target의 회전)
        const Matrix3_t R_cur = tf_cur.rotation();
        const Matrix3_t R_tar = tf_tar.rotation();
        const Matrix3_t R_diff = R_tar * R_cur.transpose();

        Vector3_t ori_error;
        // Skew-symmetric 행렬에서 벡터 추출 (so(3) -> R^3)
        ori_error[0] = 0.5 * (R_diff(2, 1) - R_diff(1, 2));
        ori_error[1] = 0.5 * (R_diff(0, 2) - R_diff(2, 0));
        ori_error[2] = 0.5 * (R_diff(1, 0) - R_diff(0, 1));

        error_twist.template tail<3>() = ori_error;

        return error_twist;
    }


    /// RPY 변환
    static Vector3_t rpy(const TransformT& tf)
    {
        const Matrix3_t R = tf.rotation();
        const Scalar p = std::asin(-R(2, 0));

        Scalar r, y;
        y = std::atan2(R(1, 0), R(0, 0));
        r = std::atan2(R(2, 1), R(2, 2));

        Vector3_t res;
        res << r, p, y;
        return res;
    }
};


using Transform     = TransformT<double>;
using ADScalar      = Eigen::AutoDiffScalar<Eigen::Matrix<double, 6, 1>>; // 6개의 입력 변수(Joint)에 대한 미분값을 가지는 스칼라
using ADTransform   = TransformT<ADScalar>;

template<int Rows>
using ADVec = Eigen::Matrix<ADScalar, Rows, 1>;
template<int Rows, int Cols>
using ADMat = Eigen::Matrix<ADScalar, Rows, Cols>;
}

