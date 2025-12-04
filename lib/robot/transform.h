#pragma once
#include <eigen3/Eigen/Dense>

#include <Eigen/Dense>
#include <iostream>
#include <cassert>
#include <vector>

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




template<int Rows, int Cols>
using mat = Eigen::Matrix<double, Rows, Cols>;
template<int Rows>
using vec = Eigen::Matrix<double, Rows, 1>;

using mat4 = Eigen::Matrix4d;
using vec3 = Eigen::Vector3d;
using quat = Eigen::Quaterniond;
using mat3 = Eigen::Matrix3d;
using AngleAxis = Eigen::AngleAxisd;
using Isometry = Eigen::Isometry3d;



class Transform {
private:
    Isometry T_;

public:
    Transform() : T_(Isometry::Identity()) {}
    explicit Transform(const Isometry& iso) : T_(iso) {}
    explicit Transform(const mat4& H) : T_(H) {}

    // 이동 → 회전으로 보고 FK를 설계하면 쉽다
    Transform(const quat& q, const vec3& t) : T_(Isometry::Identity()) {
        T_.rotate(q);
        T_.pretranslate(t);
    }

    Transform(const AngleAxis& r, const vec3& t) : T_(Isometry::Identity()) {
        T_.rotate(r);
        T_.pretranslate(t);
    }


    /// Getter
    const mat4& matrix()  const                     { return T_.matrix();                                     }
    vec3 translation()    const                     { return T_.translation();                                }
    mat3 rotation()       const                     { return T_.rotation();                                   }


    /// Setter
    void translate(const vec3& t)                   { T_.translation() = t;                                   }
    void translate(double x, double y, double z)    { T_.translation() << x, y, z;                            }
    void rotate(double rad, const vec3& axis)       { T_.linear() = AngleAxis(rad, axis).toRotationMatrix();  }


    /// Transform 대입 연산자
    Transform& operator=(const mat4& m) {
        this->T_ = m;
        return *this;
    }


    /// Transform 곱셈 연산자
    Transform operator*(const Transform& other) const {
        return Transform(this->T_ * other.T_);
    }


    /// Transform 뺄셈 연산자
    vec<6> operator-(const Transform& other) const {
        vec<6> error_twist;

        // 1. 위치 오차 (Position Error)
        error_twist.head<3>() = this->translation() - other.translation();

        // 2. 회전 오차 (Orientation Error - Log Map 근사)
        const mat3 R_cur = other.rotation(); // other: current_tf
        const mat3 R_tar = this->rotation(); // this: target_tf

        // 상대 회전 행렬: R_diff = R_tar * R_cur^T
        const mat3 R_diff = R_tar * R_cur.transpose();

        // 로그 맵 근사 (회전 벡터 계산)
        const vec3 ori_error {
            0.5 * (R_diff(2, 1) - R_diff(1, 2)),
            0.5 * (R_diff(0, 2) - R_diff(2, 0)),
            0.5 * (R_diff(1, 0) - R_diff(0, 1))
        };


        error_twist.tail<3>() = ori_error;

        return error_twist;
    }


    /// 매트릭스 접근자
    double operator()(int row, int col) const {
        return T_.matrix()(row, col);
    }


    /// 출력 함수
    friend std::ostream& operator<<(std::ostream& os, const Transform& rt)
    {
        const Eigen::IOFormat fmt(6, 0, ", ", "\n", "[", "]");
        os << rt.matrix().format(fmt);
        return os;
    }


    static vec3 rpy(const Transform& tf)
    {
        // RPY 추출은 Transform의 회전 행렬(3x3)을 사용합니다.
        const Eigen::Matrix3d R = tf.rotation(); // Transform::rot()으로 3x3 회전 행렬을 가져옴

        // Eigen::Matrix3d는 0-based 인덱싱을 사용합니다: R(row, col)

        // 1. Pitch (p) 계산: R_31 요소 사용
        // Pitch 범위: [-pi/2, pi/2]
        double p = std::asin(-R(2, 0)); // R_31

        // Gimbal Lock (짐벌 잠금) 근처인지 확인
        // Pitch가 +/- 90도 근처일 때 (cos(p)가 0에 가까울 때)
        constexpr double PITCH_THRESHOLD = 1e-6;

        // std::cos(p) 대신 R(2, 2)와 R(2, 1)을 사용해 R(2, 0)의 오차를 회피하는 것이 더 일반적입니다.
        double cos_p_squared = R(2, 1) * R(2, 1) + R(2, 2) * R(2, 2);
        double cos_p = std::sqrt(cos_p_squared);

        double r, y;

        if (cos_p < PITCH_THRESHOLD) { // Gimbal Lock 발생! (p = +/- 90 deg)
            // Roll과 Yaw가 합쳐지며, Roll을 0으로 설정하고 합쳐진 회전량을 Yaw에 할당합니다.
            r = 0.0;
            // Yaw 계산: atan2(R22, R12)를 사용 (R22와 R12의 부호는 Pitch에 따라 다르게 해석됨)
            // 여기서는 R_12, R_11 대신 R_22, R_12를 사용하는 간략화된 식을 사용합니다.
            // Yaw (y) = atan2(-R_12, R_13) for p=90
            // Yaw (y) = atan2(R_12, R_13) for p=-90
            // 가장 안정적인 단일 공식:
            y = std::atan2(R(0, 1), R(0, 2));

            // R(0, 1)과 R(0, 2)는 R_12와 R_13
            // R(1, 1)과 R(1, 2)는 R_22와 R_23

        } else { // 정상 상황
            // 2. Yaw (y) 계산: R_21과 R_11 사용
            y = std::atan2(R(1, 0), R(0, 0)); // R_21 / R_11

            // 3. Roll (r) 계산: R_32와 R_33 사용
            r = std::atan2(R(2, 1), R(2, 2)); // R_32 / R_33
        }

        // 원하는 vec3 형태로 반환 (Roll, Pitch, Yaw 순서)
        return vec3(r, p, y);
    }
};