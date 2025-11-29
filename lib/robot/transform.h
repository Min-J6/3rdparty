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



class Transform {
public:
    using Isometry = Eigen::Isometry3d;
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


    // --- 접근자 (Getters) --- //
    const mat4& matrix()  const           { return T_.matrix();                           }
    vec3 trans()          const           { return T_.translation();                      }
    mat3 rot()            const           { return T_.rotation();                         }
    quat quaternion()     const           { return quat(T_.rotation());                   }

    double const  z()     const           { return T_.translation().z();                  }
    double const  x()     const           { return T_.translation().x();                  }
    double const  y()     const           { return T_.translation().y();                  }


    // ZYX 오일러 각도 (Yaw-Pitch-Roll)
    double const  roll()  const           { return T_.rotation().eulerAngles(2, 1, 0)[2]; }
    double const  pitch() const           { return T_.rotation().eulerAngles(2, 1, 0)[1]; }
    double const  yaw()   const           { return T_.rotation().eulerAngles(2, 1, 0)[0]; }

    Transform inverse()   const           { return Transform(T_.inverse());               }



    // --- 수정자 (Setters) --- //
    void set_identity()                   { T_.setIdentity();                             }
    void set_trans(const vec3& t)         { T_.translation() = t;                         }
    void set_rotation(const quat& q)      { T_.linear() = q.toRotationMatrix();           }
    void set_rotation(const mat3& R)      { T_.linear() = R;                              }
    void set_rotation(const AngleAxis& r) { T_.linear() = r.toRotationMatrix();           }
    void set_rotation(double rad, const vec3& axis) { T_.linear() = AngleAxis(rad, axis).toRotationMatrix(); }


    // --- 연산자 오버로딩 --- //
    Transform& operator=(const mat4& m) {
        this->T_ = m;
        return *this;
    }

    Transform operator*(const Transform& other) const {
        return Transform(this->T_ * other.T_);
    }

    vec3 operator*(const vec3& point) const {
        return this->T_ * point;
    }

    vec<6> operator-(const Transform& other) const {
        vec<6> error_twist;

        // 1. 위치 오차 (Position Error)
        error_twist.head<3>() = this->trans() - other.trans();

        // 2. 회전 오차 (Orientation Error - Log Map 근사)
        const mat3 R_cur = other.rot(); // other: current_tf
        const mat3 R_tar = this->rot(); // this: target_tf

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

    double operator()(int row, int col) const {
        return T_.matrix()(row, col);
    }

    double& operator()(int row, int col) {
        if (col < 3) {
            return T_.linear()(row, col);
        } else if (col == 3) {
            return T_.translation()(row);
        }
    }


    // --- 유틸리티 --- //

    friend std::ostream& operator<<(std::ostream& os, const Transform& rt)
    {
        const Eigen::IOFormat fmt(6, 0, ", ", "\n", "[", "]");
        os << rt.matrix().format(fmt);
        return os;
    }
};