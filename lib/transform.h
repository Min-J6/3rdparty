#pragma once
#include <eigen3/Eigen/Dense>

#include <Eigen/Dense>
#include <iostream>
#include <cassert>

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


class transform {
public:
    using AngleAxis = Eigen::AngleAxisd;
    using Isometry = Eigen::Isometry3d;
    using mat4 = Eigen::Matrix4d;
    using vec3 = Eigen::Vector3d;
    using quat = Eigen::Quaterniond;
    using mat3 = Eigen::Matrix3d;
    template<int Rows, int Cols>
    using mat = Eigen::Matrix<double, Rows, Cols>;
    template<int Rows>
    using vec = Eigen::Matrix<double, Rows, 1>;



private:
    Isometry T_;

public:
    transform() : T_(Isometry::Identity()) {}
    explicit transform(const Isometry& iso) : T_(iso) {}
    explicit transform(const mat4& H) : T_(H) {}

    // 이동 -> 회전 순서
    transform(const quat& q, const vec3& t) : T_(Isometry::Identity()) {
        T_.rotate(q);
        T_.pretranslate(t);;
    }


    // --- 접근자 (Getters) --- //
    const mat4& matrix() const          { return T_.matrix();                           }
    mat3 rotation() const               { return T_.rotation();                         }
    quat quaternion() const             { return quat(T_.rotation());                   }
    vec3 translation() const            { return T_.translation();                      }

    double const  z()     const         { return T_.translation().z();                  }
    double const  x()     const         { return T_.translation().x();                  }
    double const  y()     const         { return T_.translation().y();                  }


    // ZYX 오일러 각도 (Yaw-Pitch-Roll)
    double const  roll()  const         { return T_.rotation().eulerAngles(2, 1, 0)[2]; }
    double const  pitch() const         { return T_.rotation().eulerAngles(2, 1, 0)[1]; }
    double const  yaw()   const         { return T_.rotation().eulerAngles(2, 1, 0)[0]; }

    transform inverse() const           { return transform(T_.inverse());               }



    // --- 수정자 (Setters) --- //
    void setIdentity()                  { T_.setIdentity();                             }
    void setTranslation(const vec3& t)  { T_.translation() = t;                         }
    void setRotation(const quat& q)     { T_.linear() = q.toRotationMatrix();           }
    void setRotation(const mat3& R)     { T_.linear() = R;                              }



    // --- 연산자 오버로딩 --- //
    transform& operator=(const mat4& m) {
        this->T_ = m;
        return *this;
    }

    transform operator*(const transform& other) const {
        return transform(this->T_ * other.T_);
    }

    vec3 operator*(const vec3& point) const {
        return this->T_ * point;
    }

    transform operator-(const transform& other) const {
        mat4 result = this->matrix() - other.matrix();
        return transform(result);
    }

    double operator()(int row, int col) const {
        assert(row >= 0 && row < 4 && col >= 0 && col < 4);
        return T_.matrix()(row, col);
    }

    double& operator()(int row, int col) {
        assert(row < 3 && "Cannot modify the last row of an Isometry (it's fixed at [0 0 0 1])");

        if (col < 3) {
            return T_.linear()(row, col);
        } else if (col == 3) {
            return T_.translation()(row);
        }
    }


    // --- 유틸리티 --- //

    friend std::ostream &operator<<(std::ostream &os, const transform &rt)
    {
        const Eigen::IOFormat fmt(6, 0, ", ", "\n", "[", "]");
        os << rt.matrix().format(fmt);
        return os;
    }
};


// Jacobian 인버스
inline transform::mat<6,12> jInv(const transform::mat<12, 6>& J) {
    transform::mat<3, 6> J_pos = J.block<3, 6>(0, 0);                           // 위치 자코비안(J_pos) 분리
    transform::mat<3, 3> JJ_T_pos = J_pos * J_pos.transpose();                  // J_pos * J_pos^T 계산
    Eigen::SelfAdjointEigenSolver<transform::mat<3, 3>> eigensolver(JJ_T_pos);  // 고유값( 계산
    double min_eigenvalue = eigensolver.eigenvalues()(0);                       // 고유값들은 오름차순으로 정렬


    // 최소 고유값을 기준으로 새로운 조작성 지수(w_min) 정의
    double w_min = std::sqrt(std::max(0.0, min_eigenvalue));


    // 동적 람다(λ) 계산
    double lambda_max = 0.9;       // 튜닝 필요
    double w_min_threshold = 0.14; // 튜닝 필요
    double lambda = 0.01;          // 튜닝 필요

    if (w_min < w_min_threshold) {
        lambda = lambda_max * (1.0 - (w_min / w_min_threshold));
    }


    // DLS 계산에 동적 람다 적용
    transform::mat<12, 12> I = transform::mat<12, 12>::Identity();
    return J.transpose() * (J * J.transpose() + lambda * lambda * I).inverse();
}