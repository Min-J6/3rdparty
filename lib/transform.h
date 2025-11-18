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

extern "C"{
    extern void dgesvd_(char* jobu, char* jobvt, int* m, int* n, double* a,
                        int* lda, double* s, double* u, int* ldu, double* vt,
                        int* ldvt, double* work, int* lwork, int* info);
}


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
    const mat4& matrix()  const         { return T_.matrix();                           }
    mat3 rotation()       const         { return T_.rotation();                         }
    quat quaternion()     const         { return quat(T_.rotation());                   }
    vec3 translation()    const         { return T_.translation();                      }

    double const  z()     const         { return T_.translation().z();                  }
    double const  x()     const         { return T_.translation().x();                  }
    double const  y()     const         { return T_.translation().y();                  }


    // ZYX 오일러 각도 (Yaw-Pitch-Roll)
    double const  roll()  const         { return T_.rotation().eulerAngles(2, 1, 0)[2]; }
    double const  pitch() const         { return T_.rotation().eulerAngles(2, 1, 0)[1]; }
    double const  yaw()   const         { return T_.rotation().eulerAngles(2, 1, 0)[0]; }

    transform inverse()   const         { return transform(T_.inverse());               }



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




// ============================================================================
// Jacobian 인버스
// ============================================================================
inline transform::mat<6,12> jInv(const transform::mat<12, 6>& J) {
    // Moore-Penrose 슈도 인버스: J^+ = (J^T * J)^-1 * J^T
    return (J.transpose() * J).inverse() * J.transpose();
}


inline transform::mat<6,12> jInv_svd(const transform::mat<12, 6>& J) {
    // SVD 기반 Moore-Penrose 슈도 인버스
    // J = U * Σ * V^T 일 때, J^+ = V * Σ^+ * U^T

    // 동적 크기 행렬로 변환하여 SVD 수행
    Eigen::MatrixXd J_dynamic = J;
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(J_dynamic, Eigen::ComputeThinU | Eigen::ComputeThinV);

    const double tolerance = 1e-10;  // singular value threshold

    // Σ^+ 계산 (singular values의 역수, threshold 이하는 0으로)
    Eigen::VectorXd singularValuesInv = svd.singularValues();
    for (int i = 0; i < singularValuesInv.size(); ++i) {
        if (singularValuesInv(i) > tolerance) {
            singularValuesInv(i) = 1.0 / singularValuesInv(i);
        } else {
            singularValuesInv(i) = 0.0;
        }
    }

    // J^+ = V * Σ^+ * U^T
    // V: 6×6, Σ^+: 6×6 (diagonal), U^T: 6×12
    Eigen::MatrixXd pinv = svd.matrixV() * singularValuesInv.asDiagonal() * svd.matrixU().transpose();

    // 고정 크기 행렬로 변환하여 반환
    return pinv;
}



Eigen::MatrixXd pInv_LAPACK(const Eigen::MatrixXd& A, double epsilon = std::numeric_limits<double>::epsilon()) {
    int m = A.rows();
    int n = A.cols();

    // LAPACK 함수는 입력 행렬 A를 덮어쓰므로 복사본을 만듭니다.
    Eigen::MatrixXd Acopy = A;

    // U 행렬과 VT (V transpose) 행렬을 저장할 공간 할당
    Eigen::MatrixXd U(m, m);
    Eigen::MatrixXd Vt(n, n);
    // 특이값 S를 저장할 벡터 (min(m, n) 크기)
    Eigen::VectorXd S_values(std::min(m, n));

    // LAPACK 인자 설정
    char jobu = 'A';  // U 행렬의 모든 열 계산
    char jobvt = 'A'; // V^T 행렬의 모든 행 계산
    int lda = m;
    int ldu = m;
    int ldvt = n;
    int info = 0;

    // 최적의 워크스페이스 크기를 알기 위해 lwork = -1로 초기 호출
    double work_size_query = 0.0;
    int lwork = -1;

    dgesvd_(&jobu, &jobvt, &m, &n, Acopy.data(), &lda,
            S_values.data(), U.data(), &ldu, Vt.data(), &ldvt,
            &work_size_query, &lwork, &info);

    if (info != 0) {
        throw std::runtime_error("LAPACK dgesvd_ workspace query failed.");
    }

    // 최적의 워크스페이스 크기 설정 및 할당
    lwork = static_cast<int>(work_size_query);
    std::vector<double> work(lwork);

    // 실제 SVD 계산 호출
    dgesvd_(&jobu, &jobvt, &m, &n, Acopy.data(), &lda,
            S_values.data(), U.data(), &ldu, Vt.data(), &ldvt,
            work.data(), &lwork, &info);

    if (info != 0) {
        throw std::runtime_error("LAPACK dgesvd_ computation failed.");
    }

    // --- 슈도 인버스 계산 과정 (Eigen 활용) ---

    // 1. 특이값 역수 계산 및 임계값 처리
    double tolerance = epsilon * std::max(m, n) * S_values(0);
    for (int i = 0; i < S_values.size(); ++i) {
        if (S_values(i) > tolerance) {
            S_values(i) = 1.0 / S_values(i);
        } else {
            S_values(i) = 0.0;
        }
    }

    // 2. 역수화된 특이값으로 대각 행렬 S+ 생성
    // S_values는 벡터이므로, DiagonalMatrix 클래스를 사용하여 대각 행렬로 만듭니다.
    Eigen::MatrixXd S_plus = Eigen::MatrixXd::Zero(n, m);
    for (int i = 0; i < S_values.size(); ++i) {
        S_plus(i, i) = S_values(i);
    }

    // 3. 슈도 인버스 공식 A+ = V * S+ * U^T 적용
    // Vt는 이미 V^T 형태이므로 바로 사용합니다. (V = Vt.transpose())
    Eigen::MatrixXd A_plus = Vt.transpose() * S_plus * U.transpose();

    return A_plus;
}



/*
inline transform::mat<6,12> jInv(const transform::mat<12, 6>& J) {
    transform::mat<3, 6> J_pos = J.block<3, 6>(0, 0);
    transform::mat<3, 3> JJ_T_pos = J_pos * J_pos.transpose();
    Eigen::SelfAdjointEigenSolver<transform::mat<3, 3>> eigensolver(JJ_T_pos);
    double min_eigenvalue = eigensolver.eigenvalues()(0);

    double w_min = std::sqrt(std::max(0.0, min_eigenvalue));

    double lambda_max = 0.9;
    double w_min_threshold = 0.14;
    double lambda = 0.01;

    if (w_min < w_min_threshold) {
        lambda = lambda_max * (1.0 - (w_min / w_min_threshold));
    }

    transform::mat<12, 12> I = transform::mat<12, 12>::Identity();
    return J.transpose() * (J * J.transpose() + lambda * lambda * I).inverse();
}
*/


inline transform::mat<6,12> jInv_SVD_Damped(const transform::mat<12, 6>& J, double lambda = 0.01) {
    double lambda_max       = 0.8;  // 최대값
    double w_min_threshold  = 0.10; // 임계값


    // 람다 계산
    transform::mat<3, 6> J_pos = J.block<3, 6>(0, 0);
    transform::mat<3, 3> JJ_T_pos = J_pos * J_pos.transpose();
    Eigen::SelfAdjointEigenSolver<transform::mat<3, 3>> eigensolver(JJ_T_pos);
    double min_eigenvalue = eigensolver.eigenvalues()(0);
    double w_min = std::sqrt(std::max(0.0, min_eigenvalue));


    if (w_min < w_min_threshold) {
        lambda = lambda_max * (1.0 - (w_min / w_min_threshold));
    }

    std::cout << "w_min: " << w_min << ", lambda: " << lambda << "\n" << std::endl;


    //  SVD 계산
    Eigen::JacobiSVD<transform::mat<12, 6>> svd(
        J,
        Eigen::ComputeFullU | Eigen::ComputeFullV
    );


    // 특이값(Singular Values)
    const auto& s = svd.singularValues();

    Eigen::Matrix<double, 6, 12> S_plus = Eigen::Matrix<double, 6, 12>::Zero();

    for (int i = 0; i < s.size(); ++i) {
        double sigma = s(i);

        // DLS: sigma / (sigma^2 + lambda^2)
        double damped_inv_sigma = sigma / (sigma * sigma + lambda * lambda);
        S_plus(i, i) = damped_inv_sigma;
    }



    // 댐핑된 유사 역행렬 J+ = V * S+ * U^T 계산
    return svd.matrixV() * S_plus * svd.matrixU().transpose();
}







// ============================================================================
// 관절 한계 구조체
// ============================================================================
struct JointLimits {
    std::vector<double> q_min;  // 최소 관절 각도 [rad]
    std::vector<double> q_max;  // 최대 관절 각도 [rad]

    JointLimits(int num_joints) {
        q_min.resize(num_joints, -M_PI);
        q_max.resize(num_joints, M_PI);
    }

    // 특정 관절의 한계 설정
    void setLimit(int joint_idx, double min_rad, double max_rad) {
        assert(joint_idx >= 0 && joint_idx < q_min.size());
        q_min[joint_idx] = min_rad;
        q_max[joint_idx] = max_rad;
    }
};



// ============================================================================
// 방법 1: Simple Clamping (가장 단순하고 실용적)
// ============================================================================
inline void clampJointAngles(transform::vec<6>& q, const JointLimits& limits) {
    for (int i = 0; i < 6; ++i) {
        q(i) = std::max(limits.q_min[i], std::min(q(i), limits.q_max[i]));
    }
}


// ============================================================================
// 방법 2: Soft Clamping with Damping (한계 근처에서 스텝 크기 감소)
// ============================================================================
inline double computeJointLimitScale(const transform::vec<6>& q,
                                     const JointLimits& limits,
                                     double activation_ratio = 0.2) {
    double scale = 1.0;

    for (int i = 0; i < 6; ++i) {
        double range = limits.q_max[i] - limits.q_min[i];
        double margin_min = q(i) - limits.q_min[i];
        double margin_max = limits.q_max[i] - q(i);
        double min_margin = std::min(margin_min, margin_max);

        double activation_threshold = activation_ratio * range;

        if (min_margin < activation_threshold) {
            double joint_scale = min_margin / activation_threshold;
            scale = std::min(scale, joint_scale);
        }
    }

    return scale;
}


// ============================================================================
// 방법 3: Repulsive Potential (한계 근처에서 반발력 생성)
// ============================================================================
inline transform::vec<6> computeRepulsiveForce(const transform::vec<6>& q,
                                               const JointLimits& limits,
                                               double threshold_ratio = 0.15,
                                               double gain = 0.05) {
    transform::vec<6> repulsive = transform::vec<6>::Zero();

    for (int i = 0; i < 6; ++i) {
        double range = limits.q_max[i] - limits.q_min[i];
        double threshold = threshold_ratio * range;

        double dist_min = q(i) - limits.q_min[i];
        double dist_max = limits.q_max[i] - q(i);

        // 최소 한계에 가까울 때 양의 반발력 (위로 밀어냄)
        if (dist_min < threshold && dist_min > 0) {
            repulsive(i) += gain * (1.0 / dist_min - 1.0 / threshold);
        }

        // 최대 한계에 가까울 때 음의 반발력 (아래로 밀어냄)
        if (dist_max < threshold && dist_max > 0) {
            repulsive(i) -= gain * (1.0 / dist_max - 1.0 / threshold);
        }
    }

    return repulsive;
}


// ============================================================================
// 통합 IK 업데이트 함수 (여러 방법 조합 가능)
// ============================================================================
enum class JointLimitMethod {
    NONE,                // 제약 없음
    SIMPLE_CLAMP,        // 단순 클램핑
    SOFT_CLAMP,          // 부드러운 클램핑 (스텝 크기 감소)
    REPULSIVE,           // 반발력
    SOFT_CLAMP_REPULSIVE // 부드러운 클램핑 + 반발력 조합
};

inline void applyJointLimitConstraint(
    transform::vec<6>& dq,
    transform::vec<6>& q,
    const JointLimits& limits,
    JointLimitMethod method = JointLimitMethod::SOFT_CLAMP_REPULSIVE,
    double activation_ratio = 0.02,
    double repulsive_gain = 0.01)
{
    switch (method) {
        case JointLimitMethod::NONE:
            // 제약 없음, dq 그대로 적용
            break;

        case JointLimitMethod::SIMPLE_CLAMP:
            // dq 적용 후 클램핑
            q += dq;
            clampJointAngles(q, limits);
            dq.setZero();  // 이미 적용했으므로 dq 초기화
            break;

        case JointLimitMethod::SOFT_CLAMP:
            {
                // 한계 근처에서 스텝 크기 감소
                double scale = computeJointLimitScale(q, limits, activation_ratio);
                dq *= scale;
            }
            break;

        case JointLimitMethod::REPULSIVE:
            {
                // 반발력 추가
                transform::vec<6> repulsive = computeRepulsiveForce(
                    q, limits, activation_ratio, repulsive_gain);
                dq += repulsive;
            }
            break;

        case JointLimitMethod::SOFT_CLAMP_REPULSIVE:
            {
                // 부드러운 클램핑 + 반발력 조합 (권장)
                double scale = computeJointLimitScale(q, limits, activation_ratio);
                dq *= scale;

                transform::vec<6> repulsive = computeRepulsiveForce(
                    q, limits, activation_ratio, repulsive_gain);
                dq += repulsive;
            }
            break;
    }
}