#pragma once
#include "transform.h"

extern "C"{
    extern void dgesvd_(char* jobu, char* jobvt, int* m, int* n, double* a,
                        int* lda, double* s, double* u, int* ldu, double* vt,
                        int* ldvt, double* work, int* lwork, int* info);
}







// -------------------------------------
// Jacobian 인버스
// J^+ = (J^T * J)^-1 * J^T
// -------------------------------------
template<int Rows, int Cols>
inline mat<Cols, Rows> pInv(const mat<Rows, Cols>& J)
{
    return (J.transpose() * J).inverse() * J.transpose();
}




// -------------------------------------
// Damped Least Squares (DLS)
// J^+ = (J^T * J + λ^2 * I)^-1 * J^T
// -------------------------------------
template<int Rows, int Cols>
inline mat<Cols, Rows> pInv_DLS(const mat<Rows, Cols>& J, double lambda = 0.01)
{
    mat<Rows, Rows> I = mat<Rows, Rows>::Identity();
    return J.transpose() * (J * J.transpose() + lambda * lambda * I).inverse();
}




// -------------------------------------
// Dynamic Damped Least Squares (DDLS)
// -------------------------------------
inline mat<6, 12> pInv_Dynamic_DLS(const mat<12, 6>& J)
{
    mat<3, 6> J_pos = J.block<3, 6>(0, 0);
    mat<3, 3> JJ_T_pos = J_pos * J_pos.transpose();
    Eigen::SelfAdjointEigenSolver<mat<3, 3>> eigensolver(JJ_T_pos);
    double min_eigenvalue = eigensolver.eigenvalues()(0);

    double w_min = std::sqrt(std::max(0.0, min_eigenvalue));

    double lambda_max = 0.9;
    double w_min_threshold = 0.15;
    double lambda = 0.01;

    if (w_min < w_min_threshold) {
        lambda = lambda_max * (1.0 - (w_min / w_min_threshold));
    }

    mat<12, 12> I = mat<12, 12>::Identity();
    return J.transpose() * (J * J.transpose() + lambda * lambda * I).inverse();
}





// -------------------------------------
// Singular Value Decomposition (Eigen SVD)
// -------------------------------------
template<int Rows, int Cols>
inline mat<Cols, Rows> pInv_svd(const mat<Rows, Cols>& J)
{
    Eigen::JacobiSVD<mat<Rows, Cols>> svd(J, Eigen::ComputeFullU | Eigen::ComputeFullV);

    const double tolerance = 1e-10;


    // 특이값 벡터 가져오기
    Eigen::VectorXd singularValues = svd.singularValues();
    Eigen::VectorXd singularValuesInv(singularValues.size());

    for (int i = 0; i < singularValues.size(); ++i) {
        if (singularValues(i) > tolerance) {
            singularValuesInv(i) = 1.0 / singularValues(i);
        } else {
            singularValuesInv(i) = 0.0;
        }
    }


    // 유사 역행렬의 정의에 맞게 Σ⁺ 행렬을 올바른 크기(Cols x Rows)로 구성
    mat<Cols, Rows> sigma_inv = mat<Cols, Rows>::Zero();
    sigma_inv.block(0, 0, singularValues.size(), singularValues.size()) = singularValuesInv.asDiagonal();


    // 4. J⁺ = V * Σ⁺ * Uᵀ 계산
    mat<Cols, Rows> pinv = svd.matrixV() * sigma_inv * svd.matrixU().transpose();

    return pinv;
}





// -------------------------------------
// LAPACK SVD
// -------------------------------------
template<int Rows, int Cols>
mat<Cols, Rows> pInv_LAPACK(const mat<Rows, Cols>& A, double epsilon = std::numeric_limits<double>::epsilon())
{
    int m = Rows;
    int n = Cols;
    mat<Rows, Cols> Acopy = A;

    // 컴파일 타임에 최소 차원 계산
    constexpr int min_mn = (Rows < Cols) ? Rows : Cols;

    // U, S, Vt 행렬을 mat의 타입 별칭을 사용하여 선언
    mat<Rows, Rows> U;
    mat<Cols, Cols> Vt;
    vec<min_mn> S_values;


    char jobu = 'A';  // 모든 M x M U 행렬을 계산
    char jobvt = 'A'; // 모든 N x N V^T 행렬을 계산
    int lda = Rows;
    int ldu = Rows;
    int ldvt = Cols;
    int info = 0;


    // 워크스페이스 쿼리 및 할당
    double work_size_query = 0.0;
    int lwork = -1; // lwork = -1로 설정하여 워크스페이스 크기 쿼리
    dgesvd_(&jobu, &jobvt, &m, &n, Acopy.data(), &lda, S_values.data(), U.data(), &ldu, Vt.data(), &ldvt, &work_size_query, &lwork, &info);

    lwork = static_cast<int>(work_size_query);
    std::vector<double> work(lwork);


    // 실제 SVD 계산 호출
    dgesvd_(&jobu, &jobvt, &m, &n, Acopy.data(), &lda, S_values.data(), U.data(), &ldu, Vt.data(), &ldvt, work.data(), &lwork, &info);

    if (info != 0) {
        throw std::runtime_error("LAPACK dgesvd_ failed.");
    }


    // 특이값에 대한 허용 오차 계산
    double tolerance = epsilon * std::max(m, n) * S_values(0);

    vec<min_mn> S_inv_values = S_values;
    int rank = 0;
    for (int i = 0; i < min_mn; ++i) {
        if (S_values(i) > tolerance) {
            S_inv_values(i) = 1.0 / S_values(i);
            rank++;
        } else {
            S_inv_values(i) = 0.0;
        }
    }


    // 유사역행렬 계산: A+ = V * S_inv * U^T
    mat<Cols, Rows> A_plus = mat<Cols, Rows>::Zero();
    if (rank > 0) {
        // Eigen::DiagonalMatrix 대신 asDiagonal() 사용
        A_plus = Vt.transpose().leftCols(rank) *
                 S_inv_values.head(rank).asDiagonal() *
                 U.transpose().topRows(rank);
    }

    return A_plus;
}





// -------------------------------------
// LAPACK DLS
// -------------------------------------
template<int Rows, int Cols>
mat<Cols, Rows> pInv_DLS_LAPACK(const mat<Rows, Cols>& A, double lambda = 0.05) // lambda 기본값 설정
{
    int m = Rows;
    int n = Cols;
    mat<Rows, Cols> Acopy = A;

    constexpr int min_mn = (Rows < Cols) ? Rows : Cols;

    mat<Rows, Rows> U;
    mat<Cols, Cols> Vt;
    vec<min_mn> S_values;


    // LAPACK 변수 설정
    char jobu = 'A';
    char jobvt = 'A';
    int lda = Rows;
    int ldu = Rows;
    int ldvt = Cols;
    int info = 0;



    // 워크스페이스 쿼리
    double work_size_query = 0.0;
    int lwork = -1;
    dgesvd_(&jobu, &jobvt, &m, &n, Acopy.data(), &lda, S_values.data(), U.data(), &ldu, Vt.data(), &ldvt, &work_size_query, &lwork, &info);

    lwork = static_cast<int>(work_size_query);
    std::vector<double> work(lwork);



    // 실제 SVD 계산 (A = U * S * Vt)
    dgesvd_(&jobu, &jobvt, &m, &n, Acopy.data(), &lda, S_values.data(), U.data(), &ldu, Vt.data(), &ldvt, work.data(), &lwork, &info);

    if (info != 0) {
        throw std::runtime_error("LAPACK dgesvd_ failed.");
    }



    // sigma_inv = sigma / (sigma^2 + lambda^2)
    vec<min_mn> S_inv_values;
    double lambda_sq = lambda * lambda;

    for (int i = 0; i < min_mn; ++i) {
        double sigma = S_values(i);

        // DLS 공식 적용
        // lambda가 0일 경우 일반적인 Pseudo-Inverse와 같아지지만,
        // sigma가 0일 때의 0 division 방지를 위해 분모 체크가 필요할 수 있음.
        double denominator = (sigma * sigma) + lambda_sq;

        if (denominator > std::numeric_limits<double>::epsilon()) {
             S_inv_values(i) = sigma / denominator;
        } else {
             S_inv_values(i) = 0.0;
        }
    }


    // DLS 역행렬 재구성: A_dls = V * S_inv_dls * U^T
    mat<Cols, Rows> A_dls = mat<Cols, Rows>::Zero();

    // Thin SVD 형태로 재구성
    A_dls = Vt.transpose().leftCols(min_mn) * S_inv_values.asDiagonal() * U.transpose().topRows(min_mn);

    return A_dls;
}





// -------------------------------------
// Jacobian Inverse with Damped Least Squares (DLS)
// -------------------------------------
inline mat<6,12> jInv_SVD_Damped(const mat<12, 6>& J, double lambda = 0.01)
{
    double lambda_max       = 0.8;  // 최대값
    double w_min_threshold  = 0.10; // 임계값


    // 람다 계산
    mat<3, 6> J_pos = J.block<3, 6>(0, 0);
    mat<3, 3> JJ_T_pos = J_pos * J_pos.transpose();
    Eigen::SelfAdjointEigenSolver<mat<3, 3>> eigensolver(JJ_T_pos);
    double min_eigenvalue = eigensolver.eigenvalues()(0);
    double w_min = std::sqrt(std::max(0.0, min_eigenvalue));


    if (w_min < w_min_threshold) {
        lambda = lambda_max * (1.0 - (w_min / w_min_threshold));
    }

    std::cout << "w_min: " << w_min << ", lambda: " << lambda << "\n" << std::endl;


    //  SVD 계산
    Eigen::JacobiSVD<mat<12, 6>> svd(
        J,
        Eigen::ComputeFullU | Eigen::ComputeFullV
    );


    // 특이값(Singular Values)
    const auto& s = svd.singularValues();

    mat<6, 12> S_plus = mat<6, 12>::Zero();

    for (int i = 0; i < s.size(); ++i) {
        double sigma = s(i);

        // DLS: sigma / (sigma^2 + lambda^2)
        double damped_inv_sigma = sigma / (sigma * sigma + lambda * lambda);
        S_plus(i, i) = damped_inv_sigma;
    }



    // 댐핑된 유사 역행렬 J+ = V * S+ * U^T 계산
    return svd.matrixV() * S_plus * svd.matrixU().transpose();
}