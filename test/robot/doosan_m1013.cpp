#include <chrono>

#include "common_types.h"
#include "imgui.h"
#include "implot3d.h"
#include "robot/transform.h"
#include "robot/jacobian_inverse.h"
#include "robot/imgui_draw_manipulability.hpp"

#include <iostream>
#include <cmath>
#include <iomanip>
#include <optional>
#include <vector>
#include <string>
#include <stdexcept>
#include <bits/this_thread_sleep.h>
#include <Eigen/Dense>




vec<6> s_val = vec<6>::Zero();


// 시각화용 AxisObject
AxisObject origine(0.1);
AxisObject base(0.05);
AxisObject joint1(0.05);
AxisObject joint2(0.05);
AxisObject joint3(0.05);
AxisObject joint4(0.05);
AxisObject joint5(0.05);
AxisObject joint6(0.05);
AxisObject t_tip(0.06);
AxisObject end_effector(0.06);


// 끝단의 매니풀러빌리티와 로봇 링크 관절을 그리는 함수
void draw_simulation(const mat<6, 6>& j, const Transform& fk);
void draw_simulation(const Transform& fk);


template<int Rows, int Cols>
struct SVDResult {
    mat<Rows, Rows> U;
    mat<Cols, Cols> Vt;
    vec<std::min(Rows, Cols)> S_values;
};


template<int Rows, int Cols>
SVDResult<Rows, Cols> SVD_LAPACK(const mat<Rows, Cols>& A)
{
    // 결과 구조체 초기화 (U, Vt, S_values 공간 할당)
    SVDResult<Rows, Cols> result;


    int m = Rows;
    int n = Cols;
    mat<Rows, Cols> Acopy = A; // DGESVD는 A를 덮어쓰므로 복사본 사용

    // LAPACK DGESVD를 위한 설정
    char jobu = 'A'; // U 행렬 계산
    char jobvt = 'A'; // Vt 행렬 계산
    int lda = Rows;
    int ldu = Rows;
    int ldvt = Cols;
    int info = 0;

    // 1단계: Optimal LWORK (작업 공간 크기) 쿼리
    double work_size_query = 0.0;
    int lwork = -1;

    dgesvd_(&jobu, &jobvt, &m, &n, Acopy.data(), &lda,
            result.S_values.data(), result.U.data(), &ldu, result.Vt.data(), &ldvt,
            &work_size_query, &lwork, &info);

    // 2단계: 실제 SVD 계산
    lwork = static_cast<int>(work_size_query);
    Acopy = A;
    std::vector<double> work(lwork);

    dgesvd_(&jobu, &jobvt, &m, &n, Acopy.data(), &lda,
            result.S_values.data(), result.U.data(), &ldu, result.Vt.data(), &ldvt,
            work.data(), &lwork, &info);

    if (info != 0) {
        throw std::runtime_error("LAPACK dgesvd_ failed. INFO code: " + std::to_string(info));
    }

    // 계산된 결과를 담은 구조체 반환
    return result;
}



class M1013 {
    // 관절 제한 구조체
    struct JointLimits {
        double min; // [rad]
        double max; // [rad]
    };


public:
    // ----------------------------------
    // 제어 함수
    // ----------------------------------

    vec<6> q_rad;                       // 각 Joint 각도 [rad]
    std::array<Transform, 8> tf;        // 각 조인트의 transform
    std::optional<Transform> tf_ee;     // 엔드 이펙터 transform

    std::array<JointLimits, 6> limits;  // 각도 제한 조건

    mat<6, 6> J;                        // 자코비안 매트릭스
    mat<6, 6> J_inv;
    double lambda;                      // Damping Factor (특이점 방지용)
    double lambda_min;                  // 최소 감쇠 인자 (기본값)
    double lambda_max;                  // 최대 감쇠 인자
    double sigma_threshold;             // 특잇값 임계치 (epsilon)

    M1013()
    {
        limits[0] = {DEG_TO_RAD(-360.0), DEG_TO_RAD(360.0)};
        limits[1] = {DEG_TO_RAD(-360.0), DEG_TO_RAD(360.0)};
        limits[2] = {DEG_TO_RAD(-160.0), DEG_TO_RAD(160.0)};
        limits[3] = {DEG_TO_RAD(-360.0), DEG_TO_RAD(360.0)};
        limits[4] = {DEG_TO_RAD(-360.0), DEG_TO_RAD(360.0)};
        limits[5] = {DEG_TO_RAD(-360.0), DEG_TO_RAD(360.0)};


        lambda = 0.01;                  // IK 반복에서 동적으로 변경될 현재 람다 값
        lambda_min = 0.01;              // (기본값)
        lambda_max = 0.5;               // 특이점에서의 최대 감쇠 (큰 값으로 설정 가능)
        sigma_threshold = 0.05;         // 이 값보다 σ_min이 작아지면 람다 증가 시작

        J = mat<6, 6>::Zero();

        fk();
    }

    Transform get_fk(int i)
    {
        return tf[i];
    }


    void set_end_effector_tf(const Transform& tf)
    {
        tf_ee = tf;
    }


    // MoveJ
    void movej(double q0, double q1, double q2, double q3, double q4, double q5)
    {
        q_rad[0] = q0;
        q_rad[1] = q1;
        q_rad[2] = q2;
        q_rad[3] = q3;
        q_rad[4] = q4;
        q_rad[5] = q5;

        fk();
    }


    // MoveL
    void movel(double x, double y, double z, double roll, double pitch, double yaw)
    {
        const quat t_rot(
            AngleAxis(yaw, vec3::UnitZ()) *              // Z
            AngleAxis(pitch + M_PI_2, vec3::UnitY()) *   // Y
            AngleAxis(roll, vec3::UnitZ()));             // Z

        const vec3 t_pos(x, y, z);

        const Transform target_tf(t_rot, t_pos);

        const vec<6> q = ik(target_tf);
        movej(q[0], q[1], q[2], q[3], q[4], q[5]);
    }


private:
    // ----------------------------------
    // 기구학 연산 함수들
    // ----------------------------------

    // Forward Kinematics
    Transform fk()
    {
        return fk(q_rad[0], q_rad[1], q_rad[2], q_rad[3], q_rad[4], q_rad[5]);
    }



    Transform fk(double q0, double q1, double q2, double q3, double q4, double q5)
    {
        // 각 조인트의 transform 계산
        const Transform tf0 = Transform(AngleAxis(0,  vec3::UnitZ()), vec3(0, 0, 0));     // Base
        const Transform tf1 = Transform(AngleAxis(q0, vec3::UnitZ()), vec3(0, 0, l1));    // Joint 1
        const Transform tf2 = Transform(AngleAxis(q1, vec3::UnitX()), vec3(-l2, 0, 0));   // Joint 2
        const Transform tf3 = Transform(AngleAxis(q2, vec3::UnitX()), vec3(0, 0, l3));    // Joint 3
        const Transform tf4 = Transform(AngleAxis(q3, vec3::UnitZ()), vec3(l4, 0, 0));    // Joint 4
        const Transform tf5 = Transform(AngleAxis(q4, vec3::UnitX()), vec3(-l6, 0, l5));  // Joint 5
        const Transform tf6 = Transform(AngleAxis(q5, vec3::UnitZ()), vec3(l7, 0, 0));    // Joint 6
        const Transform tf7 = Transform(AngleAxis(0,  vec3::UnitZ()), vec3(0, 0, l8));    // Tool tip


        // Forward Kinematics
        tf[0] = tf0;
        tf[1] = tf[0] * tf1;
        tf[2] = tf[1] * tf2;
        tf[3] = tf[2] * tf3;
        tf[4] = tf[3] * tf4;
        tf[5] = tf[4] * tf5;
        tf[6] = tf[5] * tf6;
        tf[7] = tf[6] * tf7;

        if (tf_ee)
        {

            tf[7] = tf[7] * tf_ee.value();

        }

        return tf[7]; // End Effector's FK
    }


    vec<6> solve_qp_box_constrained(const mat<6, 6>& H, const vec<6>& g,
                                    const vec<6>& lb, const vec<6>& ub)
    {
        vec<6> x = vec<6>::Zero(); // 초기값 0
        int max_iter = 100;
        double tol = 1e-6;

        for (int iter = 0; iter < max_iter; ++iter) {
            double max_diff = 0.0;

            // 각 변수(joint)에 대해 하나씩 업데이트 (Gauss-Seidel)
            for (int i = 0; i < 6; ++i) {
                double old_xi = x(i);

                // H*x + g = 0  =>  sum(H_ij * x_j) + g_i = 0
                // H_ii * x_i + sum_{j!=i}(H_ij * x_j) + g_i = 0
                // x_i = ( -g_i - sum_{j!=i}(H_ij * x_j) ) / H_ii

                double sigma = 0.0;
                for (int j = 0; j < 6; ++j) {
                    if (i != j) sigma += H(i, j) * x(j);
                }

                double x_unc = (-g(i) - sigma) / H(i, i);

                // Projection (Clamping)
                double x_new = std::max(lb(i), std::min(ub(i), x_unc));

                x(i) = x_new;
                max_diff = std::max(max_diff, std::abs(x_new - old_xi));
            }

            // 수렴 확인
            if (max_diff < tol) break;
        }
        return x;
    }


    vec<6> ik(const Transform& target_tf)
    {
        double CONVERGENCE_TOLERANCE = 0.001;
        int MAX_IK_ITER = 200;
        double STEP_SIZE = 0.0001;

        auto q_curr = q_rad;

        for (int iter = 0; iter < MAX_IK_ITER; ++iter)
        {
            // 1. 오차 계산
            Transform current_tf = fk(q_curr[0], q_curr[1], q_curr[2], q_curr[3], q_curr[4], q_curr[5]);
            vec<6> error_twist = target_tf - current_tf; // Log map error
            vec<6> dx;
            dx << error_twist.head<3>(), error_twist.tail<3>();

            if (dx.norm() < CONVERGENCE_TOLERANCE) break;

            // 2. Jacobian 및 Hessian 구성
            J = jacobian(q_curr[0], q_curr[1], q_curr[2], q_curr[3], q_curr[4], q_curr[5]);

            mat<6, 6> H = J.transpose() * J;
            // 댐핑 추가 (특이점 근처 안정성 확보 및 H_ii가 0이 되는 것 방지)
            for (int i = 0; i < 6; i++) H(i, i) += lambda;

            vec<6> g = -J.transpose() * dx;

            // 3. 제약조건 (Local Bounds for dq)
            vec<6> lb, ub;
            double step_limit = 5.0; // 한번에 너무 튀지 않게 안전장치
            double safety = 0.00;    // 관절 한계 살짝 안쪽까지만 허용

            for(int i=0; i<6; ++i) {
                // 현재 각도에서 얼마나 더 움직일 수 있는가?
                double d_min = (limits[i].min + safety) - q_curr[i];
                double d_max = (limits[i].max - safety) - q_curr[i];

                // 너무 큰 스텝 방지 (Local approximation 유효성 유지)
                lb(i) = d_min; // std::max(d_min, -step_limit);
                ub(i) = d_max; // std::min(d_max, step_limit);
            }

            // 4. 직접 만든 QP Solver 호출
            vec<6> dq = solve_qp_box_constrained(H, g, lb, ub);

            // 5. 업데이트
            q_curr += dq * STEP_SIZE;
        }

        for (int i=0; i<6; ++i)
        {
            q_curr[i] = NORM_RAD_180(q_curr[i]);
        }

        return q_curr;
    }


    // 자코비안 행렬
    mat<6, 6> jacobian(double q0, double q1, double q2, double q3, double q4, double q5)
    {
        mat<6, 6> J_ = mat<6, 6>::Zero();

        const double c0 = std::cos(q0);
        const double c1 = std::cos(q1);
        const double c3 = std::cos(q3);
        const double c4 = std::cos(q4);
        const double s0 = std::sin(q0);
        const double s1 = std::sin(q1);
        const double s3 = std::sin(q3);
        const double s4 = std::sin(q4);
        const double c12 = std::cos(q1 + q2);
        const double s12 = std::sin(q1 + q2);


        J_(0,0) = l2*s0 + l3*s1*c0 - l4*s0 + l5*s12*c0 + l6*(s0*c3 + s3*c0*c12) - l7*(s0*c3 + s3*c0*c12) - l8*((s0*s3 - c0*c3*c12)*s4 - s12*c0*c4);
        J_(0,1) = (l3*c1 + l5*c12 - l6*s3*s12 + l7*s3*s12 - l8*(s4*s12*c3 - c4*c12))*s0;
        J_(0,2) = (l5*c12 - l6*s3*s12 + l7*s3*s12 - l8*(s4*s12*c3 - c4*c12))*s0;
        J_(0,3) = l6*(s0*c3*c12 + s3*c0) - l7*(s0*c3*c12 + s3*c0) + l8*(-s0*s3*c12 + c0*c3)*s4;
        J_(0,4) = l8*((s0*c3*c12 + s3*c0)*c4 - s0*s4*s12);
        J_(1,0) = -l2*c0 + l3*s0*s1 + l4*c0 + l5*s0*s12 - l6*(-s0*s3*c12 + c0*c3) + l7*(-s0*s3*c12 + c0*c3) + l8*((s0*c3*c12 + s3*c0)*s4 + s0*s12*c4);
        J_(1,1) = (-l3*c1 - l5*c12 + l6*s3*s12 - l7*s3*s12 + l8*(s4*s12*c3 - c4*c12))*c0;
        J_(1,2) = (-l5*c12 + l6*s3*s12 - l7*s3*s12 + l8*(s4*s12*c3 - c4*c12))*c0;
        J_(1,3) = l6*(s0*s3 - c0*c3*c12) - l7*(s0*s3 - c0*c3*c12) + l8*(s0*c3 + s3*c0*c12)*s4;
        J_(1,4) = l8*((s0*s3 - c0*c3*c12)*c4 + s4*s12*c0);
        J_(2,1) = -l3*s1 - l5*s12 - l6*s3*c12 + l7*s3*c12 - l8*(s4*c3*c12 + s12*c4);
        J_(2,2) = -l5*s12 - l6*s3*c12 + l7*s3*c12 - l8*(s4*c3*c12 + s12*c4);
        J_(2,3) = (-l6*c3 + l7*c3 + l8*s3*s4)*s12;
        J_(2,4) = -l8*(s4*c12 + s12*c3*c4);
        J_(3,1) = c0;
        J_(3,2) = c0;
        J_(3,3) = s0*s12;
        J_(3,4) = -s0*s3*c12 + c0*c3;
        J_(3,5) = s0*s4*c3*c12 + s0*s12*c4 + s3*s4*c0;
        J_(4,1) = s0;
        J_(4,2) = s0;
        J_(4,3) = -s12*c0;
        J_(4,4) = s0*c3 + s3*c0*c12;
        J_(4,5) = s0*s3*s4 - s4*c0*c3*c12 - s12*c0*c4;
        J_(5,0) = 1;
        J_(5,3) = c12;
        J_(5,4) = s3*s12;
        J_(5,5) = -s4*s12*c3 + c4*c12;

        return J_;
    }


    // M1013 관절 길이
    const double l1 = 0.1525; // [m]
    const double l2 = 0.1985; // [m]
    const double l3 = 0.620; // [m]
    const double l4 = 0.164; // [m]
    const double l5 = 0.559; // [m]
    const double l6 = 0.146; // [m]
    const double l7 = 0.146; // [m]
    const double l8 = 0.121; // [m]
};




int main() {
    std::cout << "두산 로봇 M1013 IK - Jacobian 단계 제약조건 통합" << std::endl;


    M1013 m1013;

    // 기본 자세 설정
    float q_deg[6] = {90.0f, 30.0f, 80.0f, 0.0f, 75.0f, 0.0f};
    vec3 tPos(0.62, 0.0, 0.235);
    quat qRot(AngleAxis(DEG_TO_RAD(0), vec3(0, 1, 0)));
    Transform target_tf(qRot, tPos);

    float target_x      = target_tf.x();
    float target_y      = target_tf.y();
    float target_z      = target_tf.z();
    float target_roll   = RAD_TO_DEG(target_tf.roll());
    float target_pitch  = RAD_TO_DEG(target_tf.pitch());
    float target_yaw    = RAD_TO_DEG(target_tf.yaw());






    ImGui::start("데모");

    while (ImGui::isRunning())
    {


        ImGui::draw([&]()
        {
            ImGui::Begin("제어");
            ImGui::PushItemWidth(ImGui::GetContentRegionAvail().x);

            bool is_joint_change = false;
            float q0 = RAD_TO_DEG(m1013.q_rad[0]);
            float q1 = RAD_TO_DEG(m1013.q_rad[1]);
            float q2 = RAD_TO_DEG(m1013.q_rad[2]);
            float q3 = RAD_TO_DEG(m1013.q_rad[3]);
            float q4 = RAD_TO_DEG(m1013.q_rad[4]);
            float q5 = RAD_TO_DEG(m1013.q_rad[5]);

            ImGui::Text("Joint 설정");
            is_joint_change |= ImGui::DragFloat("##j1", &q0, 0.1f, -360.f, 360.f, "J1: %.3f");
            is_joint_change |= ImGui::DragFloat("##j2", &q1, 0.1f, -360.f, 360.f, "J2: %.3f");
            is_joint_change |= ImGui::DragFloat("##j3", &q2, 0.1f, -360.f, 360.f, "J3: %.3f");
            is_joint_change |= ImGui::DragFloat("##j4", &q3, 0.1f, -360.f, 360.f, "J4: %.3f");
            is_joint_change |= ImGui::DragFloat("##j5", &q4, 0.1f, -360.f, 360.f, "J5: %.3f");
            is_joint_change |= ImGui::DragFloat("##j6", &q5, 0.1f, -360.f, 360.f, "J6: %.3f");
            ImGui::Dummy(ImVec2(0, 20));

            ImGui::Text("Target Transform 설정");
            ImGui::DragFloat("##Target x", &target_x, 0.01f, -2.3f, 2.3f, "x: %.3f");
            ImGui::DragFloat("##Target y", &target_y, 0.01f, -2.3f, 2.3f, "y: %.3f");
            ImGui::DragFloat("##Target z", &target_z, 0.001f, -2.3f, 2.3f, "z: %.3f");
            ImGui::Dummy(ImVec2(0, 10));
            ImGui::DragFloat("##Target Roll", &target_roll, 1.0f, -180.0f, 180.0f, "Roll: %.3f");
            ImGui::DragFloat("##Target Pitch", &target_pitch, 1.0f, -130.0f, 130.0f, "Pitch: %.3f");
            ImGui::DragFloat("##Target Yaw", &target_yaw, 1.0f, -180.0f, 180.0f, "Yaw: %.3f");
            ImGui::PopItemWidth();

            if (is_joint_change)
            {
                m1013.movej(DEG_TO_RAD(q0), DEG_TO_RAD(q1), DEG_TO_RAD(q2), DEG_TO_RAD(q3), DEG_TO_RAD(q4), DEG_TO_RAD(q5));
            }







            // Ui
            {
                Transform tf = m1013.get_fk(7);
                ImGui::Text("FK: x=%.3f, y=%.3f, z=%.3f", tf.trans().x(), tf.trans().y(), tf.trans().z());
                ImGui::Text("FK: roll=%.3f, pitch=%.3f, yaw=%.3f", RAD_TO_DEG(tf.roll()), RAD_TO_DEG(tf.pitch()), RAD_TO_DEG(tf.yaw()));

                ImGui::Dummy(ImVec2(0, 10));

                draw_simulation(m1013.J, tf);

                ImGui::Dummy(ImVec2(0, 10));

                // --------------------------------------------------------------------------------
                // ✨ 매니풀러빌리티 분석 결과 출력 (추가된 부분)
                // --------------------------------------------------------------------------------
                mat<3, 6> J_pos = m1013.J.block<3, 6>(0, 0);
                mat3 M = J_pos * J_pos.transpose();
                Eigen::SelfAdjointEigenSolver<mat3> eigensolver(M);

                if (eigensolver.info() == Eigen::Success)
                {
                    vec3 eigenvalues = eigensolver.eigenvalues();
                    mat3 eigenvectors = eigensolver.eigenvectors();

                    // draw_simulation에서 사용된 scale 0.3f 적용
                    float viz_scale = 0.3f;
                    vec3 radii = eigenvalues.cwiseSqrt() * viz_scale;

                    ImGui::Text("⭐ Position Manipulability Analysis (Scale: %.1f)", viz_scale);
                    ImGui::Separator();

                    // Eigen은 고유값을 오름차순으로 정렬하는 경향이 있으므로, 2, 1, 0 순서로 출력하여
                    // 가장 긴 축(Axis 1)부터 보이도록 합니다.
                    for (int i = 2; i >= 0; --i)
                    {
                        ImGui::Text("Axis %d Length (Radius): %.6f", 3 - i, radii(i));
                        ImGui::Text("  Direction (u%d): X=%.3f, Y=%.3f, Z=%.3f",
                                    3 - i,
                                    eigenvectors(0, i), // X component
                                    eigenvectors(1, i), // Y component
                                    eigenvectors(2, i)); // Z component

                        if (i == 2) ImGui::Text("  (Longest/Easiest Direction)");
                        if (i == 0) ImGui::Text("  (Shortest/Most Difficult Direction)");
                    }
                }
                else
                {
                    ImGui::Text("Manipulability Analysis Failed.");
                }

                ImGui::Dummy(ImVec2(0, 10));
                // --------------------------------------------------------------------------------
                // 기존 s_val 출력
                // --------------------------------------------------------------------------------
                ImGui::Text("s_val (Full Jacobian SVD):");
                ImGui::Text("s_val1: %.6f", s_val[0]);
                ImGui::Text("s_val2: %.6f", s_val[1]);
                ImGui::Text("s_val3: %.6f", s_val[2]);
                ImGui::Text("s_val4: %.6f", s_val[3]);
                ImGui::Text("s_val5: %.6f", s_val[4]);
                ImGui::Text("s_val6: %.6f", s_val[5]);

            }
            ImGui::End(); // "제어"
        });

        // Axis Transform 설정
        joint1.setTransform(m1013.get_fk(1));
        joint2.setTransform(m1013.get_fk(2));
        joint3.setTransform(m1013.get_fk(3));
        joint4.setTransform(m1013.get_fk(4));
        joint5.setTransform(m1013.get_fk(5));
        joint6.setTransform(m1013.get_fk(6));
        t_tip.setTransform(m1013.get_fk(7));

        auto now = std::chrono::high_resolution_clock::now();
        m1013.movel(target_x, target_y, target_z, DEG_TO_RAD(target_roll), DEG_TO_RAD(target_pitch), DEG_TO_RAD(target_yaw));
        auto dt = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - now).count();
        std::cout << "dt: " << dt << std::endl;
        // now = std::chrono::high_resolution_clock::now();
    }
    ImGui::stop();

    return 0;
}


// -----------------------------------------------
// 시각화 함수 (draw_simulation)
// -----------------------------------------------


void draw_simulation(const mat<6, 6>& j, const Transform& fk)
{
    ImGui::Begin("시각화");
    if (ImPlot3D::BeginPlot("로봇 FK/IK", ImVec2(-1,-1), ImPlot3DFlags_Equal | ImPlot3DFlags_NoLegend))
    {
        ImPlot3D::SetupAxesLimits(-1.3, 1.3, -1.3, 1.3, 0.0, 1.3, ImPlot3DCond_Always);
        ImPlot3D::SetupAxes("X", "Y", "Z");

        origine.Draw();
        base.Draw();
        joint1.Draw();
        joint2.Draw();
        joint3.Draw();
        joint4.Draw();
        joint5.Draw();
        joint6.Draw();
        t_tip.Draw();
        end_effector.Draw();

        DrawLinkLine(base, joint1);
        DrawLinkLine(joint1, joint2);
        DrawLinkLine(joint2, joint3);
        DrawLinkLine(joint3, joint4);
        DrawLinkLine(joint4, joint5);
        DrawLinkLine(joint5, joint6);
        DrawLinkLine(joint6, t_tip);

        // J_position (3x6)을 사용하여 매니풀러빌리티 타원체를 그림
        mat<3, 6> j_pos = j.block<3, 6>(0, 0);
        imgui_draw_manipulability(j_pos, fk.trans(), 0.3f, ImVec4(0,1,1,0.1f));

        ImPlot3D::EndPlot();
    }
    ImGui::End();
}


void draw_simulation(const Transform& fk)
{
    ImGui::Begin("시각화");
    if (ImPlot3D::BeginPlot("로봇 FK/IK", ImVec2(-1,-1), ImPlot3DFlags_Equal | ImPlot3DFlags_NoLegend))
    {
        ImPlot3D::SetupAxesLimits(-1.3, 1.3, -1.3, 1.3, 0.0, 1.3, ImPlot3DCond_Always);
        ImPlot3D::SetupAxes("X", "Y", "Z");

        origine.Draw();
        base.Draw();
        joint1.Draw();
        joint2.Draw();
        joint3.Draw();
        joint4.Draw();
        joint5.Draw();
        joint6.Draw();
        t_tip.Draw();

        DrawLinkLine(base, joint1);
        DrawLinkLine(joint1, joint2);
        DrawLinkLine(joint2, joint3);
        DrawLinkLine(joint3, joint4);
        DrawLinkLine(joint4, joint5);
        DrawLinkLine(joint5, joint6);
        DrawLinkLine(joint6, t_tip);


        ImPlot3D::EndPlot();
    }
    ImGui::End();
}