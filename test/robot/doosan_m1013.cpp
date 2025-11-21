#include "common_types.h"
#include "imgui.h"
#include "implot3d.h"
#include "robot/transform.h"
#include "robot/jacobian_inverse.h"
#include "robot/imgui_draw_manipulability.hpp"


#include <iostream>
#include <cmath>
#include <vector>
#include <string>
#include <stdexcept>


// 두산 M1013 링크 길이
const double l1 = 0.135;
const double l2 = 0.1702;
const double l3 = 0.411;
const double l4 = 0.164;
const double l5 = 0.368;
const double l6 = 0.1522;
const double l7 = 0.146;
const double l8 = 0.121;


// 시각화용 AxisObject
AxisObject origine(0.1);
AxisObject base(0.05);
AxisObject joint1(0.05);
AxisObject joint2(0.05);
AxisObject joint3(0.05);
AxisObject joint4(0.05);
AxisObject joint5(0.05);
AxisObject joint6(0.05);
AxisObject endEffector(0.06);


// 끝단의 매니풀러빌리티와 로봇 링크 관절을 그리는 함수
void draw_simulation(const mat<12, 6>& j, const transform& fk);
// 두산 M1013 Jacobian 함수
mat<12, 6> Jcobian(double q0, double q1, double q2, double q3, double q4, double q5);



int main() {
    std::cout << "두산 로봇 M1013 IK - Jacobian 단계 제약조건 통합" << std::endl;




    // 기본 자세 설정
    float q_deg[6] = {90.0f, 30.0f, 80.0f, 0.0f, 75.0f, 0.0f};
    vec3 tPos(0.62, 0.0, 0.235);
    quat qRot(transform::AngleAxis(DEG_TO_RAD(90), vec3(0, 1, 0)));
    transform target_tf(qRot, tPos);

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

            ImGui::Text("Joint 설정");
            ImGui::DragFloat("##j1", &q_deg[0], 0.1f, -360.f, 360.f, "J1: %.3f");
            ImGui::DragFloat("##j2", &q_deg[1], 0.1f, -360.f, 360.f, "J2: %.3f");
            ImGui::DragFloat("##j3", &q_deg[2], 0.1f, -360.f, 360.f, "J3: %.3f");
            ImGui::DragFloat("##j4", &q_deg[3], 0.1f, -360.f, 360.f, "J4: %.3f");
            ImGui::DragFloat("##j5", &q_deg[4], 0.1f, -360.f, 360.f, "J5: %.3f");
            ImGui::DragFloat("##j6", &q_deg[5], 0.1f, -360.f, 360.f, "J6: %.3f");
            ImGui::Dummy(ImVec2(0, 20));

            ImGui::Text("Target Transform 설정");
            ImGui::DragFloat("##Target x", &target_x, 0.01f, -1.0f, 1.0f, "x: %.3f");
            ImGui::DragFloat("##Target y", &target_y, 0.01f, -1.0f, 1.0f, "y: %.3f");
            ImGui::DragFloat("##Target z", &target_z, 0.001f, -1.0f, 2.0f, "z: %.3f");
            ImGui::Dummy(ImVec2(0, 10));
            ImGui::DragFloat("##Target Roll", &target_roll, 1.0f, -180.0f, 180.0f, "Roll: %.3f");
            ImGui::DragFloat("##Target Pitch", &target_pitch, 1.0f, -130.0f, 130.0f, "Pitch: %.3f");
            ImGui::DragFloat("##Target Yaw", &target_yaw, 1.0f, -180.0f, 180.0f, "Yaw: %.3f");
            ImGui::PopItemWidth();



            // Target Transform
            quat target_qRot(transform::AngleAxis(DEG_TO_RAD(target_yaw), vec3::UnitZ()) *
                                        transform::AngleAxis(DEG_TO_RAD(target_pitch + 90.0), vec3::UnitY()) *
                                        transform::AngleAxis(DEG_TO_RAD(target_roll), vec3::UnitZ()));
            target_tf = transform(target_qRot, vec3(target_x, target_y, target_z));



            // FK
            transform tf1(quat(transform::AngleAxis(DEG_TO_RAD(q_deg[0]), vec3::UnitZ())), vec3(0, 0, l1));
            transform tf2(quat(transform::AngleAxis(DEG_TO_RAD(q_deg[1]), vec3::UnitX())), vec3(-l2, 0, 0));
            transform tf3(quat(transform::AngleAxis(DEG_TO_RAD(q_deg[2]), vec3::UnitX())), vec3(0, 0, l3));
            transform tf4(quat(transform::AngleAxis(DEG_TO_RAD(q_deg[3]), vec3::UnitZ())), vec3(l4, 0, 0));
            transform tf5(quat(transform::AngleAxis(DEG_TO_RAD(q_deg[4]), vec3::UnitX())), vec3(-l6, 0, l5));
            transform tf6(quat(transform::AngleAxis(DEG_TO_RAD(q_deg[5]), vec3::UnitZ())), vec3(l7, 0, 0));
            transform tf_ee(quat(transform::AngleAxis(DEG_TO_RAD(0), vec3::UnitZ())), vec3(0, 0, l8));

            transform final_tf1 = tf1;
            transform final_tf2 = final_tf1 * tf2;
            transform final_tf3 = final_tf2 * tf3;
            transform final_tf4 = final_tf3 * tf4;
            transform final_tf5 = final_tf4 * tf5;
            transform final_tf6 = final_tf5 * tf6;
            transform final_tf_ee = final_tf6 * tf_ee;
            transform fk = final_tf_ee;



            // Axis Transform 설정
            joint1.setTransform(final_tf1);
            joint2.setTransform(final_tf2);
            joint3.setTransform(final_tf3);
            joint4.setTransform(final_tf4);
            joint5.setTransform(final_tf5);
            joint6.setTransform(final_tf6);
            endEffector.setTransform(final_tf_ee);



            // IK


            // 1. 오차(dx) 계산
            transform error = target_tf - fk;
            vec<12> dx;
            dx << error(0,3), error(1,3), error(2,3),
                  error(0,0), error(0,1), error(0,2),
                  error(1,0), error(1,1), error(1,2),
                  error(2,0), error(2,1), error(2,2);

            // IK Gain (너무 크면 발산)
            dx = dx ;

            // 2. Jacobian 계산
            mat<12, 6> J = Jcobian(DEG_TO_RAD(q_deg[0]), DEG_TO_RAD(q_deg[1]), DEG_TO_RAD(q_deg[2]),
                                   DEG_TO_RAD(q_deg[3]), DEG_TO_RAD(q_deg[4]), DEG_TO_RAD(q_deg[5]));


            mat<6, 12> J_inv = pInv(J);

            vec<6> dq = J_inv * dx * 0.1;

            q_deg[0] += RAD_TO_DEG(dq(0));
            q_deg[1] += RAD_TO_DEG(dq(1));
            q_deg[2] += RAD_TO_DEG(dq(2));
            q_deg[3] += RAD_TO_DEG(dq(3));
            q_deg[4] += RAD_TO_DEG(dq(4));
            q_deg[5] += RAD_TO_DEG(dq(5));






            // Ui
            {
                ImGui::Text("FK: x=%.3f, y=%.3f, z=%.3f", fk.translation().x(), fk.translation().y(), fk.translation().z());
                ImGui::Text("FK: roll=%.3f, pitch=%.3f, yaw=%.3f", RAD_TO_DEG(fk.roll()), RAD_TO_DEG(fk.pitch()), RAD_TO_DEG(fk.yaw()));

                ImGui::Dummy(ImVec2(0, 10));

                draw_simulation(J, fk);

            }
            ImGui::End();



        });
    }
    ImGui::stop();

    return 0;
}





// -----------------------------------------------
// 이 아래 코드는 생성하지 않아도됨. 필요시에만 별도로 작성
// ------------------------------------------------


void draw_simulation(const mat<12, 6>& j, const transform& fk)
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
        endEffector.Draw();

        DrawLinkLine(base, joint1);
        DrawLinkLine(joint1, joint2);
        DrawLinkLine(joint2, joint3);
        DrawLinkLine(joint3, joint4);
        DrawLinkLine(joint4, joint5);
        DrawLinkLine(joint5, joint6);
        DrawLinkLine(joint6, endEffector);

        mat<3, 6> j_pos = j.block<3, 6>(0, 0);
        imgui_draw_manipulability(j_pos, fk.translation(), 0.3f, ImVec4(0,1,1,0.1f));

        ImPlot3D::EndPlot();
    }
    ImGui::End();
}


mat<12, 6> Jcobian(double q0, double q1, double q2, double q3, double q4, double q5)
{
    static mat<12, 6> J = mat<12, 6>::Zero();

    double c0 = std::cos(q0);
    double s0 = std::sin(q0);
    double c1 = std::cos(q1);
    double s1 = std::sin(q1);
    double c3 = std::cos(q3);
    double s3 = std::sin(q3);
    double c4 = std::cos(q4);
    double s4 = std::sin(q4);
    double c5 = std::cos(q5);
    double s5 = std::sin(q5);
    double c12 = std::cos(q1 + q2);
    double s12 = std::sin(q1 + q2);

    J(0,0) = l2*s0 + l3*s1*c0 - l4*s0 + l5*s12*c0 + l6*(s0*c3 + s3*c0*c12) - l7*(s0*c3 + s3*c0*c12) - l8*((s0*s3 - c0*c3*c12)*s4 - s12*c0*c4);
    J(0,1) = (l3*c1 + l5*c12 - l6*s3*s12 + l7*s3*s12 - l8*(s4*s12*c3 - c4*c12))*s0;
    J(0,2) = (l5*c12 - l6*s3*s12 + l7*s3*s12 - l8*(s4*s12*c3 - c4*c12))*s0;
    J(0,3) = l6*(s0*c3*c12 + s3*c0) - l7*(s0*c3*c12 + s3*c0) + l8*(-s0*s3*c12 + c0*c3)*s4;
    J(0,4) = l8*((s0*c3*c12 + s3*c0)*c4 - s0*s4*s12);
    J(0,5) = 0;
    J(1,0) = -l2*c0 + l3*s0*s1 + l4*c0 + l5*s0*s12 - l6*(-s0*s3*c12 + c0*c3) + l7*(-s0*s3*c12 + c0*c3) + l8*((s0*c3*c12 + s3*c0)*s4 + s0*s12*c4);
    J(1,1) = (-l3*c1 - l5*c12 + l6*s3*s12 - l7*s3*s12 + l8*(s4*s12*c3 - c4*c12))*c0;
    J(1,2) = (-l5*c12 + l6*s3*s12 - l7*s3*s12 + l8*(s4*s12*c3 - c4*c12))*c0;
    J(1,3) = l6*(s0*s3 - c0*c3*c12) - l7*(s0*s3 - c0*c3*c12) + l8*(s0*c3 + s3*c0*c12)*s4;
    J(1,4) = l8*((s0*s3 - c0*c3*c12)*c4 + s4*s12*c0);
    J(1,5) = 0;
    J(2,0) = 0;
    J(2,1) = -l3*s1 - l5*s12 - l6*s3*c12 + l7*s3*c12 - l8*(s4*c3*c12 + s12*c4);
    J(2,2) = -l5*s12 - l6*s3*c12 + l7*s3*c12 - l8*(s4*c3*c12 + s12*c4);
    J(2,3) = (-l6*c3 + l7*c3 + l8*s3*s4)*s12;
    J(2,4) = -l8*(s4*c12 + s12*c3*c4);
    J(2,5) = 0;
    J(3,0) = ((s0*s3 - c0*c3*c12)*c4 + s4*s12*c0)*s5 - (s0*c3 + s3*c0*c12)*c5;
    J(3,1) = ((s4*c12 + s12*c3*c4)*s5 + s3*s12*c5)*s0;
    J(3,2) = ((s4*c12 + s12*c3*c4)*s5 + s3*s12*c5)*s0;
    J(3,3) = (s0*s3*c12 - c0*c3)*s5*c4 - (s0*c3*c12 + s3*c0)*c5;
    J(3,4) = ((s0*c3*c12 + s3*c0)*s4 + s0*s12*c4)*s5;
    J(3,5) = (-(s0*c3*c12 + s3*c0)*c4 + s0*s4*s12)*c5 - (-s0*s3*c12 + c0*c3)*s5;
    J(4,0) = ((s0*s3 - c0*c3*c12)*c4 + s4*s12*c0)*c5 + (s0*c3 + s3*c0*c12)*s5;
    J(4,1) = ((s4*c12 + s12*c3*c4)*c5 - s3*s5*s12)*s0;
    J(4,2) = ((s4*c12 + s12*c3*c4)*c5 - s3*s5*s12)*s0;
    J(4,3) = (s0*s3*c12 - c0*c3)*c4*c5 + (s0*c3*c12 + s3*c0)*s5;
    J(4,4) = ((s0*c3*c12 + s3*c0)*s4 + s0*s12*c4)*c5;
    J(4,5) = ((s0*c3*c12 + s3*c0)*c4 - s0*s4*s12)*s5 + (s0*s3*c12 - c0*c3)*c5;
    J(5,0) = -(s0*s3 - c0*c3*c12)*s4 + s12*c0*c4;
    J(5,1) = (-s4*s12*c3 + c4*c12)*s0;
    J(5,2) = (-s4*s12*c3 + c4*c12)*s0;
    J(5,3) = (-s0*s3*c12 + c0*c3)*s4;
    J(5,4) = (s0*c3*c12 + s3*c0)*c4 - s0*s4*s12;
    J(5,5) = 0;
    J(6,0) = (-(s0*c3*c12 + s3*c0)*c4 + s0*s4*s12)*s5 + (-s0*s3*c12 + c0*c3)*c5;
    J(6,1) = -((s4*c12 + s12*c3*c4)*s5 + s3*s12*c5)*c0;
    J(6,2) = -((s4*c12 + s12*c3*c4)*s5 + s3*s12*c5)*c0;
    J(6,3) = -(s0*s3 - c0*c3*c12)*c5 - (s0*c3 + s3*c0*c12)*s5*c4;
    J(6,4) = ((s0*s3 - c0*c3*c12)*s4 - s12*c0*c4)*s5;
    J(6,5) = -((s0*s3 - c0*c3*c12)*c4 + s4*s12*c0)*c5 - (s0*c3 + s3*c0*c12)*s5;
    J(7,0) = (-(s0*c3*c12 + s3*c0)*c4 + s0*s4*s12)*c5 - (-s0*s3*c12 + c0*c3)*s5;
    J(7,1) = (-(s4*c12 + s12*c3*c4)*c5 + s3*s5*s12)*c0;
    J(7,2) = (-(s4*c12 + s12*c3*c4)*c5 + s3*s5*s12)*c0;
    J(7,3) = (s0*s3 - c0*c3*c12)*s5 - (s0*c3 + s3*c0*c12)*c4*c5;
    J(7,4) = ((s0*s3 - c0*c3*c12)*s4 - s12*c0*c4)*c5;
    J(7,5) = ((s0*s3 - c0*c3*c12)*c4 + s4*s12*c0)*s5 - (s0*c3 + s3*c0*c12)*c5;
    J(8,0) = (s0*c3*c12 + s3*c0)*s4 + s0*s12*c4;
    J(8,1) = (s4*s12*c3 - c4*c12)*c0;
    J(8,2) = (s4*s12*c3 - c4*c12)*c0;
    J(8,3) = (s0*c3 + s3*c0*c12)*s4;
    J(8,4) = (s0*s3 - c0*c3*c12)*c4 + s4*s12*c0;
    J(8,5) = 0;
    J(9,0) = 0;
    J(9,1) = -(s4*s12 - c3*c4*c12)*s5 + s3*c5*c12;
    J(9,2) = -(s4*s12 - c3*c4*c12)*s5 + s3*c5*c12;
    J(9,3) = (-s3*s5*c4 + c3*c5)*s12;
    J(9,4) = -(s4*s12*c3 - c4*c12)*s5;
    J(9,5) = (s4*c12 + s12*c3*c4)*c5 - s3*s5*s12;
    J(10,0) = 0;
    J(10,1) = -(s4*s12 - c3*c4*c12)*c5 - s3*s5*c12;
    J(10,2) = -(s4*s12 - c3*c4*c12)*c5 - s3*s5*c12;
    J(10,3) = -(s3*c4*c5 + s5*c3)*s12;
    J(10,4) = -(s4*s12*c3 - c4*c12)*c5;
    J(10,5) = -(s4*c12 + s12*c3*c4)*s5 - s3*s12*c5;
    J(11,0) = 0;
    J(11,1) = -s4*c3*c12 - s12*c4;
    J(11,2) = -s4*c3*c12 - s12*c4;
    J(11,3) = s3*s4*s12;
    J(11,4) = -s4*c12 - s12*c3*c4;
    J(11,5) = 0;

    return J;
}


/* 구현 내용. 실제로는 trasform.h에 inline으로 되어잇음. 이건 참고용
 *template<int Rows, int Cols>
transform::mat<Cols, Rows> pInv_DLS_LAPACK(const transform::mat<Rows, Cols>& A, double lambda = 0.05) // lambda 기본값 설정
{
    int m = Rows;
    int n = Cols;
    transform::mat<Rows, Cols> Acopy = A;

    // 1. 차원 결정
    constexpr int min_mn = (Rows < Cols) ? Rows : Cols;

    // 2. SVD 결과를 담을 행렬 선언
    transform::mat<Rows, Rows> U;
    transform::mat<Cols, Cols> Vt;
    transform::vec<min_mn> S_values;

    // LAPACK 변수 설정
    char jobu = 'A';
    char jobvt = 'A';
    int lda = Rows;
    int ldu = Rows;
    int ldvt = Cols;
    int info = 0;

    // 3. 워크스페이스 쿼리
    double work_size_query = 0.0;
    int lwork = -1;
    dgesvd_(&jobu, &jobvt, &m, &n, Acopy.data(), &lda, S_values.data(), U.data(), &ldu, Vt.data(), &ldvt, &work_size_query, &lwork, &info);

    lwork = static_cast<int>(work_size_query);
    std::vector<double> work(lwork);

    // 4. 실제 SVD 계산 (A = U * S * Vt)
    dgesvd_(&jobu, &jobvt, &m, &n, Acopy.data(), &lda, S_values.data(), U.data(), &ldu, Vt.data(), &ldvt, work.data(), &lwork, &info);

    if (info != 0) {
        throw std::runtime_error("LAPACK dgesvd_ failed.");
    }

    // ---------------------------------------------------------
    // 공식: sigma_inv = sigma / (sigma^2 + lambda^2)
    // ---------------------------------------------------------

    transform::vec<min_mn> S_inv_values;
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

    // 6. DLS 역행렬 재구성: A_dls = V * S_inv_dls * U^T
    // DLS는 모든 특이값을 사용하므로 rank truncation 대신 min_mn 전체를 사용합니다.

    transform::mat<Cols, Rows> A_dls = transform::mat<Cols, Rows>::Zero();

    // U는 MxM, Vt는 NxN 이므로, 대각 행렬 S(min_mn)에 맞춰 차원을 잘라내서 곱함
    // (Thin SVD 형태로 재구성)
    A_dls = Vt.transpose().leftCols(min_mn) * S_inv_values.asDiagonal() * U.transpose().topRows(min_mn);

    return A_dls;
}
*/



/*
double compute_jacobian_condition_number(const transform::mat<12, 6>& J)
{
int m = 12;
    int n = 6;
    // J의 데이터를 복사하여 LAPACK 함수에 전달할 수 있도록 준비
    transform::mat<12, 6> Acopy = J;

    constexpr int min_mn = 6;
    // SVD 결과를 저장할 행렬 및 벡터
    transform::mat<12, 12> U;
    transform::mat<6, 6> Vt;
    transform::vec<min_mn> S_values; // 특이값은 min(M, N) 개

    // LAPACK 변수 설정
    char jobu = 'N'; // U 행렬 계산 안 함 (조건수 계산에 필요 없음)
    char jobvt = 'N'; // V^T 행렬 계산 안 함 (조건수 계산에 필요 없음)
    int lda = 12;
    int ldu = 12; // jobu='N'이므로 사용되지 않음
    int ldvt = 6; // jobvt='N'이므로 사용되지 않음
    int info = 0;

    // 1. 워크스페이스(Work Buffer) 크기 쿼리
    double work_size_query = 0.0;
    int lwork = -1;

    // dgesvd_ 호출 (LWORK = -1)
    dgesvd_(&jobu, &jobvt, &m, &n, Acopy.data(), &lda, S_values.data(), U.data(), &ldu, Vt.data(), &ldvt, &work_size_query, &lwork, &info);

    // 2. 워크스페이스 크기 설정 및 메모리 할당
    if (info != 0 && lwork == -1) {
        // 쿼리 단계에서 오류가 발생했으나, 보통 -1로 호출하면 성공해야 함.
        // 여기서는 단순하게 크기를 가져왔다고 가정합니다.
    }

    // 정수형으로 변환 (안전을 위해 최대값으로 제한)
    lwork = static_cast<int>(std::max(work_size_query, (double)(m + n + 64)));
    std::vector<double> work(lwork);

    // 3. 실제 SVD 계산
    // SVD는 A를 덮어쓰므로, Acopy를 사용합니다.
    dgesvd_(&jobu, &jobvt, &m, &n, Acopy.data(), &lda, S_values.data(), U.data(), &ldu, Vt.data(), &ldvt, work.data(), &lwork, &info);

    // SVD 계산 오류 체크
    if (info > 0) {
        // SVD가 수렴하지 않았거나 문제가 발생했을 경우
        return std::numeric_limits<double>::infinity();
    } else if (info < 0) {
        // 입력 매개변수가 유효하지 않은 경우
        // 실제 운영 환경에서는 치명적인 오류 처리 필요
        return -1.0;
    }

    // 4. 조건수 계산

    // 특이값은 S_values에 내림차순으로 저장됩니다.
    // sigma_max = S_values(0)
    // sigma_min = S_values(min_mn - 1)

    double sigma_max = S_values(0);
    double sigma_min = S_values(min_mn - 1); // 6번째 특이값 (index 5)

    // 가장 작은 특이값이 0에 가깝다면 (특이점)
    if (sigma_min < std::numeric_limits<double>::epsilon()) {
        // 최대 특이값도 0이면 모두 0 행렬
        if (sigma_max < std::numeric_limits<double>::epsilon()) {
            return 1.0; // 0 행렬의 경우 조건수를 1로 정의하기도 합니다.
        }
        return std::numeric_limits<double>::infinity();
    }

    return sigma_max / sigma_min;
}
*/

