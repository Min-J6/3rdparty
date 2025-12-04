#include <iostream>
#include "web_server.hpp"
#include "robot/transform.h"
#include <optional>



#include <chrono>

#include "imgui.h"
#include "implot3d.h"
#include "robot/transform.h"
#include "robot/jacobian_inverse.h"
#include "robot/imgui_draw_manipulability.hpp"

#include <iostream>
#include <cmath>
#include <optional>
#include <vector>
#include <string>
#include <stdexcept>
#include <bits/this_thread_sleep.h>


class AxisObject
{
public:
    float axisLength;


    AxisObject(float length = 1.0f) : axisLength(length)
    {
        // 기본값: 원점(0,0,0), 회전 없음
        origin[0] = 0.0f; origin[1] = 0.0f; origin[2] = 0.0f;
        endX[0] = length; endX[1] = 0.0f; endX[2] = 0.0f;
        endY[0] = 0.0f; endY[1] = length; endY[2] = 0.0f;
        endZ[0] = 0.0f; endZ[1] = 0.0f; endZ[2] = length;
    }


    void setTransform(const Transform& tf)
    {
        // Eigen::Matrix4d (double, Column-Major)의 데이터 포인터
        const double* m = tf.matrix().data();

        // 1. 위치 (Translation) - m[12], m[13], m[14]
        origin[0] = static_cast<float>(m[12]);
        origin[1] = static_cast<float>(m[13]);
        origin[2] = static_cast<float>(m[14]);

        // 2. 회전 (Rotation) - 1, 2, 3번째 열
        // 끝점 = 원점 + (방향 * 길이)

        // X축 끝점
        endX[0] = origin[0] + static_cast<float>(m[0]) * axisLength;
        endX[1] = origin[1] + static_cast<float>(m[1]) * axisLength;
        endX[2] = origin[2] + static_cast<float>(m[2]) * axisLength;

        // Y축 끝점
        endY[0] = origin[0] + static_cast<float>(m[4]) * axisLength;
        endY[1] = origin[1] + static_cast<float>(m[5]) * axisLength;
        endY[2] = origin[2] + static_cast<float>(m[6]) * axisLength;

        // Z축 끝점
        endZ[0] = origin[0] + static_cast<float>(m[8]) * axisLength;
        endZ[1] = origin[1] + static_cast<float>(m[9]) * axisLength;
        endZ[2] = origin[2] + static_cast<float>(m[10]) * axisLength;
    }



    AxisObject& operator=(const Transform& tf)
    {
        this->setTransform(tf);
        return *this;
    }


    void Draw() const
    {
        float line_x[2], line_y[2], line_z[2];

        // --- X축 (빨간색) ---
        line_x[0] = origin[0]; line_y[0] = origin[1]; line_z[0] = origin[2];
        line_x[1] = endX[0];   line_y[1] = endX[1];   line_z[1] = endX[2];

        ImPlot3D::PushStyleColor(ImPlot3DCol_Line, ImVec4(1.0f, 0.0f, 0.0f, 1.0f));
        ImPlot3D::PlotLine("X-Axis", line_x, line_y, line_z, 2);
        ImPlot3D::PopStyleColor();

        // --- Y축 (녹색) ---
        line_x[0] = origin[0]; line_y[0] = origin[1]; line_z[0] = origin[2];
        line_x[1] = endY[0];   line_y[1] = endY[1];   line_z[1] = endY[2];

        ImPlot3D::PushStyleColor(ImPlot3DCol_Line, ImVec4(0.0f, 1.0f, 0.0f, 1.0f));
        ImPlot3D::PlotLine("Y-Axis", line_x, line_y, line_z, 2);
        ImPlot3D::PopStyleColor();

        // --- Z축 (파란색) ---
        line_x[0] = origin[0]; line_y[0] = origin[1]; line_z[0] = origin[2];
        line_x[1] = endZ[0];   line_y[1] = endZ[1];   line_z[1] = endZ[2];

        ImPlot3D::PushStyleColor(ImPlot3DCol_Line, ImVec4(0.0f, 0.0f, 1.0f, 1.0f));
        ImPlot3D::PlotLine("Z-Axis", line_x, line_y, line_z, 2);
        ImPlot3D::PopStyleColor();
    }

    // 원점 좌표를 반환하는 함수 추가
    void GetOrigin(float& x, float& y, float& z) const
    {
        x = origin[0];
        y = origin[1];
        z = origin[2];
    }

private:
    float origin[3]; // 축의 원점 (X, Y, Z)
    float endX[3];   // X축의 끝점 (X, Y, Z)
    float endY[3];   // Y축의 끝점 (X, Y, Z)
    float endZ[3];   // Z축의 끝점 (X, Y, Z)
};


// ===================================================================
//  링크선 그리기 함수 (노란색)
// ===================================================================
void DrawLinkLine(const AxisObject& from, const AxisObject& to)
{
    float from_x, from_y, from_z;
    float to_x, to_y, to_z;

    from.GetOrigin(from_x, from_y, from_z);
    to.GetOrigin(to_x, to_y, to_z);

    float line_x[2] = {from_x, to_x};
    float line_y[2] = {from_y, to_y};
    float line_z[2] = {from_z, to_z};

    // 노란색으로 링크선 그리기
    ImPlot3D::PushStyleColor(ImPlot3DCol_Line, ImVec4(1.0f, 1.0f, 0.0f, 1.0f));
    ImPlot3D::PlotLine("Link", line_x, line_y, line_z, 2);
    ImPlot3D::PopStyleColor();
}



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
AxisObject t_tip(0.06);
AxisObject end_effector(0.06);


// 끝단의 매니풀러빌리티와 로봇 링크 관절을 그리는 함수
void draw_simulation(const mat<6, 6>& j, const Transform& fk);
void draw_simulation(const Transform& fk);


mat<6, 6> pInv_DLS(const mat<6, 6>& J) {
    double lambda = 0.1; // 감쇠 계수
    mat<6, 6> I = mat<6, 6>::Identity();
    mat<6, 6> JJ_T = J * J.transpose();
    mat<6, 6> temp = JJ_T + lambda * lambda * I;
    return J.transpose() * temp.inverse();
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

    std::array<double, 6> q_rad;        // 각 Joint 각도 [rad]
    std::array<Transform, 8> tf;        // 각 조인트의 transform
    std::optional<Transform> tf_ee;     // 엔드 이펙터 transform

    std::array<JointLimits, 6> joint_limits;  // 각도 제한 조건

    mat<6, 6> J;                        // 자코비안 매트릭스
    double lambda;                      // Damping Factor (특이점 방지용)


    M1013()
    {
        joint_limits[0] = {DEG_TO_RAD(-360.0), DEG_TO_RAD(360.0)};
        joint_limits[1] = {DEG_TO_RAD(-360.0), DEG_TO_RAD(360.0)};
        joint_limits[2] = {DEG_TO_RAD(-150.0), DEG_TO_RAD(150.0)};
        joint_limits[3] = {DEG_TO_RAD(-360.0), DEG_TO_RAD(360.0)};
        joint_limits[4] = {DEG_TO_RAD(-360.0), DEG_TO_RAD(360.0)};
        joint_limits[5] = {DEG_TO_RAD(-360.0), DEG_TO_RAD(360.0)};

        lambda = 0.01;

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
        quat t_rot(AngleAxis(yaw, vec3::UnitZ()) *             // Z
                   AngleAxis(pitch + M_PI_2, vec3::UnitY()) *  // Y
                   AngleAxis(roll, vec3::UnitZ()));            // Z

        vec3 t_pos(x, y, z);

        Transform target_tf(t_rot, t_pos);

        auto q = ik(target_tf);
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
        Transform tf0 = Transform(quat(AngleAxis(0, vec3::UnitZ())), vec3(0, 0, 0));             // Base
        Transform tf1 = Transform(quat(AngleAxis(q0, vec3::UnitZ())), vec3(0, 0, l1));           // Joint 1
        Transform tf2 = Transform(quat(AngleAxis(q1, vec3::UnitX())), vec3(-l2, 0, 0));          // Joint 2
        Transform tf3 = Transform(quat(AngleAxis(q2, vec3::UnitX())), vec3(0, 0, l3));           // Joint 3
        Transform tf4 = Transform(quat(AngleAxis(q3, vec3::UnitZ())), vec3(l4, 0, 0));           // Joint 4
        Transform tf5 = Transform(quat(AngleAxis(q4, vec3::UnitX())), vec3(-l6, 0, l5));         // Joint 5
        Transform tf6 = Transform(quat(AngleAxis(q5, vec3::UnitZ())), vec3(l7, 0, 0));           // Joint 6
        Transform tf7 = Transform(quat(AngleAxis(0, vec3::UnitZ())), vec3(0, 0, l8));            // Tool tip



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


    // Inverse Kinematics
        std::array<double, 6> ik(const Transform& target_tf)
    {
        auto q = q_rad;


        // 위치 오차
        vec3 pos_error = target_tf.translation() - fk(q[0], q[1], q[2], q[3], q[4], q[5]).translation();


        // 회전 오차
        mat3 R_cur = fk(q[0], q[1], q[2], q[3], q[4], q[5]).rotation();
        mat3 R_tar = target_tf.rotation();

        // 상대 회전 행렬 계산 (R_diff = R_target * R_current^T)
        // 이 순서로 곱해야 "Global Frame(Base)" 기준의 오차가 계산됨
        mat3 R_diff = R_tar * R_cur.transpose();

        // 회전 행렬에서 회전 벡터(wx, wy, wz) 추출
        // 로그 맵(Logarithmic Map) 근사식 이용: v = 0.5 * (R - R^T)
        vec3 ori_error;
        ori_error.x() = 0.5 * (R_diff(2, 1) - R_diff(1, 2)); // wx
        ori_error.y() = 0.5 * (R_diff(0, 2) - R_diff(2, 0)); // wy
        ori_error.z() = 0.5 * (R_diff(1, 0) - R_diff(0, 1)); // wz



        // 오차 벡터
        vec<6> dx;
        dx <<   pos_error.x(),
                pos_error.y(),
                pos_error.z(),
                ori_error.x(),
                ori_error.y(),
                ori_error.z();


        // 자코비안 행렬
        J = jacobian( q[0], q[1], q[2], q[3], q[4], q[5] );




        // QP Formulation: min || J*dq - dx ||^2 + lambda || dq ||^2
        // 전개하면: dq^T * (J^T*J + lambda*I) * dq - 2*(J^T*dx)^T * dq
        // H = J^T * J + lambda * I
        // g = -J^T * dx

        mat<6, 6> J_t = J.transpose();
        mat<6, 6> H = J_t * J;
        vec<6> g = -J_t * dx;


        // Damping 추가 (H 대각 성분에 lambda 더하기)
        for (int i = 0; i < 6; ++i)
            H(i,i) += lambda * lambda;




        // 제약조건 설정 (Bounds)
        // 현재 각도에서 한계까지 남은 거리(Distance)를 계산하여 dq의 상하한(lb, ub)으로 설정
        vec<6> lb, ub;

        for(int i=0; i<6; ++i) {
            double current_rad = q[i];
            double min_rad = joint_limits[i].min;
            double max_rad = joint_limits[i].max;

            // dq는 "변위"이므로, 현재 위치에서 갈 수 있는 최대/최소 변위를 구함
            // 안전 여유(Buffer)를 조금 둘 수도 있음
            lb(i) = (min_rad - current_rad);
            ub(i) = (max_rad - current_rad);

            // 한 번에 너무 많이 움직이지 않도록 최대 속도 제한을 걸 수도 있음
            double max_step = DEG_TO_RAD(30.0);
            lb(i) = std::max(lb(i), -max_step);
            ub(i) = std::min(ub(i), max_step);
        }


        // QP Solver 실행
        vec<6> dq = vec<6>::Zero();
        const int max_iter = 20; // Gauss-Seidel 반복 횟수 (보통 10~20번이면 충분히 수렴)

        for (int iter = 0; iter < max_iter; ++iter) {
            for (int i = 0; i < 6; ++i) {
                // Sigma (H_ij * x_j) 계산 (j != i)
                double sigma = 0.0;
                for (int j = 0; j < 6; ++j) {
                    if (i != j) {
                        sigma += H(i, j) * dq(j);
                    }
                }

                // x_i 업데이트 공식: ( -g_i - sigma ) / H_ii
                double val = (-g(i) - sigma) / H(i, i);

                // 계산된 값을 즉시 제약 조건 범위 내로 투영(Project/Clamp)
                if (val < lb(i)) val = lb(i);
                if (val > ub(i)) val = ub(i);

                dq(i) = val;
            }
        }


        // 최종 각도 업데이트
        for (int i = 0; i < 6; ++i)
            q[i] = NORM_RAD_180(q[i] + dq(i) * 0.5);


        return q;
    }


    // 자코비안 행렬
    mat<6, 6> jacobian(double q0, double q1, double q2, double q3, double q4, double q5)
    {
        mat<6, 6> J_ = mat<6, 6>::Zero();

        double c0 = std::cos(q0);
        double c1 = std::cos(q1);
        double c3 = std::cos(q3);
        double c4 = std::cos(q4);
        double s0 = std::sin(q0);
        double s1 = std::sin(q1);
        double s3 = std::sin(q3);
        double s4 = std::sin(q4);
        double c12 = std::cos(q1 + q2);
        double s12 = std::sin(q1 + q2);


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
    double l1 = 0.1525; // [m]
    double l2 = 0.1985; // [m]
    double l3 = 0.620; // [m]
    double l4 = 0.164; // [m]
    double l5 = 0.559; // [m]
    double l6 = 0.146; // [m]
    double l7 = 0.146; // [m]
    double l8 = 0.121; // [m]
};




M1013 m1013;


/*int main() {
    std::cout << "두산 로봇 M1013 IK - Jacobian 단계 제약조건 통합" << std::endl;


    M1013 m1013;

    // 기본 자세 설정
    float q_deg[6] = {90.0f, 30.0f, 80.0f, 0.0f, 75.0f, 0.0f};
    vec3 tPos(0.62, 0.0, 0.235);
    quat qRot(AngleAxis(DEG_TO_RAD(0), vec3(0, 1, 0)));
    transform target_tf(qRot, tPos);

    float target_x      = target_tf.x();
    float target_y      = target_tf.y();
    float target_z      = target_tf.z();
    float target_roll   = RAD_TO_DEG(target_tf.roll());
    float target_pitch  = RAD_TO_DEG(target_tf.pitch());
    float target_yaw    = RAD_TO_DEG(target_tf.yaw());



    transform tf_ee = transform(
        quat(AngleAxis(DEG_TO_RAD(0), vec3::UnitY())),
        vec3(0, 0.1, 0.0));

    m1013.set_end_effector_tf(tf_ee);


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
            ImGui::DragFloat("##Target x", &target_x, 0.01f, -1.0f, 1.0f, "x: %.3f");
            ImGui::DragFloat("##Target y", &target_y, 0.01f, -1.0f, 1.0f, "y: %.3f");
            ImGui::DragFloat("##Target z", &target_z, 0.001f, -1.0f, 2.0f, "z: %.3f");
            ImGui::Dummy(ImVec2(0, 10));
            ImGui::DragFloat("##Target Roll", &target_roll, 1.0f, -180.0f, 180.0f, "Roll: %.3f");
            ImGui::DragFloat("##Target Pitch", &target_pitch, 1.0f, -130.0f, 130.0f, "Pitch: %.3f");
            ImGui::DragFloat("##Target Yaw", &target_yaw, 1.0f, -180.0f, 180.0f, "Yaw: %.3f");
            ImGui::PopItemWidth();

            if (is_joint_change)
            {
                m1013.movej(DEG_TO_RAD(q0), DEG_TO_RAD(q1), DEG_TO_RAD(q2), DEG_TO_RAD(q3), DEG_TO_RAD(q4), DEG_TO_RAD(q5));
            }



            // Axis Transform 설정
            joint1.setTransform(m1013.get_fk(1));
            joint2.setTransform(m1013.get_fk(2));
            joint3.setTransform(m1013.get_fk(3));
            joint4.setTransform(m1013.get_fk(4));
            joint5.setTransform(m1013.get_fk(5));
            joint6.setTransform(m1013.get_fk(6));
            t_tip.setTransform(m1013.get_fk(7));
            m1013.movel(target_x, target_y, target_z, DEG_TO_RAD(target_roll), DEG_TO_RAD(target_pitch), DEG_TO_RAD(target_yaw));



            // Ui
            {
                transform tf = m1013.get_fk(7);
                ImGui::Text("FK: x=%.3f, y=%.3f, z=%.3f", tf.P().x(), tf.P().y(), tf.P().z());
                ImGui::Text("FK: roll=%.3f, pitch=%.3f, yaw=%.3f", RAD_TO_DEG(tf.roll()), RAD_TO_DEG(tf.pitch()), RAD_TO_DEG(tf.yaw()));

                ImGui::Dummy(ImVec2(0, 10));

                draw_simulation(m1013.J, tf);

            }
            ImGui::End(); // "제어"
        });







    }
    ImGui::stop();

    return 0;
}*/





// -----------------------------------------------
// 이 아래 코드는 생성하지 않아도됨. 필요시에만 별도로 작성
// ------------------------------------------------


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

        mat<3, 6> j_pos = j.block<3, 6>(0, 0);
        imgui_draw_manipulability(j_pos, fk.translation(), 0.3f, ImVec4(0,1,1,0.1f));

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












// 문자열에서 쉼표(,)로 구분된 숫자 벡터를 반환합니다.
std::vector<double> parse_double_vector(const std::string& data_string) {
    std::vector<double> result;
    // 1. 문자열 스트림 생성
    std::stringstream ss(data_string);
    std::string segment;

    // 2. 쉼표(,)를 구분자로 문자열 분리 및 처리
    while (std::getline(ss, segment, ',')) {
        // 3. 앞뒤 공백 제거 (trim)
        // 공백 제거를 위해 string의 복사본을 만들어 처리합니다.
        // C++20부터는 std::erase_if로 더 간단하게 할 수 있지만,
        // 범용성을 위해 일반적인 방법으로 처리합니다.

        // 문자열의 시작 부분에서 공백 제거
        segment.erase(segment.begin(), std::find_if(segment.begin(), segment.end(), [](unsigned char ch) {
            return !std::isspace(ch);
        }));

        // 문자열의 끝 부분에서 공백 제거
        segment.erase(std::find_if(segment.rbegin(), segment.rend(), [](unsigned char ch) {
            return !std::isspace(ch);
        }).base(), segment.end());

        // 4. 비어있지 않은 세그먼트를 double로 변환하여 벡터에 추가
        if (!segment.empty()) {
            try {
                // std::stod를 사용하여 문자열을 double로 변환 (음수, 소수점 자동 처리)
                result.push_back(std::stod(segment));
            } catch (const std::exception& e) {
                // 변환 실패 시 (숫자가 아닌 문자가 포함된 경우 등)
                std::cerr << "경고: 유효하지 않은 숫자 형식 발견 -> \"" << segment << "\". 이 값은 건너뜁니다." << std::endl;
            }
        }
    }

    return result;
}




// 전역 세션 관리
std::mutex g_sessions_mutex;
std::set<std::shared_ptr<WebServer::session>> g_sessions;


void connect_callback(std::shared_ptr<WebServer::session> session)
{
    {
        std::lock_guard<std::mutex> lock(g_sessions_mutex);
        g_sessions.insert(session);
    }
    session->send("Connected!");
}


void disconnect_callback(std::shared_ptr<WebServer::session> session)
{
    {
        std::lock_guard<std::mutex> lock(g_sessions_mutex);
        g_sessions.erase(session);
    }
    session->send("Disconnected!");
}


double x        = 0;
double y        = 0;
double z        = 0;
double roll     = 0;
double pitch    = 0;
double yaw      = 0;
void receive_callback(std::shared_ptr<WebServer::session> session, std::string message)
{
    std::vector<double> data = parse_double_vector(message);


    if (!data.empty())
    {
        std::cout   << "Data: "
                    << data[0] << ", " << data[1] << ", " << data[2] << ", "
                    << data[3] << ", " << data[4] << ", " << data[5]
                    << std::endl;

        x        = data[0];
        y        = data[1];
        z        = data[2];
        roll     = DEG_TO_RAD(data[3]);
        pitch    = DEG_TO_RAD(data[4]);
        yaw      = DEG_TO_RAD(data[5]);


        m1013.movel(x, y, z, roll, pitch, yaw);
    }
    // session->send("You sent: " + message);
}




// =================================================================
//              메인 함수 (콜백 로직 수정됨)
// =================================================================
int main(int argc, char* argv[])
{
    auto const address = WebServer::make_address("127.0.0.1");
    auto const port = static_cast<unsigned short>(8080);
    auto const threads = std::max<int>(1, static_cast<int>(std::thread::hardware_concurrency()));

    WebServer::net::io_context ioc{threads};
    auto listener_ptr = std::make_shared<WebServer::listener>(ioc, WebServer::tcp::endpoint{address, port});

    // --- 여기에 콜백 함수들을 정의하고 리스너에 주입합니다 ---
    listener_ptr->on_session_created = [](std::shared_ptr<WebServer::session> s)
    {
        std::cout << "[Main] New session created. Setting callbacks." << std::endl;

        s->on_connect = connect_callback;

        s->on_receive = receive_callback;

        s->on_disconnect = disconnect_callback;
    };













    std::thread t_sender = std::thread([&]()
    {
        while(true)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            // std::cout << "Sending stream..." << std::endl;

            double q0 = RAD_TO_DEG(m1013.q_rad[0]);
            double q1 = RAD_TO_DEG(m1013.q_rad[1]);
            double q2 = RAD_TO_DEG(m1013.q_rad[2]);
            double q3 = RAD_TO_DEG(m1013.q_rad[3]);
            double q4 = RAD_TO_DEG(m1013.q_rad[4]);
            double q5 = RAD_TO_DEG(m1013.q_rad[5]);

            m1013.movel(x, y, z, roll, pitch, yaw);


            // 모든 활성 세션에게 메시지 전송
            std::lock_guard<std::mutex> lock(g_sessions_mutex);
            for(auto& s : g_sessions)
            {
                std::stringstream ss;
                ss << q0 << "," << q1 << "," << q2 << "," << q3 << "," << q4 << "," << q5;
                s->send(ss.str());
            }
        }
    });

    t_sender.detach();


    std::thread imgui_thread = std::thread([&]()
    {
        ImGui::start("데모");

        while (ImGui::isRunning())
        {
            // Axis Transform 설정
            ImGui::draw([&]()
            {
                ImGui::Begin("Axis Transform");
                joint1.setTransform(m1013.get_fk(1));
                joint2.setTransform(m1013.get_fk(2));
                joint3.setTransform(m1013.get_fk(3));
                joint4.setTransform(m1013.get_fk(4));
                joint5.setTransform(m1013.get_fk(5));
                joint6.setTransform(m1013.get_fk(6));
                t_tip.setTransform(m1013.get_fk(7));
                auto tf = m1013.get_fk(7);
                m1013.movel(x, y, z, DEG_TO_RAD(roll), DEG_TO_RAD(pitch), DEG_TO_RAD(yaw));
                draw_simulation(m1013.J, tf);

                ImGui::End();
            } );

        }
        ImGui::stop();



    });
    imgui_thread.detach();


    listener_ptr->run();
    std::cout << "Server listening on 127.0.0.1:" << port << std::endl;

    std::vector<std::thread> v;
    v.reserve(threads - 1);
    for(auto i = threads - 1; i > 0; --i)
        v.emplace_back([&ioc] { ioc.run(); });
    ioc.run();

    return EXIT_SUCCESS;
}

