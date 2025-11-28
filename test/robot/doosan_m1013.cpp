#include <chrono>

#include "common_types.h"
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

    std::array<JointLimits, 6> limits;  // 각도 제한 조건

    mat<6, 6> J;                        // 자코비안 매트릭스
    double lambda;                      // Damping Factor (특이점 방지용)


    M1013()
    {
        limits[0] = {DEG_TO_RAD(-0.0), DEG_TO_RAD(180.0)};
        limits[1] = {DEG_TO_RAD(-360.0), DEG_TO_RAD(360.0)};
        limits[2] = {DEG_TO_RAD(-0.0), DEG_TO_RAD(160.0)};
        limits[3] = {DEG_TO_RAD(-360.0), DEG_TO_RAD(360.0)};
        limits[4] = {DEG_TO_RAD(-360.0), DEG_TO_RAD(360.0)};
        limits[5] = {DEG_TO_RAD(-360.0), DEG_TO_RAD(360.0)};

        lambda = 0.1;

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
        tf[0] = Transform(AngleAxis(0,  vec3::UnitZ()), vec3(0, 0, 0));     // Base
        tf[1] = Transform(AngleAxis(q0, vec3::UnitZ()), vec3(0, 0, l1));    // Joint 1
        tf[2] = Transform(AngleAxis(q1, vec3::UnitX()), vec3(-l2, 0, 0));   // Joint 2
        tf[3] = Transform(AngleAxis(q2, vec3::UnitX()), vec3(0, 0, l3));    // Joint 3
        tf[4] = Transform(AngleAxis(q3, vec3::UnitZ()), vec3(l4, 0, 0));    // Joint 4
        tf[5] = Transform(AngleAxis(q4, vec3::UnitX()), vec3(-l6, 0, l5));  // Joint 5
        tf[6] = Transform(AngleAxis(q5, vec3::UnitZ()), vec3(l7, 0, 0));    // Joint 6
        tf[7] = Transform(AngleAxis(0,  vec3::UnitZ()), vec3(0, 0, l8));    // Tool tip


        // Forward Kinematics
        tf[0] = tf[0];
        tf[1] = tf[0] * tf[1];
        tf[2] = tf[1] * tf[2];
        tf[3] = tf[2] * tf[3];
        tf[4] = tf[3] * tf[4];
        tf[5] = tf[4] * tf[5];
        tf[6] = tf[5] * tf[6];
        tf[7] = tf[6] * tf[7];

        if (tf_ee)
        {

            tf[7] = tf[7] * tf_ee.value();

        }

        return tf[7]; // End Effector's FK
    }


    // Inverse Kinematics
    std::array<double, 6> ik(const Transform& target_tf)
    {
        // 초기 각도 설정
        auto q = q_rad;


        // ----------------------------------------------------
        // 1. 오차 계산 (위치 + 회전)
        // ----------------------------------------------------

        // 현재 FK 계산
        Transform current_tf = fk(q[0], q[1], q[2], q[3], q[4], q[5]);

        // 위치 오차
        vec3 pos_error = target_tf.trans() - current_tf.trans();

        // 회전 오차
        mat3 R_cur = current_tf.rot();
        mat3 R_tar = target_tf.rot();
        mat3 R_diff = R_tar * R_cur.transpose(); // Global Frame 기준 오차

        vec3 ori_error;
        // 로그 맵(Logarithmic Map) 근사식: v = 0.5 * (R - R^T)
        ori_error.x() = 0.5 * (R_diff(2, 1) - R_diff(1, 2)); // wx
        ori_error.y() = 0.5 * (R_diff(0, 2) - R_diff(2, 0)); // wy
        ori_error.z() = 0.5 * (R_diff(1, 0) - R_diff(0, 1)); // wz

        // 오차 벡터 dx (6x1)
        vec<6> dx;
        dx <<   pos_error.x(), pos_error.y(), pos_error.z(),
                ori_error.x(), ori_error.y(), ori_error.z();


        // ----------------------------------------------------
        // 2. 자코비안 계산
        // ----------------------------------------------------
        J = jacobian( q[0], q[1], q[2], q[3], q[4], q[5] );
        mat<6, 6> J_inv = pInv_DLS(J, lambda);
        vec<6> dq = J_inv * dx;



        // ----------------------------------------------------
        // 6. 최종 각도 업데이트
        // ----------------------------------------------------
        for (int i = 0; i < 6; ++i) {
            // dq를 적용하고, 각도 정규화 (예: -180도 ~ +180도)
            q[i] = NORM_RAD_180(q[i] + dq(i) * 0.1);
        }



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
            ImGui::DragFloat("##Target x", &target_x, 0.01f, -2.0f, 2.0f, "x: %.3f");
            ImGui::DragFloat("##Target y", &target_y, 0.01f, -2.0f, 2.0f, "y: %.3f");
            ImGui::DragFloat("##Target z", &target_z, 0.001f, -2.0f, 2.0f, "z: %.3f");
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
                Transform tf = m1013.get_fk(7);
                ImGui::Text("FK: x=%.3f, y=%.3f, z=%.3f", tf.trans().x(), tf.trans().y(), tf.trans().z());
                ImGui::Text("FK: roll=%.3f, pitch=%.3f, yaw=%.3f", RAD_TO_DEG(tf.roll()), RAD_TO_DEG(tf.pitch()), RAD_TO_DEG(tf.yaw()));

                ImGui::Dummy(ImVec2(0, 10));

                draw_simulation(m1013.J, tf);

            }
            ImGui::End(); // "제어"
        });







    }
    ImGui::stop();

    return 0;
}





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
