#include <chrono>

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
AxisObject endEffector(0.06);


// 끝단의 매니풀러빌리티와 로봇 링크 관절을 그리는 함수
void draw_simulation(const mat<6, 6>& j, const transform& fk);
void draw_simulation(const transform& fk);


mat<6, 6> pInv_DLS(const mat<6, 6>& J) {
    double lambda = 0.1; // 감쇠 계수
    mat<6, 6> I = mat<6, 6>::Identity();
    mat<6, 6> JJ_T = J * J.transpose();
    mat<6, 6> temp = JJ_T + lambda * lambda * I;
    return J.transpose() * temp.inverse();
}



class M1013 {
public:
    double l1 = 0.135;  // [m]
    double l2 = 0.1702; // [m]
    double l3 = 0.411;  // [m]
    double l4 = 0.164;  // [m]
    double l5 = 0.368;  // [m]
    double l6 = 0.1522; // [m]
    double l7 = 0.146;  // [m]
    double l8 = 0.121;  // [m]


    std::array<double, 6> q_rad;
    std::array<transform, 8> tf;
    mat<6, 6> J;


    M1013()
    {
        J = mat<6, 6>::Zero();
        fk();
    }

    transform get_fk(int i)
    {
        return tf[i];
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

        transform target_tf(t_rot, t_pos);

        auto q = ik(target_tf);
        movej(q[0], q[1], q[2], q[3], q[4], q[5]);
    }


private:

    // Forward Kinematics
    transform fk()
    {
        return fk(q_rad[0], q_rad[1], q_rad[2], q_rad[3], q_rad[4], q_rad[5]);
    }

    transform fk(double q0, double q1, double q2, double q3, double q4, double q5)
    {
        // 각 조인트의 transform 계산
        transform tf0 = transform(quat(AngleAxis(0, vec3::UnitZ())), vec3(0, 0, 0));             // Base
        transform tf1 = transform(quat(AngleAxis(q0, vec3::UnitZ())), vec3(0, 0, l1));           // Joint 1
        transform tf2 = transform(quat(AngleAxis(q1, vec3::UnitX())), vec3(-l2, 0, 0));          // Joint 2
        transform tf3 = transform(quat(AngleAxis(q2, vec3::UnitX())), vec3(0, 0, l3));           // Joint 3
        transform tf4 = transform(quat(AngleAxis(q3, vec3::UnitZ())), vec3(l4, 0, 0));           // Joint 4
        transform tf5 = transform(quat(AngleAxis(q4, vec3::UnitX())), vec3(-l6, 0, l5));         // Joint 5
        transform tf6 = transform(quat(AngleAxis(q5, vec3::UnitZ())), vec3(l7, 0, 0));           // Joint 6
        transform tf7 = transform(quat(AngleAxis(0, vec3::UnitZ())), vec3(0, 0, l8));            // Tool tip

        // Forward Kinematics
        tf[0] = tf0;
        tf[1] = tf[0] * tf1;
        tf[2] = tf[1] * tf2;
        tf[3] = tf[2] * tf3;
        tf[4] = tf[3] * tf4;
        tf[5] = tf[4] * tf5;
        tf[6] = tf[5] * tf6;
        tf[7] = tf[6] * tf7;

        return tf[7]; // End Effector's FK
    }


    // Inverse Kinematics
    std::array<double, 6> ik(const transform& target_tf)
    {
        auto q = q_rad;


        // 위치 오차
        vec3 pos_error = target_tf.P() - fk(q[0], q[1], q[2], q[3], q[4], q[5]).P();


        // 회전 오차
        mat3 R_cur = fk(q[0], q[1], q[2], q[3], q[4], q[5]).R();
        mat3 R_tar = target_tf.R();

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
        J = jacobian(
                q[0],
                q[1],
                q[2],
                q[3],
                q[4],
                q[5]);


        // 자코비안 인버스
        mat<6, 6> J_inv = pInv_DLS(J);


        // dq 계산
        vec<6> dq = J_inv * dx * 0.5;


        // 각도 업데이트 [rad]
        q[0] += dq(0);
        q[1] += dq(1);
        q[2] += dq(2);
        q[3] += dq(3);
        q[4] += dq(4);
        q[5] += dq(5);


        // 각도 정규화 [rad]
        q[0] = NORM_RAD_180(q[0]);
        q[1] = NORM_RAD_180(q[1]);
        q[2] = NORM_RAD_180(q[2]);
        q[3] = NORM_RAD_180(q[3]);
        q[4] = NORM_RAD_180(q[4]);
        q[5] = NORM_RAD_180(q[5]);


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


};




int main() {
    std::cout << "두산 로봇 M1013 IK - Jacobian 단계 제약조건 통합" << std::endl;


    M1013 m1013;

    // 기본 자세 설정
    float q_deg[6] = {90.0f, 30.0f, 80.0f, 0.0f, 75.0f, 0.0f};
    vec3 tPos(0.62, 0.0, 0.235);
    quat qRot(AngleAxis(DEG_TO_RAD(90), vec3(0, 1, 0)));
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




        // Axis Transform 설정
        joint1.setTransform(m1013.get_fk(1));
        joint2.setTransform(m1013.get_fk(2));
        joint3.setTransform(m1013.get_fk(3));
        joint4.setTransform(m1013.get_fk(4));
        joint5.setTransform(m1013.get_fk(5));
        joint6.setTransform(m1013.get_fk(6));
        endEffector.setTransform(m1013.get_fk(7));
        m1013.movel(target_x, target_y, target_z, DEG_TO_RAD(target_roll), DEG_TO_RAD(target_pitch), DEG_TO_RAD(target_yaw));


    }
    ImGui::stop();

    return 0;
}





// -----------------------------------------------
// 이 아래 코드는 생성하지 않아도됨. 필요시에만 별도로 작성
// ------------------------------------------------


void draw_simulation(const mat<6, 6>& j, const transform& fk)
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
        imgui_draw_manipulability(j_pos, fk.P(), 0.3f, ImVec4(0,1,1,0.1f));

        ImPlot3D::EndPlot();
    }
    ImGui::End();
}


void draw_simulation(const transform& fk)
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


        ImPlot3D::EndPlot();
    }
    ImGui::End();
}

