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
void draw_simulation(const transform& fk);

// 두산 M1013 Jacobian 함수
mat<6, 6> Jcobian(double q0, double q1, double q2, double q3, double q4, double q5);

mat<6, 6> pInv_DLS(const mat<6, 6>& J) {
    double lambda = 0.01; // 감쇠 계수 (조정이 필요할 수 있습니다)
    mat<6, 6> I = mat<6, 6>::Identity();
    mat<6, 6> JJ_T = J * J.transpose();
    mat<6, 6> temp = JJ_T + lambda * lambda * I;
    return J.transpose() * temp.inverse();
}



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
            {
                // 1. 오차(dx) 계산
                double error_x = target_tf.x() - fk.x();
                double error_y = target_tf.y() - fk.y();
                double error_z = target_tf.z() - fk.z();
                double current_alpha = atan2(fk(1, 2), fk(0, 2));
                double current_beta  = acos(fk(2, 2));
                double current_gamma = atan2(fk(2, 1), -fk(2, 0));

                double target_alpha = atan2(target_tf(1, 2), target_tf(0, 2));
                double target_beta  = acos(target_tf(2, 2));
                double target_gamma = atan2(target_tf(2, 1), -target_tf(2, 0));

                double error_roll  = target_alpha - current_alpha;   // Z축 회전 오차 (wx)
                double error_pitch = target_beta - current_beta;     // Y축 회전 오차 (wy)
                double error_yaw   = target_gamma - current_gamma;   // Z축 회전 오차 (wz)


                vec<6> dx;
                dx <<   error_x,
                        error_y,
                        error_z,
                        error_roll,
                        error_pitch,
                        error_yaw;



                // 2. Jacobian 계산
                mat<6, 6> J = Jcobian(
                        DEG_TO_RAD(q_deg[0]),
                        DEG_TO_RAD(q_deg[1]),
                        DEG_TO_RAD(q_deg[2]),
                        DEG_TO_RAD(q_deg[3]),
                        DEG_TO_RAD(q_deg[4]),
                        DEG_TO_RAD(q_deg[5])
                );


                mat<6, 6> J_inv = pInv_DLS(J);
                vec<6> dq = J_inv * dx * 0.1;

                q_deg[0] += RAD_TO_DEG(dq(0));
                q_deg[1] += RAD_TO_DEG(dq(1));
                q_deg[2] += RAD_TO_DEG(dq(2));
                q_deg[3] += RAD_TO_DEG(dq(3));
                q_deg[4] += RAD_TO_DEG(dq(4));
                q_deg[5] += RAD_TO_DEG(dq(5));
            }









            // Ui
            {
                ImGui::Text("FK: x=%.3f, y=%.3f, z=%.3f", fk.translation().x(), fk.translation().y(), fk.translation().z());
                ImGui::Text("FK: roll=%.3f, pitch=%.3f, yaw=%.3f", RAD_TO_DEG(fk.roll()), RAD_TO_DEG(fk.pitch()), RAD_TO_DEG(fk.yaw()));

                ImGui::Dummy(ImVec2(0, 10));

                draw_simulation(fk);

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


mat<6, 6> Jcobian(double q0, double q1, double q2, double q3, double q4, double q5)
{
    static mat<6, 6> J = mat<6, 6>::Zero();

J(0,0) = l2*std::sin(q0) + l3*std::sin(q1)*std::cos(q0) - l4*std::sin(q0) + l5*(std::sin(q1)*std::cos(q0)*std::cos(q2) + std::sin(q2)*std::cos(q0)*std::cos(q1)) - l6*((std::sin(q1)*std::sin(q2)*std::cos(q0) - std::cos(q0)*std::cos(q1)*std::cos(q2))*std::sin(q3) - std::sin(q0)*std::cos(q3)) + l7*((std::sin(q1)*std::sin(q2)*std::cos(q0) - std::cos(q0)*std::cos(q1)*std::cos(q2))*std::sin(q3) - std::sin(q0)*std::cos(q3)) + l8*((-(std::sin(q1)*std::sin(q2)*std::cos(q0) - std::cos(q0)*std::cos(q1)*std::cos(q2))*std::cos(q3) - std::sin(q0)*std::sin(q3))*std::sin(q4) + (std::sin(q1)*std::cos(q0)*std::cos(q2) + std::sin(q2)*std::cos(q0)*std::cos(q1))*std::cos(q4));
J(0,1) = l3*std::sin(q0)*std::cos(q1) + l5*(-std::sin(q0)*std::sin(q1)*std::sin(q2) + std::sin(q0)*std::cos(q1)*std::cos(q2)) - l6*(std::sin(q0)*std::sin(q1)*std::cos(q2) + std::sin(q0)*std::sin(q2)*std::cos(q1))*std::sin(q3) + l7*(std::sin(q0)*std::sin(q1)*std::cos(q2) + std::sin(q0)*std::sin(q2)*std::cos(q1))*std::sin(q3) + l8*((-std::sin(q0)*std::sin(q1)*std::sin(q2) + std::sin(q0)*std::cos(q1)*std::cos(q2))*std::cos(q4) - (std::sin(q0)*std::sin(q1)*std::cos(q2) + std::sin(q0)*std::sin(q2)*std::cos(q1))*std::sin(q4)*std::cos(q3));
J(0,2) = l5*(-std::sin(q0)*std::sin(q1)*std::sin(q2) + std::sin(q0)*std::cos(q1)*std::cos(q2)) - l6*(std::sin(q0)*std::sin(q1)*std::cos(q2) + std::sin(q0)*std::sin(q2)*std::cos(q1))*std::sin(q3) + l7*(std::sin(q0)*std::sin(q1)*std::cos(q2) + std::sin(q0)*std::sin(q2)*std::cos(q1))*std::sin(q3) + l8*((-std::sin(q0)*std::sin(q1)*std::sin(q2) + std::sin(q0)*std::cos(q1)*std::cos(q2))*std::cos(q4) - (std::sin(q0)*std::sin(q1)*std::cos(q2) + std::sin(q0)*std::sin(q2)*std::cos(q1))*std::sin(q4)*std::cos(q3));
J(0,3) = -l6*((std::sin(q0)*std::sin(q1)*std::sin(q2) - std::sin(q0)*std::cos(q1)*std::cos(q2))*std::cos(q3) - std::sin(q3)*std::cos(q0)) + l7*((std::sin(q0)*std::sin(q1)*std::sin(q2) - std::sin(q0)*std::cos(q1)*std::cos(q2))*std::cos(q3) - std::sin(q3)*std::cos(q0)) + l8*((std::sin(q0)*std::sin(q1)*std::sin(q2) - std::sin(q0)*std::cos(q1)*std::cos(q2))*std::sin(q3) + std::cos(q0)*std::cos(q3))*std::sin(q4);
J(0,4) = l8*((-(std::sin(q0)*std::sin(q1)*std::sin(q2) - std::sin(q0)*std::cos(q1)*std::cos(q2))*std::cos(q3) + std::sin(q3)*std::cos(q0))*std::cos(q4) - (std::sin(q0)*std::sin(q1)*std::cos(q2) + std::sin(q0)*std::sin(q2)*std::cos(q1))*std::sin(q4));
J(0,5) = 0;
J(1,0) = -l2*std::cos(q0) + l3*std::sin(q0)*std::sin(q1) + l4*std::cos(q0) + l5*(std::sin(q0)*std::sin(q1)*std::cos(q2) + std::sin(q0)*std::sin(q2)*std::cos(q1)) - l6*((std::sin(q0)*std::sin(q1)*std::sin(q2) - std::sin(q0)*std::cos(q1)*std::cos(q2))*std::sin(q3) + std::cos(q0)*std::cos(q3)) + l7*((std::sin(q0)*std::sin(q1)*std::sin(q2) - std::sin(q0)*std::cos(q1)*std::cos(q2))*std::sin(q3) + std::cos(q0)*std::cos(q3)) + l8*((-(std::sin(q0)*std::sin(q1)*std::sin(q2) - std::sin(q0)*std::cos(q1)*std::cos(q2))*std::cos(q3) + std::sin(q3)*std::cos(q0))*std::sin(q4) + (std::sin(q0)*std::sin(q1)*std::cos(q2) + std::sin(q0)*std::sin(q2)*std::cos(q1))*std::cos(q4));
J(1,1) = -l3*std::cos(q0)*std::cos(q1) + l5*(std::sin(q1)*std::sin(q2)*std::cos(q0) - std::cos(q0)*std::cos(q1)*std::cos(q2)) - l6*(-std::sin(q1)*std::cos(q0)*std::cos(q2) - std::sin(q2)*std::cos(q0)*std::cos(q1))*std::sin(q3) + l7*(-std::sin(q1)*std::cos(q0)*std::cos(q2) - std::sin(q2)*std::cos(q0)*std::cos(q1))*std::sin(q3) + l8*((std::sin(q1)*std::sin(q2)*std::cos(q0) - std::cos(q0)*std::cos(q1)*std::cos(q2))*std::cos(q4) - (-std::sin(q1)*std::cos(q0)*std::cos(q2) - std::sin(q2)*std::cos(q0)*std::cos(q1))*std::sin(q4)*std::cos(q3));
J(1,2) = l5*(std::sin(q1)*std::sin(q2)*std::cos(q0) - std::cos(q0)*std::cos(q1)*std::cos(q2)) - l6*(-std::sin(q1)*std::cos(q0)*std::cos(q2) - std::sin(q2)*std::cos(q0)*std::cos(q1))*std::sin(q3) + l7*(-std::sin(q1)*std::cos(q0)*std::cos(q2) - std::sin(q2)*std::cos(q0)*std::cos(q1))*std::sin(q3) + l8*((std::sin(q1)*std::sin(q2)*std::cos(q0) - std::cos(q0)*std::cos(q1)*std::cos(q2))*std::cos(q4) - (-std::sin(q1)*std::cos(q0)*std::cos(q2) - std::sin(q2)*std::cos(q0)*std::cos(q1))*std::sin(q4)*std::cos(q3));
J(1,3) = -l6*((-std::sin(q1)*std::sin(q2)*std::cos(q0) + std::cos(q0)*std::cos(q1)*std::cos(q2))*std::cos(q3) - std::sin(q0)*std::sin(q3)) + l7*((-std::sin(q1)*std::sin(q2)*std::cos(q0) + std::cos(q0)*std::cos(q1)*std::cos(q2))*std::cos(q3) - std::sin(q0)*std::sin(q3)) + l8*((-std::sin(q1)*std::sin(q2)*std::cos(q0) + std::cos(q0)*std::cos(q1)*std::cos(q2))*std::sin(q3) + std::sin(q0)*std::cos(q3))*std::sin(q4);
J(1,4) = l8*((-(-std::sin(q1)*std::sin(q2)*std::cos(q0) + std::cos(q0)*std::cos(q1)*std::cos(q2))*std::cos(q3) + std::sin(q0)*std::sin(q3))*std::cos(q4) - (-std::sin(q1)*std::cos(q0)*std::cos(q2) - std::sin(q2)*std::cos(q0)*std::cos(q1))*std::sin(q4));
J(1,5) = 0;
J(2,0) = 0;
J(2,1) = -l3*std::sin(q1) + l5*(-std::sin(q1)*std::cos(q2) - std::sin(q2)*std::cos(q1)) - l6*(-std::sin(q1)*std::sin(q2) + std::cos(q1)*std::cos(q2))*std::sin(q3) + l7*(-std::sin(q1)*std::sin(q2) + std::cos(q1)*std::cos(q2))*std::sin(q3) + l8*((std::sin(q1)*std::sin(q2) - std::cos(q1)*std::cos(q2))*std::sin(q4)*std::cos(q3) + (-std::sin(q1)*std::cos(q2) - std::sin(q2)*std::cos(q1))*std::cos(q4));
J(2,2) = l5*(-std::sin(q1)*std::cos(q2) - std::sin(q2)*std::cos(q1)) - l6*(-std::sin(q1)*std::sin(q2) + std::cos(q1)*std::cos(q2))*std::sin(q3) + l7*(-std::sin(q1)*std::sin(q2) + std::cos(q1)*std::cos(q2))*std::sin(q3) + l8*((std::sin(q1)*std::sin(q2) - std::cos(q1)*std::cos(q2))*std::sin(q4)*std::cos(q3) + (-std::sin(q1)*std::cos(q2) - std::sin(q2)*std::cos(q1))*std::cos(q4));
J(2,3) = -l6*(std::sin(q1)*std::cos(q2) + std::sin(q2)*std::cos(q1))*std::cos(q3) + l7*(std::sin(q1)*std::cos(q2) + std::sin(q2)*std::cos(q1))*std::cos(q3) - l8*(-std::sin(q1)*std::cos(q2) - std::sin(q2)*std::cos(q1))*std::sin(q3)*std::sin(q4);
J(2,4) = l8*(-(-std::sin(q1)*std::sin(q2) + std::cos(q1)*std::cos(q2))*std::sin(q4) + (-std::sin(q1)*std::cos(q2) - std::sin(q2)*std::cos(q1))*std::cos(q3)*std::cos(q4));
J(2,5) = 0;
J(3,0) = ((-(std::sin(q0)*std::sin(q1)*std::sin(q2) - std::sin(q0)*std::cos(q1)*std::cos(q2))*std::cos(q3) + std::sin(q3)*std::cos(q0))*std::sin(q4) + (std::sin(q0)*std::sin(q1)*std::cos(q2) + std::sin(q0)*std::sin(q2)*std::cos(q1))*std::cos(q4))*(-((std::sin(q0)*std::sin(q1)*std::sin(q2) - std::sin(q0)*std::cos(q1)*std::cos(q2))*std::cos(q3) - std::sin(q3)*std::cos(q0))*std::sin(q4) + (std::sin(q0)*std::sin(q1)*std::cos(q2) + std::sin(q0)*std::sin(q2)*std::cos(q1))*std::cos(q4))/(std::pow(-((std::sin(q0)*std::sin(q1)*std::sin(q2) - std::sin(q0)*std::cos(q1)*std::cos(q2))*std::cos(q3) - std::sin(q3)*std::cos(q0))*std::sin(q4) + (std::sin(q0)*std::sin(q1)*std::cos(q2) + std::sin(q0)*std::sin(q2)*std::cos(q1))*std::cos(q4), 2) + std::pow(-((-std::sin(q1)*std::sin(q2)*std::cos(q0) + std::cos(q0)*std::cos(q1)*std::cos(q2))*std::cos(q3) - std::sin(q0)*std::sin(q3))*std::sin(q4) + (-std::sin(q1)*std::cos(q0)*std::cos(q2) - std::sin(q2)*std::cos(q0)*std::cos(q1))*std::cos(q4), 2)) + (((-std::sin(q1)*std::sin(q2)*std::cos(q0) + std::cos(q0)*std::cos(q1)*std::cos(q2))*std::cos(q3) - std::sin(q0)*std::sin(q3))*std::sin(q4) - (-std::sin(q1)*std::cos(q0)*std::cos(q2) - std::sin(q2)*std::cos(q0)*std::cos(q1))*std::cos(q4))*((-(std::sin(q1)*std::sin(q2)*std::cos(q0) - std::cos(q0)*std::cos(q1)*std::cos(q2))*std::cos(q3) - std::sin(q0)*std::sin(q3))*std::sin(q4) + (std::sin(q1)*std::cos(q0)*std::cos(q2) + std::sin(q2)*std::cos(q0)*std::cos(q1))*std::cos(q4))/(std::pow(-((std::sin(q0)*std::sin(q1)*std::sin(q2) - std::sin(q0)*std::cos(q1)*std::cos(q2))*std::cos(q3) - std::sin(q3)*std::cos(q0))*std::sin(q4) + (std::sin(q0)*std::sin(q1)*std::cos(q2) + std::sin(q0)*std::sin(q2)*std::cos(q1))*std::cos(q4), 2) + std::pow(-((-std::sin(q1)*std::sin(q2)*std::cos(q0) + std::cos(q0)*std::cos(q1)*std::cos(q2))*std::cos(q3) - std::sin(q0)*std::sin(q3))*std::sin(q4) + (-std::sin(q1)*std::cos(q0)*std::cos(q2) - std::sin(q2)*std::cos(q0)*std::cos(q1))*std::cos(q4), 2));
J(3,1) = (-((std::sin(q0)*std::sin(q1)*std::sin(q2) - std::sin(q0)*std::cos(q1)*std::cos(q2))*std::cos(q3) - std::sin(q3)*std::cos(q0))*std::sin(q4) + (std::sin(q0)*std::sin(q1)*std::cos(q2) + std::sin(q0)*std::sin(q2)*std::cos(q1))*std::cos(q4))*((std::sin(q1)*std::sin(q2)*std::cos(q0) - std::cos(q0)*std::cos(q1)*std::cos(q2))*std::cos(q4) - (-std::sin(q1)*std::cos(q0)*std::cos(q2) - std::sin(q2)*std::cos(q0)*std::cos(q1))*std::sin(q4)*std::cos(q3))/(std::pow(-((std::sin(q0)*std::sin(q1)*std::sin(q2) - std::sin(q0)*std::cos(q1)*std::cos(q2))*std::cos(q3) - std::sin(q3)*std::cos(q0))*std::sin(q4) + (std::sin(q0)*std::sin(q1)*std::cos(q2) + std::sin(q0)*std::sin(q2)*std::cos(q1))*std::cos(q4), 2) + std::pow(-((-std::sin(q1)*std::sin(q2)*std::cos(q0) + std::cos(q0)*std::cos(q1)*std::cos(q2))*std::cos(q3) - std::sin(q0)*std::sin(q3))*std::sin(q4) + (-std::sin(q1)*std::cos(q0)*std::cos(q2) - std::sin(q2)*std::cos(q0)*std::cos(q1))*std::cos(q4), 2)) + (((-std::sin(q1)*std::sin(q2)*std::cos(q0) + std::cos(q0)*std::cos(q1)*std::cos(q2))*std::cos(q3) - std::sin(q0)*std::sin(q3))*std::sin(q4) - (-std::sin(q1)*std::cos(q0)*std::cos(q2) - std::sin(q2)*std::cos(q0)*std::cos(q1))*std::cos(q4))*((-std::sin(q0)*std::sin(q1)*std::sin(q2) + std::sin(q0)*std::cos(q1)*std::cos(q2))*std::cos(q4) - (std::sin(q0)*std::sin(q1)*std::cos(q2) + std::sin(q0)*std::sin(q2)*std::cos(q1))*std::sin(q4)*std::cos(q3))/(std::pow(-((std::sin(q0)*std::sin(q1)*std::sin(q2) - std::sin(q0)*std::cos(q1)*std::cos(q2))*std::cos(q3) - std::sin(q3)*std::cos(q0))*std::sin(q4) + (std::sin(q0)*std::sin(q1)*std::cos(q2) + std::sin(q0)*std::sin(q2)*std::cos(q1))*std::cos(q4), 2) + std::pow(-((-std::sin(q1)*std::sin(q2)*std::cos(q0) + std::cos(q0)*std::cos(q1)*std::cos(q2))*std::cos(q3) - std::sin(q0)*std::sin(q3))*std::sin(q4) + (-std::sin(q1)*std::cos(q0)*std::cos(q2) - std::sin(q2)*std::cos(q0)*std::cos(q1))*std::cos(q4), 2));
J(3,2) = (-((std::sin(q0)*std::sin(q1)*std::sin(q2) - std::sin(q0)*std::cos(q1)*std::cos(q2))*std::cos(q3) - std::sin(q3)*std::cos(q0))*std::sin(q4) + (std::sin(q0)*std::sin(q1)*std::cos(q2) + std::sin(q0)*std::sin(q2)*std::cos(q1))*std::cos(q4))*((std::sin(q1)*std::sin(q2)*std::cos(q0) - std::cos(q0)*std::cos(q1)*std::cos(q2))*std::cos(q4) - (-std::sin(q1)*std::cos(q0)*std::cos(q2) - std::sin(q2)*std::cos(q0)*std::cos(q1))*std::sin(q4)*std::cos(q3))/(std::pow(-((std::sin(q0)*std::sin(q1)*std::sin(q2) - std::sin(q0)*std::cos(q1)*std::cos(q2))*std::cos(q3) - std::sin(q3)*std::cos(q0))*std::sin(q4) + (std::sin(q0)*std::sin(q1)*std::cos(q2) + std::sin(q0)*std::sin(q2)*std::cos(q1))*std::cos(q4), 2) + std::pow(-((-std::sin(q1)*std::sin(q2)*std::cos(q0) + std::cos(q0)*std::cos(q1)*std::cos(q2))*std::cos(q3) - std::sin(q0)*std::sin(q3))*std::sin(q4) + (-std::sin(q1)*std::cos(q0)*std::cos(q2) - std::sin(q2)*std::cos(q0)*std::cos(q1))*std::cos(q4), 2)) + (((-std::sin(q1)*std::sin(q2)*std::cos(q0) + std::cos(q0)*std::cos(q1)*std::cos(q2))*std::cos(q3) - std::sin(q0)*std::sin(q3))*std::sin(q4) - (-std::sin(q1)*std::cos(q0)*std::cos(q2) - std::sin(q2)*std::cos(q0)*std::cos(q1))*std::cos(q4))*((-std::sin(q0)*std::sin(q1)*std::sin(q2) + std::sin(q0)*std::cos(q1)*std::cos(q2))*std::cos(q4) - (std::sin(q0)*std::sin(q1)*std::cos(q2) + std::sin(q0)*std::sin(q2)*std::cos(q1))*std::sin(q4)*std::cos(q3))/(std::pow(-((std::sin(q0)*std::sin(q1)*std::sin(q2) - std::sin(q0)*std::cos(q1)*std::cos(q2))*std::cos(q3) - std::sin(q3)*std::cos(q0))*std::sin(q4) + (std::sin(q0)*std::sin(q1)*std::cos(q2) + std::sin(q0)*std::sin(q2)*std::cos(q1))*std::cos(q4), 2) + std::pow(-((-std::sin(q1)*std::sin(q2)*std::cos(q0) + std::cos(q0)*std::cos(q1)*std::cos(q2))*std::cos(q3) - std::sin(q0)*std::sin(q3))*std::sin(q4) + (-std::sin(q1)*std::cos(q0)*std::cos(q2) - std::sin(q2)*std::cos(q0)*std::cos(q1))*std::cos(q4), 2));
J(3,3) = (-((std::sin(q0)*std::sin(q1)*std::sin(q2) - std::sin(q0)*std::cos(q1)*std::cos(q2))*std::cos(q3) - std::sin(q3)*std::cos(q0))*std::sin(q4) + (std::sin(q0)*std::sin(q1)*std::cos(q2) + std::sin(q0)*std::sin(q2)*std::cos(q1))*std::cos(q4))*((-std::sin(q1)*std::sin(q2)*std::cos(q0) + std::cos(q0)*std::cos(q1)*std::cos(q2))*std::sin(q3) + std::sin(q0)*std::cos(q3))*std::sin(q4)/(std::pow(-((std::sin(q0)*std::sin(q1)*std::sin(q2) - std::sin(q0)*std::cos(q1)*std::cos(q2))*std::cos(q3) - std::sin(q3)*std::cos(q0))*std::sin(q4) + (std::sin(q0)*std::sin(q1)*std::cos(q2) + std::sin(q0)*std::sin(q2)*std::cos(q1))*std::cos(q4), 2) + std::pow(-((-std::sin(q1)*std::sin(q2)*std::cos(q0) + std::cos(q0)*std::cos(q1)*std::cos(q2))*std::cos(q3) - std::sin(q0)*std::sin(q3))*std::sin(q4) + (-std::sin(q1)*std::cos(q0)*std::cos(q2) - std::sin(q2)*std::cos(q0)*std::cos(q1))*std::cos(q4), 2)) + (((-std::sin(q1)*std::sin(q2)*std::cos(q0) + std::cos(q0)*std::cos(q1)*std::cos(q2))*std::cos(q3) - std::sin(q0)*std::sin(q3))*std::sin(q4) - (-std::sin(q1)*std::cos(q0)*std::cos(q2) - std::sin(q2)*std::cos(q0)*std::cos(q1))*std::cos(q4))*((std::sin(q0)*std::sin(q1)*std::sin(q2) - std::sin(q0)*std::cos(q1)*std::cos(q2))*std::sin(q3) + std::cos(q0)*std::cos(q3))*std::sin(q4)/(std::pow(-((std::sin(q0)*std::sin(q1)*std::sin(q2) - std::sin(q0)*std::cos(q1)*std::cos(q2))*std::cos(q3) - std::sin(q3)*std::cos(q0))*std::sin(q4) + (std::sin(q0)*std::sin(q1)*std::cos(q2) + std::sin(q0)*std::sin(q2)*std::cos(q1))*std::cos(q4), 2) + std::pow(-((-std::sin(q1)*std::sin(q2)*std::cos(q0) + std::cos(q0)*std::cos(q1)*std::cos(q2))*std::cos(q3) - std::sin(q0)*std::sin(q3))*std::sin(q4) + (-std::sin(q1)*std::cos(q0)*std::cos(q2) - std::sin(q2)*std::cos(q0)*std::cos(q1))*std::cos(q4), 2));
J(3,4) = ((-(std::sin(q0)*std::sin(q1)*std::sin(q2) - std::sin(q0)*std::cos(q1)*std::cos(q2))*std::cos(q3) + std::sin(q3)*std::cos(q0))*std::cos(q4) - (std::sin(q0)*std::sin(q1)*std::cos(q2) + std::sin(q0)*std::sin(q2)*std::cos(q1))*std::sin(q4))*(((-std::sin(q1)*std::sin(q2)*std::cos(q0) + std::cos(q0)*std::cos(q1)*std::cos(q2))*std::cos(q3) - std::sin(q0)*std::sin(q3))*std::sin(q4) - (-std::sin(q1)*std::cos(q0)*std::cos(q2) - std::sin(q2)*std::cos(q0)*std::cos(q1))*std::cos(q4))/(std::pow(-((std::sin(q0)*std::sin(q1)*std::sin(q2) - std::sin(q0)*std::cos(q1)*std::cos(q2))*std::cos(q3) - std::sin(q3)*std::cos(q0))*std::sin(q4) + (std::sin(q0)*std::sin(q1)*std::cos(q2) + std::sin(q0)*std::sin(q2)*std::cos(q1))*std::cos(q4), 2) + std::pow(-((-std::sin(q1)*std::sin(q2)*std::cos(q0) + std::cos(q0)*std::cos(q1)*std::cos(q2))*std::cos(q3) - std::sin(q0)*std::sin(q3))*std::sin(q4) + (-std::sin(q1)*std::cos(q0)*std::cos(q2) - std::sin(q2)*std::cos(q0)*std::cos(q1))*std::cos(q4), 2)) + (-((std::sin(q0)*std::sin(q1)*std::sin(q2) - std::sin(q0)*std::cos(q1)*std::cos(q2))*std::cos(q3) - std::sin(q3)*std::cos(q0))*std::sin(q4) + (std::sin(q0)*std::sin(q1)*std::cos(q2) + std::sin(q0)*std::sin(q2)*std::cos(q1))*std::cos(q4))*((-(-std::sin(q1)*std::sin(q2)*std::cos(q0) + std::cos(q0)*std::cos(q1)*std::cos(q2))*std::cos(q3) + std::sin(q0)*std::sin(q3))*std::cos(q4) - (-std::sin(q1)*std::cos(q0)*std::cos(q2) - std::sin(q2)*std::cos(q0)*std::cos(q1))*std::sin(q4))/(std::pow(-((std::sin(q0)*std::sin(q1)*std::sin(q2) - std::sin(q0)*std::cos(q1)*std::cos(q2))*std::cos(q3) - std::sin(q3)*std::cos(q0))*std::sin(q4) + (std::sin(q0)*std::sin(q1)*std::cos(q2) + std::sin(q0)*std::sin(q2)*std::cos(q1))*std::cos(q4), 2) + std::pow(-((-std::sin(q1)*std::sin(q2)*std::cos(q0) + std::cos(q0)*std::cos(q1)*std::cos(q2))*std::cos(q3) - std::sin(q0)*std::sin(q3))*std::sin(q4) + (-std::sin(q1)*std::cos(q0)*std::cos(q2) - std::sin(q2)*std::cos(q0)*std::cos(q1))*std::cos(q4), 2));
J(3,5) = 0;
J(4,0) = 0;
J(4,1) = -((std::sin(q1)*std::sin(q2) - std::cos(q1)*std::cos(q2))*std::sin(q4)*std::cos(q3) + (-std::sin(q1)*std::cos(q2) - std::sin(q2)*std::cos(q1))*std::cos(q4))/std::sqrt(1 - std::pow((-std::sin(q1)*std::sin(q2) + std::cos(q1)*std::cos(q2))*std::cos(q4) - (std::sin(q1)*std::cos(q2) + std::sin(q2)*std::cos(q1))*std::sin(q4)*std::cos(q3), 2));
J(4,2) = -((std::sin(q1)*std::sin(q2) - std::cos(q1)*std::cos(q2))*std::sin(q4)*std::cos(q3) + (-std::sin(q1)*std::cos(q2) - std::sin(q2)*std::cos(q1))*std::cos(q4))/std::sqrt(1 - std::pow((-std::sin(q1)*std::sin(q2) + std::cos(q1)*std::cos(q2))*std::cos(q4) - (std::sin(q1)*std::cos(q2) + std::sin(q2)*std::cos(q1))*std::sin(q4)*std::cos(q3), 2));
J(4,3) = (-std::sin(q1)*std::cos(q2) - std::sin(q2)*std::cos(q1))*std::sin(q3)*std::sin(q4)/std::sqrt(1 - std::pow((-std::sin(q1)*std::sin(q2) + std::cos(q1)*std::cos(q2))*std::cos(q4) - (std::sin(q1)*std::cos(q2) + std::sin(q2)*std::cos(q1))*std::sin(q4)*std::cos(q3), 2));
J(4,4) = -(-(-std::sin(q1)*std::sin(q2) + std::cos(q1)*std::cos(q2))*std::sin(q4) + (-std::sin(q1)*std::cos(q2) - std::sin(q2)*std::cos(q1))*std::cos(q3)*std::cos(q4))/std::sqrt(1 - std::pow((-std::sin(q1)*std::sin(q2) + std::cos(q1)*std::cos(q2))*std::cos(q4) - (std::sin(q1)*std::cos(q2) + std::sin(q2)*std::cos(q1))*std::sin(q4)*std::cos(q3), 2));
J(4,5) = 0;
J(5,0) = 0;
J(5,1) = (-((-std::sin(q1)*std::sin(q2) + std::cos(q1)*std::cos(q2))*std::sin(q4) + (std::sin(q1)*std::cos(q2) + std::sin(q2)*std::cos(q1))*std::cos(q3)*std::cos(q4))*std::sin(q5) - (std::sin(q1)*std::cos(q2) + std::sin(q2)*std::cos(q1))*std::sin(q3)*std::cos(q5))*((std::sin(q1)*std::sin(q2) - std::cos(q1)*std::cos(q2))*std::sin(q3)*std::sin(q5) + ((-std::sin(q1)*std::sin(q2) + std::cos(q1)*std::cos(q2))*std::cos(q3)*std::cos(q4) + (-std::sin(q1)*std::cos(q2) - std::sin(q2)*std::cos(q1))*std::sin(q4))*std::cos(q5))/(std::pow(-((-std::sin(q1)*std::sin(q2) + std::cos(q1)*std::cos(q2))*std::sin(q4) + (std::sin(q1)*std::cos(q2) + std::sin(q2)*std::cos(q1))*std::cos(q3)*std::cos(q4))*std::sin(q5) - (std::sin(q1)*std::cos(q2) + std::sin(q2)*std::cos(q1))*std::sin(q3)*std::cos(q5), 2) + std::pow(((-std::sin(q1)*std::sin(q2) + std::cos(q1)*std::cos(q2))*std::sin(q4) + (std::sin(q1)*std::cos(q2) + std::sin(q2)*std::cos(q1))*std::cos(q3)*std::cos(q4))*std::cos(q5) - (std::sin(q1)*std::cos(q2) + std::sin(q2)*std::cos(q1))*std::sin(q3)*std::sin(q5), 2)) + (-((-std::sin(q1)*std::sin(q2) + std::cos(q1)*std::cos(q2))*std::sin(q4) + (std::sin(q1)*std::cos(q2) + std::sin(q2)*std::cos(q1))*std::cos(q3)*std::cos(q4))*std::cos(q5) + (std::sin(q1)*std::cos(q2) + std::sin(q2)*std::cos(q1))*std::sin(q3)*std::sin(q5))*((std::sin(q1)*std::sin(q2) - std::cos(q1)*std::cos(q2))*std::sin(q3)*std::cos(q5) + (-(-std::sin(q1)*std::sin(q2) + std::cos(q1)*std::cos(q2))*std::cos(q3)*std::cos(q4) - (-std::sin(q1)*std::cos(q2) - std::sin(q2)*std::cos(q1))*std::sin(q4))*std::sin(q5))/(std::pow(-((-std::sin(q1)*std::sin(q2) + std::cos(q1)*std::cos(q2))*std::sin(q4) + (std::sin(q1)*std::cos(q2) + std::sin(q2)*std::cos(q1))*std::cos(q3)*std::cos(q4))*std::sin(q5) - (std::sin(q1)*std::cos(q2) + std::sin(q2)*std::cos(q1))*std::sin(q3)*std::cos(q5), 2) + std::pow(((-std::sin(q1)*std::sin(q2) + std::cos(q1)*std::cos(q2))*std::sin(q4) + (std::sin(q1)*std::cos(q2) + std::sin(q2)*std::cos(q1))*std::cos(q3)*std::cos(q4))*std::cos(q5) - (std::sin(q1)*std::cos(q2) + std::sin(q2)*std::cos(q1))*std::sin(q3)*std::sin(q5), 2));
J(5,2) = (-((-std::sin(q1)*std::sin(q2) + std::cos(q1)*std::cos(q2))*std::sin(q4) + (std::sin(q1)*std::cos(q2) + std::sin(q2)*std::cos(q1))*std::cos(q3)*std::cos(q4))*std::sin(q5) - (std::sin(q1)*std::cos(q2) + std::sin(q2)*std::cos(q1))*std::sin(q3)*std::cos(q5))*((std::sin(q1)*std::sin(q2) - std::cos(q1)*std::cos(q2))*std::sin(q3)*std::sin(q5) + ((-std::sin(q1)*std::sin(q2) + std::cos(q1)*std::cos(q2))*std::cos(q3)*std::cos(q4) + (-std::sin(q1)*std::cos(q2) - std::sin(q2)*std::cos(q1))*std::sin(q4))*std::cos(q5))/(std::pow(-((-std::sin(q1)*std::sin(q2) + std::cos(q1)*std::cos(q2))*std::sin(q4) + (std::sin(q1)*std::cos(q2) + std::sin(q2)*std::cos(q1))*std::cos(q3)*std::cos(q4))*std::sin(q5) - (std::sin(q1)*std::cos(q2) + std::sin(q2)*std::cos(q1))*std::sin(q3)*std::cos(q5), 2) + std::pow(((-std::sin(q1)*std::sin(q2) + std::cos(q1)*std::cos(q2))*std::sin(q4) + (std::sin(q1)*std::cos(q2) + std::sin(q2)*std::cos(q1))*std::cos(q3)*std::cos(q4))*std::cos(q5) - (std::sin(q1)*std::cos(q2) + std::sin(q2)*std::cos(q1))*std::sin(q3)*std::sin(q5), 2)) + (-((-std::sin(q1)*std::sin(q2) + std::cos(q1)*std::cos(q2))*std::sin(q4) + (std::sin(q1)*std::cos(q2) + std::sin(q2)*std::cos(q1))*std::cos(q3)*std::cos(q4))*std::cos(q5) + (std::sin(q1)*std::cos(q2) + std::sin(q2)*std::cos(q1))*std::sin(q3)*std::sin(q5))*((std::sin(q1)*std::sin(q2) - std::cos(q1)*std::cos(q2))*std::sin(q3)*std::cos(q5) + (-(-std::sin(q1)*std::sin(q2) + std::cos(q1)*std::cos(q2))*std::cos(q3)*std::cos(q4) - (-std::sin(q1)*std::cos(q2) - std::sin(q2)*std::cos(q1))*std::sin(q4))*std::sin(q5))/(std::pow(-((-std::sin(q1)*std::sin(q2) + std::cos(q1)*std::cos(q2))*std::sin(q4) + (std::sin(q1)*std::cos(q2) + std::sin(q2)*std::cos(q1))*std::cos(q3)*std::cos(q4))*std::sin(q5) - (std::sin(q1)*std::cos(q2) + std::sin(q2)*std::cos(q1))*std::sin(q3)*std::cos(q5), 2) + std::pow(((-std::sin(q1)*std::sin(q2) + std::cos(q1)*std::cos(q2))*std::sin(q4) + (std::sin(q1)*std::cos(q2) + std::sin(q2)*std::cos(q1))*std::cos(q3)*std::cos(q4))*std::cos(q5) - (std::sin(q1)*std::cos(q2) + std::sin(q2)*std::cos(q1))*std::sin(q3)*std::sin(q5), 2));
J(5,3) = (-((-std::sin(q1)*std::sin(q2) + std::cos(q1)*std::cos(q2))*std::sin(q4) + (std::sin(q1)*std::cos(q2) + std::sin(q2)*std::cos(q1))*std::cos(q3)*std::cos(q4))*std::sin(q5) - (std::sin(q1)*std::cos(q2) + std::sin(q2)*std::cos(q1))*std::sin(q3)*std::cos(q5))*((-std::sin(q1)*std::cos(q2) - std::sin(q2)*std::cos(q1))*std::sin(q5)*std::cos(q3) - (std::sin(q1)*std::cos(q2) + std::sin(q2)*std::cos(q1))*std::sin(q3)*std::cos(q4)*std::cos(q5))/(std::pow(-((-std::sin(q1)*std::sin(q2) + std::cos(q1)*std::cos(q2))*std::sin(q4) + (std::sin(q1)*std::cos(q2) + std::sin(q2)*std::cos(q1))*std::cos(q3)*std::cos(q4))*std::sin(q5) - (std::sin(q1)*std::cos(q2) + std::sin(q2)*std::cos(q1))*std::sin(q3)*std::cos(q5), 2) + std::pow(((-std::sin(q1)*std::sin(q2) + std::cos(q1)*std::cos(q2))*std::sin(q4) + (std::sin(q1)*std::cos(q2) + std::sin(q2)*std::cos(q1))*std::cos(q3)*std::cos(q4))*std::cos(q5) - (std::sin(q1)*std::cos(q2) + std::sin(q2)*std::cos(q1))*std::sin(q3)*std::sin(q5), 2)) + (-((-std::sin(q1)*std::sin(q2) + std::cos(q1)*std::cos(q2))*std::sin(q4) + (std::sin(q1)*std::cos(q2) + std::sin(q2)*std::cos(q1))*std::cos(q3)*std::cos(q4))*std::cos(q5) + (std::sin(q1)*std::cos(q2) + std::sin(q2)*std::cos(q1))*std::sin(q3)*std::sin(q5))*((-std::sin(q1)*std::cos(q2) - std::sin(q2)*std::cos(q1))*std::cos(q3)*std::cos(q5) + (std::sin(q1)*std::cos(q2) + std::sin(q2)*std::cos(q1))*std::sin(q3)*std::sin(q5)*std::cos(q4))/(std::pow(-((-std::sin(q1)*std::sin(q2) + std::cos(q1)*std::cos(q2))*std::sin(q4) + (std::sin(q1)*std::cos(q2) + std::sin(q2)*std::cos(q1))*std::cos(q3)*std::cos(q4))*std::sin(q5) - (std::sin(q1)*std::cos(q2) + std::sin(q2)*std::cos(q1))*std::sin(q3)*std::cos(q5), 2) + std::pow(((-std::sin(q1)*std::sin(q2) + std::cos(q1)*std::cos(q2))*std::sin(q4) + (std::sin(q1)*std::cos(q2) + std::sin(q2)*std::cos(q1))*std::cos(q3)*std::cos(q4))*std::cos(q5) - (std::sin(q1)*std::cos(q2) + std::sin(q2)*std::cos(q1))*std::sin(q3)*std::sin(q5), 2));
J(5,4) = (-((-std::sin(q1)*std::sin(q2) + std::cos(q1)*std::cos(q2))*std::sin(q4) + (std::sin(q1)*std::cos(q2) + std::sin(q2)*std::cos(q1))*std::cos(q3)*std::cos(q4))*std::sin(q5) - (std::sin(q1)*std::cos(q2) + std::sin(q2)*std::cos(q1))*std::sin(q3)*std::cos(q5))*((-std::sin(q1)*std::sin(q2) + std::cos(q1)*std::cos(q2))*std::cos(q4) - (std::sin(q1)*std::cos(q2) + std::sin(q2)*std::cos(q1))*std::sin(q4)*std::cos(q3))*std::cos(q5)/(std::pow(-((-std::sin(q1)*std::sin(q2) + std::cos(q1)*std::cos(q2))*std::sin(q4) + (std::sin(q1)*std::cos(q2) + std::sin(q2)*std::cos(q1))*std::cos(q3)*std::cos(q4))*std::sin(q5) - (std::sin(q1)*std::cos(q2) + std::sin(q2)*std::cos(q1))*std::sin(q3)*std::cos(q5), 2) + std::pow(((-std::sin(q1)*std::sin(q2) + std::cos(q1)*std::cos(q2))*std::sin(q4) + (std::sin(q1)*std::cos(q2) + std::sin(q2)*std::cos(q1))*std::cos(q3)*std::cos(q4))*std::cos(q5) - (std::sin(q1)*std::cos(q2) + std::sin(q2)*std::cos(q1))*std::sin(q3)*std::sin(q5), 2)) + (-((-std::sin(q1)*std::sin(q2) + std::cos(q1)*std::cos(q2))*std::sin(q4) + (std::sin(q1)*std::cos(q2) + std::sin(q2)*std::cos(q1))*std::cos(q3)*std::cos(q4))*std::cos(q5) + (std::sin(q1)*std::cos(q2) + std::sin(q2)*std::cos(q1))*std::sin(q3)*std::sin(q5))*(-(-std::sin(q1)*std::sin(q2) + std::cos(q1)*std::cos(q2))*std::cos(q4) + (std::sin(q1)*std::cos(q2) + std::sin(q2)*std::cos(q1))*std::sin(q4)*std::cos(q3))*std::sin(q5)/(std::pow(-((-std::sin(q1)*std::sin(q2) + std::cos(q1)*std::cos(q2))*std::sin(q4) + (std::sin(q1)*std::cos(q2) + std::sin(q2)*std::cos(q1))*std::cos(q3)*std::cos(q4))*std::sin(q5) - (std::sin(q1)*std::cos(q2) + std::sin(q2)*std::cos(q1))*std::sin(q3)*std::cos(q5), 2) + std::pow(((-std::sin(q1)*std::sin(q2) + std::cos(q1)*std::cos(q2))*std::sin(q4) + (std::sin(q1)*std::cos(q2) + std::sin(q2)*std::cos(q1))*std::cos(q3)*std::cos(q4))*std::cos(q5) - (std::sin(q1)*std::cos(q2) + std::sin(q2)*std::cos(q1))*std::sin(q3)*std::sin(q5), 2));
J(5,5) = ((-(-std::sin(q1)*std::sin(q2) + std::cos(q1)*std::cos(q2))*std::sin(q4) - (std::sin(q1)*std::cos(q2) + std::sin(q2)*std::cos(q1))*std::cos(q3)*std::cos(q4))*std::cos(q5) - (-std::sin(q1)*std::cos(q2) - std::sin(q2)*std::cos(q1))*std::sin(q3)*std::sin(q5))*(-((-std::sin(q1)*std::sin(q2) + std::cos(q1)*std::cos(q2))*std::sin(q4) + (std::sin(q1)*std::cos(q2) + std::sin(q2)*std::cos(q1))*std::cos(q3)*std::cos(q4))*std::cos(q5) + (std::sin(q1)*std::cos(q2) + std::sin(q2)*std::cos(q1))*std::sin(q3)*std::sin(q5))/(std::pow(-((-std::sin(q1)*std::sin(q2) + std::cos(q1)*std::cos(q2))*std::sin(q4) + (std::sin(q1)*std::cos(q2) + std::sin(q2)*std::cos(q1))*std::cos(q3)*std::cos(q4))*std::sin(q5) - (std::sin(q1)*std::cos(q2) + std::sin(q2)*std::cos(q1))*std::sin(q3)*std::cos(q5), 2) + std::pow(((-std::sin(q1)*std::sin(q2) + std::cos(q1)*std::cos(q2))*std::sin(q4) + (std::sin(q1)*std::cos(q2) + std::sin(q2)*std::cos(q1))*std::cos(q3)*std::cos(q4))*std::cos(q5) - (std::sin(q1)*std::cos(q2) + std::sin(q2)*std::cos(q1))*std::sin(q3)*std::sin(q5), 2)) + (-((-std::sin(q1)*std::sin(q2) + std::cos(q1)*std::cos(q2))*std::sin(q4) + (std::sin(q1)*std::cos(q2) + std::sin(q2)*std::cos(q1))*std::cos(q3)*std::cos(q4))*std::sin(q5) + (-std::sin(q1)*std::cos(q2) - std::sin(q2)*std::cos(q1))*std::sin(q3)*std::cos(q5))*(-((-std::sin(q1)*std::sin(q2) + std::cos(q1)*std::cos(q2))*std::sin(q4) + (std::sin(q1)*std::cos(q2) + std::sin(q2)*std::cos(q1))*std::cos(q3)*std::cos(q4))*std::sin(q5) - (std::sin(q1)*std::cos(q2) + std::sin(q2)*std::cos(q1))*std::sin(q3)*std::cos(q5))/(std::pow(-((-std::sin(q1)*std::sin(q2) + std::cos(q1)*std::cos(q2))*std::sin(q4) + (std::sin(q1)*std::cos(q2) + std::sin(q2)*std::cos(q1))*std::cos(q3)*std::cos(q4))*std::sin(q5) - (std::sin(q1)*std::cos(q2) + std::sin(q2)*std::cos(q1))*std::sin(q3)*std::cos(q5), 2) + std::pow(((-std::sin(q1)*std::sin(q2) + std::cos(q1)*std::cos(q2))*std::sin(q4) + (std::sin(q1)*std::cos(q2) + std::sin(q2)*std::cos(q1))*std::cos(q3)*std::cos(q4))*std::cos(q5) - (std::sin(q1)*std::cos(q2) + std::sin(q2)*std::cos(q1))*std::sin(q3)*std::sin(q5), 2));

    return J;
}

