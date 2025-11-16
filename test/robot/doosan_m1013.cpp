#include "common_types.h"
#include "imgui.h"
#include "implot3d.h"
#include "transform.h"

#include <iostream>
#include <cmath>




#include <vector>
#include <string>
#include <stdexcept>
#include <iostream>









// 두산 M1013 링크 길이
const double l1 = 0.135;    // [m]
const double l2 = 0.1702;   // [m]
const double l3 = 0.411;    // [m]
const double l4 = 0.164;    // [m]
const double l5 = 0.368;    // [m]
const double l6 = 0.1522;   // [m]
const double l7 = 0.146;    // [m]
const double l8 = 0.121;    // [m]








// 두산 M1013 Jacobian 함수 [rad]
transform::mat<12, 6> Jcobian(float q0, float q1, float q2, float q3, float q4, float q5)
{
    static transform::mat<12, 6> J = transform::mat<12, 6>::Zero();

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




// Manipulability Ellipsoid 그리기
void draw_manipulabilityEllipsoid(const transform::mat<3, 6>& J_position,
                                 const transform::vec3& center,
                                 float scale = 1.0f,
                                 const ImVec4& color = ImVec4(0.0f, 1.0f, 1.0f, 0.5f))
{
    // J * J^T 계산 (3x3 행렬)
    Eigen::Matrix3d M = J_position * J_position.transpose();


    // 고유값 분해
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(M);
    if (eigensolver.info() != Eigen::Success) {
        return;
    }


    Eigen::Vector3d eigenvalues = eigensolver.eigenvalues();
    Eigen::Matrix3d eigenvectors = eigensolver.eigenvectors();


    // 타원체의 반지름 (고유값의 제곱근)
    Eigen::Vector3d radii = eigenvalues.cwiseSqrt() * scale;


    // 구 생성 (위도, 경도)
    const int lat_segments = 16;
    const int lon_segments = 16;
    std::vector<float> sphere_x, sphere_y, sphere_z;


    for (int i = 0; i <= lat_segments; ++i) {
        float theta = M_PI * i / lat_segments; // 0 to π
        float sin_theta = std::sin(theta);
        float cos_theta = std::cos(theta);

        for (int j = 0; j <= lon_segments; ++j) {
            float phi = 2.0f * M_PI * j / lon_segments; // 0 to 2π
            float sin_phi = std::sin(phi);
            float cos_phi = std::cos(phi);

            // 단위 구 좌표
            Eigen::Vector3d unit_sphere(sin_theta * cos_phi,
                                        sin_theta * sin_phi,
                                        cos_theta);

            // 타원체로 변환: 고유벡터로 회전 + 고유값으로 스케일
            Eigen::Vector3d ellipsoid_point = eigenvectors * (radii.asDiagonal() * unit_sphere);

            // 중심점으로 이동
            ellipsoid_point += center;

            sphere_x.push_back(static_cast<float>(ellipsoid_point.x()));
            sphere_y.push_back(static_cast<float>(ellipsoid_point.y()));
            sphere_z.push_back(static_cast<float>(ellipsoid_point.z()));
        }
    }


    // 와이어프레임으로 그리기
    ImPlot3D::PushStyleColor(ImPlot3DCol_Line, color);

    // 레전드 끄기



    // 위도선 그리기
    for (int i = 0; i <= lat_segments; ++i) {
        std::vector<float> line_x, line_y, line_z;
        for (int j = 0; j <= lon_segments; ++j) {
            int idx = i * (lon_segments + 1) + j;
            line_x.push_back(sphere_x[idx]);
            line_y.push_back(sphere_y[idx]);
            line_z.push_back(sphere_z[idx]);
        }
        ImPlot3D::PlotLine("Manipulability", line_x.data(), line_y.data(), line_z.data(), line_x.size());
    }


    // 경도선 그리기
    for (int j = 0; j <= lon_segments; ++j) {
        std::vector<float> line_x, line_y, line_z;
        for (int i = 0; i <= lat_segments; ++i) {
            int idx = i * (lon_segments + 1) + j;
            line_x.push_back(sphere_x[idx]);
            line_y.push_back(sphere_y[idx]);
            line_z.push_back(sphere_z[idx]);
        }
        ImPlot3D::PlotLine("Manipulability", line_x.data(), line_y.data(), line_z.data(), line_x.size());
    }

    ImPlot3D::PopStyleColor();
}



int main() {
    std::cout << "두산 로봇 M1013 IK 테스트 실행..." << std::endl;

// --------------------------------------
// 3D 축 객체
// --------------------------------------
    AxisObject origine(0.1);
    AxisObject base(0.05);
    AxisObject joint1(0.05);
    AxisObject joint2(0.05);
    AxisObject joint3(0.05);
    AxisObject joint4(0.05);
    AxisObject joint5(0.05);
    AxisObject joint6(0.05);
    AxisObject endEffector(0.06);


// --------------------------------------
// 로봇 링크 TF
// --------------------------------------
    transform base_tf;
    transform joint1_tf;
    transform joint2_tf;
    transform joint3_tf;
    transform joint4_tf;
    transform joint5_tf;
    transform joint6_tf;
    transform endEffector_tf;


// --------------------------------------
// Target Transform 생성
// --------------------------------------
    transform::vec3 tPos(0.62, 0.0, 0.235); // [m]
    transform::quat qRot(transform::AngleAxis(DEG_TO_RAD(0), transform::vec3(0, 1, 0)));//

    transform target_tf(qRot, tPos);


// --------------------------------------
// 로봇 Joint, Target TF 설정
// --------------------------------------
    float q0 = -1.0;
    float q1 = -1.0;
    float q2 = -1.0;
    float q3 = -1.0;
    float q4 = -1.0;
    float q5 = -1.0;

    float target_x      = target_tf.x();
    float target_y      = target_tf.y();
    float target_z      = target_tf.z();
    float target_roll   = RAD_TO_DEG(target_tf.roll());
    float target_pitch  = RAD_TO_DEG(target_tf.pitch());
    float target_yaw    = RAD_TO_DEG(target_tf.yaw());




    // 관절 한계 설정
    JointLimits limits(6);
    limits.setLimit(0, DEG_TO_RAD(-360), DEG_TO_RAD(360));
    limits.setLimit(1, DEG_TO_RAD(-360),  DEG_TO_RAD(360));
    limits.setLimit(2, DEG_TO_RAD(-150), DEG_TO_RAD(150));
    limits.setLimit(3, DEG_TO_RAD(-360), DEG_TO_RAD(360));
    limits.setLimit(4, DEG_TO_RAD(-360), DEG_TO_RAD(360));
    limits.setLimit(5, DEG_TO_RAD(-360), DEG_TO_RAD(360));


    int method_idx = 3;  // 0=없음, 1=단순클램핑, 2=부드러운클램핑, 3=반발력, 4=조합(권장)
    float activation_ratio = 0.02f;
    float repulsive_gain = 0.005f;

    const char* method_names[] = {
        "없음 (No Constraint)",
        "단순 클램핑 (Simple Clamp)",
        "부드러운 클램핑 (Soft Clamp)",
        "반발력 (Repulsive)",
        "조합 (Soft+Repulsive, 권장)"
    };



    ImGui::start("데모");

    while (ImGui::isRunning())
    {
        ImGui::draw([&]()
        {

            ImGui::Begin("Draw");
            ImGui::PushItemWidth(ImGui::GetContentRegionAvail().x);


// --------------------------------------
//          | Joint 각도 조절 슬라이더
// --------------------------------------
            ImGui::Text("Joint 설정");
            ImGui::DragFloat("##joint1", &q0, 0.1f, -360.0f, 360.0f, "J1: %.3f deg");
            ImGui::DragFloat("##joint2", &q1, 0.1f, -360.0f, 360.0f, "J2: %.3f deg");
            ImGui::DragFloat("##joint3", &q2, 0.1f, -150.0f, 150.0f, "J3: %.3f deg");
            ImGui::DragFloat("##joint4", &q3, 0.1f, -360.0f, 360.0f, "J4: %.3f deg");
            ImGui::DragFloat("##joint5", &q4, 0.1f, -360.0f, 360.0f, "J5: %.3f deg");
            ImGui::DragFloat("##joint6", &q5, 0.1f, -360.0f, 360.0f, "J6: %.3f deg");

            ImGui::Dummy(ImVec2(0, 20));


// --------------------------------------
//          | Target Transform 설정 슬라이더
// --------------------------------------
            ImGui::Text("Target Transform 설정");
            ImGui::DragFloat("##Target x", &target_x, 0.01f, -1.0f, 1.0f, "x: %.3f");
            ImGui::DragFloat("##Target y", &target_y, 0.01f, -1.0f, 1.0f, "y: %.3f");
            ImGui::DragFloat("##Target z", &target_z, 0.001f, -1.0f, 2.0f, "z: %.3f");

            ImGui::Dummy(ImVec2(0, 10));

            ImGui::DragFloat("##Target Roll", &target_roll, 1.0f, -180.0f, 180.0f, "Roll: %.3f");
            ImGui::DragFloat("##Target Pitch", &target_pitch, 1.0f, -130.0f, 130.0f, "Pitch: %.3f");
            ImGui::DragFloat("##Target Yaw", &target_yaw, 1.0f, -180.0f, 180.0f, "Yaw: %.3f");

            ImGui::PopItemWidth();


            ImGui::Text("관절 한계 제약 방법 (검증됨)");
           ImGui::Combo("##method", &method_idx, method_names, 5);

           if (method_idx >= 2) {
               ImGui::DragFloat("##activation", &activation_ratio, 0.001f, 0.001f, 0.2f, "활성화 비율: %.3f");
           }
           if (method_idx == 3 || method_idx == 4) {
               ImGui::DragFloat("##repulsive", &repulsive_gain, 0.001f, 0.001f, 0.2f, "반발력 게인: %.3f");
           }



// --------------------------------------
//          | Target Transform 계산
//          | ZYZ Euler 회전 사용
// --------------------------------------
            transform::quat target_qRot(transform::AngleAxis(DEG_TO_RAD(target_yaw), transform::vec3::UnitZ()) *        // Z
                                        transform::AngleAxis(DEG_TO_RAD(target_pitch + 90), transform::vec3::UnitY()) * // Y ( +90 ) 주의
                                        transform::AngleAxis(DEG_TO_RAD(target_roll), transform::vec3::UnitZ()));       // Z
            target_tf = transform(target_qRot, transform::vec3(target_x, target_y, target_z));




// --------------------------------------
//          | 각 Joint TF 계산
// --------------------------------------
            // Joint1: base 좌표계 기준
            transform::vec3 pos1(0, 0, l1);
            transform::quat rot1(transform::AngleAxis(DEG_TO_RAD(q0), transform::vec3::UnitZ()));
            transform tf1(rot1, pos1);

            // Joint2: Joint1 좌표계 기준
            transform::vec3 pos2(-l2, 0, 0);
            transform::quat rot2(transform::AngleAxis(DEG_TO_RAD(q1), transform::vec3::UnitX()));
            transform tf2(rot2, pos2);

            // Joint3: Joint2 좌표계 기준
            transform::vec3 pos3(0, 0, l3);
            transform::quat rot3(transform::AngleAxis(DEG_TO_RAD(q2), transform::vec3::UnitX()));
            transform tf3(rot3, pos3);

            // Joint4: Joint3 좌표계 기준
            transform::vec3 pos4(l4, 0, 0);
            transform::quat rot4(transform::AngleAxis(DEG_TO_RAD(q3), transform::vec3::UnitZ()));
            transform tf4(rot4, pos4);

            // Joint5: Joint4 좌표계 기준
            transform::vec3 pos5(-l6, 0, l5);
            transform::quat rot5(transform::AngleAxis(DEG_TO_RAD(q4), transform::vec3::UnitX()));
            transform tf5(rot5, pos5);

            // Joint6: Joint5 좌표계 기준
            transform::vec3 pos6(l7, 0, 0);
            transform::quat rot6(transform::AngleAxis(DEG_TO_RAD(q5), transform::vec3::UnitZ()));
            transform tf6(rot6, pos6);

            // EndEffector: Joint6 좌표계 기준
            transform::vec3 pos_ee(0, 0, l8);
            transform::quat rot_ee(transform::AngleAxis(DEG_TO_RAD(0), transform::vec3::UnitZ()));
            transform tf_ee(rot_ee, pos_ee);



// --------------------------------------
//          | FK ( Forward Kinematics )
// --------------------------------------
            transform final_tf1   = tf1;
            transform final_tf2   = final_tf1 * tf2;
            transform final_tf3   = final_tf2 * tf3;
            transform final_tf4   = final_tf3 * tf4;
            transform final_tf5   = final_tf4 * tf5;
            transform final_tf6   = final_tf5 * tf6;
            transform final_tf_ee = final_tf6 * tf_ee;

            transform fk = final_tf_ee;

            // AxisObject에 최종 변환 적용
            joint1.setTransform(final_tf1);
            joint2.setTransform(final_tf2);
            joint3.setTransform(final_tf3);
            joint4.setTransform(final_tf4);
            joint5.setTransform(final_tf5);
            joint6.setTransform(final_tf6);
            endEffector.setTransform(final_tf_ee);




// --------------------------------------
//          | IK ( Inverse Kinematics )
// --------------------------------------

            // Transform 오차 계산
            transform error = target_tf - fk;
            transform::vec<12> dx;
            dx <<   error(0, 3),
                    error(1, 3),
                    error(2, 3),
                    error(0, 0),
                    error(0, 1),
                    error(0, 2),
                    error(1, 0),
                    error(1, 1),
                    error(1, 2),
                    error(2, 0),
                    error(2, 1),
                    error(2, 2);


            // Jacobian
            transform::mat<12,6> j = Jcobian(DEG_TO_RAD(q0),
                                             DEG_TO_RAD(q1),
                                             DEG_TO_RAD(q2),
                                             DEG_TO_RAD(q3),
                                             DEG_TO_RAD(q4),
                                             DEG_TO_RAD(q5));

            transform::mat<6, 12> j_inv = jInv_SVD_Damped(j, 0.1);



            // 각도 업데이트
            transform::vec<6> dq;
            dq = j_inv * dx * 0.1;


            transform::vec<6> q_current;
            q_current << DEG_TO_RAD(q0), DEG_TO_RAD(q1), DEG_TO_RAD(q2),
                        DEG_TO_RAD(q3), DEG_TO_RAD(q4), DEG_TO_RAD(q5);

            JointLimitMethod method = static_cast<JointLimitMethod>(method_idx);
            applyJointLimitConstraint(dq, q_current, limits, method, activation_ratio, repulsive_gain);


            q0 +=RAD_TO_DEG(dq(0));
            q1 +=RAD_TO_DEG(dq(1));
            q2 +=RAD_TO_DEG(dq(2));
            q3 +=RAD_TO_DEG(dq(3));
            q4 +=RAD_TO_DEG(dq(4));
            q5 +=RAD_TO_DEG(dq(5));



            q0 = NORM_DEG_180(q0);
            q1 = NORM_DEG_180(q1);
            q2 = NORM_DEG_180(q2);
            q3 = NORM_DEG_180(q3);
            q4 = NORM_DEG_180(q4);
            q5 = NORM_DEG_180(q5);



            float x, y, z;
            x = fk.translation().x();
            y = fk.translation().y();
            z = fk.translation().z();
            ImGui::Text("x: %.3f, y: %.3f, z: %.3f", x, y, z);

            float roll = fk.roll();
            float pitch = fk.pitch();
            float yaw = fk.yaw();
            ImGui::Text("roll: %.3f, pitch: %.3f, yaw: %.3f", RAD_TO_DEG(roll), RAD_TO_DEG(pitch), RAD_TO_DEG(yaw));


            ImGui::End();


            ImGui::Begin("시각화");
            if (ImPlot3D::BeginPlot("로봇 FK/IK 시각화", ImVec2(-1, -1), ImPlot3DFlags_Equal | ImPlot3DFlags_NoLegend))
            {
                ImPlot3D::SetupAxesLimits(-1.3, 1.3, -1.3, 1.3, 0.0, 1.3, ImPlot3DCond_Always);
                ImPlot3D::SetupAxes("X", "Y", "Z");

                // 축 그리기
                origine.Draw();
                base.Draw();
                joint1.Draw();
                joint2.Draw();
                joint3.Draw();
                joint4.Draw();
                joint5.Draw();
                joint6.Draw();
                endEffector.Draw();

                // 링크선 그리기 (노란색)
                DrawLinkLine(base, joint1);
                DrawLinkLine(joint1, joint2);
                DrawLinkLine(joint2, joint3);
                DrawLinkLine(joint3, joint4);
                DrawLinkLine(joint4, joint5);
                DrawLinkLine(joint5, joint6);
                DrawLinkLine(joint6, endEffector);


                // Manipulability ellipsoid 그리기
                transform::mat<3, 6> j_position_final = j.block<3, 6>(0, 0);
                draw_manipulabilityEllipsoid(j_position_final, fk.translation(), 0.3f, ImVec4(0.0f, 1.0f, 1.0f, 0.1f));


                ImPlot3D::EndPlot();

                ImGui::End();
            }

        });
    }
    ImGui::stop();


    return 0;
}
