#include <chrono>
#include <iostream>
#include <cmath>
#include <iomanip>
#include <optional>
#include <vector>
#include <string>
#include <array>
#include <algorithm>
#include <thread>

// 사용자 환경 헤더
#include "common_types.h"
#include "imgui.h"
#include "implot3d.h"
#include "robot/transform.h"
#include "robot/jacobian_inverse.h"
#include "robot/imgui_draw_manipulability.hpp"




// ----------------------------------------------------------------------------------
// 전역 변수 및 시각화 객체
// ----------------------------------------------------------------------------------
AxisObject origine(0.1);
AxisObject base(0.05);
AxisObject joint1(0.05);
AxisObject joint2(0.05);
AxisObject joint3(0.05);
AxisObject joint4(0.05);
AxisObject joint5(0.05);
AxisObject joint6(0.05);
AxisObject t_tip(0.06);
AxisObject t_ee(0.1);

// 전방 선언
void draw_simulation(const mat<6, 6>& j, const Transform& fk, const vec3& ws_min, const vec3& ws_max);
void draw_box(const vec3& min_pos, const vec3& max_pos);


// ----------------------------------------------------------------------------------
// 로봇 클래스 M1013 (QP Solver 통합)
// ----------------------------------------------------------------------------------
class M1013 {
    struct JointLimits {
        double min; // [rad]
        double max; // [rad]
    };

    // 링크 길이
    const double l1 = 0.1525; // [m]
    const double l2 = 0.1985; // [m]
    const double l3 = 0.620;  // [m]
    const double l4 = 0.164;  // [m]
    const double l5 = 0.559;  // [m]
    const double l6 = 0.146;  // [m]
    const double l7 = 0.146;  // [m]
    const double l8 = 0.121;  // [m]

    // Ik 설정
    const double ERROR_IK_TGREASHOLD    = 0.001;    // IK 오차 정밀도
    const int MAX_IK_ITER               = 500;      // Ik 최대 반복 횟수
    const double STEP_SIZE              = 0.001;    // Alpha
    const double WORKSPACE_LIMIT_MARGIN = 0.1;      // 감속 시작 거리 [m]
    const double LAMBDA                 = 0.1;      // Damping Factor

    // 관절 범위 제한 설정
    const double JOINT_STEP_LIMIT = 1.0;            // 한 스텝에 최대 각도 움직임 [rad]
    const double JOINT_LIMIT_MARGIN = 0.00;         // 제한 각도의 마진

    // QP Solver 설정
    const int MAX_QP_ITER               = 500;      // QP 최대 반복 횟수
    const double ERROR_QP_THREASHOLD    = 0.001;    // QP 오차 정밀도



    // 로봇 상태 변수
    std::array<Transform, 9> tf;                // 각 링크 Transform
    std::optional<Transform> tf_ee;             // EE Offset


public:
    vec<6> q_rad;                               // 현재 관절 각도 [rad]
    mat<6, 6> J;                                // Jacobian

    // 작업 영역
    vec3 workspace_min;                         // 작업 영역 최소값 [m]
    vec3 workspace_max;                         // 작업 영역 최대값 [m]

    // 관절 제한
    std::array<JointLimits, 6> joint_limits;    // 관절 제한




public:
    enum TF_Name_ {
        TF_Base,
        TF_Joint1,
        TF_Joint2,
        TF_Joint3,
        TF_Joint4,
        TF_Joint5,
        TF_Joint6,
        TF_Tip,
        TF_End_Effector
    };


    M1013()
    {

        // 1. 관절 제한 설정
        joint_limits[0] = {DEG_TO_RAD(-360.0), DEG_TO_RAD(360.0)};
        joint_limits[1] = {DEG_TO_RAD(-360.0), DEG_TO_RAD(360.0)};
        joint_limits[2] = {DEG_TO_RAD(10.0),   DEG_TO_RAD(160.0)};
        joint_limits[3] = {DEG_TO_RAD(-360.0), DEG_TO_RAD(360.0)};
        joint_limits[4] = {DEG_TO_RAD(-360.0), DEG_TO_RAD(360.0)};
        joint_limits[5] = {DEG_TO_RAD(-360.0), DEG_TO_RAD(360.0)};


        // 2. 초기 자세
        q_rad.setZero();
        q_rad[0] = DEG_TO_RAD(0.0);
        q_rad[1] = DEG_TO_RAD(0.0);
        q_rad[2] = DEG_TO_RAD(0.0);
        q_rad[3] = DEG_TO_RAD(0.0);
        q_rad[4] = DEG_TO_RAD(0.0);
        q_rad[5] = DEG_TO_RAD(0.0);


        // 3. 작업 영역 초기화
        workspace_min = vec3(-1.3, -1.3, 0.0);
        workspace_max = vec3( 1.3,  1.3, 1.0);


        J = mat<6, 6>::Zero();

        fk(q_rad[0], q_rad[1], q_rad[2], q_rad[3], q_rad[4], q_rad[5]);
    }

    Transform get_fk(TF_Name_ name ) { return tf[name]; }
    Transform get_ee_fk()
    {
        if (tf_ee)
            return tf[TF_End_Effector];
        else
            return tf[TF_Tip];
    }


    // ----------------------------------------------------
    // Move Joint
    // ----------------------------------------------------
    void set_end_effector(const Transform& tf_ee) { this->tf_ee = tf_ee; }


    // ----------------------------------------------------
    // Move Joint
    // ----------------------------------------------------
    void movej(double q0, double q1, double q2, double q3, double q4, double q5)
    {
        q_rad[0] = q0;
        q_rad[1] = q1;
        q_rad[2] = q2;
        q_rad[3] = q3;
        q_rad[4] = q4;
        q_rad[5] = q5;

        fk(q_rad[0], q_rad[1], q_rad[2], q_rad[3], q_rad[4], q_rad[5]);

        J = jacobian(q_rad[0], q_rad[1], q_rad[2], q_rad[3], q_rad[4], q_rad[5]);
    }


    // ----------------------------------------------------
    // Move Linear (QP IK 사용)
    // ----------------------------------------------------
    void movel(double x, double y, double z, double roll, double pitch, double yaw)
    {
        const quat t_rot(
            AngleAxis(yaw, vec3::UnitZ()) *
            AngleAxis(pitch , vec3::UnitY()) * // + M_PI_2
            AngleAxis(roll, vec3::UnitZ()));

        const vec3 t_pos(x, y, z);
        const Transform target_tf(t_rot, t_pos);

        // QP IK 실행
        const vec<6> q = ik(target_tf);

        movej(q[0], q[1], q[2], q[3], q[4], q[5]);
    }

private:
    // ----------------------------------------------------
    // Forward Kinematics
    // ----------------------------------------------------
    Transform fk(double q0, double q1, double q2, double q3, double q4, double q5)
    {
        // 각 Transform 계산
        const Transform tf0 = Transform(AngleAxis(0,  vec3::UnitZ()), vec3(0, 0, 0));
        const Transform tf1 = Transform(AngleAxis(q0, vec3::UnitZ()), vec3(0, 0, l1));
        const Transform tf2 = Transform(AngleAxis(q1, vec3::UnitX()), vec3(-l2, 0, 0));
        const Transform tf3 = Transform(AngleAxis(q2, vec3::UnitX()), vec3(0, 0, l3));
        const Transform tf4 = Transform(AngleAxis(q3, vec3::UnitZ()), vec3(l4, 0, 0));
        const Transform tf5 = Transform(AngleAxis(q4, vec3::UnitX()), vec3(-l6, 0, l5));
        const Transform tf6 = Transform(AngleAxis(q5, vec3::UnitZ()), vec3(l7, 0, 0));
        const Transform tf7 = Transform(AngleAxis(0,  vec3::UnitZ()), vec3(0, 0, l8));


        // Forward Kinematics 계산
        tf[TF_Base]   = tf0;                    // base
        tf[TF_Joint1] = tf[TF_Base]   * tf1;    // 1
        tf[TF_Joint2] = tf[TF_Joint1] * tf2;    // 2
        tf[TF_Joint3] = tf[TF_Joint2] * tf3;    // 3
        tf[TF_Joint4] = tf[TF_Joint3] * tf4;    // 4
        tf[TF_Joint5] = tf[TF_Joint4] * tf5;    // 5
        tf[TF_Joint6] = tf[TF_Joint5] * tf6;    // 6
        tf[TF_Tip]    = tf[TF_Joint6] * tf7;    // tip


        // End Effector Transform 계산
        if (tf_ee)
            tf[TF_End_Effector] = tf[TF_Tip] * tf_ee.value();
        else
            tf[TF_End_Effector] = tf[TF_Tip];


        return tf[TF_End_Effector];
    }


    // ----------------------------------------------------
    // Jacobian Matrix
    // ----------------------------------------------------
    mat<6, 6> jacobian(double q0, double q1, double q2, double q3, double q4, double q5)
    {
        mat<6, 6> J_ = mat<6, 6>::Zero();

        const double c0 = std::cos(q0); const double c1 = std::cos(q1);
        const double c3 = std::cos(q3); const double c4 = std::cos(q4);
        const double s0 = std::sin(q0); const double s1 = std::sin(q1);
        const double s3 = std::sin(q3); const double s4 = std::sin(q4);
        const double c12 = std::cos(q1 + q2); const double s12 = std::sin(q1 + q2);

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
        J_(3,1) = c0; J_(3,2) = c0; J_(3,3) = s0*s12;
        J_(3,4) = -s0*s3*c12 + c0*c3;
        J_(3,5) = s0*s4*c3*c12 + s0*s12*c4 + s3*s4*c0;
        J_(4,1) = s0; J_(4,2) = s0; J_(4,3) = -s12*c0;
        J_(4,4) = s0*c3 + s3*c0*c12;
        J_(4,5) = s0*s3*s4 - s4*c0*c3*c12 - s12*c0*c4;
        J_(5,0) = 1; J_(5,3) = c12; J_(5,4) = s3*s12;
        J_(5,5) = -s4*s12*c3 + c4*c12;

        return J_;
    }


    // ----------------------------------------------------
    // QP Solver (Projected Gauss-Seidel)
    // ----------------------------------------------------
    vec<6> solve_QP(const mat<6, 6>& H, const vec<6>& g, const vec<6>& lb, const vec<6>& ub)
    {

        vec<6> x = vec<6>::Zero();

        for (int iter = 0; iter < MAX_QP_ITER; ++iter) {
            double max_diff = 0.0;
            for (int i = 0; i < 6; ++i) {
                double old_xi = x(i);

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


            if (max_diff < ERROR_QP_THREASHOLD)
                break;
        }

        return x;
    }


    // ----------------------------------------------------
    // Inverse Kinematics
    // ----------------------------------------------------
    vec<6> ik(const Transform& target_tf)
    {







        vec<6> q = q_rad;

        for (int iter = 0; iter < MAX_IK_ITER; ++iter)
        {

            // 오차 계산
            Transform current_tf = fk(q[0], q[1], q[2], q[3], q[4], q[5]);

            vec<6> error_twist = target_tf - current_tf;

            vec<6> dx;
            dx <<   error_twist.head<3>(),  // Linear
                    error_twist.tail<3>();  // Angular


            if (dx.norm() < ERROR_IK_TGREASHOLD)
                break;



            vec3 curr_pos = current_tf.translation();

            // QP: Workspace Limit
            // 범위에 도달할 경우 해당 축방향의 속도를 감속시킴
            for (int i = 0; i < 3; ++i)
            {
                // Min Boundary
                double dist_min = curr_pos(i) - workspace_min(i); // 벽까지 거리

                if (dist_min < WORKSPACE_LIMIT_MARGIN)
                {
                    if (dx(i) < 0) // 벽 밖으로 나가려 할 때
                    {
                        // 0.0 (벽) ~ 1.0 (안전지대) 비율 계산
                        double ratio = std::max(0.0, dist_min / WORKSPACE_LIMIT_MARGIN);

                        double scaling = ratio * ratio; // Quadratic

                        dx(i) *= scaling;
                    }
                }

                // Max Boundary
                double dist_max = workspace_max(i) - curr_pos(i);

                if (dist_max < WORKSPACE_LIMIT_MARGIN)
                {
                    if (dx(i) > 0)
                    {
                        double ratio = std::max(0.0, dist_max / WORKSPACE_LIMIT_MARGIN);

                        double scaling = ratio * ratio;

                        dx(i) *= scaling;
                    }
                }
            }


            // 자코비안 행렬 계산
            J = jacobian(q[0], q[1], q[2], q[3], q[4], q[5]);


            // QP: Hessian Matrix
            // H = J^T * J + lambda * I
            mat<6, 6> H = J.transpose() * J;
            for (int i = 0; i < 6; i++)
                H(i, i) += LAMBDA;


            // QP: Gradient Vector
            // g = -J^T * dx
            vec<6> g = -J.transpose() * dx;



            // QP: 관절 범위 제한
            vec<6> lb, ub;
            for(int i=0; i<6; ++i) {
                double d_min = (joint_limits[i].min + JOINT_LIMIT_MARGIN) - q[i];
                double d_max = (joint_limits[i].max - JOINT_LIMIT_MARGIN) - q[i];

                lb(i) = std::max(d_min, -JOINT_STEP_LIMIT);
                ub(i) = std::min(d_max, JOINT_STEP_LIMIT);
            }



            // 각도 업데이트
            vec<6> dq = solve_QP(H, g, lb, ub);
            q += dq * STEP_SIZE;
        }


        // 각도 정규화
        for (int i=0; i<6; ++i)
            q[i] = NORM_RAD_180(q[i]);

        return q;
    }
};


// ----------------------------------------------------------------------------------
// Main Function
// ----------------------------------------------------------------------------------
int main() {
    std::cout << "M1013 QP Solver Based IK Demo (Multi-Task)" << std::endl;

    M1013 m1013;

    float q0 = 0.0f;
    float q1 = 0.0f;
    float q2 = 0.0f;
    float q3 = 0.0f;
    float q4 = 0.0f;
    float q5 = 0.0f;

    // 초기 목표값
    float target_x      = 0.62f;
    float target_y      = 0.0f;
    float target_z      = 0.4f;
    float target_roll   = 0.0f;
    float target_pitch  = 90.0f;
    float target_yaw    = 0.0f;

    // 작업 영역 UI 변수
    float ws_min[3] = { (float)m1013.workspace_min.x(), (float)m1013.workspace_min.y(), (float)m1013.workspace_min.z() };
    float ws_max[3] = { (float)m1013.workspace_max.x(), (float)m1013.workspace_max.y(), (float)m1013.workspace_max.z() };

    int control_mode = 1; // 0: MoveJ, 1: MoveL

    ImGui::start("로봇 제어 시뮬레이션");

    while (ImGui::isRunning())
    {
        // 실시간으로 로봇 객체의 limit 업데이트
        m1013.workspace_min = vec3(ws_min[0], ws_min[1], ws_min[2]);
        m1013.workspace_max = vec3(ws_max[0], ws_max[1], ws_max[2]);

        ImGui::draw([&]()
        {
            ImGui::Begin("Control Panel");

            // 1. 모드 선택
            ImGui::Text("제어 모드");
            ImGui::RadioButton("MoveJ (Joint)", &control_mode, 0);
            ImGui::SameLine();
            ImGui::RadioButton("MoveL (Task)", &control_mode, 1);
            ImGui::Separator();


            q0 = RAD_TO_DEG(m1013.q_rad[0]);
            q1 = RAD_TO_DEG(m1013.q_rad[1]);
            q2 = RAD_TO_DEG(m1013.q_rad[2]);
            q3 = RAD_TO_DEG(m1013.q_rad[3]);
            q4 = RAD_TO_DEG(m1013.q_rad[4]);
            q5 = RAD_TO_DEG(m1013.q_rad[5]);

            // Joint Control UI
            ImGui::Text("Joint Control");
            ImGui::DragFloat("Joint1", &q0, 0.1f, -360, 360, "%.3f deg");
            ImGui::DragFloat("Joint2", &q1, 0.1f, -360, 360, "%.3f deg");
            ImGui::DragFloat("Joint3", &q2, 0.1f, -360, 360, "%.3f deg");
            ImGui::DragFloat("Joint4", &q3, 0.1f, -360, 360, "%.3f deg");
            ImGui::DragFloat("Joint5", &q4, 0.1f, -360, 360, "%.3f deg");
            ImGui::DragFloat("Joint6", &q5, 0.1f, -360, 360, "%.3f deg");

            ImGui::Separator();
            ImGui::Dummy(ImVec2(0, 10));

            // Task Control UI
            ImGui::Text("Task Control");
            ImGui::DragFloat("X [m]", &target_x, 0.005f, -1.3f, 1.3f);
            ImGui::DragFloat("Y [m]", &target_y, 0.005f, -1.3f, 1.3f);
            ImGui::DragFloat("Z [m]", &target_z, 0.005f, -1.3f, 1.3f);
            ImGui::DragFloat("\u03C6 [deg]", &target_roll, 1.0f, -180.f, 180.f);
            ImGui::DragFloat("\u03B8 [deg]", &target_pitch, 1.0f, -180.f, 180.f);
            ImGui::DragFloat("\u03C8 [deg]", &target_yaw, 1.0f, -180.f, 180.f);



            ImGui::Separator();

            // 2. 작업 영역 설정 (Task Space Limits)
            ImGui::Text("작업 영역 설정 (Box)");
            ImGui::SliderFloat3("최소", ws_min, -1.0f, 1.0f);
            ImGui::SliderFloat3("최대", ws_max, -1.0f, 1.0f);


            ImGui::Separator();
            ImGui::Dummy(ImVec2(0, 10));

            // Set End Effector Position
            static float ee_pos[3] = { 0, 0, 0};
            static float ee_ori[3] = { 0, 0, 0};
            ImGui::Text("엔드 이펙터 설정");
            ImGui::DragFloat3("위치", ee_pos, 0.01f);
            ImGui::DragFloat3("방향", ee_ori, 0.01f);
            if (ImGui::Button("엔드 이펙터 설정"))
            {
                vec3 pos = vec3(ee_pos[0], ee_pos[1], ee_pos[2]);
                AngleAxis ori = AngleAxis( DEG_TO_RAD(0), vec3::UnitZ());
                Transform tf = Transform(ori, pos);
                m1013.set_end_effector(tf);
            }



            // 3. 상태 모니터링
            vec3 pos = m1013.get_ee_fk().translation();
            ImGui::TextColored(ImVec4(0,1,1,1), "Current EE: (%.3f, %.3f, %.3f)", pos.x(), pos.y(), pos.z());
            vec3 rpy = Transform::rpy(m1013.get_ee_fk());
            ImGui::TextColored(ImVec4(0,1,1,1), "Current EE: (%.3f, %.3f, %.3f)", RAD_TO_DEG(rpy.x()), NORM_DEG_180( (RAD_TO_DEG(rpy.y()) )), RAD_TO_DEG(rpy.z()));

            // 위반 경고
            bool violated = false;
            for(int i=0; i<3; i++) if(pos(i) < ws_min[i] || pos(i) > ws_max[i]) violated = true;
            if(violated) ImGui::TextColored(ImVec4(1,0,0,1), "[WARNING] Workspace Limit Hit!");


            // 렌더링
            draw_simulation(m1013.J, m1013.get_ee_fk(), m1013.workspace_min, m1013.workspace_max);



            ImGui::End();
        });

        // duration
        auto t1 = std::chrono::high_resolution_clock::now();


        // 로봇 제어 실행
        if (control_mode == 1) {
            m1013.movel(target_x, target_y, target_z,
                       DEG_TO_RAD(target_roll), DEG_TO_RAD(target_pitch), DEG_TO_RAD(target_yaw));
        }

        else if (control_mode == 0) {
            m1013.movej(DEG_TO_RAD(q0), DEG_TO_RAD(q1), DEG_TO_RAD(q2), DEG_TO_RAD(q3), DEG_TO_RAD(q4), DEG_TO_RAD(q5));
        }

        // duration
        auto t2 = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed_seconds = t2 - t1;
        std::cout << "Time Elapsed: " << elapsed_seconds.count() << "s" << std::endl;

        // 시각화 객체 업데이트
        joint1.setTransform(m1013.get_fk(M1013::TF_Joint1));     // 1
        joint2.setTransform(m1013.get_fk(M1013::TF_Joint2));     // 2
        joint3.setTransform(m1013.get_fk(M1013::TF_Joint3));     // 3
        joint4.setTransform(m1013.get_fk(M1013::TF_Joint4));     // 4
        joint5.setTransform(m1013.get_fk(M1013::TF_Joint5));     // 5
        joint6.setTransform(m1013.get_fk(M1013::TF_Joint6));     // 6
        t_tip.setTransform(m1013.get_fk(M1013::TF_Tip));         // tip
        t_ee.setTransform(m1013.get_fk(M1013::TF_End_Effector)); // end

    }

    ImGui::stop();
    return 0;
}

// ----------------------------------------------------------------------------------
// Helper Functions (Drawing)
// ----------------------------------------------------------------------------------
void draw_box(const vec3& min_pos, const vec3& max_pos) {
    float x1 = min_pos.x(), x2 = max_pos.x();
    float y1 = min_pos.y(), y2 = max_pos.y();
    float z1 = min_pos.z(), z2 = max_pos.z();

    // 바닥 (Z1)
    float bx[] = {x1, x2, x2, x1, x1};
    float by[] = {y1, y1, y2, y2, y1};
    float bz[] = {z1, z1, z1, z1, z1};
    ImPlot3D::PlotLine("BoxBot", bx, by, bz, 5);

    // 천장 (Z2)
    float tx[] = {x1, x2, x2, x1, x1};
    float ty[] = {y1, y1, y2, y2, y1};
    float tz[] = {z2, z2, z2, z2, z2};
    ImPlot3D::PlotLine("BoxTop", tx, ty, tz, 5);

    // 기둥 4개
    float p1x[] = {x1, x1}, p1y[] = {y1, y1}, p1z[] = {z1, z2};
    float p2x[] = {x2, x2}, p2y[] = {y1, y1}, p2z[] = {z1, z2};
    float p3x[] = {x2, x2}, p3y[] = {y2, y2}, p3z[] = {z1, z2};
    float p4x[] = {x1, x1}, p4y[] = {y2, y2}, p4z[] = {z1, z2};

    ImPlot3D::PlotLine("P1", p1x, p1y, p1z, 2);
    ImPlot3D::PlotLine("P2", p2x, p2y, p2z, 2);
    ImPlot3D::PlotLine("P3", p3x, p3y, p3z, 2);
    ImPlot3D::PlotLine("P4", p4x, p4y, p4z, 2);
}

void draw_simulation(const mat<6, 6>& j, const Transform& fk, const vec3& ws_min, const vec3& ws_max)
{
    ImGui::Begin("3D Simulation");
    if (ImPlot3D::BeginPlot("Robot View", ImVec2(-1,-1), ImPlot3DFlags_Equal | ImPlot3DFlags_NoLegend))
    {
        ImPlot3D::SetupAxesLimits(-1.5, 1.5, -1.5, 1.5, 0.0, 1.5, ImPlot3DCond_Always);
        ImPlot3D::SetupAxes("X", "Y", "Z");

        // 로봇 링크 그리기
        base.Draw(); joint1.Draw(); joint2.Draw(); joint3.Draw();
        joint4.Draw(); joint5.Draw(); joint6.Draw(); t_tip.Draw();
        t_ee.Draw();

        DrawLinkLine(base, joint1); DrawLinkLine(joint1, joint2);
        DrawLinkLine(joint2, joint3); DrawLinkLine(joint3, joint4);
        DrawLinkLine(joint4, joint5); DrawLinkLine(joint5, joint6);
        DrawLinkLine(joint6, t_tip); DrawLinkLine(t_tip, t_ee, ImVec4(1.0f, 0.0f, 0.5f, 1.0f));

        // 작업 영역 박스 그리기 (붉은색 힌트)
        ImPlot3D::SetNextLineStyle(ImVec4(1, 0, 0, 0.6f), 2.0f);
        draw_box(ws_min, ws_max);

        // 매니풀러빌리티 타원체
        mat<3, 6> j_pos = j.block<3, 6>(0, 0);
        imgui_draw_manipulability(j_pos, fk.translation(), 0.15f, ImVec4(0,1,1,0.05f));

        ImPlot3D::EndPlot();
    }
    ImGui::End();
}