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

using namespace Robot;

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
    const double l1 = 0.1525;
    const double l2 = 0.1985;
    const double l3 = 0.620;
    const double l4 = 0.164;
    const double l5 = 0.559;
    const double l6 = 0.146;
    const double l7 = 0.146;
    const double l8 = 0.121;

    // Ik 설정
    const double ERROR_IK_TGREASHOLD    = 0.001;
    const int MAX_IK_ITER               = 500;
    const double STEP_SIZE              = 0.001; // Alpha
    const double WORKSPACE_LIMIT_MARGIN = 0.1;
    const double LAMBDA                 = 0.1;

    // 관절 범위 제한 설정
    const double JOINT_STEP_LIMIT = 1.0;
    const double JOINT_LIMIT_MARGIN = 0.00;

    // QP Solver 설정
    const int MAX_QP_ITER               = 500;
    const double ERROR_QP_THREASHOLD    = 0.001;


    // 로봇 상태 변수
    std::array<Transform, 9> tf;                // 시각화용 (double)
    std::optional<Transform> tf_ee;             // EE Offset (double)


public:
    vec<6> q_rad;                               // 현재 관절 각도
    mat<6, 6> J;                                // Jacobian

    // 작업 영역
    vec3 workspace_min;
    vec3 workspace_max;

    // 관절 제한
    std::array<JointLimits, 6> joint_limits;

    enum TF_Name_ {
        TF_Base, TF_Joint1, TF_Joint2, TF_Joint3,
        TF_Joint4, TF_Joint5, TF_Joint6, TF_Tip, TF_End_Effector
    };

    // ----------------------------------------------------------
    // AutoDiff 관련 타입 정의
    // ----------------------------------------------------------



    M1013()
    {
        // 관절 제한 설정
        joint_limits[0] = {DEG_TO_RAD(-360.0), DEG_TO_RAD(360.0)};
        joint_limits[1] = {DEG_TO_RAD(-360.0), DEG_TO_RAD(360.0)};
        joint_limits[2] = {DEG_TO_RAD(-160.0), DEG_TO_RAD(160.0)};
        joint_limits[3] = {DEG_TO_RAD(-360.0), DEG_TO_RAD(360.0)};
        joint_limits[4] = {DEG_TO_RAD(-360.0), DEG_TO_RAD(360.0)};
        joint_limits[5] = {DEG_TO_RAD(-360.0), DEG_TO_RAD(360.0)};

        // 초기 자세
        q_rad.setZero();

        // 작업 영역 초기화
        workspace_min = vec3(-1.3, -1.3, 0.0);
        workspace_max = vec3( 1.3,  1.3, 1.0);

        // Jacobian 초기화
        J = mat<6, 6>::Zero();

        // 초기 FK 계산
        fk(q_rad[0], q_rad[1], q_rad[2], q_rad[3], q_rad[4], q_rad[5]);
    }

    Transform get_fk(TF_Name_ name ) { return tf[name]; }
    Transform get_ee_fk()
    {
        return tf_ee ? tf[TF_End_Effector] : tf[TF_Tip];
    }

    void set_end_effector(const Transform& tf_ee) { this->tf_ee = tf_ee; }

    // ----------------------------------------------------
    // Move Joint
    // ----------------------------------------------------
    void movej(double q0, double q1, double q2, double q3, double q4, double q5)
    {
        q_rad << q0, q1, q2, q3, q4, q5;

        // FK 업데이트 (시각화용)
        fk(q0, q1, q2, q3, q4, q5);

        // Jacobian 업데이트 (AutoDiff 사용)
        J = jacobian(q0, q1, q2, q3, q4, q5);
    }

    // ----------------------------------------------------
    // Move Linear (QP IK)
    // ----------------------------------------------------
    void movel(double x, double y, double z, double roll, double pitch, double yaw)
    {
        const quat t_rot(
            AngleAxis(yaw, vec3::UnitZ()) *
            AngleAxis(pitch , vec3::UnitY()) *
            AngleAxis(roll, vec3::UnitZ()));

        const vec3 t_pos(x, y, z);
        const Transform target_tf(t_rot, t_pos);

        // QP IK 실행
        const vec<6> q = ik(target_tf);

        movej(q[0], q[1], q[2], q[3], q[4], q[5]);
    }

private:
    // ----------------------------------------------------
    // [핵심] FK Kernel (Template)
    // T: double 또는 ADScalar
    // ----------------------------------------------------
    template <typename T>
    TransformT<T> fk_kernel(const T& q0, const T& q1, const T& q2, const T& q3, const T& q4, const T& q5)
    {
        using Vec3 = Eigen::Matrix<T, 3, 1>;
        using Axis = Eigen::AngleAxis<T>;

        const Vec3 ax_x = Vec3::UnitX();
        // const Vec3 ax_y = Vec3::UnitY(); // 미사용
        const Vec3 ax_z = Vec3::UnitZ();

        // 링크 길이 (double -> T 타입 변환)
        T tl1(l1), tl2(l2), tl3(l3), tl4(l4), tl5(l5), tl6(l6), tl7(l7), tl8(l8);

        // 각 Transform 계산
        const TransformT<T> tf0(Axis(T(0), ax_z), Vec3(T(0), T(0), T(0)));
        const TransformT<T> tf1(Axis(q0,   ax_z), Vec3(T(0), T(0), tl1));
        const TransformT<T> tf2(Axis(q1,   ax_x), Vec3(-tl2, T(0), T(0)));
        const TransformT<T> tf3(Axis(q2,   ax_x), Vec3(T(0), T(0), tl3));
        const TransformT<T> tf4(Axis(q3,   ax_z), Vec3(tl4, T(0), T(0)));
        const TransformT<T> tf5(Axis(q4,   ax_x), Vec3(-tl6, T(0), tl5));
        const TransformT<T> tf6(Axis(q5,   ax_z), Vec3(tl7, T(0), T(0)));
        const TransformT<T> tf7(Axis(T(0), ax_z), Vec3(T(0), T(0), tl8));

        // 체인 곱: Base -> ... -> End
        TransformT<T> t_curr = tf0;

        // (옵션) 중간 관절 위치가 필요하다면 여기서 저장 가능하지만,
        // AutoDiff용으로는 End-Effector만 계산하면 효율적입니다.
        // double 버전에서는 멤버변수 tf[] 업데이트를 위해 각각 계산합니다.

        t_curr = t_curr * tf1;
        t_curr = t_curr * tf2;
        t_curr = t_curr * tf3;
        t_curr = t_curr * tf4;
        t_curr = t_curr * tf5;
        t_curr = t_curr * tf6;
        t_curr = t_curr * tf7; // Tip

        return t_curr;
    }

    // ----------------------------------------------------
    // FK (Wrapper for double - 시각화용)
    // ----------------------------------------------------
    Transform fk(double q0, double q1, double q2, double q3, double q4, double q5)
    {
        // 1. 각 Transform 개별 계산 (시각화 변수 업데이트를 위해 기존 로직 유지)
        const Transform tf0 = Transform(AngleAxis(0,  vec3::UnitZ()), vec3(0, 0, 0));
        const Transform tf1 = Transform(AngleAxis(q0, vec3::UnitZ()), vec3(0, 0, l1));
        const Transform tf2 = Transform(AngleAxis(q1, vec3::UnitX()), vec3(-l2, 0, 0));
        const Transform tf3 = Transform(AngleAxis(q2, vec3::UnitX()), vec3(0, 0, l3));
        const Transform tf4 = Transform(AngleAxis(q3, vec3::UnitZ()), vec3(l4, 0, 0));
        const Transform tf5 = Transform(AngleAxis(q4, vec3::UnitX()), vec3(-l6, 0, l5));
        const Transform tf6 = Transform(AngleAxis(q5, vec3::UnitZ()), vec3(l7, 0, 0));
        const Transform tf7 = Transform(AngleAxis(0,  vec3::UnitZ()), vec3(0, 0, l8));

        tf[TF_Base]   = tf0;
        tf[TF_Joint1] = tf[TF_Base]   * tf1;
        tf[TF_Joint2] = tf[TF_Joint1] * tf2;
        tf[TF_Joint3] = tf[TF_Joint2] * tf3;
        tf[TF_Joint4] = tf[TF_Joint3] * tf4;
        tf[TF_Joint5] = tf[TF_Joint4] * tf5;
        tf[TF_Joint6] = tf[TF_Joint5] * tf6;
        tf[TF_Tip]    = tf[TF_Joint6] * tf7;

        if (tf_ee)
            tf[TF_End_Effector] = tf[TF_Tip] * tf_ee.value();
        else
            tf[TF_End_Effector] = tf[TF_Tip];

        return tf[TF_End_Effector];
    }


    // ----------------------------------------------------
    // Jacobian (AutoDiff 구현)
    // ----------------------------------------------------
    mat<6, 6> jacobian(double q0, double q1, double q2, double q3, double q4, double q5)
    {
        // 1. 입력 변수(Joint)를 AutoDiffScalar로 변환 및 Seed 설정
        ADScalar q_ad[6];
        const double q_vals[6] = {q0, q1, q2, q3, q4, q5};

        for (int i = 0; i < 6; ++i) {
            q_ad[i].value() = q_vals[i];
            q_ad[i].derivatives() = vec<6>::Unit(6, i); // 미분 Seed (단위 벡터)
        }


        // 2. FK Kernel 실행 (AutoDiff 타입)
        // Tip까지의 변환을 구함
        const ADTransform tf_res = fk_kernel<ADScalar>(q_ad[0], q_ad[1], q_ad[2], q_ad[3], q_ad[4], q_ad[5]);

        // EE Offset 적용 (만약 존재한다면)
        if (tf_ee) {
            // // double형 Transform을 ADScalar형으로 변환
            // ADTransform tf_offset;
            // mat4 m = tf_ee->matrix();
            // for(int r=0; r<4; ++r) for(int c=0; c<4; ++c)
            //     tf_offset.matrix()(r,c) = ADScalar(m(r,c));
            //     // 상수는 미분값이 0이므로 생성자에서 자동 처리됨
            //
            // tf_res = tf_res * tf_offset;
        }


        // 3. 자코비안 추출
        mat<6, 6> J_ad = mat<6, 6>::Zero();


        // 3-1. Linear Velocity Jacobian (Position의 미분)
        const ADVec<3> pos = tf_res.translation();
        for (int i = 0; i < 3; ++i) {
            // pos(i).derivatives()는 6x1 벡터
            J_ad.row(i) = pos(i).derivatives();
        }


        // 3-2. Angular Velocity Jacobian (Rotation 미분)
        // 공식: [w]x = R_dot * R^T
        ADMat<3, 3> R_ad = tf_res.rotation();
        mat3 R_val; // 값만 추출 (double)
        for(int r=0; r<3; r++)
        {
            for(int c=0; c<3; c++)
            {
                R_val(r,c) = R_ad(r,c).value();
            }
        }


        // 각 Joint 변수(q_j)에 대해
        for (int j = 0; j < 6; ++j)
        {
            mat3 dR_dq; // dR / dq_j
            for(int r=0; r<3; r++) for(int c=0; c<3; c++)
            {

                dR_dq(r,c) = R_ad(r,c).derivatives()(j);

            }


            // Skew Matrix S = dR_dq * R^T
            mat3 S = dR_dq * R_val.transpose();

            // S = [[ 0, -wz, wy], [wz, 0, -wx], [-wy, wx, 0]]
            J_ad(3, j) = S(2, 1); // wx
            J_ad(4, j) = S(0, 2); // wy
            J_ad(5, j) = S(1, 0); // wz
        }


        return J_ad;
    }


    // ----------------------------------------------------
    // QP Solver (기존 유지)
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
                double x_new = std::max(lb(i), std::min(ub(i), x_unc));
                x(i) = x_new;
                max_diff = std::max(max_diff, std::abs(x_new - old_xi));
            }
            if (max_diff < ERROR_QP_THREASHOLD) break;
        }
        return x;
    }


    // ----------------------------------------------------
    // Inverse Kinematics (기존 로직 유지, Jacobian 호출만 변경됨)
    // ----------------------------------------------------
    vec<6> ik(const Transform& target_tf)
    {
        vec<6> q = q_rad;

        for (int iter = 0; iter < MAX_IK_ITER; ++iter)
        {
            Transform current_tf = fk(q[0], q[1], q[2], q[3], q[4], q[5]);
            vec<6> error_twist = Transform::twist(current_tf, target_tf);

            vec<6> dx;
            dx << error_twist.head<3>(), error_twist.tail<3>();

            if (dx.norm() < ERROR_IK_TGREASHOLD) break;

            vec3 curr_pos = current_tf.translation();

            // QP: Workspace Limit (감속 로직)
            for (int i = 0; i < 3; ++i) {
                double dist_min = curr_pos(i) - workspace_min(i);
                if (dist_min < WORKSPACE_LIMIT_MARGIN) {
                    if (dx(i) < 0) {
                        double ratio = std::max(0.0, dist_min / WORKSPACE_LIMIT_MARGIN);
                        dx(i) *= (ratio * ratio);
                    }
                }
                double dist_max = workspace_max(i) - curr_pos(i);
                if (dist_max < WORKSPACE_LIMIT_MARGIN) {
                    if (dx(i) > 0) {
                        double ratio = std::max(0.0, dist_max / WORKSPACE_LIMIT_MARGIN);
                        dx(i) *= (ratio * ratio);
                    }
                }
            }

            // [변경점] AutoDiff Jacobian 계산 호출
            J = jacobian(q[0], q[1], q[2], q[3], q[4], q[5]);

            // QP 구성 (Hessian, Gradient)
            mat<6, 6> H = J.transpose() * J;
            for (int i = 0; i < 6; i++) H(i, i) += LAMBDA;

            vec<6> g = -J.transpose() * dx;

            // 관절 범위 제한
            vec<6> lb, ub;
            for(int i=0; i<6; ++i) {
                double d_min = (joint_limits[i].min + JOINT_LIMIT_MARGIN) - q[i];
                double d_max = (joint_limits[i].max - JOINT_LIMIT_MARGIN) - q[i];
                lb(i) = std::max(d_min, -JOINT_STEP_LIMIT);
                ub(i) = std::min(d_max, JOINT_STEP_LIMIT);
            }

            vec<6> dq = solve_QP(H, g, lb, ub);
            q += dq * STEP_SIZE;
        }

        for (int i=0; i<6; ++i) q[i] = NORM_RAD_180(q[i]);

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

        ImGui::context([&]()
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
                Eigen::AngleAxis ori = Eigen::AngleAxis( DEG_TO_RAD(0), vec3::UnitZ());
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

        auto t1 = std::chrono::high_resolution_clock::now();


        // 로봇 제어 실행
        if (control_mode == 1) {
            m1013.movel(target_x, target_y, target_z,
                       DEG_TO_RAD(target_roll), DEG_TO_RAD(target_pitch), DEG_TO_RAD(target_yaw));
        }

        else if (control_mode == 0) {
            m1013.movej(DEG_TO_RAD(q0), DEG_TO_RAD(q1), DEG_TO_RAD(q2), DEG_TO_RAD(q3), DEG_TO_RAD(q4), DEG_TO_RAD(q5));
        }


        auto t2 = std::chrono::high_resolution_clock::now();


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