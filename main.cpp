#include <implot.h>
#include <implot3d.h>
#include <iostream>

#include "imgui.h"




#include <iostream>
#include <eigen3/Eigen/Dense>




// 3D 강체 변환(회전 + 이동)을 나타내는 클래스
class Transform {
public:
    using AngleAxis = Eigen::AngleAxisd;
    using Isometry = Eigen::Isometry3d;
    using mat4 = Eigen::Matrix4d;
    using vec3 = Eigen::Vector3d;
    using quat = Eigen::Quaterniond;
    using mat3 = Eigen::Matrix3d;

private:
    Isometry T_;

public:
    Transform() : T_(Isometry::Identity()) {}
    explicit Transform(const Isometry& iso) : T_(iso) {}
    explicit Transform(const mat4& H) : T_(H) {}

    Transform(const quat& q, const vec3& t) : T_(Isometry::Identity()) {
        T_.rotate(q);
        T_.pretranslate(t);
    }

    Transform(const mat3& R, const vec3& t) : T_(Isometry::Identity()) {
        T_.linear() = R;
        T_.translation() = t;
    }

    // --- 접근자 (Getters) --- //

    const mat4& matrix() const {
        return T_.matrix();
    }

    mat3 rotation() const {
        return T_.rotation();
    }

    quat quaternion() const {
        return quat(T_.rotation());
    }

    vec3 translation() const {
        return T_.translation();
    }

    const Isometry& isometry() const {
        return T_;
    }


    // --- 수정자 (Setters) --- //

    void setIdentity() {
        T_.setIdentity();
    }

    void setTranslation(const vec3& t) {
        T_.translation() = t;
    }

    void setRotation(const quat& q) {
        T_.linear() = q.toRotationMatrix();
    }

    void setRotation(const mat3& R) {
        T_.linear() = R;
    }

    // --- 다른 타입으로부터의 대입 연산자 --- //
    Transform& operator=(const mat4& m) {
        this->T_ = m;
        return *this;
    }


    // --- 핵심 연산 --- //

    Transform inverse() const {
        return Transform(T_.inverse());
    }

    Transform operator*(const Transform& other) const {
        return Transform(this->T_ * other.T_);
    }

    vec3 operator*(const vec3& point) const {
        return this->T_ * point;
    }


    // --- 유틸리티 --- //

    static Transform fromRPY(double roll, double pitch, double yaw, const vec3& t = vec3::Zero()) {
        Eigen::AngleAxisd rollAngle(roll, vec3::UnitX());
        Eigen::AngleAxisd pitchAngle(pitch, vec3::UnitY());
        Eigen::AngleAxisd yawAngle(yaw, vec3::UnitZ());
        quat q = yawAngle * pitchAngle * rollAngle;
        return Transform(q, t);
    }

    friend std::ostream& operator<<(std::ostream& os, const Transform& rt) {
        os  << rt.matrix();
        return os;
    }
};


// ===================================================================
// 2. AxisObject 클래스 (ImPlot3D 렌더링 객체)
// ===================================================================
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


    void SetTransform(const Transform& tf)
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
        this->SetTransform(tf);
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

private:
    float origin[3]; // 축의 원점 (X, Y, Z)
    float endX[3];   // X축의 끝점 (X, Y, Z)
    float endY[3];   // Y축의 끝점 (X, Y, Z)
    float endZ[3];   // Z축의 끝점 (X, Y, Z)
};









#define DEG_TO_RAD(x) ((x) * 0.017453292519943295769236907684886)








int main() {
    std::cout << "Hello, World!" << std::endl;

    AxisObject origine(0.1);
    AxisObject base(0.2);
    AxisObject joint1(0.2);
    AxisObject joint2(0.2);
    AxisObject joint3(0.2);



    Transform base_tf;
    Transform joint1_tf;
    Transform joint2_tf;
    Transform joint3_tf;



    // ----------------------------------
    // 원점 생성
    // ----------------------------------
    {
        Transform::vec3 t(0, 0, 0);
        Transform::quat q(Transform::AngleAxis(0.f, Transform::vec3(0, 0, 0)));
        Transform tf = Transform(q, t);

        origine = tf;
    }


    // ----------------------------------
    // Base -
    // ----------------------------------
    {
        Transform::vec3 t(0, 0, 0);
        Transform::quat q(Transform::AngleAxis(0.f, Transform::vec3(0, 0, 1)));
        Transform tf = Transform(q, t);
        base = tf;
        base_tf = tf;
    }


    // ----------------------------------
    // joint 1
    // ----------------------------------
    {

        Transform::vec3 t(0, 0, 0);
        Transform::quat q(Transform::AngleAxis(0.f, Transform::vec3(0, 0, 1)));
        Transform tf = Transform(q, t);
        joint1 = tf;
        joint1_tf = tf;
    }


    // ----------------------------------
    // joint 2
    // ----------------------------------
    {

        Transform::vec3 t(0, 0, 1);
        Transform::quat q(Transform::AngleAxis(0.f, Transform::vec3(0, 0, 1)));
        Transform tf = Transform(q, t);
        joint2 = tf;
        joint2_tf = tf;
    }


    // ----------------------------------
    // joint 3
    // ----------------------------------
    {

        Transform::vec3 t(0, 0, 1);
        Transform::quat q(Transform::AngleAxis(0.f, Transform::vec3(0, 0, 1)));
        Transform tf = Transform(q, t);
        joint3 = tf;
        joint3_tf = tf;
    }













    ImGui::start("데모");

    while (ImGui::isRunning())
    {
        joint1 = base_tf * joint1_tf;
        joint2 = base_tf * joint1_tf * joint2_tf;
        joint3 = base_tf * joint1_tf * joint2_tf * joint3_tf;


        ImGui::draw([&]()
        {
            ImGui::Begin("Draw");
            if (ImPlot3D::BeginPlot("Axis Plot", ImVec2(-1, -1), ImPlot3DFlags_Equal))
            {
                origine.Draw();
                base.Draw();
                joint1.Draw();
                joint2.Draw();
                joint3.Draw();

                ImPlot3D::EndPlot();
            }
            ImGui::End();

        });
    }
    ImGui::stop();


    return 0;
}
