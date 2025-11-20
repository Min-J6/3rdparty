#pragma once
#include "../../lib/robot/transform.h"
#include "imgui/imgui.h"
#include "imgui/implot3d.h"

// ===================================================================
// AxisObject 클래스 (ImPlot3D 렌더링 객체)
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


    void setTransform(const transform& tf)
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



    AxisObject& operator=(const transform& tf)
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