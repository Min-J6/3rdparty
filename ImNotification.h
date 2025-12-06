#pragma once

#include <string>


#include "imgui.h"

enum class ImGui_NotificationType {
    Info,
    Suceess,
    Error
};

struct ImGui_Notification;


namespace ImGui
{
    // 알림 센터
    void NotificationCenter();

    // 알림센터 표시 함수
    void push_info_noti(const std::string& title, const std::string& content);
    void push_sucesses_noti(const std::string& title, const std::string& content);
    void push_error_noti(const std::string& title, const std::string& content);
}



struct ImGui_Notification
{
    std::string title;              // 제목
    std::string content;            // 내용
    std::string timestamp;          // 시간
    ImGui_NotificationType type;    // 타입
    float swipeOffset = 0.0f;
    std::chrono::steady_clock::time_point creationTime;


    ImGui_Notification(const std::string& t, const std::string& c, ImGui_NotificationType type)
        : title(t), content(c), type(type),
          timestamp(GetTimestamp()),
          creationTime(std::chrono::steady_clock::now()) {}


    static std::string GetTimestamp()
    {
        auto now = std::chrono::system_clock::now();
        auto in_time_t = std::chrono::system_clock::to_time_t(now);
        std::tm buf{};

        localtime_r(&in_time_t, &buf);

        std::ostringstream oss;
        oss << std::put_time(&buf, "%H:%M:%S");
        return oss.str();
    }
};