#pragma once


#include <functional>
#include <mutex>
#include <thread>
#include <atomic>
#include <iomanip>
#include <string>
#include <queue>
#include <iostream>


#include "imgui.h"



// GLFW 윈도우 구조체
struct GLFWwindow;
// 알림 데이터 구조체
struct ImGui_Notification;









namespace ImGui
{
    inline ImFont* Regular = NULL;
    inline ImFont* Bold = NULL;


    // 알림 센터
    void NotificationCenter();

    // 알림센터 표시 함수
    void push_info_noti(const std::string& title, const std::string& content);
    void push_sucesses_noti(const std::string& title, const std::string& content);
    void push_error_noti(const std::string& title, const std::string& content);
}






class ImguiApp {
public:
    // 렌더링 콜백 설정 (매 프레임마다 호출됨)
    static void show_imgui(std::function<void()> func);

    // 백그라운드 스레드에서 ImGui 앱 시작
    static void start_background(const std::string& title);

    // ImGui 앱 종료
    static void stop_background();

    // ImGui 앱이 실행 중인지 확인
    static bool is_running();


private:
    // 초기화
    bool init();

    // 메인 루프
    bool run();

    // 백그라운드 스레드에서 시작
    void startBackground();

    // 백그라운드 스레드 종료
    void stopBackground();

    // 메인 독스페이스
    void show_dockspace();

    // 커스텀 타이틀바
    void show_titlebar();





    // 싱글톤
    ImguiApp() = default;
    ImguiApp(const ImguiApp&) = delete;
    ImguiApp& operator=(const ImguiApp&) = delete;

    static ImguiApp& getInstance();


    // 렌더링 콜백
    std::function<void()> render_callback = nullptr;
    std::mutex callback_mutex;

    std::thread render_thread;
    std::atomic<bool> _is_running{false};
};


