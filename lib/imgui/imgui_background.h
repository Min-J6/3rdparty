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



class ImGuiBackground {
public:
    // 렌더링 콜백 설정 (매 프레임마다 호출됨)
    static void show_imgui(std::function<void()> func);

    // 백그라운드 스레드에서 ImGui 앱 시작
    static void start_background(const std::string& title, const ImVec2& size = ImVec2(1280, 720));

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
    void _start_background();

    // 백그라운드 스레드 종료
    void stopBackground();

    // 메인 독스페이스 GUI
    void show_dockspace();

    // 커스텀 타이틀바 GUI
    void show_titlebar();



    // 싱글톤
    ImGuiBackground() = default;
    ImGuiBackground(const ImGuiBackground&) = delete;
    ImGuiBackground& operator=(const ImGuiBackground&) = delete;

    static ImGuiBackground& getInstance();


    // 렌더링 콜백
    std::function<void()> render_callback = nullptr;
    std::mutex callback_mutex;

    std::thread render_thread;
    std::atomic<bool> _is_running{false};
};


