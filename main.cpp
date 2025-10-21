// main.cpp (Ubuntu/Linux X11 환경 기준)









#include "lib/imgui/imgui_app.h"







int main(int, char**)
{
    // 백그라운드에서 ImGui 앱 시작
    ImguiApp::start_background("DEEEEEEMO");


    int message_count = 0;

    // 사용자가 원했던 바로 그 코드 구조
    while (ImguiApp::IsRunning()) {

        // 매 루프마다 렌더링 콜백 설정
        ImguiApp::show_imgui([message_count](){
            ImGui::Begin("Main Loop Message");
            ImGui::Text("This message is from the main loop.");
            ImGui::Text("The loop is spinning freely!");
            ImGui::Text("Latest message number: %d", message_count);
            ImGui::End();
        });

        message_count++;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // ImGui 앱 종료
    ImguiApp::stop_background();

    return 0;
}