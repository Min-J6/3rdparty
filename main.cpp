#include "lib/imgui/icon.h"
#include "lib/imgui/imgui_app.h"
#include "lib/imgui/ImCoolBar.h"

int main(int, char**) {
    // 백그라운드에서 ImGui 앱 시작
    ImguiApp::start_background("데모");



    // 사용자가 원했던 바로 그 코드 구조
    while (ImguiApp::is_running()) {

        // 매 루프마다 렌더링 콜백 설정
        ImguiApp::show_imgui([](){
            ImGui::Begin("시리얼 모니터");



            ImGui::End();

        });

    }

    // ImGui 앱 종료
    ImguiApp::stop_background();

    return 0;
}


