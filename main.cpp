#include "lib/imgui/icon.h"
#include "lib/imgui/imgui_app.h"
#include "lib/imgui/ImCoolBar.h"
#include "node_editor.h"
#include <vector>
#include <string>
#include <memory>
#include <algorithm>
#include <cmath>
#include <map>
#include <set>




int main(int, char**) {
    // 백그라운드에서 ImGui 앱 시작
    ImguiApp::start_background("노드 에디터 데모");





    // 배경 크기 입력 변수
    static float bg_size[2] = {0, 0};

    while (ImguiApp::is_running()) {
        // 매 루프마다 렌더링 콜백 설정
        ImguiApp::show_imgui([&](){
            ImGui::Begin("시리얼 모니터");



            ImGui::End();
        });
    }


    // ImGui 앱 종료
    ImguiApp::stop_background();

    return 0;
}

