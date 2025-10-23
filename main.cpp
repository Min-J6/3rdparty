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

    // 노드 에디터 인스턴스 생성
    NodeEditor::Editor editor;

    // 배경 크기 설정 (기본값: 2000x2000)
    editor.SetBackgroundSize(ImVec2(0, 0));

    // 샘플 노드 추가
    editor.AddNode("시작 노드", ImVec2(100, 100));
    editor.AddNode("처리 노드", ImVec2(300, 150));
    editor.AddNode("출력 노드", ImVec2(500, 100));
    editor.AddNode("변환 노드", ImVec2(300, 300));



    // 배경 크기 입력 변수
    static float bg_size[2] = {0, 0};

    while (ImguiApp::is_running()) {
        // 매 루프마다 렌더링 콜백 설정
        ImguiApp::show_imgui([&](){
            ImGui::Begin("노드 에디터", nullptr, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);

            // 상단 툴바
            if (ImGui::Button("노드 추가")) {
                static int node_counter = 0;
                editor.AddNode("새 노드 " + std::to_string(++node_counter), ImVec2(200, 200));
            }

            ImGui::SameLine();

            // 배경 크기 설정
            ImGui::SetNextItemWidth(100);
            bg_size[0] = editor.GetBackgroundSize().x;
            bg_size[1] = editor.GetBackgroundSize().y;
            if (ImGui::InputFloat2("배경 크기", bg_size)) {
                editor.SetBackgroundSize(ImVec2(bg_size[0], bg_size[1]));
            }

            ImGui::SameLine();

            // 배경 이미지 로드 버튼
            if (ImGui::Button("배경 이미지 로드")) {
                // 예제: 파일 경로를 직접 지정 (실제로는 파일 다이얼로그 사용)
                // 여기서는 테스트용 이미지 경로를 사용
                const char* test_image = "background.png";

                editor.SetBackgroundImage(test_image);
                // bg_loaded = true;

            }

            ImGui::SameLine();

            // 배경 이미지 제거 버튼
            // if (ImGui::Button("배경 이미지 제거")) {
            //     editor.ClearBackgroundImage();
            //     if (bg_loaded && bg_texture != 0) {
            //         GLuint texture_id = (GLuint)(intptr_t)bg_texture;
            //         glDeleteTextures(1, &texture_id);
            //         bg_texture = 0;
            //         bg_loaded = false;
            //     }
            // }

            ImGui::Separator();

            // 노드 에디터 렌더링
            editor.Render();

            ImGui::End();
        });
    }

    // 텍스처 정리
    // if (bg_loaded && bg_texture != 0) {
    //     GLuint texture_id = (GLuint)(intptr_t)bg_texture;
    //     glDeleteTextures(1, &texture_id);
    // }

    // ImGui 앱 종료
    ImguiApp::stop_background();

    return 0;
}

