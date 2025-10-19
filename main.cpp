// main.cpp (Ubuntu/Linux X11 환경 기준)

#include <stdio.h>
#include <GLFW/glfw3.h>

#define GLFW_EXPOSE_NATIVE_X11
#include <GLFW/glfw3native.h>

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

static void glfw_error_callback(int error, const char* description)
{
    fprintf(stderr, "[Error] [GLFW]: %d: %s\n", error, description);
}




int main(int, char**)
{
    const char* TITLE     = "Demo";
    const float WIDTH     = 1280.0f;
    const float HEIGHT    = 720.0f;



    glfwSetErrorCallback(glfw_error_callback);
    if (!glfwInit())
    {
        return 1;
    }

    const char* glsl_version = "#version 130";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
    glfwWindowHint(GLFW_DECORATED, GLFW_FALSE);
    glfwWindowHint(GLFW_TRANSPARENT_FRAMEBUFFER, GLFW_TRUE);

    GLFWwindow* window = glfwCreateWindow(WIDTH, HEIGHT, TITLE, NULL, NULL);
    if (window == NULL)
    {
        return 1;
    }

    // ========== 윈도우를 화면 중앙에 배치 ==========
    GLFWmonitor* primary_monitor = glfwGetPrimaryMonitor();
    const GLFWvidmode* mode = glfwGetVideoMode(primary_monitor);

    int window_width = 1280;
    int window_height = 720;
    int window_pos_x = (mode->width - window_width) / 2;
    int window_pos_y = (mode->height - window_height) / 2;

    glfwSetWindowPos(window, window_pos_x, window_pos_y);
    // ================================================

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;

    ImGuiStyle& style = ImGui::GetStyle();
    style.WindowRounding = 3.0f;
    style.Colors[ImGuiCol_WindowBg].w = 0.5f;

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);




    const ImVec4 clear_color    = ImVec4(0.0f, 0.0f, 0.0f, 0.8f);       // 검은색(0.0f, 0.0f, 0.0f)에 투명도 0.5f로 배경색을 설정합니다.
    const float MARGIN          = 1.0f;                                 // GLFW 윈도우와 메인 도킹 사이의 여백 (픽셀)
    const float TITLEBAR_HEIGHT = 30.0f;                                // 커스텀 타이틀바 높이

    ImVec2 docking_size         = ImVec2(1280 - MARGIN, 720 - MARGIN);  // 메인 도킹 크기를 고정값으로 설정 (GLFW 윈도우보다 MARGIN픽셀 작게)
    ImVec2 prev_docking_size    = docking_size;                         // 이전 프레임의 도킹 크기를 저장 (윈도우 리사이즈 감지용)


    // 타이틀바 드래그 상태
    bool is_dragging_titlebar   = false;

    // 드래그 시작 시점의 마우스 스크린 절대 좌표 (윈도우 좌상단 기준 오프셋)
    double drag_offset_x        = 0.0;
    double drag_offset_y        = 0.0;

    // X11 Display 가져오기
    Display* display = glfwGetX11Display();

    while (!glfwWindowShouldClose(window))
    {
        glfwPollEvents();

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGuiViewport* viewport = ImGui::GetMainViewport();

        // 중앙 정렬을 위한 위치 계산
        ImVec2 docking_pos = ImVec2(
            viewport->Pos.x + (viewport->Size.x - docking_size.x) * 0.5f,
            viewport->Pos.y + (viewport->Size.y - docking_size.y) * 0.5f
        );





        // ========== 커스텀 타이틀바 ==========
        {

            ImGui::SetNextWindowPos(docking_pos);
            ImGui::SetNextWindowSize(ImVec2(docking_size.x, TITLEBAR_HEIGHT));
            ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 3.0f);
            ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 5.0f));
            ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.0f, 0.0f, 0.0f, 1.0f));

            ImGuiWindowFlags titlebar_flags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
                                              ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoCollapse;

            ImGui::Begin("##TITLE_BAR", nullptr, titlebar_flags);

            // 타이틀 텍스트 (중앙 정렬)
            float text_width = ImGui::CalcTextSize(TITLE).x;
            float title_pos_x = (docking_size.x - text_width) * 0.5f;
            ImGui::SetCursorPosX(title_pos_x);
            ImGui::Text("%s", TITLE);

            // 닫기 버튼 (오른쪽 정렬)
            ImGui::SameLine(docking_size.x - 33.0f);
            if (ImGui::Button("X", ImVec2(25, 20)))
            {
                glfwSetWindowShouldClose(window, GLFW_TRUE);
            }

            // 타이틀바 드래그 처리 (스크린 절대 좌표 사용)
            if (ImGui::IsWindowHovered() && ImGui::IsMouseClicked(0))
            {
                is_dragging_titlebar = true;

                // 현재 윈도우 위치
                int window_x, window_y;
                glfwGetWindowPos(window, &window_x, &window_y);

                // 현재 마우스 스크린 절대 좌표 가져오기 (X11 API 사용)
                Window root_return, child_return;
                int root_x, root_y;
                int win_x, win_y;
                unsigned int mask_return;

                XQueryPointer(display, glfwGetX11Window(window),
                             &root_return, &child_return,
                             &root_x, &root_y,
                             &win_x, &win_y,
                             &mask_return);

                // 드래그 오프셋 = 마우스 절대 좌표 - 윈도우 절대 좌표
                drag_offset_x = root_x - window_x;
                drag_offset_y = root_y - window_y;
            }

            if (is_dragging_titlebar)
            {
                if (ImGui::IsMouseDown(0))
                {
                    // 현재 마우스 스크린 절대 좌표 가져오기
                    Window root_return, child_return;
                    int root_x, root_y;
                    int win_x, win_y;
                    unsigned int mask_return;

                    XQueryPointer(display, glfwGetX11Window(window),
                                 &root_return, &child_return,
                                 &root_x, &root_y,
                                 &win_x, &win_y,
                                 &mask_return);

                    // 새로운 윈도우 위치 = 마우스 절대 좌표 - 드래그 오프셋
                    int new_x = root_x - (int)drag_offset_x;
                    int new_y = root_y - (int)drag_offset_y;

                    glfwSetWindowPos(window, new_x, new_y);
                }
                else
                {
                    is_dragging_titlebar = false;
                }
            }

            ImGui::End();
            ImGui::PopStyleVar(2);
            ImGui::PopStyleColor(1);
        }


        // ========== 메인 도킹 영역 ==========
        {
            ImVec2 docking_content_pos = ImVec2(docking_pos.x, docking_pos.y + TITLEBAR_HEIGHT);
            ImVec2 docking_content_size = ImVec2(docking_size.x, docking_size.y - TITLEBAR_HEIGHT);

            ImGui::SetNextWindowPos(docking_content_pos);
            ImGui::SetNextWindowSize(docking_content_size);
            ImGui::SetNextWindowViewport(viewport->ID);
            ImGui::SetNextWindowBgAlpha(0.0f); // 도킹 공간을 담는 창 자체는 투명하게 유지

            ImGuiWindowFlags window_flags = ImGuiWindowFlags_NoDocking;
            window_flags |= ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoMove;
            window_flags |= ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoNavFocus;
            // NoResize 플래그를 제거하여 리사이즈 핸들이 보이도록 함

            ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
            ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
            ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));

            ImGui::Begin("DockSpace Demo", nullptr, window_flags);

            ImGui::PopStyleVar(3);

            ImGuiID dockspace_id = ImGui::GetID("MyDockSpace");
            // PassthruCentralNode 플래그 덕분에 중앙 노드가 투명해져서 clear_color가 보이게 됩니다.
            ImGui::DockSpace(dockspace_id, ImVec2(0.0f, 0.0f), ImGuiDockNodeFlags_PassthruCentralNode);

            // 현재 윈도우 크기 가져오기
            ImVec2 current_window_size = ImGui::GetWindowSize();

            // 전체 도킹 크기 (타이틀바 포함)
            ImVec2 total_docking_size = ImVec2(current_window_size.x, current_window_size.y + TITLEBAR_HEIGHT);

            // 도킹 윈도우 크기가 변경되었는지 확인
            if (total_docking_size.x != prev_docking_size.x || total_docking_size.y != prev_docking_size.y)
            {
                // GLFW 윈도우 크기를 도킹 크기 + MARGIN으로 설정
                int new_glfw_width = (int)(total_docking_size.x + MARGIN);
                int new_glfw_height = (int)(total_docking_size.y + MARGIN);

                glfwSetWindowSize(window, new_glfw_width, new_glfw_height);

                // 다음 프레임을 위해 도킹 크기 업데이트
                docking_size = total_docking_size;
            }

            // 현재 크기를 다음 프레임을 위해 저장
            prev_docking_size = total_docking_size;

            ImGui::End();

            ImGui::Begin("My Opaque Window");
            ImGui::Text("This UI is opaque.");
            ImGui::Text("The window background is semi-transparent black!");
            ImGui::Text("You can drag this window by its title bar.");
            ImGui::End();

            ImGui::Begin("Another Window");
            ImGui::Text("Hello from another window!");
            ImGui::End();
        }


        ImGui::Render();
        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);

        // 설정된 clear_color로 화면을 지웁니다.
        glClearColor(clear_color.x * clear_color.w, clear_color.y * clear_color.w, clear_color.z * clear_color.w, clear_color.w);
        glClear(GL_COLOR_BUFFER_BIT);

        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable)
        {
            GLFWwindow* backup_current_context = glfwGetCurrentContext();
            ImGui::UpdatePlatformWindows();
            ImGui::RenderPlatformWindowsDefault();
            glfwMakeContextCurrent(backup_current_context);
        }

        glfwSwapBuffers(window);
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}