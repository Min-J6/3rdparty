#include "imgui_app.h"


#define GLFW_EXPOSE_NATIVE_X11
#include <algorithm>
#include <GLFW/glfw3.h>
#include <GLFW/glfw3native.h>


#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"


#include "font.cpp"
#include "font_bold.cpp"
#include "icon.h"
#include "icon.cpp"

#include "ImNotification.h"


// 윈도우 크기
std::string TITLE               = "DEMO";
const float WINDOW_WIDTH        = 1280;
const float WINDOW_HEIGHT       = 720;


const ImVec4 CLEAR_COLOR        = ImVec4(0.1f, 0.1f, 0.1f, 0.7f);
const float DOCKSPACE_MARGIN    = 1.0f;
const float TITLEBAR_HEIGHT     = 30.0f;


// 폰트 관련 상수
const float FONST_SIZE          = 16.0f;
const float ICON_SIZE           = 20.0f;
const ImVec2 GLYPH_OFFSET       = ImVec2(0.5f, 2.f);


// 타이틀바 관련 상수
const ImVec2 TITLEBAR_PADDING       = ImVec2(0, 6.0f);
const ImVec2 TITLEBAR_BUTTON_OFFSET = ImVec2(-33.0f, 5.0f);
const ImVec2 TITLEBAR_BUTTON_SIZE   = ImVec2(25.f, 20.f);



// 도킹 위치 이동 및 크기 조절 변수
ImVec2 docking_size             = ImVec2(WINDOW_WIDTH - DOCKSPACE_MARGIN, WINDOW_HEIGHT - DOCKSPACE_MARGIN);
ImVec2 prev_docking_size        = docking_size;
ImVec2 docking_pos              = ImVec2(0, 0);


// 타이틀바 드래깅관련 변수
bool is_dragging_titlebar       = false;
double drag_offset_x            = 0.0;
double drag_offset_y            = 0.0;


GLFWwindow* window;



ImguiApp& ImguiApp::getInstance() {
    static ImguiApp instance;
    return instance;
}



void ImguiApp::show_imgui(std::function<void()> func) {
    std::lock_guard<std::mutex> lock(getInstance().callback_mutex);
    getInstance().render_callback = func;
}



void ImguiApp::start_background(const std::string& title) {
    TITLE = title;

    getInstance().startBackground();

    while (!ImguiApp::is_running()) {
        // 쓰레드가 켜질 때까지 대기
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}



void ImguiApp::stop_background() {
    getInstance().stopBackground();
}



bool ImguiApp::is_running() {
    return getInstance()._is_running.load();
}



bool ImguiApp::init() {
    // GLFW 초기화
    if (!glfwInit()) {
        std::cout << "[Error] [ImGui App]: GLFW 초기화 실패\n";
        return false;
    }




    // 윈도우 힌트 설정
    const char* glsl_version = "#version 130";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
    glfwWindowHint(GLFW_DECORATED, GLFW_FALSE);
    glfwWindowHint(GLFW_TRANSPARENT_FRAMEBUFFER, GLFW_TRUE);





    // 윈도우 생성
    window = glfwCreateWindow(WINDOW_WIDTH, WINDOW_HEIGHT, TITLE.c_str(), NULL, NULL);
    if (window == NULL) {
        std::cout << "[Error] [ImGui App]: GLFW 윈도우 생성 실패\n";
        return false;
    }




    // 윈도우 창이 화면 가운데에 뜨도록 설정
    GLFWmonitor* primary_monitor = glfwGetPrimaryMonitor();
    const GLFWvidmode* mode = glfwGetVideoMode(primary_monitor);
    int window_pos_x = (mode->width - WINDOW_WIDTH) / 2;
    int window_pos_y = (mode->height - WINDOW_HEIGHT) / 2;
    glfwSetWindowPos(window, window_pos_x, window_pos_y);




    // glfw 컨택스트 생성
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);





    // ImGui 초기화
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;




    // Imgui 스타일 설정
    ImGuiStyle& style = ImGui::GetStyle();
    style.WindowRounding = 3.0f;
    style.FrameRounding = 3.0f;
    style.Colors[ImGuiCol_WindowBg].w = 0.5f;




    // 폰트 추가
    ImFontConfig config;
    config.MergeMode = true;
    config.GlyphOffset = GLYPH_OFFSET;
    config.GlyphMinAdvanceX = FONST_SIZE;
    static const ImWchar icon_ranges[] = { ICON_MIN_MD, ICON_MAX_16_MD, 0 };


    {
        io.Fonts->Clear();
        ImGui::Regular = io.Fonts->AddFontFromMemoryCompressedTTF(
            NEXON_Lv2_Gothic_Medium_compressed_data,
            NEXON_Lv2_Gothic_Medium_compressed_size,
            FONST_SIZE,
            NULL,
            io.Fonts->GetGlyphRangesKorean()
        );

        // 아이콘 폰트 추가
        io.Fonts->AddFontFromMemoryCompressedTTF(
            MaterialSymbolsRounded_compressed_data,
            MaterialSymbolsRounded_compressed_size,
            ICON_SIZE,
            &config,
            icon_ranges
        );
    }


    {
        // 폰트 추가 (Bold)
        ImGui::Bold = io.Fonts->AddFontFromMemoryCompressedTTF(
            NEXON_Lv2_Gothic_Bold_compressed_data,
            NEXON_Lv2_Gothic_Bold_compressed_size,
            FONST_SIZE,
            NULL,
            io.Fonts->GetGlyphRangesKorean()
        );

        // 아이콘 폰트 추가
        io.Fonts->AddFontFromMemoryCompressedTTF(
            MaterialSymbolsRounded_compressed_data,
            MaterialSymbolsRounded_compressed_size,
            ICON_SIZE,
            &config,
            icon_ranges
        );
    }



    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);



    return true;
}



bool ImguiApp::run() {
    _is_running.store(true);

    while (!glfwWindowShouldClose(window) && _is_running.load()) {
        glfwPollEvents();


        // ImGui 프레임 시작
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();




        ImGuiViewport* viewport = ImGui::GetMainViewport();

        // 독스페이스 중앙 정렬을 위한 위치 계산
        docking_pos = ImVec2(
            viewport->Pos.x + (viewport->Size.x - docking_size.x) * 0.5f,
            viewport->Pos.y + (viewport->Size.y - docking_size.y) * 0.5f
        );




        // 커스텀 GUI 렌더링
        show_titlebar();
        show_dockspace();




        // 렌더링 콜백 실행 (매 프레임마다)
        {
            std::lock_guard<std::mutex> lock(callback_mutex);
            if (render_callback) {
                render_callback();
            }
        }


        // 알림 센터
        ImGui::NotificationCenter();



        // ImGui 렌더링
        ImGui::Render();
        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);




        // 화면 지우기
        glClearColor(CLEAR_COLOR.x * CLEAR_COLOR.w, CLEAR_COLOR.y * CLEAR_COLOR.w, CLEAR_COLOR.z * CLEAR_COLOR.w, CLEAR_COLOR.w);
        glClear(GL_COLOR_BUFFER_BIT);

        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());




        // 멀티 뷰포트 처리
        if (ImGui::GetIO().ConfigFlags & ImGuiConfigFlags_ViewportsEnable)
        {
            GLFWwindow* backup_current_context = glfwGetCurrentContext();
            ImGui::UpdatePlatformWindows();
            ImGui::RenderPlatformWindowsDefault();
            glfwMakeContextCurrent(backup_current_context);
        }



        // 버퍼 스왑
        glfwSwapBuffers(window);
    }

    // ImGui 정리
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();



    // GLFW 정리
    glfwDestroyWindow(window);
    glfwTerminate();



    _is_running.store(false);
    return false;
}



void ImguiApp::show_dockspace() {
    // 도킹 크기와 위치 계산
    ImVec2 docking_content_pos = ImVec2(docking_pos.x, docking_pos.y + TITLEBAR_HEIGHT);
    ImVec2 docking_content_size = ImVec2(docking_size.x, docking_size.y - TITLEBAR_HEIGHT);

    ImGui::SetNextWindowPos(docking_content_pos);
    ImGui::SetNextWindowSize(docking_content_size);
    ImGui::SetNextWindowViewport(ImGui::GetMainViewport()->ID);
    ImGui::SetNextWindowBgAlpha(0.0f); // 도킹 공간을 담는 창 자체는 투명하게 유지




    // 도킹창 플래그 설정
    ImGuiWindowFlags window_flags = ImGuiWindowFlags_NoDocking;
    window_flags |= ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoMove;
    window_flags |= ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoNavFocus;



    // 도킹창 스타일 설정
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));



    // 도킹창
    ImGui::Begin("##Main Dockspace", nullptr, window_flags);
    ImGui::PopStyleVar(3);




    // 독스페이스
    ImGuiID dockspace_id = ImGui::GetID("Main Dockspace");
    ImGui::DockSpace(dockspace_id, ImVec2(0.0f, 0.0f), ImGuiDockNodeFlags_PassthruCentralNode);




    // 현재 윈도우 크기 가져오기
    ImVec2 current_window_size = ImGui::GetWindowSize();
    // 전체 도킹 크기 (타이틀바 포함)
    ImVec2 total_docking_size = ImVec2(current_window_size.x, current_window_size.y + TITLEBAR_HEIGHT);




    // 도킹 윈도우 크기가 변경되었는지 확인
    if (total_docking_size.x != prev_docking_size.x || total_docking_size.y != prev_docking_size.y)
    {
        // GLFW 윈도우 크기를 도킹 크기 + MARGIN으로 설정
        int new_glfw_width = (int)(total_docking_size.x + DOCKSPACE_MARGIN);
        int new_glfw_height = (int)(total_docking_size.y + DOCKSPACE_MARGIN);

        glfwSetWindowSize(window, new_glfw_width, new_glfw_height);

        // 다음 프레임을 위해 도킹 크기 업데이트
        docking_size = total_docking_size;
    }



    // 현재 크기를 다음 프레임을 위해 저장
    prev_docking_size = total_docking_size;
    ImGui::End();
}



void ImguiApp::show_titlebar() {
    // X11 Display 가져오기
    Display* display = glfwGetX11Display();

    ImGui::SetNextWindowPos(docking_pos);
    ImGui::SetNextWindowSize(ImVec2(docking_size.x, TITLEBAR_HEIGHT));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 3.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, TITLEBAR_PADDING);       // 타이틀바 타이틀 레이블 패딩
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.0f, 0.0f, 0.0f, 1.0f));


    // 타이틀바 윈도우 플래그
    ImGuiWindowFlags titlebar_flags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
                                      ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoCollapse;


    // 타이틀바 윈도우
    ImGui::Begin("##TITLE_BAR", nullptr, titlebar_flags);



    // 타이틀 텍스트 (중앙 정렬)
    float text_width = ImGui::CalcTextSize(TITLE.c_str()).x;
    float title_pos_x = (docking_size.x - text_width) * 0.5f;
    ImGui::SetCursorPosX(title_pos_x);
    ImGui::PushFont(ImGui::Bold);
    ImGui::Text("%s", TITLE.c_str());
    ImGui::PopFont();



    // 닫기 버튼 (오른쪽 정렬)
    ImGui::SameLine(docking_size.x + TITLEBAR_BUTTON_OFFSET.x);
    ImGui::SetCursorPosY(TITLEBAR_BUTTON_OFFSET.y);
    if (ImGui::Button(ICON_MD_CLOSE, TITLEBAR_BUTTON_SIZE))
    {
        glfwSetWindowShouldClose(window, GLFW_TRUE); // GLFW 윈도우 종료 시그널
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
        int root_x, root_y, win_x, win_y;
        unsigned int mask_return;


        XQueryPointer(display, glfwGetX11Window(window), &root_return, &child_return, &root_x, &root_y, &win_x, &win_y, &mask_return);


        // 드래그 오프셋 = 마우스 절대 좌표 - 윈도우 절대 좌표
        drag_offset_x = root_x - window_x;
        drag_offset_y = root_y - window_y;
    }



    // 타이틀바 윈도우 드래깅 상태일 때
    if (is_dragging_titlebar)
    {
        if (ImGui::IsMouseDown(0))
        {
            // 현재 마우스 스크린 절대 좌표 가져오기
            Window root_return, child_return;
            int root_x, root_y, win_x, win_y;
            unsigned int mask_return;


            XQueryPointer(display, glfwGetX11Window(window), &root_return, &child_return, &root_x, &root_y, &win_x, &win_y, &mask_return);


            // 새로운 윈도우 위치 = 마우스 절대 좌표 - 드래그 오프셋
            int new_x = root_x - (int)drag_offset_x;
            int new_y = root_y - (int)drag_offset_y;


            glfwSetWindowPos(window, new_x, new_y);
        }
        else
        {
            // 마우스 클릭을 해제해서 드래깅 종료
            is_dragging_titlebar = false;
        }
    }



    ImGui::End();
    ImGui::PopStyleVar(2);
    ImGui::PopStyleColor(1);
}



void ImguiApp::startBackground() {
    if (render_thread.joinable()) {
        std::cout << "[Error] [ImGui App]: 에러\n";
        return;
    }


    render_thread = std::thread([this]() {
        if (init()) {
            run();
        }
    });
}



void ImguiApp::stopBackground() {
    _is_running.store(false);
    if (render_thread.joinable()) {
        render_thread.join();
    }
}
