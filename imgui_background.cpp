#include "imgui_background.h"


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
#include "implot.h"
#include "implot3d.h"

// 윈도우 크기
std::string TITLE               = "DEMO";
float WINDOW_WIDTH              = 1280;
float WINDOW_HEIGHT             = 720;


const ImVec4 CLEAR_COLOR        = ImVec4(0.1f, 0.1f, 0.1f, 0.7f);
const float DOCKSPACE_MARGIN    = 1.0f;
const float TITLEBAR_HEIGHT     = 30.0f;


// 폰트 관련 상수
const float FONST_SIZE          = 16.0f;
const float ICON_SIZE           = 20.0f;
const ImVec2 GLYPH_OFFSET       = ImVec2(0.5f, 3.f);


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



ImGuiBackground& ImGuiBackground::getInstance() {
    static ImGuiBackground instance;
    return instance;
}



void ImGuiBackground::show_imgui(std::function<void()> func) {
    std::lock_guard<std::mutex> lock(getInstance().callback_mutex);
    getInstance().render_callback = func;
}



void ImGuiBackground::start_background(const std::string& title, const ImVec2& size) {
    TITLE = title;
    WINDOW_WIDTH = size.x;
    WINDOW_HEIGHT = size.y;

    docking_size             = ImVec2(WINDOW_WIDTH - DOCKSPACE_MARGIN, WINDOW_HEIGHT - DOCKSPACE_MARGIN);
    prev_docking_size        = docking_size;
    docking_pos              = ImVec2(0, 0);

    getInstance()._start_background();

    while (!ImGuiBackground::is_running()) {
        // 쓰레드가 켜질 때까지 대기
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}



void ImGuiBackground::stop_background() {
    getInstance().stopBackground();
}



bool ImGuiBackground::is_running() {
    return getInstance()._is_running.load();
}



bool ImGuiBackground::init() {
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
    ImPlot::CreateContext();
    ImPlot3D::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;




    // Imgui 스타일 설정
    ImGuiStyle& style = ImGui::GetStyle();
    ImVec4* colors = style.Colors;

    // Base colors for a pleasant and modern dark theme with dark accents
    colors[ImGuiCol_Text] = ImVec4(0.92f, 0.93f, 0.94f, 1.00f);                  // Light grey text for readability
    colors[ImGuiCol_TextDisabled] = ImVec4(0.50f, 0.52f, 0.54f, 1.00f);          // Subtle grey for disabled text
    colors[ImGuiCol_WindowBg] = ImVec4(0.14f, 0.14f, 0.16f, 0.5f);              // Dark background with a hint of blue
    colors[ImGuiCol_ChildBg] = ImVec4(0.16f, 0.16f, 0.18f, 1.00f);               // Slightly lighter for child elements
    colors[ImGuiCol_PopupBg] = ImVec4(0.18f, 0.18f, 0.20f, 1.00f);               // Popup background
    colors[ImGuiCol_Border] = ImVec4(0.28f, 0.29f, 0.30f, 0.60f);                // Soft border color
    colors[ImGuiCol_BorderShadow] = ImVec4(0.00f, 0.00f, 0.00f, 0.00f);          // No border shadow
    colors[ImGuiCol_FrameBg] = ImVec4(0.20f, 0.22f, 0.24f, 1.00f);               // Frame background
    colors[ImGuiCol_FrameBgHovered] = ImVec4(0.22f, 0.24f, 0.26f, 1.00f);        // Frame hover effect
    colors[ImGuiCol_FrameBgActive] = ImVec4(0.24f, 0.26f, 0.28f, 1.00f);         // Active frame background
    colors[ImGuiCol_TitleBg] = ImVec4(0.14f, 0.14f, 0.16f, 1.00f);               // Title background
    colors[ImGuiCol_TitleBgActive] = ImVec4(0.16f, 0.16f, 0.18f, 1.00f);         // Active title background
    colors[ImGuiCol_TitleBgCollapsed] = ImVec4(0.14f, 0.14f, 0.16f, 1.00f);      // Collapsed title background
    colors[ImGuiCol_MenuBarBg] = ImVec4(0.20f, 0.20f, 0.22f, 1.00f);             // Menu bar background
    colors[ImGuiCol_ScrollbarBg] = ImVec4(0.16f, 0.16f, 0.18f, 1.00f);           // Scrollbar background
    colors[ImGuiCol_ScrollbarGrab] = ImVec4(0.24f, 0.26f, 0.28f, 1.00f);         // Dark accent for scrollbar grab
    colors[ImGuiCol_ScrollbarGrabHovered] = ImVec4(0.28f, 0.30f, 0.32f, 1.00f);  // Scrollbar grab hover
    colors[ImGuiCol_ScrollbarGrabActive] = ImVec4(0.32f, 0.34f, 0.36f, 1.00f);   // Scrollbar grab active
    colors[ImGuiCol_CheckMark] = ImVec4(0.46f, 0.56f, 0.66f, 1.00f);             // Dark blue checkmark
    colors[ImGuiCol_SliderGrab] = ImVec4(0.36f, 0.46f, 0.56f, 1.00f);            // Dark blue slider grab
    colors[ImGuiCol_SliderGrabActive] = ImVec4(0.40f, 0.50f, 0.60f, 1.00f);      // Active slider grab
    colors[ImGuiCol_Button] = ImVec4(0.24f, 0.34f, 0.44f, 1.00f);                // Dark blue button
    colors[ImGuiCol_ButtonHovered] = ImVec4(0.28f, 0.38f, 0.48f, 1.00f);         // Button hover effect
    colors[ImGuiCol_ButtonActive] = ImVec4(0.32f, 0.42f, 0.52f, 1.00f);          // Active button
    colors[ImGuiCol_Header] = ImVec4(0.24f, 0.34f, 0.44f, 1.00f);                // Header color similar to button
    colors[ImGuiCol_HeaderHovered] = ImVec4(0.28f, 0.38f, 0.48f, 1.00f);         // Header hover effect
    colors[ImGuiCol_HeaderActive] = ImVec4(0.32f, 0.42f, 0.52f, 1.00f);          // Active header
    colors[ImGuiCol_Separator] = ImVec4(0.28f, 0.29f, 0.30f, 1.00f);             // Separator color
    colors[ImGuiCol_SeparatorHovered] = ImVec4(0.46f, 0.56f, 0.66f, 1.00f);      // Hover effect for separator
    colors[ImGuiCol_SeparatorActive] = ImVec4(0.46f, 0.56f, 0.66f, 1.00f);       // Active separator
    colors[ImGuiCol_ResizeGrip] = ImVec4(0.36f, 0.46f, 0.56f, 1.00f);            // Resize grip
    colors[ImGuiCol_ResizeGripHovered] = ImVec4(0.40f, 0.50f, 0.60f, 1.00f);     // Hover effect for resize grip
    colors[ImGuiCol_ResizeGripActive] = ImVec4(0.44f, 0.54f, 0.64f, 1.00f);      // Active resize grip
    colors[ImGuiCol_Tab] = ImVec4(0.20f, 0.22f, 0.24f, 1.00f);                   // Inactive tab
    colors[ImGuiCol_TabHovered] = ImVec4(0.28f, 0.38f, 0.48f, 1.00f);            // Hover effect for tab
    colors[ImGuiCol_TabActive] = ImVec4(0.24f, 0.34f, 0.44f, 1.00f);             // Active tab color
    colors[ImGuiCol_TabUnfocused] = ImVec4(0.20f, 0.22f, 0.24f, 1.00f);          // Unfocused tab
    colors[ImGuiCol_TabUnfocusedActive] = ImVec4(0.24f, 0.34f, 0.44f, 1.00f);    // Active but unfocused tab
    colors[ImGuiCol_PlotLines] = ImVec4(0.46f, 0.56f, 0.66f, 1.00f);             // Plot lines
    colors[ImGuiCol_PlotLinesHovered] = ImVec4(0.46f, 0.56f, 0.66f, 1.00f);      // Hover effect for plot lines
    colors[ImGuiCol_PlotHistogram] = ImVec4(0.36f, 0.46f, 0.56f, 1.00f);         // Histogram color
    colors[ImGuiCol_PlotHistogramHovered] = ImVec4(0.40f, 0.50f, 0.60f, 1.00f);  // Hover effect for histogram
    colors[ImGuiCol_TableHeaderBg] = ImVec4(0.20f, 0.22f, 0.24f, 1.00f);         // Table header background
    colors[ImGuiCol_TableBorderStrong] = ImVec4(0.28f, 0.29f, 0.30f, 1.00f);     // Strong border for tables
    colors[ImGuiCol_TableBorderLight] = ImVec4(0.24f, 0.25f, 0.26f, 1.00f);      // Light border for tables
    colors[ImGuiCol_TableRowBg] = ImVec4(0.20f, 0.22f, 0.24f, 1.00f);            // Table row background
    colors[ImGuiCol_TableRowBgAlt] = ImVec4(0.22f, 0.24f, 0.26f, 1.00f);         // Alternate row background
    colors[ImGuiCol_TextSelectedBg] = ImVec4(0.24f, 0.34f, 0.44f, 0.35f);        // Selected text background
    colors[ImGuiCol_DragDropTarget] = ImVec4(0.46f, 0.56f, 0.66f, 0.90f);        // Drag and drop target
    colors[ImGuiCol_NavHighlight] = ImVec4(0.46f, 0.56f, 0.66f, 1.00f);          // Navigation highlight
    colors[ImGuiCol_NavWindowingHighlight] = ImVec4(1.00f, 1.00f, 1.00f, 0.70f); // Windowing highlight
    colors[ImGuiCol_NavWindowingDimBg] = ImVec4(0.80f, 0.80f, 0.80f, 0.20f);     // Dim background for windowing
    colors[ImGuiCol_ModalWindowDimBg] = ImVec4(0.80f, 0.80f, 0.80f, 0.35f);      // Dim background for modal windows

    // Style adjustments
    style.WindowPadding = ImVec2(8.00f, 8.00f);
    style.FramePadding = ImVec2(5.00f, 2.00f);
    style.CellPadding = ImVec2(6.00f, 6.00f);
    style.ItemSpacing = ImVec2(6.00f, 6.00f);
    style.ItemInnerSpacing = ImVec2(6.00f, 6.00f);
    style.TouchExtraPadding = ImVec2(0.00f, 0.00f);
    style.IndentSpacing = 25;
    style.ScrollbarSize = 11;
    style.GrabMinSize = 10;
    style.WindowBorderSize = 1;
    style.ChildBorderSize = 1;
    style.PopupBorderSize = 1;
    style.FrameBorderSize = 1;
    style.TabBorderSize = 1;
    style.WindowRounding = 3;
    style.ChildRounding = 4;
    style.FrameRounding = 3;
    style.PopupRounding = 4;
    style.ScrollbarRounding = 9;
    style.GrabRounding = 3;
    style.LogSliderDeadzone = 4;
    style.TabRounding = 4;



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



bool ImGuiBackground::run() {
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
    ImPlot3D::DestroyContext();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();



    // GLFW 정리
    glfwDestroyWindow(window);
    glfwTerminate();



    _is_running.store(false);
    return false;
}



void ImGuiBackground::show_dockspace() {
    // 도킹 크기와 위치 계산
    ImVec2 docking_content_pos = ImVec2(docking_pos.x, docking_pos.y + TITLEBAR_HEIGHT + 1.5f);
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



void ImGuiBackground::show_titlebar() {
    // X11 Display 가져오기
    Display* display = glfwGetX11Display();

    ImGui::SetNextWindowPos(docking_pos);
    ImGui::SetNextWindowSize(ImVec2(docking_size.x, TITLEBAR_HEIGHT));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 3.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, TITLEBAR_PADDING);       // 타이틀바 타이틀 레이블 패딩
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.0f, 0.0f, 0.0f, 0.5f));


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



void ImGuiBackground::_start_background() {
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



void ImGuiBackground::stopBackground() {
    _is_running.store(false);
    if (render_thread.joinable()) {
        render_thread.join();
    }
}


namespace ImGui
{
    void start(const char* title, const ImVec2& size)
    {
        ImGuiBackground::start_background(title, size);
    }
    void stop()
    {
        ImGuiBackground::stop_background();
    }
    void context(std::function<void()> render_callback)
    {
        ImGuiBackground::show_imgui(render_callback);
    }
    bool isRunning()
    {
        return ImGuiBackground::is_running();
    }
}
