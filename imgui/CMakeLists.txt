cmake_policy(SET CMP0072 NEW)
set(OpenGL_GL_PREFERENCE GLVND)

cmake_minimum_required(VERSION 3.10)



# ImGui 라이브러리 정의
add_library(imgui
    imgui_background.cpp
    imgui.cpp
    imgui_draw.cpp
    imgui_widgets.cpp
    imgui_tables.cpp
    imgui_demo.cpp

    ImCoolBar.cpp

    ImNotification.cpp

    implot_items.cpp
    implot.cpp

    implot3d.cpp
    implot3d_items.cpp
    implot3d_meshes.cpp

    backends/imgui_impl_glfw.cpp
    backends/imgui_impl_opengl3.cpp
)




# include 경로 설정
target_include_directories(imgui PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/backends
)





# 외부 라이브러리 연결 (ImGui 백엔드용)
find_package(glfw3 QUIET)
if(NOT glfw3_FOUND)
    message(FATAL_ERROR "\n"
            "-------------------------------------------------------\n"
            " [Error] GLFW library not found!\n"
            " GLFW 라이브러리를 찾을 수 없습니다.\n"
            " 아래 명령어를 사용하여 설치해 주세요:\n\n"
            " sudo apt install libglfw3-dev\n"
            "-------------------------------------------------------"
    )
endif()

find_package(OpenGL REQUIRED)
find_package(X11 REQUIRED)


target_link_libraries(imgui PUBLIC
    glfw
    OpenGL::GL
    X11::X11
)
