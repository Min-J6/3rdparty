cmake_policy(SET CMP0072 NEW)
set(OpenGL_GL_PREFERENCE GLVND)

cmake_minimum_required(VERSION 3.10)



# ImGui 라이브러리 정의
add_library(imgui
    imgui_app.cpp
    imgui.cpp
    imgui_draw.cpp
    imgui_widgets.cpp
    imgui_tables.cpp
    ImCoolBar.cpp
    backends/imgui_impl_glfw.cpp
    backends/imgui_impl_opengl3.cpp
)




# include 경로 설정
target_include_directories(imgui PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/backends
)





# 외부 라이브러리 연결 (ImGui 백엔드용)
find_package(glfw3 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(X11 REQUIRED)


target_link_libraries(imgui PUBLIC
    glfw
    OpenGL::GL
    X11::X11
)
