cmake_minimum_required(VERSION 3.10)
project(ImGuiTransparentDemo)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- 변경: lib/imgui 폴더 추가 ---
add_subdirectory(lib/imgui)

add_executable(main
    main.cpp
    vSerial.cpp
)

# ImGui 라이브러리 연결
target_link_libraries(main PRIVATE imgui)
