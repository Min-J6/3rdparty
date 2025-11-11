cmake_minimum_required(VERSION 3.10)
project(ImGuiTransparentDemo)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_subdirectory(lib/imgui)

add_executable(main
    main.cpp
)

add_subdirectory(lib/CycloneDDS)

target_link_libraries(main PRIVATE imgui)




find_package(Boost REQUIRED COMPONENTS system regex)


add_subdirectory(test)