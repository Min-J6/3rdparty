cmake_minimum_required(VERSION 3.10)
project(ImGuiTransparentDemo)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)



find_package(Boost REQUIRED COMPONENTS system regex)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)



add_subdirectory(lib/imgui)
add_subdirectory(lib/CycloneDDS)



add_executable(main
    main.cpp
)


target_link_libraries(main PRIVATE imgui Eigen3::Eigen)



add_subdirectory(test)