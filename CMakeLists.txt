cmake_minimum_required(VERSION 3.10)
project(ImGuiTransparentDemo)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_subdirectory(lib/imgui)

add_executable(main
    main.cpp
)

add_subdirectory(lib/CycloneDDS)

target_link_libraries(main PRIVATE imgui CycloneDDS)




find_package(Boost REQUIRED COMPONENTS system regex)
# test
add_executable( server test/socket/server.cpp )
add_executable( client test/socket/client.cpp )

target_link_libraries(server Boost::system Boost::regex)
target_link_libraries(client Boost::system Boost::regex)
