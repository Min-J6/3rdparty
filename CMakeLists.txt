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


# [TEST] Socket
add_executable( test_server test/socket/server.cpp )
add_executable( test_client test/socket/client.cpp )
target_link_libraries(test_server Boost::system Boost::regex)
target_link_libraries(test_client Boost::system Boost::regex)


# [TEST] Cyclone DDS
add_executable(test_pulish test/cyclone_dds/publish.cpp)
idlc_generate(
    TARGET Ping_idl
    FILES "${CMAKE_CURRENT_SOURCE_DIR}/test/cyclone_dds/idl/PingData.idl"
    WARNINGS no-implicit-extensibility
)
target_link_libraries(test_pulish PUBLIC CycloneDDS::ddsc Ping_idl)
target_include_directories(test_pulish PUBLIC ${CMAKE_CURRENT_BINARY_DIR})



add_executable(test_subscribe test/cyclone_dds/subscrib.cpp)
target_link_libraries(test_subscribe PUBLIC CycloneDDS::ddsc Ping_idl)
target_include_directories(test_subscribe PUBLIC ${CMAKE_CURRENT_BINARY_DIR})