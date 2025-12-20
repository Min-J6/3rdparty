add_subdirectory(${CMAKE_SOURCE_DIR}/3rdparty/imgui ${CMAKE_BINARY_DIR}/3rdparty/imgui)

add_executable(test_imgui example_imgui.cpp)

target_link_libraries(test_imgui imgui)