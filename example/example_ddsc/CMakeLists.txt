add_subdirectory(${CMAKE_SOURCE_DIR}/3rdparty/ddsc  ${CMAKE_BINARY_DIR}/3rdparty/ddsc)

add_executable(example_ddsc example_ddsc.cpp)

target_link_libraries(example_ddsc ddsc)