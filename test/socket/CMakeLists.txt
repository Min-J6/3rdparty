cmake_minimum_required(VERSION 3.10)

# Socket Server 테스트
add_executable(Test_Server server.cpp)

target_include_directories(Test_Server PUBLIC
        ${CMAKE_SOURCE_DIR}/lib
)

target_link_libraries(Test_Server PUBLIC
        Boost::system Boost::regex
)


# Socker Client 테스트
add_executable(Test_Client client.cpp)

target_include_directories(Test_Client PUBLIC
        ${CMAKE_SOURCE_DIR}/lib
)

target_link_libraries(Test_Client PUBLIC
        Boost::system Boost::regex
)