# Imgui 서브 모듈


```shell
  git clone --branch imgui --single-branch https://github.com/Min-J6/lib/tree/imgui
```

CMakeLists.txt:
```cmake
cmake_minimum_required(VERSION 3.10)
cmake_policy(SET CMP0167 NEW)


project(imgui)


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


# 서브 모듈
add_subdirectory(lib/imgui)  # <- 추가


# 실행 파일
add_executable(main
    main.cpp
)


# 라이브러리 링크
target_link_libraries(main
        imgui           # <- 추가
)
```

main.cpp:
```c++
#include "imgui.h"

int main() {

    ImGui::start("데모");

    while (ImGui::isRunning())
    {
        ImGui::context([]()
                {
                    ImGui::Begin("창1");

                    ImGui::Text("abc123");
                    ImGui::Text("ABC456");

                    ImGui::End();
                });
    }

    ImGui::stop();

    std::cout << "Hello World!" << std::endl;

    return 0;
}
```