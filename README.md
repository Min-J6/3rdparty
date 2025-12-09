# Cyclone DDS Ping Demo
*   **Cyclone DDS** 라이브러리를 서브 모듈로 포함하여 빌드합니다.
*   IDL(Interface Definition Language)을 사용하여 데이터 타입을 정의하고, 이를 통해 코드를 생성합니다.
*   ROS2와의 상호 운용성을 테스트할 수 있는 구조를 가지고 있습니다.

## 프로젝트 구조

```text
ImGuiThread/
├── CycloneDDS/         # 3rdparty 라이브러리
├── idl/
│   └── ping.idl        # 메시지 정의 파일
├── CMakeLists.txt      # 빌드 설정
├── main_pub.cpp        # Publisher 예제 소스
└── main_sub.cpp        # Subscriber 예제 소스
```


## 1. 3rdparty 설정
Cyclone DDS 라이브러리 클론 (프로젝트 루트에서 실행)

```bash
  # 프로젝트 디렉토리에 3rdparty가 생김
  git clone --branch ddsc --single-branch https://github.com/Min-J6/3rdparty.git
```

> **참고:** 현재 설정으로 `ros2 topic list`에는 토픽이 노출되지만, ROS2 시스템이 `ping.idl`의 정의를 알지 못해 메시지 내용을 직접 읽을 수는 없습니다.\
> 반면, ROS2의 표준 메시지는 대부분 IDL을 지원하므로 ROS2에서 발행한 데이터를 이 프로젝트에서 수신하는 것은 비교적 수월합니다.

<br>

## 2. CMakeLists.txt 설정

```cmake
cmake_minimum_required(VERSION 3.10)
cmake_policy(SET CMP0167 NEW)

project(main)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Lib 추가
add_subdirectory(3rdparty/CycloneDDS)


# IDL 코드 생성
idlc_generate(
        TARGET Ping_idl
        FILES "${CMAKE_CURRENT_SOURCE_DIR}/idl/ping.idl" # idl 파일
        WARNINGS no-implicit-extensibility
)


# Publisher
add_executable(main_pub main_pub.cpp)
target_link_libraries(main_pub 
        PRIVATE 
        ddsc     # cyclonedds
        Ping_idl # idl
)


# Subscriber
add_executable(main_sub main_sub.cpp)
target_link_libraries(main_sub 
        PRIVATE 
        ddsc     # cyclonedds
        Ping_idl # idl
)
```

<br>

## 3. IDL 정의 (idl/ping.idl)

```c++
module PingData
{
  struct Msg
  {
    @key
    long client_id;      // Unique ID
    long long timestamp; // ms
  };
};
```

<br>

## 4. 예제 코드

### Publisher

```c++
// main_pub.cpp

#include <chrono>
#include <iostream>
#include <thread>

#include "dds_publisher.hpp"
#include "ping.h"

int main()
{
    Publisher<PingData_Msg> pub("rt/ping", &PingData_Msg_desc, 0); // topic

    for (int i = 0; i < 100; i++) 
    {    
        // 메세지 생성
        PingData_Msg msg;
        msg.timestamp = i;
        msg.client_id = 0;
        
        pub.publish(msg); // publish
        
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    return 0;
}
```

<br>

### Subscriber
```c++
// main_sub.cpp

#include "dds_subscriber.hpp"
#include "ping.h"
#include <iostream>

// 토픽 수신시 실행될 콜백
void recive_callback(const PingData_Msg &msg) 
{
    std::cout << "Received PingData_Msg: " << msg.timestamp << std::endl;
}

int main() 
{
    Subscriber<PingData_Msg> sub("rt/ping", &PingData_Msg_desc, 0); // topic
    sub.on_received = recive_callback;

    while (true) 
    {
        // keep running
    }
}
```