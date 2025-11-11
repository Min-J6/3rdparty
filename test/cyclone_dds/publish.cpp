#include "dds_publisher.hpp"
#include "dds_qos.hpp"
#include "PingData.h"

#include <chrono>
#include <iostream>
#include <thread>



long long getTimeStamp_ms() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()
    ).count();
}


int main() {
    std::cout << "[테스트] Publish 실행" << std::endl;


    // ----------------------------------------------
    // 퍼블리셔 생성
    //  - 토픽 이름
    //  - 메시지 타입: idl 파일에서 정의한 타입
    //  - 도메인 아이디: ros2와 맞추고 싶으면 도메인 아이디를 같게 설정
    // ----------------------------------------------
    Publisher<PingData_Msg> publisher("PingTopic",  // 토픽 이름
                &PingData_Msg_desc,                 // 메시지 타입
                0                                   // 도메인 아이디
    );



    // ----------------------------------------------
    // 메시지 발행
    // ----------------------------------------------
    PingData_Msg msg;

    for (int i = 0; i < 10; ++i)
    {
        msg.client_id = 123;
        msg.timestamp = getTimeStamp_ms(); // 데이터 설정

        publisher.publish(msg); // 퍼블리시

        std::cout << "[Info ] 발행됨: " << msg.client_id << " at " << msg.timestamp << " ms" << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }


    return 0;
}