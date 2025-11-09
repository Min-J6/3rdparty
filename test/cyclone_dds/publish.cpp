#include <chrono>
#include <iostream>
#include <thread>

#include "../../lib/dds_publisher.hpp"
#include "../../lib/dds_qos.hpp"
#include "PingData.h"

long long getTimeStamp_ms() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()
    ).count();
}


int main() {
    std::cout << "Hello, Publisher Example!" << std::endl;


    // 퍼블리셔 생성
    Publisher<PingData_Msg> publisher("PingTopic", &PingData_Msg_desc); // 토픽 생성


    PingData_Msg msg;
    msg.client_id = 123;

    for (int i = 0; i < 10; ++i) {
        msg.timestamp = getTimeStamp_ms(); // 데이터 설정

        publisher.publish(msg); // 퍼블리시

        std::cout << "Sending: client " << msg.client_id << " at " << msg.timestamp << " ms" << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }


    return 0;
}