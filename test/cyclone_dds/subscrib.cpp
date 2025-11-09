#include <iostream>
#include "../../lib/dds_subscriber.hpp"
#include "PingData.h"


void on_ping_data_received(const PingData_Msg& msg)
{
    std::cout << "Received: client " << msg.client_id << " at " << msg.timestamp << " ms" << std::endl;
}


int main()
{
    std::cout << "Hello, Subscriber Example!" << std::endl;


    Subscriber<PingData_Msg> sub("PingTopic", &PingData_Msg_desc);  // 토픽 생성
    sub.on_received = on_ping_data_received;                        // 수신시 호출되는 콜백 등록

    std::this_thread::sleep_for(std::chrono::seconds(10));

    return 0;
}