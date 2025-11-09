#include <iostream>
#include "../../lib/dds_subscriber.hpp"
#include "../../lib/dds_qos.hpp"
#include "PingData.h"


void on_ping_data_received(const PingData_Msg& msg)
{
    std::cout << "Received: client " << msg.client_id << " at " << msg.timestamp << " ms" << std::endl;
}


int main()
{
    std::cout << "Hello, Subscriber Example!" << std::endl;



    CycloneDdsQos qos;
    qos.reliability_reliable()       // [연결 후] 유실 없이 받겠다 (재전송 O)
    .durability_transient_local() // [연결 전] 과거 데이터도 받겠다 (Latch O)
    .history_keep_last(10);       // [내구성용] 과거 데이터 중 최신 10개만 저장



    Subscriber<PingData_Msg> sub("PingTopic", &PingData_Msg_desc);  // 토픽 생성
    sub.on_received = on_ping_data_received;                        // 수신시 호출되는 콜백 등록

    std::this_thread::sleep_for(std::chrono::seconds(10));

    return 0;
}