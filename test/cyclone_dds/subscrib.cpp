#include <iostream>
#include "../../lib/dds_subscriber.hpp"
#include "../../lib/dds_qos.hpp"
#include "PingData.h"


void on_ping_data_received(const PingData_Msg& msg)
{
    std::cout << "[Info ] 수신됨: " << msg.client_id << " at " << msg.timestamp << " ms" << std::endl;
}


int main()
{
    std::cout << "[테스트] Subscriber 실행" << std::endl;


    // ----------------------------------------------
    // 구독자 생성
    //  - 토픽 이름
    //  - 메시지 타입: idl 파일에서 정의한 타입
    //  - 도메인 아이디: ros2와 맞추고 싶으면 도메인 아이디를 같게 설정
    // ----------------------------------------------
    Subscriber<PingData_Msg> sub("PingTopic",   // 토픽 이름
        &PingData_Msg_desc,                     // 메시지 타입
        0                                       // 도메인 아이디
    );




    // ----------------------------------------------
    // 메세지 수신 콜백
    // ----------------------------------------------
    sub.on_received = on_ping_data_received;    // 수신시 호출되는 콜백 등록




    std::this_thread::sleep_for(std::chrono::seconds(10));
    return 0;
}