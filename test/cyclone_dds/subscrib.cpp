#include <iostream>
#include "dds_subscriber.hpp"
#include "PingData.h"
#include "String.h"


void on_ping_data_received(const std_msgs_msg_dds__String_& msg)
{
    std::cout << "[Info ] 수신됨: " << msg.data << " at " << std::endl;
}


int main()
{
    std::cout << "[테스트] Subscriber 실행" << std::endl;


    // ----------------------------------------------
    // 구독자 생성
    //  - 토픽 이름
    //  - 메시지 타입: idl 파일에서 정의한 타입
    //  - 도메인 아이디: ros2와 맞추고 싶으면 도메인 아이디를 같게 설정
    //  - ros2는 rt/를 숨겨서 퍼블리시 하나봄
    // ----------------------------------------------
    Subscriber<std_msgs_msg_dds__String_> sub("rt/chatter",   // 토픽 이름
        &std_msgs_msg_dds__String__desc,        // 메시지 타입
        0                                       // 도메인 아이디
    );




    // ----------------------------------------------
    // 메세지 수신 콜백
    // ----------------------------------------------
    sub.on_received = on_ping_data_received;    // 수신시 호출되는 콜백 등록




    std::this_thread::sleep_for(std::chrono::seconds(10));
    return 0;
}