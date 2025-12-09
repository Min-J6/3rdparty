#include <chrono>
#include <iostream>
#include <thread>

#include "dds_publisher.hpp"
#include "ping.h"

int main()
{
    std::cout << "Hello World!" << std::endl;


    Publisher<PingData_Msg> pub("rt/ping", &PingData_Msg_desc, 0);


    for (int i = 0; i < 100; i++) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        PingData_Msg msg;
        msg.timestamp = i;
        msg.client_id = 0;
        pub.publish(msg);
    }


    return 0;
}