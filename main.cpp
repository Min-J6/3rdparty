#include "dds_publisher.hpp"
#include "example.h"

int main() {


    Publisher<Message> pub("rt/example_topic", &Message_desc);

    for (int i = 0; i < 100; ++i)
    {
        Message msg;
        msg.text =  ("Hello, World!" + std::to_string(i)).data();
        pub.publish(msg);
    }

    return 0;
}
