#include "../../lib/SocketClient.hpp"




void on_message_handler(const std::string& message) {
    std::cout << "[Info ] [Client] 받음: " << message << std::endl;
}

void on_disconnect_handler() {
    std::cout << "[Info ] [Client] 서버와 연결 끊어짐" << std::endl;
}

void on_connect_handler()
{
    std::cout << "[Info ] [Client] 서버와 연결됨" << std::endl;
}

int main(){
    Client client;

    client.on_receive = on_message_handler;
    client.on_disconnect = on_disconnect_handler;
    client.on_connect = on_connect_handler;


    client.connect("127.0.0.1", 12345);

    try {
        for (std::string line; std::getline(std::cin, line);) {
            if (line == "quit" || line == "exit") {
                break;
            }
            client.send(line);
        }
    } catch (const std::exception& e) {
        std::cerr << "오류 발생: " << e.what() << std::endl;
    }


    client.close();
    return 0;
}
