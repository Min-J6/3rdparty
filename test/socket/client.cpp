#include "../../lib/SocketClient.hpp"
#include <sstream>
#include <iomanip>

std::string bytesToHexString(const std::vector<char>& data) {
    std::stringstream ss;
    ss << std::hex << std::uppercase; // 16진수, 대문자로 설정

    for (size_t i = 0; i < data.size(); ++i) {
        ss << std::setw(2) << std::setfill('0') << static_cast<int>(static_cast<unsigned char>(data[i]));
        if (i < data.size() - 1) {
            ss << " ";
        }
    }

    return ss.str();
}

void on_message_handler(const std::vector<char>& message) {
    // std::cout << "[Info ] [Client] 받음: " << bytesToHexString(message) << std::endl;
    std::cout << "[Info ] [Client] 받음: " << std::string(message.begin(), message.end());
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


    client.connect("127.0.0.1", 5000);

    try {
        for (std::string line; std::getline(std::cin, line);) {
            if (line == "quit" || line == "exit") {
                break;
            }
            line.push_back('\n');
            client.send(line);
        }
    } catch (const std::exception& e) {
        std::cerr << "오류 발생: " << e.what() << std::endl;
    }


    client.close();
    return 0;
}
