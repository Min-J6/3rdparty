#include "SocketClient.hpp"
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


// ----------------------------------------------
// 클라이언트 메시지 수신시 호출되는 콜백
// ----------------------------------------------
void on_message_handler(const std::vector<char>& message) {
    // std::cout << "[Info ] [Client] 받음: " << bytesToHexString(message) << std::endl;
    std::cout << "[Info ] [Client] 받음: " << std::string(message.begin(), message.end());
}


// ----------------------------------------------
// 클라이언트 연결 끊기시 호출되는 콜백
// ----------------------------------------------
void on_disconnect_handler() {
    std::cout << "[Info ] [Client] 서버와 연결 끊어짐" << std::endl;
}


// ----------------------------------------------
// 클라이언트 연결시 호출되는 콜백
// ----------------------------------------------
void on_connect_handler()
{
    std::cout << "[Info ] [Client] 서버와 연결됨" << std::endl;
}



int main(){
    std::cout << "[테스트] Client 실행" << std::endl;

// ----------------------------------------------
//  | 클라이언트 생성
// ----------------------------------------------
    Client client;



// ----------------------------------------------
//  | 클라이언트 콜백 등록 후 연결
// ----------------------------------------------
    client.on_receive    = on_message_handler;
    client.on_disconnect = on_disconnect_handler;
    client.on_connect    = on_connect_handler;

    client.connect("127.0.0.1", 5000);


// ----------------------------------------------
//  | 터미널에서 입력받아서 서버로 전송
// ----------------------------------------------
    for (std::string line; std::getline(std::cin, line);) {
        if (line == "quit" || line == "exit") {
            break;
        }
        line.push_back('\n');
        client.send(line);
    }


// ----------------------------------------------
//  | 클라이언트 연결 종료
// ----------------------------------------------
    client.disconnect();
    return 0;
}