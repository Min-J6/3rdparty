#include "SocketServer.hpp"
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
// 세션 수신 콜백
// ----------------------------------------------
void receive_callback(std::shared_ptr<Session> s, const std::vector<char>& data)
{
    std::string message(data.begin(), data.end());

    auto msg = bytesToHexString(data);
    std::cout << "[Info ] [Session] "
              << s->socket().remote_endpoint()
              << ": " << msg << std::endl;

    // 에코
    s->send(data);
}


// ----------------------------------------------
// 세션 에러 콜백
// ----------------------------------------------
void error_callback(std::shared_ptr<Session> session, const boost::system::error_code& ec)
{
    std::cerr << "[Error] [Session] "
              << session->socket().remote_endpoint()
              << " 에러: " << ec.message() << std::endl;
}


// ----------------------------------------------
// 서버 에러 콜백
// ----------------------------------------------
void error_server_callback(const boost::system::error_code& ec)
{
    std::cout << "[Info ] [Server] 종료" << std::endl;
}





int main() {
    std::cout << "[테스트] Server 실행" << std::endl;

// ----------------------------------------------
//  | 서버 생성
// ----------------------------------------------
    Server server(5000);


// ----------------------------------------------
//  | 클라이언트 연결시 호출되는 콜백 등록
// ----------------------------------------------
    server.on_accept = [](std::shared_ptr<Session> session)
    {
        // 환영 메시지 전송
        session->send("서버 연결 성공\n");
        session->on_receive = receive_callback;  // 세션 수신 콜백
        session->on_error   = error_callback;    // 세션 에러 콜백

    };

    server.on_error = error_server_callback;     // 서버 에러 콜백



    server.run(); // 블락됨
    return 0;
}