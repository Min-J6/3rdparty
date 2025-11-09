#include "../../lib/SocketServer.hpp"
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


int main() {
    try
    {
        Server server(5000);

        // 새 클라이언트가 연결될 때마다 호출됨
        server.on_accept = [](std::shared_ptr<Session> session)
        {
            // 환영 메시지 전송
            session->send("서버 연결 성공\n");

            // 세션 수신 콜백
            session->on_receive = [](std::shared_ptr<Session> s, const std::vector<char>& data)
            {
                std::string message(data.begin(), data.end());

                auto msg = bytesToHexString(data);
                std::cout << "[Info ] [Session] " << s->socket().remote_endpoint() << ": " << msg << std::endl;

                // 에코
                s->send(data);
            };

            // 세션 에러 콜백
            session->on_error = [](std::shared_ptr<Session> s, const boost::system::error_code& ec)
            {

                std::cerr << "[Error] [Session]" << s->socket().remote_endpoint() << "세션 에러: " <<  ec.message() << std::endl;

            };
        };

        server.on_error = [](const boost::system::error_code& ec)
        {
            std::cerr << "[Error] [Server] Accept 에러: " << ec.message() << std::endl;
        };

        server.run();
    }
    catch (const std::exception& e)
    {
        std::cerr << "예외 발생: " << e.what() << std::endl;
    }

    return 0;
}