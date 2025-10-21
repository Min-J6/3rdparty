#pragma once


#include <iostream>
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>
#include <functional>
#include <unordered_map>
#include <stdexcept>
#include <csignal>
#include <chrono>

// POSIX/BSD 소켓 라이브러리
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>






// Python 코드의 타입을 C++로 재정의
using ClientID = int;
using Address = std::pair<std::string, int>;

class SocketServer {
public:
    // 콜백 함수 타입 정의
    using ConnectCallback = std::function<void(ClientID, Address)>;
    using MessageCallback = std::function<void(ClientID, Address, const std::vector<char>&)>;
    using DisconnectCallback = std::function<void(ClientID, Address)>;

    SocketServer(const std::string& host = "127.0.0.1", int port = 5000, int recv_size = 4096)
        : host(host), port(port), recv_size(recv_size), server_sock(-1), next_client_id(1) {
        _running.store(false);
    }

    ~SocketServer() {
        stop();
    }

    // 콜백 설정
    ConnectCallback on_connect;
    MessageCallback on_message;
    DisconnectCallback on_disconnect;



    // 백그라운드에서 서버 실행
    void open(int backlog = 5) {
        // 소켓 생성
        server_sock = socket(AF_INET, SOCK_STREAM, 0);
        if (server_sock < 0) {
            throw std::runtime_error("[Error] [Server] 소켓 생성 에러");
        }



        // 비정상 종료시 즉시 포트를 재사용 할 수 있게 설정
        int opt = 1;
        setsockopt(server_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));



        // 소켓 주소, 포트 설정
        sockaddr_in server_addr{};
        server_addr.sin_family = AF_INET;
        inet_pton(AF_INET, host.c_str(), &server_addr.sin_addr);
        server_addr.sin_port = htons(port);


        // 바인딩
        if (bind(server_sock, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
            close(server_sock);
            throw std::runtime_error("[Error] [Server] 바인딩 에러");
        }



        // 리슨
        if (listen(server_sock, backlog) < 0) {
            close(server_sock);
            throw std::runtime_error("[Error] [Server] 리슨 에러");
        }



        // 서버 실행 완료
        _running.store(true);
        _accept_thread = std::thread(&SocketServer::_accept_loop, this);

        std::cout << "[Info ] [Server] 서버 시작:" << host << ":" << port << "\n";
    }


    // 서버 종료
    void stop() {
        if (!_running.exchange(false)) {
            return; // 이미 중지됨
        }



        // accept 루프를 중단시키기 위해 소켓을 닫음
        if (server_sock != -1) {
            shutdown(server_sock, SHUT_RDWR);
            close(server_sock);
            server_sock = -1;
        }



        // 모든 클라이언트 연결 종료
        std::lock_guard<std::mutex> lock(_lock);
        for (auto const& [cid, sock] : _clients) {
            shutdown(sock, SHUT_RDWR);
            close(sock);
        }


        // 정리
        _clients.clear();
        _addrs.clear();
        _client_threads.clear();


        std::cout << "[Info ] [Server] 서버 종료\n";
    }


    // 특정 클라이언트에게 패킷 보내기
    bool send_to(ClientID client_id, const std::string& data) {
        int sock = -1;
        {
            std::lock_guard<std::mutex> lock(_lock);
            auto it = _clients.find(client_id);
            if (it == _clients.end()) {
                return false; //
            }
            sock = it->second;
        }



        if (send(sock, data.c_str(), data.length(), 0) < 0) {
            return false;
        }


        return true;
    }


    // 모든 클라이언트에게 패킷 보내기
    void broadcast(const std::string& data) {
        std::lock_guard<std::mutex> lock(_lock);

        for (auto const& [cid, sock] : _clients) {
            send(sock, data.c_str(), data.length(), 0);
        }
    }


    // 클라이언트 리스트 리턴
    std::unordered_map<ClientID, Address> list_clients() {
        std::lock_guard<std::mutex> lock(_lock);
        return _addrs;
    }



private:
    std::string host;
    int port;
    int recv_size;

    int server_sock;
    std::atomic<bool> _running;
    std::thread _accept_thread;

    std::unordered_map<ClientID, int> _clients;
    std::unordered_map<ClientID, Address> _addrs;
    std::unordered_map<ClientID, std::thread> _client_threads;
    std::mutex _lock;
    std::atomic<ClientID> next_client_id;

    void _accept_loop() {
        while (_running.load()) {
            sockaddr_in client_addr{};
            socklen_t client_len = sizeof(client_addr);
            int client_sock = accept(server_sock, (struct sockaddr*)&client_addr, &client_len);

            if (client_sock < 0) {
                if (_running.load()) { // 서버가 실행 중인데 accept 실패
                    std::cout << "[Info ] [Server] accept 에러\n";
                }
                break; // 루프 종료
            }




            // 클라이언트 정보
            ClientID cid = next_client_id++;
            char client_ip[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, &client_addr.sin_addr, client_ip, INET_ADDRSTRLEN);
            int client_port = ntohs(client_addr.sin_port);
            Address addr = {std::string(client_ip), client_port};



            // 클라이언트 추가
            {
                std::lock_guard<std::mutex> lock(_lock);
                _clients[cid] = client_sock;
                _addrs[cid] = addr;
            }



            // 연결
            if (on_connect) {
                try {
                    on_connect(cid, addr);
                } catch (const std::exception& e) {
                    std::cerr << "[Info ] [Server] 클라이언트 연결 중 에러 발생: " << e.what() << std::endl;
                }
            }


            std::lock_guard<std::mutex> lock(_lock);
            _client_threads[cid] = std::thread(&SocketServer::_client_loop, this, cid);
            _client_threads[cid].detach(); // 스레드를 분리하여 독립적으로 실행
        }
    }

    void _client_loop(ClientID client_id) {
        int sock;
        Address addr;
        {
            std::lock_guard<std::mutex> lock(_lock);
            sock = _clients.at(client_id);
            addr = _addrs.at(client_id);
        }

        std::vector<char> buffer(recv_size);
        while (_running.load()) {
            ssize_t bytes_received = recv(sock, buffer.data(), recv_size, 0);

            if (bytes_received <= 0) { // 0이면 연결 종료, -1이면 에러
                break;
            }

            if (on_message) {
                try {
                    std::vector<char> received_data(buffer.begin(), buffer.begin() + bytes_received);
                    on_message(client_id, addr, received_data);
                } catch (const std::exception& e) {
                    std::cerr << "[Info ] [Server] 메시지를 받는 중 에러 발생: " << e.what() << std::endl;
                }
            }
        }

        // 정리
        close(sock);
        {
            std::lock_guard<std::mutex> lock(_lock);
            _clients.erase(client_id);
            _addrs.erase(client_id);
            _client_threads.erase(client_id);
        }

        if (on_disconnect) {
            try {
                on_disconnect(client_id, addr);
            } catch (const std::exception& e) {
                std::cerr << "[Info ] [Server] 서버 종료 중 에러 발생: " << e.what() << std::endl;
            }
        }
    }
};



/*
 *
int main() {
    // 1. 서버 객체 생성
    SocketServer server("127.0.0.1", 5000);



    // 2. 콜백 함수 정의 및 할당
    server.on_connect = [](ClientID cid, Address addr) {
        std::cout << "[Info ][Server] Client 연결됨: #" << cid
                  << " from " << addr.first << ":" << addr.second << std::endl;
    };



    server.on_message = [&](ClientID cid, Address addr, const std::vector<char>& msg) {
        std::string text(msg.begin(), msg.end());
        std::cout << "[Info ] [Server] 메시지 수신: [" << cid << " " << addr.first << ":" << addr.second << "] -> '" << text << "'" << std::endl;

        // 받은 클라이언트에게 그대로 에코
        server.send_to(cid, text);
    };



    server.on_disconnect = [](ClientID cid, Address addr) {
        std::cout << "[Info] [Server] Client가 연결 종료: #" << cid
                  << " from " << addr.first << ":" << addr.second << std::endl;
    };



    // 3. 서버 실행
    try {
        // 이 함수는 Ctrl+C로 종료할 때까지 블로킹됩니다.
        server.open();
    } catch (const std::exception& e) {
        std::cerr << "[Error] [Server] 서버 에러: " << e.what() << std::endl;
        server.stop();
    }

    return 0;
}

 */