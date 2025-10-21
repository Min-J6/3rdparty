// SocketClient.h (work_guard 적용 최종 버전)
#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <functional>
#include <thread>
#include <atomic>
#include <memory>
#include <deque>
#include <future>

#include <boost/asio.hpp>
#include <boost/system/error_code.hpp>

using boost::asio::ip::tcp;

class SocketClient : public std::enable_shared_from_this<SocketClient> {
public:
    using ConnectCallback = std::function<void()>;
    using MessageCallback = std::function<void(const std::vector<char>&)>;
    using DisconnectCallback = std::function<void()>;

    SocketClient(const std::string& host = "127.0.0.1", int port = 5000)
        : host_(host),
          port_(std::to_string(port)),
          socket_(io_context_),
          resolver_(io_context_),
          work_guard_(boost::asio::make_work_guard(io_context_)),
          write_strand_(boost::asio::make_strand(io_context_)) {
        _running.store(false);
    }

    ~SocketClient() {
        disconnect();
    }

    // 콜백 설정
    ConnectCallback on_connect;
    MessageCallback on_message;
    DisconnectCallback on_disconnect;

    bool connect() {
        if (_running.load()) {
            std::cout << "[Warning] [Client] 이미 연결되어 있음\n";
            return true;
        }

        // I/O 스레드를 먼저 시작
        io_thread_ = std::thread([this]() { io_context_.run(); });

        auto promise = std::make_shared<std::promise<bool>>();
        std::future<bool> future = promise->get_future();




        // post를 사용하여 resolve 작업을 io_context 스레드에서 실행
        boost::asio::post(io_context_, [this, self = shared_from_this(), promise]() {
            resolver_.async_resolve(host_, port_,
                [this, self, promise](const boost::system::error_code& ec, tcp::resolver::results_type endpoints) {
                    if (!ec) {
                        boost::asio::async_connect(socket_, endpoints,
                            [this, self, promise](const boost::system::error_code& ec, const tcp::endpoint& endpoint) {
                                if (!ec) {
                                    promise->set_value(true);
                                } else {
                                    std::cerr << "[Error] [Client] " << host_ << ":" << port_ << " 서버 연결 실패: " << ec.message() << std::endl;
                                    promise->set_value(false);
                                }
                            });
                    } else {
                        std::cerr << "[Error] [Client] " << host_ << ":" << port_ << " 주소 변환 실패: " << ec.message() << std::endl;
                        promise->set_value(false);
                    }
                });
        });

        bool result = future.get();



        if (result) {
            std::cout << "[Info ] [Client] " << host_ << ":" << port_ << " 서버에 연결\n";
            _running.store(true);

            // 연결 성공 후, 메시지 수신 시작
            do_read();

            if (on_connect) on_connect();
        }

        else {
            // 연결 실패 시 io_context와 스레드 정리
            stop_context();
        }



        return result;
    }



    void send(const std::string& data) {

        if (!is_connected()) {
            std::cerr << "[Error] [Client] 서버에 연결되어 있지 않음" << std::endl;
            return;
        }


        boost::asio::post(write_strand_, [this, data, self = shared_from_this()]() {
            bool write_in_progress = !write_msgs_.empty();
            write_msgs_.push_back(data);
            if (!write_in_progress) {
                do_write();
            }
        });
    }



    void disconnect() {
        if (!_running.exchange(false)) return;



        // io_context 스레드에서 소켓을 닫도록 post
        boost::asio::post(io_context_, [this]() {
            boost::system::error_code ec;
            if (socket_.is_open()) {
                socket_.shutdown(tcp::socket::shutdown_both, ec);
                socket_.close(ec);
            }
        });



        stop_context();
        std::cout << "[Info ] [Client] 서버와 연결 끊김\n";
    }



    bool is_connected() const {
        return _running.load() && socket_.is_open();
    }

private:
    void stop_context() {
        // work_guard를 리셋하여 io_context가 작업을 마치고 run()을 종료할 수 있도록 함
        work_guard_.reset();
        if (io_thread_.joinable()) {
            io_thread_.join();
        }
        io_context_.reset();
    }



    void do_read() {
        // post를 사용하여 io_context 스레드에서 실행되도록 보장
        boost::asio::post(io_context_, [this, self = shared_from_this()]() {
            socket_.async_read_some(boost::asio::buffer(buffer_),
                [this, self](const boost::system::error_code& ec, std::size_t length) {
                    if (!ec) {
                        if (on_message) on_message({buffer_.begin(), buffer_.begin() + length});
                        do_read(); // 다음 읽기 시작
                    } else {
                        if (ec != boost::asio::error::operation_aborted) {
                            if (on_disconnect) on_disconnect();
                        }
                    }
                });
        });
    }




    void do_write() {
        boost::asio::async_write(socket_, boost::asio::buffer(write_msgs_.front()),
            boost::asio::bind_executor(write_strand_,
                [this, self = shared_from_this()](const boost::system::error_code& ec, std::size_t) {
                    if (!ec) {
                        write_msgs_.pop_front();
                        if (!write_msgs_.empty()) {
                            do_write();
                        }
                    }


                    else {
                         if (ec != boost::asio::error::operation_aborted) {
                            std::cerr << "[Error] [Client] 전송 오류: " << ec.message() << std::endl;
                            write_msgs_.clear();
                         }
                    }
                })
            );
    }




    std::string host_;
    std::string port_;
    boost::asio::io_context io_context_;
    tcp::socket socket_;
    tcp::resolver resolver_;
    std::thread io_thread_;

    boost::asio::executor_work_guard<boost::asio::io_context::executor_type> work_guard_;

    std::vector<char> buffer_{std::vector<char>(4096)};
    boost::asio::strand<boost::asio::io_context::executor_type> write_strand_;
    std::deque<std::string> write_msgs_;
    std::atomic<bool> _running;
};



/*
void on_message_handler(const std::vector<char>& msg) {
    std::cout << "[Info ] [Client] 메세지 수신: " << std::string(msg.begin(), msg.end()) << std::endl;
}

void on_disconnect_handler() {
    std::cout << "[Info ] [Client] 서버와 연결 끊어짐" << std::endl;
}

#include "lib/socket_client.hpp"

int main() {
    auto client = std::make_shared<SocketClient>("127.0.0.1", 5000);

    client->on_message = on_message_handler;
    client->on_disconnect = on_disconnect_handler;
    client->on_connect = [](){
        // 연결 성공 시 호출될 콜백 (선택적)
    };

    // connect 함수 호출, 성공/실패 결과가 나올 때까지 여기서 블로킹됨
    if (client->connect()) {

        try {
            for (std::string line; std::getline(std::cin, line);) {
                if (line == "quit" || line == "exit") {
                    break;
                }
                client->send(line);
            }
        } catch (const std::exception& e) {
            std::cerr << "오류 발생: " << e.what() << std::endl;
        }

    } else {
        return 1;
    }

    client->disconnect();
    return 0;
}
 */