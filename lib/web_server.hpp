#pragma once
#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/strand.hpp>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>
#include <chrono>
#include <sstream>
#include <iomanip>
#include <memory>
#include <functional>
#include <thread>
#include <mutex>
#include <set>


namespace WebServer
{
namespace beast = boost::beast;
namespace websocket = boost::beast::websocket;
namespace net = boost::asio;
using tcp = boost::asio::ip::tcp;




// 에러 헬퍼 함수
void fail(beast::error_code ec, char const* what) {
    if(ec != websocket::error::closed)
        std::cerr << what << ": " << ec.message() << "\n";
}

// 세션 클래스 =
class session : public std::enable_shared_from_this<session>
{
    websocket::stream<beast::tcp_stream> ws_;
    beast::flat_buffer buffer_;
    std::vector<std::string> write_queue_;
    bool is_writing_ = false;

public:
    // 콜백 함수 멤버
    std::function<void(std::shared_ptr<session>)> on_connect;
    std::function<void(std::shared_ptr<session>, std::string)> on_receive;
    std::function<void(std::shared_ptr<session>)> on_disconnect;

    explicit session(tcp::socket&& socket)
        : ws_(std::move(socket)) {}


    // 세션 시작
    void run() {
        ws_.async_accept(beast::bind_front_handler(&session::on_accept, shared_from_this()));
    }


    // 메시지 전송
    void send(const std::string& message) {
        write_queue_.push_back(message);
        if (!is_writing_) {
            do_write();
        }
    }

private:
    // 메시지 전송
    void do_write() {
        if (write_queue_.empty()) {
            is_writing_ = false;
            return;
        }
        is_writing_ = true;
        ws_.async_write(net::buffer(write_queue_.front()), beast::bind_front_handler(&session::on_write, shared_from_this()));
    }


    // 메시지 전송
    void on_write(beast::error_code ec, std::size_t) {
        if (ec) {
            if (on_disconnect) on_disconnect(shared_from_this());
            return fail(ec, "write");
        }
        write_queue_.erase(write_queue_.begin());
        if (!write_queue_.empty()) {
            do_write();
        } else {
            is_writing_ = false;
        }
    }


    // 세션 연결 성공
    void on_accept(beast::error_code ec) {
        if(ec) return fail(ec, "accept");
        if (on_connect) on_connect(shared_from_this());
        do_read();
    }


    // 메시지 수신
    void do_read() {
        ws_.async_read(buffer_, beast::bind_front_handler(&session::on_read, shared_from_this()));
    }


    // 메시지 수신
    void on_read(beast::error_code ec, std::size_t) {
        if(ec) {
            if (on_disconnect) on_disconnect(shared_from_this());
            return fail(ec, "read");
        }
        if (on_receive) {
            on_receive(shared_from_this(), beast::buffers_to_string(buffer_.data()));
        }
        buffer_.consume(buffer_.size());
        do_read();
    }
};



// 리스너 클래스
class listener : public std::enable_shared_from_this<listener>
{
    net::io_context& ioc_;
    tcp::acceptor acceptor_;
public:
    std::function<void(std::shared_ptr<session>)> on_session_created;

    listener(net::io_context& ioc, tcp::endpoint endpoint) : ioc_(ioc), acceptor_(ioc) {
        beast::error_code ec;
        acceptor_.open(endpoint.protocol(), ec); if(ec) { fail(ec, "open"); return; }
        acceptor_.set_option(net::socket_base::reuse_address(true), ec); if(ec) { fail(ec, "set_option"); return; }
        acceptor_.bind(endpoint, ec); if(ec) { fail(ec, "bind"); return; }
        acceptor_.listen(net::socket_base::max_listen_connections, ec); if(ec) { fail(ec, "listen"); return; }
    }
    void run() { do_accept(); }
private:

    // 클라이언트 연결
    void do_accept() {
        acceptor_.async_accept(net::make_strand(ioc_), beast::bind_front_handler(&listener::on_accept, shared_from_this()));
    }

    // 클라이언트 연결
    void on_accept(beast::error_code ec, tcp::socket socket) {
        if(ec) { fail(ec, "accept"); }
        else {
            auto new_session = std::make_shared<session>(std::move(socket));
            if (on_session_created) {
                on_session_created(new_session);
            }
            new_session->run();
        }
        do_accept();
    }
};


// ip 생성 헬퍼 함수
net::ip::address make_address(const std::string& ip)
{
    return net::ip::make_address(ip);
}

/* namespace WebServer */}

