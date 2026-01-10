#pragma once
#include <boost/asio.hpp>
#include <string>
#include <functional>
#include <memory>
#include <set>
#include <mutex>
#include <deque>
#include <iostream>
#include <thread>

using boost::asio::ip::tcp;

// 세션 인터페이스
class SessionInterface {
public:
    virtual ~SessionInterface() {}
    virtual void deliver(const std::string &msg) = 0;
    virtual void stop() = 0;
    virtual std::string ip() const = 0;
};

using ClinetHandler = std::shared_ptr<SessionInterface>;

class Server {
public:
    using reciveCallback = std::function<void(ClinetHandler, const std::string &)>;
    using AcceptCallback = std::function<void(ClinetHandler)>;
    using StopCallback   = std::function<void(ClinetHandler)>;

    reciveCallback on_receive;
    AcceptCallback on_accept;
    StopCallback on_disconnect;

    Server(short port) : acceptor_(io_context_, tcp::endpoint(tcp::v4(), port))
    {
        if (io_context_.stopped())
            io_context_.restart();

        _do_accept();
        std::cout << "[Server] 서버가 시작되었습니다.\n";
    }

    ~Server()
    {
        stop();
    }

    // 서버 동작
    void run()
    {
        try {
            io_context_.run();
        } catch (std::exception &e) {
            std::cerr << "[Server] 서버 에러: " << e.what() << std::endl;
        }
    }

    // 서버 종료
    void stop()
    {
        acceptor_.close();
        {
            std::lock_guard<std::mutex> lock(sessions_mutex_);
            for (auto &session : sessions_) {
                session->stop();
            }
            sessions_.clear();
        }
        io_context_.stop();
    }

    // 모든 클라이언트에게 메세지 보내기
    void broadcast(const std::string &msg)
    {
        std::lock_guard<std::mutex> lock(sessions_mutex_);
        for (auto &session : sessions_) {
            session->deliver(msg);
        }
    }

private:
    void _do_accept();
    void _join(std::shared_ptr<SessionInterface> session);
    void _leave(std::shared_ptr<SessionInterface> session);

    friend class Session;

    boost::asio::io_context io_context_;
    tcp::acceptor acceptor_;

    std::set<std::shared_ptr<SessionInterface>> sessions_;
    std::mutex sessions_mutex_;
};

// --- Session 구현부 ---
class Session : public SessionInterface, public std::enable_shared_from_this<Session> {
public:
    Session(tcp::socket socket, Server &server, Server::reciveCallback &handler)
        : socket_(std::move(socket)), server_(server), handler_(handler) {
        try {
            remote_endpoint_ = socket_.remote_endpoint();
        } catch (...) {
            remote_endpoint_ = tcp::endpoint();
        }
    }

    void start() {
        server_._join(shared_from_this());
        do_read();
    }

    void deliver(const std::string &msg) override {
        auto self(shared_from_this());
        boost::asio::post(socket_.get_executor(), [this, self, msg]() {
            bool write_in_progress = !write_msgs_.empty();
            write_msgs_.push_back(msg);
            if (!write_in_progress) {
                do_write();
            }
        });
    }

    void stop() override {
        auto self(shared_from_this());
        boost::asio::post(socket_.get_executor(), [this, self]() {
            boost::system::error_code ec;
            if (socket_.is_open()) {
                socket_.shutdown(tcp::socket::shutdown_both, ec);
                socket_.close(ec);
            }
        });
    }

    std::string ip() const override {
        return remote_endpoint_.address().to_string();
    }

private:
    void do_read() {
        auto self(shared_from_this());
        // \n을 기준으로 읽기 (telnet 등 테스트 용이)
        boost::asio::async_read_until(socket_, buffer_, '\n',
            [this, self](boost::system::error_code ec, std::size_t bytes_transferred) {
                if (!ec) {
                    std::string data;
                    std::istream is(&buffer_);
                    std::getline(is, data);

                    if (handler_) handler_(shared_from_this(), data);
                    do_read();
                } else {
                    server_._leave(shared_from_this());
                }
            });
    }

    void do_write() {
        auto self(shared_from_this());
        boost::asio::async_write(socket_, boost::asio::buffer(write_msgs_.front()),
            [this, self](boost::system::error_code ec, std::size_t) {
                if (!ec) {
                    write_msgs_.pop_front();
                    if (!write_msgs_.empty()) {
                        do_write();
                    }
                } else {
                    server_._leave(shared_from_this());
                }
            });
    }

    tcp::socket socket_;
    tcp::endpoint remote_endpoint_;
    boost::asio::streambuf buffer_;
    std::deque<std::string> write_msgs_;
    Server &server_;
    Server::reciveCallback &handler_;
};

// --- Server 나머지 구현부 ---
void Server::_join(std::shared_ptr<SessionInterface> session) {
    std::lock_guard<std::mutex> lock(sessions_mutex_);
    sessions_.insert(session);
    if (on_accept) on_accept(session);
}

void Server::_leave(std::shared_ptr<SessionInterface> session) {
    std::lock_guard<std::mutex> lock(sessions_mutex_);
    if (sessions_.erase(session) > 0) {
        if (on_disconnect) on_disconnect(session);
    }
}

void Server::_do_accept() {
    acceptor_.async_accept([this](boost::system::error_code ec, tcp::socket socket) {
        if (!ec) {
            std::make_shared<Session>(std::move(socket), *this, on_receive)->start();
        }
        if (acceptor_.is_open()) _do_accept();
    });
}