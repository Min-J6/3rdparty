#pragma once
#include <boost/asio.hpp>
#include <string>
#include <functional>
#include <memory>
#include <set>
#include <mutex>
#include <deque>
#include <iostream>

using boost::asio::ip::tcp;

// 세션 인터페이스
class SessionInterface {
public:
    virtual ~SessionInterface() {}

    virtual void deliver(const std::string &msg) = 0;
    virtual void stop() = 0;
    virtual std::string ip() const = 0;
};


class Server {
public:
    using MessageHandler = std::function<void(std::shared_ptr<SessionInterface>, const std::string &)>;
    using AcceptHandler = std::function<void(std::shared_ptr<SessionInterface>)>;
    using StopHandler = std::function<void(std::shared_ptr<SessionInterface>)>;

    Server(short port);

    ~Server();

    void start();
    void stop();

    MessageHandler on_receive;  // 수신 콜백
    AcceptHandler on_accept;    // 연결 콜백
    StopHandler on_disconnect;  // 연결 종료 콜백

    void broadcast(const std::string &msg);

private:
    void do_accept();

    void join(std::shared_ptr<SessionInterface> session);
    void leave(std::shared_ptr<SessionInterface> session);

    friend class Session;

    boost::asio::io_context io_context_;
    tcp::acceptor acceptor_;
    std::thread worker_thread_;

    std::set<std::shared_ptr<SessionInterface> > sessions_;
    std::mutex sessions_mutex_;
};





























// 세션 구현부
class Session : public SessionInterface, public std::enable_shared_from_this<Session> {
public:
    Session(tcp::socket socket, Server &server, Server::MessageHandler &handler)
        : socket_(std::move(socket)), server_(server), handler_(handler)
    {
        try {
            remote_endpoint_ = socket_.remote_endpoint();
        } catch (...) {
            remote_endpoint_ = tcp::endpoint();
        }
    }

    void start()
    {
        server_.join(shared_from_this());
        do_read();
    }

    void deliver(const std::string &msg) override
    {
        auto self(shared_from_this());
        // 메인 스레드 등 외부에서 호출될 수 있으므로 io_context 스레드로 안전하게 전달
        boost::asio::post(socket_.get_executor(), [this, self, msg]()
        {
            bool write_in_progress = !write_msgs_.empty();
            write_msgs_.push_back(msg);
            if (!write_in_progress)
            {
                do_write();
            }
        });
    }

    void stop() override
    {
        auto self(shared_from_this());
        boost::asio::post(socket_.get_executor(), [this, self]()
        {
            boost::system::error_code ec;
            if (socket_.is_open())
            {
                socket_.shutdown(tcp::socket::shutdown_both, ec);
                socket_.close(ec);
            }
        });
    }

    std::string ip() const override
    {
        return remote_endpoint_.address().to_string();
    }

private:
    void do_read()
    {
        auto self(shared_from_this());
        boost::asio::async_read_until(socket_, buffer_, '\n',
                                      [this, self](boost::system::error_code ec, std::size_t)
                                      {
                                          if (!ec)
                                          {
                                              std::string data((std::istreambuf_iterator<char>(&buffer_)),
                                                               std::istreambuf_iterator<char>());

                                              if (handler_) handler_(shared_from_this(), data);
                                              do_read();
                                          } else
                                          {
                                              server_.leave(shared_from_this());
                                          }
                                      });
    }

    void do_write()
    {
        auto self(shared_from_this());
        boost::asio::async_write(socket_, boost::asio::buffer(write_msgs_.front()),
                                 [this, self](boost::system::error_code ec, std::size_t)
                                 {
                                     if (!ec)
                                     {
                                         write_msgs_.pop_front();
                                         if (!write_msgs_.empty())
                                         {
                                             do_write();
                                         }
                                     } else
                                     {
                                         server_.leave(shared_from_this());
                                     }
                                 });
    }

    tcp::socket socket_;
    tcp::endpoint remote_endpoint_;
    boost::asio::streambuf buffer_;
    std::deque<std::string> write_msgs_; // 쓰기 큐 추가 (데이터 꼬임 방지)
    Server &server_;
    Server::MessageHandler &handler_;
};

// --- SimpleServer 구현부 ---

Server::Server(short port)
    : acceptor_(io_context_, tcp::endpoint(tcp::v4(), port))
{
}

Server::~Server()
{
    stop();
}

void Server::start()
{
    if (io_context_.stopped()) io_context_.restart();

    do_accept();
    worker_thread_ = std::thread([this]()
    {
        try
        {
            io_context_.run();
        } catch (std::exception &e)
        {
            std::cerr << "Server Exception: " << e.what() << std::endl;
        }
    });
}

void Server::stop()
{
    acceptor_.close(); {
        std::lock_guard<std::mutex> lock(sessions_mutex_);
        for (auto &session: sessions_)
        {
            session->stop();
        }
        sessions_.clear();
    }

    io_context_.stop();
    if (worker_thread_.joinable())
    {
        worker_thread_.join();
    }
}

void Server::join(std::shared_ptr<SessionInterface> session)
{
    std::lock_guard<std::mutex> lock(sessions_mutex_);
    sessions_.insert(session);
    if (on_accept) on_accept(session);
}

void Server::leave(std::shared_ptr<SessionInterface> session)
{
    std::lock_guard<std::mutex> lock(sessions_mutex_);
    sessions_.erase(session);
    if (on_disconnect) on_disconnect(session);
}

void Server::broadcast(const std::string &msg)
{
    std::lock_guard<std::mutex> lock(sessions_mutex_);
    for (auto &session: sessions_)
    {
        session->deliver(msg);
    }
}

void Server::do_accept()
{
    acceptor_.async_accept([this](boost::system::error_code ec, tcp::socket socket)
    {
        if (!ec)
        {
            std::make_shared<Session>(std::move(socket), *this, on_receive)->start();
        }
        if (acceptor_.is_open()) do_accept();
    });
}
