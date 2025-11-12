#pragma once

#include <string>
#include <vector>
#include <iostream>
#include <deque>
#include <thread>
#include <mutex>
#include <atomic>
#include <functional>
#include <memory>
#include <boost/asio.hpp>
#include <dirent.h>
#include <algorithm>
#include <cstring>
#include <cctype>

using namespace boost::asio;

class SerialPort {
public:
    explicit SerialPort(boost::asio::io_context& io)
        : ioContext_(io), serialPort_(io), readStreamBuffer_(8192), writingInProgress_(false)
    {
    }

    ~SerialPort()
    {
        close();
    }


    std::function<void()> on_open;                              // 포트가 열릴 때 호출되는 콜백
    std::function<void()> on_close;                             // 포트가 닫힐 때 호출되는 콜백
    std::function<void(const std::string&)> on_error;           // 에러 발생 시 호출되는 콜백
    std::function<void(const std::vector<char>&)> on_receive;   // 메세지를 받을 때 호출되는 콜백


    /// 시리얼 포트 열기
    bool open(const std::string &portName, unsigned int baudRate)
    {
        if (is_open())
            return true;

        try
        {
            serialPort_.open(portName);
            serialPort_.set_option(serial_port_base::baud_rate(baudRate));
            serialPort_.set_option(serial_port_base::character_size(8));
            serialPort_.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));
            serialPort_.set_option(serial_port_base::parity(serial_port_base::parity::none));
            serialPort_.set_option(serial_port_base::flow_control(serial_port_base::flow_control::none));

            // 저수준 버퍼 크기 증가 (OS 레벨)
            #ifdef __linux__
            int fd = serialPort_.native_handle();
            int rx_buffer_size = 65536; // 수신 버퍼 크기 증가
            setsockopt(fd, SOL_SOCKET, SO_RCVBUF, &rx_buffer_size, sizeof(rx_buffer_size));
            int tx_buffer_size = 65536; // 송신 버퍼 크기 증가
            setsockopt(fd, SOL_SOCKET, SO_SNDBUF, &tx_buffer_size, sizeof(tx_buffer_size));
            #endif

            do_read();

            if (on_open) on_open();
            return true;
            
        } catch (std::exception &e)
        {
            if (on_error) on_error(e.what());
            std::cerr << "[Error] [Serial Port] 포트 열기 실패: " << e.what() << std::endl;
            return false;
        }
    }


    /// 시리얼 포트 닫기
    void close()
    {
        if (!is_open())
            return;


        boost::system::error_code ec;
        serialPort_.cancel(ec);
        serialPort_.close(ec);


        std::scoped_lock lock(writeMutex_);
        writeQueue_.clear();


        if (on_close) on_close();

    }


    /// 포트가 열려 있는지 확인
    bool is_open() const
    {
        return serialPort_.is_open();
    }

    
    /// byte 배열을 전송  std::vector<char>({0x01, 0x02, 0x03, 0x04});
    void write(const std::vector<char> &data)
    {
        if (!is_open() || data.empty())
            return;

        {
            // 쓰기 큐에 추가
            std::scoped_lock lock(writeMutex_);
            writeQueue_.push_back(data);
        }


        boost::asio::dispatch(ioContext_, [this]()
        {
            // 데이터 전송중이 아니면 doe_write 실행
            if (!writingInProgress_.exchange(true))
            {
                do_write();
            }
        });
        
    }


    /// 문자열을 전송 "message"
    void write(const std::string &text)
    {
        if (text.empty())
            return;

        write(std::vector<char>(text.begin(), text.end()));
        
    }


    /// 포트 목록 가져오기
    static std::vector<std::string> get_port_list()
    {
        std::vector<std::string> ports;
        ports.reserve(16);

        auto scan_dir = [&](const char *path, const char *prefix[], int count)
        {
            DIR *dir = opendir(path);
            if (!dir) return;

            struct dirent *entry;
            while ((entry = readdir(dir)))
            {
                for (int i = 0; i < count; i++)
                {
                    size_t prefix_len = strlen(prefix[i]);
                    if (strncmp(entry->d_name, prefix[i], prefix_len) == 0)
                    {
                        // string concatenation 최적화
                        std::string port;
                        port.reserve(strlen(path) + strlen(entry->d_name) + 2);
                        port = path;
                        port += "/";
                        port += entry->d_name;
                        ports.push_back(std::move(port));
                        break;
                    }
                }
            }
            closedir(dir);
        };

        const char *dev_prefixes[] = {"ttyS", "ttyUSB", "ttyACM"};
        scan_dir("/dev", dev_prefixes, 3);

        std::sort(ports.begin(), ports.end());
        return ports;
    }

private:
    /// 데이터 읽기
    void do_read()
    {
        if (!is_open())
            return;


        // '\r'가 들어오면 실행
        boost::asio::async_read_until(serialPort_, readStreamBuffer_, '\r',
              [this](const boost::system::error_code &ec, std::size_t bytesTransferred)
              {

                  // 데이터 읽기 실패
                  if (ec)
                  {
                      if (ec != boost::asio::error::operation_aborted)
                      {
                          if (on_error)
                              on_error(ec.message());
                          else
                            std::cerr << "[Error] [Serial Port] 읽기 실패: " << ec.message() << std::endl;

                          close(); // 포트 닫기
                      }
                      return;
                  }


                  // 데이터 추출
                  std::istream is(&readStreamBuffer_);
                  std::string message;
                  std::getline(is, message, '\r');


                  if (on_receive)
                  {
                      on_receive(std::vector<char>(message.begin(), message.end()));
                  }


                  // 다음 데이터 읽기
                  do_read();
              });

    }


    /// 데이터 쓰기
    void do_write()
    {
        if (!is_open())
        {
            writingInProgress_.store(false);
            return;
        }


        // 이중 버퍼 교환
        {
            std::scoped_lock lock(writeMutex_);
            activeWriteQueue_.swap(writeQueue_);
        }


        // 전송할 데이터가 없으면 바로 종료
        if (activeWriteQueue_.empty())
        {
            writingInProgress_.store(false);
            return;
        }


        // 버퍼 시퀀스 생성
        writeBuffers_.clear();
        writeBuffers_.reserve(activeWriteQueue_.size()); // 재할당 방지
        for (const auto &data: activeWriteQueue_)
        {
            writeBuffers_.push_back(boost::asio::buffer(data));
        }


        // 분산-수집 I/O를 사용하여 한 번에 모든 데이터를 비동기 전송
        boost::asio::async_write(serialPort_, writeBuffers_,
                 [this](const boost::system::error_code &ec, std::size_t /*bytesTransferred*/)
                 {
                     // 전송에 사용된 큐는 비우기
                     activeWriteQueue_.clear();


                     // 에러 발생 시 쓰기 작업 플래그를 해제
                     if (ec)
                     {
                         writingInProgress_.store(false);

                         if (on_error)
                             on_error(ec.message());
                         else
                            std::cerr << "[Error] [Serial Port] 쓰기 실패: " << ec.message() << std::endl;

                         close(); // 포트 닫기

                         return;
                     }


                     // 더 이상 쓰기 작업이 필요한지 확인
                     bool more_to_write = false; {
                         std::scoped_lock lock(writeMutex_);
                         more_to_write = !writeQueue_.empty();
                     }


                     // 쓰기 작업 중에 큐에 새로 추가된 데이터가 있다면 다음 쓰기 시작
                     if (more_to_write)
                     {
                         do_write();
                     } else
                     {
                         writingInProgress_.store(false); // 더 이상 쓸 데이터가 없으면 플래그를 내림
                     }
                 });
    }


    boost::asio::io_context &ioContext_;
    boost::asio::serial_port serialPort_;
    boost::asio::streambuf readStreamBuffer_;

    std::mutex writeMutex_;
    std::deque<std::vector<char> > writeQueue_; // 데이터가 추가되는 기본 큐 (스레드 안전)
    std::deque<std::vector<char> > activeWriteQueue_; // 실제 전송에 사용될 데이터 큐 (do_write 전용)
    std::vector<boost::asio::const_buffer> writeBuffers_; // 분산-수집 I/O를 위한 버퍼 시퀀스
    std::atomic<bool> writingInProgress_; // atomic으로 변경하여 락 없는 플래그 관리
};