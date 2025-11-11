#include "SerialPort.hpp"
#include <boost/asio.hpp>
#include <iostream>


void read_callback(const std::vector<char>& data)
{
    std::string str(data.begin(), data.end());
    std::cout << str << std::endl;
}

int main()
{
    std::cout << "[테스트] Serial Port" << std::endl;

    // io_context
    boost::asio::io_context io_context;
    boost::asio::executor_work_guard<boost::asio::io_context::executor_type> work_guard = boost::asio::make_work_guard(io_context);


    // 시리얼 포트 객체 생성
    SerialPort serial_port(io_context);
    serial_port.on_receive = read_callback; // Receive Callback 등록
    serial_port.open("/dev/pts/2", 115200); // 포트 열기



    // 다른 쓰레드에서 io_context를 실행
    std::thread thread([&io_context]()
    {
        io_context.run();
    });



    // 100ms 마다 hello world를 전송
    for (int i = 0; i < 10; ++i)
    {
        serial_port.write("hello world\r\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }



    // 종료
    work_guard.reset();
    serial_port.close();
    thread.join();

    return 0;
}