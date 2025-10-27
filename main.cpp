#include <iostream>
#include "lib/serial_port.hpp"
#include <boost/asio.hpp>


class MoterDriver {
public:
    MoterDriver(boost::asio::io_context& io) : serial(io)
    {

    }
    ~MoterDriver() = default;


    SerialPort serial;
};



int main() {
    boost::asio::io_context io_context;

    // work gaurd

    auto work_guard = boost::asio::make_work_guard(io_context);

    std::thread io_thread([&io_context]()
    {
        io_context.run();
    });






    work_guard.reset();
    io_thread.join();


    return 0;
}
