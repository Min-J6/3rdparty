#pragma once
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <string>
#include <type_traits>

using namespace boost::interprocess;

template <typename T>
class SharedMemory {
private:
    std::string name_;
    shared_memory_object shm_;
    mapped_region region_;

    static_assert(std::is_trivially_copyable_v<T>, "copyable 타입이어야 합니다");

public:
    // 생성자에서 Mode를 받지 않고 자동으로 처리
    SharedMemory(const std::string& name) : name_(name)
    {
        try
        {
            // 없으면 생성, 있으면 열기
            shm_ = shared_memory_object(open_or_create, name_.c_str(), read_write);

            // 메모리 크기 설정
            shm_.truncate(sizeof(T));

            // 주소 공간 매핑
            region_ = mapped_region(shm_, read_write);
        } catch (const interprocess_exception& e)
        {
            throw std::runtime_error("공유 메모리 에러: " + std::string(e.what()));
        }
    }

    // 데이터 쓰기
    void set(const T& data)
    {
        std::memcpy(region_.get_address(), &data, sizeof(T));
    }

    // 데이터 읽기
    T& get()
    {
        return *static_cast<T*>(region_.get_address());
    }

    // 메모리 해제, 소멸자에서 자동 호출하지 않음
    // unlink를 호출하지 않으면 시스템이 재부팅할 때까지 데이터가 유지됨
    static void unlink(const std::string& name)
    {
        shared_memory_object::remove(name.c_str());
    }
};