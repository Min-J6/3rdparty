#pragma once
extern "C" {
// #include <dds/dds.h>
#include "CycloneDDS/core/ddsc/include/dds/dds.h"
}
#include <stdexcept>


/*
 *
 * Reliability (신뢰성): reliable() vs best_effort()
 * 데이터가 꼭 도착해야 하는지(TCP 방식), 아니면 속도가 중요하고 일부 유실되어도 되는지(UDP 방식)를 결정합니다.
 *
 *
 * Durability (내구성): transient_local() vs volatile()
 * 새로운 Subscriber가 접속했을 때, 이전에 Publisher가 보냈던 데이터를 받을 수 있게 할지(latch 기능) 결정합니다.
 *
 * History (히스토리): keep_last(N) vs keep_all()
 * 내부 버퍼에 데이터를 몇 개나 저장해 둘지 결정합니다.
 *
 */



class CycloneDdsQos {
public:
    CycloneDdsQos() {
        qos_ = dds_create_qos();
        if (!qos_) {
            throw std::runtime_error("CycloneDdsQos: Failed to create QoS");
        }
    }

    ~CycloneDdsQos() {
        if (qos_) {
            dds_delete_qos(qos_);
        }
    }

    // 복사 생성자 (Deep Copy)
    CycloneDdsQos(const CycloneDdsQos& other) {
        qos_ = dds_create_qos();
        if (!qos_) {
            throw std::runtime_error("CycloneDdsQos: Failed to create QoS for copy");
        }
        // dds_copy_qos를 사용하여 기존 QoS 설정을 복사합니다.
        dds_copy_qos(qos_, other.qos_);
    }

    // 복사 대입 연산자
    CycloneDdsQos& operator=(const CycloneDdsQos& other) {
        if (this != &other) {
            dds_copy_qos(qos_, other.qos_);
        }
        return *this;
    }

    // 이동 생성자
    CycloneDdsQos(CycloneDdsQos&& other) noexcept : qos_(other.qos_) {
        other.qos_ = nullptr; // 원본의 소유권 이전
    }

    // 이동 대입 연산자
    CycloneDdsQos& operator=(CycloneDdsQos&& other) noexcept {
        if (this != &other) {
            if (qos_) {
                dds_delete_qos(qos_); // 기존 리소스 해제
            }
            qos_ = other.qos_;
            other.qos_ = nullptr;
        }
        return *this;
    }




    // 마지막 N개의 히스토리 저장
    CycloneDdsQos& history_keep_last(int32_t depth) {
        dds_qset_history(qos_, DDS_HISTORY_KEEP_LAST, depth);
        return *this;
    }


    // 모든 히스토리 저장
    CycloneDdsQos& history_keep_all() {
        dds_qset_history(qos_, DDS_HISTORY_KEEP_ALL, DDS_LENGTH_UNLIMITED);
        return *this;
    }


    // 신뢰성: 데이터가 반드시 도착해야 하는지(TCP 방식)
    CycloneDdsQos& reliability_reliable(dds_duration_t max_blocking = DDS_SECS(1)) {
        dds_qset_reliability(qos_, DDS_RELIABILITY_RELIABLE, max_blocking);
        return *this;
    }


    // 신뢰성: 데이터가 도착하지 않아도 되는지(UDP 방식)
    CycloneDdsQos& reliability_best_effort() {
        dds_qset_reliability(qos_, DDS_RELIABILITY_BEST_EFFORT, DDS_SECS(0));
        return *this;
    }


    // 내구성: 나중에 연결된 subscriber가 접속했을 때, 이전에 Subscriber가 보냈던 데이터를 받을 수 있음
    CycloneDdsQos& durability_transient_local() {
        dds_qset_durability(qos_, DDS_DURABILITY_TRANSIENT_LOCAL);
        return *this;
    }


    // 내구성: 데이터는 오직 현재 접속해 있는 Subscriber에게만 전송됨
    CycloneDdsQos& durability_volatile() {
        dds_qset_durability(qos_, DDS_DURABILITY_VOLATILE);
        return *this;
    }

    operator const dds_qos_t*() const {
        return qos_;
    }

private:
    dds_qos_t* qos_ = nullptr;
};