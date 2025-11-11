#pragma once
#include <string>
#include <thread>
#include <atomic>
#include <functional>

extern "C" {
// #include <dds/dds.h>
#include "CycloneDDS/core/ddsc/include/dds/dds.h"
}

template<typename T>
class Subscriber {
public:
    std::function<void(const T&)> on_received;  // 수신시 호출되는 콜백

    // topic_name: 토픽 이름
    // desc: 토픽 설명자 (idl 파일을 빌드한 헤더에 정의되어 있음)
    // domain_id: DDS 도메인 ID (기본값: DDS_DOMAIN_DEFAULT)
    Subscriber(const std::string& topic_name, const dds_topic_descriptor_t* desc, dds_domainid_t domain_id = DDS_DOMAIN_DEFAULT)
        : topic_name_(topic_name), domain_id_(domain_id) {
        participant_ = dds_create_participant(domain_id_, NULL, NULL);
        topic_       = dds_create_topic(participant_, desc, topic_name_.c_str(), NULL, NULL);
        reader_      = dds_create_reader(participant_, topic_, NULL, NULL);
        listener_    = dds_create_listener(this); // arg == this

        dds_lset_data_available(listener_, &Subscriber::on_data_available);
        dds_set_listener(reader_, listener_);
    }


    ~Subscriber() {
        dds_set_listener(reader_, NULL);
        dds_delete_listener(listener_);
        dds_delete(reader_);
        dds_delete(participant_);
    }

    // 현재 도메인 ID 반환
    dds_domainid_t get_domain_id() const {
        return domain_id_;
    }


private:
    // on_received를 cyclone dds에 전달하는 헬퍼
    static void on_data_available(dds_entity_t rdr, void* arg) {
        auto* self = static_cast<Subscriber*>(arg);
        if (!self) return;

        void*            samples[1] = {NULL};
        dds_sample_info_t infos[1]  = {};

        int ret = dds_take(rdr, samples, infos, 1, 1);
        if (ret > 0 && infos[0].valid_data)
        {
            const auto* data = static_cast<const T*>(samples[0]);
            if (self->on_received) self->on_received(*data);
        }

        // loan 해제
        if (ret > 0)
            dds_return_loan(rdr, samples, ret);
    }


private:
    std::string                   topic_name_;
    dds_domainid_t                domain_id_;
    dds_entity_t                  participant_{DDS_ENTITY_NIL};
    dds_entity_t                  topic_{DDS_ENTITY_NIL};
    dds_entity_t                  reader_{DDS_ENTITY_NIL};
    dds_listener_t*               listener_{NULL};
};
