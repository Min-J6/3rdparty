#pragma once
extern "C" {
    // #include <dds/dds.h>
#include "CycloneDDS/core/ddsc/include/dds/dds.h"
#include "CycloneDDS/core/ddsc/include/dds/ddsc/dds_public_impl.h"
}
#include <string>


template<typename T>
class Publisher {
public:

    // topic_name: 토픽 이름
    // desc: 토픽 설명자 (idl 파일을 빌드한 헤더에 정의되어 있음)
    // domain_id: DDS 도메인 ID (기본값: DDS_DOMAIN_DEFAULT)
    Publisher(const std::string& topic_name, const dds_topic_descriptor_t* desc, dds_domainid_t domain_id = DDS_DOMAIN_DEFAULT)
        : topic_name_(topic_name), domain_id_(domain_id) {
        participant_ = dds_create_participant(domain_id_, NULL, NULL);
        topic_       = dds_create_topic(participant_, desc, topic_name.c_str(), NULL, NULL);
        writer_      = dds_create_writer(participant_, topic_, NULL, NULL);
    }


    ~Publisher() {
        dds_delete(writer_);
        dds_delete(participant_);
    }


    // 토픽에 메세지 전송
    void publish(const T& msg) {
        dds_write(writer_, &msg);
    }

    // 현재 도메인 ID 반환
    dds_domainid_t get_domain_id() const {
        return domain_id_;
    }


private:
    std::string topic_name_;
    dds_domainid_t domain_id_;
    dds_entity_t participant_, topic_, writer_;
};
