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
    Publisher(const std::string& topic_name, const dds_topic_descriptor_t* desc) : topic_name_(topic_name) {
        participant_ = dds_create_participant(DDS_DOMAIN_DEFAULT, NULL, NULL);
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


private:
    std::string topic_name_;
    dds_entity_t participant_, topic_, writer_;
};