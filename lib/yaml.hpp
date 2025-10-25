#pragma once
#include <cstdio>
#include <exception>
#include <iosfwd>
#include <iostream>
#include <ostream>
#include <sstream>
#include <string>
#include <vector>

#include "_ryml_core.hpp"



class Yaml {
public:
    // 생성자
    Yaml() {
        m_tree.rootref() |= c4::yml::MAP;
    }

    // YAML 파일 로드
    bool load(const std::string& filepath) {
        m_filepath = filepath;
        std::ifstream file(m_filepath);

        if (!file.is_open()) {
            std::cerr << "[Info] [Yaml] 파일을 찾을 수 없음: " << m_filepath << ". 새로 생성합니다." << std::endl;
            return false; // 파일을 새로 만드는 경우이므로 false를 반환하지만 오류는 아님
        }

        // 파일 내용 읽기
        std::stringstream buffer;
        buffer << file.rdbuf();
        m_content_buffer = buffer.str();

        // 파싱
        try {
            m_tree = c4::yml::parse_in_arena(c4::to_csubstr(m_content_buffer));
            return true;
        } catch (const std::exception& e) {
            std::cerr << "[Error] [Yaml] YAML 파일 파싱 실패: " << e.what() << std::endl;
            m_content_buffer.clear();
            return false;
        }
    }

    // YAML 파일 저장
    bool save() const {
        FILE* f = std::fopen(m_filepath.c_str(), "w");

        if (!f) {
            std::cerr << "[Error] [Yaml] 파일 저장 실패: " << m_filepath << std::endl;
            return false;
        }

        c4::yml::emit_yaml(m_tree, f);
        std::fclose(f);

        return true;
    }

    // 값 쓰기 (일반 타입)
    template <typename T>
    void set(const std::string& key_path, const T& value) {
        c4::yml::NodeRef node = _get_node_create(key_path);

        if (!node.valid()) {
            std::cerr << "[Error] [Yaml] 노드 설정 실패: " << key_path << std::endl;
            return;
        }

        node << value;
    }

    // 값 쓰기 (std::vector<T> 타입에 대한 템플릿 특수화)
    template <typename T>
    void set(const std::string& key_path, const std::vector<T>& values) {
        c4::yml::NodeRef node = _get_node_create(key_path);

        if (!node.valid()) {
            std::cerr << "[Error] [Yaml] 노드 설정 실패: " << key_path << std::endl;
            return;
        }

        node |= c4::yml::SEQ; // 노드 타입을 시퀀스(리스트)로 설정
        node.clear_children(); // 기존 리스트 내용을 비움

        for (const auto& value : values) {
            node.append_child() << value;
        }
    }

    // 값 읽기 (일반 타입)
    template <typename T>
    T get(const std::string& key_path, const T& default_value) {
        c4::yml::ConstNodeRef node = _get_node_read_only(key_path);

        if (node.valid() && node.has_val()) {
            T result;
            node >> result;
            return result;
        }

        set(key_path, default_value);
        return default_value;
    }

    // 값 읽기 (std::vector<T> 타입에 대한 템플릿 특수화)
    template <typename T>
    std::vector<T> get(const std::string& key_path, const std::vector<T>& default_value) {
        c4::yml::ConstNodeRef node = _get_node_read_only(key_path);

        if (node.valid() && node.is_seq()) {
            std::vector<T> result;
            for (c4::yml::ConstNodeRef child : node.children()) {
                T val;
                child >> val;
                result.push_back(val);
            }
            return result;
        }

        set(key_path, default_value);
        return default_value;
    }

private:
    std::string m_filepath;
    c4::yml::Tree m_tree;
    std::string m_content_buffer;

    // 값 쓰기 전용 노드 찾기
    c4::yml::NodeRef _get_node_create(const std::string& key_path) {
        c4::yml::NodeRef current_node = m_tree.rootref();
        std::stringstream ss(key_path);
        std::string segment;
        std::vector<std::string> segments;

        while (std::getline(ss, segment, '.')) {
            segments.push_back(segment);
        }

        for (size_t i = 0; i < segments.size(); ++i) {
            const auto& seg = segments[i];
            if (!current_node.is_map()) {
                current_node |= c4::yml::MAP;
            }

            c4::csubstr key_cs_safe = m_tree.to_arena(c4::to_csubstr(seg));
            c4::yml::NodeRef next_node = current_node.find_child(key_cs_safe);

            if (!next_node.valid()) {
                next_node = current_node.append_child();
                next_node.set_key(key_cs_safe);
            }
            current_node = next_node;
        }
        return current_node;
    }

    // 읽기 전용 노드 찾기
    c4::yml::ConstNodeRef _get_node_read_only(const std::string& key_path) const {
        c4::yml::ConstNodeRef current_node = m_tree.rootref();
        std::stringstream ss(key_path);
        std::string segment;

        while (std::getline(ss, segment, '.')) {
            if (!current_node.valid() || !current_node.is_map() || !current_node.has_child(c4::to_csubstr(segment))) {
                return {};
            }
            current_node = current_node[c4::to_csubstr(segment)];
        }
        return current_node;
    }
};
