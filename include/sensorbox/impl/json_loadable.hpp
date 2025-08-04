#ifndef SENSORBOX_IMPL_JSON_LOADABLE_HPP
#define SENSORBOX_IMPL_JSON_LOADABLE_HPP

#include <algorithm>
#include <fstream>
#include <stdexcept>

#include "sensorbox/json_loadable.hpp"

namespace sensorbox {

template<std::size_t N>
constexpr SchemaFilepath<N>::SchemaFilepath(const char (&string)[N]) {
    std::copy_n(string, N, value.begin());
}

template<std::size_t N>
constexpr std::string_view SchemaFilepath<N>::string() const noexcept {
    return std::string_view{value.data(), value.size()};
}

template<SchemaFilepath schema_filepath, SchemaLoader schema_loader>
JsonLoadable<schema_filepath, schema_loader>::JsonLoadable(const nlohmann::json& json, const bool validate_) {
    if (validate_) {
        validate(json);
    }
}

template<SchemaFilepath schema_filepath, SchemaLoader schema_loader>
nlohmann::json JsonLoadable<schema_filepath, schema_loader>::validate(const nlohmann::json& json) {
    return validator_.validate(json);
}

template<SchemaFilepath schema_filepath, SchemaLoader schema_loader>
const nlohmann::json JsonLoadable<schema_filepath, schema_loader>::schema_ = []() {
    const std::filesystem::path filepath{schema_filepath.string()};
    if (!std::filesystem::exists(filepath)) {
        throw std::runtime_error("Schema filepath \"" + filepath.string() + "\" does not exist.");
    }
    std::ifstream file{filepath};
    if (!file) {
        throw std::runtime_error("Schema filepath \"" + filepath.string() + "\" existed but could not be opened.");
    }
    return nlohmann::json::parse(file);
}();

template<SchemaFilepath schema_filepath, SchemaLoader schema_loader>
const nlohmann::json_schema::json_validator JsonLoadable<schema_filepath, schema_loader>::validator_ =
        []() { return nlohmann::json_schema::json_validator{schema_, schema_loader}; }();

}

#endif
