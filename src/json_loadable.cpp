#include "sensorbox/json_loadable.hpp"

#include <filesystem>
#include <fstream>

#ifndef SENSORBOX_SCHEMAS_DIRECTORY
#error "SENSORBOX_SCHEMAS_DIRECTORY is not defined."
#endif

namespace sensorbox {

void sensorbox_schema_loader(const nlohmann::json_uri& uri, nlohmann::json& schema) {
    const std::filesystem::path schema_filepath = std::filesystem::path{std::string(SENSORBOX_SCHEMAS_DIRECTORY)} /
                                                  std::filesystem::path{uri.path().substr(1)};
    if (!std::filesystem::exists(schema_filepath)) {
        throw std::runtime_error("Schema \"" + schema_filepath.string() + "\" does not exist.");
    }
    std::ifstream schema_file{schema_filepath};
    if (!schema_file) {
        throw std::runtime_error("Schema \"" + schema_filepath.string() + "\" exists but could not be opened.");
    }
    try {
        schema_file >> schema;
    } catch (const std::exception& ex) {
        throw std::runtime_error(
                "Schema \"" + schema_filepath.string() + "\" exists and was opened but could not be parsed.");
    }
}

}
