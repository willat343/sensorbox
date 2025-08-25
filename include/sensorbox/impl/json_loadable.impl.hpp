#ifndef SENSORBOX_IMPL_JSON_LOADABLE_IMPL_HPP
#define SENSORBOX_IMPL_JSON_LOADABLE_IMPL_HPP

#include <cppbox/exceptions.hpp>
#include <filesystem>
#include <fstream>
#include <stdexcept>

#include "sensorbox/impl/json_loadable.hpp"
#include "sensorbox/impl/sensorbox.hpp"

#ifndef SENSORBOX_SCHEMAS_DIRECTORY
#error "SENSORBOX_SCHEMAS_DIRECTORY is not defined."
#endif

namespace sensorbox {

SENSORBOX_INLINE void sensorbox_schema_loader(const nlohmann::json_uri& uri, nlohmann::json& schema) {
    const std::filesystem::path schema_filepath = std::filesystem::path{std::string(SENSORBOX_SCHEMAS_DIRECTORY)} /
                                                  std::filesystem::path{uri.path().substr(1)};
    throw_if(!std::filesystem::exists(schema_filepath), "Schema \"" + schema_filepath.string() + "\" does not exist.");
    std::ifstream schema_file{schema_filepath};
    throw_if(!schema_file, "Schema \"" + schema_filepath.string() + "\" exists but could not be opened.");
    try {
        schema_file >> schema;
    } catch (const std::exception& ex) {
        throw_here("Schema \"" + schema_filepath.string() + "\" exists and was opened but could not be parsed.");
    }
}

}

#endif
