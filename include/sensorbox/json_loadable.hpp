#ifndef SENSORBOX_SCHEMA_LOADABLE_HPP
#define SENSORBOX_SCHEMA_LOADABLE_HPP

#include <array>
#include <filesystem>
#include <string_view>
#include <nlohmann/json.hpp>
#include <nlohmann/json-schema.hpp>

namespace sensorbox {

/**
 * @brief NTTP wrapper for string representing a JSON Scheme filepath.
 * 
 * Example:
 * ```
 * constexpr SchemaFilepath MyClassSchemaFilepath{SCHEMAS_DIRECTORY "MyClass.schema.json"};
 * ```
 * 
 * @tparam N 
 */
template<std::size_t N>
class SchemaFilepath {
public:
    explicit constexpr SchemaFilepath(const char (&string)[N]);

    constexpr bool operator==(const SchemaFilepath&) const = default;

    constexpr std::string_view filepath() const noexcept;

    std::array<char, N> value;
};

/**
 * @brief Function pointer version of `nlohmann::json_schema::schema_loader`.
 * 
 */
using SchemaLoader = void(*)(const nlohmann::json_uri&, nlohmann::json&);

/**
 * @brief This class is intended to be inherited by classes that can be initialised by a nlohmann::json object according
 * to a JSON schema defined at a known filepath.
 * 
 * Example:
 * ```
 * constexpr SchemaFilepath MyClassSchemaFilepath{SCHEMAS_DIRECTORY "MyClass.schema.json"};
 *
 * class MyClass : public JsonLoadable<MyClassSchemaFilepath> {
 * public:
 *      MyClass(const nlohmann::json& json): JsonLoadable<MyClassSchemaFilepath>(json) {};
 * };
 * ```
 * 
 * The SchemaLoader can be left as `nullptr` if the Schema is self-contained or the uri paths can be loaded directly.
 * Otherwise implement a schema loader. See `sensorbox_schema_loader` for an example of a custom schema loader.
 * 
 * @tparam schema_filepath 
 */
template<SchemaFilepath schema_filepath, SchemaLoader schema_loader = nullptr>
class JsonLoadable {
public:
    /**
     * @brief Default construction with no checks.
     *
     */
    explicit JsonLoadable() = default;

    /**
     * @brief Checks json is valid, throwing an exception if invalid.
     *
     * @param json
     */
    explicit JsonLoadable(const nlohmann::json& json);

    /**
     * @brief Default destructor
     *
     */
    virtual ~JsonLoadable() = default;

    /**
     * @brief Validate a json object, throwing an exception if invalid and returning a patch to the default if valid.
     *
     * @param json
     * @return nlohmann::json
     */
    static nlohmann::json validate(const nlohmann::json& json);

private:
    /**
     * @brief JSON Schema
     *
     */
    static const nlohmann::json schema_;

    /**
     * @brief JSON Schema validator
     *
     */
    static const nlohmann::json_schema::json_validator validator_;
};

/**
 * @brief Function of type `nlohmann::json_schema::schema_loader` for validator to load schemas.
 * 
 * @param uri 
 * @param schema 
 */
void sensorbox_schema_loader(const nlohmann::json_uri& uri, nlohmann::json& schema);

}

#include "sensorbox/impl/json_loadable.hpp"

#endif
