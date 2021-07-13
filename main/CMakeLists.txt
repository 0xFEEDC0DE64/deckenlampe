idf_build_get_property(project_dir PROJECT_DIR)
message(STATUS "The project dir is: ${project_dir}")

set(headers
)

set(sources
    main.cpp
)

set(dependencies
    freertos nvs_flash esp_http_server esp_https_ota mdns app_update esp_system mqtt
    arduino-esp32 cpputils date espchrono espcpputils espwifistack expected fmt
    Adafruit_BMP085_Unified Adafruit_TSL2561 DHT-sensor-library
)

idf_component_register(
    SRCS
        ${headers}
        ${sources}
    INCLUDE_DIRS
        .
    REQUIRES
        ${dependencies}
)

target_compile_options(${COMPONENT_TARGET}
    PRIVATE
        -fstack-reuse=all
        -fstack-protector-all
        -Wno-unused-function
        -Wno-deprecated-declarations
        -Wno-missing-field-initializers
        -Wno-parentheses
)
