idf_build_get_property(project_dir PROJECT_DIR)
message(STATUS "The project dir is: ${project_dir}")

set(headers
    configwrapper.h
    feature_bmp.h
    feature_dht.h
    feature_lamp.h
    feature_pms.h
    feature_switch.h
    feature_tsl.h
    myble.h
    myconfig.h
    mymdns.h
    mymqtt.h
    myota.h
    mywifi.h
    nvswrappers.h
    webserver.h
)

set(sources
    configwrapper.cpp
    feature_bmp.cpp
    feature_dht.cpp
    feature_lamp.cpp
    feature_pms.cpp
    feature_switch.cpp
    feature_tsl.cpp
    main.cpp
    myble.cpp
    myconfig.cpp
    mymdns.cpp
    mymqtt.cpp
    myota.cpp
    mywifi.cpp
    webserver.cpp
)

set(dependencies
    freertos nvs_flash esp_http_server esp_https_ota mdns app_update esp_system mqtt
    arduino-esp32 date esp-nimble-cpp expected fmt
    airgradient Adafruit_BMP085_Unified Adafruit_TSL2561 DHT-sensor-library
    cpputils espasyncota espchrono espconfiglib espcpputils esphttpdutils espwifistack
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
