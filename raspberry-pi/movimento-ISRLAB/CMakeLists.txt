cmake_minimum_required(VERSION 3.25)

set(TARGET main)

project(movimento-ISRLAB)

find_package( Eigen3 REQUIRED )


find_package(kobuki_core REQUIRED)
find_package(ecl_config REQUIRED)
find_package(ecl_console REQUIRED)
find_package(ecl_devices REQUIRED)
find_package(ecl_geometry REQUIRED)
find_package(ecl_mobile_robot REQUIRED)
find_package(ecl_sigslots REQUIRED)
find_package(ecl_time REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(eclipse-paho-mqtt-c REQUIRED)
find_package(PahoMqttCpp REQUIRED)

file(GLOB SRC "src/*.h" "src/*.cpp")

MESSAGE("-- Build type: ${CMAKE_BUILD_TYPE}")

add_executable(${TARGET} ${SRC})

target_include_directories( ${TARGET} PRIVATE ${PROJECT_SOURCE_DIR}/kobuki/src/kobuki_core/include/kobuki_core)


target_link_libraries(${TARGET}
    Eigen3::Eigen
    kobuki_core::kobuki_core
    ecl_console::ecl_console
    ecl_devices::ecl_devices
    ecl_geometry::ecl_geometry
    ecl_mobile_robot::ecl_mobile_robot
    ecl_sigslots::ecl_sigslots
    ecl_time::ecl_time
    PahoMqttCpp::paho-mqttpp3
)
 